"""GPU negotiated-congestion rough routing for the global planning pass.

global_plan.plan_global_routes probes each net ONCE on the CPU, blind to the
other nets (or, with SEQ, sequentially aware but never renegotiated). This
module produces the same GlobalPlan.rough_paths from a PathFinder run on the
GPU (grid_router.GpuGlobalRouter): every net is routed together on the
board's 3D routing grid, nets that overlap pay present + history costs, and
the overlap is renegotiated until no grid cell is claimed by two nets'
keep-outs. Everything downstream (reservations, conflict graphs, layer
assignment, ordering, attraction lanes) consumes the plan unchanged, and the
CPU detailed router remains the authority on exact geometry and DRC.

Engine selection: the routing dialog's "GPU global plan (experimental)"
option or route.py --gpu-plan (both turn the planning pass on and use the GPU
when usable), or KICAD_GLOBAL_PLAN=1 with KICAD_GLOBAL_PLAN_ENGINE = cpu | gpu
| auto. 'auto' uses the GPU when grid_router was built with the `gpu` feature
and a CUDA device is usable, else falls back to the CPU probes.
"""
from __future__ import annotations

import math
import os
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

_GPU_OK = None
# Set by the routing dialog's "GPU global plan" option (around its routing
# run) and by route.py --gpu-plan: turns the global planning pass on and
# selects the GPU engine when one is usable (CPU probes otherwise).
_REQUESTED = False


def request_gpu_plan(on: bool) -> None:
    global _REQUESTED
    _REQUESTED = bool(on)


def gpu_plan_requested() -> bool:
    return _REQUESTED


def gpu_engine_available() -> bool:
    """grid_router built with the gpu feature and a CUDA device present."""
    global _GPU_OK
    if _GPU_OK is None:
        try:
            import grid_router
            _GPU_OK = bool(getattr(grid_router, 'GpuGlobalRouter', None)
                           and grid_router.gpu_available())
        except Exception:                                   # noqa: BLE001
            _GPU_OK = False
    return _GPU_OK


def plan_engine() -> str:
    """'gpu' or 'cpu' -- the engine plan_global_routes should use."""
    want = os.environ.get('KICAD_GLOBAL_PLAN_ENGINE', 'cpu').strip().lower()
    if _REQUESTED and want == 'cpu':
        want = 'auto'   # the GUI / --gpu-plan option asks for the GPU if usable
    if want in ('gpu', 'auto'):
        if gpu_engine_available():
            return 'gpu'
        if want == 'gpu':
            print("  [plan] GPU engine requested but unavailable "
                  "(no CUDA device or grid_router built without `gpu`); "
                  "using CPU probes")
    return 'cpu'


def _knob(name, default, cast=float):
    v = os.environ.get(name)
    try:
        return cast(v) if v not in (None, '') else default
    except ValueError:
        return default


def _net_terminals(pcb_data, net_id, layer_map, n_layers, coord, gx0, gy0, W, H):
    """[[(x, y, layer), ...] per pad] in router-local grid coords."""
    terms = []
    for pad in pcb_data.pads_by_net.get(net_id, []):
        gx, gy = coord.to_grid(pad.global_x, pad.global_y)
        x, y = gx - gx0, gy - gy0
        if not (0 <= x < W and 0 <= y < H):
            continue
        layers = []
        for ln in pad.layers:
            if ln in layer_map:
                layers.append(layer_map[ln])
            elif ln in ('*.Cu', '*.Mask') or ln.startswith('*'):
                layers = list(range(n_layers))
                break
        if getattr(pad, 'drill', 0) and pad.drill > 0:
            layers = list(range(n_layers))       # a barrel reaches every layer
        layers = sorted(set(l for l in layers if 0 <= l < n_layers))
        if layers:
            terms.append([(x, y, l) for l in layers])
    return terms


def _simplify(path):
    """Drop interior points of straight same-layer runs; keep leg breaks."""
    if len(path) < 3:
        return list(path)
    out = [path[0]]
    for a, b, c in zip(path, path[1:], path[2:]):
        adj_ab = max(abs(b[0] - a[0]), abs(b[1] - a[1])) <= 1
        adj_bc = max(abs(c[0] - b[0]), abs(c[1] - b[1])) <= 1
        straight = (adj_ab and adj_bc and a[2] == b[2] == c[2]
                    and (b[0] - a[0], b[1] - a[1]) == (c[0] - b[0], c[1] - b[1]))
        if not straight:
            out.append(b)
    out.append(path[-1])
    return out


def _result_for(path, net_id, width, config, via_size, via_drill, layer_names):
    """A route_net_with_obstacles-shaped result for the plan's reservation
    stamp (compute_ripped_route_costs reads new_segments)."""
    from kicad_parser import Segment, Via
    g = config.grid_step
    segs, vias = [], []
    for (x1, y1, l1), (x2, y2, l2) in zip(path, path[1:]):
        if max(abs(x2 - x1), abs(y2 - y1)) > 1 and (x1, y1) != (x2, y2) and l1 == l2:
            pass  # straight run (simplified) -- still a segment
        if (x1, y1) == (x2, y2) and l1 != l2:
            vias.append(Via(x=x1 * g, y=y1 * g, size=via_size, drill=via_drill,
                            layers=[layer_names[0], layer_names[-1]], net_id=net_id))
            continue
        if l1 != l2:
            continue  # leg break across layers
        segs.append(Segment(start_x=x1 * g, start_y=y1 * g, end_x=x2 * g, end_y=y2 * g,
                            width=width, layer=layer_names[l1], net_id=net_id))
    return {'path': path, 'new_segments': segs, 'new_vias': vias}


def _pin_mask(pcb_data, net_ids, layer_map, n_layers, coord, gx0, gy0, W, H):
    """[L, H, W] u8: 1 inside any pad of the planned nets (on its layers).
    Occupancy is not counted there -- neighbouring fine-pitch pins of
    different nets overlap within their keep-outs by construction."""
    mask = np.zeros((n_layers, H, W), np.uint8)
    g = coord.grid_step if hasattr(coord, 'grid_step') else None
    for nid in net_ids:
        for pad in pcb_data.pads_by_net.get(nid, []):
            rot = math.radians(getattr(pad, 'rect_rotation', 0.0) or 0.0)
            hx = abs(pad.size_x / 2 * math.cos(rot)) + abs(pad.size_y / 2 * math.sin(rot))
            hy = abs(pad.size_x / 2 * math.sin(rot)) + abs(pad.size_y / 2 * math.cos(rot))
            ax, ay = coord.to_grid(pad.global_x - hx, pad.global_y - hy)
            bx, by = coord.to_grid(pad.global_x + hx, pad.global_y + hy)
            x0, x1 = max(0, ax - gx0), min(W - 1, bx - gx0)
            y0, y1 = max(0, ay - gy0), min(H - 1, by - gy0)
            if x0 > x1 or y0 > y1:
                continue
            if getattr(pad, 'drill', 0) and pad.drill > 0:
                layers = range(n_layers)
            else:
                layers = [layer_map[ln] for ln in pad.layers if ln in layer_map]
                if any(ln.startswith('*') for ln in pad.layers):
                    layers = range(n_layers)
            for l in layers:
                if 0 <= l < n_layers:
                    mask[l, y0:y1 + 1, x0:x1 + 1] = 1
    return mask


def gpu_rough_paths(pcb_data, config, net_ids: List[Tuple[str, int]], layer_map,
                    probe_map) -> Optional[dict]:
    """Rough paths for every net from the GPU PathFinder.

    Returns {'paths': {nid: [(gx, gy, layer), ...] absolute grid coords},
             'results': {nid: result dict}, 'stats': {...}} or None on failure
    (the caller then falls back to CPU probes)."""
    import grid_router
    from routing_config import GridCoord
    t0 = time.time()
    coord = GridCoord(config.grid_step)
    bounds = pcb_data.board_info.board_bounds
    if not bounds:
        return None
    (bx0, by0, bx1, by1) = bounds
    gx0, gy0 = coord.to_grid(bx0, by0)
    gx1, gy1 = coord.to_grid(bx1, by1)
    W, H = gx1 - gx0 + 1, gy1 - gy0 + 1
    layer_names = list(config.layers)
    L = len(layer_names)
    blocked, via_blocked = probe_map.export_dense(gx0, gy0, W, H)
    t_export = time.time() - t0

    clearance = config.clearance
    g = config.grid_step
    via_size = config.via_size
    nets = []
    for _name, nid in net_ids:
        terms = _net_terminals(pcb_data, nid, layer_map, L, coord, gx0, gy0, W, H)
        if len(terms) < 2:
            continue
        width = config.get_net_track_width(nid, layer_names[0])
        # Two nets' halos overlap when their centrelines are closer than
        # w_a/2 + clearance + w_b/2: give each net half the clearance.
        tr = max(0, int(round((width / 2 + clearance / 2) / g)))
        vr = max(1, int(round((via_size / 2 + clearance / 2) / g)))
        nets.append((int(nid), terms, tr, vr))
    if not nets:
        return None
    pin_mask = _pin_mask(pcb_data, [nid for _n, nid in net_ids], layer_map, L,
                         coord, gx0, gy0, W, H)
    _dump = os.environ.get('KICAD_GPU_PLAN_DUMP')
    if _dump:
        # Offline engine work: everything the GPU call needs, plus pad areas.
        import pickle
        pads = []
        for _name, nid in net_ids:
            for pad in pcb_data.pads_by_net.get(nid, []):
                pads.append((int(nid), pad.global_x, pad.global_y, pad.size_x, pad.size_y,
                             getattr(pad, 'rect_rotation', 0.0), list(pad.layers),
                             float(getattr(pad, 'drill', 0) or 0)))
        with open(_dump, 'wb') as fh:
            pickle.dump({'blocked': np.asarray(blocked), 'via_blocked': np.asarray(via_blocked),
                         'layer_costs': list(config.get_layer_costs()),
                         'via_cost': int(config.via_cost_units()), 'nets': nets,
                         'origin': (gx0, gy0), 'grid_step': g, 'layers': layer_names,
                         'pads': pads, 'pin_mask': pin_mask}, fh)
        print(f"  [plan] GPU inputs dumped to {_dump}")
    router = grid_router.GpuGlobalRouter(blocked, via_blocked,
                                         list(config.get_layer_costs()),
                                         int(config.via_cost_units()), pin_mask)
    paths, stats = router.route(
        nets,
        max_iters=_knob('KICAD_GPU_PLAN_ITERS', 40, int),
        margin=_knob('KICAD_GPU_PLAN_MARGIN', int(round(3.0 / g)), int),
        pres0=_knob('KICAD_GPU_PLAN_PRES0', 0.5),
        pres_mult=_knob('KICAD_GPU_PLAN_PRES_MULT', 1.6),
        hfac=_knob('KICAD_GPU_PLAN_HFAC', 0.4),
        big_net_pins=_knob('KICAD_GPU_PLAN_BIG_NET', 8, int),
        batches_per_iter=_knob('KICAD_GPU_PLAN_BATCHES', 4, int))
    out_paths, results = {}, {}
    widths = {n[0]: config.get_net_track_width(n[0], layer_names[0]) for n in nets}
    for nid, pts in paths.items():
        absolute = [(x + gx0, y + gy0, l) for (x, y, l) in pts]
        simp = _simplify(absolute)
        out_paths[nid] = simp
        results[nid] = _result_for(simp, nid, widths[nid], config, via_size,
                                   config.via_drill, layer_names)
    st = dict(stats)
    st['export_s'] = t_export
    st['grid'] = (W, H, L)
    st['nets'] = len(nets)
    st['total_s'] = time.time() - t0
    return {'paths': out_paths, 'results': results, 'stats': st}
