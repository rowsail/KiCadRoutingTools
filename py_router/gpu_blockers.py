"""GPU rip-up planning: a blocker finder plus parallel rip-up trials.

When a net's route fails, the rip ladder must guess which routed nets to rip.
The frontier report names nets by how much of the failed search's perimeter
they cover -- exposure, not cut-ness -- and the CPU min-cut probe
(blocking_analysis.mincut_probe_order) improves on it, but is slow and only
considers the frontier-named nets.

Blocker finder: on a CLONE of the working map, every rippable net routed this
run near the failed connection has its copper lifted and re-added as an
EXPENSIVE cell cost instead of a wall; one GPU flood then gives the cheapest
path, and the nets that path crosses (in order) are the smallest set whose
removal lets the net through.

Parallel rip-up trials: alternative sets are found by making each found
set's nets dearer and re-flooding. Every candidate set is then TESTED: for
each net it would rip, a GPU reachability flood on a clone with the whole
set lifted and the failed net's new path (plus a clearance halo) stamped in
checks that the ripped net could still connect its pins. Sets are ranked by
(all ripped nets re-routable, fewest nets, lowest cost); the rip ladder then
takes the winner in order. The ladder still does the real rip / reroute /
restore, so this changes only WHICH nets are ripped.
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_STATS = {'calls': 0, 'sets': 0, 'infeasible': 0, 'ms': 0.0}


def stats() -> dict:
    return dict(_STATS)


def _router(config):
    from grid_router import GridRouter
    return GridRouter(via_cost=config.via_cost_units(), h_weight=1.0,
                      via_proximity_cost=config.via_proximity_cost_int(),
                      layer_costs=config.get_layer_costs())


def _clip(pcb_data, config):
    try:
        from routing_config import GridCoord
        bb = pcb_data.board_info.board_bounds
        if bb:
            c = GridCoord(config.grid_step)
            gx0, gy0 = c.to_grid(bb[0], bb[1])
            gx1, gy1 = c.to_grid(bb[2], bb[3])
            return (min(gx0, gx1) - 2, min(gy0, gy1) - 2, max(gx0, gx1) + 2, max(gy0, gy1) + 2)
    except Exception:
        pass
    return None


def _pin_cells(pcb_data, net_id, config):
    """[(gx, gy, layer_idx)] per pin of the net (pin centres on the pin's
    copper layers among the routing layers)."""
    from routing_config import GridCoord
    c = GridCoord(config.grid_step)
    lmap = {l: i for i, l in enumerate(config.layers)}
    out = []
    for p in pcb_data.pads_by_net.get(net_id, []):
        gx, gy = c.to_grid(p.global_x, p.global_y)
        pl = list(getattr(p, 'layers', None) or [])
        if (getattr(p, 'drill', 0) or 0) > 0 or any('*' in L for L in pl):
            lays = list(range(len(config.layers)))
        else:
            lays = [lmap[L] for L in pl if L in lmap]
        if lays:
            out.append([(gx, gy, li) for li in lays])
    return out


def _net_cells(data):
    """All TRACK keep-out cells of a cached net: blocked_cells plus the #815
    capsule spans ([gx, gy_lo, gy_hi, layer]) expanded -- segment keep-outs
    live ONLY in the spans."""
    parts = []
    bc = getattr(data, 'blocked_cells', None)
    if bc is not None and len(bc):
        parts.append(np.asarray(bc, dtype=np.int64).reshape(-1, 3))
    sp = getattr(data, 'blocked_cell_spans', None)
    if sp is not None and len(sp):
        sp = np.asarray(sp, dtype=np.int64).reshape(-1, 4)
        n = sp[:, 2] - sp[:, 1] + 1
        rep_ = np.repeat(np.arange(len(sp)), n)
        off = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)
        parts.append(np.stack([sp[rep_, 0], sp[rep_, 1] + off, sp[rep_, 3]], axis=1))
    return np.concatenate(parts) if parts else np.empty((0, 3), dtype=np.int64)


def _net_via_cells(data):
    """All via keep-out cells of a cached net (blocked_vias + via spans)."""
    parts = []
    bv = getattr(data, 'blocked_vias', None)
    if bv is not None and len(bv):
        parts.append(np.asarray(bv, dtype=np.int64).reshape(-1, 2))
    sp = getattr(data, 'blocked_via_spans', None)
    if sp is not None and len(sp):
        sp = np.asarray(sp, dtype=np.int64).reshape(-1, 3)
        n = sp[:, 2] - sp[:, 1] + 1
        rep_ = np.repeat(np.arange(len(sp)), n)
        off = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)
        parts.append(np.stack([sp[rep_, 0], sp[rep_, 1] + off], axis=1))
    return np.concatenate(parts) if parts else np.empty((0, 2), dtype=np.int64)


def _cells_near(cells: np.ndarray, box) -> bool:
    if cells is None or not len(cells):
        return False
    x0, y0, x1, y1 = box
    m = (cells[:, 0] >= x0) & (cells[:, 0] <= x1) & (cells[:, 1] >= y0) & (cells[:, 1] <= y1)
    return bool(m.any())


def plan_rip_sets(pcb_data, config, working_obstacles, net_id, sources, targets,
                  rippable_ids: Sequence[int], net_obstacles_cache: Dict,
                  max_sets: int = 3, window_mm: float = 10.0,
                  soft_cost_mm: float = 5.0) -> Tuple[List[dict], bool]:
    """Ranked candidate rip sets for a failed connection.

    Returns (sets, feasible): sets = [{'nets': [ids in crossing order],
    'cost': int, 'reroutable': n, 'checked': n}] best first; feasible False
    means no path exists even through the soft copper (the wall is static),
    or the GPU is unavailable (then sets is empty too)."""
    t0 = time.perf_counter()
    _STATS['calls'] += 1
    from obstacle_cache import remove_net_obstacles_from_cache
    if not sources or not targets:
        return [], False
    xs = [s[0] for s in list(sources) + list(targets)]
    ys = [s[1] for s in list(sources) + list(targets)]
    pad = int(window_mm / config.grid_step)
    box = (min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad)
    cand = []
    for nid in rippable_ids:
        if nid == net_id:
            continue
        data = net_obstacles_cache.get(nid)
        if data is not None and _cells_near(_net_cells(data), box):
            cand.append(nid)
    if not cand:
        return [], True

    base = working_obstacles.clone_fresh()
    own = net_obstacles_cache.get(net_id)
    if own is not None:
        remove_net_obstacles_from_cache(base, own)
    cell_sets, via_sets = {}, {}
    for nid in cand:
        data = net_obstacles_cache[nid]
        remove_net_obstacles_from_cache(base, data)
        cell_sets[nid] = {(int(x), int(y), int(l)) for x, y, l in _net_cells(data)}
        via_sets[nid] = {(int(x), int(y)) for x, y in _net_via_cells(data)}
    unit = int(config.cell_cost(soft_cost_mm))
    router = _router(config)
    clip = _clip(pcb_data, config)
    margin = max(200, pad)

    def soft_path(weights):
        extra = {}
        for nid, cs in cell_sets.items():
            c = int(unit * weights.get(nid, 1.0))
            for cell in cs:
                extra[cell] = extra.get(cell, 0) + c
        rows = [(x, y, l, min(c, 2_000_000_000)) for (x, y, l), c in extra.items()]
        return router.gpu_soft_path(base, list(sources), list(targets), rows, clip=clip, margin=margin)

    def crossed(path):
        order, seen = [], set()
        prev = None
        for cell in path:
            hits = [nid for nid, cs in cell_sets.items() if cell in cs]
            if prev is not None and prev[2] != cell[2]:      # a via at (x, y)
                hits += [nid for nid, vs in via_sets.items() if (cell[0], cell[1]) in vs]
            for nid in hits:
                if nid not in seen:
                    seen.add(nid)
                    order.append(nid)
            prev = cell
        return order

    sets, weights, paths = [], {}, {}
    for _ in range(max_sets):
        path, cost, _ms = soft_path(weights)
        if not path:
            break
        s = crossed(path)
        key = frozenset(s)
        if not s or any(frozenset(x['nets']) == key for x in sets):
            if not sets and not s:
                # routable without ripping anything (the CPU search failed for
                # a reason the flood does not model) -- nothing to plan
                _STATS['ms'] += (time.perf_counter() - t0) * 1000.0
                return [], True
            break
        sets.append({'nets': s, 'cost': int(cost)})
        paths[key] = path
        for nid in s:
            weights[nid] = weights.get(nid, 1.0) * 8.0
    if not sets:
        _STATS['infeasible'] += 1
        _STATS['ms'] += (time.perf_counter() - t0) * 1000.0
        return [], False

    # Parallel rip-up trials: can every net a set rips still connect?
    halo = int(math.ceil((config.track_width + config.clearance) / config.grid_step))
    for cs in sets:
        key = frozenset(cs['nets'])
        trial = working_obstacles.clone_fresh()
        if own is not None:
            remove_net_obstacles_from_cache(trial, own)
        for nid in cs['nets']:
            remove_net_obstacles_from_cache(trial, net_obstacles_cache[nid])
        stamp = set()
        for (x, y, l) in paths[key]:
            for dx in range(-halo, halo + 1):
                for dy in range(-halo, halo + 1):
                    if dx * dx + dy * dy <= halo * halo:
                        stamp.add((x + dx, y + dy, l))
        if stamp:
            trial.add_blocked_cells_batch(np.array(sorted(stamp), dtype=np.int32))
        ok = 0
        for nid in cs['nets']:
            pins = _pin_cells(pcb_data, nid, config)
            if len(pins) < 2:
                ok += 1
                continue
            src = pins[0]
            tgt = [c for p in pins[1:] for c in p]
            r = router.route_with_frontier_gpu(trial, src, tgt, 0, clip=clip, margin=margin,
                                               prove_only=True)
            ok += r[4] != 'unreachable'
        cs['reroutable'] = ok
        cs['checked'] = len(cs['nets'])
    sets.sort(key=lambda c: (c['reroutable'] < c['checked'], len(c['nets']), c['cost']))
    _STATS['sets'] += len(sets)
    _STATS['ms'] += (time.perf_counter() - t0) * 1000.0
    return sets, True
