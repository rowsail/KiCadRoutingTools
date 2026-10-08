"""Redundant leaf-copper pruning: the end-of-run tidy for dead-end copper the
dead-end sweep keeps.

sweep_dead_ends treats a via at a track end as an anchor, so a track that
ends on a via joining nothing else ships (KiCad: via_dangling), and it never
looks at plane nets' redundant links -- a pin already dropped to its plane by
one via whose trace then carries on to a SECOND via on the same plane.

This pass removes LEAF items -- a track with an end touching no other
same-net copper, pad or via; a via touching at most one same-net track and
no same-net pad -- one at a time, keeping a removal only when the
authoritative zone-aware connectivity check grades the net no worse (still
connected if it was, no more stranded pads, no more components). Repeated
until nothing changes, so a pad -> via -> trace -> via chain on a plane net
unwinds from the far via inward and stops at the first via the pad needs.

Protected (unfinished) nets are skipped -- their dead ends are the landing
sites the reconciliation retries from, exactly as for the sweep -- and so are
locked copper and footprint graphics. Removed copper is split by origin like
sweep_dead_ends: this run's copper is dropped from the write-lists, input-file
copper is returned for the writer / GUI applier to strip, and pcb_data is
updated to the board that will be written (#319 mutation contract).
"""

from __future__ import annotations

import math
from typing import List, Tuple

_EPS = 1e-6


def _pt_seg(px, py, s) -> float:
    ax, ay, bx, by = s.start_x, s.start_y, s.end_x, s.end_y
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _pad_pts(pads):
    out = []
    for p in pads:
        if getattr(p, 'pad_type', '') == 'np_thru_hole':
            continue
        px = getattr(p, 'global_x', getattr(p, 'x', 0.0))
        py = getattr(p, 'global_y', getattr(p, 'y', 0.0))
        r = max(getattr(p, 'size_x', 0.5), getattr(p, 'size_y', 0.5)) / 2.0
        out.append((px, py, r))
    return out


def _leaf_candidates(segs, vias, pad_pts, single_layer_vias=()):
    """(kind, obj) for leaf items, vias first, in a deterministic order.
    A via is a candidate when it touches at most one same-net track and no
    pad, OR when same-net copper reaches it on at most one layer
    (``single_layer_vias``: KiCad's via_dangling -- it joins no layers, e.g.
    tracks meeting on one layer, or a via in a single-layer SMD pad)."""
    cands = []
    single = {id(v) for v in single_layer_vias}
    for v in vias:
        if id(v) in single:
            cands.append(('via', v))
            continue
        vr = (getattr(v, 'size', 0.0) or 0.0) / 2.0
        if any(math.hypot(v.x - px, v.y - py) < r + vr for px, py, r in pad_pts):
            continue
        n = sum(1 for s in segs if _pt_seg(v.x, v.y, s) < vr + s.width / 2.0 - _EPS)
        if n <= 1:
            cands.append(('via', v))
    for s in segs:
        free = False
        for ex, ey in ((s.start_x, s.start_y), (s.end_x, s.end_y)):
            on = (any(t is not s and _pt_seg(ex, ey, t) < t.width / 2.0 + _EPS for t in segs)
                  or any(math.hypot(ex - v.x, ey - v.y) < (getattr(v, 'size', 0.0) or 0.0) / 2.0 + _EPS
                         for v in vias)
                  or any(math.hypot(ex - px, ey - py) < r + _EPS for px, py, r in pad_pts))
            if not on:
                free = True
                break
        if free:
            cands.append(('seg', s))
    cands.sort(key=lambda kv: (0 if kv[0] == 'via' else 1,
                               round(getattr(kv[1], 'x', getattr(kv[1], 'start_x', 0.0)), 4),
                               round(getattr(kv[1], 'y', getattr(kv[1], 'start_y', 0.0)), 4)))
    return cands


def _zone_net_units(segs, vias, pad_pts, zones):
    """Candidate removal UNITS on a plane (zone) net. Deliberately narrow --
    stitching vias and a pin's own drop to the plane are intentional even
    when the pours make them topologically redundant (measured: a general
    leaf rule would strip 136 GND vias):

      * a trace RUN joining two vias A and B (one or more segments; interior
        joints touch exactly two segments and nothing else): "the pin is
        already dropped to the plane by a via, but the trace carries on to
        another via". If one of the vias has no other connection (no other
        trace, not in a pin), the run AND that via go; if both vias connect
        onward (e.g. the far via also serves a pin), ONLY the run goes;
      * a track with an end touching nothing at all (no same-net track, via,
        pin, or same-net pour on its layer).

    Runs that end on a pin or at a branch are never candidates, nor are vias
    with no trace (stitching). Every unit is still subject to the
    connectivity check, so a run that is a pin's only route to the plane
    (e.g. the far via is outside the pour) is kept."""
    from check_connected import point_in_polygon
    def on_pad(x, y, extra=0.0):
        return any(math.hypot(x - px, y - py) < r + extra + _EPS for px, py, r in pad_pts)

    def vias_at(x, y, skip=None):
        return [w for w in vias if w is not skip and
                math.hypot(w.x - x, w.y - y) < (getattr(w, 'size', 0.0) or 0.0) / 2.0 + _EPS]

    def segs_at(x, y, skip):
        return [t for t in segs if t is not skip and
                (math.hypot(t.start_x - x, t.start_y - y) < t.width / 2.0 + _EPS or
                 math.hypot(t.end_x - x, t.end_y - y) < t.width / 2.0 + _EPS or
                 _pt_seg(x, y, t) < t.width / 2.0 + _EPS)]

    def via_tracks(v):
        vr = (getattr(v, 'size', 0.0) or 0.0) / 2.0
        return [x for x in segs if _pt_seg(v.x, v.y, x) < vr + x.width / 2.0 - _EPS]

    def via_on_pad(v):
        return on_pad(v.x, v.y, (getattr(v, 'size', 0.0) or 0.0) / 2.0)

    seen = set()
    via_units, dead_units, run_units = [], [], []
    for a in vias:
        for first in via_tracks(a):
            # far end of `first` relative to via a
            def far_of(t, x, y):
                return ((t.end_x, t.end_y)
                        if math.hypot(t.start_x - x, t.start_y - y) <= math.hypot(t.end_x - x, t.end_y - y)
                        else (t.start_x, t.start_y))
            run = [first]
            cur, (fx, fy) = first, far_of(first, a.x, a.y)
            b = None
            for _ in range(10000):
                if on_pad(fx, fy):
                    break                          # reaches a pin: not a via-via run
                wv = vias_at(fx, fy, skip=a)
                if wv:
                    b = wv[0]
                    break
                nxt = segs_at(fx, fy, cur)
                if len(nxt) != 1:
                    break                          # dead end or branch
                cur = nxt[0]
                if cur in run:
                    break
                run.append(cur)
                fx, fy = far_of(cur, fx, fy)
            if b is None or b is a:
                continue
            key = frozenset(id(t) for t in run)
            if key in seen:
                continue
            seen.add(key)
            run_ids = {id(t) for t in run}

            def alone(v):
                return (not via_on_pad(v) and
                        all(id(t) in run_ids for t in via_tracks(v)))
            unit = [('seg', t) for t in run]
            if alone(b):
                via_units.append([('via', b)] + unit)
            elif alone(a):
                via_units.append([('via', a)] + unit)
            else:
                run_units.append(unit)
    for x in segs:
        for ex, ey in ((x.start_x, x.start_y), (x.end_x, x.end_y)):
            touched = (any(t is not x and _pt_seg(ex, ey, t) < t.width / 2.0 + _EPS for t in segs)
                       or any(math.hypot(ex - w.x, ey - w.y) < (getattr(w, 'size', 0.0) or 0.0) / 2.0 + _EPS
                              for w in vias)
                       or on_pad(ex, ey)
                       or any(z.layer == x.layer and point_in_polygon(ex, ey, z.polygon) for z in zones))
            if not touched:
                dead_units.append([('seg', x)])
                break
    # Continuations that take their via with them first (unwinding a chain
    # from its far end), then dead-end tracks, then trace-only runs; each
    # group in a deterministic geometric order.
    def _key(u):
        return tuple((k, round(getattr(o, 'x', getattr(o, 'start_x', 0.0)), 4),
                      round(getattr(o, 'y', getattr(o, 'start_y', 0.0)), 4)) for k, o in u)
    return (sorted(via_units, key=_key) + sorted(dead_units, key=_key)
            + sorted(run_units, key=_key))


def prune_redundant_leaf_copper(results, pcb_data, scope_net_ids=None,
                                protect_net_ids=None,
                                keep_input_copper: bool = False,
                                max_rounds: int = 500
                                ) -> Tuple[int, int, List, List, int]:
    """Returns (segments_removed, vias_removed, input_segments_to_strip,
    input_vias_to_strip, nets_touched)."""
    from check_connected import check_net_connectivity, make_real_fill_validator

    protect = set(protect_net_ids or ())
    routed_seg = {id(s) for r in results for s in (r.get('new_segments') or [])}
    routed_via = {id(v) for r in results for v in (r.get('new_vias') or [])}
    all_zones = getattr(pcb_data, 'zones', None) or []

    net_ids = sorted({s.net_id for s in pcb_data.segments} | {v.net_id for v in pcb_data.vias}
                     | {v.net_id for r in results for v in (r.get('new_vias') or [])})
    rm_seg, rm_via = set(), set()
    strip_segs, strip_vias = [], []
    nets_touched = 0
    for net_id in net_ids:
        if not net_id or net_id in protect:
            continue
        if scope_net_ids is not None and net_id not in scope_net_ids:
            continue
        segs = [s for s in pcb_data.segments if s.net_id == net_id]
        vias, seen = [], set()
        for v in [v for v in pcb_data.vias if v.net_id == net_id] + \
                 [v for r in results for v in (r.get('new_vias') or []) if v.net_id == net_id]:
            if id(v) not in seen:
                seen.add(id(v))
                vias.append(v)
        if not segs and not vias:
            continue
        pads = pcb_data.pads_by_net.get(net_id, [])
        if not pads:
            continue      # a pad-less net's copper is the orphan-island pass's business
        zones = [z for z in all_zones if z.net_id == net_id]
        zcv = make_real_fill_validator(pcb_data, net_id) if zones else None
        pad_pts = _pad_pts(pads)

        def grade(sg, vs):
            r = check_net_connectivity(net_id, sg, vs, pads, zones,
                                       zone_credit_validator=zcv, pcb_data=pcb_data)
            return (bool(r.get('connected')), len(r.get('disconnected_pads') or []),
                    r.get('num_components') or 1)

        base = grade(segs, vias)
        touched = False
        copper_layers = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])
        for _ in range(max_rounds):
            changed = False
            if zones:
                units = _zone_net_units(segs, vias, pad_pts, zones)
            else:
                single = []
                if copper_layers:
                    from pcb_modification import _via_support_layers
                    single = [v for v in vias
                              if len(_via_support_layers(v, segs, pads, zones, copper_layers)) <= 1]
                units = [[c] for c in _leaf_candidates(segs, vias, pad_pts, single)]
            for unit in units:
                # skip units already invalidated by an earlier removal this round
                if any((o not in vias) if k == 'via' else (o not in segs) for k, o in unit):
                    continue
                if any(getattr(o, 'locked', False) or getattr(o, 'graphic', False) for _, o in unit):
                    continue
                routed = [id(o) in (routed_via if k == 'via' else routed_seg) for k, o in unit]
                if keep_input_copper and not all(routed):
                    continue
                drop_v = {id(o) for k, o in unit if k == 'via'}
                drop_s = {id(o) for k, o in unit if k == 'seg'}
                trial_s = [x for x in segs if id(x) not in drop_s]
                trial_v = [x for x in vias if id(x) not in drop_v]
                after = grade(trial_s, trial_v)
                worse = ((base[0] and not after[0]) or after[1] > base[1] or after[2] > base[2])
                if worse:
                    continue
                segs, vias, base = trial_s, trial_v, after
                changed = touched = True
                for (k, o), rt in zip(unit, routed):
                    if k == 'via':
                        (rm_via.add(id(o)) if rt else strip_vias.append(o))
                    else:
                        (rm_seg.add(id(o)) if rt else strip_segs.append(o))
                if zones:
                    break    # plane nets: re-derive the units after every removal
            if not changed:
                break
        nets_touched += touched

    # Mirror into the write-lists and pcb_data (the board that will be written).
    if rm_seg or rm_via:
        for r in results:
            if r.get('new_segments'):
                r['new_segments'] = [s for s in r['new_segments'] if id(s) not in rm_seg]
            if r.get('new_vias'):
                r['new_vias'] = [v for v in r['new_vias'] if id(v) not in rm_via]
    ss = {id(s) for s in strip_segs} | rm_seg
    sv = {id(v) for v in strip_vias} | rm_via
    if ss:
        pcb_data.segments = [s for s in pcb_data.segments if id(s) not in ss]
    if sv:
        pcb_data.vias = [v for v in pcb_data.vias if id(v) not in sv]
    return (len(rm_seg) + len(strip_segs), len(rm_via) + len(strip_vias),
            strip_segs, strip_vias, nets_touched)
