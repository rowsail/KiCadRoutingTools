"""Redundant copper pruning: the end-of-run tidy for copper the dead-end
sweep keeps.

Every removal is kept only when the authoritative zone-aware connectivity
check grades the net no worse (still connected if it was, no more stranded
pins, no more components).

Junk -- removed on ANY net, finished or not (never a landing site):
  * a duplicate via (barrels overlapping a same-net via) and any sliver
    lying inside the pair's barrels;
  * a duplicate segment (same layer, same two endpoints);
  * a sliver lying wholly inside one via's barrel;
  * a track whose end lies inside a same-net pin that has NO copper on the
    track's layer, with nothing else at that end -- aimed at a pin on
    another layer, it connects to nothing there.

Finished nets without pours -- LEAF items:
  * a via joining no layers (copper reaches it on at most one layer --
    KiCad's via_dangling) or touching at most one track and no pin;
  * a track with an end touching no other same-net track, via, or pin on
    the track's layer.

Finished plane (pour) nets -- deliberately narrow (a general leaf rule
measured 136 GND stitching vias "redundant" through the pours):
  * a trace run between two vias (plain bends only): the pin is already on
    the plane, the trace carries on to another via. If one via has no other
    connection, the run AND that via go (chains unwind from the far end);
    if both vias connect onward (the far via also serves a pin), only the
    run goes;
  * a track with an end touching nothing; an end inside the net's own pour
    counts as touching only when the stub starts at a pin (a deliberate fat
    pin-to-pour connection).
  Stitching vias and a pin's own via are never candidates.

Unfinished (protected) nets keep their dead ends -- the landing sites the
reconciliation retries from -- and locked copper and footprint graphics are
never touched. Removed copper is split by origin like sweep_dead_ends (this
run's copper leaves the write-lists, input-file copper is returned for the
writer / GUI applier to strip) and pcb_data is updated (#319).

Performance: one spatial index per net answers every "what reaches this
point" query from a single cell; removals are applied in rounds (units near
a removal wait for the next round, so chains still unwind from the far end),
and units the connectivity check refused are not retried unless something
nearby changed.
"""

from __future__ import annotations

import math
from collections import defaultdict
from typing import List, Tuple

_EPS = 1e-6
_CELL = 0.5          # spatial-index cell (mm)
_STALE_R = 2.0       # units this close to a removal wait for the next round


_SHADOW_MIN_LEN = 0.05     # mm: shorter pieces are left to the sliver rules
_SHADOW_OVERLAP = 0.005    # mm: copper must truly overlap, not just meet
_SHADOW_CONTAIN_TOL = 0.005  # mm
_SHADOW_PARALLEL_MIN_LEN = 1.0  # mm


def _pt_seg(px, py, s) -> float:
    ax, ay, bx, by = s.start_x, s.start_y, s.end_x, s.end_y
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    t = 0.0 if l2 <= 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _vr(v) -> float:
    return (getattr(v, 'size', 0.0) or 0.0) / 2.0


class _NetGeom:
    """Spatial index of one net's live tracks / vias / pins."""

    def __init__(self, segs, vias, pads, zones):
        self.segs = list(segs)
        self.vias = list(vias)
        self.alive = set(id(o) for o in self.segs) | set(id(o) for o in self.vias)
        self.zones = zones
        self.sgrid = defaultdict(list)
        self.vgrid = defaultdict(list)
        for s in self.segs:
            r = s.width / 2.0 + _EPS
            for c in self._cells(min(s.start_x, s.end_x) - r, min(s.start_y, s.end_y) - r,
                                 max(s.start_x, s.end_x) + r, max(s.start_y, s.end_y) + r):
                self.sgrid[c].append(s)
        for v in self.vias:
            r = _vr(v) + _EPS
            for c in self._cells(v.x - r, v.y - r, v.x + r, v.y + r):
                self.vgrid[c].append(v)
        # pins: (x, y, r, layers|None, half_x, half_y, cos, sin)
        self.pins = []
        for p in pads:
            if getattr(p, 'pad_type', '') == 'np_thru_hole':
                continue
            pl = list(getattr(p, 'layers', None) or [])
            lay = None if ((getattr(p, 'drill', 0) or 0) > 0 or any('*' in L for L in pl)) \
                else {L for L in pl if L.endswith('.Cu')}
            rot = math.radians(getattr(p, 'rect_rotation', 0.0) or 0.0)
            hx, hy = getattr(p, 'size_x', 0.5) / 2.0, getattr(p, 'size_y', 0.5) / 2.0
            self.pins.append((getattr(p, 'global_x', 0.0), getattr(p, 'global_y', 0.0),
                              max(hx, hy), lay, hx, hy, math.cos(rot), math.sin(rot)))

    @staticmethod
    def _cells(x0, y0, x1, y1):
        for cx in range(int(math.floor(x0 / _CELL)), int(math.floor(x1 / _CELL)) + 1):
            for cy in range(int(math.floor(y0 / _CELL)), int(math.floor(y1 / _CELL)) + 1):
                yield (cx, cy)

    @staticmethod
    def _key(x, y):
        return (int(math.floor(x / _CELL)), int(math.floor(y / _CELL)))

    def remove(self, o):
        self.alive.discard(id(o))

    def restore(self, o):
        self.alive.add(id(o))

    def live(self, o):
        return id(o) in self.alive

    def segs_reaching(self, x, y, layer=None):
        return [t for t in self.sgrid.get(self._key(x, y), ())
                if id(t) in self.alive and (layer is None or t.layer == layer)
                and _pt_seg(x, y, t) < t.width / 2.0 + _EPS]

    def vias_at(self, x, y, skip=None):
        return [w for w in self.vgrid.get(self._key(x, y), ())
                if id(w) in self.alive and w is not skip and math.hypot(w.x - x, w.y - y) < _vr(w) + _EPS]

    def via_tracks(self, v):
        r = _vr(v)
        out, seen = [], set()
        for c in self._cells(v.x - r, v.y - r, v.x + r, v.y + r):
            for t in self.sgrid.get(c, ()):
                if id(t) in self.alive and id(t) not in seen and _pt_seg(v.x, v.y, t) < r + t.width / 2.0 - _EPS:
                    seen.add(id(t))
                    out.append(t)
        return out

    def touchers(self, x, ex, ey, ox, oy, same_layer=False):
        """Other tracks reaching end (ex, ey) of x, excluding those that also
        reach x's other end (ox, oy) -- on a sliver shorter than a track
        half-width the neighbour joined at the other end covers this end too."""
        return [t for t in self.segs_reaching(ex, ey, x.layer if same_layer else None)
                if t is not x and not _pt_seg(ox, oy, t) < t.width / 2.0 + _EPS]

    def end_on_pin(self, x, y, layer):
        return any(math.hypot(x - px, y - py) < r + _EPS and (lay is None or layer in lay)
                   for px, py, r, lay, *_ in self.pins)

    def via_in_pin(self, v):
        r = _vr(v)
        return any(math.hypot(v.x - px, v.y - py) < pr + r for px, py, pr, *_ in self.pins)

    def pin_dist(self, v):
        return min((math.hypot(v.x - p[0], v.y - p[1]) for p in self.pins), default=0.0)

    def over_wrong_layer_pin(self, x, y, layer):
        for px, py, _r, lay, hx, hy, c, sn in self.pins:
            if lay is None or layer in lay:
                continue
            dx, dy = x - px, y - py
            if abs(dx * c + dy * sn) <= hx + _EPS and abs(-dx * sn + dy * c) <= hy + _EPS:
                return True
        return False

    def in_pour(self, x, y, layer):
        from check_connected import point_in_polygon
        return any(z.layer == layer and point_in_polygon(x, y, z.polygon) for z in self.zones)

    def support_layers(self, v, copper_layers):
        """Layers same-net copper reaches the via on (KiCad via_dangling: <= 1)."""
        from check_connected import point_in_polygon
        sup = {t.layer for t in self.via_tracks(v)}
        r = _vr(v)
        for px, py, pr, lay, *_ in self.pins:
            if math.hypot(v.x - px, v.y - py) < pr + r:
                sup |= set(copper_layers) if lay is None else lay
        for z in self.zones:
            if point_in_polygon(v.x, v.y, z.polygon):
                sup.add(z.layer)
        return sup & set(copper_layers)


def _ends(x):
    return (((x.start_x, x.start_y), (x.end_x, x.end_y)),
            ((x.end_x, x.end_y), (x.start_x, x.start_y)))


# Units whose removal provably cannot disconnect anything (a pure leaf or a
# pure copy) skip the connectivity check; keyed by frozenset of item ids.
_SAFE = set()


def _safe(unit):
    _SAFE.add(frozenset(id(o) for _, o in unit))
    return unit


def _seg_seg(a, b) -> float:
    """Distance between two segments' centrelines."""
    return min(_pt_seg(a.start_x, a.start_y, b), _pt_seg(a.end_x, a.end_y, b),
               _pt_seg(b.start_x, b.start_y, a), _pt_seg(b.end_x, b.end_y, a))


def _end_free(g, x, ex, ey, ox, oy):
    """x is a PURE LEAF hanging from its other end (ox, oy): nothing touches
    it anywhere except there -- no other track along its body (a T-joint
    mid-body makes it a spine, not a tail), no via or pin on its body, and
    no same-net pour on its layer at all. Removing such a segment cannot
    disconnect anything, so the connectivity check is skipped."""
    if any(z.layer == x.layer for z in g.zones):
        return False
    r_o = x.width / 2.0
    seen = set()
    for c in g._cells(min(x.start_x, x.end_x) - 1.0, min(x.start_y, x.end_y) - 1.0,
                      max(x.start_x, x.end_x) + 1.0, max(x.start_y, x.end_y) + 1.0):
        for t in g.sgrid.get(c, ()):
            if t is x or id(t) in seen or not g.live(t):
                continue
            seen.add(id(t))
            if _seg_seg(x, t) < (x.width + t.width) / 2.0 + _EPS:
                # allowed only if it touches x solely at the other end
                if _pt_seg(ox, oy, t) >= t.width / 2.0 + r_o + _EPS:
                    return False
        for v in g.vgrid.get(c, ()):
            if not g.live(v) or id(v) in seen:
                continue
            seen.add(id(v))
            if _pt_seg(v.x, v.y, x) < _vr(v) + r_o + _EPS and \
                    math.hypot(v.x - ox, v.y - oy) >= _vr(v) + r_o + _EPS:
                return False
    for px, py, pr, lay, *_ in g.pins:
        if (lay is None or x.layer in lay) and _pt_seg(px, py, x) < pr + r_o + _EPS and \
                math.hypot(px - ox, py - oy) >= pr + r_o + _EPS:
            return False
    return True


def _junk_units(g: _NetGeom):
    units = []
    # duplicate vias
    done = set()
    for a in g.vias:
        if not g.live(a):
            continue
        for b in g.vias_at(a.x, a.y, skip=a):
            if math.hypot(a.x - b.x, a.y - b.y) >= min(_vr(a), _vr(b)):
                continue
            key = frozenset((id(a), id(b)))
            if key in done:
                continue
            done.add(key)

            def score(v):
                inp = g.via_in_pin(v)
                return (1 if inp else 0, -g.pin_dist(v) if inp else 0.0, len(g.via_tracks(v)))
            keep, drop = (a, b) if score(a) >= score(b) else (b, a)
            sl = {id(x): x for x in g.via_tracks(keep) + g.via_tracks(drop)
                  if all(math.hypot(ex - keep.x, ey - keep.y) < _vr(keep) + _EPS or
                         math.hypot(ex - drop.x, ey - drop.y) < _vr(drop) + _EPS
                         for ex, ey in ((x.start_x, x.start_y), (x.end_x, x.end_y)))}
            units.append([('via', drop)] + [('seg', x) for x in sl.values()])
    # duplicate segments
    seen = {}
    for x in g.segs:
        if not g.live(x):
            continue
        a = (round(x.start_x, 3), round(x.start_y, 3))
        b = (round(x.end_x, 3), round(x.end_y, 3))
        key = (x.layer,) + (a + b if a <= b else b + a)
        if key in seen:
            first = seen[key]
            dup = x if x.width <= first.width else first
            if dup is first:
                seen[key] = x
            units.append(_safe([('seg', dup)]))     # an exact copy
        else:
            seen[key] = x
    # shadow tracks: a segment whose whole centreline overlaps other same-net
    # copper on its layer adds nothing (measured: a single-ended join of USB-C
    # D+ pins ran 40 mm alongside the coupled pair's own D+, overlapping it,
    # and ended in a dangling stub). Graded like any unit, so a shadow that is
    # the only link for a via or pin on its body is kept.
    for x in g.segs:
        if not g.live(x):
            continue
        L = math.hypot(x.end_x - x.start_x, x.end_y - x.start_y)
        if L < _SHADOW_MIN_LEN:
            continue
        cand = []
        seen = set()
        r = x.width / 2.0
        for c in g._cells(min(x.start_x, x.end_x) - 1.0, min(x.start_y, x.end_y) - 1.0,
                          max(x.start_x, x.end_x) + 1.0, max(x.start_y, x.end_y) + 1.0):
            for t in g.sgrid.get(c, ()):
                if t is x or id(t) in seen or not g.live(t) or t.layer != x.layer:
                    continue
                seen.add(id(t))
                if _seg_seg(x, t) < (x.width + t.width) / 2.0 - _SHADOW_OVERLAP:
                    cand.append(t)
        if not cand:
            continue
        n = max(2, int(L / 0.1) + 1)
        pts = [(x.start_x + k / (n - 1) * (x.end_x - x.start_x),
                x.start_y + k / (n - 1) * (x.end_y - x.start_y)) for k in range(n)]
        # (a) its copper lies wholly inside other same-net copper: removing it
        #     changes nothing physical, at any length or width;
        contained = all(any(_pt_seg(px, py, t) + r <= t.width / 2.0 + _SHADOW_CONTAIN_TOL
                            for t in cand) for px, py in pts)
        # (b) a LONG parallel overlap of a track at least as wide (the USB
        #     case: same width, offset 0.15 mm). Never a wider track under a
        #     thinner one -- that would neck a power trace.
        wide = [t for t in cand if t.width >= x.width - 1e-6]
        overlapped = (L >= _SHADOW_PARALLEL_MIN_LEN and wide and
                      all(any(_pt_seg(px, py, t) < r + t.width / 2.0 - _SHADOW_OVERLAP
                              for t in wide) for px, py in pts))
        if contained or overlapped:
            units.append([('seg', x)])
    # slivers inside one via; ends aimed at a pin on another layer
    for x in g.segs:
        if not g.live(x):
            continue
        if any(math.hypot(x.end_x - v.x, x.end_y - v.y) < _vr(v) + _EPS
               for v in g.vias_at(x.start_x, x.start_y)):
            units.append(_safe([('seg', x)]))       # wholly inside one barrel
            continue
        for (ex, ey), (ox, oy) in _ends(x):
            if (g.over_wrong_layer_pin(ex, ey, x.layer) and not g.vias_at(ex, ey)
                    and not g.touchers(x, ex, ey, ox, oy, same_layer=True)
                    and not g.end_on_pin(ex, ey, x.layer)):
                units.append([('seg', x)])
                break
    return units


def _leaf_units(g: _NetGeom, copper_layers):
    via_u, seg_u = [], []
    for v in g.vias:
        if not g.live(v):
            continue
        if copper_layers and len(g.support_layers(v, copper_layers)) <= 1:
            via_u.append([('via', v)])
        elif not g.via_in_pin(v) and len(g.via_tracks(v)) <= 1:
            pour = any(g.in_pour(v.x, v.y, z.layer) for z in g.zones)
            via_u.append(_safe([('via', v)]) if not pour else [('via', v)])
    for s in g.segs:
        if not g.live(s):
            continue
        for (ex, ey), (ox, oy) in _ends(s):
            if not (g.touchers(s, ex, ey, ox, oy) or g.vias_at(ex, ey) or g.end_on_pin(ex, ey, s.layer)):
                seg_u.append([('seg', s)])
                break
    return via_u + seg_u


def _plane_units(g: _NetGeom):
    via_u, dead_u, run_u = [], [], []
    seen = set()

    def far_of(t, x, y):
        return ((t.end_x, t.end_y)
                if math.hypot(t.start_x - x, t.start_y - y) <= math.hypot(t.end_x - x, t.end_y - y)
                else (t.start_x, t.start_y))

    for a in g.vias:
        if not g.live(a):
            continue
        for first in g.via_tracks(a):
            run, cur = [first], first
            fx, fy = far_of(first, a.x, a.y)
            b = None
            for _ in range(10000):
                if g.end_on_pin(fx, fy, cur.layer):
                    break                                   # reaches a pin
                wv = g.vias_at(fx, fy, skip=a)
                if wv:
                    b = wv[0]
                    break
                nxt = [t for t in g.segs_reaching(fx, fy) if t is not cur]
                if len(nxt) != 1 or nxt[0] in run:
                    break                                   # dead end or branch
                cur = nxt[0]
                run.append(cur)
                fx, fy = far_of(cur, fx, fy)
            if b is None:
                continue
            key = frozenset(id(t) for t in run)
            if key in seen:
                continue
            seen.add(key)
            ids = set(key)

            def alone(v):
                return not g.via_in_pin(v) and all(id(t) in ids for t in g.via_tracks(v))
            unit = [('seg', t) for t in run]
            if alone(b):
                via_u.append([('via', b)] + unit)
            elif alone(a):
                via_u.append([('via', a)] + unit)
            else:
                run_u.append(unit)
    for x in g.segs:
        if not g.live(x):
            continue
        for (ex, ey), (ox, oy) in _ends(x):
            touched = bool(g.touchers(x, ex, ey, ox, oy) or g.vias_at(ex, ey)
                           or g.end_on_pin(ex, ey, x.layer))
            if not touched and g.end_on_pin(ox, oy, x.layer) and g.in_pour(ex, ey, x.layer):
                touched = True                              # pin-to-pour stub
            if not touched:
                dead_u.append([('seg', x)])
                break
    return via_u + dead_u + run_u


def _pos(o):
    if hasattr(o, 'start_x'):
        return ((o.start_x + o.end_x) / 2.0, (o.start_y + o.end_y) / 2.0)
    return (o.x, o.y)


def prune_redundant_leaf_copper(results, pcb_data, scope_net_ids=None,
                                protect_net_ids=None,
                                keep_input_copper: bool = False,
                                max_rounds: int = 200
                                ) -> Tuple[int, int, List, List, int]:
    """Returns (segments_removed, vias_removed, input_segments_to_strip,
    input_vias_to_strip, nets_touched)."""
    from check_connected import check_net_connectivity, make_real_fill_validator

    _SAFE.clear()
    protect = set(protect_net_ids or ())
    routed_seg = {id(s) for r in results for s in (r.get('new_segments') or [])}
    routed_via = {id(v) for r in results for v in (r.get('new_vias') or [])}
    all_zones = getattr(pcb_data, 'zones', None) or []
    copper_layers = list(getattr(pcb_data.board_info, 'copper_layers', None) or [])

    segs_by = defaultdict(list)
    for s in pcb_data.segments:
        segs_by[s.net_id].append(s)
    vias_by = defaultdict(list)
    seen_v = set()
    for v in list(pcb_data.vias) + [v for r in results for v in (r.get('new_vias') or [])]:
        if id(v) not in seen_v:
            seen_v.add(id(v))
            vias_by[v.net_id].append(v)

    rm_seg, rm_via = set(), set()
    strip_segs, strip_vias = [], []
    nets_touched = 0
    for net_id in sorted(set(segs_by) | set(vias_by)):
        if not net_id:
            continue
        if scope_net_ids is not None and net_id not in scope_net_ids:
            continue
        pads = pcb_data.pads_by_net.get(net_id, [])
        if not pads:
            continue      # a pin-less net's copper is the orphan-island pass's business
        segs, vias = segs_by.get(net_id, []), vias_by.get(net_id, [])
        if not segs and not vias:
            continue
        sig = (net_id in protect, len(segs), len(vias),
               round(sum(s.start_x + 3 * s.start_y + 7 * s.end_x + 11 * s.end_y + 13 * s.width for s in segs), 6),
               round(sum(v.x + 3 * v.y + 7 * v.size for v in vias), 6),
               tuple(sorted({s.layer for s in segs})))
        memo = getattr(pcb_data, '_redundant_copper_memo', None)
        if memo is None:
            memo = {}
            try:
                pcb_data._redundant_copper_memo = memo
            except Exception:
                pass
        if memo.get(net_id) == sig:
            continue          # unchanged since a pass that found nothing to remove
        zones = [z for z in all_zones if z.net_id == net_id]
        g = _NetGeom(segs, vias, pads, zones)
        dup_only = net_id in protect
        zcv = make_real_fill_validator(pcb_data, net_id) if zones else None

        def grade():
            ls = [x for x in g.segs if g.live(x)]
            lv = [x for x in g.vias if g.live(x)]
            r = check_net_connectivity(net_id, ls, lv, pads, zones,
                                       zone_credit_validator=zcv, pcb_data=pcb_data)
            return (bool(r.get('connected')), len(r.get('disconnected_pads') or []),
                    r.get('num_components') or 1)

        base = None
        refused = {}          # unit key -> representative position
        touched = False
        for _ in range(max_rounds):
            units = _junk_units(g)
            if not dup_only:
                units += _plane_units(g) if zones else _leaf_units(g, copper_layers)
            removed_pts = []
            for unit in units:
                if any(not g.live(o) for _, o in unit):
                    continue
                if any(getattr(o, 'locked', False) or getattr(o, 'graphic', False) for _, o in unit):
                    continue
                key = frozenset(id(o) for _, o in unit)
                if key in refused:
                    continue
                pts = [_pos(o) for _, o in unit]
                if any(math.hypot(px - qx, py - qy) < _STALE_R for px, py in pts for qx, qy in removed_pts):
                    continue      # near this round's removals: re-derive next round
                routed = [id(o) in (routed_via if k == 'via' else routed_seg) for k, o in unit]
                if keep_input_copper and not all(routed):
                    continue
                if key in _SAFE:
                    for _, o in unit:
                        g.remove(o)
                    after = base
                else:
                    if base is None:
                        base = grade()
                    for _, o in unit:
                        g.remove(o)
                    after = grade()
                if after is not None and base is not None and (
                        (base[0] and not after[0]) or after[1] > base[1] or after[2] > base[2]):
                    for _, o in unit:
                        g.restore(o)
                    refused[key] = pts[0]
                    continue
                base = after
                touched = True
                removed_pts.extend(pts)
                for (k, o), rt in zip(unit, routed):
                    if k == 'via':
                        (rm_via.add(id(o)) if rt else strip_vias.append(o))
                    else:
                        (rm_seg.add(id(o)) if rt else strip_segs.append(o))
            if not removed_pts:
                break
            # a removal can change a nearby refusal's verdict: retry those
            refused = {k: p for k, p in refused.items()
                       if not any(math.hypot(p[0] - qx, p[1] - qy) < _STALE_R for qx, qy in removed_pts)}
        nets_touched += touched
        if not touched:
            memo[net_id] = sig
        else:
            memo.pop(net_id, None)

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
