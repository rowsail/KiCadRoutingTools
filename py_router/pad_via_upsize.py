"""Grow vias that sit inside small pads (post-route, KiCad-exact checks).

Small vias (0.3/0.15 mm) let the router escape dense parts, but every one of
them is a costlier drill. A via that ends up INSIDE a pad usually has room to
grow: this pass enlarges such vias in two phases.

  1. Diameter: up to the largest size that stays inside the pad copper
     (never larger than any pad dimension), stepping down until it clears
     every foreign item on every copper layer it spans.
  2. Drill: up to diameter - 2 x minimum annular ring, stepping down until it
     meets hole-to-hole spacing and copper-to-hole clearance.

Only pads whose largest dimension is <= `max_pad_mm` (default 3 mm) qualify,
so big thermal pads with via arrays are left alone. Every check uses KiCad's
own shapes (GetEffectiveShape / GetEffectiveHoleShape / SHAPE.Collide) at the
pairwise clearance KiCad's DRC applies (max of both items' own clearance);
zones are not checked because a refill re-carves them around the new size.

Works on a live pcbnew BOARD (the dialog) or a file (CLI below).
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional

import pcbnew

_NM = 1_000_000


def _mm(v_nm: int) -> float:
    return v_nm / _NM


def _nm(v_mm: float) -> int:
    return int(round(v_mm * _NM))


def _copper_layers(board) -> List[int]:
    return [l for l in board.GetEnabledLayers().CuStack()]


def _ring_inside_pad(pad, cx, cy, r_nm, layer) -> bool:
    """True when the whole via ring (radius r_nm at cx, cy) lies inside the
    pad's copper on `layer` (32 rim samples plus a 1 um inset tolerance)."""
    for k in range(32):
        a = 2 * math.pi * k / 32
        p = pcbnew.VECTOR2I(int(cx + (r_nm - 1000) * math.cos(a)),
                            int(cy + (r_nm - 1000) * math.sin(a)))
        if not pad.GetEffectiveShape(layer).Collide(p, 0):
            return False
    return True


def _board_rules(board):
    ds = board.GetDesignSettings()
    ann = _mm(ds.m_ViasMinAnnularWidth) or 0.075
    h2h = _mm(ds.m_HoleToHoleMin) or 0.25
    hole_clr = _mm(ds.m_HoleClearance) or 0.0
    min_drill = _mm(ds.m_MinThroughDrill) or 0.15
    return ann, h2h, hole_clr, min_drill


def upsize_pad_vias(board, max_pad_mm: float = 3.0, step_mm: float = 0.05,
                    annular_mm: Optional[float] = None, max_drill_mm: Optional[float] = None,
                    dry_run: bool = False, verbose: bool = False,
                    only_exposed: bool = False) -> Dict:
    """Grow in-pad vias on `board` in place. Returns a summary dict:
    {'candidates', 'grown', 'unchanged', 'by_size': {(old_d, old_h) -> (new_d, new_h) counts},
     'changes': [(x, y, net, old_d, old_h, new_d, new_h)]}."""
    ann_b, h2h, hole_clr, _ = _board_rules(board)
    ann = annular_mm if annular_mm is not None else ann_b
    layers = _copper_layers(board)

    pads = [p for fp in board.GetFootprints() for p in fp.Pads()]
    tracks = list(board.GetTracks())
    vias = [t for t in tracks if t.GetClass() == 'PCB_VIA']
    copper_items = [t for t in tracks] + pads

    def nearby(item_list, cx, cy, reach):
        out = []
        for it in item_list:
            bb = it.GetBoundingBox()
            if (bb.GetLeft() - reach <= cx <= bb.GetRight() + reach and
                    bb.GetTop() - reach <= cy <= bb.GetBottom() + reach):
                out.append(it)
        return out

    def host_pads(via):
        """Same-net small SMD pads that contain the via centre."""
        c = via.GetPosition()
        res = []
        for p in pads:
            if p.GetNetCode() != via.GetNetCode() or p.GetNetCode() <= 0:
                continue
            if p.HasHole():
                continue
            if only_exposed:
                from pad_rules import is_exposed_pad
                if not is_exposed_pad(p):
                    continue
            sz = p.GetSize(pcbnew.F_Cu) if hasattr(p, 'GetSize') else p.GetSize()
            if max(_mm(sz.x), _mm(sz.y)) > max_pad_mm:
                continue
            for L in layers:
                if p.IsOnLayer(L) and p.GetEffectiveShape(L).Collide(c, 0):
                    res.append((p, L))
                    break
        return res

    def copper_ok(via, d_nm, others):
        """Ring of diameter d_nm clears every foreign item on every layer."""
        via.SetWidth(d_nm)
        for L in layers:
            if not via.IsOnLayer(L):
                continue
            vs = via.GetEffectiveShape(L)
            for it in others:
                if it.GetNetCode() == via.GetNetCode() or not it.IsOnLayer(L):
                    continue
                clr = max(via.GetOwnClearance(L), it.GetOwnClearance(L))
                if vs.Collide(it.GetEffectiveShape(L), clr):
                    return False
        return True

    def hole_ok(via, h_nm, others):
        via.SetDrill(h_nm)
        hs = via.GetEffectiveHoleShape()
        for it in others:
            if it is via:
                continue
            # drill to drill (any net)
            if it.HasHole() if hasattr(it, 'HasHole') else False:
                ohs = it.GetEffectiveHoleShape()
                if ohs is not None and hs.Collide(ohs, _nm(h2h)):
                    return False
            # copper to hole (foreign nets)
            if hole_clr > 0 and it.GetNetCode() != via.GetNetCode():
                for L in layers:
                    if it.IsOnLayer(L) and hs.Collide(it.GetEffectiveShape(L), _nm(hole_clr)):
                        return False
        return True

    summary = {'candidates': 0, 'grown': 0, 'unchanged': 0, 'by_size': {}, 'changes': []}
    step = _nm(step_mm)
    for via in vias:
        hosts = host_pads(via)
        if not hosts:
            continue
        summary['candidates'] += 1
        c = via.GetPosition()
        d0, h0 = via.GetWidth(pcbnew.F_Cu), via.GetDrillValue()
        # Phase 1 upper bound: the largest ring inside every host pad, never
        # larger than any pad dimension.
        lim = min(min(p.GetSize(L).x, p.GetSize(L).y) for p, L in hosts)
        d_max = (lim // step) * step
        while d_max > d0 and not all(_ring_inside_pad(p, c.x, c.y, d_max // 2, L)
                                     for p, L in hosts):
            d_max -= step
        best_d = d0
        if d_max > d0:
            others = nearby(copper_items, c.x, c.y, d_max + _nm(1.0))
            d = d_max
            while d > d0:
                if copper_ok(via, d, others):
                    best_d = d
                    break
                d -= step
        via.SetWidth(best_d)
        # Phase 2: drill, keeping the minimum annular ring.
        h_max = ((best_d - 2 * _nm(ann)) // step) * step
        if max_drill_mm is not None:
            h_max = min(h_max, _nm(max_drill_mm))
        best_h = h0
        if h_max > h0:
            others = nearby(copper_items, c.x, c.y, best_d + _nm(1.0))
            h = h_max
            while h > h0:
                if hole_ok(via, h, others):
                    best_h = h
                    break
                h -= step
        via.SetDrill(best_h)
        key = (round(_mm(d0), 3), round(_mm(h0), 3))
        new = (round(_mm(best_d), 3), round(_mm(best_h), 3))
        if new != key:
            summary['grown'] += 1
            summary['by_size'].setdefault(f'{key[0]}/{key[1]} -> {new[0]}/{new[1]}', 0)
            summary['by_size'][f'{key[0]}/{key[1]} -> {new[0]}/{new[1]}'] += 1
            summary['changes'].append((round(_mm(c.x), 3), round(_mm(c.y), 3), via.GetNetname(),
                                       key[0], key[1], new[0], new[1]))
        else:
            summary['unchanged'] += 1
        if dry_run:
            via.SetWidth(d0)
            via.SetDrill(h0)
    return summary


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('input')
    ap.add_argument('output')
    ap.add_argument('--max-pad', type=float, default=3.0,
                    help='only pads whose largest dimension is <= this (mm, default 3)')
    ap.add_argument('--step', type=float, default=0.05, help='size step in mm (default 0.05)')
    ap.add_argument('--annular', type=float, default=None,
                    help='minimum annular ring in mm (default: Board Setup)')
    ap.add_argument('--max-drill', type=float, default=None, help='cap on the grown drill (mm)')
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args()
    board = pcbnew.LoadBoard(a.input)
    s = upsize_pad_vias(board, a.max_pad, a.step, a.annular, a.max_drill, a.dry_run)
    print(f"In-pad vias: {s['candidates']} candidates, {s['grown']} grown, {s['unchanged']} unchanged")
    for k, n in sorted(s['by_size'].items(), key=lambda kv: -kv[1]):
        print(f"  {n:4d} x {k}")
    if not a.dry_run:
        pcbnew.SaveBoard(a.output, board)


if __name__ == '__main__':
    main()
