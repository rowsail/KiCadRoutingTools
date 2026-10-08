"""Weld stranded pour fragments to their plane with a via (post-route).

A pad of a plane net (e.g. GND) can end up in its own little piece of pour,
boxed in by signal tracks on that layer: the fill exists around the pad but is
cut off from the main pour, and no track can reach it on that layer. The fix
is a via inside the fragment, down (or up) to a layer where the same net's
pour is continuous.

For every filled island of a zone that holds an SMD pad of the zone's own
net but no via / through-hole pad of that net, try via sites inside the
island: the net's class via first, then the smallest via Board Setup allows.
Each site is checked with KiCad's own shapes (pairwise clearance on every
copper layer, copper-to-hole clearance, hole-to-hole spacing) and is kept only
when a refill shows KiCad's unconnected count went down; otherwise it is
removed again. Works on a live pcbnew BOARD.
"""

from __future__ import annotations

import math
from typing import Dict, List

import pcbnew

_NM = 1_000_000


def _nm(mm: float) -> int:
    return int(round(mm * _NM))


def _refill(board):
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.BuildConnectivity()


def _unconnected(board) -> int:
    return board.GetConnectivity().GetUnconnectedCount(False)


def _via_fits(board, via, layers, hole_clr_nm, h2h_nm, items) -> bool:
    for L in layers:
        vs = via.GetEffectiveShape(L)
        for it in items:
            if it.GetNetCode() == via.GetNetCode() or not it.IsOnLayer(L):
                continue
            clr = max(via.GetOwnClearance(L), it.GetOwnClearance(L))
            if vs.Collide(it.GetEffectiveShape(L), clr):
                return False
    hs = via.GetEffectiveHoleShape()
    for it in items:
        if hasattr(it, 'HasHole') and it.HasHole():
            ohs = it.GetEffectiveHoleShape()
            if ohs is not None and hs.Collide(ohs, h2h_nm):
                return False
        if hole_clr_nm > 0 and it.GetNetCode() != via.GetNetCode():
            for L in layers:
                if it.IsOnLayer(L) and hs.Collide(it.GetEffectiveShape(L), hole_clr_nm):
                    return False
    return True


def weld_stranded_pour_fragments(board, step_mm: float = 0.025, verbose: bool = False) -> Dict:
    """Returns {'fragments': n_found, 'welded': n, 'vias': [(x, y, net, d, h)]}."""
    ds = board.GetDesignSettings()
    layers = list(board.GetEnabledLayers().CuStack())
    hole_clr = ds.m_HoleClearance or 0
    h2h = ds.m_HoleToHoleMin or _nm(0.25)
    min_d = ds.m_ViasMinSize or _nm(0.3)
    min_h = ds.m_MinThroughDrill or _nm(0.15)

    _refill(board)
    before = _unconnected(board)
    out = {'fragments': 0, 'welded': 0, 'vias': [], 'unconnected_before': before}
    if before == 0:
        out['unconnected_after'] = 0
        return out

    pads = [p for fp in board.GetFootprints() for p in fp.Pads()]
    tracks = list(board.GetTracks())
    items = tracks + pads

    # Islands: (zone, layer, outline index) holding an own-net SMD pad and no
    # own-net via / TH pad.
    islands = []
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetCode() <= 0:
            continue
        net = z.GetNetCode()
        own_vias = [t for t in tracks if t.GetClass() == 'PCB_VIA' and t.GetNetCode() == net]
        own_th = [p for p in pads if p.GetNetCode() == net and p.HasHole()]
        own_smd = [p for p in pads if p.GetNetCode() == net and not p.HasHole()]
        for L in z.GetLayerSet().Seq():
            if L not in layers:
                continue
            fp = z.GetFilledPolysList(L)
            for i in range(fp.OutlineCount()):
                if any(fp.Contains(v.GetPosition(), i) for v in own_vias):
                    continue
                if any(fp.Contains(p.GetPosition(), i) for p in own_th):
                    continue
                hosted = [p for p in own_smd if p.IsOnLayer(L) and fp.Contains(p.GetPosition(), i)]
                if hosted:
                    islands.append((z, L, i, hosted))
    out['fragments'] = len(islands)

    for z, L, i, hosted in islands:
        fp = z.GetFilledPolysList(L)
        bb = fp.Outline(i).BBox()
        anchor = hosted[0].GetPosition()
        step = _nm(step_mm)
        cands = []
        x = bb.GetLeft()
        while x <= bb.GetRight():
            y = bb.GetTop()
            while y <= bb.GetBottom():
                p = pcbnew.VECTOR2I(int(x), int(y))
                if fp.Contains(p, i):
                    cands.append(p)
                y += step
            x += step
        cands.sort(key=lambda p: (p.x - anchor.x) ** 2 + (p.y - anchor.y) ** 2)
        # The net's usual via (most common size among its vias) first, then the
        # smallest Board Setup allows.
        from collections import Counter
        common = Counter((t.GetWidth(pcbnew.F_Cu), t.GetDrillValue()) for t in tracks
                         if t.GetClass() == 'PCB_VIA' and t.GetNetCode() == z.GetNetCode())
        sizes = [common.most_common(1)[0][0]] if common else []
        if (min_d, min_h) not in sizes:
            sizes.append((min_d, min_h))
        placed = None
        for d, h in sizes:
            near = []
            for it in items:
                ib = it.GetBoundingBox()
                if (ib.GetLeft() - _nm(3) <= anchor.x <= ib.GetRight() + _nm(3) and
                        ib.GetTop() - _nm(3) <= anchor.y <= ib.GetBottom() + _nm(3)):
                    near.append(it)
            for p in cands:
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(p)
                v.SetWidth(d)
                v.SetDrill(h)
                v.SetNetCode(z.GetNetCode())
                v.SetViaType(pcbnew.VIATYPE_THROUGH)
                if not _via_fits(board, v, layers, hole_clr, h2h, near):
                    continue
                board.Add(v)
                _refill(board)
                after = _unconnected(board)
                if after < before:
                    placed = v
                    before = after
                    out['welded'] += 1
                    out['vias'].append((round(p.x / _NM, 3), round(p.y / _NM, 3), z.GetNetname(),
                                        d / _NM, h / _NM))
                    break
                board.Remove(v)
                _refill(board)
                break       # the best-placed fitting via did not help: try the next size
            if placed is not None:
                break
    out['unconnected_after'] = before
    return out


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('input')
    ap.add_argument('output')
    a = ap.parse_args()
    board = pcbnew.LoadBoard(a.input)
    s = weld_stranded_pour_fragments(board)
    print(f"Stranded pour fragments: {s['fragments']} found, {s['welded']} welded "
          f"(KiCad unconnected {s['unconnected_before']} -> {s['unconnected_after']})")
    for v in s['vias']:
        print(f"  via {v[3]}/{v[4]} at ({v[0]}, {v[1]}) on {v[2]}")
    if s['welded']:
        pcbnew.SaveBoard(a.output, board)


if __name__ == '__main__':
    main()
