"""Widen power-net track necks after routing (KiCad-exact checks).

The router narrows power tracks where they enter pads and squeeze past
obstacles, often far more than it needs to (measured: 79 of 96 power necks on
one board had room for the net's full width). Each narrowed segment of a power
net is widened to the largest width that fits:

  * never wider than the net's power width;
  * at an end that lands on a pad, never wider than that pad's smaller side
    (a 0.4 mm track flooding a 0.3 mm fine-pitch pad helps nothing);
  * clear of every foreign item on its layer at KiCad's pairwise clearance,
    and of every foreign drilled hole at Board Setup's hole clearance.

Widths are tried in 0.025 mm steps from the cap downwards. Works on a live
pcbnew BOARD; the caller re-checks with KiCad DRC.
"""

from __future__ import annotations

import fnmatch
from typing import Dict, List, Sequence

import pcbnew

_NM = 1_000_000
STEP = 25_000


def power_targets(board, patterns: Sequence[str], widths: Sequence[float]) -> Dict[int, int]:
    """{net code: power width nm} from glob patterns and widths (first match wins)."""
    out = {}
    for code, net in board.GetNetsByNetcode().items():
        name = net.GetNetname()
        for pat, w in zip(patterns, widths):
            if fnmatch.fnmatchcase(name, pat):
                out[code] = int(round(float(w) * _NM))
                break
    return out


def widen_power_necks(board, targets: Dict[int, int], index_range=None) -> Dict:
    ds = board.GetDesignSettings()
    hole_clr = ds.m_HoleClearance or 0
    pads = [p for f in board.GetFootprints() for p in f.Pads()]
    tracks = [t for t in board.GetTracks() if t.GetClass() == 'PCB_TRACK']
    items = list(board.GetTracks()) + pads
    summary = {'necks': 0, 'widened': 0, 'full': 0, 'by_net': {}}

    def end_cap(t, pt, L):
        cap = None
        for p in pads:
            if p.GetNetCode() == t.GetNetCode() and p.IsOnLayer(L) and \
                    p.GetEffectiveShape(L).Collide(pt, 0):
                s = p.GetSize(L)
                c = min(s.x, s.y)
                cap = c if cap is None else min(cap, c)
        return cap

    necks = [t for t in tracks if targets.get(t.GetNetCode()) and t.GetWidth() < targets[t.GetNetCode()]]
    necks.sort(key=lambda t: (t.GetNetname(), t.GetStart().x, t.GetStart().y, t.GetEnd().x, t.GetEnd().y))
    if index_range is not None:
        necks = necks[index_range[0]:index_range[1]]
    for t in necks:
        target = targets[t.GetNetCode()]
        summary['necks'] += 1
        L = t.GetLayer()
        cap = target
        for pt in (t.GetStart(), t.GetEnd()):
            c = end_cap(t, pt, L)
            if c is not None:
                cap = min(cap, c)
        w0 = t.GetWidth()
        if cap <= w0:
            continue
        bb = t.GetBoundingBox()
        reach = target + 1_000_000
        near = [it for it in items
                if it is not t and it.GetNetCode() != t.GetNetCode()
                and it.GetBoundingBox().GetRight() >= bb.GetLeft() - reach
                and it.GetBoundingBox().GetLeft() <= bb.GetRight() + reach
                and it.GetBoundingBox().GetBottom() >= bb.GetTop() - reach
                and it.GetBoundingBox().GetTop() <= bb.GetBottom() + reach]
        best = w0
        w = (cap // STEP) * STEP
        while w > w0:
            t.SetWidth(w)
            sh = t.GetEffectiveShape(L)
            ok = True
            for it in near:
                if it.IsOnLayer(L) and sh.Collide(it.GetEffectiveShape(L),
                                                  max(t.GetOwnClearance(L), it.GetOwnClearance(L))):
                    ok = False
                    break
                if hole_clr and hasattr(it, 'HasHole') and it.HasHole():
                    hs = it.GetEffectiveHoleShape()
                    if hs is not None and sh.Collide(hs, hole_clr):
                        ok = False
                        break
            if ok:
                best = w
                break
            w -= STEP
        t.SetWidth(best)
        if best > w0:
            summary['widened'] += 1
            summary['full'] += best >= cap
            n = t.GetNetname()
            summary['by_net'][n] = summary['by_net'].get(n, 0) + 1
    return summary


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('input')
    ap.add_argument('output')
    ap.add_argument('--power-nets', nargs='+', required=True)
    ap.add_argument('--power-nets-widths', nargs='+', type=float, required=True)
    ap.add_argument('--range', nargs=2, type=int, default=None, metavar=('START', 'END'),
                    help='only necks [START, END) of the sorted list (for bisection)')
    a = ap.parse_args()
    b = pcbnew.LoadBoard(a.input)
    s = widen_power_necks(b, power_targets(b, a.power_nets, a.power_nets_widths),
                          tuple(a.range) if a.range else None)
    print(f"WIDEN necks={s['necks']} widened={s['widened']} full={s['full']} "
          + ' '.join(f"{k}:{v}" for k, v in sorted(s['by_net'].items())))
    if s['widened']:
        pcbnew.SaveBoard(a.output, b)


if __name__ == '__main__':
    main()
