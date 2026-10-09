"""Via-in-pad policy helpers for the post-route passes (pcbnew objects).

When via-in-pad is off (a same-net pad keep-off > 0, as KiCad records in the
project's kicad_routing_tools.same_net_pad_clearance), a via must keep that
distance from every same-net SMD pad -- except a part's EXPOSED centre pad,
where thermal / ground vias are normal practice.
"""

from __future__ import annotations

import json
import os

_NM = 1_000_000


def project_pad_keepoff(pro_path):
    """The project's recorded same-net pad keep-off in mm (> 0 means via-in-pad
    is off), or None when the project records nothing."""
    try:
        v = json.load(open(pro_path)).get('kicad_routing_tools', {}).get('same_net_pad_clearance')
        return float(v) if v is not None else None
    except Exception:                                          # noqa: BLE001
        return None


def is_exposed_pad(pad):
    """A part's large centre pad: SMD, at least 1 mm^2, the biggest pad of a
    footprint with 5+ pads, and at least 3x the median of its other pads."""
    if pad.HasHole():
        return False
    fp = pad.GetParentFootprint()
    if fp is None:
        return False
    pads = list(fp.Pads())
    if len(pads) < 5:
        return False

    def area(p):
        s = p.GetSize(p.GetPrincipalLayer()) if hasattr(p, 'GetPrincipalLayer') else p.GetSize()
        return (s.x / _NM) * (s.y / _NM)
    a = area(pad)
    if a < 1.0:
        return False
    others = sorted(area(p) for p in pads if p is not pad)
    if not others or a < max(area(p) for p in pads) - 1e-9:
        return False
    median = others[len(others) // 2]
    return a >= 3.0 * median


def via_respects_pads(via, pads, keepoff_mm, layers):
    """True when the via keeps keepoff_mm from every same-net SMD pad that is
    not an exposed centre pad (keepoff <= 0: via-in-pad allowed, always True)."""
    if not keepoff_mm or keepoff_mm <= 0:
        return True
    clr = int(round(keepoff_mm * _NM))
    for p in pads:
        if p.GetNetCode() != via.GetNetCode() or p.HasHole():
            continue
        for L in layers:
            if p.IsOnLayer(L) and via.IsOnLayer(L) and \
                    via.GetEffectiveShape(L).Collide(p.GetEffectiveShape(L), clr):
                if is_exposed_pad(p):
                    break
                return False
    return True


def exposed_parser_pads(pcb_data):
    """id()s of exposed centre pads among kicad_parser Pad objects -- the same
    test as is_exposed_pad (SMD, >= 1 mm^2, a footprint's biggest pad among
    5+, >= 3x the median of its other pads)."""
    from collections import defaultdict
    by_ref = defaultdict(list)
    for pads in pcb_data.pads_by_net.values():
        for p in pads:
            by_ref[getattr(p, 'component_ref', None)].append(p)
    out = set()
    for ref, pads in by_ref.items():
        if not ref or len(pads) < 5:
            continue
        areas = [(p.size_x * p.size_y, p) for p in pads]
        a, big = max(areas, key=lambda ap: ap[0])
        if a < 1.0 or (getattr(big, 'drill', 0) or 0) > 0:
            continue
        others = sorted(x for x, p in areas if p is not big)
        if others and a >= 3.0 * others[len(others) // 2]:
            out.add(id(big))
    return out
