"""Nudge a few unlocked parts so small-drill vias can become full-size.

For a hard spot (e.g. a fine-pitch part whose pins only escape through
0.15 mm vias), move the named parts (--movers) a little and re-route the
nets around them with small vias forbidden.

  1. Candidate poses: each mover translated within +/- --max-move mm (grid
     --step) in each of 4 rotations (or kept), legal when its courtyard
     overlaps no other courtyard and its pads clear all foreign copper that
     will stay (KiCad shapes, pairwise clearance).
  2. Proxy score per combination: the number of legal full-size via sites
     (clear of all remaining copper on every layer) within --reach mm of each
     small via's pad, with the ripped nets removed. The worst site count over
     the small vias is maximised, ties broken by total displacement.
  3. The best --trials combinations are re-routed for real (rip the small
     vias' nets and blockers, the movers' own stubs; route.py with small vias
     forbidden; weld cut-off pour islands with full-size vias) and checked with
     KiCad DRC. The first that removes small vias without adding unconnected
     items or errors is kept.

CLI: python3 nudge_for_vias.py in.kicad_pcb out.kicad_pcb --movers C12 C13 --plan plan.json
"""

from __future__ import annotations

import argparse
import itertools
import math
import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import small_via_eliminate as sve  # noqa: E402

NM = 1_000_000


def _pcbnew():
    sys.path.insert(0, sve.KICAD_PY)
    import pcbnew
    return pcbnew


def _near(it, x, y, r):
    bb = it.GetBoundingBox()
    return (bb.GetLeft() - r <= x <= bb.GetRight() + r and bb.GetTop() - r <= y <= bb.GetBottom() + r)


def strip_attached(path, refs):
    """Remove the movers' own stub copper from the board FILE (tracks with an
    end on one of their pads, vias inside their pads, same net) by editing the
    text -- pcbnew's BOARD.Remove() corrupts the footprint wrappers in KiCad
    10's Python bindings. Returns the number of blocks removed."""
    import re
    pcbnew = _pcbnew()
    b = pcbnew.LoadBoard(path)
    layers = list(b.GetEnabledLayers().CuStack())
    pads = [p for r in refs for p in b.FindFootprintByReference(r).Pads()]
    keys = set()
    for t in b.GetTracks():
        pts = [t.GetPosition()] if t.GetClass() == 'PCB_VIA' else [t.GetStart(), t.GetEnd()]
        for p in pads:
            if t.GetNetCode() == p.GetNetCode() and any(
                    any(p.IsOnLayer(L) and p.GetEffectiveShape(L).Collide(q, 0) for L in layers) for q in pts):
                keys.add(tuple(sorted((round(q.x / NM, 4), round(q.y / NM, 4)) for q in pts)) + (t.GetNetname(),))
                break
    text = open(path).read()
    out, last, n = [], 0, 0
    for s0, e0, kind in sve._blocks(text):
        blk = text[s0:e0]
        net = re.search(r'\(net "([^"]*)"\)', blk)
        pts = tuple(sorted((round(float(x), 4), round(float(y), 4)) for x, y in
                           re.findall(r'\((?:start|end|at) ([-\d.]+) ([-\d.]+)\)', blk)))
        if net and pts + (net.group(1),) in keys:
            out.append(text[last:s0])
            last = e0
            n += 1
    out.append(text[last:])
    open(path, 'w').write(''.join(out))
    return n


def remove_attached(board, fp):
    """Remove the movers' own stub copper: tracks with an end on one of its
    pads, and vias inside its pads (same net). Returns the count."""
    pcbnew = _pcbnew()
    layers = list(board.GetEnabledLayers().CuStack())
    gone = 0
    for t in list(board.GetTracks()):
        for p in fp.Pads():
            if t.GetNetCode() != p.GetNetCode():
                continue
            pts = [t.GetPosition()] if t.GetClass() == 'PCB_VIA' else [t.GetStart(), t.GetEnd()]
            if any(any(p.IsOnLayer(L) and p.GetEffectiveShape(L).Collide(q, 0) for L in layers) for q in pts):
                board.Remove(t)
                gone += 1
                break
    return gone


def pad_ok(board, fp, keep_nets_ripped, layers):
    """Mover pads clear every foreign item that will stay; courtyard clear."""
    pcbnew = _pcbnew()
    for p in fp.Pads():
        c = p.GetPosition()
        for it in list(board.GetTracks()) + [q for f in board.GetFootprints() if f is not fp for q in f.Pads()]:
            if it.GetNetCode() == p.GetNetCode() or it.GetNetname() in keep_nets_ripped:
                continue
            if not _near(it, c.x, c.y, 2 * NM):
                continue
            for L in layers:
                if p.IsOnLayer(L) and it.IsOnLayer(L) and p.GetEffectiveShape(L).Collide(
                        it.GetEffectiveShape(L), max(p.GetOwnClearance(L), it.GetOwnClearance(L))):
                    return False
    side = pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd
    try:
        cy = fp.GetCourtyard(side)
    except Exception:
        return True
    if cy.OutlineCount() == 0:
        return True
    for f in board.GetFootprints():
        if f is fp or f.IsFlipped() != fp.IsFlipped():
            continue
        bb, fb = fp.GetBoundingBox(), f.GetBoundingBox()
        if not bb.Intersects(fb):
            continue
        oc = f.GetCourtyard(side)
        if oc.OutlineCount() == 0:
            continue
        inter = pcbnew.SHAPE_POLY_SET(cy)
        inter.BooleanIntersection(oc)
        if inter.Area() > 1e3:          # > 0.001 mm^2 of courtyard overlap
            return False
    return True


def static_sites(board, x, y, reach_mm, full, net_code, layers, step_mm=0.1):
    """Legal full-size via VIA OBJECTS within reach of (x, y) [mm] against
    everything currently on the board (call with the movers parked away)."""
    pcbnew = _pcbnew()
    from stranded_pour_vias import _via_fits
    ds = board.GetDesignSettings()
    items = [it for it in list(board.GetTracks()) + [p for f in board.GetFootprints() for p in f.Pads()]
             if _near(it, x * NM, y * NM, (reach_mm + 1.5) * NM)]
    out = []
    r = int(round(reach_mm / step_mm))
    for i in range(-r, r + 1):
        for j in range(-r, r + 1):
            if i * i + j * j > r * r:
                continue
            v = pcbnew.PCB_VIA(board)
            v.SetWidth(int(full[0] * NM))
            v.SetDrill(int(full[1] * NM))
            v.SetNetCode(net_code)
            v.SetPosition(pcbnew.VECTOR2I(int((x + i * step_mm) * NM), int((y + j * step_mm) * NM)))
            if _via_fits(board, v, layers, ds.m_HoleClearance, ds.m_HoleToHoleMin, items):
                out.append(v)
    return out


def sites_left(sites, movers, layers, hole_clr):
    """How many cached sites the movers' pads (at their current pose) leave."""
    pads = [p for m in movers for p in m.Pads()]
    n = 0
    for v in sites:
        c = v.GetPosition()
        hit = False
        for p in pads:
            if p.GetNetCode() == v.GetNetCode() or not _near(p, c.x, c.y, NM):
                continue
            for L in layers:
                if p.IsOnLayer(L) and v.GetEffectiveShape(L).Collide(
                        p.GetEffectiveShape(L), max(v.GetOwnClearance(L), p.GetOwnClearance(L))):
                    hit = True
                    break
            if not hit and hole_clr and any(p.IsOnLayer(L) and v.GetEffectiveHoleShape().Collide(
                    p.GetEffectiveShape(L), hole_clr) for L in layers):
                hit = True
            if hit:
                break
        n += not hit
    return n


def via_sites(board, x, y, reach_mm, full, net_code, layers, step_mm=0.05):
    """Count legal full-size via sites within reach of (x, y) [mm]."""
    pcbnew = _pcbnew()
    from stranded_pour_vias import _via_fits
    ds = board.GetDesignSettings()
    items = [it for it in list(board.GetTracks()) + [p for f in board.GetFootprints() for p in f.Pads()]
             if _near(it, x * NM, y * NM, (reach_mm + 1.5) * NM)]
    v = pcbnew.PCB_VIA(board)
    v.SetWidth(int(full[0] * NM))
    v.SetDrill(int(full[1] * NM))
    v.SetNetCode(net_code)
    n = 0
    r = int(reach_mm / step_mm)
    for i in range(-r, r + 1):
        for j in range(-r, r + 1):
            if i * i + j * j > r * r:
                continue
            v.SetPosition(pcbnew.VECTOR2I(int((x + i * step_mm) * NM), int((y + j * step_mm) * NM)))
            if _via_fits(board, v, layers, ds.m_HoleClearance, ds.m_HoleToHoleMin, items):
                n += 1
    return n


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('input')
    ap.add_argument('output')
    ap.add_argument('--movers', nargs='+', required=True)
    ap.add_argument('--plan')
    ap.add_argument('--max-move', type=float, default=1.5)
    ap.add_argument('--step', type=float, default=0.25)
    ap.add_argument('--reach', type=float, default=1.0)
    ap.add_argument('--trials', type=int, default=4)
    ap.add_argument('--max-small-drill', type=float, default=0.15)
    ap.add_argument('--full-via', type=float, nargs=2, default=(0.45, 0.2))
    a = ap.parse_args()
    pcbnew = _pcbnew()
    full = tuple(a.full_via)
    pro_rules = os.path.splitext(a.input)[0] + '.kicad_pro'
    work = tempfile.mkdtemp(prefix='nfv_')
    log = os.path.splitext(a.output)[0] + '.nudge.log'
    open(log, 'w').close()
    plan_args = sve.plan_route_args(a.plan)

    base_unc, base_err = sve.drc(a.input, pro_rules)
    base_small = sve.count_small(a.input, a.max_small_drill)
    vias = sve.analyse(a.input, a.max_small_drill, full)
    rip = set()
    for v in vias:
        rip |= v['blockers']
        if not v['plane']:
            rip.add(v['net'])
    plane_vias = [(v['net'], v['x'], v['y']) for v in vias if v['plane']]
    print(f"Start: {base_small} small via(s) "
          + ', '.join(f"{v['net']}@({v['x']:.2f},{v['y']:.2f})" for v in vias)
          + f"; KiCad unconnected {base_unc}, {sum(base_err.values())} error(s)")
    print(f"Nets ripped for every trial: {sorted(rip)}")

    # Board with the small vias' nets ripped (the scoring base).
    text, _ = sve.strip(open(a.input).read(), rip, plane_vias)
    ripped = os.path.join(work, 'ripped.kicad_pcb')
    open(ripped, 'w').write(text)
    shutil.copy(pro_rules, os.path.join(work, 'ripped.kicad_pro'))

    print(f"Movers' stub copper removed: {strip_attached(ripped, a.movers)} item(s)")
    b = pcbnew.LoadBoard(ripped)
    layers = list(b.GetEnabledLayers().CuStack())
    netcode = {v['net']: b.FindNet(v['net']).GetNetCode() for v in vias}
    movers = [b.FindFootprintByReference(r) for r in a.movers]
    if any(m is None or m.IsLocked() for m in movers):
        sys.exit("a mover is missing or KiCad-locked")
    home = {m.GetReference(): (m.GetPosition(), m.GetOrientationDegrees()) for m in movers}
    attached_nets = {p.GetNetname() for m in movers for p in m.Pads()}

    # Legal poses per mover.
    steps = int(round(a.max_move / a.step))
    poses = {}
    for m in movers:
        p0, r0 = home[m.GetReference()]
        ok = []
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                if math.hypot(i, j) * a.step > a.max_move + 1e-9:
                    continue
                for rot in (r0, r0 + 90, r0 + 180, r0 + 270):
                    m.SetPosition(pcbnew.VECTOR2I(p0.x + int(i * a.step * NM), p0.y + int(j * a.step * NM)))
                    m.SetOrientationDegrees(rot % 360)
                    if pad_ok(b, m, rip, layers):
                        ok.append((i * a.step, j * a.step, rot % 360))
        m.SetPosition(p0)
        m.SetOrientationDegrees(r0)
        poses[m.GetReference()] = ok
        print(f"{m.GetReference()}: {len(ok)} legal pose(s) within {a.max_move} mm")

    def apply(combo):
        for m, (dx, dy, rot) in zip(movers, combo):
            p0, _ = home[m.GetReference()]
            m.SetPosition(pcbnew.VECTOR2I(p0.x + int(dx * NM), p0.y + int(dy * NM)))
            m.SetOrientationDegrees(rot)

    # Cache the legal sites with the movers parked far away, then score each
    # pose by how many of them the movers' pads leave free.
    for m in movers:
        m.SetPosition(pcbnew.VECTOR2I(m.GetPosition().x + 200 * NM, m.GetPosition().y))
    cache = [static_sites(b, v['x'], v['y'], a.reach, full, netcode[v['net']], layers) for v in vias]
    print("Full-size via sites near each small via with the movers out of the way: "
          + ', '.join(str(len(c)) for c in cache))
    hole_clr = b.GetDesignSettings().m_HoleClearance

    def score():
        return min(sites_left(c, movers, layers, hole_clr) for c in cache)

    apply([(0, 0, home[m.GetReference()][1]) for m in movers])
    s0 = score()
    print(f"Proxy at the current placement: {s0} full-size via site(s) at the worst small via")
    # Coordinate search: optimise one mover at a time, then the other again.
    best = [(0.0, 0.0, home[m.GetReference()][1]) for m in movers]
    best_s = (s0, 0.0)
    ranked = {}
    for sweep in range(2):
        for k, m in enumerate(movers):
            for pose in poses[m.GetReference()]:
                trial = list(best)
                trial[k] = pose
                if any(t1 is not t2 and t1 == t2 for t1, t2 in itertools.combinations(trial, 2)):
                    continue
                apply(trial)
                # movers must not collide with each other
                if not all(pad_ok(b, mm, rip, layers) for mm in movers):
                    continue
                disp = sum(math.hypot(dx, dy) for dx, dy, _ in trial)
                s = (score(), -disp)
                ranked[tuple(trial)] = s
                if s > best_s:
                    best_s, best = s, trial
    apply(best)
    top = sorted(ranked.items(), key=lambda kv: kv[1], reverse=True)[:a.trials]
    print("Best combinations by proxy (worst-site count, -displacement):")
    for combo, s in top:
        print(f"   {s}  " + '  '.join(f"{m.GetReference()} d=({dx:+.2f},{dy:+.2f}) rot={rot:.0f}"
                                      for m, (dx, dy, rot) in zip(movers, combo)))

    accepted = None
    for ti, (combo, s) in enumerate(top, 1):
        if s[0] <= s0:
            print(f"\nTrial {ti}: proxy no better than the current placement -- stop")
            break
        bt = pcbnew.LoadBoard(ripped)
        mv = [bt.FindFootprintByReference(r) for r in a.movers]
        for m, (dx, dy, rot) in zip(mv, combo):
            p0, _ = home[m.GetReference()]
            m.SetPosition(pcbnew.VECTOR2I(p0.x + int(dx * NM), p0.y + int(dy * NM)))
            m.SetOrientationDegrees(rot)
        tin = os.path.join(work, f't{ti}_in.kicad_pcb')
        pcbnew.SaveBoard(tin, bt)
        nets = sorted(rip | {v['net'] for v in vias} | attached_nets)
        print(f"\nTrial {ti}: " + '  '.join(f"{r} d=({dx:+.2f},{dy:+.2f}) rot={rot:.0f}"
                                         for r, (dx, dy, rot) in zip(a.movers, combo))
              + f"; re-routing {nets}")
        for attempt in (1, 2, 3):
            outp = os.path.join(work, f't{ti}_a{attempt}.kicad_pcb')
            src = tin
            planes = [n for n in nets if n in {pn for pn, _, _ in plane_vias} | (attached_nets - rip)]
            others = [n for n in nets if n not in planes]
            ok, hints = True, []
            if attempt == 2 and planes and others:
                mid = os.path.join(work, f't{ti}_a{attempt}_p.kicad_pcb')
                ok, hints = sve.route(src, mid, pro_rules, planes, full, plan_args, log=log,
                                      extra=['--same-net-pad-clearance', '0'])
                if ok:
                    src = mid
            if ok:
                ok, h2 = sve.route(src, outp, pro_rules, others if src != tin else nets, full,
                                   plan_args, log=log, extra=['--same-net-pad-clearance', '0'])
            if not ok:
                print(f"   attempt {attempt}: route.py failed")
                continue
            sve.weld(outp, full)
            unc, err = sve.drc(outp, pro_rules)
            small = sve.count_small(outp, a.max_small_drill)
            worse = sum((err - base_err).values())
            print(f"   attempt {attempt}: unconnected {unc} (was {base_unc}), new errors {worse}, "
                  f"small vias {small} (was {base_small})")
            if unc > base_unc:
                for u in sve.drc.last_unconnected[:3]:
                    print(f"      unconnected: {u}")
            if unc <= base_unc and worse == 0 and small < base_small:
                accepted = (combo, outp, small)
                break
        if accepted:
            break
    if accepted:
        shutil.copy(accepted[1], a.output)
        shutil.copy(pro_rules, os.path.splitext(a.output)[0] + '.kicad_pro')
        print(f"\nACCEPTED: small vias {base_small} -> {accepted[2]}; moves: "
              + '  '.join(f"{r} d=({dx:+.2f},{dy:+.2f}) rot={rot:.0f}"
                          for r, (dx, dy, rot) in zip(a.movers, accepted[0])))
    else:
        print("\nNo trial improved the board; nothing written.")
    print(f"Log: {log}")


if __name__ == '__main__':
    main()
