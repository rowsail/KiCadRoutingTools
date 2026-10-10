"""Eliminate small-drill vias by ripping and re-routing around them.

Many fabs price a board by its SMALLEST drill, so a handful of 0.15 mm vias
can cost as much as hundreds. 'Small' and 'full size' come from Board Setup:
any via drilled below its minimum drill is small, and the replacement is its
minimum via (at least 0.45/0.2). For each small via:

  1. Find what stops a full-size via
     from sitting where it is: the foreign TRACKS that collide with it, using
     KiCad's own shapes and pairwise clearance. Pads cannot move and are not
     ripped.
  2. Rip those blocking nets plus the via's own net. For a plane net (one that
     owns a zone) only the via itself and tracks ending on it are removed; the
     pour carries the rest.
  3. Re-route the ripped nets with route.py, with small vias forbidden: the
     temporary project's Board Setup minimum is raised to the full-size via and
     the fab tier pinned to 'standard' so the ladder cannot escalate.
  4. Keep the result only if KiCad DRC (on the ORIGINAL rules) has no more
     unconnected items or errors than before and the small-via count fell
     without any new small via. Otherwise retry once letting route.py rip the
     nets it names as blockers, else restore.

Vias that share a blocker or a net are handled together.

CLI:  python3 small_via_eliminate.py in.kicad_pcb out.kicad_pcb [--plan plan.json]
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
# Where KiCad's pcbnew module lives: the first PYTHONPATH entry that has it
# (the dialog passes KiCad's own), else the local KiCad 10 install.
KICAD_PY = next((d for d in os.environ.get('PYTHONPATH', '').split(os.pathsep)
                 if d and os.path.exists(os.path.join(d, 'pcbnew.py'))),
                '/usr/local/lib/python3/dist-packages')


def _python():
    """A real python3: inside KiCad, sys.executable is the KiCad binary."""
    exe = sys.executable or ''
    if 'python' in os.path.basename(exe).lower():
        return exe
    return shutil.which('python3') or shutil.which('python') or exe


# ---------------------------------------------------------------- analysis --

def analyse(board_path, max_small_drill, full=(0.45, 0.2)):
    """[{'x','y','net','plane','blockers':set(net names)}] for every small via."""
    sys.path.insert(0, KICAD_PY)
    import pcbnew
    from stranded_pour_vias import _via_fits  # noqa: F401  (same shape rules)
    b = pcbnew.LoadBoard(board_path)
    layers = list(b.GetEnabledLayers().CuStack())
    zone_nets = {z.GetNetname() for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetCode() > 0}
    tracks = list(b.GetTracks())
    pads = [p for f in b.GetFootprints() for p in f.Pads()]
    ds = b.GetDesignSettings()
    out = []
    for v in [t for t in tracks if t.GetClass() == 'PCB_VIA'
              and t.GetDrillValue() <= int(round(max_small_drill * 1e6))]:
        c = v.GetPosition()
        d0, h0 = v.GetWidth(pcbnew.F_Cu), v.GetDrillValue()
        v.SetWidth(int(full[0] * 1e6))
        v.SetDrill(int(full[1] * 1e6))
        blockers, pad_block = set(), False
        hs = v.GetEffectiveHoleShape()
        for it in tracks + pads:
            if it is v or it.GetNetCode() == v.GetNetCode():
                continue
            bb = it.GetBoundingBox()
            if not (bb.GetLeft() - 2e6 <= c.x <= bb.GetRight() + 2e6 and
                    bb.GetTop() - 2e6 <= c.y <= bb.GetBottom() + 2e6):
                continue
            hit = False
            for L in layers:
                if it.IsOnLayer(L) and v.GetEffectiveShape(L).Collide(
                        it.GetEffectiveShape(L), max(v.GetOwnClearance(L), it.GetOwnClearance(L))):
                    hit = True
                    break
            if not hit and ds.m_HoleClearance and any(
                    it.IsOnLayer(L) and hs.Collide(it.GetEffectiveShape(L), ds.m_HoleClearance)
                    for L in layers):
                hit = True
            if not hit and hasattr(it, 'HasHole') and it.HasHole():
                ohs = it.GetEffectiveHoleShape()
                hit = ohs is not None and hs.Collide(ohs, ds.m_HoleToHoleMin)
            if hit:
                if it.GetClass() == 'PAD':
                    pad_block = True
                elif it.GetNetname() not in zone_nets:
                    blockers.add(it.GetNetname())
        v.SetWidth(d0)
        v.SetDrill(h0)
        in_pad = any(p.GetNetCode() == v.GetNetCode() and
                     any(p.IsOnLayer(L) and p.GetEffectiveShape(L).Collide(c, 0) for L in layers)
                     for p in pads)
        out.append({'x': c.x / 1e6, 'y': c.y / 1e6, 'net': v.GetNetname(),
                    'plane': v.GetNetname() in zone_nets, 'blockers': blockers,
                    'pad_block': pad_block, 'in_pad': in_pad})
    return out


def group(vias):
    """Merge vias whose rip sets (own non-plane net + blockers) overlap."""
    groups = []
    for v in vias:
        rip = set(v['blockers']) | ({v['net']} if not v['plane'] else set())
        g = {'vias': [v], 'rip': rip}
        merged = [x for x in groups if x['rip'] & rip]
        for x in merged:
            g['vias'] += x['vias']
            g['rip'] |= x['rip']
            groups.remove(x)
        groups.append(g)
    return groups


# ----------------------------------------------------------- board editing --

_BLOCK = re.compile(r'\n\t\((segment|via|arc)\b')


def _blocks(text):
    """Yield (start, end, kind) for every top-level segment/via/arc block."""
    i = 0
    while True:
        m = _BLOCK.search(text, i)
        if not m:
            return
        j, depth = m.start() + 2, 0
        while True:
            ch = text[j]
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
                if depth == 0:
                    break
            elif ch == '"':
                j = text.index('"', j + 1)
            j += 1
        yield m.start(), j + 1, m.group(1)
        i = j + 1


def strip(text, rip_nets, plane_vias):
    """Remove all copper of `rip_nets`, plus each plane-net via in
    `plane_vias` [(net, x, y)] and that net's tracks ending on it."""
    def _net(b):
        m = re.search(r'\(net "([^"]*)"\)', b)
        return m.group(1) if m else None

    def _pts(b):
        return [(float(a), float(c)) for a, c in
                re.findall(r'\((?:start|end|at) ([-\d.]+) ([-\d.]+)\)', b)]

    out, last, removed = [], 0, 0
    for s, e, kind in _blocks(text):
        b = text[s:e]
        n = _net(b)
        drop = n in rip_nets
        if not drop:
            for pn, x, y in plane_vias:
                if n == pn and any(abs(px - x) < 0.01 and abs(py - y) < 0.01 for px, py in _pts(b)):
                    drop = True
                    break
        if drop:
            out.append(text[last:s])
            last = e
            removed += 1
    out.append(text[last:])
    return ''.join(out), removed


# ------------------------------------------------------------------ checks --

def drc(board, pro):
    """(unconnected, error_keys Counter) from kicad-cli DRC with zones refilled."""
    import collections
    d = tempfile.mkdtemp(prefix='sve_drc_')
    try:
        b2 = os.path.join(d, 'b.kicad_pcb')
        shutil.copy(board, b2)
        shutil.copy(pro, os.path.join(d, 'b.kicad_pro'))
        dru = os.path.splitext(pro)[0] + '.kicad_dru'
        if os.path.exists(dru):
            shutil.copy(dru, os.path.join(d, 'b.kicad_dru'))
        rep = os.path.join(d, 'r.json')
        subprocess.run(['kicad-cli', 'pcb', 'drc', '--format', 'json', '--severity-all',
                        '--refill-zones', '-o', rep, b2], capture_output=True)
        j = json.load(open(rep))
        drc.last_json = j
        errs = collections.Counter(
            (x['type'], tuple(sorted(i['description'][:60] for i in x['items'])))
            for x in j['violations'] if x.get('severity') == 'error')
        drc.last_dangling = [(x['type'], i['description'], i['pos']['x'], i['pos']['y'])
                             for x in j['violations'] if x['type'] in ('via_dangling', 'track_dangling')
                             for i in x['items']]
        drc.last_unconnected = [' <-> '.join(f"{i['description'][:50]} @({i['pos']['x']:.2f},{i['pos']['y']:.2f})"
                                             for i in x['items']) for x in j.get('unconnected_items', [])]
        return len(j.get('unconnected_items', [])), errs
    finally:
        shutil.rmtree(d, ignore_errors=True)


def sweep_dangling(cur, pro_rules, work, base_unc, base_err, rounds=6):
    """Remove the vias / track stubs KiCad's DRC reports as dangling, one
    round at a time (a removed stub can expose the next), keeping a round
    only if unconnected items and errors do not rise. Returns
    (removed, base_unc, base_err, dangling_left)."""
    removed = 0
    unc, err = drc(cur, pro_rules)
    for rnd in range(rounds):
        dang = list(drc.last_dangling)
        if not dang:
            break
        text = open(cur).read()
        out, last, n = [], 0, 0
        for s0, e0, kind in _blocks(text):
            blk = text[s0:e0]
            net = re.search(r'\(net "([^"]*)"\)', blk)
            net = net.group(1) if net else None
            pts = [(float(x), float(y)) for x, y in
                   re.findall(r'\((?:start|end|at) ([-\d.]+) ([-\d.]+)\)', blk)]
            hit = False
            for typ, desc, x, y in dang:
                m = re.search(r'\[([^\]]*)\]', desc)
                if not m or m.group(1) != net:
                    continue
                if (typ == 'via_dangling') != (kind == 'via'):
                    continue
                if any(abs(px - x) < 0.005 and abs(py - y) < 0.005 for px, py in pts):
                    if kind != 'via':
                        lm = re.search(r'length ([\d.]+) mm', desc)
                        if lm and len(pts) >= 2:
                            ln = ((pts[0][0] - pts[1][0]) ** 2 + (pts[0][1] - pts[1][1]) ** 2) ** 0.5
                            if abs(ln - float(lm.group(1))) > 0.01:
                                continue
                    hit = True
                    break
            if hit:
                out.append(text[last:s0])
                last = e0
                n += 1
        out.append(text[last:])
        if not n:
            break
        cand = os.path.join(work, f'sweep{rnd}.kicad_pcb')
        open(cand, 'w').write(''.join(out))
        _with_rules(cand)
        unc2, err2 = drc(cand, pro_rules)
        if unc2 <= unc and sum((err2 - err).values()) == 0:
            shutil.copy(cand, cur)
            removed += n
            unc, err = unc2, err2
        else:
            break
    return removed, unc, err, len(drc.last_dangling)


def widen(cur, pro_rules, work, plan_args, base_unc, base_err):
    """Widen power-net necks (widen_necks.py) one power-net pattern at a
    time; keep each only if KiCad DRC shows no more unconnected items or
    errors (a wider track can split a small piece of pour off even when every
    clearance passes). Returns (widened, base_unc, base_err)."""
    def _vals(flag):
        if flag not in plan_args:
            return []
        i, out = plan_args.index(flag) + 1, []
        while i < len(plan_args) and not plan_args[i].startswith('--'):
            out.append(plan_args[i])
            i += 1
        return out
    nets, widths = _vals('--power-nets'), _vals('--power-nets-widths')
    if not nets or len(nets) != len(widths):
        return 0, base_unc, base_err
    total, kept, skipped = 0, [], []
    state = {'unc': base_unc, 'err': base_err}

    def attempt(pat, w, rng, tag):
        """Widen one pattern (optionally a [start, end) slice of its sorted
        necks) on a copy of cur; keep it if DRC does not get worse.
        Returns (necks_in_slice, widened_kept)."""
        cand = os.path.join(work, f'widened_{tag}.kicad_pcb')
        _with_rules(cur)
        cmd = [_python(), os.path.join(HERE, 'widen_necks.py'), cur, cand,
               '--power-nets', pat, '--power-nets-widths', w]
        if rng:
            cmd += ['--range', str(rng[0]), str(rng[1])]
        p = subprocess.run(cmd, capture_output=True, text=True, env=dict(os.environ, PYTHONPATH=KICAD_PY))
        m = re.search(r'WIDEN necks=(\d+) widened=(\d+) full=(\d+)', p.stdout)
        if not m or int(m.group(2)) == 0 or not os.path.exists(cand):
            return (int(m.group(1)) if m else 0), 0
        _with_rules(cand)
        unc, err = drc(cand, pro_rules)
        if unc <= state['unc'] and sum((err - state['err']).values()) == 0:
            shutil.copy(cand, cur)
            state['unc'], state['err'] = unc, err
            return int(m.group(1)), int(m.group(2))
        return int(m.group(1)), -1

    def bisect(pat, w, lo, hi, depth, tag):
        """Keep the halves of a failing slice that pass, down to 4 necks."""
        n, got = attempt(pat, w, (lo, hi), tag)
        if got >= 0:
            return max(got, 0)
        if depth >= 4 or hi - lo <= 4:
            return 0
        mid = (lo + hi) // 2
        return bisect(pat, w, lo, mid, depth + 1, tag + 'a') + bisect(pat, w, mid, hi, depth + 1, tag + 'b')

    for k, (pat, w) in enumerate(zip(nets, widths)):
        if _REPAIR_SCOPE is not None:
            import fnmatch
            if not any(fnmatch.fnmatchcase(pat, sc) or fnmatch.fnmatchcase(sc, pat)
                       for sc in _REPAIR_SCOPE):
                continue          # outside the repair scope: leave its widths alone
        n, got = attempt(pat, w, None, f'{k}')
        if got > 0:
            total += got
            kept.append(f"{pat}:{got}")
        elif got < 0:
            part = bisect(pat, w, 0, n, 0, f'{k}s')
            if part:
                total += part
                kept.append(f"{pat}:{part} (partial)")
            if part < n:
                skipped.append(pat)
    base_unc, base_err = state['unc'], state['err']
    if kept or skipped:
        print(f"Power necks widened: {total} segment(s) [{' '.join(kept)}]"
              + (f"; left narrow where DRC got worse: {skipped}" if skipped else ""))
    return total, base_unc, base_err


def count_small(board, max_small_drill):
    t = open(board).read()
    return sum(1 for s, e, k in _blocks(t) if k == 'via' and
               float(re.search(r'\(drill ([\d.]+)\)', t[s:e]).group(1)) <= max_small_drill + 1e-9)


# ------------------------------------------------------------------- route --

def route(inp, out, pro_rules, nets, full, plan_args, rip_existing=None, log=None,
          extra=()):
    """route.py on a temp copy whose Board Setup forbids vias below `full`."""
    d = os.path.dirname(inp)
    pro = json.load(open(pro_rules))
    r = pro['board']['design_settings']['rules']
    r['min_via_diameter'] = full[0]
    r['min_through_hole_diameter'] = full[1]
    json.dump(pro, open(os.path.splitext(inp)[0] + '.kicad_pro', 'w'), indent=2)
    dru = os.path.splitext(pro_rules)[0] + '.kicad_dru'
    if os.path.exists(dru):
        shutil.copy(dru, os.path.splitext(inp)[0] + '.kicad_dru')
    cmd = [_python(), os.path.join(HERE, 'route.py'), inp, '--output', out,
           '--nets', *nets, '--fab-tier', 'standard',
           # Board Setup's minimums are hard floors: a squeezed net may not
           # neck below the board's own track / clearance (the user's site
           # policy does the same inside KiCad; a CLI run never loads it).
           '--escalation', 'board'] + plan_args + list(extra)
    if rip_existing:
        cmd += ['--rip-existing-nets', *rip_existing]
    env = dict(os.environ, PYTHONPATH=KICAD_PY)
    p = subprocess.run(cmd, capture_output=True, text=True, env=env, cwd=d)
    if log:
        open(log, 'a').write(' '.join(cmd) + '\n' + p.stdout + p.stderr)
    hints = []
    for m in re.finditer(r"--rip-existing-nets ((?:'[^']+' ?)+)", p.stdout):
        hints += re.findall(r"'([^']+)'", m.group(1))
    return p.returncode == 0 and os.path.exists(out), sorted(set(hints))


_PRO_RULES = None     # the project the pass grades against (set in main)
_PAD_KEEPOFF = None   # same-net pad keep-off (mm) when via-in-pad is off
_REPAIR_SCOPE = None  # net globs the pass may change (None = all)


def _with_rules(board_path):
    """Make sure the project rules sit next to a board pcbnew will load: without
    a sibling .kicad_pro, LoadBoard falls back to KiCad's default rules."""
    if _PRO_RULES:
        base = os.path.splitext(board_path)[0]
        if os.path.abspath(base + '.kicad_pro') != os.path.abspath(_PRO_RULES):
            shutil.copy(_PRO_RULES, base + '.kicad_pro')
            dru = os.path.splitext(_PRO_RULES)[0] + '.kicad_dru'
            if os.path.exists(dru):
                shutil.copy(dru, base + '.kicad_dru')


def weld(board_path, full):
    """Weld pour islands the re-route cut off, with full-size vias only."""
    _with_rules(board_path)
    code = ("import sys, pcbnew; sys.path.insert(0, %r)\n"
            "from stranded_pour_vias import weld_stranded_pour_fragments\n"
            "b = pcbnew.LoadBoard(%r)\n"
            "s = weld_stranded_pour_fragments(b, min_via=%r, pad_keepoff=%r)\n"
            "print('WELD', s['fragments'], s['welded'])\n"
            "print('STRANDED', ' '.join(s.get('stranded_pads', [])))\n"
            "pcbnew.SaveBoard(%r, b) if s['welded'] else None\n") % (HERE, board_path, tuple(full), _PAD_KEEPOFF, board_path)
    p = subprocess.run([_python(), '-c', code], capture_output=True, text=True,
                       env=dict(os.environ, PYTHONPATH=KICAD_PY))
    m = re.search(r'WELD (\d+) (\d+)', p.stdout)
    st = re.search(r'STRANDED ?(.*)', p.stdout)
    weld.stranded = (st.group(1).split() if st else [])
    return (int(m.group(1)), int(m.group(2))) if m else (0, 0)


def stranded_groups(board_path, full, work, max_island_mm2=2.0, reach_mm=1.5):
    """Groups for plane-net pads boxed in by signal tracks: the welder found
    their pour island but could not reach any full-size via from it. For each
    such pad (island under max_island_mm2 -- big islands are tied in by other
    copper), pick the escape direction blocked ONLY by signal tracks (no pads,
    no plane copper) with the fewest nets, and rip those nets."""
    import math
    tmp = os.path.join(work, 'strand_probe.kicad_pcb')
    shutil.copy(board_path, tmp)
    weld(tmp, full)
    _with_rules(board_path)
    pad_ids = list(getattr(weld, 'stranded', []))
    if not pad_ids:
        return []
    sys.path.insert(0, KICAD_PY)
    import pcbnew
    b = pcbnew.LoadBoard(board_path)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    zone_nets = {z.GetNetname() for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetCode() > 0}
    items = list(b.GetTracks()) + [p for f in b.GetFootprints() for p in f.Pads()]
    groups = []
    for pid in pad_ids:
        ref, num = pid.split('.', 1)
        fp = b.FindFootprintByReference(ref)
        pad = next((p for p in fp.Pads() if p.GetNumber() == num), None) if fp else None
        if pad is None:
            continue
        c = pad.GetPosition()
        L = next((l for l in b.GetEnabledLayers().CuStack() if pad.IsOnLayer(l)), None)
        area = None
        for z in b.Zones():
            if z.GetNetCode() == pad.GetNetCode() and z.IsOnLayer(L) and not z.GetIsRuleArea():
                fps = z.GetFilledPolysList(L)
                for i in range(fps.OutlineCount()):
                    if fps.Contains(c, i):
                        area = fps.Outline(i).Area() / 1e12
        if area is None or area > max_island_mm2:
            continue
        near = [it for it in items if it.GetNetCode() != pad.GetNetCode() and it.IsOnLayer(L)
                and abs(it.GetBoundingBox().Centre().x - c.x) < (reach_mm + 2) * 1e6
                and abs(it.GetBoundingBox().Centre().y - c.y) < (reach_mm + 2) * 1e6]
        best = None
        for deg in range(0, 360, 15):
            a = math.radians(deg)
            t = pcbnew.PCB_TRACK(b)
            t.SetStart(c)
            t.SetEnd(pcbnew.VECTOR2I(c.x + int(reach_mm * 1e6 * math.cos(a)), c.y + int(reach_mm * 1e6 * math.sin(a))))
            t.SetWidth(127000)
            t.SetLayer(L)
            t.SetNetCode(pad.GetNetCode())
            sh = t.GetEffectiveShape(L)
            hits = [it for it in near if sh.Collide(it.GetEffectiveShape(L),
                                                    max(t.GetOwnClearance(L), it.GetOwnClearance(L)))]
            if any(it.GetClass() == 'PAD' or it.GetNetname() in zone_nets for it in hits):
                continue
            nets = {it.GetNetname() for it in hits}
            if best is None or len(nets) < len(best):
                best = nets
        if best:
            groups.append({'vias': [], 'pads': [pid], 'rip': set(best),
                           'pt': (c.x / 1e6, c.y / 1e6), 'island': area})
    return groups


def keepoff_violations(board_path):
    """[(via net, x, y, plane?)] for vias that break the same-net pad keep-off
    (exposed centre pads exempt). Empty when via-in-pad is allowed."""
    if not _PAD_KEEPOFF:
        return []
    _with_rules(board_path)
    code = ("import sys, json, pcbnew; sys.path.insert(0, %r)\n"
            "from pad_rules import via_respects_pads\n"
            "b = pcbnew.LoadBoard(%r)\n"
            "layers = list(b.GetEnabledLayers().CuStack())\n"
            "pads = [p for f in b.GetFootprints() for p in f.Pads()]\n"
            "zn = {z.GetNetname() for z in b.Zones() if not z.GetIsRuleArea() and z.GetNetCode() > 0}\n"
            "out = [(v.GetNetname(), v.GetPosition().x / 1e6, v.GetPosition().y / 1e6, v.GetNetname() in zn)\n"
            "       for v in b.GetTracks() if v.GetClass() == 'PCB_VIA'\n"
            "       and not via_respects_pads(v, pads, %r, layers)]\n"
            "print('KEEPOFF ' + json.dumps(out))\n") % (HERE, board_path, _PAD_KEEPOFF)
    p = subprocess.run([_python(), '-c', code], capture_output=True, text=True,
                       env=dict(os.environ, PYTHONPATH=KICAD_PY))
    m = re.search(r'KEEPOFF (.*)', p.stdout)
    return [tuple(x) for x in json.loads(m.group(1))] if m else []


def repair_groups(board_path, zone_nets):
    """Groups for what the last DRC (drc.last_json) and the keep-off check
    report: keep-off violators, unconnected signal nets, starved thermal pads.
    Each rips only the nets involved."""
    groups = []
    for net, x, y, plane in keepoff_violations(board_path):
        v = {'net': net, 'x': x, 'y': y, 'plane': plane, 'blockers': set(), 'pad_block': False,
             'in_pad': True}
        groups.append({'vias': [v], 'rip': set() if plane else {net}, 'repair': 'keep-off',
                       'label': f"via in/at a {net} pad @({x:.2f},{y:.2f})"})
    j = getattr(drc, 'last_json', {}) or {}
    seen = set()
    for x in j.get('unconnected_items', []):
        for it in x['items']:
            m = re.search(r'\[([^\]]*)\]', it['description'])
            if m and m.group(1) not in zone_nets and m.group(1) not in seen:
                seen.add(m.group(1))
                groups.append({'vias': [], 'rip': {m.group(1)}, 'repair': 'unconnected',
                               'label': f"unconnected {m.group(1)}",
                               'pt': (it['pos']['x'], it['pos']['y'])})
    for x in j.get('violations', []):
        if x['type'] != 'clearance':
            continue
        for it in x['items']:
            if not it['description'].startswith(('Via', 'Track')):
                continue
            m = re.search(r'\[([^\]]*)\]', it['description'])
            if m and m.group(1) not in zone_nets:
                groups.append({'vias': [], 'rip': {m.group(1)}, 'repair': 'clearance',
                               'label': f"clearance error on {m.group(1)}",
                               'pt': (it['pos']['x'], it['pos']['y'])})
                break
    for x in j.get('unconnected_items', []):
        # a plane net cut by signal tracks: re-route the signal nets with copper
        # near the cut, kept off that net's plane layer(s)
        its = [i for i in x['items'] if not i['description'].startswith('Zone')]
        m = re.search(r'\[([^\]]*)\]', its[0]['description']) if its else None
        if not m or m.group(1) not in zone_nets:
            continue
        near = set()
        for i in its:
            near |= set(signal_nets_near(board_path, i['pos']['x'], i['pos']['y'], 2.5, zone_nets))
        if near:
            groups.append({'vias': [], 'rip': near, 'repair': 'plane cut',
                           'avoid_net_layers': m.group(1),
                           'label': f"{m.group(1)} plane cut near ({its[0]['pos']['x']:.1f},{its[0]['pos']['y']:.1f})",
                           'pt': (its[0]['pos']['x'], its[0]['pos']['y'])})
    for x in j.get('violations', []):
        if x['type'] != 'starved_thermal':
            continue
        pad = next((i for i in x['items'] if i['description'].startswith('Pad')), None)
        if pad is None:
            continue
        near = signal_nets_near(board_path, pad['pos']['x'], pad['pos']['y'], 0.6, zone_nets)
        if near:
            groups.append({'vias': [], 'rip': set(near), 'repair': 'starved thermal',
                           'label': f"starved thermal {pad['description'][:40]}",
                           'pt': (pad['pos']['x'], pad['pos']['y'])})
    return groups


def signal_nets_near(board_path, x, y, radius_mm, zone_nets):
    """Signal nets with track copper within radius_mm of (x, y)."""
    t = open(board_path).read()
    out = set()
    for s0, e0, kind in _blocks(t):
        if kind == 'via':
            continue
        blk = t[s0:e0]
        net = re.search(r'\(net "([^"]*)"\)', blk)
        if not net or net.group(1) in zone_nets:
            continue
        pts = [(float(a), float(b)) for a, b in re.findall(r'\((?:start|end) ([-\d.]+) ([-\d.]+)\)', blk)]
        w = float(re.search(r'\(width ([\d.]+)\)', blk).group(1)) if '(width' in blk else 0
        if len(pts) == 2:
            (x1, y1), (x2, y2) = pts
            dx, dy = x2 - x1, y2 - y1
            L2 = dx * dx + dy * dy
            u = 0 if L2 == 0 else max(0, min(1, ((x - x1) * dx + (y - y1) * dy) / L2))
            d = ((x1 + u * dx - x) ** 2 + (y1 + u * dy - y) ** 2) ** 0.5 - w / 2
            if d < radius_mm:
                out.add(net.group(1))
    return sorted(out)


def avoid_plane_layers(plan_args, board_path, net):
    """plan_args with every copper layer that carries a pour of `net`
    forbidden (layer cost -1) -- route.py's --layer-costs is per layer in
    board stack order."""
    t = open(board_path).read()
    layers = re.findall(r'\n\t\t\(\d+ "([^"]+\.Cu)" (?:signal|power|mixed|jumper)', t) or \
        ['F.Cu', 'In1.Cu', 'In2.Cu', 'B.Cu']
    # route.py orders --layer-costs by the STACK (F.Cu, In1..InN, B.Cu); KiCad 9+
    # files list copper by layer id (F.Cu, B.Cu, In1, ...), so sort by stack.
    def _stack(L):
        if L == 'F.Cu':
            return (0, 0)
        if L == 'B.Cu':
            return (2, 0)
        m = re.match(r'In(\d+)\.Cu', L)
        return (1, int(m.group(1)) if m else 0)
    layers = sorted(set(layers), key=_stack)
    plane = set()
    for m in re.finditer(r'\n\t\(zone\b', t):
        blk = t[m.start():m.start() + 600]
        n = re.search(r'\((?:net_name|net) "([^"]+)"\)', blk)
        if n and n.group(1) == net:
            ls = re.search(r'\(layers? ([^)]*)\)', blk)
            if ls:
                plane |= set(re.findall(r'"([^"]+)"', ls.group(1)))
    args = list(plan_args)
    if '--layer-costs' in args:
        i = args.index('--layer-costs') + 1
        costs = []
        while i + len(costs) < len(args) and not args[i + len(costs)].startswith('--'):
            costs.append(args[i + len(costs)])
    else:
        args += ['--layer-costs']
        i = len(args)
        costs = ['1.0'] * len(layers)
        args += costs
    for k, L in enumerate(layers[:len(costs)]):
        if L in plane and L not in ('F.Cu', 'B.Cu'):      # inner plane layers only
            args[i + k] = '-1'
    return args


def pads_near(board_path, pts, radius_mm):
    """{'REF.NUM'} of pads within radius_mm of any of pts [(x, y) mm]."""
    sys.path.insert(0, KICAD_PY)
    import pcbnew
    b = pcbnew.LoadBoard(board_path)
    out = set()
    for f in b.GetFootprints():
        for p in f.Pads():
            c = p.GetPosition()
            if any(((c.x / 1e6 - x) ** 2 + (c.y / 1e6 - y) ** 2) ** 0.5 <= radius_mm for x, y in pts):
                out.add(f"{f.GetReference()}.{p.GetNumber()}")
    return out


def reserve(board_path, pad_ids, full):
    """Give caged pads a stub + full-size via on the PRE-route board."""
    _with_rules(board_path)
    code = ("import sys, pcbnew; sys.path.insert(0, %r)\n"
            "from stranded_pour_vias import reserve_escape\n"
            "b = pcbnew.LoadBoard(%r)\n"
            "n = reserve_escape(b, %r, %r, pad_keepoff=%r)\n"
            "print('RESERVED', n)\n"
            "pcbnew.SaveBoard(%r, b) if n else None\n") % (HERE, board_path, list(pad_ids), tuple(full), _PAD_KEEPOFF, board_path)
    p = subprocess.run([_python(), '-c', code], capture_output=True, text=True,
                       env=dict(os.environ, PYTHONPATH=KICAD_PY))
    m = re.search(r'RESERVED (\d+)', p.stdout)
    return int(m.group(1)) if m else 0


def board_via_minimum(pro_path):
    """(min via diameter, min drill) mm from the project's Board Setup, or (0, 0)."""
    try:
        r = json.load(open(pro_path))['board']['design_settings']['rules']
        return float(r.get('min_via_diameter') or 0), float(r.get('min_through_hole_diameter') or 0)
    except Exception:                                          # noqa: BLE001
        return 0.0, 0.0


def plan_route_args(plan_path):
    """Layer costs / power widths / heuristic from the plan's main route step."""
    if not plan_path:
        return []
    p = json.load(open(plan_path))
    steps = [s for s in p['steps'] if s['action'] == 'route']
    s = next((s for s in steps if s['params'].get('power_nets')), steps[0] if steps else None)
    if not s:
        return []
    a, pr = [], s['params']
    if pr.get('layer_costs'):
        a += ['--layer-costs', *[str(x) for x in pr['layer_costs']]]
    if pr.get('power_nets'):
        a += ['--power-nets', *pr['power_nets'],
              '--power-nets-widths', *[str(x) for x in pr['power_nets_widths']]]
    if pr.get('heuristic_weight'):
        a += ['--heuristic-weight', str(pr['heuristic_weight'])]
    if pr.get('track_width'):
        a += ['--track-width', str(pr['track_width'])]
    return a


# -------------------------------------------------------------------- main --

def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('input')
    ap.add_argument('output')
    ap.add_argument('--plan', help='plan JSON to take route parameters from')
    ap.add_argument('--repair-scope', nargs='+', default=None, metavar='NET_GLOB',
                    help='only rip / re-route nets matching these globs; any repair or '
                         'replacement that would need other nets is skipped (e.g. after '
                         'routing a few nets, leave hand-routed nets alone)')
    ap.add_argument('--pad-keepoff', type=float, default=None,
                    help='same-net pad keep-off in mm (via-in-pad off when > 0; -1 = allowed). '
                         'Default: the project\'s kicad_routing_tools.same_net_pad_clearance')
    ap.add_argument('--route-args', default=None,
                    help='route.py arguments as one string (overrides --plan)')
    ap.add_argument('--max-small-drill', type=float, default=None,
                    help='drills at or below this count as small (default: anything '
                         'below Board Setup\'s minimum drill; 0.15 if that is not set above it)')
    ap.add_argument('--full-via', type=float, nargs=2, default=None,
                    metavar=('DIA', 'DRILL'))
    a = ap.parse_args()

    pro_rules = os.path.splitext(a.input)[0] + '.kicad_pro'
    work = tempfile.mkdtemp(prefix='sve_')
    log = os.path.splitext(a.output)[0] + '.small_vias.log'
    open(log, 'w').close()
    cur = os.path.join(work, 'cur.kicad_pcb')
    shutil.copy(a.input, cur)
    global _PRO_RULES, _PAD_KEEPOFF, _REPAIR_SCOPE
    _PRO_RULES = pro_rules
    _REPAIR_SCOPE = a.repair_scope
    if a.pad_keepoff is None:
        from pad_rules import project_pad_keepoff
        a.pad_keepoff = project_pad_keepoff(pro_rules)
    _PAD_KEEPOFF = a.pad_keepoff if (a.pad_keepoff or 0) > 0 else None
    print("Via-in-pad: " + (f"OFF (vias keep {_PAD_KEEPOFF} mm from same-net pads; "
                            f"exposed centre pads exempt)" if _PAD_KEEPOFF else "allowed"))
    _with_rules(cur)
    import shlex
    plan_args = shlex.split(a.route_args) if a.route_args else plan_route_args(a.plan)
    bs_via, bs_drill = board_via_minimum(pro_rules)
    if a.full_via is None:
        # Board Setup's minimum via is the smallest the fab wants; use it as
        # the replacement size when it is at least the classic 0.45/0.2.
        a.full_via = (max(bs_via, 0.45), max(bs_drill, 0.2))
    if a.max_small_drill is None:
        a.max_small_drill = max(0.15, round(a.full_via[1] - 0.01, 4))
    full = tuple(a.full_via)
    print(f"Small = drill <= {a.max_small_drill} mm; replacement via {full[0]}/{full[1]} mm")

    base_unc, base_err = drc(cur, pro_rules)
    base_small = count_small(cur, a.max_small_drill)
    start_small = base_small
    start_unc = base_unc
    print(f"Start: {base_small} small via(s); KiCad unconnected {base_unc}, "
          f"{sum(base_err.values())} error(s)")
    vias = analyse(cur, a.max_small_drill, full)
    groups = group(vias)
    zone_nets = set()
    try:
        _t = open(cur).read()
        for _m in re.finditer(r'\n\t\(zone\b', _t):
            _n = re.search(r'\((?:net_name|net) "([^"]+)"\)', _t[_m.start():_m.start() + 400])
            if _n:
                zone_nets.add(_n.group(1))
    except Exception:                                          # noqa: BLE001
        pass
    base_viol = len(keepoff_violations(cur))
    drc(cur, pro_rules)
    groups += repair_groups(cur, zone_nets)
    if base_viol:
        print(f"Via-in-pad keep-off violations: {base_viol}")
    if base_unc > 0:
        probe = os.path.join(work, 'plain_weld.kicad_pcb')
        shutil.copy(cur, probe)
        frag, welded = weld(probe, full)
        if welded:
            u2, e2 = drc(probe, pro_rules)
            if u2 < base_unc and sum((e2 - base_err).values()) == 0:
                shutil.copy(probe, cur)
                print(f"Plain weld: {welded} pour island(s) reconnected with full-size vias "
                      f"(unconnected {base_unc} -> {u2})")
                base_unc, base_err = u2, e2
                drc(cur, pro_rules)
                groups = [g for g in groups if g.get('repair') != 'unconnected'] + \
                    [g for g in repair_groups(cur, zone_nets) if g.get('repair') in ('unconnected', 'plane cut')
                     and not any(g2.get('label') == g.get('label') for g2 in groups)]
    if base_unc > 0:
        sg = stranded_groups(cur, full, work)
        for g in sg:
            print(f"Boxed-in pad {g['pads'][0]} (pour island {g['island']:.2f} mm^2): "
                  f"escape needs {sorted(g['rip'])} moved")
        groups += sg
    if a.repair_scope:
        import fnmatch
        def _in_scope(n):
            return any(fnmatch.fnmatchcase(n, p) for p in a.repair_scope)
        kept = []
        for g in groups:
            if g.get('repair') and not g['vias']:
                inner = {n for n in g['rip'] if _in_scope(n)}
                if inner:
                    g['rip'] = inner
                    kept.append(g)
                continue
            nets_needed = set(g['rip']) | {v['net'] for v in g['vias']}
            if all(_in_scope(n) for n in nets_needed):
                kept.append(g)
        if len(kept) < len(groups):
            print(f"Repair scope {a.repair_scope}: {len(groups) - len(kept)} group(s) left alone "
                  f"(they would touch other nets)")
        groups = kept
    # One group per distinct job: several DRC items on the same net (e.g. one
    # misplaced via clashing with four neighbours) are one re-route, not four.
    _seen, _uniq = set(), []
    for g in groups:
        k = (g.get('repair'), frozenset(g['rip']),
             frozenset((v['net'], round(v['x'], 2), round(v['y'], 2)) for v in g['vias']),
             tuple(g.get('pads') or ()))
        if k not in _seen:
            _seen.add(k)
            _uniq.append(g)
    groups = _uniq
    for gi, g in enumerate(groups, 1):
        names = g.get('label') or ', '.join(f"{v['net']} @({v['x']:.2f},{v['y']:.2f})" for v in g['vias']) or \
            f"boxed-in pad {g['pads'][0]}"
        plane_vias = [(v['net'], v['x'], v['y']) for v in g['vias'] if v['plane']]
        # Plane nets are never handed to route.py here: routing a whole pour
        # net re-checks every one of its pads board-wide and, with small vias
        # forbidden, rips unrelated nets it then cannot restore (measured: a
        # GND pad repair 15 mm away broke +1.8V). Their small via is removed
        # and the weld step reconnects that pad locally with a full-size via.
        nets = sorted(g['rip'] | {v['net'] for v in g['vias'] if not v['plane']})
        print(f"\n[{gi}/{len(groups)}] {names}")
        print(f"   rip + re-route: {nets}"
              + ("  (a pad also crowds the full-size site)" if any(v['pad_block'] for v in g['vias']) else ""))
        text, n = strip(open(cur).read(), g['rip'], plane_vias)
        stripped = os.path.join(work, f'g{gi}_in.kicad_pcb')
        open(stripped, 'w').write(text)
        accepted = False
        rip_existing = None
        reserved = set()
        if g.get('pads'):
            # A boxed-in pad: give it its escape BEFORE the signals go back.
            nres = reserve(stripped, g['pads'], full)
            reserved.update(g['pads'])
            print(f"   reserved {nres} full-size escape(s) for {g['pads']}")
        near_pads = pads_near(cur, [(v['x'], v['y']) for v in g['vias']] + ([g['pt']] if g.get('pt') else []), 3.0)
        for attempt in (1, 2, 3):
            outp = os.path.join(work, f'g{gi}_a{attempt}.kicad_pcb')
            # A via that already sat in a pad may sit in a pad again (it is
            # filled and capped either way); otherwise the project's same-net
            # pad keep-out seals pads that have no other via site in reach.
            # Via-in-pad only where the board allows it: with a keep-off the
            # project's own setting reaches route.py through the .kicad_pro.
            extra = (['--same-net-pad-clearance', '0']
                     if not _PAD_KEEPOFF and any(v['in_pad'] for v in g['vias']) else [])
            # Plane nets first, on their own: a pad that must escape the pour
            # gap (e.g. a GND pad between signal pins) gets its path before
            # the ripped signal nets are laid back around it.
            ok, hints = True, []
            g_args = plan_args
            if g.get('avoid_net_layers'):
                g_args = avoid_plane_layers(plan_args, cur, g['avoid_net_layers'])
            if nets:
                ok, hints = route(stripped, outp, pro_rules, nets, full, g_args,
                                  rip_existing=rip_existing, log=log, extra=extra)
            else:
                shutil.copy(stripped, outp)
            if ok:
                frag, welded = weld(outp, full)
                if welded:
                    print(f"   attempt {attempt}: welded {welded} cut-off pour island(s) with full-size vias")
            if not ok:
                print(f"   attempt {attempt}: route.py failed")
            else:
                unc, err = drc(outp, pro_rules)
                small = count_small(outp, a.max_small_drill)
                worse = sum((err - base_err).values())
                print(f"   attempt {attempt}: unconnected {unc} (was {base_unc}), "
                      f"new errors {worse}, small vias {small} (was {base_small})")
                if unc > base_unc:
                    for u in drc.last_unconnected[:4]:
                        print(f"      unconnected: {u}")
                # Only pads near the vias being replaced: islands elsewhere
                # (e.g. RF ground pads already tied in by other copper) are
                # not this group's business.
                caged = [pid for pid in getattr(weld, 'stranded', [])
                         if pid not in reserved and pid in near_pads]
                if g.get('repair'):
                    viol = len(keepoff_violations(outp)) if _PAD_KEEPOFF else 0
                    better = (unc < base_unc or sum(err.values()) < sum(base_err.values())
                              or viol < base_viol)
                    gain = better and unc <= base_unc and small <= base_small and viol <= base_viol
                    if gain:
                        base_viol = viol
                else:
                    gain = None
                gain = gain if gain is not None else (unc < base_unc and small <= base_small) if g.get('pads') else \
                    (unc <= base_unc and small < base_small)
                # Any attempt that did not pass and boxed in a nearby pad: give
                # that pad its escape before the next attempt (an unchanged
                # count can hide one pad fixed and another newly caged).
                if not (gain and worse == 0) and caged and attempt < 3:
                    nres = reserve(stripped, caged, full)
                    reserved.update(caged)
                    print(f"   caged pad(s) {caged}: reserved {nres} full-size escape(s) before the next attempt")
                if gain and worse == 0:
                    shutil.copy(outp, cur)
                    base_unc, base_err, base_small = unc, err, small
                    accepted = True
                    print("   ACCEPTED")
                    break
            more = [h for h in hints if h not in nets]
            if more and not rip_existing:
                rip_existing = more
                print(f"   retrying with route.py allowed to rip {more}")
            elif attempt < 3:
                print("   retrying (routing is not fully deterministic)")
        if not accepted:
            print("   kept the original copper here")
    widened, base_unc, base_err = widen(cur, pro_rules, work, plan_args, base_unc, base_err)
    gone, base_unc, base_err, dang_left = sweep_dangling(cur, pro_rules, work, base_unc, base_err)
    if gone or dang_left:
        print(f"Dangling sweep: removed {gone} dangling via/track stub(s); {dang_left} left")
    shutil.copy(cur, a.output)
    out_pro = os.path.splitext(a.output)[0] + '.kicad_pro'
    if os.path.abspath(out_pro) != os.path.abspath(pro_rules):
        shutil.copy(pro_rules, out_pro)
    print(f"\nDone: {base_small} small via(s) left; KiCad unconnected {base_unc}, "
          f"{sum(base_err.values())} error(s). Log: {log}")
    print(f"SVE_RESULT start={start_small} end={base_small} unc_start={start_unc} unc_end={base_unc} "
          f"swept={gone} widened={widened}")
    shutil.rmtree(work, ignore_errors=True)


if __name__ == '__main__':
    main()
