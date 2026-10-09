"""Run py_router/small_via_eliminate.py on the live board (dialog / plans).

The pass works on files and calls route.py itself, so: save the live board
to a temp dir next to a copy of its project, run the pass in a real python3,
and if it removed small vias, copy back only the copper that changed.
"""

import os
import shlex
import shutil
import subprocess
import tempfile

import pcbnew

_NM = 1e6


def _py_router_dir():
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'py_router')


def _python():
    try:
        from .deps_check import _find_python_executable
        return _find_python_executable()
    except Exception:                                          # noqa: BLE001
        return shutil.which('python3') or 'python3'


def _route_args(config):
    a = []
    if config.get('track_width'):
        a += ['--track-width', str(config['track_width'])]
    if config.get('layer_costs'):
        a += ['--layer-costs'] + [str(c) for c in config['layer_costs']]
    if config.get('power_nets') and config.get('power_nets_widths') and \
            len(config['power_nets']) == len(config['power_nets_widths']):
        a += ['--power-nets'] + list(config['power_nets'])
        a += ['--power-nets-widths'] + [str(w) for w in config['power_nets_widths']]
    if config.get('heuristic_weight'):
        a += ['--heuristic-weight', str(config['heuristic_weight'])]
    return a


def _key(t, board):
    ln = board.GetLayerName(t.GetLayer())
    if t.GetClass() == 'PCB_VIA':
        p = t.GetPosition()
        return ('V', round(p.x / _NM, 4), round(p.y / _NM, 4), t.GetWidth(pcbnew.F_Cu),
                t.GetDrillValue(), t.GetNetname())
    s, e = t.GetStart(), t.GetEnd()
    ends = tuple(sorted(((round(s.x / _NM, 4), round(s.y / _NM, 4)),
                         (round(e.x / _NM, 4), round(e.y / _NM, 4)))))
    mid = ()
    if t.GetClass() == 'PCB_ARC':
        m = t.GetMid()
        mid = (round(m.x / _NM, 4), round(m.y / _NM, 4))
    return ('T', ln, ends, mid, t.GetWidth(), t.GetNetname())


def sync_copper(board, result_path):
    """Make the live board's tracks/vias equal the result file's.
    Returns (removed, added)."""
    res = pcbnew.LoadBoard(result_path)
    want = {}
    for t in res.GetTracks():
        want.setdefault(_key(t, res), t)
    have = {}
    for t in list(board.GetTracks()):
        have.setdefault(_key(t, board), []).append(t)
    removed = added = 0
    for k, items in have.items():
        keep = 1 if k in want else 0
        for t in items[keep:]:
            if t.IsLocked():
                continue
            board.Remove(t)
            removed += 1
    for k, src in want.items():
        if k in have:
            continue
        net = board.FindNet(src.GetNetname())
        if src.GetClass() == 'PCB_VIA':
            n = pcbnew.PCB_VIA(board)
            n.SetPosition(src.GetPosition())
            n.SetViaType(src.GetViaType())
            n.SetWidth(src.GetWidth(pcbnew.F_Cu))
            n.SetDrill(src.GetDrillValue())
            top, bot = src.TopLayer(), src.BottomLayer()
            n.SetLayerPair(top, bot)
        else:
            n = pcbnew.PCB_ARC(board) if src.GetClass() == 'PCB_ARC' else pcbnew.PCB_TRACK(board)
            n.SetStart(src.GetStart())
            if src.GetClass() == 'PCB_ARC':
                n.SetMid(src.GetMid())
            n.SetEnd(src.GetEnd())
            n.SetWidth(src.GetWidth())
            n.SetLayer(src.GetLayer())
        if net is not None:
            n.SetNet(net)
        board.Add(n)
        added += 1
    return removed, added


def run_small_via_pass(board, config, log_fn=print):
    """Returns a one-line summary, or None when skipped."""
    orig = board.GetFileName()
    if not orig:
        log_fn("(small-via pass skipped: board has no file name / project)")
        return None
    pro = os.path.splitext(orig)[0] + '.kicad_pro'
    if not os.path.exists(pro):
        log_fn(f"(small-via pass skipped: no project file {pro})")
        return None
    # Small = below Board Setup's minimum drill (never counting > 0.15 mm
    # vias as small when the minimum is left at or below 0.2).
    min_drill = board.GetDesignSettings().m_MinThroughDrill or 0
    limit = max(150000, min_drill - 10000)
    small = sum(1 for t in board.GetTracks()
                if t.GetClass() == 'PCB_VIA' and t.GetDrillValue() <= limit)
    # No quick skip: besides small vias the pass repairs boxed-in plane pads
    # and sweeps dangling copper, which only KiCad's DRC sees reliably.
    work = tempfile.mkdtemp(prefix='krt_sve_')
    try:
        inp = os.path.join(work, 'in.kicad_pcb')
        out = os.path.join(work, 'out.kicad_pcb')
        pcbnew.SaveBoard(inp, board)
        if board.GetFileName() != orig:
            board.SetFileName(orig)
        shutil.copy(pro, os.path.join(work, 'in.kicad_pro'))
        dru = os.path.splitext(orig)[0] + '.kicad_dru'
        if os.path.exists(dru):
            shutil.copy(dru, os.path.join(work, 'in.kicad_dru'))
        cmd = [_python(), os.path.join(_py_router_dir(), 'small_via_eliminate.py'), inp, out,
               '--route-args', ' '.join(shlex.quote(x) for x in _route_args(config))]
        env = dict(os.environ)
        env['PYTHONPATH'] = os.pathsep.join(
            [os.path.dirname(pcbnew.__file__)] + ([env['PYTHONPATH']] if env.get('PYTHONPATH') else []))
        p = subprocess.run(cmd, capture_output=True, text=True, env=env, cwd=work)
        for line in p.stdout.splitlines():
            if line.startswith(('Start:', '[', '   ', 'Done:')):
                log_fn(line)
        import re
        m = re.search(r'SVE_RESULT start=(\d+) end=(\d+)(?: unc_start=(\d+) unc_end=(\d+))?(?: swept=(\d+))?(?: widened=(\d+))?', p.stdout)
        if not m:
            log_fn("(small-via pass failed: " + (p.stderr.strip().splitlines() or ['no output'])[-1] + ")")
            return None
        start, end = int(m.group(1)), int(m.group(2))
        u0 = int(m.group(3)) if m.group(3) else 0
        u1 = int(m.group(4)) if m.group(4) else u0
        swept = int(m.group(5)) if m.group(5) else 0
        widened = int(m.group(6)) if m.group(6) else 0
        if (end < start or u1 < u0 or swept or widened) and os.path.exists(out):
            removed, added = sync_copper(board, out)
            board.SetModified()
            return (f"Small-drill vias: {start} -> {end}; KiCad unconnected: {u0} -> {u1}; "
                    f"dangling stubs removed: {swept}; power necks widened: {widened} (copper synced: -{removed} +{added} item(s))")
        return f"Small-drill vias: {start} -> {end}; KiCad unconnected: {u0} -> {u1} (no change)"
    finally:
        shutil.rmtree(work, ignore_errors=True)
