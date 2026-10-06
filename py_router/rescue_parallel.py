"""Speculative parallel evaluation of the rescue rung ladder.

`_attempt_edge` (net_rescue) walks a ladder of rung configs and keeps the
FIRST rung whose route succeeds. Every rung is an independent search from the
same board state, so the serial walk spends most of its time on rungs that
fail. Here rungs 1.. are tried in forked worker processes while the parent runs
rung 0 itself; the parent then replays the ladder IN ORDER:

  * a rung whose worker failed it AND left no observable state change is
    skipped (its captured stdout is printed so the log reads the same);
  * the first rung a worker reports as a success is run again in the parent,
    so the result, and every side effect of producing it, are exactly the
    serial run's;
  * anything else (a worker error, a failed rung that changed state, a parent
    re-run that disagrees with its worker) falls back to running the rest of
    the ladder serially in the parent, as before.

"Observable state" is a fingerprint of the plugin's module globals (caches
excluded) and of the watched objects (the board and the rescue window), taken
in the worker before and after its rung. A rung that records anything --
fab-escalation ledger, clearance ledger, deferred diagnostics, board_info --
therefore never gets skipped. The outcome is always the serial outcome; only
the wall clock changes.

Forking is only safe in a single-threaded process, so this is active for the
CLI scripts (the AI tab's subprocesses) and never inside KiCad, where the
dialog runs the router on a thread. KICAD_RESCUE_PARALLEL=0 disables it;
KICAD_RESCUE_PARALLEL=N caps the workers per ladder (default: CPU count).
"""

import io
import os
import pickle
import signal
import sys
import threading
import time

_PKG_DIR = os.path.dirname(os.path.abspath(__file__))
_SKIP_NAME_PARTS = ('cache', 'memo', '_rows', '_bytes')
_SCALARS = (int, float, str, bool, type(None), bytes)


def _workers_allowed():
    knob = os.environ.get('KICAD_RESCUE_PARALLEL', '').strip()
    if knob == '0' or not hasattr(os, 'fork'):
        return 0
    if threading.active_count() != 1:
        return 0                      # e.g. inside KiCad: never fork there
    try:
        n = int(knob) if knob else (os.cpu_count() or 1)
    except ValueError:
        n = os.cpu_count() or 1
    return max(0, n)


def _fp(v, depth=0):
    """Cheap structural fingerprint: catches appends, inserts, removals and
    scalar changes without walking huge payloads."""
    if isinstance(v, _SCALARS):
        return v
    if depth >= 3:
        return (type(v).__name__, len(v) if hasattr(v, '__len__') else 0)
    if isinstance(v, dict):
        if len(v) > 256:
            return ('dict', len(v))
        return ('dict', tuple(sorted(((repr(k), _fp(x, depth + 1))
                                      for k, x in v.items()), key=lambda t: t[0])))
    if isinstance(v, (list, tuple)):
        if len(v) > 256:
            return (type(v).__name__, len(v))
        return (type(v).__name__, tuple(_fp(x, depth + 1) for x in v))
    if isinstance(v, (set, frozenset)):
        if len(v) > 256:
            return ('set', len(v))
        return ('set', tuple(sorted(repr(x) for x in v)))
    if hasattr(v, '__dict__') and not isinstance(v, type):
        return _fp_attrs(v, depth + 1)
    if hasattr(v, '__len__'):
        try:
            return (type(v).__name__, len(v))
        except Exception:
            pass
    return (type(v).__name__,)


def _skip(name):
    low = name.lower()
    return any(p in low for p in _SKIP_NAME_PARTS)


def _fp_attrs(obj, depth=0):
    return tuple(sorted((k, _fp(x, depth)) for k, x in vars(obj).items()
                        if not _skip(k)))


def _fingerprint(watch, rung_scoped=()):
    """{entry: fingerprint} over plugin module globals and watched objects.

    rung_scoped: attributes every rung SETS before it reads them (so their
    value going into a rung cannot matter); left out of the comparison."""
    out = {}
    for name, mod in list(sys.modules.items()):
        f = getattr(mod, '__file__', None) or ''
        if not f.startswith(_PKG_DIR) or name == __name__:
            continue
        for k, v in vars(mod).items():
            if k.startswith('__') or _skip(k) or callable(v) \
                    or type(v).__name__ == 'module':
                continue
            if isinstance(v, (dict, list, set, frozenset, tuple) + _SCALARS):
                out[f'{name}.{k}'] = _fp(v)
    for i, o in enumerate(watch):
        for k, v in vars(o).items():
            if not _skip(k) and k not in rung_scoped:
                out[f'watch{i}.{k}'] = _fp(v)
    return out


def _changed(a, b):
    return sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k))


def _spawn(rung_fn, cfg, watch, rung_scoped):
    """Fork a worker for one rung. Returns (pid, read_fd)."""
    r, w = os.pipe()
    pid = os.fork()
    if pid == 0:                                   # worker
        os.close(r)
        try:
            sys.stdout = io.StringIO()
            before = _fingerprint(watch, rung_scoped)
            ok = rung_fn(cfg) is not None
            clean = _fingerprint(watch, rung_scoped) == before
            msg = ('ok', ok, clean, sys.stdout.getvalue())
        except BaseException as e:                 # noqa: BLE001
            msg = ('error', repr(e))
        try:
            data = pickle.dumps(msg)
            with os.fdopen(w, 'wb') as fh:
                fh.write(data)
        finally:
            os._exit(0)
    os.close(w)
    return pid, r


def _collect(pid, fd):
    with os.fdopen(fd, 'rb') as fh:
        data = fh.read()
    os.waitpid(pid, 0)
    try:
        return pickle.loads(data)
    except Exception as e:                         # noqa: BLE001
        return ('error', repr(e))


def _kill(workers):
    for pid, fd in workers:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            os.close(fd)
        except OSError:
            pass
        try:
            os.waitpid(pid, 0)
        except ChildProcessError:
            pass


def run_ladder(rungs, rung_fn, watch=(), rung_scoped=()):
    """Return (result, cfg) of the first rung whose rung_fn(cfg) is not None,
    or (None, None) -- exactly what the serial loop returns."""
    def serial(from_idx):
        for cfg in rungs[from_idx:]:
            result = rung_fn(cfg)
            if result is not None:
                return result, cfg
        return None, None

    n_workers = _workers_allowed()
    if n_workers < 1 or len(rungs) < 2:
        return serial(0)

    sys.stdout.flush()
    sys.stderr.flush()
    _t = time.perf_counter()
    before = _fingerprint(watch, rung_scoped)
    _t = _time('fingerprint', _t)
    workers = {}
    for i in range(1, min(len(rungs), n_workers + 1)):
        workers[i] = _spawn(rung_fn, rungs[i], watch, rung_scoped)
    _t = _time('spawn', _t)
    try:
        result = rung_fn(rungs[0])                 # rung 0 runs for real
        _t = _time('rung0', _t)
        if result is not None:
            _stat('rung0')
            return result, rungs[0]
        _diff = _changed(before, _fingerprint(watch, rung_scoped))
        _t = _time('fingerprint', _t)
        if _diff:
            if os.environ.get('KICAD_RESCUE_PARALLEL_DEBUG') == '2':
                print(f"[rescue_parallel] rung0 changed: {_diff}", file=sys.stderr)
            # Rung 0 failed but changed state: the workers started from the
            # state BEFORE it, so their answers don't apply. Stay exact.
            _stat('fallback_rung0_dirty')
            return serial(1)
        for i in range(1, len(rungs)):
            if i not in workers:                   # beyond the worker cap
                _stat('fallback_cap')
                return serial(i)
            msg = _collect(*workers.pop(i))
            _t = _time('wait', _t)
            if msg[0] != 'ok' or not msg[2]:
                # Worker error, or the rung changed observable state: the
                # remaining workers' premise (an untouched start state) may no
                # longer hold, so finish the ladder exactly as before.
                _stat('fallback_error' if msg[0] != 'ok' else 'fallback_dirty')
                return serial(i)
            _, ok, _, out = msg
            if not ok:
                if out:
                    sys.stdout.write(out)         # same log as a serial run
                _stat('skipped')
                continue
            _kill(list(workers.values()))
            workers.clear()
            _t = time.perf_counter()
            result = rung_fn(rungs[i])             # the winner runs for real
            _t = _time('winner', _t)
            if result is not None:
                _stat('winner')
                return result, rungs[i]
            _stat('fallback_disagree')
            return serial(i + 1)                   # disagreement: stay exact
        _stat('all_failed')
        return None, None
    finally:
        _kill(list(workers.values()))


_STATS = {}


def _time(key, t0):
    t = time.perf_counter()
    _STATS['t_' + key] = round(_STATS.get('t_' + key, 0.0) + (t - t0), 2)
    return t


def _stat(key):
    _STATS[key] = _STATS.get(key, 0) + 1


if os.environ.get('KICAD_RESCUE_PARALLEL_DEBUG'):
    import atexit
    atexit.register(lambda: print(f"[rescue_parallel] {dict(sorted(_STATS.items()))}",
                                  file=sys.stderr))
