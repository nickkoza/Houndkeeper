"""[jlr]: a DimOS night holds the GPU alone; standalone sims share it, and wait for a night that is running or waiting."""
import signal, subprocess, sys, textwrap, time
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def deadline():
    """a lock that never lets go fails its test in 60 s instead of hanging the suite on a read"""
    def late(*_): raise TimeoutError("the GPU lock test ran past 60 s: a hold never let go")
    old = signal.signal(signal.SIGALRM, late); signal.alarm(60)
    yield
    signal.alarm(0); signal.signal(signal.SIGALRM, old)


def _sim(tmp, env_extra=None):
    """a standalone sim: takes the shared hold, prints 'held', keeps it 2 s"""
    code = textwrap.dedent(f"""
        import sys, time; sys.path.insert(0, {str(HK / 'src' / 'houndkeeper')!r})
        import gpu_share as g; from pathlib import Path
        g.LOCK = Path({str(tmp)!r}) / 'gpu.lock'; g.NIGHT_WAITING = g.LOCK.with_suffix('.night-waiting')
        _sl = time.sleep; time.sleep = lambda s: _sl(min(s, 0.05))
        g.share(say=lambda m: print(m, flush=True)); print('held', flush=True); _sl(2)""")
    return subprocess.Popen([sys.executable, "-c", code], stdout=subprocess.PIPE, text=True)


def _night(tmp, hold_s):
    """a DimOS night: sources gpu_lock.sh with HOME pointed at tmp, prints 'night', holds it hold_s"""
    return subprocess.Popen(["bash", "-c", f"source {HK}/sim/dimos/gpu_lock.sh; echo night; sleep {hold_s}"],
                            env={"HOME": str(tmp), "PATH": "/usr/bin:/bin"}, stdout=subprocess.PIPE, text=True)


def test_standalone_sims_share(tmp_path):
    a, b = _sim(tmp_path), _sim(tmp_path)
    assert a.stdout.readline().strip() == "held" and b.stdout.readline().strip() == "held"   # both at once
    a.wait(); b.wait()


def test_sim_waits_for_a_running_night(tmp_path):
    lock_dir = tmp_path / ".cache" / "houndkeeper"
    n = _night(tmp_path, 1.5); assert n.stdout.readline().strip() == "night"
    s = _sim(lock_dir); t0 = time.time()
    assert s.stdout.readline().startswith("GPU: waiting")
    assert s.stdout.readline().strip() == "held" and time.time() - t0 > 1.0   # only after the night let go
    n.wait(); s.wait()


def test_night_waits_for_sims_and_blocks_new_ones(tmp_path):
    lock_dir = tmp_path / ".cache" / "houndkeeper"
    a = _sim(lock_dir); assert a.stdout.readline().strip() == "held"
    n = _night(tmp_path, 1.0); assert n.stdout.readline().startswith("GPU: waiting")   # the night waits for the running sim
    assert (lock_dir / "gpu.night-waiting").exists()                        # announced while it waits
    b = _sim(lock_dir); assert b.stdout.readline().startswith("GPU: waiting")   # a new sim holds off for it
    assert n.stdout.readline().strip() == "night"                            # the night runs once the first sim ends
    assert b.stdout.readline().strip() == "held"                             # and the new sim after the night
    for p in (a, n, b): p.wait()


def test_a_night_that_died_waiting_does_not_hold_sims_off(tmp_path):
    """a night killed while it waited (Ctrl-C, a stopped task) leaves its announcement behind; the sims must not wait
    on it forever: the announcement names the waiting night's process, and a gone process's is cleared"""
    lock_dir = tmp_path / ".cache" / "houndkeeper"; lock_dir.mkdir(parents=True)
    gone = subprocess.Popen(["true"]); gone.wait()                           # a process id no longer running
    (lock_dir / "gpu.night-waiting").write_text(f"{gone.pid}\n")
    s = _sim(lock_dir); t0 = time.time()
    try:
        assert s.stdout.readline().strip() == "held" and time.time() - t0 < 5.0
        assert not (lock_dir / "gpu.night-waiting").exists()
    finally:
        s.kill(); s.wait()                                                   # a waiting sim would otherwise wait on


def test_the_waiting_night_names_its_process(tmp_path):
    lock_dir = tmp_path / ".cache" / "houndkeeper"
    a = _sim(lock_dir); assert a.stdout.readline().strip() == "held"
    n = _night(tmp_path, 0.2); assert n.stdout.readline().startswith("GPU: waiting")
    assert (lock_dir / "gpu.night-waiting").read_text().strip() == str(n.pid)
    a.wait(); n.wait()
