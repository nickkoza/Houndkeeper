"""[dmT], decision [fxk]: a loop closure enters the house's frame only when the robot's own scans agree with it, on by
default in the pipeline (the stitch, house_frame.py, the pose-graph bench), DimOS on one thread so a frame repeats; a
house's map made by an older stitch is refused by the live tidy and the confirmer, naming why."""
import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]; SIM = HK / "sim" / "house"
sys.path.insert(0, str(HK / "src")); sys.path.insert(0, str(SIM))
STITCH = (HK / "sim/dimos/stitch_baselines.sh").read_text()

from houndkeeper import calibrate as cal


def _stitch_line():
    """the stitch's line that runs house_frame.py"""
    return next(l for l in STITCH.splitlines() if "$PY house_frame.py" in l)


def test_the_stitch_verifies_loops_on_one_thread():
    """[dmT]: the stitch's DimOS pose graph runs with the verifier and one thread; the frame's verdicts are a required output"""
    line = _stitch_line()
    assert "--verify lidar" in line and "OMP_NUM_THREADS=1" in line
    i = STITCH.index("$PY house_frame.py")
    assert '"$RUN/frame/verify.json"' in STITCH[STITCH.rindex("build --clear", 0, i):i]
    assert "--verify none" not in STITCH


def test_house_frame_verifies_by_default_and_hands_dimos_the_verifier():
    """[dmT]: house_frame.py's default is the lidar verifier, handed to DimOS's PGO; 'none' is DimOS as it ships"""
    src = (SIM / "house_frame.py").read_text()
    assert re.search(r'add_argument\("--verify", default="lidar"', src)
    assert "PGO(loop_verifier=ver" in src
    import house_frame as hf
    assert hf.verifier(["unused.db"], "none", None) is None


def test_the_bench_verifies_and_runs_one_thread_by_default():
    """[fxk]: joint_bench.py verifies DimOS's loops unless told not to, and sets one thread before Open3D loads"""
    env = {k: v for k, v in os.environ.items() if k != "OMP_NUM_THREADS"}
    out = subprocess.run([sys.executable, "-c", "import os, joint_bench as jb; print(os.environ.get('OMP_NUM_THREADS'), jb.VERIFY)"],
                         cwd=SIM, env=env, capture_output=True, text=True, check=True).stdout.split()
    assert out == ["1", "either"]


def test_the_stitch_stamp_carries_the_map_version_the_readers_expect():
    """[dmT]: the stitch's stamp starts with its MAP_VERSION, the same one houndkeeper.calibrate refuses older maps by"""
    m = re.search(r"^MAP_VERSION=(\S+)", STITCH, re.M)
    assert m and m.group(1) == cal.MAP_VERSION
    stamp = next(l for l in STITCH.splitlines() if l.startswith("STAMP="))
    assert stamp.startswith('STAMP="$MAP_VERSION ') and "loops=verified" in stamp


def _run(tmp_path, stamp=None, shape=True):
    run = tmp_path / "nights_s70"; run.mkdir(parents=True)
    if shape: (run / "shape").mkdir()
    if stamp is not None: (run / "stitch.args").write_text(stamp + "\n")
    return run


def test_an_older_map_is_named_stale(tmp_path):
    """[dmT]: a v8 stitch (unverified loops) is stale, the current one is not, and a stitched house without a stamp is
    stale too (night_picture.sh removes it to force a re-stitch)"""
    old = cal.stale_map(_run(tmp_path / "a", "v8 head d1 d2 mount=x"))
    assert old and "v8" in old[0] and "verified" in old[0]
    assert cal.stale_map(_run(tmp_path / "b", f"{cal.MAP_VERSION} head d1 d2 loops=verified")) == []
    assert cal.stale_map(_run(tmp_path / "c", None)) != []


def test_a_house_with_no_map_yet_is_no_reason(tmp_path):
    """a house before its second baseline night has no stitched map; its first night's pictures are still usable"""
    assert cal.stale_map(_run(tmp_path, None, shape=False)) == []


def _confirmer_args(**kw):
    a = argparse.Namespace(relief="none", dt="robot", mount=None, lag_file=None, views=[], near_views=None,
                           mixed_calibration=False, stale_map=False); vars(a).update(kw); return a


def test_the_confirmer_refuses_an_older_map_unless_asked(tmp_path):
    """[dmT]: the confirmer stops on a map made before its loops were verified, naming why; --stale-map compares anyway"""
    import confirm_stream as cs
    run = _run(tmp_path, "v8 head d1 d2")
    with pytest.raises(SystemExit, match="loop closures"):
        cs.calibration_used(_confirmer_args(), run, [1, 2])
    cs.calibration_used(_confirmer_args(stale_map=True), run, [1, 2])
    (run / "stitch.args").write_text(f"{cal.MAP_VERSION} head d1 d2\n")
    cs.calibration_used(_confirmer_args(), run, [1, 2])


def test_the_live_tidy_refuses_an_older_map(tmp_path):
    """[dmT]: the live tidy stops before comparing tonight with a map made before its loops were verified"""
    from houndkeeper import tidy_module as tm
    run = _run(tmp_path, "v8 head d1 d2")
    with pytest.raises(RuntimeError, match="made again"):
        tm.baseline_calibration(run, [1, 2], say=lambda m: None, lag_file=tmp_path / "no_lag.json")
    (run / "stitch.args").write_text(f"{cal.MAP_VERSION} head d1 d2\n")
    tm.baseline_calibration(run, [1, 2], say=lambda m: None, lag_file=tmp_path / "no_lag.json")
