"""[gJq]: the camera's frame lag and the head mount are the robot's calibration, written only by the calibration steps
over tuning houses, read by every night; a night's own estimate only checks them."""
import re
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src")); sys.path.insert(0, str(ROOT / "sim" / "house"))
from houndkeeper import calibrate as cal  # noqa: E402

GOOD = dict(lag_s=0.14, corr=0.85, pairs=900, turning=400, edge=False)


def test_a_night_with_no_turning_leaves_the_robots_lag_unchanged():
    t = np.arange(0, 60, 0.07); yaw = np.zeros_like(t)                       # walking straight: nothing to correlate
    rng = np.random.default_rng(1); wall = rng.random((120, 1400)) * 255
    frames = [(tk, np.vstack([wall[:, 300:940], np.zeros((120, 640))])) for tk in t]
    d = cal.frame_lag_detail(frames, t, yaw)
    assert cal.lag_refusal(d) is not None
    lag, why = cal.robot_lag([GOOD, d, dict(GOOD, lag_s=0.16)])
    assert lag == pytest.approx(0.15) and why[1][1] is not None


def test_a_sign_flipped_edge_or_thin_estimate_leaves_the_robots_lag_unchanged():
    bad = [dict(GOOD, lag_s=-0.25, corr=0.89),                                # house 93's sign flip, high correlation
           dict(GOOD, lag_s=0.3, edge=True), dict(GOOD, turning=40), dict(GOOD, corr=0.5), dict(GOOD, lag_s=None)]
    for b in bad: assert cal.lag_refusal(b) is not None
    assert cal.robot_lag([GOOD] + bad)[0] == pytest.approx(0.14)
    assert cal.robot_lag(bad)[0] is None


def test_the_nights_estimate_only_checks_the_stored_lag():
    stored = {"lag_s": 0.15, "clock": "robot"}
    got = cal.camera_lags("robot", ["color_image"], stored, lambda s: dict(GOOD, lag_s=0.25), clock="robot")
    assert got["color_image"][0] == 0.15 and "differs" in got["color_image"][1]
    assert cal.check_lag(0.15, dict(GOOD, lag_s=0.17)) is None
    assert cal.camera_lags("night", ["color_image"], stored, lambda s: dict(GOOD, lag_s=0.25))["color_image"][0] == 0.25


def test_the_lag_file_names_its_clock(tmp_path):
    p = tmp_path / "frame_lag.json"; p.write_text('{"lag_s": 0.1}')
    with pytest.raises(ValueError): cal.load_lag(p, "wall")


def test_the_calibration_steps_refuse_hold_out_seeds(tmp_path):
    import calibrate_lag, calibrate_robot_mount
    called = []
    for seeds in ([70, 80], [81], [72, 100], [119]):
        with pytest.raises(ValueError, match="hold-out"):
            calibrate_lag.calibrate(seeds, lambda s: called.append(s) or [], "wall")
        with pytest.raises(ValueError, match="hold-out"):
            calibrate_robot_mount.calibrate(seeds, tmp_path)
    assert called == []                                                       # refused before any recording was read
    good = lambda s: [("n1", {"color_image": dict(GOOD, lag_s=0.12)}), ("n2", {"color_image": dict(GOOD, lag_s=-0.25)})]
    assert calibrate_lag.calibrate([70, 71], good, "robot")["lag_s"] == pytest.approx(0.12)


def test_a_picture_saves_the_head_cameras_lag_not_the_wrists():
    fm = pytest.importorskip("floor_mosaic2")                                 # dimos venv
    f = fm.picture_lag_fields([{"stream": "color_image", "dt": -0.15}, {"stream": "arm_color_image", "dt": -0.05}], None, None)
    assert float(f["lag_s"]) == pytest.approx(0.15) and float(f["wrist_lag_s"]) == pytest.approx(0.05)
    f = fm.picture_lag_fields([{"stream": "arm_color_image", "dt": -0.05}], None, None)
    assert "lag_s" not in f


def test_stitch_baselines_never_writes_the_robots_mount():
    sh = (ROOT / "sim" / "dimos" / "stitch_baselines.sh").read_text()
    code = "\n".join(l.split("#", 1)[0] if not l.lstrip().startswith("#") else "" for l in sh.splitlines())
    for var in ("ROBOT_MOUNT", "ROBOT_LAG"):
        assert not re.search(r'(cp|mv|install|tee)\b[^\n;|]*"\$' + var + r'"\s*(;|$|\n|&&|\|)', code, re.M), var
        assert not re.search(r'>\s*"?\$' + var, code), var
        assert not re.search(r'--(out|mount-out)\s+"?\$' + var, code), var
    stamp = re.search(r'^STAMP=.*$', code, re.M).group(0)
    assert "$(sha \"$ROBOT_MOUNT\")" in stamp and "$(sha \"$ROBOT_LAG\")" in stamp
    assert "--dt robot" in code and "calibration.json" in code


def test_floor_mosaic2_reads_the_stored_lag_by_default():
    src = (ROOT / "sim" / "house" / "floor_mosaic2.py").read_text()
    assert re.search(r'"--dt", default="robot"', src)
    assert "lag_s=np.float32(-dt)" not in src


def test_a_replay_places_each_camera_with_its_own_stored_lag(tmp_path, monkeypatch):
    """[gJq]: the confirmer's replay and the mount calibration read the robot's stored lag, each camera its own (the
    wrist's, not the head's); 'auto' is the robot's too, 'night' tonight's estimate, a number that lag"""
    fm = pytest.importorskip("floor_mosaic2")                                 # dimos venv
    p = tmp_path / "frame_lag.json"
    p.write_text('{"lag_s": 0.135, "clock": "wall", "streams": {"color_image": {"lag_s": 0.135}, "arm_color_image": {"lag_s": 0.155}}}')
    both = ["color_image", "arm_color_image"]
    for dt in ("robot", "auto"):
        got = fm.frame_lags(None, dt, both, p, check=False, clock="wall")
        assert got["color_image"][0] == pytest.approx(0.135) and got["arm_color_image"][0] == pytest.approx(0.155)
    assert fm.frame_lags(None, "0.08", both, p)["arm_color_image"] == (0.08, "given")
    est = lambda s: dict(GOOD, lag_s=0.2 if s == "color_image" else 0.1)
    monkeypatch.setattr(fm, "lag_detail_from_turns", lambda st, s: est(s))
    got = fm.frame_lags(None, "night", both, p)
    assert got["color_image"][0] == 0.2 and got["arm_color_image"][0] == 0.1
    got = fm.frame_lags(None, "robot", both, p, clock="wall")                 # tonight's estimate only checks it
    assert got["color_image"][0] == pytest.approx(0.135) and "differs" in got["color_image"][1]


def test_the_replay_and_the_mount_default_to_the_stored_lag():
    """[gJq]: confirm_stream's replay and calibrate_mount.py read the robot's stored lag unless told otherwise, and the
    replay looks each frame's pose up with its own camera's lag"""
    cs = pytest.importorskip("confirm_stream")                                # dimos venv
    import inspect, calibrate_mount as cm
    a = cs.parser().parse_args(["--run-dir", "r", "--seed", "70", "--night", "3", "--db", "m.db"])
    assert a.dt == "robot" and a.lag_file.endswith("frame_lag.json") and not hasattr(a, "truth")
    assert inspect.signature(cm.frames_from).parameters["dt"].default == "robot"
    src = inspect.getsource(cs.replay)
    assert "lag = lags[stream][0]" in src and "lag_from_turns" not in src
    assert "lag_from_turns" not in inspect.getsource(cm.frames_from)


def test_a_number_is_the_lag_itself_everywhere(tmp_path):
    """[gJq]: --dt's number is the lag, each frame placed that much earlier, the same in calibrate.camera_lags as in
    floor_mosaic2.frame_lags (which placed it that way while camera_lags negated it), and its help says so"""
    fm = pytest.importorskip("floor_mosaic2")                                 # dimos venv
    assert cal.camera_lags("0.08", ["color_image"], None, None)["color_image"] == (0.08, "given")
    assert fm.frame_lags(None, "0.08", ["color_image"])["color_image"] == cal.camera_lags("0.08", ["color_image"], None, None)["color_image"]
    import inspect
    src = inspect.getsource(fm.main); h = src[src.index('"--dt"'):]; h = h[:h.index("\n")]
    assert "much earlier" in h and "later (s; negative: earlier)" not in h
