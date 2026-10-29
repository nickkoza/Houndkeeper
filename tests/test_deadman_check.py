"""[Mj2]: the deadman check (real/tests/deadman.py) reads the stop time off odometry: a robot turning 40 deg/s that
stops 0.4 s after the silence passes at about 0.4 s, one that stops at 0.9 s fails, one that never drove fails."""
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "real" / "tests"))


def odom(stop_after, silence=10.0, rate_deg=40.0, seed=0):
    rng = np.random.default_rng(seed)
    t = np.arange(5.0, 15.0, 1 / 15) + rng.normal(0, 0.003, 150)
    moving_until = silence + stop_after
    yaw = np.radians(rate_deg) * (np.minimum(t, moving_until) - 5.0) + rng.normal(0, 0.0005, len(t))
    yaw = (yaw + np.pi) % (2 * np.pi) - np.pi                       # wrapped, as odometry reports it
    xy = rng.normal(0, 0.0005, (len(t), 2))
    return np.c_[t, xy, yaw]


def check(o, silence=10.0):
    import deadman
    o = o.copy(); o[:, 3] = np.unwrap(o[:, 3])
    return deadman.analyse(o, silence)


def test_stops_in_time():
    r = check(odom(0.4))
    assert r["pass"] and 0.3 <= r["stop_s"] <= 0.5 and 35 < r["drive_deg_s"] < 45


def test_stops_late_fails():
    r = check(odom(0.9))
    assert not r["pass"] and r["stop_s"] > 0.8


def test_not_driving_fails():
    r = check(odom(0.0, rate_deg=2.0))
    assert not r["pass"] and "not driving" in r["why"]


def test_check_reads_files(tmp_path):
    import json
    import deadman
    np.savetxt(tmp_path / "odom.csv", odom(0.2), delimiter=",", fmt="%.4f")
    (tmp_path / "ev.jsonl").write_text(json.dumps({"case": "killed", "silence": 10.0}) + "\n"
                                       + json.dumps({"case": "hidden", "error": "teleop never armed"}) + "\n")
    r = deadman.check(tmp_path / "odom.csv", tmp_path / "ev.jsonl")
    assert r[0]["pass"] and not r[1]["pass"]


def test_settling_back_after_the_stop_is_not_driving():
    import deadman
    o = odom(0.3)
    after = o[:, 0] > 10.3                                           # halted, then the legs settle 6 deg/s back
    o[after, 3] -= np.radians(6.0) * np.clip(o[after, 0] - 10.3, 0, 0.6)
    o[:, 3] = np.unwrap(o[:, 3])
    r = deadman.analyse(o, 10.0)
    assert r["pass"] and r["stop_s"] <= 0.45 and r["still_s"] > 0.8
