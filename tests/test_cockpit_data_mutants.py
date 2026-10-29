"""Tests written to kill mutants that survived the suite (docs/research/test-quality-2026-10-03.md): the phone cockpit's
procedures ([vRJ]) where the earlier tests' walks hid a bug: a walk that starts facing another way than odometry's
zero, a heading that ends past 180 deg, a robot that wanders back near the start without a lap, a second start while
recording, and the stand power written on confirm."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "src")); sys.path.insert(0, str(HK / "real" / "calibration"))
pytest.importorskip("dimos")                                                   # no DimOS here: skip; a broken module: fail
import houndkeeper.cockpit_data as cd  # noqa: E402


def harness(tmp_path, monkeypatch):
    import measure
    params = tmp_path / "robot.yaml"
    params.write_text("go2_air:\n  walk_power_W:      {value: 170,   source: estimate, note: \"est.\"}\n"
                      "  mass_kg:           {value: 15,    source: vendor}\n")
    (tmp_path / "params").mkdir(); (tmp_path / "params" / "robot.yaml").symlink_to(params)
    monkeypatch.setattr(measure, "PARAMS", params); monkeypatch.setattr(cd, "HK", tmp_path)
    monkeypatch.setattr(cd, "RUNS_LOG", tmp_path / "runs.jsonl")
    m = object.__new__(cd.CockpitData)
    m._buf = cd.Buffers(); m._state = cd.ProcedureState(); m._t0 = 0.0; m._truth = {}; m._pending = {}; m._frame_n = 0
    return m, params


def _square(start_yaw, laps=5, side=2.5, drift_fwd=0.0):
    """laps of a square in the robot's own start frame, then turned by start_yaw into odometry's frame"""
    s = np.linspace(0, 4 * side * laps, 2001); k = (s % (4 * side)) // side; a = (s % (4 * side)) % side
    x = np.select([k == 0, k == 1, k == 2, k == 3], [a, np.full_like(s, side), side - a, np.zeros_like(s)]) + drift_fwd * s / s[-1]
    y = np.select([k == 0, k == 1, k == 2, k == 3], [np.zeros_like(s), a, np.full_like(s, side), side - a])
    c, sn = math.cos(start_yaw), math.sin(start_yaw)
    return s, c * x - sn * y + 1.0, sn * x + c * y - 2.0, k * math.pi / 2 + start_yaw


def test_the_taped_square_measures_the_end_in_the_robots_own_start_frame(tmp_path, monkeypatch):
    """[vRJ] odometry's zero heading is not the taped line's: the end 6 cm ahead of the cross reads 6 cm forward, so the
    tape's forward/left agree with it"""
    m, _ = harness(tmp_path, monkeypatch)
    m.handle({"op": "start", "id": "taped-square"}, now=0.0)
    s, x, y, yaw = _square(math.radians(130), drift_fwd=0.06)
    m._buf.odom.extend(zip((s * 2.5).tolist(), x.tolist(), y.tolist(), yaw.tolist()))
    m.handle({"op": "stop"}, now=125.0)
    m.handle({"op": "truth", "forward_cm": 0, "left_cm": 0, "heading_deg": 0}, now=126.0)
    assert abs(m._state.result["end_error_m"] - 0.06) < 0.005
    m.handle({"op": "truth", "forward_cm": 6, "left_cm": 0, "heading_deg": 0}, now=127.0)
    assert m._state.result["end_error_m"] < 0.005


def test_a_heading_that_ends_past_180_degrees_is_a_small_error(tmp_path, monkeypatch):
    """[vRJ] five laps end where they began, give or take a degree: not 1800 deg of heading error"""
    m, _ = harness(tmp_path, monkeypatch)
    m.handle({"op": "start", "id": "taped-square"}, now=0.0)
    s, x, y, yaw = _square(0.0)
    yaw = np.unwrap(yaw) + np.linspace(0, math.radians(1.0), len(yaw)); yaw[-1] = 5 * 2 * math.pi + math.radians(1.0)
    m._buf.odom.extend(zip((s * 2.5).tolist(), x.tolist(), y.tolist(), yaw.tolist()))
    m.handle({"op": "stop"}, now=125.0)
    m.handle({"op": "truth", "forward_cm": 0, "left_cm": 0, "heading_deg": 0}, now=126.0)
    assert abs(m._state.result["heading_error_deg"] - 1.0) < 0.1


def test_wandering_near_the_start_is_not_a_lap(tmp_path, monkeypatch):
    """[vRJ] the live lap count counts returns to the cross after going a metre away, not every brush past it"""
    m, _ = harness(tmp_path, monkeypatch)
    m.handle({"op": "start", "id": "taped-square"}, now=0.0)
    t = np.arange(0, 60, 0.1); x = 0.4 * np.sin(t)                                    # to and fro across the cross
    m._buf.odom.extend(zip(t.tolist(), x.tolist(), np.zeros_like(t).tolist(), np.zeros_like(t).tolist()))
    assert cd.live_numbers("taped-square", m._buf.arrays(0.0), 0.0, 60.0)["laps"] == 0


def test_a_second_start_while_recording_keeps_the_run(tmp_path, monkeypatch):
    """[vRJ] a second viewer pressing start mid-procedure does not throw away what is being recorded"""
    m, _ = harness(tmp_path, monkeypatch)
    m.handle({"op": "start", "id": "stream"}, now=1000.0); run = m._state.run
    m.handle({"op": "start", "id": "power"}, now=1100.0)
    assert m._state.active == "stream" and m._state.run == run and m._t0 == 1000.0


def test_confirming_a_power_run_writes_the_stand_power_beside_the_walk_power(tmp_path, monkeypatch):
    """[vRJ] the file has no stand_power_W line yet: the write adds it after walk_power_W"""
    import yaml
    m, params = harness(tmp_path, monkeypatch)
    m.handle({"op": "start", "id": "power"}, now=0.0)
    bt = np.arange(0.0, 1500.0, 1.0); P = np.where(bt < 300, 60.0, 170.0)
    soc = 100 - np.r_[0, np.cumsum(P[1:])] / 3600 / 236.8 * 100
    m._buf.battery.extend(zip(bt.tolist(), np.floor(soc).tolist(), np.full(len(bt), 29.0).tolist(), P.tolist()))
    ot = np.arange(0.0, 1500.0, 0.1); s = np.clip(ot - 300, 0, None) * 0.4
    m._buf.odom.extend(zip(ot.tolist(), (2 * np.sin(s / 2)).tolist(), (2 - 2 * np.cos(s / 2)).tolist(), (s / 2).tolist()))
    m.handle({"op": "stop"}, now=1500.0)
    m.handle({"op": "write", "run": m._state.run}, now=1501.0)
    d = yaml.safe_load(params.read_text())["go2_air"]; lines = params.read_text().splitlines()
    assert abs(d["stand_power_W"]["value"] - 60) < 3 and d["stand_power_W"]["source"] == "measured"
    assert lines[2].startswith("  stand_power_W:") and "written" in m._state.message


def test_true_and_false_are_not_numbers_to_write():
    assert not cd._finite(True) and not cd._finite({"drop": False}) and cd._finite({"drop": 0.1})
