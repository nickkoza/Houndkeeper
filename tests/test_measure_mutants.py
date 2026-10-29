"""Tests written to kill mutants that survived the suite (docs/research/test-quality-2026-10-03.md): the measurements read off
a recording ([4hm]) where the earlier tests' inputs hid a bug: odometry whose frame starts turned and offset from the
truth's, a truth heading that wraps, a robot that stands still before it walks, an out-and-back walk, a heavy drop
rate; and writing a measurement into one field of a dict value or a key only another group has."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "real" / "calibration"))


def _truth(t, x, y, yaw): return {"wall_t": t, "true_x": x, "true_y": y, "true_yaw": yaw}


def test_odometry_in_its_own_frame_is_laid_on_the_truth_at_the_start():
    """[4hm] the odometry starts at (0, 0) heading 0 while the truth starts at (3, -1) heading 90 deg: perfect
    odometry has no end drift, whichever way its frame is turned"""
    import measure
    s = np.linspace(0, 12, 1201); t = s / 0.4; th = 0.3 * np.sin(s / 2)                    # a gently weaving walk
    ds = np.r_[0, np.diff(s)]; ox = np.cumsum(ds * np.cos(th)); oy = np.cumsum(ds * np.sin(th))
    R = np.array([[0.0, -1.0], [1.0, 0.0]]); tx, ty = (R @ np.c_[ox, oy].T) + np.array([[3.0], [-1.0]])
    r = measure.odom_vs_truth(t, ox, oy, th, _truth(t, tx, ty, th + math.pi / 2))
    assert r["end_m"] < 0.01 and abs(r["scale"]) < 0.002 and abs(r["end_yaw_deg"]) < 0.1


def test_a_truth_heading_that_wraps_past_180_is_not_a_turn():
    """[4hm] the sim's true heading is reported in (-180, 180]; walking a circle crosses it"""
    import measure
    s = np.linspace(0, 20, 2001); t = s / 0.4; th = math.pi - 0.5 + s / 2.0
    x, y = 2 * np.sin(th), -2 * np.cos(th)
    wrapped = np.angle(np.exp(1j * th))
    r = measure.odom_vs_truth(t, x, y, th, _truth(t, x, y, wrapped))
    assert abs(r["end_yaw_deg"]) < 0.1 and r["yaw_deg_per_root_m"] < 0.05


def test_the_end_drift_is_at_the_end_not_the_worst_on_the_way():
    """[4hm] out 5 m and back on odometry 2% long: 10 cm off at the far end, home again at the end"""
    import measure
    s = np.linspace(0, 10, 1001); t = s / 0.4; x = np.where(s < 5, s, 10 - s); y = np.zeros_like(s)
    yaw = np.where(s < 5, 0.0, math.pi)
    r = measure.odom_vs_truth(t, x * 1.02, y, yaw, _truth(t, x, y, yaw))
    assert r["end_m"] < 0.01 and abs(r["scale"] - 0.02) < 0.002


def test_position_drift_is_per_root_metre():
    """[4hm] odometry drifting sideways 1 cm x sqrt(metres walked): 0.01 per root metre, at any length"""
    import measure
    s = np.linspace(0, 16, 1601); t = s / 0.4; y = np.zeros_like(s); yaw = np.zeros_like(s)
    r = measure.odom_vs_truth(t, s, y + 0.01 * np.sqrt(s), yaw, _truth(t, s, y, yaw))
    assert abs(r["xy_per_root_m"] - 0.01) < 0.001


def test_standing_still_jitter_is_not_distance_walked():
    """[4hm] a minute standing (odometry jittering by a millimetre) then 10 m: the scale is the walk's alone"""
    import measure
    rng = np.random.default_rng(1); still = np.zeros(1200); walk = np.linspace(0, 10, 1001)[1:]
    s = np.r_[still, walk]; t = np.r_[np.arange(1200) * 0.05, 60 + walk / 0.4]
    ox = s + np.r_[rng.normal(0, 0.001, 1200), np.zeros(1000)]
    r = measure.odom_vs_truth(t, ox, np.zeros_like(s), np.zeros_like(s), _truth(t, s, np.zeros_like(s), np.zeros_like(s)))
    assert abs(r["scale"]) < 0.005 and abs(r["walked_m"] - 10) < 0.05


def test_frame_gaps_share_is_of_the_frames_that_should_have_come():
    """[4hm] a fifth of the frames lost singly is a drop of 0.2 (of those expected), not 0.25 (of those received)"""
    import measure
    rng = np.random.default_rng(3); t = np.arange(0, 600, 1 / 15); keep = rng.random(len(t)) > 0.2
    keep[0] = keep[-1] = True
    assert abs(measure.frame_gaps(t[keep])["drop"] - 0.2) < 0.01


ROBOT = ("go2_air:\n  lidar:             {value: {points_per_s: 21600, range_sd: 0.02}, source: estimate}\n"
         "  mass_kg:           {value: 15,    source: vendor}\nkit:\n  budget_kg:         {value: 1.8, source: estimate}\n")


def test_writing_one_field_of_a_dict_keeps_the_others(tmp_path):
    """[4hm] measure.py --write sets go2_air.lidar.range_sd: the lidar's points_per_s stays"""
    import measure, yaml
    p = tmp_path / "robot.yaml"; p.write_text(ROBOT)
    measure.set_param("go2_air.lidar.range_sd", 0.031, "wall", params=p)
    d = yaml.safe_load(p.read_text())["go2_air"]["lidar"]
    assert d["value"] == {"points_per_s": 21600, "range_sd": 0.031} and d["source"] == "measured"


def test_a_key_only_another_group_has_is_not_written_there(tmp_path):
    """[4hm] go2_air has no budget_kg: the kit's must not be overwritten in its place"""
    import measure
    p = tmp_path / "robot.yaml"; p.write_text(ROBOT)
    with pytest.raises(KeyError): measure.set_param("go2_air.budget_kg", 2.5, "scale", params=p)
    assert p.read_text() == ROBOT
