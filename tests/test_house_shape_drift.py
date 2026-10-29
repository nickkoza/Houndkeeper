"""House shape's drift gate judges a lidar match by what it does at the keyframe, wherever the keyframe stands in the
house ([FHA], [KkQ]): a turn about the keyframe is a turn, not a shift that grows with the distance from the origin."""
import math
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def _turn_about(o, deg):
    th = math.radians(deg); c, s = math.cos(th), math.sin(th); R = np.array([[c, -s], [s, c]])
    T = np.eye(3); T[:2, :2] = R; T[:2, 2] = np.asarray(o) - R @ np.asarray(o); return T


def test_a_one_degree_turn_is_judged_alike_at_a_corner_and_near_the_centre():
    """[FHA], [KkQ]: 0.5 m walked between the pair (gate 0.125 m, 2.75 deg); about the origin the corner's turn read as
    a 0.141 m shift and was dropped"""
    import house_shape as hs
    for at in ((7.0, 4.0), (0.1, 0.1), (-7.5, -4.2)):
        assert hs.drift_ok(_turn_about(at, 1.0), np.array(at), 0.5), at


def test_a_shift_beyond_the_gate_is_dropped_wherever_it_is():
    """[FHA], [KkQ]"""
    import house_shape as hs
    for at in ((7.0, 4.0), (0.1, 0.1)):
        T = np.eye(3); T[0, 2] = 0.20
        assert not hs.drift_ok(T, np.array(at), 0.5)
        assert not hs.drift_ok(_turn_about(at, 4.0), np.array(at), 0.5)   # and a turn beyond it


def test_the_shift_is_measured_at_the_keyframe():
    """[FHA], [KkQ]: a turn about the origin moves a keyframe at (7, 4) by 2 sin(th/2) |o|"""
    import house_shape as hs
    T = _turn_about((0.0, 0.0), 1.0); at = np.array([7.0, 4.0])
    assert abs(hs.shift_at(T, at) - 2 * math.sin(math.radians(0.5)) * math.hypot(7, 4)) < 1e-9
    assert not hs.drift_ok(T, at, 0.5)
