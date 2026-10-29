"""[WFI]'s stitch solves the camera's mount: a small turn of the camera moves the floor it paints, far points much more
than near ones and a pitch along the line of sight, which is what makes the mount observable from overlapping views."""
import math
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "src")); sys.path.insert(0, str(HK / "sim" / "house"))
import stitch_floor as sf  # noqa: E402


def _camera(h=0.30, down_deg=10.0):
    """a camera h m up at the origin looking along +x, tipped down_deg below level (world <- optical: x right, y down,
    z forward)"""
    a = math.radians(down_deg)
    z = np.array([math.cos(a), 0.0, -math.sin(a)]); x = np.array([0.0, -1.0, 0.0]); y = np.cross(z, x)
    T = np.eye(4); T[:3, :3] = np.stack([x, y, z], 1); T[:3, 3] = [0.0, 0.0, h]; return T


def test_far_points_move_more_and_pitch_moves_along_the_view():
    T = _camera()
    near, far = sf.floor_jac(T, np.array([1.0, 0.0])), sf.floor_jac(T, np.array([2.0, 0.0]))
    assert near is not None and far is not None
    assert abs(far[0, 0]) > 3 * abs(near[0, 0])                 # pitch: ~range^2 / height
    assert abs(far[0, 0]) > 5 * abs(far[1, 0])                   # along the line of sight, not across it
    assert abs(far[0, 0] * math.radians(0.6)) > 0.05              # 0.6 deg of pitch: over 5 cm at 2 m
