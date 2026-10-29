"""The stereo pair's ELP 720p boards have their own camera model (waymark 0011 [A8K]): a wide lens calibrated away
leaves the edges softer than the middle and a residual that differs between the two cameras."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


def test_a_calibrated_wide_lens_is_soft_at_the_edges_not_the_middle():
    from houndkeeper import camera_imperfect as ci
    img = (np.random.default_rng(0).random((720, 1280)) * 255).astype(np.float32)
    f = 360 / np.tan(np.radians(54.4) / 2)
    out = ci.lens_roundtrip(img, f, -0.25, 0.05, 0.0)
    mid = np.s_[330:390, 610:670]; edge = np.s_[330:390, 5:65]
    assert np.abs(out[mid] - img[mid]).mean() < 0.1                             # the middle untouched (float rounding in the remap)
    assert out[edge].std() < 0.8 * img[edge].std()                                # the edge smoothed by the stretch


def test_the_two_cameras_residuals_differ_and_the_profile_is_the_boards():
    from houndkeeper import camera_imperfect as ci, params as P
    img = np.zeros((720, 1280), np.float32); img[:, 1200:1210] = 255             # a stripe near the edge
    f = 360 / np.tan(np.radians(54.4) / 2)
    a = ci.lens_roundtrip(img, f, -0.25, 0.05, 0.01); b = ci.lens_roundtrip(img, f, -0.25, 0.05, -0.01)
    ca, cb = (float((x.sum(0) * np.arange(1280)).sum() / x.sum()) for x in (a, b))
    assert abs(ca - cb) > 0.3                                                     # left and right land apart: a stereo error
    s = P.get("maker_arm.elp_camera"); r2 = 0.9   # barrel at the frame's edge (the Arducam lens's fit, 29 Sep)
    assert s["dip_per_min"] == 0 and s["lens_k1"] * r2 + s["lens_k2"] * r2 * r2 < 0
