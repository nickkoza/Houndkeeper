"""Tests written to kill mutants that survived the suite (docs/research/test-quality-2026-10-03.md): the camera's frame lag
from turning ([uKM] 2) on a real robot's uneven clocks, and the head camera's mount found from its own frames ([KkQ]),
which had no test."""
import math
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage as ndi

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from houndkeeper import calibrate as cal  # noqa: E402


def _turning(rng, t):
    return 0.6 * np.sin(2 * math.pi * t / 7) + 0.3 * np.sin(2 * math.pi * t / 2.3)


def test_the_lag_is_found_when_frames_and_odometry_arrive_unevenly():
    """[uKM] 2: real frames come 40-90 ms apart and odometry on its own uneven clock; the turn rates must be per second
    of each clock, not per sample"""
    rng = np.random.default_rng(4); scene = rng.random((120, 3000)) * 255; f, lag = 400.0, 0.08
    ot = np.cumsum(rng.uniform(0.01, 0.05, 2000)); ot = ot[ot < 40]                    # odometry: 20-100 Hz, uneven
    fine = np.arange(0, 40, 0.001); yaw_fine = np.cumsum(_turning(rng, fine)) * 0.001
    yaw = np.interp(ot, fine, yaw_fine)
    ft = np.cumsum(rng.uniform(0.04, 0.09, 900)); ft = ft[ft < 39]                     # frames: 11-25 Hz, uneven
    frames = []
    for tk in ft:
        x0 = int(round(1500 - f * np.interp(tk, fine, yaw_fine)))
        img = np.zeros((240, 640)); img[:120] = scene[:, x0 - 320: x0 + 320]
        frames.append((tk + lag, img))
    got, c = cal.frame_lag(frames, ot, yaw)
    assert got is not None and abs(got - lag) < 0.003 and c > 0.99, (got, c)     # clean frames: the 5 ms grid point
                                                                                 # itself (px per frame, not per s: 0.075, 0.977)


def test_frames_that_do_not_match_their_neighbour_are_not_used():
    """[uKM] 2: a blank or garbled frame (a decode error, a stall) gives no turn rate rather than a wild one"""
    rng = np.random.default_rng(0); scene = rng.random((120, 2400)) * 255
    t = np.arange(0, 40, 0.05); yaw = np.cumsum(_turning(rng, t)) * 0.05; f, lag = 400.0, 0.08
    frames = []
    for k, tk in enumerate(t):
        x0 = int(round(1000 - f * yaw[k])); img = np.zeros((240, 640)); img[:120] = scene[:, x0 - 320: x0 + 320]
        if k % 7 == 3: img = rng.random((240, 640)) * 255                                  # a garbled frame
        frames.append((tk + lag, img))
    got, c = cal.frame_lag(frames, t, yaw)
    assert got is not None and abs(got - lag) < 0.015 and c > 0.4, (got, c)     # (some garbled pairs pass the 0.05
                                                                                 # response floor: c falls from 0.99)


# ---- [KkQ] the mount from the camera's own frames -------------------------------------------------------------------

H, W, K = 180, 240, (150.0, 150.0, 120.0, 90.0)


def _floor_tex(rng):
    tex = ndi.gaussian_filter(rng.random((1200, 1200)), 2.0)                             # 12 m of textured floor, 1 cm
    return (tex - tex.min()) / np.ptp(tex) * 255


def _pose(x, y, heading, h=0.35, down_deg=8.0):
    """world <- optical (x right, y down, z forward): a camera h m up, heading along the floor, tipped down_deg"""
    a = math.radians(down_deg); fwd = np.array([math.cos(heading) * math.cos(a), math.sin(heading) * math.cos(a), -math.sin(a)])
    right = np.array([math.sin(heading), -math.cos(heading), 0.0]); T = np.eye(4)
    T[:3, :3] = np.stack([right, np.cross(fwd, right), fwd], 1); T[:3, 3] = [x, y, h]; return T


def _render(tex, T, corr):
    fx, fy, cx, cy = K; v, u = np.mgrid[:H, :W]
    d = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1).reshape(-1, 3)
    w = d @ (T[:3, :3] @ corr).T; C = T[:3, 3]; down = w[:, 2] < -1e-3
    s = -C[2] / np.where(down, w[:, 2], -1e-3); P = C + w * s[:, None]
    g = ndi.map_coordinates(tex, [(P[:, 1] + 6) / 0.01, (P[:, 0] + 6) / 0.01], order=1, mode="wrap")
    return np.repeat(np.where(down, g, 128.0).reshape(H, W)[..., None], 3, -1).astype(np.uint8)


def test_the_mounts_pitch_and_yaw_are_found_from_walking_lanes_both_ways():
    """[KkQ] a camera truly pitched 0.6 deg further down and turned 0.4 deg right of where the model puts it: walking
    lanes both ways over a textured floor, the correction found lays the far views on the near ones (pitch) and the
    two directions' views on each other (yaw)"""
    tex = _floor_tex(np.random.default_rng(0)); true = (0.6, 0.4, 0.0)
    corr = cal._rot(*(math.radians(a) for a in true)); frames = []
    for lane in (-0.4, 0.4):
        for x in np.arange(-1.5, 1.6, 0.5):
            for heading, x0 in ((0.0, x - 1.5), (math.pi, x + 1.5)):
                T = _pose(x0, lane, heading); frames.append((_render(tex, T, corr), T, K))
    assert cal.agreement(frames, np.eye(3)) < 0.3 < 0.8 < cal.agreement(frames, corr)    # the uncorrected far views disagree
    (p, y, r), scores = cal.mount_correction(frames)
    assert abs(p - 0.6) < 0.1 and abs(y - 0.4) < 0.1 and abs(r) < 0.2, (p, y, r)
    assert min(scores) > 0.8
