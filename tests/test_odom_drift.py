"""[2Ak]: the sim's odometry drifts as a real robot's does ([x3f]), and the lidar lands in the same drifted frame."""
import math

import numpy as np

from dimos.simulation.mujoco.odom_drift import OdomDrift

SPEC = "scale_sd=0.02,xy_sd=0.005,yaw_sd_deg=0.5,yaw_rate_sd_deg=0.05,start_xy_sd=0.02,start_yaw_sd_deg=1.0,seed={}"


def _walk(d, metres=16.0, step=0.01):
    t, p, errs = 0.0, np.zeros(3), []
    q = np.array([1.0, 0, 0, 0])
    for k in range(int(metres / step)):
        t += 0.02; p = np.array([k * step, 0.0, 0.3])
        b, qb = d.step(p, q, t); errs.append(float(np.linalg.norm(b[:2] - p[:2])))
    return p, b, errs


def test_the_estimate_drifts_with_distance_and_differs_each_night():
    ends = []
    for seed in range(8):
        _, b, errs = _walk(OdomDrift(SPEC.format(seed)))
        assert errs[-1] > errs[50]                                          # grows as it walks
        ends.append(errs[-1])
    assert 0.01 * 16 < np.median(ends) < 0.06 * 16                         # 1-6% of the distance walked (legged odometry)
    assert np.std(ends) > 0.005                                            # each night its own


def test_the_lidar_is_placed_where_the_robot_believes_it_is():
    d = OdomDrift(SPEC.format(1)); p, b, _ = _walk(d)
    ahead = np.array([[p[0] + 2.0, p[1], 0.1]])                             # a wall 2 m straight ahead, truly
    got = d.to_belief(np.vstack([p, ahead]))
    assert np.allclose(got[0, :2], b[:2], atol=1e-9)                       # the robot's own spot lands on its belief
    assert abs(np.linalg.norm(got[1, :2] - got[0, :2]) - 2.0) < 1e-9       # distances are kept
