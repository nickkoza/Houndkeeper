"""The joint pose graph's constraints and solve ([JzW]): a match measured as a move in the world becomes the right
between factor, and a planted odometry bend comes out when the other sensors' constraints hold the true shape."""
import math
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def test_a_world_move_becomes_the_between_factors_error_by_the_adjoint():
    """[JzW]: the information of a match (a move about a point, laying j's scan or view on i's) carried into the
    between factor's frame matches finite differences"""
    import gtsam
    import joint_graph as jg
    Xa, Xb = jg.se2(1, 2, 0.3), jg.se2(3, -1, 1.2); o = np.array([2.0, 0.5]); M = jg.about(o, 0.02, (0.03, -0.01))
    Z = jg.inv(Xa) @ M @ Xb; S = np.eye(3); S[:2, 2] = o; J = np.zeros((3, 3)); eps = 1e-6
    for k in range(3):
        d = np.zeros(3); d[k] = eps
        Zd = jg.inv(Xa) @ S @ jg.se2(*d) @ jg.inv(S) @ M @ Xb
        J[:, k] = np.array(gtsam.Pose2.Logmap(gtsam.Pose2(*jg.params(Z)).between(gtsam.Pose2(*jg.params(Zd))))) / eps
    I = np.diag([1e4, 4e4, 9e3]); Ib = jg.info_between(I, o, M @ Xb)
    assert np.allclose(np.linalg.inv(J).T @ I @ np.linalg.inv(J), Ib, rtol=1e-4, atol=1e-3)


def test_a_planted_odometry_bend_comes_out_where_the_other_constraints_hold_the_shape():
    """[JzW]: the positive control in small: a walk whose odometry turns 5 deg over its second half; scan-pair
    constraints between poses a few steps apart hold the true relative poses; the solve takes back nearly all of it"""
    import joint_graph as jg
    n = 41; true = [jg.se2(0.25 * k, 0.0, 0.0) for k in range(n)]
    bend = [math.radians(5.0) * max(0, k - n // 2) / (n // 2) for k in range(n)]
    odo = [true[0]]
    for k in range(1, n):
        odo.append(odo[-1] @ jg.se2(0, 0, bend[k] - bend[k - 1]) @ (jg.inv(true[k - 1]) @ true[k]))
    nodes = jg.Nodes.of({1: list(range(n))})
    fam = {"odom": [(k, k + 1, jg.inv(odo[k]) @ odo[k + 1], np.diag([1 / 0.01 ** 2, 1 / 0.01 ** 2, 1 / math.radians(0.3) ** 2]))
                    for k in range(n - 1)],
           "lidar": [(k, k + 4, jg.inv(true[k]) @ true[k + 4], np.diag([1 / 0.003 ** 2, 1 / 0.003 ** 2, 1 / math.radians(0.05) ** 2]))
                     for k in range(n - 4)]}
    X, st = jg.solve(nodes, odo, fam, [(0, (1e-4, 1e-5))], log=lambda m: None)
    planted = np.mean([np.hypot(*(a[:2, 2] - b[:2, 2])) for a, b in zip(odo, true)])
    left = np.mean([np.hypot(*(a[:2, 2] - b[:2, 2])) for a, b in zip(X, true)])
    assert planted > 0.03 and left < 0.05 * planted, (planted, left)
