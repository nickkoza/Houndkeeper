"""[KkQ]: the lidar places the robot in the house's frame despite its odometry's drift: a known error is recovered,
a corridor (walls that pin one direction only) doesn't let it slide along, and furniture that moved doesn't pull it."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from houndkeeper import localize as lz  # noqa: E402


def _wall(a, b, step=0.03, z=0.5):
    n = max(2, int(np.hypot(*(np.subtract(b, a))) / step))
    t = np.linspace(0, 1, n)[:, None]; p = np.asarray(a) + t * (np.subtract(b, a)); return np.c_[p, np.full(len(p), z)]


def _room():
    walls = [((0, 0), (6, 0)), ((6, 0), (6, 4)), ((6, 4), (0, 4)), ((0, 4), (0, 0)), ((2, 1.5), (3, 1.5)), ((4.5, 2.5), (4.5, 3.5))]
    return np.vstack([_wall(a, b) for a, b in walls])


def _moved(points, T):
    """points as a robot whose belief is off by T would place them"""
    q = points.copy(); q[:, :2] = points[:, :2] @ T[:2, :2].T + T[:2, 2]; return q


def test_a_known_odometry_error_is_recovered():
    house = lz._voxel(_room()[:, :2]); err = lz.se2(0.06, -0.04, math.radians(1.5))   # the belief off by 6, 4 cm, 1.5 deg
    rng = np.random.default_rng(0); scan = _room() + rng.normal(0, 0.02, (len(_room()), 3)) * [1, 1, 0]
    loc = lz.Localizer(house).run([(0.0, _moved(scan, np.linalg.inv(err)))])
    x, y, th = lz.se2_params(loc.Ts[-1] @ np.linalg.inv(err))                  # the correction undoes the error
    assert np.hypot(x, y) < 0.01 and abs(math.degrees(th)) < 0.3


def test_a_corridor_does_not_let_it_slide():
    corridor = np.vstack([_wall((0, 0), (8, 0)), _wall((0, 1.2), (8, 1.2))])    # nothing pins x
    house = lz._voxel(corridor[:, :2]); rng = np.random.default_rng(1)
    scans = [(0.5 * k, corridor + rng.normal(0, 0.02, corridor.shape) * [1, 1, 0]) for k in range(60)]
    loc = lz.Localizer(house).run(scans)
    xs = [lz.se2_params(T)[0] for T in loc.Ts]
    assert max(abs(v) for v in xs) < 0.02                                         # the belief was right: it stays put


def test_furniture_that_moved_does_not_pull_it():
    room = _room(); house = lz._voxel(room[:, :2]); rng = np.random.default_rng(2)
    chairs = np.vstack([_wall((1.0 + 0.5 * k, 2.8), (1.3 + 0.5 * k, 2.8)) for k in range(6)])   # not in the map
    scan = np.vstack([room, chairs + [0.0, 0.0, 0.0]]) + rng.normal(0, 0.02, (len(room) + len(chairs), 3)) * [1, 1, 0]
    loc = lz.Localizer(house).run([(0.0, scan)])
    x, y, th = lz.se2_params(loc.Ts[-1])
    assert np.hypot(x, y) < 0.01 and abs(math.degrees(th)) < 0.3


def test_it_never_wanders_far_from_what_odometry_could_have_drifted():
    """27 Sep: scan after scan in a symmetric room, the correction crept to -65 deg and then snapped to a wrong fit: it
    must stay within what the odometry can drift (a few cm and deg, growing slowly with distance walked)"""
    square = np.vstack([_wall((0, 0), (4, 0)), _wall((4, 0), (4, 4)), _wall((4, 4), (0, 4)), _wall((0, 4), (0, 0))])   # 90 deg symmetric
    house = lz._voxel(square[:, :2]); rng = np.random.default_rng(3)
    scans = []
    for k in range(200):                                                       # walking about, the belief right all along,
        pts = square + rng.normal(0, 0.03, square.shape) * [1, 1, 0]           # but each scan seeing only part of the room
        keep = rng.random(len(pts)) < 0.35
        scans.append((0.5 * k, pts[keep]))
    loc = lz.Localizer(house).run(scans, travelled=[0.1 * k for k in range(200)])
    worst = max(max(math.hypot(*lz.se2_params(T)[:2]), abs(math.degrees(lz.se2_params(T)[2])) / 20) for T in loc.Ts)
    assert worst < 0.10                                                        # never beyond 10 cm, nor 2 deg


def test_a_scan_that_mostly_does_not_match_the_map_is_not_trusted():
    """27 Sep, night 2: 172 of ~3,000 structure points matched (the rest new to the map), the fit was taken, and the
    correction jumped 17 deg; a fit must match most of what the scan saw"""
    house = lz._voxel(_room()[:, :2]); rng = np.random.default_rng(4)
    unknown = np.vstack([_wall((8 + 0.3 * k, -2), (8 + 0.3 * k, 6)) for k in range(12)])   # a part of the house not mapped
    few = _room()[rng.random(len(_room())) < 0.12]
    scan = np.vstack([few, unknown]) + rng.normal(0, 0.02, (len(few) + len(unknown), 3)) * [1, 1, 0]
    loc = lz.Localizer(house).run([(0.0, _room()), (0.5, _moved(scan, lz.se2(0.3, 0.0, math.radians(8))))])
    x, y, th = lz.se2_params(loc.Ts[-1])
    assert np.hypot(x, y) < 0.02 and abs(math.degrees(th)) < 0.5                 # held where it was


def test_heights_are_measured_from_the_floor_under_a_tilted_scan():
    """27 Sep: the walking body's pitch and roll (~0.5 deg) put the floor's far returns 3 cm up and the tail past 6 cm,
    which the pictures took for things standing; a box 20 cm tall stays 20 cm"""
    rng = np.random.default_rng(5)
    xy = rng.uniform(-4, 4, (6000, 2)); tilt = np.tan(math.radians(0.6))
    floor = np.c_[xy, 0.03 + tilt * xy[:, 0] + rng.normal(0, 0.014, len(xy))]
    box = np.c_[rng.uniform(1.0, 1.2, (200, 2)), np.full(200, 0.20)]; box[:, 2] += 0.03 + tilt * box[:, 0]
    L = lz.level(np.vstack([floor, box]))
    assert (L[:6000, 2] > 0.06).mean() < 0.002                                 # the floor stays floor
    assert abs(np.median(L[6000:, 2]) - 0.20) < 0.01
