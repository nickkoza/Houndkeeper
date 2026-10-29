"""Tonight placed live by the house's graph ([x9X], [JzW]): a synthetic room whose baseline keyframes are a fixed map;
tonight's odometry bent or slipped; the placer, fed one sweep at a time, takes the plant back where odometry alone
cannot, reads nothing after the moment asked, and a floor lock pulls the pose by its own measurement."""
import math
import sys
import time
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def room():
    """a 7 x 5 m room's walls with a cupboard and a pillar, points every 2 cm at 0.5 m up (the house frame)"""
    def seg(a, b):
        a, b = np.asarray(a, float), np.asarray(b, float); k = max(2, int(np.hypot(*(b - a)) / 0.02))
        return a + (b - a) * np.linspace(0, 1, k)[:, None]
    w = [seg((0, 0), (7, 0)), seg((7, 0), (7, 5)), seg((7, 5), (0, 5)), seg((0, 5), (0, 0)),
         seg((2, 3), (3.2, 3)), seg((3.2, 3), (3.2, 3.6)), seg((3.2, 3.6), (2, 3.6)), seg((2, 3.6), (2, 3)),
         seg((5, 1), (5.4, 1)), seg((5.4, 1), (5.4, 1.4)), seg((5.4, 1.4), (5, 1.4)), seg((5, 1.4), (5, 1))]
    return np.vstack(w)


def seen(W, X, r=5.0):
    """the walls within r of the robot at X (house <- base), in its body frame"""
    import joint_graph as jg
    B = (W - X[:2, 2]) @ X[:2, :2]; return B[np.hypot(*B.T) < r]


def walk(n=140):
    """a loop through the room, 5 cm a step (true poses, house frame)"""
    import joint_graph as jg
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n; out.append(jg.se2(3.5 + 2.2 * math.cos(a), 2.3 + 1.4 * math.sin(a), a + math.pi / 2))
    return out


def house(W):
    import live_place as lp
    import joint_graph as jg
    X = walk(60); pts = [seen(W, x) for x in X]
    return lp.HouseKeyframes(np.arange(len(X), dtype=float), np.ones(len(X), int), [jg.params(x) for x in X], np.vstack(pts),
                             np.cumsum([0] + [len(p) for p in pts]))


def odometry(true, bend_deg=0.0, slip=None):
    """odometry from the true steps, its heading bent bend_deg over the second half, or slipped (dx, dy) at the middle"""
    import joint_graph as jg
    n = len(true); out = [true[0]]
    for k in range(1, n):
        extra = math.radians(bend_deg) / (n // 2) if k > n // 2 else 0.0
        out.append(out[-1] @ jg.se2(0, 0, extra) @ (jg.inv(true[k - 1]) @ true[k]))
    if slip is not None:
        out = [o if k < n // 2 else jg.se2(*slip, 0.0) @ o for k, o in enumerate(out)]
    return out


def sweep(W, Xt, O):
    """a sweep as recorded: the walls the robot truly sees, laid in odometry's world frame by its belief O"""
    B = seen(W, Xt); P = B @ O[:2, :2].T + O[:2, 2]; return np.c_[P, np.full(len(P), 0.5)]


def run(placer, W, true, odo):
    est = []
    for k, (Xt, O) in enumerate(zip(true, odo)):
        placer.scan(float(k), sweep(W, Xt, O), O); est.append(placer.correction() @ O)
    return est


def err(a, b): return float(np.mean([np.hypot(*(x[:2, 2] - y[:2, 2])) for x, y in zip(a, b)]))


def test_a_planted_heading_bend_is_taken_back_by_the_house_keyframes():
    """[x9X]: tonight's odometry turns 12 deg over the walk's second half; matched to the house's keyframes as it
    walks, the placement stays within a few cm where odometry alone ends up decimetres off"""
    import live_place as lp
    W = room(); true = walk(); odo = odometry(true, bend_deg=12.0)
    est = run(lp.Placer(house(W), floor=False), W, true, odo)
    assert err(odo, true) > 0.08 and err(est, true) < 0.2 * err(odo, true), (err(odo, true), err(est, true))


def test_a_planted_slip_is_taken_back():
    """[x9X]: odometry jumps 8 cm mid-walk and stays; the placement follows the walls back"""
    import live_place as lp
    W = room(); true = walk(); odo = odometry(true, slip=(0.08, -0.04))
    est = run(lp.Placer(house(W), floor=False), W, true, odo)
    late = slice(len(true) // 2 + 20, None)
    assert err(odo[late], true[late]) > 0.08 and err(est[late], true[late]) < 0.02, (err(odo[late], true[late]), err(est[late], true[late]))


def test_without_the_house_the_placement_is_odometry():
    """[x9X]: lidar off and no floor lock: the graph holds odometry (the control's other side)"""
    import live_place as lp
    W = room(); true = walk(); odo = odometry(true, bend_deg=8.0)
    est = run(lp.Placer(house(W), lidar=False, floor=False), W, true, odo)
    assert abs(err(est, true) - err(odo, true)) < 0.005


def test_the_feed_reads_nothing_after_the_moment_asked():
    """[x9X]: causal: until(t) takes only the sweeps stamped up to t, and the correction then is what those gave"""
    import live_place as lp
    W = room(); true = walk(60); odo = odometry(true, bend_deg=6.0)
    pl = lp.Placer(house(W), floor=False)
    fd = lp.Feed.__new__(lp.Feed); fd.placer, fd.hook, fd.i, fd.seconds, fd._clock = pl, None, 0, [], time.perf_counter
    fd.scans = [(float(k), sweep(W, x, o)) for k, (x, o) in enumerate(zip(true, odo))]
    O4 = {}
    for k, o in enumerate(odo):
        M = np.eye(4); M[:2, :2] = o[:2, :2]; M[:2, 3] = o[:2, 2]; O4[float(k)] = M
    fd.pose = lambda t: O4.get(float(t))
    fd.until(29.5); C1 = pl.correction(); n1 = len(pl.nodes)
    assert fd.i == 30 and all(t <= 29.5 for t, _k, _s in fd.seconds)
    pl2 = lp.Placer(house(W), floor=False)
    for k in range(30): pl2.scan(float(k), fd.scans[k][1], odo[k])
    assert np.allclose(C1, pl2.correction()) and n1 == len(pl2.nodes)


def test_a_floor_lock_pulls_the_pose_by_its_measurement_and_an_outlier_less():
    """[x9X]: with no lidar, a floor lock saying the frame sits 3 cm east moves the estimate most of the way there;
    a lock 1 m off moves it far less than its own size (robust kernel)"""
    import live_place as lp
    import joint_graph as jg
    hk = house(room()); O = jg.se2(1.0, 1.0, 0.0)
    pl = lp.Placer(hk, lidar=False); pl.lock(0.0, O, jg.se2(0.03, 0.0, 0.0), O[:2, 2], np.eye(3))
    dx = (pl.correction() @ O)[0, 2] - O[0, 2]
    assert 0.015 < dx <= 0.031, dx
    pl2 = lp.Placer(hk, lidar=False); pl2.lock(0.0, O, jg.se2(0.03, 0.0, 0.0), O[:2, 2], np.eye(3))
    for k in range(1, 6): pl2.lock(float(k), O, jg.se2(0.03, 0.0, 0.0), O[:2, 2], np.eye(3))
    x0 = (pl2.correction() @ O)[0, 2]; pl2.lock(6.0, O, jg.se2(1.0, 0.0, 0.0), O[:2, 2], np.eye(3))
    assert (pl2.correction() @ O)[0, 2] - x0 < 0.25


def test_the_confirmers_lock_is_the_whole_track_step():
    """[x9X]: the lock handed to the graph is the floor tracking's step at gain 1, turning about the window's middle"""
    import confirm_stream as cs
    P = (slice(100, 300), slice(400, 650))
    S, pc = cs.lock_pose(1.0, 3, -2, P)
    assert np.allclose(S, cs.track_step(1.0, 3, -2, P, 1.0)) and np.allclose(S[:2, :2] @ pc + S[:2, 2] - pc, [-0.02, -0.03])


def test_the_live_placer_takes_sweeps_in_a_worker_and_answers_the_newest():
    """[x9X]: the live seam: odometry with heading and time, sweeps queued, correction() the newest estimate"""
    import live_place as lp
    import joint_graph as jg
    W = room(); true = walk(60); odo = odometry(true, bend_deg=6.0); lv = lp.LivePlacer(house(W))
    try:
        for k, (x, o) in enumerate(zip(true, odo)):
            lv.odom(o[:2, 2], yaw=jg.params(o)[2], t=float(k) - 0.05); lv.odom(o[:2, 2], yaw=jg.params(o)[2], t=float(k) + 0.05)
            lv.scan(float(k), sweep(W, x, o))
            for _ in range(200):
                if not lv._q: break
                time.sleep(0.01)
        time.sleep(0.2)
        est = lv.correction() @ odo[-1]
        assert np.hypot(*(est[:2, 2] - true[-1][:2, 2])) < 0.03
    finally:
        lv.close()
