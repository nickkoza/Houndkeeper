"""Loop-closure verification before a loop enters DimOS's pose graph (loop_verify.py, [JzW]'s front end, [myl]): a
loop whose proposed alignment the scans around it agree with is accepted; one that lays them a step or a turn apart
is refused; one with no evidence either way is refused."""
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def _room():
    """an L-shaped room's walls with a few posts, sampled every 2 cm (world frame)"""
    pts = []
    for (x0, y0, x1, y1) in ((0, 0, 6, 0), (6, 0, 6, 3), (6, 3, 3, 3), (3, 3, 3, 5), (3, 5, 0, 5), (0, 5, 0, 0)):
        n = int(math.hypot(x1 - x0, y1 - y0) / 0.02); t = np.linspace(0, 1, n)
        pts.append(np.c_[x0 + (x1 - x0) * t, y0 + (y1 - y0) * t])
    for cx, cy in ((1.5, 1.2), (4.5, 1.8), (1.2, 3.8)):
        a = np.linspace(0, 2 * math.pi, 40); pts.append(np.c_[cx + 0.15 * np.cos(a), cy + 0.15 * np.sin(a)])
    return np.vstack(pts)


def _tf(T):
    """a 2-D pose as the minimal Transform the verifier reads (to_matrix)"""
    M = np.eye(4); M[:2, :2] = T[:2, :2]; M[:2, 3] = T[:2, 2]
    return SimpleNamespace(to_matrix=lambda: M)


def _verifier(drift):
    """a robot that walks the room on night 1 and again on night 2 whose odometry is off by `drift` (3x3): keyframes
    every 0.25 s along the same path, each scan the room's walls within 4 m, in its body frame"""
    import joint_graph as jg
    import loop_verify as lv
    W = _room(); path = [jg.se2(0.8 + 0.2 * k, 0.8 + 0.05 * k, 0.1) for k in range(20)]
    tracks, kf = {}, []
    for n, off, t0 in ((1, np.eye(3), 0.0), (2, drift, 100.0)):
        ts = t0 + 0.25 * np.arange(len(path)); odo = [off @ P for P in path]
        tracks[n] = jg.Track(ts, np.array([jg.params(O) for O in odo]))
        for t, P in zip(ts, path):
            B = (W - P[:2, 2]) @ P[:2, :2]; B = B[np.hypot(*B.T) < 4.0]
            kf.append((n, float(t), B, 0.0))
    return lv.LoopVerifier(tracks, kf, None, "lidar"), path, tracks


def _candidate(v, path, tracks, k_tgt, k_src, err):
    """a loop from night 2's keyframe k_src to night 1's k_tgt, its offset the true one times err (3x3)"""
    import joint_graph as jg
    t_t, t_s = tracks[1].t[k_tgt], tracks[2].t[k_src]
    Wt = path[k_tgt]; Z = jg.inv(path[k_tgt]) @ path[k_src] @ err
    return SimpleNamespace(target_ts=t_t, source_ts=t_s, target=_tf(Wt), source=_tf(Wt @ Z), offset=_tf(Z), score=0.01)


def test_a_true_loop_is_accepted():
    """[JzW], [myl]: the scans around the loop lie on each other at its proposed alignment"""
    import joint_graph as jg
    v, path, tr = _verifier(jg.se2(0.4, -0.3, math.radians(8)))
    assert v(_candidate(v, path, tr, 10, 10, np.eye(3)))
    assert v.record[-1]["lidar_ok"] is True


def test_a_loop_a_step_or_a_turn_wrong_is_refused():
    """[JzW], [myl]: a false loop (the planted-loop control's 0.6 m and 15 deg, and a subtler 15 cm) is refused"""
    import joint_graph as jg
    v, path, tr = _verifier(jg.se2(0.4, -0.3, math.radians(8)))
    for err in (jg.se2(0.6, 0.0, math.radians(15)), jg.se2(0.15, 0.0, 0.0), jg.se2(0.0, 0.0, math.radians(6))):
        assert not v(_candidate(v, path, tr, 10, 10, err)), err


def test_a_loop_with_no_evidence_is_refused():
    """[JzW], [myl]: no scans near the loop's keyframes (and no floor views): no evidence, refused"""
    import joint_graph as jg
    v, path, tr = _verifier(np.eye(3))
    c = _candidate(v, path, tr, 10, 10, np.eye(3)); c.target_ts += 50.0; c.source_ts += 50.0
    assert not v(c)
