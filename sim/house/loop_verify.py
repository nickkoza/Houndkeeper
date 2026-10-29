#!/usr/bin/env python3
"""Loop-closure verification before a loop enters DimOS's pose graph ([JzW] front end, 8 Oct 2026). DimOS's pose graph
accepts a loop on ICP fitness alone, the mean squared distance of the inliers it found, which says nothing of how many
it found: a scan laid on a wrong place with a few close points scores well (house 72's dock, [myl]). A false loop does
its damage through the start it bends, which the later solves associate from (5 Oct), so the place to stop it is
before it enters the graph. DimOS asks a verifier (its `loop_verifier` extension point, `LoopCandidate`); this is ours:
a loop is accepted only when the robot's other evidence agrees with the alignment it proposes:
  - the lidar: the scans around the loop's two keyframes (house_shape's keyframes, a few either side, each placed from
    its keyframe by odometry), laid on each other at the proposed alignment and matched again (house_shape's
    point-to-line ICP): consistent when most matched pairs need less than LIDAR_TOL more to lie on each other;
  - or the floor: the floor views nearest the two keyframes' times, facing within FLOOR_MAX_TURN_DEG, laid at the
    proposed alignment and registered (stitch_floor's lock): agreeing when the lock needs less than FLOOR_TOL more.
No match either way (too little wall in common, no floor views) is no evidence, and the loop is refused: the scan pairs
and floor locks of the later solves tie revisits without it (5 Oct: houses 70, 72, 73 identical without DimOS's loops).
Reads only the robot's recordings and what it derived from them ([1B2])."""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import math
from dataclasses import dataclass, field

import numpy as np

LIDAR_TOL = (0.05, math.radians(2.0))     # a re-match needing more than this is the loop's alignment disagreeing
LIDAR_SIDE = 3                            # house_shape keyframes either side of each loop keyframe (within LIDAR_WINDOW_S)
LIDAR_WINDOW_S = 4.0
LIDAR_MIN_AGREE = 0.6                     # the share of matched pairs that must agree
FLOOR_TOL = (0.05, math.radians(1.5))
FLOOR_WINDOW_S = 1.5                      # views this near the keyframes' times
FLOOR_MAX_TURN_DEG = 25.0                 # only like-facing views ([4t7]: locks between views facing apart are biased)
MODES = ("lidar", "floor", "either", "log")   # "log": every loop accepted, each verdict recorded (the bench's confusion)


def se2_of_tf(tf):
    M = tf.to_matrix(); th = math.atan2(M[1, 0], M[0, 0]); c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, M[0, 3]], [s, c, M[1, 3]], [0, 0, 1.0]])


def _inv(T):
    R = T[:2, :2]; o = np.eye(3); o[:2, :2] = R.T; o[:2, 2] = -R.T @ T[:2, 2]; return o


@dataclass
class LoopVerifier:
    """callable(candidate) -> bool for DimOS's PGO(loop_verifier=...). tracks: {night: joint_graph.Track} (odometry);
    kf: [(night, t, body points (n, 2), travelled)] (house_shape's keyframes, joint_graph.load_problem's); views:
    optional (floor_refine views, ts, night, painted {night: Localizer}, meta {C, F}) for the floor check"""
    tracks: dict
    kf: list
    views: object = None
    mode: str = "either"
    record: list = field(default_factory=list)

    def __post_init__(self):
        self.kt = np.array([k[1] for k in self.kf]); self.kn = np.array([k[0] for k in self.kf])

    def night_of(self, t):
        for n, tr in self.tracks.items():
            if tr.t[0] - 1 <= t <= tr.t[-1] + 1: return n
        return None

    def near(self, n, t):
        """house_shape keyframes of night n nearest time t: indices, up to 2 LIDAR_SIDE + 1"""
        idx = np.nonzero((self.kn == n) & (np.abs(self.kt - t) <= LIDAR_WINDOW_S))[0]
        return idx[np.argsort(np.abs(self.kt[idx] - t))][:2 * LIDAR_SIDE + 1]

    def lidar(self, Wt, Ws, nt, ns, tt, ts):
        """the scans around the two keyframes, placed by the proposed alignment, matched again: (verdict, detail)"""
        import house_shape as hs
        Ot, Os = self.tracks[nt].at(tt), self.tracks[ns].at(ts)
        def placed(i, W, O):
            n, t, B = self.kf[i][0], self.kf[i][1], self.kf[i][2]; X = W @ _inv(O) @ self.tracks[n].at(t)
            return B @ X[:2, :2].T + X[:2, 2], X[:2, 2]
        A = [placed(i, Wt, Ot) for i in self.near(nt, tt)]; B = [placed(i, Ws, Os) for i in self.near(ns, ts)]
        good = bad = 0; moves = []
        for pb, cb in B:
            for pa, _ca in A:
                r = hs.icp_info(pb, pa, np.eye(3))
                if r is None or r[2] > hs.MAX_RMS: continue
                T = r[0]; mv = hs.shift_at(T, cb); tn = abs(math.atan2(T[1, 0], T[0, 0]))
                moves.append((round(mv, 3), round(math.degrees(tn), 2)))
                if mv <= LIDAR_TOL[0] and tn <= LIDAR_TOL[1]: good += 1
                else: bad += 1
        if good + bad == 0: return None, {"pairs": len(A) * len(B), "matched": 0}
        return good / (good + bad) >= LIDAR_MIN_AGREE, {"pairs": len(A) * len(B), "matched": good + bad, "agree": good,
                                                         "move_median_m": float(np.median([m[0] for m in moves]))}

    def floor(self, Wt, Ws, nt, ns, tt, ts):
        """the floor views nearest the two keyframes, laid at the proposed alignment and locked: (verdict, detail)"""
        if self.views is None: return None, {"views": 0}
        import confirm_stream as cs, floor_refine as fr, stitch_floor as sf
        V, vts, vn, painted, meta = self.views
        def pick(n, t):
            return [k for k in np.nonzero((vn == n) & (np.abs(vts - t) <= FLOOR_WINDOW_S))[0]]
        def moved(k, W, O, n):
            Xp = painted[n].correction(vts[k]) @ self.tracks[n].clean(vts[k]); X = W @ _inv(O) @ self.tracks[n].at(vts[k])
            M = X @ _inv(Xp); v = V[k]; o = fr.centre(v.box); d = M[:2, :2] @ o + M[:2, 2] - o
            pl = v.placed(np.array([d[0], d[1], math.atan2(M[1, 0], M[0, 0])]))
            return pl, M[:2, :2] @ meta["C"][k] + M[:2, 2], M[:2, :2] @ meta["F"][k]
        Ot, Os = self.tracks[nt].at(tt), self.tracks[ns].at(ts)
        A = [moved(k, Wt, Ot, nt) for k in pick(nt, tt)]; B = [moved(k, Ws, Os, ns) for k in pick(ns, ts)]
        pairs = [(a, b) for a in A for b in B if a[2] @ b[2] > math.cos(math.radians(FLOOR_MAX_TURN_DEG))]
        if not pairs: return None, {"views": len(A) + len(B), "like_pairs": 0}
        good = bad = 0
        for a, b in pairs:
            sf._V = cs.Views([a[1], b[1]], [a[2], b[2]], [a[0].box, b[0].box], [a[0].rgb, b[0].rgb], np.zeros(2))
            m = sf.measure((0, 1))
            if m is None: continue
            if math.hypot(m[4], m[5]) <= FLOOR_TOL[0] and abs(m[6]) <= FLOOR_TOL[1]: good += 1
            else: bad += 1
        if good + bad == 0: return None, {"like_pairs": len(pairs), "locked": 0}
        return good >= bad and good > 0, {"like_pairs": len(pairs), "locked": good + bad, "agree": good}

    def __call__(self, c):
        Wt = se2_of_tf(c.target); Z = se2_of_tf(c.offset); Ws = Wt @ Z
        nt, ns = self.night_of(c.target_ts), self.night_of(c.source_ts)
        rec = {"t_target": float(c.target_ts), "t_source": float(c.source_ts), "score": float(c.score),
               "offset": [float(Z[0, 2]), float(Z[1, 2]), math.atan2(Z[1, 0], Z[0, 0])]}
        cur = _inv(Wt) @ se2_of_tf(c.source); ask = _inv(cur) @ Z
        rec.update(ask_turn_deg=math.degrees(math.atan2(ask[1, 0], ask[0, 0])), ask_move_m=float(np.hypot(*ask[:2, 2])))
        lv = fv = None
        if nt is not None and ns is not None:
            if self.mode in ("lidar", "either", "log"): lv, rec["lidar"] = self.lidar(Wt, Ws, nt, ns, c.target_ts, c.source_ts)
            if self.mode in ("floor", "log") or (self.mode == "either" and not lv): fv, rec["floor"] = self.floor(Wt, Ws, nt, ns, c.target_ts, c.source_ts)
        rec["lidar_ok"], rec["floor_ok"] = lv, fv
        ok = {"lidar": bool(lv), "floor": bool(fv), "either": bool(lv) or bool(fv), "log": True}[self.mode]
        rec["accepted"] = ok; self.record.append(rec)
        return ok
