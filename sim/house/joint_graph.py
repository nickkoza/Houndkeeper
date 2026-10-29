#!/usr/bin/env python3
"""One pose graph for the house's nights, the floor a sensor in it ([JzW]): the floor a
part of pose-based SLAM, not bolted onto its side. Today three solves run in a row,
each holding the one before fixed: DimOS's pose graph (house_frame.py, [KkQ]), the house's shape from every lidar scan
pair (house_shape.py, [FHA]), then the floor stitch (stitch_floor.py, [WFI]). Here one GTSAM factor graph over the
robot's poses on both nights (SE(2), a node at each lidar keyframe, each floor view and each loop's keyframes) holds
  - odometry between a night's consecutive poses, as loose as house_shape's (it drifts ~3 cm and ~1 deg a metre);
  - every lidar scan pair's match (house_shape's own pairs and matcher), weighed by its information J'J (a wall holds
    across itself, not along), never surer than 3 mm (decision [ObC]);
  - DimOS's loop closures (its pose graph's own, with its own noise: the ICP fitness as translation variance);
  - every floor-view pair's lock (stitch_floor's own pairs and snap), at stitch_floor's weight;
every factor robust (Huber), solved by Levenberg-Marquardt, and matched again from the new poses for a few passes, as
the shape solve and the stitch are. The measurements are each sensor's own relative pose between two of the robot's
poses: nothing asks for "no change from the stage before" ([qLu]: [G9q]'s refine held the shape as solved and took
back 1% of a planted error). Only the solve is new. Writes each night's corrections (times, 3x3 SE(2) world <-
believed) for floor_mosaic2.py --corrections, as house_frame/house_shape/stitch_floor do.
Reads only the robot's recordings and what it derived from them ([1B2]): the sim's truth is the bench's alone.
Usage: joint_graph.py frame <db1> <db2> --out <dir> [--pgo k=v ...]                      (DimOS PGO with its loops)
       joint_graph.py solve <db1> <db2> --frame <dir> --views v1.npz v2.npz --painted <dir> --out <dir>   (dimos venv)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import argparse, json, math, time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ODO_SD = (0.005, 0.03, math.radians(0.1), math.radians(1.0))   # house_shape's SEQ_SD: per keyframe step (~25 cm) m + m/m,
ODO_STEP_M = 0.25                                               # rad + rad/m; a shorter step's constant part shrinks as
ODO_TURN = 0.01                                                 # sqrt(d / 25 cm); and 1% of what it turned
LIDAR_SD_FLOOR_M, LIDAR_SD_FLOOR_DEG = 0.003, 0.05              # a lidar pair never surer than this ([ObC])
FLOOR_SD_FLOOR_M = 0.015                                        # nor a floor lock: locks between views are biased by the
FLOOR_MAX_TURN_DEG = 25.0                                       # camera's geometry along each view (5 Oct, house 70, against
                                                                # the truth: 12 mm median between views facing within 15 deg,
                                                                # 23 at 15-30, 32 at 30-60, 56 at 60-120, 100 facing apart),
                                                                # so only views facing within this are paired ([WFI]'s rule:
                                                                # from about the same view their errors fall the same way)
FLOOR_FRONT = "like"                                            # "like": only views facing within FLOOR_MAX_TURN_DEG, each
                                                                # lock at stitch_floor's weight; "dir" ([4t7]): every pair,
                                                                # each lock corrected by the directional floor model's
                                                                # displacement and weighed by what its facing difference earns
DIR_BINS = 8                                                    # the directional model's azimuth bins (dirfloor.NB)
DIR_BANDS = (0.0, 15.0, 30.0, 60.0, 120.0, 181.0)               # facing difference bands, each its own spread
DIR_CROSS_PER_VIEW = 4                                          # pairs facing apart a view, besides its like-facing ones
DIR_SD_MODE = "table"                                           # each band's spread: "table", fixed below; "self", from the
                                                                # locks' own residuals at the current poses (which the floor
                                                                # has pulled: overconfident facing apart, house 70: 25 mm
                                                                # against 38 mm judged by the truth)
DIR_SD_TABLE_M = {(0.0, 15.0): 0.012, (15.0, 30.0): 0.019, (30.0, 60.0): 0.025, (60.0, 120.0): 0.045, (120.0, 181.0): 0.038}
                                                                # house 70 alone (6 Oct, the corrected locks against the
                                                                # truth, median / 1.177); every other house is fresh to it
DIR_MIN_BAND = 20                                               # a band's spread from this many locks, else the widest's
FLOOR_PASSES = (0, 2)                                           # the floor matched again on these passes (3 min a pass)
HUBER_K = {"odom": 3.0, "lidar": 1.345, "floor": 1.345, "loop": 1.345}   # in sds of each factor's own noise
KERNEL = {"odom": "huber", "lidar": "huber", "floor": "huber", "loop": "huber"}   # or "cauchy" (redescending-ish: a gross
                                                                # outlier's pull fades, where Huber's stays constant)
LOOP_ROT_VAR = 0.05                                             # house_frame's PGO_SETTINGS: DimOS's loop noise
FIRST_N2_SD = (1.0, 0.5)                                        # night 2's first pose: loosely where the frame put it
PASSES = 4


def se2(x, y, th):
    c, s = math.cos(th), math.sin(th); return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def params(T):
    return float(T[0, 2]), float(T[1, 2]), math.atan2(T[1, 0], T[0, 0])


def inv(T):
    R = T[:2, :2]; o = np.eye(3); o[:2, :2] = R.T; o[:2, 2] = -R.T @ T[:2, 2]; return o


def about(o, th, d):
    """the SE(2) that turns by th about the point o, then shifts by d"""
    c, s = math.cos(th), math.sin(th); R = np.array([[c, -s], [s, c]]); T = np.eye(3); T[:2, :2] = R
    T[:2, 2] = np.asarray(o) - R @ np.asarray(o) + np.asarray(d); return T


def adjoint(T):
    """GTSAM Pose2's adjoint (tangent order x, y, theta): Exp(Ad_T xi) = T Exp(xi) T^-1"""
    c, s = T[0, 0], T[1, 0]; x, y = T[0, 2], T[1, 2]
    return np.array([[c, -s, y], [s, c, -x], [0, 0, 1.0]])


def info_between(I_world, at, G):
    """a match measured as a world-frame move (a left perturbation about the point `at`, information I_world in (x, y,
    turn)) re-expressed for a between factor whose measurement is Z = X_i^-1 M X_j, G = S_at^-1 M X_j: the error xi of
    Z = Ad(G^-1) delta, so its information is A^-T I A^-1 with A = Ad(G^-1)"""
    S = np.eye(3); S[:2, 2] = at
    A = adjoint(inv(inv(S) @ G)); Ai = np.linalg.inv(A)
    I = Ai.T @ I_world @ Ai; return 0.5 * (I + I.T)


# --- odometry ------------------------------------------------------------------------------------------------------

@dataclass
class Track:
    """a night's odometry (world <- base_link) over time, from the recording's transforms; with a planted error E(t)
    (world-left, 3x3) for the bench's positive controls"""
    t: np.ndarray
    xyh: np.ndarray
    E: object = None                                       # callable t -> 3x3, or None

    @classmethod
    def of(cls, db):
        import house_shape as hs
        from dimos.memory.store.sqlite import SqliteStore
        st = SqliteStore(path=db); st.start(); t, xyh = hs.poses(st); st.stop()
        xyh = xyh.copy(); xyh[:, 2] = np.unwrap(xyh[:, 2]); return cls(t, xyh)

    def clean(self, t):
        return se2(np.interp(t, self.t, self.xyh[:, 0]), np.interp(t, self.t, self.xyh[:, 1]), np.interp(t, self.t, self.xyh[:, 2]))

    def at(self, t):
        O = self.clean(t); return self.E(t) @ O if self.E is not None else O

    def travelled(self, t):
        d = np.r_[0, np.cumsum(np.hypot(*np.diff(self.xyh[:, :2], axis=0).T))]; return np.interp(t, self.t, d)


# --- DimOS's pose graph, with its loops kept -----------------------------------------------------------------------

def pgo_with_loops(dbs, settings, tmp, plants=None, bad_loop=None, posed_cache=None, verifier=None):
    """house_frame.py's run (DimOS's PGO over the nights' posed lidar as one stream), keeping what DimOS's public
    PoseGraph drops: each loop's measured offset and its keyframes' times. plants: {night: E(t)} planted in the
    odometry (the scan, registered by it, moved with it); bad_loop: (keyframe index at which to add it, target index,
    (dx, dy, dth) error on the true-looking offset) for the positive control. verifier: DimOS's loop_verifier (a
    callable on each proposed loop, loop_verify.LoopVerifier); the planted loop is put to it too. Returns (corrections {night: (times, Ts)},
    loops [(t_target, t_source, offset 3x3, score)], keyframe times, posed db paths)"""
    import house_frame as hf
    from dimos.mapping.loop_closure import pgo as P
    from dimos.memory.store.sqlite import SqliteStore
    from dimos.msgs.geometry_msgs.Transform import Transform
    paths = []
    for db in dbs:
        p = Path(posed_cache) / (Path(db).parent.name + "_posed.db") if posed_cache else None
        if p is None or not p.exists(): p = hf.posed(db, Path(posed_cache or tmp))
        paths.append(str(p))
    stores = [SqliteStore(path=p) for p in paths]
    for st in stores: st.start()
    obs = [[o for o in st.streams.lidar if o.pose_tuple is not None] for st in stores]

    class State(P._PGOState):
        def _search_for_loops(self):
            super()._search_for_loops()
            if bad_loop is not None and len(self._key_poses) - 1 == bad_loop[0]:   # beside any loop found there
                cur = len(self._key_poses) - 1; tgt = bad_loop[1]
                off = self._key_poses[tgt].optimized.between(self._key_poses[cur].optimized)
                e = bad_loop[2]; c, s = math.cos(e[2]), math.sin(e[2]); M = np.eye(4); M[:2, :2] = [[c, -s], [s, c]]; M[:2, 3] = e[:2]
                import gtsam
                pair = P._LoopPair(source=cur, target=tgt, offset=off.compose(gtsam.Pose3(M)), score=0.02)
                if verifier is not None and not verifier(self._candidate(pair, self._get_submap(cur, 0), self._get_submap(tgt, self._cfg.loop_submap_half_range))):
                    self._rejected_loops.append(pair); pgo_with_loops.planted_rejected = True; return
                self._pending_loops.append(pair)

    cfg = P.PGOConfig(**settings); st_ = State(cfg, loop_verifier=verifier); pgo_with_loops.planted_rejected = False
    for n, ob in enumerate(obs, 1):
        E = (plants or {}).get(n)
        for o in ob:
            pose = o.pose
            if pose.position.is_zero() or pose.orientation.is_zero(): continue
            p3 = P._obs_to_pose3(o); cloud = o.data
            if E is not None:
                M = np.eye(4); e = E(o.ts); M[:2, :2] = e[:2, :2]; M[:2, 3] = e[:2, 2]
                import gtsam
                p3 = gtsam.Pose3(M).compose(p3); cloud = cloud.transform(Transform.from_matrix(M))
            st_.process(p3, o.ts, cloud)
    graph = st_.snapshot()
    corr = {n: (np.array([o.ts for o in ob]), np.array([hf.se2_of(graph.correction_at(o.ts).to_matrix()) for o in ob]))
            for n, ob in enumerate(obs, 1)}
    loops = []
    for lp in st_._accepted_loops:
        a, b = st_._key_poses[lp.target], st_._key_poses[lp.source]
        loops.append((a.timestamp, b.timestamp, hf.se2_of(lp.offset.matrix()), float(lp.score)))
    kts = np.array([k.timestamp for k in st_._key_poses])
    pgo_with_loops.rejected = [(st_._key_poses[lp.target].timestamp, st_._key_poses[lp.source].timestamp, hf.se2_of(lp.offset.matrix()), float(lp.score))
                               for lp in st_._rejected_loops]
    for st in stores: st.stop()
    return corr, loops, kts, paths


def save_frame(out, corr, loops, kts, dbs):
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    for n, (t, T) in corr.items(): np.savez(out / f"corrections_{n}.npz", times=t, Ts=T, db=dbs[n - 1])
    np.savez(out / "loops.npz", t_target=np.array([l[0] for l in loops]), t_source=np.array([l[1] for l in loops]),
             offset=np.array([l[2] for l in loops]).reshape(-1, 3, 3), score=np.array([l[3] for l in loops]), keyframe_ts=kts)
    (out / "summary.json").write_text(json.dumps({"nights": len(corr), "keyframes": len(kts), "loop_closures": len(loops)}))


def load_loops(d):
    z = np.load(Path(d) / "loops.npz"); return [(a, b, O, s) for a, b, O, s in zip(z["t_target"], z["t_source"], z["offset"], z["score"])]


def localizer(d, n):
    from houndkeeper.localize import Localizer
    z = np.load(Path(d) / f"corrections_{n}.npz"); return Localizer(times=list(z["times"]), Ts=list(z["Ts"]))


# --- the graph -----------------------------------------------------------------------------------------------------

@dataclass
class Nodes:
    """the robot's poses the graph solves for: each night's times, merged and sorted"""
    night: np.ndarray
    t: np.ndarray
    index: dict = field(default_factory=dict)

    @classmethod
    def of(cls, times_by_night):
        night, t = [], []
        for n in sorted(times_by_night):
            u = np.unique(np.asarray(times_by_night[n], float)); u = u[np.r_[True, np.diff(u) > 1e-4]]; night += [n] * len(u); t += list(u)
        nd = cls(np.array(night), np.array(t))
        nd.index = {int(n): np.nonzero(nd.night == n)[0] for n in sorted(times_by_night)}   # each night's, in time order
        return nd

    def at(self, n, t, tol=1e-4):
        """the pose at night n's time t (KeyError if none within tol s)"""
        idx = self.index[int(n)]; k = int(np.searchsorted(self.t[idx], t))
        for c in (k - 1, k):
            if 0 <= c < len(idx) and abs(self.t[idx[c]] - t) <= tol: return int(idx[c])
        raise KeyError((n, t))

    def find(self, t):
        """the pose at time t in whichever night has it, or None"""
        for n in self.index:
            try: return self.at(n, t)
            except KeyError: pass
        return None


def keyframes_cached(db, n, cache):
    """house_shape's keyframes (night, t, x, y, heading, structure points in the odometry frame, travelled), read once"""
    import house_shape as hs
    p = Path(cache) / f"keyframes_{Path(db).parent.name}.npz" if cache else None
    if p is not None and p.exists():
        z = np.load(p, allow_pickle=False); st = z["start"]
        return [(n, *z["meta"][k][:4], z["pts"][st[k]:st[k + 1]], z["meta"][k][4]) for k in range(len(z["meta"]))]
    KF = hs.keyframes(db, n)
    if p is not None:
        p.parent.mkdir(parents=True, exist_ok=True)
        np.savez(p, meta=np.array([[k[1], k[2], k[3], k[4], k[6]] for k in KF]), pts=np.vstack([k[5] for k in KF]),
                 start=np.cumsum([0] + [len(k[5]) for k in KF]))
    return KF


@dataclass
class Problem:
    """everything the graph is built from: the nights' odometry, the lidar keyframes (scan points in their own body
    frame), the floor views (their rasters and the poses they were painted at), and DimOS's loops"""
    tracks: dict
    kf: list                     # [(night, t, body points (n, 2), travelled)]
    views: object = None         # floor_refine.load_views output (views, ts, night) or None
    painted: dict = None         # {night: Localizer} the views were painted with
    loops: list = field(default_factory=list)


def odometry_sd(d, dth):
    f = math.sqrt(min(1.0, max(d, 1e-3) / ODO_STEP_M))
    return ODO_SD[0] * f + ODO_SD[1] * d, ODO_SD[2] * f + ODO_SD[3] * d + ODO_TURN * abs(dth)


def lidar_factors(prob, nodes, X, workers, log):
    """house_shape's pairs and matcher at the current poses: [(i, j, Z, information)]"""
    import house_shape as hs, multiprocessing as mp
    K = prob.kf; N = len(K); idx = [nodes.at(k[0], k[1]) for k in K]
    pts = [k[2] @ X[i][:2, :2].T + X[i][:2, 2] for k, i in zip(K, idx)]
    P = np.array([X[i][:2, 2] for i in idx]); night = np.array([k[0] for k in K]); trav = np.array([k[3] for k in K])
    pairs = set()
    for i in range(N):
        d = np.hypot(*(P - P[i]).T); ok = d < hs.NEAR_M; ok[i] = False
        for j in np.nonzero(ok)[0][np.argsort(d[ok])][:hs.PER_KEY]: pairs.add((min(i, j), max(i, j)))
    hs._K = pts
    with mp.get_context("fork").Pool(workers) as pool:
        meas = [m for m in pool.map(hs.pair_match, sorted(pairs), chunksize=32) if m is not None]
    out = []; top = 1 / LIDAR_SD_FLOOR_M ** 2; topth = 1 / math.radians(LIDAR_SD_FLOOR_DEG) ** 2
    for i, j, T, I, rms, n in meas:
        dd = abs(trav[j] - trav[i]) if night[i] == night[j] else 50.0
        if not hs.drift_ok(T, P[j], dd): continue
        a, b = idx[i], idx[j]; Z = inv(X[a]) @ T @ X[b]
        Ib = info_between(I, np.zeros(2), T @ X[b])                # never surer than the floor ([ObC])
        s = min(1.0, top / max(np.linalg.eigvalsh(Ib[:2, :2]).max(), 1e-12), topth / max(Ib[2, 2], 1e-12))
        out.append((a, b, Z, Ib * s))
    log(f"  lidar: {len(pairs)} pairs, {len(meas)} matched, {len(out)} kept")
    return out


def moved_views(prob, nodes, X):
    """the views' rasters moved from where they were painted to where the poses now put them: a confirm_stream.Views,
    and each view's node"""
    import confirm_stream as cs, floor_refine as fr
    views, ts, night = prob.views; out_sls, out_p, C, F, vi = [], [], [], [], []
    z = prob.views_meta
    for k, (v, t, n) in enumerate(zip(views, ts, night)):
        i = nodes.at(n, t); Xp = prob.painted[n].correction(t) @ prob.tracks[n].clean(t)
        M = X[i] @ inv(Xp); o = fr.centre(v.box); d = M[:2, :2] @ o + M[:2, 2] - o; th = math.atan2(M[1, 0], M[0, 0])
        pl = v.placed(np.array([d[0], d[1], th]))
        out_sls.append(pl.box); out_p.append(pl.rgb); vi.append(i)
        C.append(M[:2, :2] @ z["C"][k] + M[:2, 2]); F.append(M[:2, :2] @ z["F"][k])
    return cs.Views(C, F, out_sls, out_p, ts), np.array(vi)


def directional(V, meas, log, nb=None):
    """[4t7]'s front end: each lock (stitch_floor.measure's) corrected by the directional floor model built from the
    views where the poses now put them (dirfloor.DirFloor: the floor seen from each direction registered on the floor
    seen from the others, as two numbers for the house, along and across the line of sight), and each lock's spread by
    its facing difference (DIR_SD_MODE: a fixed table, or the locks' own residuals: the views stand where the poses put
    them, so a lock's move is its residual; median / 1.177 is a 2-D Gaussian's sd), never surer than stitch_floor's
    weight. Returns (corrected
    meas, sd per lock, record)"""
    import dirfloor as dfl
    nb = nb or DIR_BINS; m = dfl.DirFloor.build(V, nb, grey_only=True); rd = m.radial(m.bias())
    D = rd[0] if rd is not None else np.zeros((nb, 2))
    out, ang = [], []
    for i, j, ox, oy, dx, dy, dth, sc in meas:
        rc = np.array([(dfl.HY - oy) / dfl.RES - 0.5]), np.array([(ox + dfl.HX) / dfl.RES - 0.5])
        bi, bj = int(dfl.azimuth_bin(*rc, V.C[i], nb)[0]), int(dfl.azimuth_bin(*rc, V.C[j], nb)[0])
        c = D[bi] - D[bj]; out.append((i, j, ox, oy, dx - c[0], dy - c[1], dth, sc))
        ang.append(math.degrees(math.acos(float(np.clip(V.F[i] @ V.F[j], -1, 1)))))
    ang = np.array(ang); r = np.array([math.hypot(x[4], x[5]) for x in out]); sd_band = {}
    for lo, hi in zip(DIR_BANDS[:-1], DIR_BANDS[1:]):
        s_ = (ang >= lo) & (ang < hi)
        if s_.sum() >= DIR_MIN_BAND: sd_band[(lo, hi)] = float(np.median(r[s_]) / 1.177)
    if DIR_SD_MODE == "table": sd_band = dict(DIR_SD_TABLE_M)
    wide = max(sd_band.values()) if sd_band else 0.1
    sd = np.array([max(next((v for (lo, hi), v in sd_band.items() if lo <= a_ < hi), wide), max(0.01 / x[7], FLOOR_SD_FLOOR_M)) for a_, x in zip(ang, out)])
    rec = {"along_across_mm": None if rd is None else [round(v * 1000, 1) for v in rd[1]],
           "sd_mm_by_band": {f"{int(lo)}-{int(hi)}": round(v * 1000, 1) for (lo, hi), v in sd_band.items()}}
    log(f"  floor, directional: along {rec['along_across_mm']} mm (along, across), sd by facing {rec['sd_mm_by_band']}")
    return out, sd, rec


def floor_factors(prob, nodes, X, workers, log, coarse_cells=3, max_turn_deg=FLOOR_MAX_TURN_DEG, front=None):
    """stitch_floor's pairs and snap over the views where the poses now put them: [(i, j, Z, information)]. front
    "dir" ([4t7]): every pair whatever the facing, corrected and weighed by the directional floor model (directional)"""
    import stitch_floor as sf, multiprocessing as mp
    front = front or FLOOR_FRONT
    if front == "dir": max_turn_deg = None
    V, vi = moved_views(prob, nodes, X); _v, ts, night = prob.views; N = len(vi)
    P = np.array([[-sf.cs.HX + ((s_[2] + s_[3]) / 2) * sf.cs.RES, sf.cs.HY - ((s_[0] + s_[1]) / 2) * sf.cs.RES] for s_ in V.sls])
    pairs = set()
    for i in range(N):                                        # stitch_floor.main's own pair rules
        d = np.hypot(*(P - P[i]).T); ok = (d < sf.NEAR_M) & ((night != night[i]) | (np.abs(ts - ts[i]) > sf.SAME_NIGHT_GAP_S)); ok[i] = False
        same_spot = (np.hypot(*(V.C - V.C[i]).T) < sf.SAME_SPOT_M) & (V.F @ V.F[i] > math.cos(math.radians(sf.SAME_SPOT_DEG)))
        ok &= ~(same_spot & (night == night[i]))
        if front == "dir":                                    # the like-facing pairs as before, and the nearest
            like = V.F @ V.F[i] > math.cos(math.radians(FLOOR_MAX_TURN_DEG))   # facing apart besides ([4t7])
            for sel, k_ in ((ok & like, sf.PER_VIEW), (ok & ~like, DIR_CROSS_PER_VIEW)):
                for j in np.nonzero(sel)[0][np.argsort(d[sel])][:k_]: pairs.add((min(i, j), max(i, j)))
            continue
        if max_turn_deg is not None: ok &= V.F @ V.F[i] > math.cos(math.radians(max_turn_deg))
        for j in np.nonzero(ok)[0][np.argsort(d[ok])][:sf.PER_VIEW]: pairs.add((min(i, j), max(i, j)))
    sf._V = V; sf.COARSE_CELLS = coarse_cells
    with mp.get_context("fork").Pool(workers) as pool:
        meas = [m for m in pool.map(sf.measure, sorted(pairs), chunksize=16) if m is not None]
    out = []; floor_factors.meta = []; floor_factors.views = (V, vi); floor_factors.record = None
    sds = [max(0.01 / x[7], FLOOR_SD_FLOOR_M) for x in meas]
    if front == "dir" and meas: meas, sds, floor_factors.record = directional(V, meas, log)
    for (i, j, ox, oy, dx, dy, dth, sc), sd in zip(meas, sds):
        o = np.array([ox, oy]); M = about(o, dth, (dx, dy)); a, b = vi[i], vi[j]
        if a == b: continue
        sd0 = max(0.01 / sc, FLOOR_SD_FLOOR_M)                # stitch_floor's weight; a cross-direction lock's turn
        I = np.diag([1 / sd ** 2, 1 / sd ** 2, (sc / 0.01) ** 2 * (sd0 / sd) ** 2])   # loosened as its shift is
        out.append((a, b, inv(X[a]) @ M @ X[b], info_between(I, o, inv(np.eye(3)) @ M @ X[b])))
        floor_factors.meta.append((i, j, ox, oy, dx, dy, dth, sc, math.degrees(math.acos(float(np.clip(V.F[i] @ V.F[j], -1, 1))))))
    log(f"  floor: {N} views, {len(pairs)} pairs, {len(meas)} locked")
    return out


def odometry_factors(prob, nodes):
    out = []
    for n in sorted(prob.tracks):
        idx = np.nonzero(nodes.night == n)[0]; tr = prob.tracks[n]; d = tr.travelled(nodes.t[idx])
        O = [tr.at(t) for t in nodes.t[idx]]
        for k in range(len(idx) - 1):
            Z = inv(O[k]) @ O[k + 1]; sxy, sth = odometry_sd(abs(d[k + 1] - d[k]), params(Z)[2])
            out.append((idx[k], idx[k + 1], Z, np.diag([1 / sxy ** 2, 1 / sxy ** 2, 1 / sth ** 2])))
    return out


def loop_factors(prob, nodes):
    out = []
    for ta, tb, Z, score in prob.loops:
        a, b = nodes.find(ta), nodes.find(tb)
        if a is None or b is None or a == b: continue
        tv = max(1e-4, float(score)); out.append((a, b, Z, np.diag([1 / tv, 1 / tv, 1 / LOOP_ROT_VAR])))
    return out


def solve(nodes, X0, families, anchor, log, max_iter=100):
    """one Levenberg-Marquardt solve: every family's between factors robust (Huber at its own k), night 1's first
    pose held, night 2's loosely. Returns (X, stats)"""
    import gtsam
    g = gtsam.NonlinearFactorGraph(); v = gtsam.Values()
    for i, T in enumerate(X0): v.insert(i, gtsam.Pose2(*params(T)))
    for name, fs in families.items():
        est = (gtsam.noiseModel.mEstimator.Cauchy if KERNEL.get(name) == "cauchy" else gtsam.noiseModel.mEstimator.Huber).Create(HUBER_K[name])
        for a, b, Z, I in fs:
            m = gtsam.noiseModel.Robust.Create(est, gtsam.noiseModel.Gaussian.Information(I))
            g.add(gtsam.BetweenFactorPose2(int(a), int(b), gtsam.Pose2(*params(Z)), m))
    for i, sd in anchor:
        g.add(gtsam.PriorFactorPose2(int(i), gtsam.Pose2(*params(X0[i])), gtsam.noiseModel.Diagonal.Sigmas(np.array([sd[0], sd[0], sd[1]]))))
    p = gtsam.LevenbergMarquardtParams(); p.setMaxIterations(max_iter); p.setRelativeErrorTol(1e-7)
    e0 = g.error(v); res = gtsam.LevenbergMarquardtOptimizer(g, v, p).optimize()
    X = [res.atPose2(i).matrix() for i in range(len(X0))]
    st = {"error_start": float(e0), "error_end": float(g.error(res))}
    for name, fs in families.items():                          # each family's whitened residual left, and outliers
        r = []
        for a, b, Z, I in fs:
            e = np.array(gtsam.Pose2.Logmap(gtsam.Pose2(*params(Z)).between(gtsam.Pose2(*params(inv(X[a]) @ X[b])))))
            r.append(math.sqrt(max(float(e @ I @ e), 0.0)))
        r = np.array(r) if r else np.zeros(1)
        st[name] = {"n": len(fs), "whitened_median": round(float(np.median(r)), 2), "beyond_k": int((r > HUBER_K[name]).sum())}
    log("  solve: " + "; ".join(f"{k} {v_['n']} (med {v_['whitened_median']} sd, {v_['beyond_k']} past k)" for k, v_ in st.items() if isinstance(v_, dict))
        + f"; error {e0:.3g} -> {st['error_end']:.3g}")
    return X, st


def run(prob, init, passes=PASSES, workers=4, floor=True, lidar=True, log=print, floor_passes=None):
    """the joint solve from the initial corrections init ({night: Localizer}): (nodes, X, history)"""
    times = {n: [] for n in prob.tracks}
    for k in prob.kf: times[k[0]].append(k[1])
    if prob.views is not None and floor:
        for t, n in zip(prob.views[1], prob.views[2]): times[int(n)].append(t)
    lt = {}
    for ta, tb, *_ in prob.loops:
        for t in (ta, tb):
            n = min(prob.tracks, key=lambda n: 0 if prob.tracks[n].t[0] - 1 <= t <= prob.tracks[n].t[-1] + 1 else 1)
            times[n].append(t)
    nodes = Nodes.of(times)
    X = [init[n].correction(t) @ prob.tracks[n].at(t) for n, t in zip(nodes.night, nodes.t)]
    first = [int(np.nonzero(nodes.night == n)[0][0]) for n in sorted(prob.tracks)]
    anchor = [(first[0], (1e-4, 1e-5))] + [(f, FIRST_N2_SD) for f in first[1:]]
    odo, loops = odometry_factors(prob, nodes), loop_factors(prob, nodes); hist = []
    fp = set(FLOOR_PASSES) if floor_passes is None else set(floor_passes)
    log(f"{len(nodes.t)} poses ({', '.join(f'night {n}: {(nodes.night == n).sum()}' for n in sorted(prob.tracks))}); "
        f"{len(prob.kf)} lidar keyframes, {0 if prob.views is None else len(prob.views[1])} views, {len(loops)} loops")
    fl = []
    for p in range(passes):
        t0 = time.time(); fam = {"odom": odo, "loop": loops}
        if lidar: fam["lidar"] = lidar_factors(prob, nodes, X, workers, log)
        if floor and prob.views is not None:
            if p in fp: fl = floor_factors(prob, nodes, X, workers, log)
            fam["floor"] = fl
        X2, st = solve(nodes, X, fam, anchor, log)
        mv = np.array([np.hypot(*(a[:2, 2] - b[:2, 2])) for a, b in zip(X, X2)]) * 1000
        st.update(pass_=p + 1, moved_mm_median=round(float(np.median(mv)), 2), moved_mm_max=round(float(mv.max()), 1), seconds=round(time.time() - t0, 1))
        log(f"pass {p + 1}: poses moved {np.median(mv):.1f} mm median, {mv.max():.0f} max ({time.time() - t0:.0f} s)")
        X = X2; hist.append(st)
    run.families = fam                                        # the last pass's constraints, for the bench's audit
    return nodes, X, hist


def corrections(prob, nodes, X):
    """each night's corrections (times, Ts world <- believed) at its poses"""
    out = {}
    for n in sorted(prob.tracks):
        idx = np.nonzero(nodes.night == n)[0]
        out[n] = (nodes.t[idx], np.array([X[i] @ inv(prob.tracks[n].at(nodes.t[i])) for i in idx]))
    return out


def load_problem(dbs, views=None, painted=None, frame=None, cache=None):
    import floor_refine as fr
    tracks = {n: Track.of(db) for n, db in enumerate(dbs, 1)}
    kf = []
    for n, db in enumerate(dbs, 1):
        for k in keyframes_cached(db, n, cache):
            O = tracks[n].clean(k[1]); kf.append((n, k[1], (k[5] - O[:2, 2]) @ O[:2, :2], k[6]))
    prob = Problem(tracks=tracks, kf=kf, loops=load_loops(frame) if frame and (Path(frame) / "loops.npz").exists() else [])
    if views: attach_views(prob, views, painted)
    return prob


def attach_views(prob, views, painted):
    """the floor views (baseline_views.py's) and the corrections they were painted with"""
    import floor_refine as fr
    if True:
        prob.views = fr.load_views(views)
        meta = {"C": [], "F": []}
        for p in views:
            z = np.load(p); meta["C"] += list(z["C"]); meta["F"] += list(z["F"])
        prob.views_meta = {k: np.array(v) for k, v in meta.items()}
        prob.painted = {n: localizer(painted, n) for n in (1, 2)}


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("cmd", choices=["frame", "solve"]); ap.add_argument("dbs", nargs=2)
    ap.add_argument("--out", required=True); ap.add_argument("--pgo", nargs="*", default=[])
    ap.add_argument("--frame", help="solve: the frame step's dir (DimOS's corrections, the start, and its loops.npz)")
    ap.add_argument("--views", nargs=2); ap.add_argument("--painted", help="the corrections the views were painted with (dir)")
    ap.add_argument("--passes", type=int, default=PASSES); ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--cache", default=None, help="where the recordings' keyframes and posed lidar are kept between runs")
    ap.add_argument("--no-floor", action="store_true"); ap.add_argument("--no-loops", action="store_true")
    a = ap.parse_args(argv); out = Path(a.out); out.mkdir(parents=True, exist_ok=True); t0 = time.time()
    if a.cmd == "frame":
        import house_frame as hf, tempfile
        settings = dict(hf.PGO_SETTINGS, **{k: float(v) for k, v in (x.split("=") for x in a.pgo)})
        with tempfile.TemporaryDirectory() as td:
            corr, loops, kts, _ = pgo_with_loops(a.dbs, settings, td, posed_cache=a.cache)
        save_frame(out, corr, loops, kts, a.dbs)
        print(f"frame: {len(kts)} keyframes, {len(loops)} loops, {time.time() - t0:.0f} s -> {out}"); return
    prob = load_problem(a.dbs, a.views, a.painted, a.frame, a.cache)
    if a.no_loops: prob.loops = []
    init = {n: localizer(a.frame, n) for n in (1, 2)}
    nodes, X, hist = run(prob, init, a.passes, a.workers, floor=not a.no_floor)
    for n, (t, T) in corrections(prob, nodes, X).items(): np.savez(out / f"corrections_{n}.npz", times=t, Ts=T)
    (out / "summary.json").write_text(json.dumps({"poses": len(nodes.t), "passes": hist, "seconds": round(time.time() - t0, 1)}, indent=1))
    print(f"joint graph: {len(nodes.t)} poses, {time.time() - t0:.0f} s -> {out}/corrections_<n>.npz")


if __name__ == "__main__":
    main()
