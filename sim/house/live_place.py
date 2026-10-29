#!/usr/bin/env python3
"""Tonight's frames placed live by the house's pose graph ([x9X]; [JzW]). Today the confirmer places each frame by
odometry and the floor tracking's running correction (confirm_stream --track: each floor lock carried into the pose
as a gain); here the same house graph that made the floor picture places tonight, one keyframe at a time:

  - the baseline nights' keyframes held as a fixed map ([JzW]'s nodes after the house's solve: house_shape's
    keyframes, each where the house's shape solve put it, with its own lidar scan); tonight is matched to it and never
    moves it;
  - tonight's poses are nodes added as the night runs (iSAM2, one update per keyframe or floor lock), at each lidar
    keyframe (house_shape's rule, every 25 cm or 5 deg) and at each frame whose floor locks;
  - odometry between consecutive nodes (joint_graph's noise: ~3 cm and ~1 deg a metre);
  - each lidar keyframe matched to the house's keyframes within 3 m (house_shape's point-to-line matcher, its
    information J'J, never surer than 3 mm, [ObC]), refused when it asks for more than odometry could have drifted since
    the last match (house_shape's drift bound): an absolute constraint on that node;
  - each floor lock of the confirmer's (its snap of the earlier nights' views onto tonight's frame, the cells it has
    found changed left out, so a toy is never a constraint) the same, at the lock's spread;
  - every constraint robust (Huber).

A frame's pose is read from the estimate at its own time: the newest node's estimate carried forward by odometry
(`correction()`, house <- believed). Nothing here reads the sim's truth; only the robot's recordings and the house's
own map go in ([1B2]).

Usage: live_place.py build <run dir> --out <house_keyframes.npz> [--cache <dir>]     the fixed map, from the run's
                                                                                     floor_frame.json (its nights and
                                                                                     the shape solve's corrections)
       live_place.py run <tonight.db> --keyframes <npz> --out <placed.npz>           lidar and odometry alone, causal
(dimos venv python)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import argparse, collections, hashlib, json, math, threading
from pathlib import Path

import numpy as np

import house_shape as hs
import joint_graph as jg
from houndkeeper.localize import structure, level, _voxel, real_scan, scans_from

KEY_M, KEY_DEG, STACK = hs.KEY_M, hs.KEY_DEG, hs.STACK   # a keyframe each 25 cm or 5 deg; its scan with the two before
NEAR_M, PER_KEY = hs.NEAR_M, hs.PER_KEY                  # (causal: house_shape stacks the two after) matched to the
                                                         # house's keyframes within 3 m, the nearest 12
SUBMAP_VOXEL = 0.03                                      # m: the house keyframes' scans merged, one point per 3 cm
FIRST_SD = (0.10, math.radians(3.0))                     # the first node: tonight starts where the baselines started
                                                         # (the dock; localize.PRIOR_FIRST), this loosely
FLOOR_SD_M, FLOOR_SD_DEG = 0.015, 0.5                    # a floor lock: joint_graph's like-facing floor spread (the
                                                         # confirmer locks on views within 25 deg, [WFI]); its turn a
                                                         # 0.5 deg step of the snap's search
HUBER_K = {"lidar": 1.345, "floor": 1.345, "odom": 3.0}  # in sds of each constraint's own noise (joint_graph's)
RELIN = 0.01                                             # iSAM2's relinearization threshold


# --- the house's keyframes, a fixed map -----------------------------------------------------------------------------

class HouseKeyframes:
    """the baseline nights' lidar keyframes where the house's solve put them: t, night, X (N x 3: x, y, heading, the
    house's frame), each scan's structure in its own body frame; optionally each keyframe's marginal sd (sd_xy m,
    sd_th rad) from the house's solve, folded into a match's noise"""

    def __init__(self, t, night, X, pts, start, sd_xy=None, sd_th=None, meta=None):
        from scipy.spatial import cKDTree
        self.t, self.night, self.X = np.asarray(t, float), np.asarray(night, int), np.asarray(X, float)
        self.pts, self.start = np.asarray(pts, np.float32), np.asarray(start, int)
        self.sd_xy = None if sd_xy is None else np.asarray(sd_xy, float)
        self.sd_th = None if sd_th is None else np.asarray(sd_th, float)
        self.meta = meta or {}; self.tree = cKDTree(self.X[:, :2])

    def __len__(self): return len(self.t)

    @classmethod
    def load(cls, path):
        z = np.load(path, allow_pickle=False)
        return cls(z["t"], z["night"], z["X"], z["pts"], z["start"], z["sd_xy"] if "sd_xy" in z.files else None,
                   z["sd_th"] if "sd_th" in z.files else None, json.loads(str(z["meta"])) if "meta" in z.files else {})

    def save(self, path):
        extra = {} if self.sd_xy is None else {"sd_xy": self.sd_xy, "sd_th": self.sd_th}
        np.savez_compressed(path, t=self.t, night=self.night, X=self.X, pts=self.pts, start=self.start,
                            meta=np.str_(json.dumps(self.meta)), **extra)

    def body(self, i): return self.pts[self.start[i]:self.start[i + 1]].astype(float)

    def world(self, i):
        x, y, th = self.X[i]; c, s = math.cos(th), math.sin(th)
        return self.body(i) @ np.array([[c, s], [-s, c]]) + np.array([x, y])

    def near(self, xy, r=NEAR_M, k=PER_KEY):
        """the nearest k keyframes within r of xy"""
        d, i = self.tree.query(np.asarray(xy, float), k=min(k, len(self)), distance_upper_bound=r)
        d, i = np.atleast_1d(d), np.atleast_1d(i); return i[np.isfinite(d)]

    def submap(self, idx):
        return _voxel(np.vstack([self.world(i) for i in idx]), SUBMAP_VOXEL)


def build(run, out, cache=None):
    """the fixed map from a run directory: its nights' keyframes (house_shape's) placed by the house's shape solve
    (floor_frame.json names both; the floor pictures and views are painted in that frame)"""
    from houndkeeper.localize import Localizer
    run = Path(run); ff = json.loads((run / "floor_frame.json").read_text())
    ff["corrections"] = [str(run / Path(c).parent.name / Path(c).name) if (run / Path(c).parent.name / Path(c).name).exists() else c
                         for c in ff["corrections"]]           # a copied run directory: its own solve, not the original's
    t, night, X, pts = [], [], [], []
    for n, (db, corr) in enumerate(zip(ff["dbs"], ff["corrections"]), 1):
        z = np.load(corr); L = Localizer(times=list(z["times"]), Ts=list(z["Ts"]))
        for k in jg.keyframes_cached(db, n, cache):
            O = jg.se2(k[2], k[3], k[4]); Xk = L.correction(k[1]) @ O
            t.append(k[1]); night.append(n); X.append(jg.params(Xk)); pts.append((k[5] - O[:2, 2]) @ O[:2, :2])
    sha = {Path(c).name + f"@{Path(c).parent.name}": hashlib.sha256(Path(c).read_bytes()).hexdigest()[:12] for c in ff["corrections"]}
    hk = HouseKeyframes(t, night, X, np.vstack(pts), np.cumsum([0] + [len(p) for p in pts]),
                        meta={"run": str(run), "dbs": ff["dbs"], "corrections": ff["corrections"], "corrections_sha": sha})
    hk.save(out); return hk


# --- the placer -----------------------------------------------------------------------------------------------------

def _pose2(T):
    import gtsam
    return gtsam.Pose2(*jg.params(T))


class Placer:
    """tonight's poses in the house's frame, one node at a time (iSAM2). Feed it in time order: scan(t, points, O) for
    each lidar sweep (points in odometry's world frame, O odometry's world <- base at t, 3x3), lock(t, O, S, at, C_used)
    for each floor lock; correction() is house <- believed from the newest estimate"""

    def __init__(self, house, start=None, lidar=True, floor=True, huber=None):
        import gtsam
        self.house, self.lidar, self.floor = house, lidar, floor
        self.k = dict(HUBER_K, **(huber or {}))
        p = gtsam.ISAM2Params(); p.setRelinearizeThreshold(RELIN); p.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(p)
        self.start = np.eye(3) if start is None else np.asarray(start, float)
        self.C = self.start.copy()                               # house <- believed, from the newest node's estimate
        self.nodes: list = []                                    # (t, O odometry 3x3, kind)
        self._recent = collections.deque(maxlen=STACK)           # the last scans' structure (odometry's world frame)
        self._kf = None                                          # the last lidar keyframe's odometry pose
        self._trav, self._O_last_scan, self._fix_trav = 0.0, None, 0.0   # metres walked; at the last accepted fix
        self.factors: list = []                                  # (kind, t, Z (x, y, th), accepted, why)

    # each node and its odometry ----------------------------------------------------------------------------------
    def _add_node(self, t, O, kind):
        import gtsam
        g, v = gtsam.NonlinearFactorGraph(), gtsam.Values(); key = len(self.nodes)
        if not self.nodes:
            g.add(gtsam.PriorFactorPose2(0, _pose2(self.start @ O), gtsam.noiseModel.Diagonal.Sigmas(
                np.array([FIRST_SD[0], FIRST_SD[0], FIRST_SD[1]]))))
        else:
            t0, O0, _ = self.nodes[-1]; Z = jg.inv(O0) @ O; d = float(np.hypot(*Z[:2, 2]))
            sxy, sth = jg.odometry_sd(d, jg.params(Z)[2])
            m = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(self.k["odom"]),
                                               gtsam.noiseModel.Diagonal.Sigmas(np.array([sxy, sxy, sth])))
            g.add(gtsam.BetweenFactorPose2(key - 1, key, _pose2(Z), m))
        v.insert(key, _pose2(self.C @ O)); self.nodes.append((float(t), O.copy(), kind))
        return key, g, v

    def _absolute(self, g, key, Z, I, kind):
        import gtsam
        m = gtsam.noiseModel.Robust.Create(gtsam.noiseModel.mEstimator.Huber.Create(self.k[kind]), gtsam.noiseModel.Gaussian.Information(I))
        g.add(gtsam.PriorFactorPose2(int(key), _pose2(Z), m))

    def _update(self, g, v):
        self.isam.update(g, v)
        key = len(self.nodes) - 1; X = self.isam.calculateEstimatePose2(key).matrix()
        self.C = X @ jg.inv(self.nodes[key][1])

    # lidar --------------------------------------------------------------------------------------------------------
    def scan(self, t, pts, O):
        """one lidar sweep: a keyframe when the robot has moved 25 cm or turned 5 deg since the last (by odometry), then
        matched to the house. Returns True when it made a keyframe"""
        O = np.asarray(O, float)
        if self._O_last_scan is not None: self._trav += float(np.hypot(*(O[:2, 2] - self._O_last_scan[:2, 2])))
        self._O_last_scan = O
        s = structure(level(pts))
        if len(s) >= 50: self._recent.append(s)
        if self._kf is not None:
            D = jg.inv(self._kf) @ O
            if math.hypot(*D[:2, 2]) < KEY_M and abs(jg.params(D)[2]) < math.radians(KEY_DEG): return False
        P = np.vstack(list(self._recent)) if self._recent else np.zeros((0, 2))
        if len(P) < hs.MIN_PTS: return False
        self._kf = O.copy(); key, g, v = self._add_node(t, O, "lidar")
        if self.lidar: self._match(g, key, t, O, (P - O[:2, 2]) @ O[:2, :2])
        self._update(g, v); return True

    def _match(self, g, key, t, O, body):
        X = self.C @ O; P = X[:2, 2]; idx = self.house.near(P)
        if not len(idx): self.factors.append(("lidar", t, None, False, "no house keyframe near")); return
        src = _voxel(body @ X[:2, :2].T + P, SUBMAP_VOXEL)
        r = hs.icp_info(src, self.house.submap(idx), np.eye(3))
        if r is None: self.factors.append(("lidar", t, None, False, "too little wall")); return
        T, I, rms, n = r; Z = T @ X
        if rms > hs.MAX_RMS: self.factors.append(("lidar", t, jg.params(Z), False, f"rms {rms * 1000:.0f} mm")); return
        if not hs.drift_ok(T, P, self._trav - self._fix_trav):                # more than odometry could have drifted
            self.factors.append(("lidar", t, jg.params(Z), False, "past the drift bound")); return
        Ib = jg.info_between(I, np.zeros(2), Z)
        s = min(1.0, 1 / jg.LIDAR_SD_FLOOR_M ** 2 / max(np.linalg.eigvalsh(Ib[:2, :2]).max(), 1e-12),
                1 / math.radians(jg.LIDAR_SD_FLOOR_DEG) ** 2 / max(Ib[2, 2], 1e-12)); Ib = Ib * s   # never surer ([ObC])
        if self.house.sd_xy is not None:                                     # the house's own doubt about those keyframes
            m_xy, m_th = float(np.median(self.house.sd_xy[idx])), float(np.median(self.house.sd_th[idx]))
            Ib = np.linalg.inv(np.linalg.inv(Ib) + np.diag([m_xy ** 2, m_xy ** 2, m_th ** 2]))
        self._absolute(g, key, Z, 0.5 * (Ib + Ib.T), "lidar"); self._fix_trav = self._trav
        self.factors.append(("lidar", t, jg.params(Z), True, f"rms {rms * 1000:.0f} mm, {n} points"))

    # the floor ----------------------------------------------------------------------------------------------------
    def lock(self, t, O, S, at, C_used):
        """a floor lock of the frame at time t (odometry O there), placed with C_used (house <- believed): the
        confirmer's snap says the frame's floor sits where S (house-frame correction, turned about `at`) moves it, so
        the robot was at S C_used O. A node at t (or, when t is before the newest node, the node before it, carried by
        odometry)"""
        if not self.floor: return
        O = np.asarray(O, float); Z = np.asarray(S, float) @ np.asarray(C_used, float) @ O
        I = np.diag([1 / FLOOR_SD_M ** 2, 1 / FLOOR_SD_M ** 2, 1 / math.radians(FLOOR_SD_DEG) ** 2])
        if not self.nodes or t >= self.nodes[-1][0]:
            key, g, v = self._add_node(t, O, "floor")
        else:                                                    # a frame from before the newest node: the node before it
            import gtsam
            j = max(int(np.searchsorted([n[0] for n in self.nodes], t, side="right")) - 1, 0)
            D = jg.inv(self.nodes[j][1]) @ O; Z = Z @ jg.inv(D)  # the robot at node j, as the lock and odometry say
            sxy, sth = jg.odometry_sd(float(np.hypot(*D[:2, 2])), jg.params(D)[2])
            I = np.diag([1 / (FLOOR_SD_M ** 2 + sxy ** 2)] * 2 + [1 / (math.radians(FLOOR_SD_DEG) ** 2 + sth ** 2)])
            key, g, v = j, gtsam.NonlinearFactorGraph(), gtsam.Values()
        self._absolute(g, key, Z, jg.info_between(I, np.asarray(at, float), Z), "floor")
        self.factors.append(("floor", t, jg.params(Z), True, "")); self._update(g, v)

    def correction(self, t=None):
        """house <- believed from the newest estimate (the frame at t is placed from what was known then)"""
        return self.C.copy()


# --- feeds ------------------------------------------------------------------------------------------------------------

class Feed:
    """a recording replayed into a Placer in time order (confirm_stream's replay, `run` below): until(t) feeds every
    lidar sweep stamped up to t. pose(t): odometry's world <- base (4x4) as the replay reads it; scan_hook(t, points):
    an offline ablation's (a planted odometry error moves the scans with it), never set by the robot. Keeps each
    sweep's compute time (s, the host's: what the robot's computer would need)"""

    def __init__(self, st, placer, pose, scan_hook=None):
        import time
        self._clock = time.thread_time                          # this thread's CPU time on the host (what the robot's
                                                                # computer would need), not the robot's clock; the wall
                                                                # time swings with the machine's other load
        self.scans, self.placer, self.pose, self.hook, self.i = scans_from(st), placer, pose, scan_hook, 0
        self.seconds: list = []                                 # (t, keyframe?, compute s)

    def odom(self, t):
        M = self.pose(t)
        if M is None: return None
        th = math.atan2(M[1, 0], M[0, 0]); return jg.se2(M[0, 3], M[1, 3], th)

    def until(self, t):
        while self.i < len(self.scans) and self.scans[self.i][0] <= t:
            ts, p = self.scans[self.i]; self.i += 1; O = self.odom(ts)
            if O is None: continue
            if self.hook is not None: p = self.hook(ts, p)
            c0 = self._clock(); kf = self.placer.scan(ts, p, O); self.seconds.append((ts, kf, self._clock() - c0))

    def lock_timed(self, *a):
        c0 = self._clock(); self.placer.lock(*a); self.seconds.append((a[0], "floor", self._clock() - c0))


class LivePlacer:
    """the Placer fed live (the tidy, [x9X]): odometry kept as a short history, each lidar sweep and floor lock queued
    in arrival order and taken in a worker thread (a match takes tens of ms); correction() the newest estimate. Same
    seam as live_localize.LiveLocalizer (odom, scan, correction), plus the confirmer's floor locks (frame_context,
    on_lock)"""

    HISTORY = 400                                               # odometry messages kept (10-30 Hz: 15-40 s)

    def __init__(self, house, start=None):
        self.placer = Placer(house, start); self._odom = collections.deque(maxlen=self.HISTORY)
        self._q: collections.deque = collections.deque(); self._cv = threading.Condition(); self._stop = False
        self._lock = threading.Lock(); self._ctx = None; self._t_last = None
        threading.Thread(target=self._run, daemon=True).start()

    def odom(self, xy, yaw=None, t=None):
        if yaw is None or t is None: return                     # the placer needs the heading and the time
        self._odom.append((float(t), float(xy[0]), float(xy[1]), float(yaw)))

    def odom_at(self, t):
        h = list(self._odom)
        if not h or t < h[0][0] - 0.2 or t > h[-1][0] + 0.2: return None
        T = np.array(h); yaw = np.unwrap(T[:, 3])
        return jg.se2(np.interp(t, T[:, 0], T[:, 1]), np.interp(t, T[:, 0], T[:, 2]), float(np.interp(t, T[:, 0], yaw)))

    def scan(self, t, pts):
        if not real_scan(len(pts), t, self._t_last): return
        self._t_last = t; O = self.odom_at(t)
        if O is None: return
        with self._cv:
            while len(self._q) > 4 and self._q[0][0] == "scan": self._q.popleft()   # behind: the oldest sweeps dropped
            self._q.append(("scan", t, np.asarray(pts), O)); self._cv.notify()

    def frame_context(self, t, C_used):
        """the frame the confirmer is about to judge: its time and the correction it is placed with"""
        self._ctx = (t, np.asarray(C_used, float).copy())

    def on_lock(self, S, at, score=None, lead=None):
        """the confirmer's floor lock on the frame of frame_context"""
        if self._ctx is None: return
        t, C = self._ctx; O = self.odom_at(t)
        if O is None: return
        with self._cv: self._q.append(("lock", t, O, np.asarray(S, float), np.asarray(at, float), C)); self._cv.notify()

    def close(self):
        with self._cv: self._stop = True; self._cv.notify()

    def _run(self):
        while True:
            with self._cv:
                while not self._q and not self._stop: self._cv.wait()
                if self._stop: return
                job = self._q.popleft()
            with self._lock:
                if job[0] == "scan": self.placer.scan(job[1], job[2], job[3])
                else: self.placer.lock(job[1], job[2], job[3], job[4], job[5])

    def correction(self, t=None):
        with self._lock: return self.placer.correction(t)


# --- command line ---------------------------------------------------------------------------------------------------

def run(db, keyframes, out=None, floor=False):
    """lidar and odometry alone over a recording, causal: each sweep's correction as it stood once that sweep was in"""
    from dimos.memory.store.sqlite import SqliteStore
    from houndkeeper.rec_poses import RecordedPoses
    st = SqliteStore(path=db); st.start(); pose = RecordedPoses(st)
    pl = Placer(HouseKeyframes.load(keyframes), floor=floor); fd = Feed(st, pl, pose); times, Ts = [], []
    for ts, _p in list(fd.scans):
        fd.until(ts); times.append(ts); Ts.append(pl.correction())
    st.stop()
    if out:
        np.savez(out, times=np.array(times), Ts=np.array(Ts), seconds=np.array([(a, float(b == True), c) for a, b, c in fd.seconds]),
                 nodes=np.array([n[0] for n in pl.nodes]))
    return pl, fd, np.array(times), np.array(Ts)


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("cmd", choices=["build", "run"]); ap.add_argument("src")
    ap.add_argument("--out", required=True); ap.add_argument("--cache", default=None, help="build: the recordings' keyframes kept between runs")
    ap.add_argument("--keyframes", help="run: the house's keyframes (build's)")
    a = ap.parse_args(argv)
    if a.cmd == "build":
        hk = build(a.src, a.out, a.cache); print(f"house keyframes: {len(hk)} from {a.src} -> {a.out}"); return
    pl, fd, _t, _T = run(a.src, a.keyframes, a.out)
    kf = [s for _t, k, s in fd.seconds if k is True]; acc = sum(1 for f in pl.factors if f[3])
    print(f"placed: {len(pl.nodes)} nodes, lidar matches {acc}/{len(pl.factors)} taken; "
          f"{np.median(kf) * 1000 if kf else 0:.0f} ms a keyframe (median), {np.percentile(kf, 95) * 1000 if kf else 0:.0f} p95 -> {a.out}")


if __name__ == "__main__":
    main()
