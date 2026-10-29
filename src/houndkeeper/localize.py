"""Where the robot really is, from its lidar ([FHA]; the imperfections it answers are [nAH] 2Ak): the robot's odometry
drifts (a few cm and ~1 deg over a room, more over a night), so frames placed by it land in a different frame each
night and the floor comparison fails. Walls and furniture don't move: each lidar scan's structure (points 0.1-1.5 m up,
seen from above) is matched to the house's map, and the difference is the odometry's error at that moment.

A house's first night makes the map as it goes (each scan matched to the map so far, then added: its drift stops
accumulating); every later night matches its scans to that map. The matching is 2D point-to-line ICP, trimmed (the
worst-matched fifth ignored: chairs moved since, a bag), with a motion prior: each scan's correction stays near the
last's as far as odometry is trusted, so walls that pin only one direction (a corridor) don't let it slide along. A correction maps
the robot's believed world frame to the house frame, and frames between scans take it interpolated.
"""
import math
from dataclasses import dataclass, field

import numpy as np

Z_BAND = (0.10, 1.50)      # structure: above the floor and its noise, below the ceiling (m)
TRIM = 0.8                 # the best-matched share of points each ICP step uses
ICP_ITERS = 25
MAP_VOXEL = 0.05           # the map keeps one point per 5 cm cell


def se2(x, y, th):
    c, s = math.cos(th), math.sin(th); return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def se2_params(T):
    return float(T[0, 2]), float(T[1, 2]), math.atan2(T[1, 0], T[0, 0])


def level(points: np.ndarray) -> np.ndarray:
    """a scan's points with heights measured from the floor under them: the floor plane fitted to the scan's own low
    returns (trimmed least squares), so the body's pitch and roll while walking (~0.5 deg: 3 cm at 3 m, [nAH] JaG) don't
    lift the floor's far side into "something stands here" """
    P = np.asarray(points, float)
    if len(P) < 30: return P
    m = P[:, 2] < 0.12; A = np.c_[P[:, 0], P[:, 1], np.ones(len(P))]
    for _ in range(4):
        if m.sum() < 30: return P
        c, *_ = np.linalg.lstsq(A[m], P[m, 2], rcond=None); r = P[:, 2] - A @ c
        m = np.abs(r) < max(3 * 1.4826 * float(np.median(np.abs(r[m]))), 0.01)
    return np.c_[P[:, :2], r]


def structure(points: np.ndarray) -> np.ndarray:
    """a scan's walls and furniture, as 2D points"""
    p = np.asarray(points, float)
    return p[(p[:, 2] > Z_BAND[0]) & (p[:, 2] < Z_BAND[1]), :2]


LIDAR_SD = 0.02            # m: a return's range noise (params go2_air.lidar), how far off a wall a good match may lie
PRIOR_FIRST = (0.10, math.radians(3.0))    # how far the first scan's correction may be from odometry (m, rad): the start
PRIOR_STEP = (0.004, math.radians(0.08))   # and each next scan's from the last's, before any walking: the fit's own
STEP_PER_M = (0.03, math.radians(1.0))     # jitter; plus this per metre walked since (odometry drifts ~2 cm and ~0.5 deg a
                                           # metre, params odom_drift, [nAH] 2Ak, with room). A fixed 2 cm and 0.4 deg a scan
                                           # let the correction slide 0.2-0.9 m over a tidy night with good-looking fits,
                                           # most of its scans taken standing still at finds (27 Sep, night 70/1 round 2)
ABS_SD = (0.05, 0.04, math.radians(1.5), math.radians(0.6))   # and never far from odometry itself: sd of the whole
                           # correction, m + m per m walked, rad + rad per root m walked (odometry's drift, [nAH] 2Ak, with room).
                           # Without it the correction crept, scan by scan, to -65 deg and snapped to a wrong fit (27 Sep)
GOOD_FIT = (0.05, 100, 0.5)   # a scan whose kept residuals' rms is above 5 cm, or with under 100 kept, or matching under half
                              # of what it saw, doesn't move the correction (27 Sep, night 2: 172 of ~3,000 matched, the fit was
                              # taken, and the correction jumped 17 deg for 2 min)


def normals(points: np.ndarray, tree, k=8) -> np.ndarray:
    """each map point's wall direction's normal (2D PCA of its neighbours)"""
    _, j = tree.query(points, k=min(k, len(points)))
    nb = points[j] - points[j].mean(1, keepdims=True)
    C = np.einsum("nki,nkj->nij", nb, nb); w, v = np.linalg.eigh(C)
    return v[:, :, 0]                                                          # the least-spread direction: across the wall


def at(T, r):
    """a correction as the robot feels it: the shift it applies at the robot's position r, and its turn"""
    x, y, th = se2_params(T); c, s_ = math.cos(th), math.sin(th)
    return np.array([c * r[0] - s_ * r[1] + x - r[0], s_ * r[0] + c * r[1] + y - r[1], th])


def icp_prior(src, tree, dst, dst_n, T0, priors, r, max_d=0.5):
    """point-to-line ICP with a motion prior: the SE(2) T laying src onto the map (dst, its normals dst_n), pulled toward
    each prior [(T_prior, (sd m, sd rad))] as far as it's trusted, compared where it matters: the shift at the robot's
    position r and the turn (a turn about the map's origin, metres away, moves r a lot: priors on the raw translation
    fought every heading fix, 27 Sep). Where walls pin a direction, they decide it; where they don't (a corridor's
    length), the priors hold. Returns (T, rms of the kept residuals (m), residuals kept)."""
    T = T0.copy(); rms, n = float("inf"), 0; r = np.asarray(r, float)
    pw = [(at(Tp, r), 1.0 / np.array([sd[0], sd[0], sd[1]]) ** 2 * LIDAR_SD ** 2) for Tp, sd in priors]   # in residual units
    for it in range(ICP_ITERS):
        p = src @ T[:2, :2].T + T[:2, 2]
        d, j = tree.query(p, distance_upper_bound=max_d); ok = np.isfinite(d)
        if ok.sum() < 30: break
        keep = ok & (d <= np.quantile(d[ok], TRIM))
        a, q, nrm = p[keep], dst[j[keep]], dst_n[j[keep]]
        res = np.einsum("ij,ij->i", nrm, a - q)                                # distance to the wall, along its normal
        J = np.c_[nrm, nrm[:, 0] * -a[:, 1] + nrm[:, 1] * a[:, 0]]            # d res / d (tx, ty, small turn about the origin)
        cur = at(T, r); H = J.T @ J; g = J.T @ res
        cr = T[:2, :2] @ r + T[:2, 2]                                           # where the correction puts the robot now
        Jp = np.array([[1.0, 0, -cr[1]], [0, 1.0, cr[0]], [0, 0, 1.0]])         # d (shift at r, turn) / d (increment)
        for pr, wp in pw:
            dev = cur - pr; dev[2] = (dev[2] + math.pi) % (2 * math.pi) - math.pi
            H = H + Jp.T @ np.diag(wp) @ Jp; g = g + Jp.T @ (wp * dev)
        dx, dy, dth = -np.linalg.solve(H, g)
        T = se2(dx, dy, dth) @ T
        rms, n = float(np.sqrt(np.mean(res ** 2))), int(keep.sum())
        max_d = max(0.08, min(max_d, 4 * rms + 0.03))
        if abs(dth) < 1e-6 and math.hypot(dx, dy) < 1e-5: break
    return T, rms, n


def _voxel(p, res=MAP_VOXEL):
    k = np.floor(p / res).astype(np.int64); _, i = np.unique(k[:, 0] * 1000003 + k[:, 1], return_index=True)
    return p[i]


@dataclass
class Localizer:
    """corrections for one night's scans; `house` is the map to match (None: build it from this night)"""
    house: np.ndarray | None = None
    times: list = field(default_factory=list)
    Ts: list = field(default_factory=list)
    fits: list = field(default_factory=list)

    def run(self, scans, travelled=None, positions=None) -> "Localizer":
        """scans: [(t, points Nx3 in the believed world frame)] in time order; travelled: metres the robot had walked
        by each scan, by its odometry (None: 0.3 m/s since the first scan, est.); positions: the robot's believed (x, y)
        at each scan (None: the middle of what the scan saw)"""
        t0 = scans[0][0] if scans else 0.0
        for k, (t, pts) in enumerate(scans):
            self.step(t, pts, travelled[k] if travelled is not None else 0.3 * (t - t0),
                      positions[k] if positions is not None else None)
        return self.finish()

    def step(self, t, pts, travelled, position=None) -> np.ndarray:
        """one scan, as it arrives (live, or replayed by run): its correction, from this scan and the earlier ones only"""
        from scipy.spatial import cKDTree
        if not hasattr(self, "_m"):                                           # first call: the map to match, or none yet
            self._building = self.house is None; self._pending = []; self._first = True; self._T = np.eye(3)
            self._m = None if self._building else np.asarray(self.house)
            if self._m is not None: self._tree = cKDTree(self._m); self._mn = normals(self._m, self._tree)
        s = structure(pts)
        if len(s) < 50: return self._T
        if self._m is None:                                                   # the first scan founds the house frame
            self._m = _voxel(s); self._tree = cKDTree(self._m); self._mn = normals(self._m, self._tree)
            self.times.append(t); self.Ts.append(self._T.copy()); self.fits.append((0.0, len(s))); self._first = False; return self._T
        absd = (ABS_SD[0] + ABS_SD[1] * travelled, ABS_SD[2] + ABS_SD[3] * math.sqrt(travelled))
        dd = max(travelled - getattr(self, "_trav", travelled), 0.0); self._trav = travelled
        step = (PRIOR_STEP[0] + STEP_PER_M[0] * dd, PRIOR_STEP[1] + STEP_PER_M[1] * dd)
        rpos = position if position is not None else s.mean(0)
        T = self._T
        Tn, rms, n = icp_prior(s, self._tree, self._m, self._mn, T, [(T, PRIOR_FIRST if self._first else step), (np.eye(3), absd)], rpos)
        self._first = False
        if rms <= GOOD_FIT[0] and n >= GOOD_FIT[1] and n >= GOOD_FIT[2] * len(s): T = Tn   # a poor fit leaves it where it was
        self._T = T; self.times.append(t); self.Ts.append(T.copy()); self.fits.append((rms, n))
        if self._building:                                                    # grow the map with what's new, every few scans
            self._pending.append(s @ T[:2, :2].T + T[:2, 2])
            if len(self._pending) >= 4:
                self._m = _voxel(np.vstack([self._m] + self._pending)); self._tree = cKDTree(self._m)
                self._mn = normals(self._m, self._tree); self._pending = []
        return T

    def finish(self) -> "Localizer":
        """the map as built (a first night): what later nights match"""
        if getattr(self, "_building", False):
            self.house = _voxel(np.vstack([self._m] + self._pending)) if self._pending else self._m
        return self

    def correction(self, t: float) -> np.ndarray:
        """the 3x3 SE(2) correction at time t (interpolated between scans; held at the ends)"""
        ts = self.times
        if not ts: return np.eye(3)
        i = int(np.searchsorted(ts, t))
        if i <= 0: return self.Ts[0]
        if i >= len(ts): return self.Ts[-1]
        f = (t - ts[i - 1]) / (ts[i] - ts[i - 1]); a, b = se2_params(self.Ts[i - 1]), se2_params(self.Ts[i])
        dth = (b[2] - a[2] + math.pi) % (2 * math.pi) - math.pi
        return se2(a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), a[2] + f * dth)

    def apply(self, T_world: np.ndarray, t: float) -> np.ndarray:
        """a 4x4 pose in the believed world frame, in the house frame"""
        C = self.correction(t); M = np.eye(4); M[:2, :2] = C[:2, :2]; M[:2, 3] = C[:2, 2]
        return M @ T_world


def real_scan(n_points: int, t: float, t_last: float | None) -> bool:
    """a lidar message that is one sweep of the sensor: not the sim harness's injected house map (inject_map.py: the
    earlier nights' whole map, ~24,000 points, stamped out of order, on the same stream; one sweep is ~4,500-6,200),
    which the matcher took for a scan and was knocked 40 -> 115 mm (27 Sep)"""
    return n_points <= MAX_SCAN_POINTS and (t_last is None or t > t_last)


MAX_SCAN_POINTS = 12000     # twice the L1's sweep (params lidar: 21,600 points a second, ~2.5 sweeps a second in the sim)


def scans_from(st) -> list:
    """a recording's lidar scans: [(t, Nx3)], real sweeps only (real_scan)"""
    out = []
    for o in st.stream("lidar"):
        p = o.data.as_numpy(); p = np.asarray(p[0] if isinstance(p, tuple) else p)
        if real_scan(len(p), o.ts, out[-1][0] if out else None): out.append((o.ts, p))
    return out
