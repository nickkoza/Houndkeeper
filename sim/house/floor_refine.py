#!/usr/bin/env python3
"""Each view registered against the floor, not only against other views ([G9q]): a feature
can exist at one place only, and the registration finds that place. An experiment beside the stitch, offline: it
reads what the stitch made (both baseline nights' views painted in the house's frame, baseline_views.py, and the
corrections they were painted with, house_shape.py) and the recordings, and writes refined corrections and pictures to
a directory of its own. Each pass:
  - every view is laid where the corrections so far put it, and the floor picture built from all of them;
  - each view is registered (floor_register.ncc_surfaces, edge correlation) against that picture built without it
    and without its own night's views from the same few seconds, which carry its own error (decision [ObC]);
  - each view may move only as far as the bound allows (MOVE_BOUND_M, the search shrinking as a view uses it up), so
    on a repeating floor no view can jump a whole tile;
  - one solve places the keyframes and the views: the lidar's matches between keyframes (house_shape's, measured once
    at the shape as solved) keep the house's shape; odometry ties each view to its keyframes; the locks pull the views;
  - the two nights' pictures are rebuilt from the views as moved, and the pass is kept only if they agree better by
    the detector's own test (the nights' disagreement, no truth), refining stopping at the first that doesn't.
Reads only the robot's own recordings and what the stitch derived from them ([1B2]).
Usage: floor_refine.py --views views_n1.npz views_n2.npz --dbs n1.db n2.db --init <dir with corrections_<n>.npz>
       --occ mosaic_n1.npz mosaic_n2.npz --out <scratch dir> [--passes 4]   (dimos venv)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import argparse, json, math, time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy import ndimage as ndi
from scipy.sparse import coo_matrix

from detect_changes2 import feats, tolerant_diff, RES, HX, HY
from houndkeeper import floor_register as freg

W, H = int(2 * HX / RES), int(2 * HY / RES)
SEARCH_CELLS = 6                  # a pass's search, cm either way (1 cm cells): the lidar places views to 2-3 cm
TURNS_DEG = (-0.4, -0.2, 0.0, 0.2, 0.4)
MOVE_BOUND_M = 0.08               # decision [ObC]: no view ever moves further from where the lidar put it: the lidar's
TURN_BOUND_DEG = 1.0              # tail (floor_register.SNAP_MAX_CELLS), well inside half a 30 cm tile
LEAVE_S = 5.0                     # the view's own night's views this close in time leave with it (stitch_floor's gap)
MIN_SCORE, MIN_LEAD = freg.SNAP_MIN_SCORE, 0.0   # a lock: a good match inside the search; how sure it is, from its
TAU = 0.05                        # whole surface (surface_belief at this softmax temperature), not a lead over the next
LOCK_SD_FLOOR_M, LOCK_SD_FLOOR_DEG = 0.002, 0.05   # best: at 1 cm a peak is cells broad (house 70, 4 Oct: median score
                                  # 0.68, lead 0.02 two cells away; 1 in 80 views passed the snap's 0.08). Never surer than this
TIE_SD_M, TIE_SD_DEG = 0.004, 0.1     # a view against its keyframes: the shape solve's own disagreement (3.8 mm median,
SEQ_SD = (0.005, 0.03, math.radians(0.1), math.radians(1.0))   # house 90) plus odometry over the way between (house_shape)
LIDAR_SD_FLOOR_M = 0.003          # a lidar match is never surer than this (its information J'J/sd^2 over thousands of
LOCK_F_SCALE = 2.0                # points says 0.3 mm; the solve's own matches disagree by ~4 mm). A lock is robust past
                                  # this many of its own sd (soft_l1); every other row stays quadratic
MIN_GAIN = 0.02                   # a pass must cut the nights' disagreement by this fraction of itself


def cells(m):
    """metres as whole 1 cm cells, rounded down"""
    return int(math.floor(m / RES + 1e-9))


def reach_cells(y):
    """how far a view moved by y (dx, dy, dth) may still be searched: what is left of the bound, at most a pass's search"""
    return max(0, min(SEARCH_CELLS, cells(MOVE_BOUND_M - math.hypot(y[0], y[1]))))


def centre(box):
    """the world (x, y) of a box's centre (rows r0:r1, cols c0:c1 on the floor grid)"""
    return np.array([-HX + (box[2] + box[3]) / 2 * RES, HY - (box[0] + box[1]) / 2 * RES])


def se2_about(y, o):
    """the SE(2) (3x3) that turns by y[2] about o, then shifts by y[:2]"""
    c, s = math.cos(y[2]), math.sin(y[2]); R = np.array([[c, -s], [s, c]])
    T = np.eye(3); T[:2, :2] = R; T[:2, 2] = o - R @ o + np.asarray(y[:2]); return T


def compose(m, y):
    """the move m (about the same point as y) after y: one move"""
    c, s = math.cos(m[2]), math.sin(m[2]); R = np.array([[c, -s], [s, c]])
    t = R @ np.asarray(y[:2]) + np.asarray(m[:2]); return np.array([t[0], t[1], y[2] + m[2]])


@dataclass
class Placed:
    """a view as laid on the floor grid: its box, its colour and where it has floor, and where it was moved from"""
    box: tuple
    rgb: np.ndarray
    mask: np.ndarray
    o: np.ndarray

    def grey(self):
        return self.rgb.astype(np.float32).mean(-1)


@dataclass
class View:
    """one view of the floor (baseline_views.py): its colour over its box, its time and night"""
    rgb: np.ndarray
    box: tuple
    t: float
    night: int

    def placed(self, y=None):
        """the view moved by y (dx, dy, dth about its box's centre): resampled onto the grid"""
        o = centre(self.box); r0, r1, c0, c1 = (int(v) for v in self.box)
        if y is None or not np.any(y): return Placed((r0, r1, c0, c1), self.rgb, self.rgb.any(-1), o)
        half = math.hypot(r1 - r0, c1 - c0) / 2 * RES
        pad = int(math.ceil((math.hypot(y[0], y[1]) + abs(y[2]) * half) / RES)) + 1
        R0, R1, C0, C1 = max(r0 - pad, 0), min(r1 + pad, H), max(c0 - pad, 0), min(c1 + pad, W)
        rr, cc = np.mgrid[R0:R1, C0:C1]
        qx, qy = -HX + (cc + 0.5) * RES, HY - (rr + 0.5) * RES
        c, s = math.cos(-y[2]), math.sin(-y[2]); dx, dy = qx - o[0] - y[0], qy - o[1] - y[1]
        sx, sy = c * dx - s * dy + o[0], s * dx + c * dy + o[1]      # where each cell's floor came from, before the move
        sr, sc = (HY - sy) / RES - 0.5 - r0, (sx + HX) / RES - 0.5 - c0
        m = ndi.map_coordinates(self.rgb.any(-1).astype(np.float32), [sr, sc], order=1, cval=0) > 0.999
        rgb = np.stack([ndi.map_coordinates(self.rgb[..., k].astype(np.float32), [sr, sc], order=1, cval=0)
                        for k in range(3)], -1)
        return Placed((R0, R1, C0, C1), np.where(m[..., None], np.clip(rgb + 0.5, 0, 255), 0).astype(np.uint8), m, o)


class Floor:
    """the floor picture built from placed views (each cell the mean of the views over it), from which any of them can
    be left out"""

    def __init__(self, placed, nights=None):
        self.placed = placed; self.S = np.zeros((H, W), np.float64); self.N = np.zeros((H, W), np.int32)
        self.rgb = {}
        for k, p in enumerate(placed):
            r0, r1, c0, c1 = p.box
            self.S[r0:r1, c0:c1] += np.where(p.mask, p.grey(), 0); self.N[r0:r1, c0:c1] += p.mask
            if nights is not None:
                n = int(nights[k])
                if n not in self.rgb: self.rgb[n] = (np.zeros((H, W, 3), np.float32), np.zeros((H, W), np.int32))
                Sr, Nr = self.rgb[n]; Sr[r0:r1, c0:c1] += np.where(p.mask[..., None], p.rgb, 0); Nr[r0:r1, c0:c1] += p.mask

    def reference(self, box, leave, pad):
        """the picture over box (grown by pad) without the views in leave: (window box, grey, seen)"""
        r0, r1, c0, c1 = max(box[0] - pad, 0), min(box[1] + pad, H), max(box[2] - pad, 0), min(box[3] + pad, W)
        S = self.S[r0:r1, c0:c1].copy(); N = self.N[r0:r1, c0:c1].copy()
        for j in leave:
            p = self.placed[j]; a0, a1, b0, b1 = p.box
            R0, R1, C0, C1 = max(a0, r0), min(a1, r1), max(b0, c0), min(b1, c1)
            if R1 <= R0 or C1 <= C0: continue
            m = p.mask[R0 - a0:R1 - a0, C0 - b0:C1 - b0]
            S[R0 - r0:R1 - r0, C0 - c0:C1 - c0] -= np.where(m, p.grey()[R0 - a0:R1 - a0, C0 - b0:C1 - b0], 0)
            N[R0 - r0:R1 - r0, C0 - c0:C1 - c0] -= m
        seen = N > 0
        return (r0, r1, c0, c1), np.where(seen, S / np.maximum(N, 1), 0.0), seen

    def loo(self, i, leave, pad):
        """view i's reference: the picture without the views in leave (i among them), over view i's box grown by pad"""
        return self.reference(self.placed[i].box, leave, pad)

    def picture(self, night):
        """one night's picture from its views: (rgb, seen)"""
        Sr, Nr = self.rgb[night]; seen = Nr > 0
        return np.where(seen[..., None], Sr / np.maximum(Nr, 1)[..., None] + 0.5, 0).astype(np.uint8), seen


def leave_out(i, ts, night, leave_s=LEAVE_S):
    """the views left out of view i's reference: itself and its own night's views within leave_s seconds of it"""
    return list(np.nonzero((night == night[i]) & (np.abs(ts - ts[i]) <= leave_s))[0]) if leave_s > 0 else [i]


@dataclass
class Measure:
    move: np.ndarray              # (dx, dy, dth): how the view must move, about its own centre, to lie on the floor
    score: float
    lead: float
    info: np.ndarray = None       # how sure, per m^2 in (x, y): from the whole surface, so a ridge holds only across it
    info_th: float = 0.0          # and per rad^2


def _sub(m_, c0, p_):
    d_ = m_ - 2 * c0 + p_; return 0.0 if d_ >= 0 else float(np.clip(0.5 * (m_ - p_) / d_, -0.5, 0.5))


def measure(pv, floor, leave, search_cells=SEARCH_CELLS, turns_deg=TURNS_DEG, min_score=MIN_SCORE, min_lead=MIN_LEAD):
    """where the placed view pv lies on the floor picture without the views in leave: a lock (Measure), or None when
    there is too little to match, no clear best, or the best is on the search's edge (the floor may lie beyond reach)"""
    if search_cells < 1: return None
    pad = search_cells + 2; win, ref, rm = floor.reference(pv.box, leave, pad)
    r0, r1, c0, c1 = win; a0, a1, b0, b1 = pv.box
    F = np.zeros(ref.shape, np.float32); Fm = np.zeros(ref.shape, bool)
    F[a0 - r0:a1 - r0, b0 - c0:b1 - c0] = pv.grey(); Fm[a0 - r0:a1 - r0, b0 - c0:b1 - c0] = pv.mask
    S = freg.ncc_surfaces(F, Fm, ref, rm, search_cells, turns_deg, sigma=1.0)
    if S is None: return None
    bi = int(np.argmax([c.max() for _td, c in S])); td, c = S[bi]; k = np.unravel_index(np.argmax(c), c.shape); sc = float(c[k])
    n = 2 * search_cells
    if sc < min_score or k[0] in (0, n) or k[1] in (0, n): return None
    rr, cc = np.mgrid[:c.shape[0], :c.shape[1]]; far = np.hypot(rr - k[0], cc - k[1]) >= 4   # past the peak's own breadth
    lead = sc - float(c[far].max()) if far.any() else 0.0
    if lead < min_lead: return None
    dr = k[0] - search_cells + _sub(c[k[0] - 1, k[1]], sc, c[k[0] + 1, k[1]])
    dc = k[1] - search_cells + _sub(c[k[0], k[1] - 1], sc, c[k[0], k[1] + 1])
    if 0 < bi < len(S) - 1:                                     # the turn between the turns tried
        step = turns_deg[bi + 1] - turns_deg[bi]; td = td + step * _sub(S[bi - 1][1].max(), sc, S[bi + 1][1].max())
    th = -math.radians(td); d = np.array([dc * RES, -dr * RES])  # the view's floor at (r, c) lies at (r + dr, c + dc) of
    ow = centre(win); cs_, sn = math.cos(th), math.sin(th); R = np.array([[cs_, -sn], [sn, cs_]])   # the reference turned
    t = R @ (pv.o + d - ow) + ow - pv.o                          # by td about the window's centre: shift, then turn back
    _mu, cov = freg.surface_belief(c, TAU)                       # rows, cols in cells^2 -> x, y in m^2
    A = np.array([[0.0, 1.0], [-1.0, 0.0]]); cxy = A @ cov @ A.T * RES ** 2 + np.eye(2) * LOCK_SD_FLOOR_M ** 2
    tw = np.exp((np.array([cc_.max() for _t, cc_ in S]) - sc) / TAU); tw /= tw.sum(); tdeg = np.array(turns_deg, float)
    tv = float((tw * (tdeg - (tw * tdeg).sum()) ** 2).sum()) + LOCK_SD_FLOOR_DEG ** 2
    return Measure(np.array([t[0], t[1], th]), sc, lead, np.linalg.inv(cxy), 1.0 / math.radians(math.sqrt(tv)) ** 2)


@dataclass
class Keyframes:
    P: np.ndarray                 # where each stands in the house's frame as the shape solve placed it (x, y)
    night: np.ndarray
    trav: np.ndarray              # metres walked to it, in its night
    ts: np.ndarray


@dataclass
class ViewTerms:
    o: np.ndarray                 # each view's centre (x, y)
    a: np.ndarray                 # the keyframes either side of it in time, and how far between them it lies
    b: np.ndarray
    f: np.ndarray
    dd: np.ndarray                # metres walked between it and its nearer keyframe
    target: np.ndarray            # where its lock says it should be moved to (dx, dy, dth about o), NaN: no lock
    info: np.ndarray              # the lock's information per m^2: (Nv, 2, 2), or (Nv,) for x and y alike
    info_th: np.ndarray           # and per rad^2


def _lock_loss(is_lock):
    """soft_l1 on the locks' rows, plain squares on the rest (scipy's loss(z) contract: rho, rho', rho'' per row): a
    lock far from what the rest agree on counts less, while the lidar's, odometry's and the priors' rows stay quadratic,
    so no robust loss can let the images bend the shape"""
    def loss(z):
        out = np.empty((3, len(z))); out[0], out[1], out[2] = z, 1.0, 0.0
        zl = z[is_lock]; t = 1 + zl
        out[0, is_lock] = 2 * (np.sqrt(t) - 1); out[1, is_lock] = 1 / np.sqrt(t); out[2, is_lock] = -0.5 * t ** -1.5
        return out
    return loss


def solve(K, lidar, V, tie_scale=1.0):
    """one robust least-squares solve (scipy.optimize.least_squares, each row whitened by its own information) for
    every keyframe's and view's move (dx, dy, dth; a keyframe's about where it stands, a view's about its centre) from
    where the shape solve put them: the lidar's matches between keyframes keep their relative places (lidar: [(i, j,
    information 3x3 for (x, y, turn) at keyframe j)]); odometry ties a night's consecutive keyframes and each view to
    its keyframes; the locks pull the views, through a soft_l1 loss (f_scale LOCK_F_SCALE of their own sd); the first
    keyframe holds the frame. The bound is the views' box bounds, then clipped to a circle. Returns (X keyframes
    (Nk, 3), Y views (Nv, 3))"""
    from scipy.optimize import least_squares
    Nk, Nv = len(K.P), len(V.o); nx = 3 * (Nk + Nv); P = K.P; perp = lambda v: np.array([-v[1], v[0]])
    yv = lambda v: 3 * Nk + 3 * v
    has = np.isfinite(V.target).all(1)
    rows, cols, vals, rhs, lock = [], [], [], [], []; r = [0]
    def eq(terms, b, wt, is_lock=False):
        for c_, v_ in terms: rows.append(r[0]); cols.append(c_); vals.append(v_ * wt)
        rhs.append(b * wt); lock.append(is_lock); r[0] += 1
    for i, j, Io in lidar:                                      # the shape as solved: no change in their relative place
        o = P[j]; wv, Vv = np.linalg.eigh(Io); L = (Vv * np.sqrt(np.clip(wv, 0, None))) @ Vv.T
        pi_, pj = perp(o - P[i]), perp(o - P[j])
        terms = [[(3 * j, 1.0), (3 * j + 2, pj[0]), (3 * i, -1.0), (3 * i + 2, -pi_[0])],
                 [(3 * j + 1, 1.0), (3 * j + 2, pj[1]), (3 * i + 1, -1.0), (3 * i + 2, -pi_[1])],
                 [(3 * j + 2, 1.0), (3 * i + 2, -1.0)]]
        for rr in range(3): eq([(c_, v_ * L[rr, k]) for k in range(3) for c_, v_ in terms[k]], 0.0, 1.0)
    for i in range(Nk - 1):                                     # odometry between a night's consecutive keyframes
        if K.night[i] != K.night[i + 1]: continue
        dd = abs(K.trav[i + 1] - K.trav[i]); wt, wr = 1 / (SEQ_SD[0] + SEQ_SD[1] * dd), 1 / (SEQ_SD[2] + SEQ_SD[3] * dd)
        pi_ = perp(P[i + 1] - P[i])
        for k in (0, 1): eq([(3 * (i + 1) + k, 1.0), (3 * i + k, -1.0), (3 * i + 2, -pi_[k])], 0.0, wt)
        eq([(3 * (i + 1) + 2, 1.0), (3 * i + 2, -1.0)], 0.0, wr)
    for v in range(Nv):                                         # each view where its keyframes put it, as odometry allows
        o = V.o[v]; a, b, f = int(V.a[v]), int(V.b[v]), float(V.f[v])
        sd = (TIE_SD_M + SEQ_SD[0] + SEQ_SD[1] * V.dd[v]) * tie_scale
        sdr = (math.radians(TIE_SD_DEG) + SEQ_SD[2] + SEQ_SD[3] * V.dd[v]) * tie_scale
        pa, pb = perp(o - P[a]), perp(o - P[b])
        for k in (0, 1):
            eq([(yv(v) + k, 1.0), (3 * a + k, -(1 - f)), (3 * a + 2, -(1 - f) * pa[k]), (3 * b + k, -f), (3 * b + 2, -f * pb[k])], 0.0, 1 / sd)
        eq([(yv(v) + 2, 1.0), (3 * a + 2, -(1 - f)), (3 * b + 2, -f)], 0.0, 1 / sdr)
        if has[v]:                                              # its lock, whitened by its own information
            I2 = V.info[v] if np.ndim(V.info[v]) == 2 else np.eye(2) * V.info[v]
            wv, Vv = np.linalg.eigh(I2); L = (Vv * np.sqrt(np.clip(wv, 0, None))) @ Vv.T
            for k in (0, 1): eq([(yv(v), L[k, 0]), (yv(v) + 1, L[k, 1])], float(L[k] @ V.target[v, :2]), 1.0, True)
            if V.info_th[v] > 0: eq([(yv(v) + 2, 1.0)], float(V.target[v, 2]), math.sqrt(V.info_th[v]), True)
    for k in range(3): eq([(k, 1.0)], 0.0, 1e4)                   # the first keyframe holds the frame
    for i in range(Nk + Nv):                                     # nothing wanders off (weak)
        for k in range(3): eq([(3 * i + k, 1.0)], 0.0, 1.0 / (0.5 if k < 2 else 0.2))
    A = coo_matrix((vals, (rows, cols)), shape=(r[0], nx)).tocsr(); bvec = np.array(rhs); is_lock = np.array(lock)
    lo = np.full(nx, -np.inf); hi = np.full(nx, np.inf); tb = math.radians(TURN_BOUND_DEG)
    lo[3 * Nk:] = np.tile([-MOVE_BOUND_M, -MOVE_BOUND_M, -tb], Nv); hi[3 * Nk:] = -lo[3 * Nk:]   # decision [ObC]
    sol = least_squares(lambda x: A @ x - bvec, np.zeros(nx), jac=lambda x: A, bounds=(lo, hi), method="trf",
                        tr_solver="lsmr", loss=_lock_loss(is_lock) if is_lock.any() else "linear",
                        f_scale=LOCK_F_SCALE, x_scale=1.0, max_nfev=200)
    x = sol.x; X = x[:3 * Nk].reshape(Nk, 3); Y = x[3 * Nk:].reshape(Nv, 3).copy()
    sh = np.hypot(Y[:, 0], Y[:, 1]); over = sh > MOVE_BOUND_M          # the box's corners: back to the circle
    Y[over, :2] *= (MOVE_BOUND_M / sh[over])[:, None]
    return X, Y


def agreement_share(rgb_a, Va, rgb_b, Vb, occ, band_m=None, tc=None, slack=None):
    """the nights' disagreement without truth (floor_agreement's 'all'): the share of the floor both pictures saw, away
    from the structures both mapped (occ), that reads as changed by the detector's own test"""
    import confirm_stream as cs
    C = cs.CONFIRM; band_m = C["band_m"] if band_m is None else band_m
    band = ndi.distance_transform_edt(~occ) * RES < band_m; J = Va & Vb & ~band
    ca, ra = feats(rgb_a, Va); cb, rb = feats(rgb_b, Vb)
    dC, _ = tolerant_diff(ca, ra, cb, rb, slack=C["slack"] if slack is None else slack)
    ch = dC > (C["tc"] if tc is None else tc)
    return float((ch & J).sum() / J.sum()) if J.sum() else float("nan")


def refine(state, one_pass, agree, passes=4, min_gain=MIN_GAIN, log=print):
    """passes of one_pass(state) -> state, each kept only if agree(state) (the nights' disagreement, lower is better)
    falls by more than min_gain of itself; the first that doesn't is thrown away and refining stops. (best state, history)"""
    best, best_s = state, agree(state); hist = [{"pass": 0, "share": best_s, "kept": True}]
    log(f"pass 0: the nights disagree on {best_s:.3%} of the floor")
    for p in range(1, passes + 1):
        new = one_pass(best); s = agree(new); kept = bool(s < best_s * (1 - min_gain))
        hist.append({"pass": p, "share": s, "kept": kept})
        log(f"pass {p}: {s:.3%} ({'kept' if kept else 'not kept: no better than ' + format(best_s, '.3%')})")
        if not kept: break
        best, best_s = new, s
    return best, hist


# --- the run: views, keyframes and the lidar's matches from the recordings -------------------------------------------

def load_views(paths):
    """both nights' views (baseline_views.py), in order: (views, times, nights)"""
    out = []
    for n, p in enumerate(paths, 1):
        z = np.load(p); flat = z["flat"]; o = 0
        for s_, t in zip(z["sls"], z["ts"]):
            k = (s_[1] - s_[0]) * (s_[3] - s_[2]) * 3
            out.append(View(flat[o:o + k].reshape(s_[1] - s_[0], s_[3] - s_[2], 3), tuple(int(v) for v in s_), float(t), n)); o += k
    return out, np.array([v.t for v in out]), np.array([v.night for v in out])


def keyframes_and_lidar(dbs, init, workers):
    """the shape solve's keyframes, placed by its corrections, and the lidar's match between each nearby pair there
    (house_shape.py's own matcher and pairs): (Keyframes, lidar terms [(i, j, info at keyframe j)], corrections)"""
    import house_shape as hs
    from concurrent.futures import ProcessPoolExecutor
    from houndkeeper.localize import Localizer
    KF = [k for n, db in enumerate(dbs, 1) for k in hs.keyframes(db, n)]; N = len(KF)
    night = np.array([k[0] for k in KF]); ts = np.array([k[1] for k in KF]); trav = np.array([k[6] for k in KF])
    locs = {n: Localizer(times=list(np.load(Path(init) / f"corrections_{n}.npz")["times"]),
                         Ts=list(np.load(Path(init) / f"corrections_{n}.npz")["Ts"])) for n in sorted(set(night))}
    C = np.array([locs[n].correction(t) for n, t in zip(night, ts)])
    B = np.array([[k[2], k[3]] for k in KF]); P = np.array([c[:2, :2] @ b + c[:2, 2] for c, b in zip(C, B)])
    pts = [k[5] @ c[:2, :2].T + c[:2, 2] for k, c in zip(KF, C)]; pairs = set()
    for i in range(N):
        d = np.hypot(*(P - P[i]).T); ok = d < hs.NEAR_M; ok[i] = False
        for j in np.nonzero(ok)[0][np.argsort(d[ok])][:hs.PER_KEY]: pairs.add((min(i, j), max(i, j)))
    with ProcessPoolExecutor(workers, initializer=hs._init, initargs=(pts,)) as ex:
        meas = [m for m in ex.map(hs.pair_match, sorted(pairs), chunksize=32) if m is not None]
    lidar = []
    for i, j, T, I, rms, n in meas:
        dd = abs(trav[j] - trav[i]) if night[i] == night[j] else 50.0
        if not hs.drift_ok(T, P[j], dd): continue
        o = P[j]; Jo = np.array([[1, 0, -o[1]], [0, 1, o[0]], [0, 0, 1.0]]); Io = np.linalg.inv(Jo).T @ I @ np.linalg.inv(Jo)
        top = float(np.linalg.eigvalsh(Io[:2, :2]).max())            # never surer than LIDAR_SD_FLOOR_M
        lidar.append((i, j, Io * min(1.0, (1 / LIDAR_SD_FLOOR_M ** 2) / max(top, 1e-12))))
    return Keyframes(P=P, night=night, trav=trav, ts=ts), lidar, locs


def link_views(K, ts, night, o):
    """each view's keyframes either side of it in time, its place between them, and the metres walked to the nearer"""
    a, b, f, dd = [], [], [], []
    for t, n in zip(ts, night):
        idx = np.nonzero(K.night == n)[0]; kt = K.ts[idx]; k = int(np.searchsorted(kt, t))
        ia, ib = idx[max(k - 1, 0)], idx[min(k, len(idx) - 1)]
        fr_ = 0.0 if ia == ib else float(np.clip((t - K.ts[ia]) / (K.ts[ib] - K.ts[ia]), 0, 1))
        tv = np.interp(t, kt, K.trav[idx]); a.append(ia); b.append(ib); f.append(fr_)
        dd.append(min(abs(tv - K.trav[ia]), abs(K.trav[ib] - tv)))
    nv = len(ts)
    return ViewTerms(o=o, a=np.array(a), b=np.array(b), f=np.array(f), dd=np.array(dd), target=np.full((nv, 3), np.nan),
                     info=np.zeros((nv, 2, 2)), info_th=np.zeros(nv))


_G = {}                                                          # a pass's floor, shared with the measuring workers (fork)


def _measure_one(i):
    g = _G; leave = leave_out(i, g["ts"], g["night"], g["leave_s"])
    return i, measure(g["placed"][i], g["floor"], leave, reach_cells(g["Y"][i]))


def corrections_out(K, X, locs, views, Y, ts, night):
    """each night's refined corrections (times, 3x3): the keyframes' and the views' own, merged in time"""
    out = {}
    for n in sorted(locs):
        T, M = [], []
        for i in np.nonzero(K.night == n)[0]:
            T.append(K.ts[i]); M.append(se2_about(X[i], K.P[i]) @ locs[n].correction(K.ts[i]))
        for v in np.nonzero(night == n)[0]:
            T.append(ts[v]); M.append(se2_about(Y[v], centre(views[v].box)) @ locs[n].correction(ts[v]))
        T = np.array(T); o_ = np.argsort(T, kind="stable"); T, M = T[o_], np.array(M)[o_]; u = np.r_[True, np.diff(T) > 1e-6]
        out[n] = (T[u], M[u])
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("--views", nargs=2, required=True); ap.add_argument("--dbs", nargs=2, required=True)
    ap.add_argument("--init", required=True, help="the corrections the views were painted with (dir with corrections_<n>.npz: house_shape's)")
    ap.add_argument("--occ", nargs=2, default=None, help="the two nights' pictures whose mapped structures (occ) bound the judged floor")
    ap.add_argument("--out", required=True); ap.add_argument("--passes", type=int, default=4)
    ap.add_argument("--workers", type=int, default=3); ap.add_argument("--leave-s", type=float, default=LEAVE_S)
    ap.add_argument("--min-gain", type=float, default=MIN_GAIN)
    a = ap.parse_args(argv); t0 = time.time(); out = Path(a.out).resolve()
    if out in {Path(a.init).resolve(), *(Path(v).resolve().parent for v in a.views)}:
        raise SystemExit(f"--out {out}: never over the run's own files; give a scratch directory")
    out.mkdir(parents=True, exist_ok=True)
    views, ts, night = load_views(a.views); o = np.array([centre(v.box) for v in views])
    print(f"{len(views)} views ({(night == 1).sum()} + {(night == 2).sum()})", flush=True)
    K, lidar, locs = keyframes_and_lidar(a.dbs, a.init, a.workers)
    print(f"{len(K.P)} keyframes, {len(lidar)} lidar matches kept ({time.time() - t0:.0f} s)", flush=True)
    V = link_views(K, ts, night, o)
    occ = (np.load(a.occ[0])["occ"] & np.load(a.occ[1])["occ"]) if a.occ else np.zeros((H, W), bool)
    stats = []
    def build(state):
        X, Y = state; placed = [v.placed(y) for v, y in zip(views, Y)]; return placed, Floor(placed, night)
    def agree(state):
        _, F = build(state); (A, Va), (B, Vb) = F.picture(1), F.picture(2)
        return agreement_share(A, Va, B, Vb, occ)
    def one_pass(state):
        import multiprocessing as mp
        X, Y = state; placed, F = build(state)
        _G.update(placed=placed, floor=F, ts=ts, night=night, leave_s=a.leave_s, Y=Y)
        with mp.get_context("fork").Pool(a.workers) as pool: res = pool.map(_measure_one, range(len(views)), chunksize=8)
        V.target[:] = np.nan; V.info[:] = 0; V.info_th[:] = 0; nl = 0; mv = []
        for i, m in res:
            if m is None: continue
            V.target[i] = compose(m.move, Y[i]); V.info[i] = m.info * m.score   # a weaker match, trusted less
            V.info_th[i] = m.info_th * m.score; nl += 1; mv.append(math.hypot(*m.move[:2]))
        X2, Y2 = solve(K, lidar, V)
        sh = np.hypot(Y2[:, 0], Y2[:, 1]) * 1000; kx = np.hypot(X2[:, 0], X2[:, 1]) * 1000
        stats.append({"locked": nl, "lock_move_mm_median": round(float(np.median(mv)) * 1000, 1) if mv else None,
                      "views_moved_mm_median": round(float(np.median(sh)), 1), "views_moved_mm_max": round(float(sh.max()), 1),
                      "keyframes_moved_mm_median": round(float(np.median(kx)), 1), "keyframes_moved_mm_max": round(float(kx.max()), 1)})
        print(f"  {nl} of {len(views)} views locked (their pull {stats[-1]['lock_move_mm_median']} mm median); views moved "
              f"{np.median(sh):.1f} mm median, {sh.max():.1f} max; keyframes {np.median(kx):.1f} median, {kx.max():.1f} max "
              f"({time.time() - t0:.0f} s)", flush=True)
        return X2, Y2
    zero = (np.zeros((len(K.P), 3)), np.zeros((len(views), 3)))
    (X, Y), hist = refine(zero, one_pass, agree, a.passes, a.min_gain)
    for h, s in zip(hist[1:], stats): h.update(s)
    for n, (T, M) in corrections_out(K, X, locs, views, Y, ts, night).items():
        np.savez(out / f"corrections_{n}.npz", times=T, Ts=M)
    _, F = build((X, Y))
    for n in (1, 2):
        rgb, Vn = F.picture(n)
        np.savez_compressed(out / f"views_picture_n{n}.npz", rgb=rgb, dist=np.where(Vn, 1.0, np.inf).astype(np.float16), occ=occ)
    kept = sum(h["kept"] for h in hist[1:])
    (out / "summary.json").write_text(json.dumps({"views": len(views), "keyframes": len(K.P), "lidar_matches": len(lidar),
                                                  "passes_kept": kept, "history": hist}, indent=1))
    print(f"floor refine: {kept} pass{'es' if kept != 1 else ''} kept of {len(hist) - 1}; {time.time() - t0:.0f} s -> {out}/corrections_<n>.npz")


if __name__ == "__main__":
    main()
