#!/usr/bin/env python3
"""The floor stitched like a panorama ([WFI]; principle [fXD]: the views fit
together like a puzzle, in exactly one configuration where they all agree). Every
earlier night's views of the floor (baseline_views.py) that overlap are laid on each other (floor_register.snap, coarse
to fine: the nights' odometry frames sit up to ~25 cm apart), each lock measuring how far one view sits from the other;
then one least-squares solve gives every view the small correction (shift and turn about its patch's centre) under which
all the measurements agree at once: a panorama's global alignment (Szeliski, "Image Alignment and Stitching", 2006).
Consecutive views of a night are tied as odometry allows (it drifts ~3 cm and ~1 deg a metre), matches that disagree
with the rest are down-weighted (Huber), and the first view holds the frame. Writes each night's corrections (times,
3x3 SE(2)) for floor_mosaic2.py --corrections.
Usage: stitch_floor.py <run dir> --nights 1 2 --out <dir> (dimos venv)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import argparse, math, time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from scipy import ndimage as ndi

from houndkeeper import floor_register as fr
import confirm_stream as cs
import robust_ls as rl

NEAR_M, PER_VIEW, MIN_OVERLAP = 1.2, 8, 4000     # pairs: patches' centres within 1.2 m, 8 a view, 0.4 m2 in common
SAME_NIGHT_GAP_S = 5.0                           # views of one night closer in time than this are tied by odometry only
SAME_SPOT_M, SAME_SPOT_DEG = 0.15, 10.0          # nor are a night's views from where it stood still: they match at exactly
                                                 # zero and tell nothing (house 71, 28 Sep: its pairs "agreed" to 0.1 mm)
SEQ_SD = (0.005, 0.03, math.radians(0.1), math.radians(1.0))   # odometry between consecutive views: m + m/m, rad + rad/m
HUBER = 0.02                                     # m: a pair's disagreement beyond this counts linearly
COARSE_CELLS = 8                                 # the coarse search, in 4 cm cells: +-32 cm for views placed by odometry
                                                 # (nights ~25 cm apart); +-12 cm (--near) for views the lidar's pose graph
                                                 # placed (nights 7-9 cm apart), inside a patterned tile's repeat, where a
                                                 # wider search sees several equal placements and nothing locks (house 71,
                                                 # 28 Sep: 68% of pairs matched well, 3% clearly best)
_V = None
MIN_SCORE, MIN_LEAD = fr.SNAP_MIN_SCORE, fr.SNAP_MIN_LEAD   # a lock's test (--min-score, --min-lead)


def _grey(i):
    p = _V.patches[i]; return p.astype(np.float32).mean(-1), p.any(-1)


def _window(i, j, pad):
    a, b = _V.sls[i], _V.sls[j]
    r0, r1, c0, c1 = max(a[0], b[0]) - pad, min(a[1], b[1]) + pad, max(a[2], b[2]) - pad, min(a[3], b[3]) + pad
    return (r0, r1, c0, c1) if r1 - r0 > 2 * pad + 20 and c1 - c0 > 2 * pad + 20 else None


def _crop(i, w):
    g, m = _grey(i); a = _V.sls[i]; r0, r1, c0, c1 = w
    G = np.zeros((r1 - r0, c1 - c0), np.float32); M = np.zeros(G.shape, bool)
    R0, R1, C0, C1 = max(r0, a[0]), min(r1, a[1]), max(c0, a[2]), min(c1, a[3])
    if R1 > R0 and C1 > C0:
        G[R0 - r0:R1 - r0, C0 - c0:C1 - c0] = g[R0 - a[0]:R1 - a[0], C0 - a[2]:C1 - a[2]]
        M[R0 - r0:R1 - r0, C0 - c0:C1 - c0] = m[R0 - a[0]:R1 - a[0], C0 - a[2]:C1 - a[2]]
    return G, M


def _down(g, m, k=4):
    h, w = (g.shape[0] // k) * k, (g.shape[1] // k) * k
    gs = (g[:h, :w] * m[:h, :w]).reshape(h // k, k, w // k, k).sum((1, 3)); n = m[:h, :w].reshape(h // k, k, w // k, k).sum((1, 3))
    return np.where(n > 0, gs / np.maximum(n, 1), 0).astype(np.float32), n >= k * k // 2


def measure(pair):
    """how far view j sits from view i where they overlap: (i, j, overlap centre x, y, dx, dy, dth, score) or None"""
    i, j = pair; w = _window(i, j, 36)
    if w is None: return None
    Gi, Mi = _crop(i, w); Gj, Mj = _crop(j, w)
    if (Mi & Mj).sum() < MIN_OVERLAP: return None
    gi, mi = _down(Gi, Mi); gj, mj = _down(Gj, Mj)                      # coarse: 4 cm cells, +-32 cm, +-2 deg
    c = fr.snap(gj, mj, gi, mi, maxs=COARSE_CELLS, turns_deg=(-2.0, -1.0, 0.0, 1.0, 2.0), sigma=0.7, turn_edge_locks=True)
    if c is None or c[3] < MIN_SCORE or c[4] < MIN_LEAD: return None
    td0, dr0, dc0 = math.degrees(c[0]), int(c[1]) * 4, int(c[2]) * 4
    Mr = ndi.rotate(np.where(Mi, Gi, 0), td0, reshape=False, order=1) if td0 else Gi   # fine: 1 cm, +-4 cm, +-0.5 deg
    Mmr = (ndi.rotate(Mi.astype(np.float32), td0, reshape=False, order=0) > 0.5) if td0 else Mi
    Mr, Mmr = np.roll(Mr, (-dr0, -dc0), (0, 1)), np.roll(Mmr, (-dr0, -dc0), (0, 1))
    f = fr.snap(Gj, Mj, Mr, Mmr, maxs=6, turns_deg=(-0.5, 0.0, 0.5), lead_cells=4, turn_edge_locks=True)   # 1 cm peaks are broad
    if f is None or f[3] < MIN_SCORE or f[4] < MIN_LEAD: return None
    td, dr, dc = td0 + math.degrees(f[0]), dr0 + f[1], dc0 + f[2]
    ox, oy = -cs.HX + ((w[2] + w[3]) / 2) * cs.RES, cs.HY - ((w[0] + w[1]) / 2) * cs.RES
    return i, j, ox, oy, dc * cs.RES, -dr * cs.RES, -math.radians(td), float(f[3])   # view j must move by this to lie on i


SOFT = False                                     # --soft: every pair kept, weighed by its own peak
SOFT_MIN_SCORE = 0.10                            # below this a "match" is noise: it lays nothing on anything
SOFT_SCALE = 1.0                                 # information per (NCC curvature per cell^2 x score), est.
SOFT_MODE, TAU, COARSE_UNC = "peak", 0.05, True  # --soft-mode belief: each match from its whole surface (fr.surface_belief),
FLOOR_SD_M = 0.005                               # at softmax temperature TAU; its coarse stage's doubt added; never surer than this


def _sqrtm(I):
    w, V = np.linalg.eigh(I); return (V * np.sqrt(np.clip(w, 0, None))) @ V.T


def measure_soft(pair):
    """as measure, but no pair is thrown away for being unsure (a view must lie
    within a known range, so the best placement there is taken, placed as strongly as it fits): the best
    placement within the search, with how sharply it peaks in each direction (fr.peak_information) times how well it
    fits (the score) as its weight. A ridge (wood grain, a stripe) holds across and not along; a flat peak, hardly at
    all; a section seen only by unsure matches is placed by them, loosely. Only a best on the search's edge (the truth
    may lie beyond) or a score below SOFT_MIN_SCORE is dropped.
    Returns (i, j, ox, oy, dx, dy, dth, score, info 2x2 in x, y per m^2, info in turn per rad^2) or None"""
    i, j = pair; w = _window(i, j, 36)
    if w is None: return None
    Gi, Mi = _crop(i, w); Gj, Mj = _crop(j, w)
    if (Mi & Mj).sum() < MIN_OVERLAP: return None
    gi, mi = _down(Gi, Mi); gj, mj = _down(Gj, Mj)                      # coarse: 4 cm cells
    S = fr.ncc_surfaces(gj, mj, gi, mi, COARSE_CELLS, (-2.0, -1.0, 0.0, 1.0, 2.0), sigma=0.7)
    if S is None: return None
    sc0, td0, k0 = max(((float(c.max()), td, np.unravel_index(np.argmax(c), c.shape)) for td, c in S), key=lambda x: x[0])
    if sc0 < SOFT_MIN_SCORE or k0[0] in (0, 2 * COARSE_CELLS) or k0[1] in (0, 2 * COARSE_CELLS): return None
    dr0, dc0 = (int(k0[0]) - COARSE_CELLS) * 4, (int(k0[1]) - COARSE_CELLS) * 4
    Mr = ndi.rotate(np.where(Mi, Gi, 0), td0, reshape=False, order=1) if td0 else Gi   # fine: 1 cm, +-6 cm, +-0.5 deg
    Mmr = (ndi.rotate(Mi.astype(np.float32), td0, reshape=False, order=0) > 0.5) if td0 else Mi
    Mr, Mmr = np.roll(Mr, (-dr0, -dc0), (0, 1)), np.roll(Mmr, (-dr0, -dc0), (0, 1))
    S = fr.ncc_surfaces(Gj, Mj, Mr, Mmr, 6, (-0.5, 0.0, 0.5))
    if S is None: return None
    peaks = [(float(c.max()), td, np.unravel_index(np.argmax(c), c.shape), c) for td, c in S]
    b = int(np.argmax([x[0] for x in peaks])); sc, tdf, k, c = peaks[b]
    if sc < SOFT_MIN_SCORE or k[0] in (0, 12) or k[1] in (0, 12): return None
    if SOFT_MODE == "belief":                                            # the whole surface's say, not just its peak
        bf = fr.surface_belief(c, TAU); bc = fr.surface_belief(next(cc for td_, cc in fr.ncc_surfaces(gj, mj, gi, mi, COARSE_CELLS, (td0,), sigma=0.7)), TAU) if COARSE_UNC else None
        if bf is None: return None
        mu, cov = bf; cov = cov + (bc[1] * 16.0 if bc is not None else 0.0)          # coarse cells are 4 cm
        A = np.array([[0.0, 1.0], [-1.0, 0.0]]); cxy = A @ cov @ A.T * cs.RES ** 2 + np.eye(2) * FLOOR_SD_M ** 2
        Ixy = np.linalg.inv(cxy)
        tw = np.exp((np.array([x[0] for x in peaks]) - sc) / TAU); tw /= tw.sum(); tdeg = np.array([x[1] for x in peaks])
        tm = float((tw * tdeg).sum()); tv = float((tw * (tdeg - tm) ** 2).sum()) + 0.05 ** 2
        Ith = 1.0 / math.radians(math.sqrt(tv)) ** 2
        td, dr, dc = td0 + tm, dr0 + (mu[0] - 6), dc0 + (mu[1] - 6)
        ox, oy = -cs.HX + ((w[2] + w[3]) / 2) * cs.RES, cs.HY - ((w[0] + w[1]) / 2) * cs.RES
        return i, j, ox, oy, dc * cs.RES, -dr * cs.RES, -math.radians(td), sc, Ixy, Ith
    Irc = fr.peak_information(c, k) * sc * SOFT_SCALE                   # rows, cols per cell^2
    J = np.array([[0.0, -1.0], [1.0, 0.0]]) / cs.RES                     # (rows, cols) per (x, y) m
    Ixy = J.T @ Irc @ J
    s3 = [x[0] for x in peaks]; curv = s3[0] - 2 * s3[1] + s3[2]          # the score over the three turns
    Ith = max(0.0, -curv) / math.radians(0.5) ** 2 * sc * SOFT_SCALE if b == 1 else 0.0
    td, dr, dc = td0 + tdf, dr0 + (k[0] - 6), dc0 + (k[1] - 6)
    ox, oy = -cs.HX + ((w[2] + w[3]) / 2) * cs.RES, cs.HY - ((w[0] + w[1]) / 2) * cs.RES
    return i, j, ox, oy, dc * cs.RES, -dr * cs.RES, -math.radians(td), sc, Ixy, Ith


def floor_jac(T, o, eps=1e-3):
    """how the floor point o, as camera T (world <- optical) paints it, moves when the camera turns a little about each
    of its own axes (pitch, yaw, roll as calibrate._rot): (2, 3) m per rad, or None if o isn't below the horizon.
    Far points move most (the error grows with the square of the range): views of one spot from different distances
    disagree by the mount, which is what lets the stitch solve for it"""
    from houndkeeper import calibrate as cal
    C, R = T[:3, 3], T[:3, :3]; o3 = np.array([o[0], o[1], 0.0]); dc = R.T @ (o3 - C); J = np.zeros((2, 3))
    for k in range(3):
        ang = [0.0, 0.0, 0.0]; ang[k] = eps; w = R @ cal._rot(*ang) @ dc
        if w[2] >= -1e-6: return None
        J[:, k] = ((C + w * (-C[2] / w[2]))[:2] - o) / eps
    return J


MOUNT_PRIOR_RAD = math.radians(2.0)              # the mount correction's prior: a printed mount, est. +-1 deg ([nAH] 55b)
YAW_PRIOR_RAD = math.radians(0.02)               # yaw is held where it starts: a camera's small yaw looks almost like each
                                                 # view turned, which the views' own turns already take, so the floor can't
                                                 # tell them apart and it drifts (28 Sep: from +0.19 to -0.11 on a second
                                                 # pass, the truth +0.4); pitch and roll have no such twin and settle
MOUNT_MAX_DEG = 1.5                              # a correction beyond this on any axis is the stitch not seeing the mount
MOUNT_MIN_PAIRS = 150                            # nor is one resting on fewer locked pairs than this (house 70: 451-473;
                                                 # house 71's patterned tile: 25-40, 28 Sep)


def _init(paths, coarse=None, score=None, lead=None, soft=False, mode="peak", tau=0.05, coarse_unc=True):
    global _V, COARSE_CELLS, MIN_SCORE, MIN_LEAD, SOFT, SOFT_MODE, TAU, COARSE_UNC; _V = cs.Views.load(paths); SOFT = soft
    SOFT_MODE, TAU, COARSE_UNC = mode, tau, coarse_unc
    if coarse: COARSE_CELLS = coarse
    if score is not None: MIN_SCORE = score
    if lead is not None: MIN_LEAD = lead


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("run"); ap.add_argument("--nights", type=int, nargs="+", default=[1, 2])
    ap.add_argument("--out", required=True); ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--mount", default=None, help="the mount the views were painted with (.json)")
    ap.add_argument("--mount-out", default=None, help="solve the camera's mount with the views and write it here (.json)")
    ap.add_argument("--near", action="store_true", help="the views are already placed by the lidar's pose graph (house_frame.py): search +-12 cm, not +-32")
    ap.add_argument("--search-cm", type=int, default=None, help="the coarse search's reach (cm, in 4 cm cells); default +-32, or +-12 with --near")
    ap.add_argument("--min-score", type=float, default=None); ap.add_argument("--min-lead", type=float, default=None)
    ap.add_argument("--soft", action="store_true", help="keep every pair, weighed by how sharply and how well it matches (measure_soft)")
    ap.add_argument("--soft-mode", default="peak", choices=["peak", "belief"]); ap.add_argument("--tau", type=float, default=0.05)
    ap.add_argument("--no-coarse-unc", action="store_true", help="belief: leave the coarse stage's doubt out")
    ap.add_argument("--solver", default="irls", choices=["irls", "ls", "gtsam"], help="irls (default): Huber by reweighting lsqr, four passes; ls: one scipy least_squares solve with a true robust loss; gtsam: a GTSAM factor graph, Levenberg-Marquardt, robust noise models, and how firmly each view is pinned (marginals.npz) ([2Ke])")
    ap.add_argument("--loss", default="huber", choices=["huber", "soft_l1", "cauchy"], help="--solver ls (huber, soft_l1) or gtsam (huber, cauchy): the pairs' robust loss, its scale measured from the pairs")
    a = ap.parse_args(); t0 = time.time()
    coarse = a.search_cm // 4 if a.search_cm else (3 if a.near else None)
    run, out = Path(a.run), Path(a.out); out.mkdir(parents=True, exist_ok=True)
    paths = [str(run / f"views_n{k}.npz") for k in a.nights]; _init(paths)
    night = np.concatenate([np.full(len(np.load(p)["C"]), k) for p, k in zip(paths, a.nights)])
    N = len(_V.C); ts = _V.ts; P = np.array([[-cs.HX + ((s_[2] + s_[3]) / 2) * cs.RES, cs.HY - ((s_[0] + s_[1]) / 2) * cs.RES] for s_ in _V.sls])
    pairs = set()
    for i in range(N):
        d = np.hypot(*(P - P[i]).T); ok = (d < NEAR_M) & ((night != night[i]) | (np.abs(ts - ts[i]) > SAME_NIGHT_GAP_S)); ok[i] = False
        same_spot = (np.hypot(*(_V.C - _V.C[i]).T) < SAME_SPOT_M) & (_V.F @ _V.F[i] > math.cos(math.radians(SAME_SPOT_DEG)))
        ok &= ~(same_spot & (night == night[i]))                        # the robot standing still: the same view again
        for j in np.nonzero(ok)[0][np.argsort(d[ok])][:PER_VIEW]: pairs.add((min(i, j), max(i, j)))
    with ProcessPoolExecutor(a.workers, initializer=_init, initargs=(paths, coarse, a.min_score, a.min_lead, a.soft, a.soft_mode, a.tau, not a.no_coarse_unc)) as ex:
        meas = [m for m in ex.map(measure_soft if a.soft else measure, sorted(pairs), chunksize=16) if m is not None]
    if not a.soft: meas = [m + (np.eye(2) * (m[7] / 0.01) ** 2, (m[7] / 0.01) ** 2) for m in meas]   # a lock: its score, 1 cm (est.)
    print(f"{N} views, {len(pairs)} overlapping pairs, {len(meas)} locked ({time.time() - t0:.0f} s)", flush=True)
    if meas:                                                 # the texture springs themselves, for one solve with the lidar's
        np.savez(out / "springs.npz", i=np.array([m[0] for m in meas]), j=np.array([m[1] for m in meas]),   # (house_shape.py
                 o=np.array([[m[2], m[3]] for m in meas]), d=np.array([[m[4], m[5], m[6]] for m in meas]),  # --texture)
                 Ixy=np.array([m[8] for m in meas]), Ith=np.array([m[9] for m in meas]), ts=ts, night=night, P=P)
    perp = lambda v: np.array([-v[1], v[0]])
    w_pair = np.ones(len(meas))                                          # Huber's down-weighting; each pair's own weight is its info
    solve_mount = a.mount_out is not None and _V.Ts is not None          # the camera's mount solved with the views ([fXD]):
    Jm = {}                                                              # a self-calibrating bundle adjustment
    if solve_mount:
        for (i, j, ox, oy, *_r) in meas:
            o = np.array([ox, oy]); Ji, Jj = floor_jac(_V.Ts[i], o), floor_jac(_V.Ts[j], o)
            if Ji is not None and Jj is not None: Jm[(i, j)] = Jj - Ji
    nx = 3 * N + (3 if solve_mount else 0)
    S = rl.Rows()                                                        # the system, built once; a pair's rows are its group
    for p_, (i, j, ox, oy, dx, dy, dth, sc, Ixy, Ith) in enumerate(meas):
        o = np.array([ox, oy]); pi_, pj = perp(o - P[i]), perp(o - P[j])
        jm = Jm.get((i, j)); L = _sqrtm(Ixy)                             # the pair's two equations, weighed by its
        terms = []                                                       # information: firm directions count, flat ones not
        for k in (0, 1):
            mt = [(3 * N + c_, jm[k, c_]) for c_ in range(3)] if jm is not None else []
            terms.append([(3 * j + k, 1.0), (3 * j + 2, pj[k]), (3 * i + k, -1.0), (3 * i + 2, -pi_[k])] + mt)
        for r_ in (0, 1):
            if L[r_, 0] == 0 and L[r_, 1] == 0: continue
            S.eq([(c_, v_ * L[r_, 0]) for c_, v_ in terms[0]] + [(c_, v_ * L[r_, 1]) for c_, v_ in terms[1]], L[r_, 0] * dx + L[r_, 1] * dy, 1.0, p_)
        if Ith > 0: S.eq([(3 * j + 2, 1.0), (3 * i + 2, -1.0)], dth, math.sqrt(Ith), p_)
    for i in range(N - 1):                                              # odometry between a night's consecutive views
        if night[i] != night[i + 1]: continue
        dd = float(np.hypot(*(_V.C[i + 1] - _V.C[i])))
        wt, wr = 1.0 / (SEQ_SD[0] + SEQ_SD[1] * dd), 1.0 / (SEQ_SD[2] + SEQ_SD[3] * dd)
        for k in (0, 1): S.eq([(3 * (i + 1) + k, 1.0), (3 * i + k, -1.0)], 0.0, wt)
        S.eq([(3 * (i + 1) + 2, 1.0), (3 * i + 2, -1.0)], 0.0, wr)
    for k in range(3): S.eq([(k, 1.0)], 0.0, 1e4)                       # the first view holds the frame
    for i in range(N):
        for k in range(3): S.eq([(3 * i + k, 1.0)], 0.0, 1.0 / (0.3 if k < 2 else 0.1))   # and nothing wanders off (weak)
    if solve_mount:
        for k in range(3): S.eq([(3 * N + k, 1.0)], 0.0, 1.0 / (MOUNT_PRIOR_RAD if k != 1 else YAW_PRIOR_RAD))
    A, bvec, groups = S.system(nx)

    def disagree(x):                                                    # each pair's disagreement left, in m
        res = []
        for (i, j, ox, oy, dx, dy, dth, sc, *_i) in meas:
            o = np.array([ox, oy]); d_ = (x[3 * j:3 * j + 2] + x[3 * j + 2] * perp(o - P[j])) - (x[3 * i:3 * i + 2] + x[3 * i + 2] * perp(o - P[i]))
            if (i, j) in Jm: d_ = d_ + Jm[(i, j)] @ x[3 * N:3 * N + 3]
            res.append(float(np.hypot(*(d_ - [dx, dy]))))
        return np.array(res)
    say = lambda tag, res: print(f"{tag}: " + (f"pairs disagree by {np.median(res) * 1000:.1f} mm median, {np.percentile(res, 95) * 1000:.1f} p95"
                                               if len(res) else "no pairs locked"), flush=True)   # none: the mount below is refused, not a crash
    t_solve = time.time()
    if a.solver == "irls":                                              # the old path: Huber by reweighting, each pair's
        for it in range(4):                                             # rows scaled by its weight (so its squared
            x = rl.lsqr_weighted(A, bvec, groups, w_pair)               # residual by the weight squared: critique #12)
            res = disagree(x); w_pair = np.minimum(1.0, HUBER / np.maximum(res, 1e-9))
            say(f"solve {it + 1}", res)
    elif a.solver == "ls":                                              # one robust solve, the loss's scale measured ([2Ke])
        x, info = rl.solve_robust(A, bvec, groups, loss=a.loss)
        say(f"solve ({a.loss}, scale {info['f_scale']:.2f} sd, {info['nfev']} evaluations)", disagree(x))
    else:                                                               # the same, as a GTSAM factor graph ([2Ke])
        x, info = rl.solve_gtsam(A, bvec, groups, loss=a.loss, marginals=True)
        say(f"solve (gtsam {a.loss}, scale {info['f_scale']:.2f} sd)", disagree(x))
        sxy, sth = rl.pinned(info["cov"][:N]); worst = np.argsort(-sxy)[:5]
        np.savez(out / "marginals.npz", ts=ts, night=night, P=P, sd_xy=sxy, sd_th=sth)
        print(f"pinned: views' position sd {np.median(sxy) * 1000:.1f} mm median, {np.percentile(sxy, 95) * 1000:.1f} p95; loosest "
              + ", ".join(f"n{night[i]} t{ts[i]:.0f} {sxy[i] * 1000:.0f} mm" for i in worst), flush=True)
    print(f"solve time {time.time() - t_solve:.2f} s ({a.solver})", flush=True)
    for k in a.nights:                                                  # each view's correction as a world SE(2) at its time
        idx = np.nonzero(night == k)[0]; Ts = []
        for i in idx:
            th = x[3 * i + 2]; R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            T = np.eye(3); T[:2, :2] = R; T[:2, 2] = P[i] - R @ P[i] + x[3 * i:3 * i + 2]; Ts.append(T)
        o_ = np.argsort(ts[idx]); np.savez(out / f"corrections_{k}.npz", times=ts[idx][o_], Ts=np.array(Ts)[o_])
        sh = np.hypot(x[3 * idx], x[3 * idx + 1]) * 1000
        print(f"night {k}: views moved {np.median(sh):.0f} mm median, {sh.max():.0f} max; turned {np.degrees(np.median(np.abs(x[3 * idx + 2]))):.2f} deg median")
    if solve_mount:                                                     # the mount the views were painted with, turned by
        import json                                                     # the solved correction (small: angles add)
        old = json.loads(Path(a.mount).read_text()) if a.mount else {"pitch_deg": 0.0, "yaw_deg": 0.0, "roll_deg": 0.0}
        dm = np.degrees(x[3 * N:3 * N + 3]); refused = bool(np.max(np.abs(dm)) > MOUNT_MAX_DEG or len(Jm) < MOUNT_MIN_PAIRS)
        if len(Jm) < MOUNT_MIN_PAIRS:
            print(f"mount correction refused ({np.round(dm, 2)} deg on only {len(Jm)} pairs); keeping {a.mount}"); dm = np.zeros(3)
        elif np.max(np.abs(dm)) > MOUNT_MAX_DEG:                        # beyond what a printed mount can be off: the stitch
            print(f"mount correction refused ({np.round(dm, 2)} deg: over {MOUNT_MAX_DEG} deg); keeping {a.mount}")   # couldn't
            dm = np.zeros(3)                                            # see it (house 71, 28 Sep: yaw +4.1 deg)
        new = {k: round(float(old[k]) + float(d_), 3) for k, d_ in zip(("pitch_deg", "yaw_deg", "roll_deg"), dm)}
        Path(a.mount_out).write_text(json.dumps(dict(new, from_stitch=True, correction_deg=[round(float(v), 3) for v in dm],
                                                     pairs=len(Jm), refused=refused), indent=1))
        print(f"mount corrected by pitch {dm[0]:+.2f} yaw {dm[1]:+.2f} roll {dm[2]:+.2f} deg ({len(Jm)} pairs) -> {a.mount_out}")
    import json as _json                                                # how much the stitch rested on, for the pipeline's gate
    (out / "summary.json").write_text(_json.dumps({"views": N, "pairs": len(pairs), "locked": len(meas)}))
    print(f"stitched in {time.time() - t0:.0f} s -> {out}/corrections_<n>.npz")


if __name__ == "__main__":
    main()
