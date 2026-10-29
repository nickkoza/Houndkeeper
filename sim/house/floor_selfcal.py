#!/usr/bin/env python3
"""The head camera's floor self-calibration, the robot's own ([gJq], decision [yFg]; the nightly check, [9tQ]): from
the views the robot keeps of its own floor (baseline_views.py), with no target and no truth, the camera's pitch, yaw,
height and the floor rows' time offset, and each 0.6 m tile's height above the house's floor (its relief).

Each view is laid again with the geometry so far (regeom: each painted cell moved along its own line of sight, no frame
painted again), the views blended, each registered on the blend tile by tile, and one least squares (Huber) solves the
camera's terms and each tile's floor height; repeated until the views agree. The laws (camgeo.py, which also holds the
sim-only diagnosis): a pitch error moves the floor by r^2, a height error and a tile's relief by r, a time offset by the
turning rate and velocity.

Subcommands (sim/house, dimos venv; the robot's side: reads only the views, never the sim's truth):
  relief <views.npz ...> --out <run>/relief.npy [--frame shape/corrections_1.npz ...]
        the house's floor relief with the camera held at the robot's calibration (the views were painted with it):
        written with relief.json beside it (the views' calibration, the frame it is in, the tiles raised). Part of the
        house's map ([yFg]); stitch_baselines.sh solves it.
  check <views.npz ...> --out <json>
        the nightly check ([9tQ]): the geometry solved free from these views (painted with the robot's calibration)
        is the night's disagreement with the record; warned past CHECK_TOL. Never writes the robot's calibration."""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import argparse, json, math, time
from pathlib import Path

import numpy as np

BANDS = ((0.5, 0.9), (0.9, 1.3), (1.3, 1.7), (1.7, 2.1), (2.1, 2.5))   # ranges from the camera's foot (m)
CHECK_TOL = dict(pitch_deg=0.1, yaw_deg=0.3, up_m=0.005, lag_s=0.010)   # the nightly check's tolerances ([9tQ], [yFg]); yaw 0.3: houses differ by 0.12-0.25 deg (7 Oct)
CHECK_MIN_REGS = 2000   # a night's views with fewer tile registrations are too little floor to check (the proposal's
                        # houses under it scattered by up to 0.4 deg, sim 6 Oct 2026)


def structure(g, V, s=10):
    """brightness relative to its surroundings (mean and spread over s cells): lighting and exposure fall away"""
    from scipy import ndimage as ndi
    V = V.astype(np.float32); den = np.maximum(ndi.gaussian_filter(V, s), 1e-3)
    mu = ndi.gaussian_filter(g * V, s) / den; sd = np.sqrt(np.maximum(ndi.gaussian_filter((g - mu) ** 2 * V, s) / den, 1))
    return np.where(V > 0, (g - mu) / sd, 0)


TILE_M = 0.6          # the floor's height is solved per tile this size (m): a carpet or rug is several tiles


TILE_MIN_CELLS = 1200 # a view's cells in a tile for a registration (of 3,600)


REACH = 16            # cells: the registration's search, either way (16 cm: a 0.5 deg pitch error moves floor 2.3 m out
                      # 16 cm), every second shift first, then the best refined


def _register_cells(A, pr, pc, Bpad, rr, cc, S, step=1):
    """_register_cells_loop's result, every shift's correlation gathered at once (8 Oct 2026: the same numbers, about
    ten times faster; the finer relief registers four times as many tiles)"""
    a = A[pr, pc]; a = (a - a.mean()) / (a.std() + 1e-6); n = len(a)
    def corr(sh):
        sh = np.asarray(sh, int).reshape(-1, 2)
        b = Bpad[rr[None, :] + S + sh[:, :1], cc[None, :] + S + sh[:, 1:]]
        bm = b - b.mean(1, keepdims=True); sd = b.std(1)
        return np.where(sd > 1e-6, (bm @ a) / (n * np.maximum(sd, 1e-12)), -1.0)
    g = np.arange(-S, S + 1, step); sh = np.array([(i, j) for i in g for j in g]); c = corr(sh)
    sc = {(int(i), int(j)): float(v) for (i, j), v in zip(sh, c)}
    k = max(sc, key=sc.get)
    if step > 1:
        ex = [(i, j) for i in range(k[0] - step + 1, k[0] + step) for j in range(k[1] - step + 1, k[1] + step)
              if abs(i) <= S and abs(j) <= S and (i, j) not in sc]
        if ex: sc.update({q: float(v) for q, v in zip(ex, corr(ex))})
        k = max(sc, key=sc.get)
    if abs(k[0]) >= S - 1 or abs(k[1]) >= S - 1: return None
    ex = [q for q in ((k[0] + 1, k[1]), (k[0] - 1, k[1]), (k[0], k[1] + 1), (k[0], k[1] - 1)) if q not in sc]
    if ex: sc.update({q: float(v) for q, v in zip(ex, corr(ex))})
    sub = lambda m_, c0, p_: 0.0 if m_ - 2 * c0 + p_ >= 0 else float(np.clip(0.5 * (m_ - p_) / (m_ - 2 * c0 + p_), -0.5, 0.5))
    di = k[0] + sub(sc[k[0] - 1, k[1]], sc[k], sc[k[0] + 1, k[1]])
    dj = k[1] + sub(sc[k[0], k[1] - 1], sc[k], sc[k[0], k[1] + 1])
    return di, dj, float(sc[k])


def _register_cells_loop(A, pr, pc, Bpad, rr, cc, S, step=1):
    """the shift (di, dj) cells, to a fraction, that best lays painted structure A (at its cells pr, pc) on the
    reference's (Bpad, padded by S) at their grid cells (rr, cc): (di, dj, peak correlation), or None at the search's
    edge. step 2: the search first on every second shift (S reach for the cost of S/2), then the best refined"""
    a = A[pr, pc]; a = (a - a.mean()) / (a.std() + 1e-6)
    def corr(i, j):
        b = Bpad[rr + S + i, cc + S + j]; sd = b.std()
        return float(a @ (b - b.mean())) / (len(a) * sd) if sd > 1e-6 else -1.0
    sc = {}
    for i in range(-S, S + 1, step):
        for j in range(-S, S + 1, step): sc[i, j] = corr(i, j)
    k = max(sc, key=sc.get)
    if step > 1:
        for i in range(k[0] - step + 1, k[0] + step):
            for j in range(k[1] - step + 1, k[1] + step):
                if abs(i) <= S and abs(j) <= S and (i, j) not in sc: sc[i, j] = corr(i, j)
        k = max(sc, key=sc.get)
    if abs(k[0]) >= S - 1 or abs(k[1]) >= S - 1: return None
    for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        q = (k[0] + d[0], k[1] + d[1])
        if q not in sc: sc[q] = corr(*q)
    sub = lambda m_, c0, p_: 0.0 if m_ - 2 * c0 + p_ >= 0 else float(np.clip(0.5 * (m_ - p_) / (m_ - 2 * c0 + p_), -0.5, 0.5))
    di = k[0] + sub(sc[k[0] - 1, k[1]], sc[k], sc[k[0] + 1, k[1]])
    dj = k[1] + sub(sc[k[0], k[1] - 1], sc[k], sc[k[0], k[1] + 1])
    return di, dj, float(sc[k])


def regeom(views, dpitch=0.0, dup=0.0, dfwd=0.0, dyaw=0.0, relief=None, moves=None):
    """the views as their cameras would have painted them had the believed geometry been changed by: dpitch (rad, the
    camera taken as turned down by this much more), dup (m, as this much higher), dfwd (m, as this much further
    forward; along the view's facing), dyaw (rad, as turned this much more to the left), and with the floor taken to
    stand relief (a grid, m; floor_relief) above the plane. Each painted cell's colour is moved along its own line of
    sight onto where the changed camera would lay that line, so no frame is painted again. The views must have been
    painted on the plane (no relief). moves: each view's rigid move on the floor (3x3 SE(2), world), made after the
    change: the same views placed by other poses. Returns a new Views"""
    import confirm_stream as cs
    from scipy import ndimage as ndi
    from houndkeeper import calibrate as cal
    from detect_changes2 import RES, HX, HY
    if views.Ts is None: raise SystemExit("the views carry no camera poses (painted before [JzW]): paint them again")
    from houndkeeper import floor_relief as frl
    Rp = cal._rot(-dpitch, -dyaw, 0.0); P, S, Tn, Cn, Fn = [], [], [], [], []; GH, GW = int(round(2 * HY / RES)), int(round(2 * HX / RES))
    for k, (p, (r0, r1, c0, c1)) in enumerate(zip(views.patches, views.sls)):
        T = views.Ts[k]; R, C = T[:3, :3], T[:3, 3]; f = views.F[k]
        R2 = R @ Rp; C2 = C + np.array([dfwd * f[0], dfwd * f[1], dup]); T2 = T.copy(); T2[:3, :3] = R2; T2[:3, 3] = C2
        D = np.eye(3) if moves is None else np.asarray(moves[k], float); D3 = np.eye(4); D3[:2, :2] = D[:2, :2]; D3[:2, 3] = D[:2, 2]
        Tn.append(D3 @ T2); Cn.append(D[:2, :2] @ views.C[k] + D[:2, 2]); Fn.append(D[:2, :2] @ f)
        m = int(math.ceil(0.12 / RES)); q0, q1, p0, p1 = r0, r1, c0, c1
        if moves is not None:                                   # the box where the moved view lands
            cx_ = -HX + (np.array([c0, c1, c0, c1]) + 0.5) * RES; cy_ = HY - (np.array([r0, r0, r1, r1]) + 0.5) * RES
            q = D[:2, :2] @ np.vstack([cx_, cy_]) + D[:2, 2:3]
            q0, q1 = int((HY - q[1].max()) / RES), int((HY - q[1].min()) / RES) + 1; p0, p1 = int((q[0].min() + HX) / RES), int((q[0].max() + HX) / RES) + 1
        a0, a1, b0, b1 = max(q0 - m, 0), min(q1 + m, GH), max(p0 - m, 0), min(p1 + m, GW)
        if a1 <= a0 or b1 <= b0: P.append(np.zeros((1, 1, 3), np.uint8)); S.append((0, 1, 0, 1)); continue
        rr, cc = np.mgrid[a0:a1, b0:b1]; X = -HX + (cc + 0.5) * RES; Y = HY - (rr + 0.5) * RES
        Z = frl.at(relief, X, Y) if relief is not None else np.zeros(X.shape)
        if moves is not None:                                   # where each cell was before the move
            Di = np.linalg.inv(D); X, Y = Di[0, 0] * X + Di[0, 1] * Y + Di[0, 2], Di[1, 0] * X + Di[1, 1] * Y + Di[1, 2]
        d = np.stack([X - C2[0], Y - C2[1], Z - C2[2]], -1) @ R2                           # the changed camera's sight lines
        w = d @ R.T; s = -C[2] / np.where(w[..., 2] < -1e-6, w[..., 2], np.nan)         # ... as the old camera laid them
        Xs, Ys = C[0] + w[..., 0] * s, C[1] + w[..., 1] * s
        cs_ = (Xs + HX) / RES - 0.5 - c0; rs_ = (HY - Ys) / RES - 0.5 - r0
        V = p.any(-1); ok = np.isfinite(cs_) & np.isfinite(rs_)
        cs_, rs_ = np.where(ok, cs_, -10), np.where(ok, rs_, -10)
        Vn = ndi.map_coordinates(V.astype(np.float32), [rs_, cs_], order=1, cval=0) > 0.999
        if not Vn.any(): P.append(np.zeros((1, 1, 3), np.uint8)); S.append((r0, r0 + 1, c0, c0 + 1)); continue
        out = np.stack([ndi.map_coordinates(p[..., j].astype(np.float32), [rs_, cs_], order=1, cval=0) for j in range(3)], -1)
        out = np.where(Vn[..., None], np.clip(out, 1, 255), 0).astype(np.uint8)
        ri, ci = np.nonzero(Vn); out = out[ri.min():ri.max() + 1, ci.min():ci.max() + 1]
        P.append(out); S.append((a0 + ri.min(), a0 + ri.max() + 1, b0 + ci.min(), b0 + ci.max() + 1))
    return cs.Views(np.array(Cn), np.array(Fn), S, P, views.ts, np.array(Tn))


def heights(views):
    """the cameras' heights over the floor plane, median (m): the range law's h"""
    return float(np.median(views.Ts[:, 2, 3])) if views.Ts is not None else 0.30


def consensus_sums(views):
    """the views' grey summed per cell, and how many views saw each cell (consensus's own, before dividing)"""
    from detect_changes2 import RES, HX, HY
    Hh, Ww = int(round(2 * HY / RES)), int(round(2 * HX / RES)); S = np.zeros((Hh, Ww), np.float32); N = np.zeros((Hh, Ww), np.int32)
    for p, (r0, r1, c0, c1) in zip(views.patches, views.sls):
        V = p.any(-1); S[r0:r1, c0:c1] += np.where(V, p.astype(np.float32).mean(-1), 0); N[r0:r1, c0:c1] += V
    return S, N


def consensus(views):
    """the views' blended floor (mean grey over every view of each cell, the floor picture's own way), and the cells
    seen by 3 or more views: (structure over the grid, seen)"""
    from detect_changes2 import RES, HX, HY
    Hh, Ww = int(round(2 * HY / RES)), int(round(2 * HX / RES)); S = np.zeros((Hh, Ww), np.float32); N = np.zeros((Hh, Ww), np.int32)
    for p, (r0, r1, c0, c1) in zip(views.patches, views.sls):
        V = p.any(-1); S[r0:r1, c0:c1] += np.where(V, p.astype(np.float32).mean(-1), 0); N[r0:r1, c0:c1] += V
    seen = N >= 3; G = np.where(seen, S / np.maximum(N, 1), 0)
    return structure(G, seen), seen


def view_rates(views, gap_s=1.0):
    """each view's camera turning rate (rad/s, left +) and velocity (x, y m/s) from its neighbours in time (the
    views are ~0.4 s apart; a night's views are in time order, nights apart by more than gap_s)"""
    ts = np.asarray(views.ts, float); h = np.unwrap(np.arctan2(views.F[:, 1], views.F[:, 0])); C = np.asarray(views.C, float)
    w = np.zeros(len(ts)); v = np.zeros((len(ts), 2))
    for k in range(len(ts)):
        a, b = max(k - 1, 0), min(k + 1, len(ts) - 1)
        if a < k and ts[k] - ts[a] > gap_s: a = k
        if b > k and ts[b] - ts[k] > gap_s: b = k
        if b == a or ts[b] <= ts[a]: continue
        w[k] = (h[b] - h[a]) / (ts[b] - ts[a]); v[k] = (C[b] - C[a]) / (ts[b] - ts[a])
    return w, v


def lag_moves(views, rates, dt):
    """each view's floor moved as its camera would have stood dt s later (turned w dt about the camera, moved v dt):
    3x3 SE(2) moves for regeom"""
    w, v = rates; out = []
    for k in range(len(views.C)):
        a = w[k] * dt; c, s_ = math.cos(a), math.sin(a); C = np.asarray(views.C[k], float)
        D = np.eye(3); D[:2, :2] = [[c, -s_], [s_, c]]; D[:2, 2] = C - D[:2, :2] @ C + v[k] * dt; out.append(D)
    return out


def tile_registrations(views, G, seen, every=1, rng_m=(0.5, 2.5), log=None, rates=None, tile_m=None, min_cells=None,
                       node_m=None, mask=None, sums=None):
    """each view's floor, tile by tile (tile_m, default TILE_M, house-wide grid), registered on the consensus G: rows of
    (tile row, tile col, view, range m (mean over the tile's cells), the unit vector from the cells toward the camera
    (x, y), the camera's facing (x, y), the displacement of the view's floor from the consensus (x, y m), peak
    correlation, the camera's turning rate (rad/s) and velocity (x, y m/s) then (view_rates; 0 without rates)).
    mask: cells left out of every registration (the change mask: what is not flat floor). node_m: also each
    registration's footprint on a node grid that spacing (surface_nodes), as (node indices, bilinear weights averaged
    over its cells): returns (rows, footprints). sums: the blend's per-cell grey sums and counts (consensus_sums):
    each view is then registered on the blend of the other views only (leave one out), not partly on its own copy,
    which pulls every registration toward zero (6 Oct: a round saw most of an offset, not all)"""
    from detect_changes2 import RES, HX, HY
    tile_m = tile_m or TILE_M; min_cells = min_cells or int(round(TILE_MIN_CELLS * (tile_m / TILE_M) ** 2))
    S = REACH; Gp = np.pad(G, S) if sums is None else None; tc = int(round(tile_m / RES)); out = []; feet = []; t0 = time.time()
    PW = 30                                                   # the structure filter's context around a view's window
    if sums is not None: Ssp, Nsp = np.pad(sums[0], S + PW), np.pad(sums[1], S + PW)
    use = seen if mask is None else (seen & ~mask)
    if node_m: NA, NB = surface_nodes(node_m)
    for k in range(0, len(views.patches), every):
        p = views.patches[k]; r0, r1, c0, c1 = views.sls[k]; V = p.any(-1)
        if V.sum() < min_cells: continue
        A = structure(p.astype(np.float32).mean(-1), V); C = views.C[k]; f = views.F[k]
        pr, pc = np.nonzero(V); R, Cc = pr + r0, pc + c0; X = -HX + (Cc + 0.5) * RES; Y = HY - (R + 0.5) * RES
        d = np.hypot(C[0] - X, C[1] - Y); ok = (d >= rng_m[0]) & (d < rng_m[1]) & use[R, Cc]
        ti, tj = R // tc, Cc // tc; key = ti * 10000 + tj
        if sums is not None:                                  # the blend of the others, over this view's window
            Sw = Ssp[r0:r1 + 2 * (S + PW), c0:c1 + 2 * (S + PW)].copy(); Nw = Nsp[r0:r1 + 2 * (S + PW), c0:c1 + 2 * (S + PW)].copy()
            o_ = S + PW; Sw[o_:o_ + r1 - r0, o_:o_ + c1 - c0] -= np.where(V, p.astype(np.float32).mean(-1), 0); Nw[o_:o_ + r1 - r0, o_:o_ + c1 - c0] -= V
            sw = Nw >= 2; Bw = structure(np.where(sw, Sw / np.maximum(Nw, 1), 0), sw)
            ok &= sw[pr + o_, pc + o_]
        for kk in np.unique(key[ok]):
            m = ok & (key == kk)
            if m.sum() < min_cells: continue
            if sums is None: reg = _register_cells(A, pr[m], pc[m], Gp, R[m], Cc[m], S, step=2)
            else: reg = _register_cells(A, pr[m], pc[m], Bw, pr[m] + PW, pc[m] + PW, S, step=2)
            if reg is None or reg[2] < 0.3: continue
            di, dj, pk = reg; e = (-dj * RES, di * RES)                 # the view's floor less the consensus's (world m)
            xm, ym = X[m].mean(), Y[m].mean(); u = np.array([C[0] - xm, C[1] - ym]); u /= max(np.linalg.norm(u), 1e-9)
            wk, vk = (rates[0][k], rates[1][k]) if rates is not None else (0.0, (0.0, 0.0))
            out.append((int(kk // 10000), int(kk % 10000), k, float(d[m].mean()), u[0], u[1], f[0], f[1], e[0], e[1], pk, wk, vk[0], vk[1]))
            if node_m: feet.append(_footprint(R[m], Cc[m], node_m, NB))
        if log and k % (200 * every) == 0: log(f"  view {k}/{len(views.patches)}: {len(out)} tile registrations, {time.time() - t0:.0f} s")
    return (np.array(out, float), feet) if node_m else np.array(out, float)


# --- the relief as a smooth surface ([JzW] front end, 8 Oct 2026) ---------------------------------------------------
# The 6 Oct relief was one height per 0.6 m tile, clipped at the floor's level (it could only raise floors): it agreed
# with the plan on 69% of tiles. The surface is a height at every node of a finer grid (bilinear between nodes),
# solved from registrations on smaller tiles, each registration tied to the nodes under its own cells; both ways
# (a floor reading low is lowered); held together by an edge-keeping smoothness prior (Huber on the differences of
# neighbouring nodes: a carpet's edge stays a step) and a weak pull to the floor's level where nothing is seen; the
# registrations robust (Huber) and the cells that are not flat floor (the change mask: the views disagree about them,
# as a chair leg or a toy seen from two sides does) left out.
SURFACE = dict(tile_m=0.3, node_m=0.1, lam=8.0, smooth_huber_m=0.002, mu=0.02, huber_m=0.005, loss="cauchy", iters=8,
               clip_m=(-0.03, 0.05), spread_k=3.0, loo=1, pairs=4, min_lever_m=1.5)
# lam, huber_m, loss and min_lever_m chosen 8 Oct 2026 on tuning houses 70, 73, 76, 77, 83 (round-0 view pairs, every 4th
# view; relief/tune1.txt in the scratch): of 16 settings, this one's tiles agreed with the plan on 0.69-0.96 (0.65-0.93
# for the next) and its raised floor read 9-10 mm where the plan has 8-12; a short lever (under 1.5 m) reads mostly noise


def surface_nodes(node_m):
    """the node grid's shape (rows, cols) over the floor grid: nodes at every node_m from the grid's corner"""
    from detect_changes2 import RES, HX, HY
    return int(math.ceil(2 * HY / node_m)) + 1, int(math.ceil(2 * HX / node_m)) + 1


def _footprint(R, Cc, node_m, NB):
    """a registration's cells (grid rows, cols) as bilinear weights on the node grid, averaged over its cells"""
    from detect_changes2 import RES
    fa = (R + 0.5) * RES / node_m; fb = (Cc + 0.5) * RES / node_m; a0 = np.floor(fa).astype(int); b0 = np.floor(fb).astype(int)
    wa, wb = fa - a0, fb - b0; idx, w = [], []
    for da, db, ww in ((0, 0, (1 - wa) * (1 - wb)), (1, 0, wa * (1 - wb)), (0, 1, (1 - wa) * wb), (1, 1, wa * wb)):
        idx.append((a0 + da) * NB + b0 + db); w.append(ww)
    idx, w = np.concatenate(idx), np.concatenate(w); u, inv_ = np.unique(idx, return_inverse=True)
    return u.astype(np.int64), (np.bincount(inv_, weights=w) / len(R)).astype(np.float64)


def surface_grid(zn, shape, node_m):
    """the node heights as a relief on the floor grid (bilinear)"""
    from scipy import ndimage as ndi
    from detect_changes2 import RES
    rr = (np.arange(shape[0]) + 0.5) * RES / node_m; cc = (np.arange(shape[1]) + 0.5) * RES / node_m
    A, B = np.meshgrid(rr, cc, indexing="ij")
    return ndi.map_coordinates(np.asarray(zn, np.float64), [A, B], order=1, mode="nearest").astype(np.float32)


def change_mask(views, k=None):
    """the cells that are not flat floor, from the views alone: where the views' brightness disagrees past k times the
    house's median disagreement (a thing standing up looks different from each side; flat floor looks the same). The
    views' own relative brightness (structure) is compared, so lighting falls away"""
    from detect_changes2 import RES, HX, HY
    k = k or SURFACE["spread_k"]; Hh, Ww = int(round(2 * HY / RES)), int(round(2 * HX / RES))
    S1 = np.zeros((Hh, Ww), np.float32); S2 = np.zeros((Hh, Ww), np.float32); N = np.zeros((Hh, Ww), np.int32)
    for p, (r0, r1, c0, c1) in zip(views.patches, views.sls):
        V = p.any(-1)
        if not V.any(): continue
        s = structure(p.astype(np.float32).mean(-1), V, s=6)
        S1[r0:r1, c0:c1] += np.where(V, s, 0); S2[r0:r1, c0:c1] += np.where(V, s * s, 0); N[r0:r1, c0:c1] += V
    ok = N >= 3; sd = np.sqrt(np.maximum(S2 / np.maximum(N, 1) - (S1 / np.maximum(N, 1)) ** 2, 0))
    if not ok.any(): return np.zeros((Hh, Ww), bool)
    return ok & (sd > k * float(np.median(sd[ok])))


def solve_surface(regs, feet, h, zn, node_m, opts=None, min_regs=4):
    """the relief's change at every node, by robust least squares over the registrations: a view's floor sits from
    the consensus along the line of sight to its camera by the floor's height under it (the r law, the camera held at
    the robot's calibration), less the consensus's own offset in that tile:
        e = - (r / h) sum_k w_k dz_k u - c_t
    with an edge-keeping smoothness prior on the new heights (zn + dz) between neighbouring nodes and a weak pull of
    them to 0. Tiles seen from one direction only are left out (their offset and their height cannot be told apart).
    Returns (dz node grid, stats, node support: summed weight of the registrations on each node)"""
    from scipy.sparse import coo_matrix, diags, vstack as svstack
    from scipy.sparse.linalg import lsqr
    o = dict(SURFACE, **(opts or {})); NA, NB = zn.shape; nn = NA * NB
    tiles = {}; [tiles.setdefault((int(r[0]), int(r[1])), []).append(i) for i, r in enumerate(regs)]
    keep = {t: ix for t, ix in tiles.items() if len(ix) >= min_regs and len({int(round(math.degrees(math.atan2(regs[i, 5], regs[i, 4])) / 45)) % 8 for i in ix}) >= 2}
    tid = {t: n for n, t in enumerate(keep)}; nt = len(keep)
    rows, cols, vals, rhs = [], [], [], []; eq = 0; support = np.zeros(nn)
    for t, ix in keep.items():
        ct = nn + 2 * tid[t]
        for i in ix:
            r, ux, uy, ex, ey = regs[i, 3], regs[i, 4], regs[i, 5], regs[i, 8], regs[i, 9]; ni, w = feet[i]; support[ni] += w
            for comp, (uc, ec) in enumerate(((ux, ex), (uy, ey))):
                rows += [eq] * len(ni); cols += list(ni); vals += list(-(r / h) * uc * w)
                rows.append(eq); cols.append(ct + comp); vals.append(-1.0); rhs.append(ec); eq += 1
    nd = eq; n = nn + 2 * nt
    A = coo_matrix((vals, (rows, cols)), shape=(nd, n)).tocsr(); b = np.array(rhs)
    a_i = np.arange(NA * (NB - 1)); ra, rb = a_i // (NB - 1), a_i % (NB - 1); h1, h2 = ra * NB + rb, ra * NB + rb + 1
    v_i = np.arange((NA - 1) * NB); va, vb = v_i // NB, v_i % NB; v1, v2 = va * NB + vb, (va + 1) * NB + vb
    p1, p2 = np.r_[h1, v1], np.r_[h2, v2]; ne = len(p1)
    D = coo_matrix((np.r_[np.ones(ne), -np.ones(ne)], (np.r_[np.arange(ne), np.arange(ne)], np.r_[p1, p2])), shape=(ne, n)).tocsr()
    z0 = zn.reshape(-1); dprev = z0[p1] - z0[p2]                  # the smoothness on the new heights: D dz = -(D z0)
    M = coo_matrix((np.ones(nn), (np.arange(nn), np.arange(nn))), shape=(nn, n)).tocsr()
    wd = np.ones(nd); ws = np.ones(ne); x = np.zeros(n); sl, sm = math.sqrt(o["lam"]), math.sqrt(o["mu"])
    for _ in range(o["iters"]):
        AA = svstack([diags(wd) @ A, diags(sl * ws) @ D, sm * M]).tocsr()
        bb = np.r_[wd * b, sl * ws * (-dprev), sm * (-z0)]
        x = lsqr(AA, bb, atol=1e-10, btol=1e-10, iter_lim=20000, x0=x)[0]
        rd = np.abs(A @ x - b); wd = np.sqrt(np.where(rd < o["huber_m"], 1.0, o["huber_m"] / np.maximum(rd, 1e-12)))
        rs = np.abs(D @ x + dprev); ws = np.sqrt(np.where(rs < o["smooth_huber_m"], 1.0, o["smooth_huber_m"] / np.maximum(rs, 1e-12)))
    dz = x[:nn].reshape(NA, NB)
    st = {"tiles": nt, "registrations": int(sum(len(v) for v in keep.values())), "rms_mm": float(np.sqrt(np.mean((A @ x - b) ** 2)) * 1000),
          "dz_mm_abs_median_seen": float(np.median(np.abs(dz.reshape(-1)[support > 0.5])) * 1000) if (support > 0.5).any() else None}
    return dz, st, support.reshape(NA, NB)


def pair_registrations(views, every=1, rng_m=(0.5, 2.5), tile_m=0.3, node_m=0.1, partners=3, min_gap_s=2.0, mask=None,
                       log=None, lever_m=None):
    """each view's floor, tile by tile, registered on another view's floor of the same tile, not on the blend: the
    partners are the views whose line of sight differs most (r u, the camera's offset from the tile: a floor's height
    moves each view's copy of it away from its camera by r z / h, so two views from opposite sides see twice the move
    and two from the same place none; lever_m: the partners whose lines of sight differ by nearest this instead, so a
    rug's repeating pattern (16 cm, sim 8 Oct) is not read a period off). The blend's own offset then never enters (6 Oct, 8 Oct: registered on the
    blend, a carpet tile seen mostly from one side moved with its blend and read a quarter of its height).
    Returns (rows of (tile row, tile col, view k, view j, r_k, u_k (x, y), r_j, u_j (x, y), the displacement of k's
    floor from j's (x, y m), peak), footprints on the node grid)"""
    from detect_changes2 import RES, HX, HY
    S = REACH; tc = int(round(tile_m / RES)); min_cells = int(round(TILE_MIN_CELLS * (tile_m / TILE_M) ** 2))
    NA, NB = surface_nodes(node_m); t0 = time.time()
    Hh, Ww = int(round(2 * HY / RES)), int(round(2 * HX / RES)); use = np.ones((Hh, Ww), bool) if mask is None else ~mask
    St, ent = {}, {}                                              # each view's structure; each tile's view entries
    for k in range(0, len(views.patches), every):
        p = views.patches[k]; r0, r1, c0, c1 = views.sls[k]; V = p.any(-1)
        if V.sum() < min_cells: continue
        A = structure(p.astype(np.float32).mean(-1), V).astype(np.float32); C = views.C[k]
        pr, pc = np.nonzero(V); R, Cc = pr + r0, pc + c0; X = -HX + (Cc + 0.5) * RES; Y = HY - (R + 0.5) * RES
        d = np.hypot(C[0] - X, C[1] - Y); ok = (d >= rng_m[0]) & (d < rng_m[1]) & use[R, Cc]
        key = (R // tc) * 10000 + Cc // tc; kept = False
        for kk in np.unique(key[ok]):
            m = ok & (key == kk)
            if m.sum() < min_cells: continue
            xm, ym = X[m].mean(), Y[m].mean(); u = np.array([C[0] - xm, C[1] - ym]); r = float(np.linalg.norm(u)); u /= max(r, 1e-9)
            ent.setdefault(int(kk), []).append((k, float(d[m].mean()), u)); kept = True
        if kept: St[k] = (A, V)
    out, feet = [], []
    for kk, E in ent.items():
        if len(E) < 2: continue
        ti, tj = kk // 10000, kk % 10000; q0, p0 = ti * tc, tj * tc
        RU = np.array([e[1] * e[2] for e in E]); T = np.array([views.ts[e[0]] for e in E]); done = set()
        for a, (k, rk, uk) in enumerate(E):
            lev = np.hypot(*(RU - RU[a]).T); lev[np.abs(T - T[a]) < min_gap_s] = -1
            order = np.argsort(-lev) if lever_m is None else np.argsort(np.where(lev > 0, np.abs(lev - lever_m), np.inf))
            for b in order[:partners]:
                if lev[b] < 0.3 or (min(a, b), max(a, b)) in done: continue   # the lines of sight differ by 30 cm or more
                done.add((min(a, b), max(a, b)))
                j, rj, uj = E[b]; Ak, Vk = St[k]; Aj, Vj = St[j]
                rk0, rk1, ck0, ck1 = views.sls[k]; rj0, rj1, cj0, cj1 = views.sls[j]
                g0, g1, h0, h1 = q0, q0 + tc, p0, p0 + tc                   # the tile, in grid cells
                gr, gc = np.mgrid[g0:g1, h0:h1]; gr, gc = gr.ravel(), gc.ravel()
                ink = (gr >= rk0) & (gr < rk1) & (gc >= ck0) & (gc < ck1); inj = (gr >= rj0) & (gr < rj1) & (gc >= cj0) & (gc < cj1)
                sel = ink & inj
                if sel.sum() < min_cells: continue
                gr, gc = gr[sel], gc[sel]; okc = Vk[gr - rk0, gc - ck0] & Vj[gr - rj0, gc - cj0] & use[gr, gc]
                if okc.sum() < min_cells: continue
                gr, gc = gr[okc], gc[okc]
                W = np.zeros((tc + 2 * S, tc + 2 * S), np.float32)          # j's structure over the tile and the search
                wr, wc = np.mgrid[g0 - S:g1 + S, h0 - S:h1 + S]
                inw = (wr >= rj0) & (wr < rj1) & (wc >= cj0) & (wc < cj1)
                vw = np.zeros(wr.shape, bool); vw[inw] = Vj[wr[inw] - rj0, wc[inw] - cj0]
                W[vw] = Aj[wr[vw] - rj0, wc[vw] - cj0]
                reg = _register_cells(Ak, gr - rk0, gc - ck0, W, gr - g0, gc - h0, S, step=2)
                if reg is None or reg[2] < 0.3: continue
                di, dj, pk = reg
                out.append((ti, tj, k, j, rk, uk[0], uk[1], rj, uj[0], uj[1], -dj * RES, di * RES, pk))
                feet.append(_footprint(gr, gc, node_m, NB))
    if log: log(f"  {len(out)} view-pair registrations over {len(ent)} tiles, {time.time() - t0:.0f} s")
    return np.array(out, float).reshape(-1, 13), feet


def solve_surface_pairs(pr, feet, h, zn, opts=None):
    """the relief's change at every node from view-pair registrations: k's floor sits from j's by
        e = - (sum_k w_k dz_k / h) (r_k u_k - r_j u_j)
    (no blend, so no offset of its own to solve), with solve_surface's smoothness prior and pull to 0, robust (Huber).
    Returns (dz, stats, support)"""
    from scipy.sparse import coo_matrix, diags, vstack as svstack
    from scipy.sparse.linalg import lsqr
    o = dict(SURFACE, **(opts or {})); NA, NB = zn.shape; nn = NA * NB
    rows, cols, vals, rhs = [], [], [], []; eq = 0; support = np.zeros(nn)
    for i, rr in enumerate(pr):
        g = rr[4] * rr[5:7] - rr[7] * rr[8:10]
        if np.hypot(*g) < o.get("min_lever_m", 0.0): continue    # a short lever reads the height through its noise
        ni, w = feet[i]; support[ni] += w
        for comp in (0, 1):
            rows += [eq] * len(ni); cols += list(ni); vals += list(-(g[comp] / h) * w); rhs.append(rr[10 + comp]); eq += 1
    A = coo_matrix((vals, (rows, cols)), shape=(eq, nn)).tocsr(); b = np.array(rhs); nd = eq
    a_i = np.arange(NA * (NB - 1)); ra, rb = a_i // (NB - 1), a_i % (NB - 1); h1, h2 = ra * NB + rb, ra * NB + rb + 1
    v_i = np.arange((NA - 1) * NB); va, vb = v_i // NB, v_i % NB; v1, v2 = va * NB + vb, (va + 1) * NB + vb
    p1, p2 = np.r_[h1, v1], np.r_[h2, v2]; ne = len(p1)
    D = coo_matrix((np.r_[np.ones(ne), -np.ones(ne)], (np.r_[np.arange(ne), np.arange(ne)], np.r_[p1, p2])), shape=(ne, nn)).tocsr()
    z0 = zn.reshape(-1); dprev = z0[p1] - z0[p2]
    M = coo_matrix((np.ones(nn), (np.arange(nn), np.arange(nn))), shape=(nn, nn)).tocsr()
    wd = np.ones(nd); ws = np.ones(ne); x = np.zeros(nn); sl, sm = math.sqrt(o["lam"]), math.sqrt(o["mu"])
    for _ in range(o["iters"]):
        AA = svstack([diags(wd) @ A, diags(sl * ws) @ D, sm * M]).tocsr(); bb = np.r_[wd * b, sl * ws * (-dprev), sm * (-z0)]
        x = lsqr(AA, bb, atol=1e-10, btol=1e-10, iter_lim=20000, x0=x)[0]
        rd = np.abs(A @ x - b)
        if o.get("loss", "huber") == "cauchy": wd = np.sqrt(1.0 / (1.0 + (rd / o["huber_m"]) ** 2))   # a misread pair's pull fades
        else: wd = np.sqrt(np.where(rd < o["huber_m"], 1.0, o["huber_m"] / np.maximum(rd, 1e-12)))
        rs = np.abs(D @ x + dprev); ws = np.sqrt(np.where(rs < o["smooth_huber_m"], 1.0, o["smooth_huber_m"] / np.maximum(rs, 1e-12)))
    dz = x.reshape(NA, NB)
    st = {"tiles": int(len({(int(r[0]), int(r[1])) for r in pr})), "registrations": int(len(pr)),
          "rms_mm": float(np.sqrt(np.mean((A @ x - b) ** 2)) * 1000) if nd else None,
          "dz_mm_abs_median_seen": float(np.median(np.abs(x[support > 0.5])) * 1000) if (support > 0.5).any() else None}
    return dz, st, support.reshape(NA, NB)


def surface_level(zn, support, share=0.15, step=0.001):
    """the house's floor level in a solved surface: the lowest broad level (carpets and rugs lie on the floor), the lowest
    height, in 1 mm steps, whose +-2 mm holds `share` of the seen nodes at its peak (floor_level's rule, for heights)"""
    z = zn[support > 0.5]
    if not len(z): return 0.0
    grid = np.arange(z.min() - 0.003, z.max() + 0.003, step); dens = np.array([(np.abs(z - g) <= 0.002).mean() for g in grid])
    peaks = [i for i in range(len(grid)) if dens[i] >= share and dens[i] == dens[max(i - 4, 0):i + 5].max()]
    i = peaks[0] if peaks else int(np.argmax(dens)); sel = np.abs(z - grid[i]) <= 0.002
    return float(np.median(z[sel]))


def anchored(zn, support, dead_m=0.0):
    """the surface with the house's floor level taken off (the camera's height is the robot's calibration, held: a level
    common to the whole house is the poses' scale or the camera's, not the floor's), and heights within dead_m of it 0"""
    z = zn - surface_level(zn, support)
    return np.where(np.abs(z) < dead_m, 0.0, z) if dead_m else z


def selfcal_surface(views, h=None, rounds=3, every=1, log=None, plant=None, fixed=(0.0, 0.0, 0.0, 0.0), opts=None, mask=True):
    """the floor's relief as a smooth surface (SURFACE), the camera held at the robot's calibration (fixed: dpitch rad,
    dyaw rad, dup m, dlag s, applied throughout): rounds of laying the views on the relief so far, blending,
    registering on the finer tiles and solving the surface's change. plant: a relief grid (m, the floor grid) the
    views are first laid as if the true floor stood that much proud (the positive control). Returns {relief grid,
    nodes, support, rounds}"""
    from detect_changes2 import RES, HX, HY
    o = dict(SURFACE, **(opts or {})); log = log or (lambda *_: None); h = h or heights(views)
    shape = (int(round(2 * HY / RES)), int(round(2 * HX / RES))); NA, NB = surface_nodes(o["node_m"])
    P, Y, U, L = fixed; v0 = views
    rates = view_rates(v0) if L else None
    if plant is not None: v0 = regeom(v0, relief=-np.asarray(plant, np.float32))   # laid on a floor that low: as a floor
    zn = np.zeros((NA, NB)); hist = []; support = np.zeros((NA, NB))                # that proud would be painted on the plane
    for it in range(rounds + 1):
        rel = surface_grid(zn, shape, o["node_m"]) if it else None
        mv = lag_moves(v0, rates, L) if L else None
        v = regeom(v0, P, U, 0.0, Y, rel, mv) if (P or Y or U or L or rel is not None) else v0
        cm = change_mask(v, o["spread_k"]) if mask else None
        if o.get("pairs"):                                    # view on view (pair_registrations)
            regs, feet = pair_registrations(v, every, tile_m=o["tile_m"], node_m=o["node_m"], partners=int(o["pairs"]), mask=cm, log=log,
                                            lever_m=o.get("lever_m"))
            dz, st, sup = solve_surface_pairs(regs, feet, h, zn, o)
        else:                                                 # view on the blend of the others
            G, seen = consensus(v)
            regs, feet = tile_registrations(v, G, seen, every, tile_m=o["tile_m"], node_m=o["node_m"], mask=cm,
                                            sums=consensus_sums(v) if o.get("loo", 1) else None)
            dz, st, sup = solve_surface(regs, feet, h, zn, o["node_m"], o)
        support = np.maximum(support, sup)
        st.update(round=it, masked_cells=int(cm.sum()) if cm is not None else 0,
                  raised_nodes=int((zn[sup > 0.5] > 0.003).sum()), lowered_nodes=int((zn[sup > 0.5] < -0.003).sum()))
        hist.append(st)
        log(f"round {it}: {st['registrations']} registrations on {st['tiles']} tiles ({st['masked_cells']} cells masked), rms {st['rms_mm']:.1f} mm, "
            f"change |dz| median {st['dz_mm_abs_median_seen']} mm; nodes raised {st['raised_nodes']}, lowered {st['lowered_nodes']}")
        if it == rounds: break
        zn = np.clip(zn + dz, *o["clip_m"])
    return {"relief": surface_grid(zn, shape, o["node_m"]), "nodes": zn, "support": support, "rounds": hist, "h_m": h,
            "opts": o, "mode": "surface"}


def solve_tiles(regs, h, free=("pitch", "fwd", "yaw"), min_regs=6, huber=0.015, iters=8):
    """the camera's geometry and each tile's floor, by least squares (Huber) over the tile registrations: a view's
    floor sits from the consensus by g(r) along the line of sight to its camera, plus a sideways turn, less the
    consensus's own offset there:
        e = [ -dp (r^2 + h^2)/h + a_t r/h + df ] u - dy r l - c_t  - dt (w r l + v)
    globals dp (the camera truly turned down more than believed, rad), df (truly further forward, m), dy (truly turned
    left, rad), dt (the frame truly taken dt s later than its stamp less the lag puts it: the camera had turned w dt and
    moved v dt further, w and v its turning rate and velocity then); per tile a_t (the camera's height over that tile's
    floor truly more than believed, m: the camera's own
    height error less the floor's relief there) and c_t (the consensus's offset, x y m). Returns (globals dict, {tile:
    (a m, registrations)})"""
    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import lsqr
    tiles = {}; [tiles.setdefault((int(r[0]), int(r[1])), []).append(i) for i, r in enumerate(regs)]
    keep = {t: ix for t, ix in tiles.items() if len(ix) >= min_regs and len({int(round(math.degrees(math.atan2(regs[i, 5], regs[i, 4])) / 45)) % 8 for i in ix}) >= 2}
    tid = {t: n for n, t in enumerate(keep)}; G_ = [n for n in ("pitch", "fwd", "yaw", "lag") if n in free]; ng = len(G_)
    rows, cols, vals, rhs = [], [], [], []; eq = 0; idx = []
    for t, ix in keep.items():
        base = ng + 4 * 0 + 3 * tid[t]
        for i in ix:
            r, ux, uy, ex, ey = regs[i, 3], regs[i, 4], regs[i, 5], regs[i, 8], regs[i, 9]; fx, fy = regs[i, 6], regs[i, 7]
            lx, ly = -fy, fx
            for comp, (uc, lc, ec) in enumerate(((ux, lx, ex), (uy, ly, ey))):
                vc = regs[i, 12 + comp] if regs.shape[1] > 13 else 0.0; wk = regs[i, 11] if regs.shape[1] > 13 else 0.0
                coef = {"pitch": -(r * r + h * h) / h * uc, "fwd": uc, "yaw": -r * lc, "lag": -(wk * r * lc + vc)}
                for gi, n in enumerate(G_): rows.append(eq); cols.append(gi); vals.append(coef[n])
                rows.append(eq); cols.append(base); vals.append(r / h * uc)                 # a_t
                rows.append(eq); cols.append(base + 1 + comp); vals.append(-1.0)            # c_t
                rhs.append(ec); idx.append(i); eq += 1
    n = ng + 3 * len(keep); rhs = np.array(rhs); w = np.ones(eq); x = np.zeros(n)
    A = coo_matrix((vals, (rows, cols)), shape=(eq, n)).tocsr()
    for _ in range(iters):
        from scipy.sparse import diags
        x = lsqr(diags(w) @ A, w * rhs, atol=1e-10, btol=1e-10, iter_lim=20000)[0]
        res = np.abs(A @ x - rhs); w = np.sqrt(np.where(res < huber, 1.0, huber / np.maximum(res, 1e-12)))
    glob = {n: float(x[i]) for i, n in enumerate(G_)}
    out = {"dpitch_deg": math.degrees(glob.get("pitch", 0.0)), "dfwd_m": glob.get("fwd", 0.0), "dyaw_deg": math.degrees(glob.get("yaw", 0.0)),
           "dlag_s": glob.get("lag", 0.0),
           "tiles": len(keep), "registrations": int(sum(len(v) for v in keep.values())), "rms_mm": float(np.sqrt(np.mean((A @ x - rhs) ** 2)) * 1000)}
    a_t = {t: (float(x[ng + 3 * tid[t]]), len(keep[t])) for t in keep}
    solve_tiles.consensus_offset = {t: x[ng + 3 * tid[t] + 1:ng + 3 * tid[t] + 3].copy() for t in keep}
    return out, a_t


def along_by_range(regs, a_t, up, log=None, floor_mm=3.0):
    """the facing-apart offset left, by range band, over the tiles at the floor's level (a within floor_mm of the
    camera's own height error: the lowest floor) and over all: each registration's displacement with its tile's
    consensus offset (solve_tiles' last) taken off, along the line of sight (toward the camera +) and across (to the
    camera's left +); medians, mm"""
    c = getattr(solve_tiles, "consensus_offset", {}); out = []
    rows = []
    for r in regs:
        t = (int(r[0]), int(r[1]))
        if t not in c: continue
        e = r[8:10] + c[t]; u = r[4:6]; l = np.array([-r[7], r[6]]); rows.append((r[3], e @ u, e @ l, abs(a_t[t][0] - up) * 1000 <= floor_mm))
    R = np.array(rows, float)
    for lo, hi in BANDS:
        m = (R[:, 0] >= lo) & (R[:, 0] < hi) if len(R) else np.zeros(0, bool); fl = m & (R[:, 3] > 0) if len(R) else m
        d = {"band": [lo, hi], "n": int(m.sum()), "along_mm": round(float(np.median(R[m, 1])) * 1000, 1) if m.any() else None,
             "across_mm": round(float(np.median(R[m, 2])) * 1000, 1) if m.any() else None, "n_floor_level": int(fl.sum()),
             "along_floor_level_mm": round(float(np.median(R[fl, 1])) * 1000, 1) if fl.any() else None}
        out.append(d)
        if log: log(f"  {lo}-{hi} m: along {d['along_mm']} mm, across {d['across_mm']} (n {d['n']}); floor level only: along {d['along_floor_level_mm']} (n {d['n_floor_level']})")
    return out


def floor_level(a_t, share=0.15):
    """the camera's own height error from the tiles' a (a = the height error less the floor's relief): the floor's
    lowest broad level is the house's floor (carpets and rugs lie on it), where a is largest: the highest a, in 2 mm
    steps, that holds `share` of the tiles' registrations within +-3 mm, at its peak"""
    a = np.array([v[0] for v in a_t.values()]); n = np.array([v[1] for v in a_t.values()], float)
    if not len(a): return 0.0
    grid = np.arange(a.min() - 0.004, a.max() + 0.004, 0.001)
    dens = np.array([n[np.abs(a - g) <= 0.003].sum() for g in grid]) / n.sum()
    peaks = [i for i in range(len(grid)) if dens[i] >= share and dens[i] == dens[max(i - 4, 0):i + 5].max()]
    i = peaks[-1] if peaks else int(np.argmax(dens))
    sel = np.abs(a - grid[i]) <= 0.003
    return float(np.average(a[sel], weights=n[sel]))


SELFCAL = dict(rounds=6, gain=1.0, free=("pitch", "yaw", "lag"), relief_min_m=0.003, stop_deg=0.01, stop_m=0.0005, stop_s=0.002)
# each round's change is taken whole and the rounds repeat until it is small: the consensus registration sees most of an
# offset, not all (each view registers partly on its own copy in the blend); the fixed point (every view agreeing with
# the blend) does not depend on that share. The forward place is not solved (weakly seen; 4 mm in the sim).


def relief_grid(tiles, shape, res=0.01):
    """{(tile row, tile col): relief m} as a grid on the floor grid (each tile's value over its cells)"""
    from scipy import ndimage as ndi
    tc = int(round(TILE_M / res)); g = np.zeros(((shape[0] + tc - 1) // tc, (shape[1] + tc - 1) // tc), np.float32)
    for (i, j), z in tiles.items():
        if 0 <= i < g.shape[0] and 0 <= j < g.shape[1]: g[i, j] = z
    return np.repeat(np.repeat(g, tc, 0), tc, 1)[:shape[0], :shape[1]]


def selfcal(views, h=None, rounds=None, gain=None, free=None, every=1, log=None, plant=None, fixed=None):
    """the head camera's geometry and the floor's relief from the robot's own views, by rounds: lay the views with
    the geometry so far, blend them, register each view tile by tile on the blend, solve (solve_tiles), take the
    change at `gain`, again. plant: a (dpitch rad, dup m[, dlag s]) change made to every view first (the positive
    control). fixed: (dpitch rad, dyaw rad, dup m, dlag s), the robot's calibration, applied throughout and never
    changed: only the floor's relief is solved, its level the robot's (a house whose carpet covers most of its floor
    cannot tell its own level from the carpet's). Returns {dpitch_deg, dyaw_deg, dup_m, dlag_s (true less believed:
    what to add to the belief), relief grid, rounds}"""
    from detect_changes2 import RES, HX, HY
    rounds = rounds or SELFCAL["rounds"]; gain = gain or SELFCAL["gain"]; free = free or SELFCAL["free"]; log = log or (lambda *_: None)
    h = h or heights(views); shape = (int(round(2 * HY / RES)), int(round(2 * HX / RES)))
    v0 = regeom(views, plant[0], plant[1]) if plant else views
    rates = view_rates(v0) if "lag" in free else None
    if plant and len(plant) > 2 and plant[2]:                  # a planted lag: every view as its camera stood that much later
        v0 = regeom(v0, moves=lag_moves(v0, view_rates(v0), plant[2]))
    P, Y, U, L = fixed if fixed else (0.0, 0.0, 0.0, 0.0); tiles = {}; hist = []
    for it in range(rounds + 1):
        rel = relief_grid(tiles, shape) if tiles else None
        mv = lag_moves(v0, rates, L) if L else None
        v = regeom(v0, P, U, 0.0, Y, rel, mv) if (P or Y or U or L or rel is not None) else v0
        G, seen = consensus(v); regs = tile_registrations(v, G, seen, every, rates=rates)
        g, a_t = solve_tiles(regs, h, free); up = floor_level(a_t) if not fixed else 0.0; br = along_by_range(regs, a_t, up)
        hist.append({"round": it, "applied": {"dpitch_deg": math.degrees(P), "dyaw_deg": math.degrees(Y), "dup_m": U, "dlag_s": L,
                                              "raised_tiles": int(sum(1 for z in tiles.values() if z > 0))},
                     "seen": {"dpitch_deg": g["dpitch_deg"], "dyaw_deg": g["dyaw_deg"], "dup_m": up, "dlag_s": g["dlag_s"]}, "by_range": br,
                     "tiles": g["tiles"], "registrations": g["registrations"], "rms_mm": g["rms_mm"]})
        log(f"round {it}: applied pitch {math.degrees(P):+.3f} deg, yaw {math.degrees(Y):+.3f} deg, up {U * 1000:+.1f} mm, lag {L * 1000:+.1f} ms, "
            f"{sum(1 for z in tiles.values() if z > 0)} raised tiles; seen pitch {g['dpitch_deg']:+.3f}, yaw {g['dyaw_deg']:+.3f}, up {up * 1000:+.1f} mm, "
            f"lag {g['dlag_s'] * 1000:+.1f} ms; along by range {[b['along_mm'] for b in br]} mm (floor level {[b['along_floor_level_mm'] for b in br]}), "
            f"across {[b['across_mm'] for b in br]}")
        if it == rounds: break
        small = abs(g["dpitch_deg"]) < SELFCAL["stop_deg"] and abs(g["dyaw_deg"]) < SELFCAL["stop_deg"] and abs(up) < SELFCAL["stop_m"] and abs(g["dlag_s"]) < SELFCAL["stop_s"]
        if fixed: small = False                               # the relief alone: its rounds as asked (each tile's own noise
                                                              # is 2-3 mm, so no round leaves every tile still)
        if small and it > 0: break
        if not fixed: P += gain * math.radians(g["dpitch_deg"]); Y += gain * math.radians(g["dyaw_deg"]); U += gain * up; L += gain * g["dlag_s"]
        for t, (aa, n) in a_t.items():                       # the floor's relief: the tile stands this far above the floor
            z = tiles.get(t, 0.0) + gain * (up - aa)
            tiles[t] = float(np.clip(z, 0.0, 0.05))
        tiles = {t: (z if z >= SELFCAL["relief_min_m"] else 0.0) for t, z in tiles.items()}
    return {"dpitch_deg": math.degrees(P), "dyaw_deg": math.degrees(Y), "dup_m": U, "dlag_s": L, "relief": relief_grid(tiles, shape) if tiles else None,
            "relief_tiles": {f"{i},{j}": round(z, 4) for (i, j), z in tiles.items()}, "rounds": hist, "h_m": h}


# --- the house's relief and the nightly check, from the views alone -------------------------------------------------

def views_stamp(paths):
    """the calibration the views were painted with (calibrate.stamp_of), one for all, or a SystemExit when they differ
    (views painted with two calibrations are never solved together)"""
    from houndkeeper import calibrate as cal
    st = [cal.stamp_of(p) for p in paths]
    if any(s != st[0] for s in st): raise SystemExit(f"the views were painted with different calibrations: {dict(zip(map(str, paths), st))}")
    return st[0]


def relief_main(a):
    """the house's relief, the camera held at the robot's calibration (the views were painted with it): stored in the
    run directory with what it is ([yFg])"""
    import confirm_stream as cs
    from houndkeeper import floor_relief as frl
    t0 = time.time(); v = cs.Views.load(a.views); st = views_stamp(a.views)
    if st and st.get("relief_sha"): raise SystemExit("the views were painted on a relief already: the relief is solved from views laid on the plane")
    lg = lambda m: print(f"{m} ({time.time() - t0:.0f} s)", flush=True)
    if a.mode == "surface":                                  # the finer surface, both ways (8 Oct 2026, proposed)
        if a.every > 1:
            k = list(range(0, len(v.patches), a.every)); v = cs.Views(v.C[k], v.F[k], v.sls[k], [v.patches[i] for i in k], v.ts[k], v.Ts[k])
        r = selfcal_surface(v, rounds=a.rounds, log=lg)
        r["relief_tiles"] = {}; rel = r["relief"]
    else:
        r = selfcal(v, rounds=a.rounds, every=a.every, fixed=(0.0, 0.0, 0.0, 0.0), log=lg)
        rel = r["relief"]
    if rel is None:                                          # nothing raised: the house's floor is level
        from detect_changes2 import RES, HX, HY
        rel = np.zeros((int(round(2 * HY / RES)), int(round(2 * HX / RES))), np.float32)
    p = frl.save(Path(a.out).parent, rel, a.frame, views=[str(x) for x in a.views], views_calibration=st, rounds=a.rounds,
                 tiles=len(r["relief_tiles"]), raised_tiles=sum(1 for z in r["relief_tiles"].values() if z > 0),
                 relief_tiles=r["relief_tiles"], mode=a.mode, step="floor_selfcal.py relief")
    if Path(a.out).resolve() != p.resolve(): raise SystemExit(f"--out must be the run directory's {frl.RELIEF_FILE}")
    print(f"floor relief: {sum(1 for z in r['relief_tiles'].values() if z > 0)} of {len(r['relief_tiles'])} tiles raised -> {p} ({time.time() - t0:.0f} s)")


def check_result(r, tol=None, min_regs=CHECK_MIN_REGS):
    """the nightly check's verdict on a free self-calibration (selfcal's dict) of views painted with the robot's
    calibration: the change it finds is the night's disagreement with the record. {change, registrations, lag_seen,
    warnings (past tol), judged (False: too little floor)}"""
    tol = dict(CHECK_TOL, **(tol or {}))
    regs = int(r["rounds"][-1]["registrations"]); lag_seen = r["rounds"][0]["seen"].get("dlag_s", 0.0) != 0.0
    ch = {"pitch_deg": r["dpitch_deg"], "yaw_deg": r["dyaw_deg"], "up_m": r["dup_m"], "lag_s": r["dlag_s"] if lag_seen else None}
    out = {"change": ch, "registrations": regs, "lag_seen": lag_seen, "tolerance": tol, "warnings": [], "judged": regs >= min_regs}
    if not out["judged"]:
        out["note"] = f"too little floor to check ({regs} tile registrations, needs {min_regs}): not judged"; return out
    unit = {"pitch_deg": (1, "deg"), "yaw_deg": (1, "deg"), "up_m": (1000, "mm"), "lag_s": (1000, "ms")}
    for k, t in tol.items():
        v = ch.get(k)
        if v is None or abs(v) <= t: continue
        f, u = unit[k]
        out["warnings"].append(f"the floor's {k.split('_')[0]} disagrees with the robot's calibration by {v * f:+.{1 if f > 1 else 3}f} {u} (over {t * f:g} {u})")
    out["note"] = "; ".join(out["warnings"]) or "agrees with the robot's calibration"
    return out


def check_main(a):
    """the nightly check ([9tQ]): reports only, never writes the robot's calibration"""
    import confirm_stream as cs
    t0 = time.time(); v = cs.Views.load(a.views); st = views_stamp(a.views)
    r = selfcal(v, rounds=a.rounds, every=a.every, log=lambda m: print(f"{m} ({time.time() - t0:.0f} s)", flush=True))
    out = check_result(r)
    out.update(views=[str(x) for x in a.views], views_calibration=st, made=time.strftime("%Y-%m-%d %H:%M"), rounds=a.rounds,
               every=a.every, wall_s=round(time.time() - t0))
    Path(a.out).write_text(json.dumps(out, indent=1))
    c = out["change"]; lag = "unseen" if c["lag_s"] is None else f"{c['lag_s'] * 1000:+.1f} ms"
    print(f"calibration check: {out['note']} (pitch {c['pitch_deg']:+.3f} deg, yaw {c['yaw_deg']:+.3f} deg, up {c['up_m'] * 1000:+.1f} mm, "
          f"floor lag {lag}; {out['registrations']} registrations) -> {a.out}")


def main(argv=None):
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("relief"); r.add_argument("views", nargs="+"); r.add_argument("--out", required=True)
    r.add_argument("--frame", nargs="*", default=[], help="the pose-corrections files the views were painted in (the frame the relief lies in)")
    r.add_argument("--rounds", type=int, default=3); r.add_argument("--every", type=int, default=1)
    r.add_argument("--mode", default="tiles", choices=["tiles", "surface"], help="tiles: one height per 0.6 m tile, raised only (7 Oct); surface: a height every 10 cm, both ways, from view pairs (selfcal_surface)")
    c = sub.add_parser("check"); c.add_argument("views", nargs="+"); c.add_argument("--out", required=True)
    c.add_argument("--rounds", type=int, default=3); c.add_argument("--every", type=int, default=1)
    a = ap.parse_args(argv)
    {"relief": relief_main, "check": check_main}[a.cmd](a)


if __name__ == "__main__":
    main()
