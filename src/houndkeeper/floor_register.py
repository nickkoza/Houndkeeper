"""The last centimetres of a frame's pose, from the floor itself ([KkQ]; [FHA]): the lidar's pose graph places frames
to 2-3 cm, and the floor comparison needs millimetres. A short run of frames (a batch, ~0.5 s) is projected onto the
floor as one patch, and slid and turned over the reference floor picture (the house's earlier night) until its detail
lines up: masked normalised correlation over all shifts at once (FFT), for a few small turns, coarse (3 cm blur) then
fine (7 mm). Matched on edge strength, so lighting and exposure don't matter; the slate tiles, the carpet edge and
the grout give it plenty to hold. A batch whose best match is weak keeps the pose it came with.

A frame here is (time, grey image straightened by lens.head, camera pose world <- optical 4x4, (fx, fy, cx, cy))."""
import math

import numpy as np

HX, HY, RES = 8.4, 4.9, 0.01                        # the floor picture's grid (floor_mosaic2's)
W_, H_ = int(2 * HX / RES), int(2 * HY / RES)
RANGE = (0.4, 2.2)                                  # m from the camera's foot: nearer is under the body, farther too coarse
MIN_SCORE = 0.25                                    # a weaker best match keeps the incoming pose


def shift_corr(F, Fm, M, Mm, maxs):
    """the shift (rows, cols) by which patch F (mask Fm) best matches map window M (mask Mm), F[i] ~ M[i + shift], by
    correlation normalised over where both have data (Padfield's masked NCC, all shifts at once by FFT), to a fraction
    of a cell (a parabola through the peak): (score, rows, cols)"""
    from scipy.signal import fftconvolve
    H, W = F.shape; fm, mm = Fm.astype(float), Mm.astype(float); A, B = F * fm, M * mm
    corr = lambda a, b: fftconvolve(b, a[::-1, ::-1], mode="full")[H - 1 - maxs:H + maxs, W - 1 - maxs:W + maxs]
    n = corr(fm, mm); sab = corr(A, B); sa = corr(A, mm); sb = corr(fm, B); saa = corr(A * F, mm); sbb = corr(fm, B * M)
    n_ = np.maximum(n, 1.0); num = sab - sa * sb / n_
    den = np.sqrt(np.maximum(saa - sa ** 2 / n_, 0) * np.maximum(sbb - sb ** 2 / n_, 0))
    c = np.where((n > 400) & (den > 1e-6), num / np.maximum(den, 1e-9), -2.0)
    k = np.unravel_index(np.argmax(c), c.shape); best = c[k]
    def sub(m_, c0, p_):
        d_ = m_ - 2 * c0 + p_; return 0.0 if d_ == 0 else float(np.clip(0.5 * (m_ - p_) / d_, -0.5, 0.5))
    fr = sub(c[k[0] - 1, k[1]], best, c[k[0] + 1, k[1]]) if 0 < k[0] < c.shape[0] - 1 else 0.0
    fc = sub(c[k[0], k[1] - 1], best, c[k[0], k[1] + 1]) if 0 < k[1] < c.shape[1] - 1 else 0.0
    return float(best), k[0] - maxs + fr, k[1] - maxs + fc


def floor_points(img, T, K, step=2):
    """a frame's floor pixels as world points (x, y) and grey values"""
    fx, fy, cx, cy = K; H, W = img.shape[:2]
    v, u = np.mgrid[H // 3:H:step, 0:W:step]
    d = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1).reshape(-1, 3) @ T[:3, :3].T
    C = T[:3, 3]; ok = d[:, 2] < -1e-3; s = -C[2] / np.where(ok, d[:, 2], -1.0); P = C[:2] + d[:, :2] * s[:, None]
    r = np.hypot(P[:, 0] - C[0], P[:, 1] - C[1]); ok &= (r >= RANGE[0]) & (r < RANGE[1]) & (img[v.ravel(), u.ravel()] > 0)
    return P[ok], img[v.ravel(), u.ravel()][ok].astype(np.float32)


def _edges(m, mask, sigma):
    from scipy.ndimage import gaussian_filter
    num = gaussian_filter(np.where(mask, m, 0.0), sigma); den = gaussian_filter(mask.astype(float), sigma)
    b = np.where(den > 0.3, num / np.maximum(den, 1e-6), 0.0); ok = den > 0.3
    e = np.hypot(*np.gradient(b)); ok = ok & np.roll(ok, 1, 0) & np.roll(ok, -1, 0) & np.roll(ok, 1, 1) & np.roll(ok, -1, 1)
    return e, ok


def register_batch(P, g, ref, ref_mask, turns_deg=(-1.0, -0.5, 0.0, 0.5, 1.0)):
    """the correction (dx, dy, dyaw about the batch's centre, score) laying the batch's floor points P (world x, y) with
    grey g onto the reference picture ref (grey, NaN-free where ref_mask)"""
    if len(P) < 2000: return None
    ctr = P.mean(0); pad = 12
    c = np.floor((P[:, 0] + HX) / RES).astype(int); r = np.floor((HY - P[:, 1]) / RES).astype(int)
    r0, r1, c0, c1 = max(r.min() - pad, 0), min(r.max() + pad, H_), max(c.min() - pad, 0), min(c.max() + pad, W_)
    if r1 - r0 < 30 or c1 - c0 < 30: return None
    Mw, Mm = ref[r0:r1, c0:c1], ref_mask[r0:r1, c0:c1]
    if Mm.sum() < 3000: return None
    def patch(dyaw):
        cs, sn = math.cos(dyaw), math.sin(dyaw); Q = (P - ctr) @ np.array([[cs, sn], [-sn, cs]]) + ctr
        cc = np.floor((Q[:, 0] + HX) / RES).astype(int) - c0; rr = np.floor((HY - Q[:, 1]) / RES).astype(int) - r0
        k = (rr >= 0) & (rr < r1 - r0) & (cc >= 0) & (cc < c1 - c0)
        F = np.zeros(Mw.shape); n = np.zeros(Mw.shape); np.add.at(F, (rr[k], cc[k]), g[k]); np.add.at(n, (rr[k], cc[k]), 1)
        return np.where(n > 0, F / np.maximum(n, 1), 0.0), n > 0
    best = None
    for sigma, maxs, turns in ((3.0, 8, [math.radians(t) for t in turns_deg]), (0.7, 3, None)):
        if turns is None: turns = [best[1] + math.radians(d) for d in (-0.25, 0.0, 0.25)]
        Me, Mo = _edges(Mw, Mm, sigma); cand = None
        for dy in turns:
            F, Fm = patch(dy); Fe, Fo = _edges(F, Fm, sigma)
            sc, dr, dc = shift_corr(Fe, Fo, Me, Mo, maxs)
            if cand is None or sc > cand[0]: cand = (sc, dy, dr, dc)
        best = cand if best is None else (cand[0], cand[1], cand[2] + 0.0, cand[3] + 0.0)
    sc, dyaw, dr, dc = best
    return (float(dc * RES), float(-dr * RES), float(dyaw), float(sc)) if sc >= MIN_SCORE else None


def reference(picture_npz):
    """a floor picture (floor_mosaic2's .npz) as grey and its seen mask"""
    z = np.load(picture_npz); rgb = z["rgb"].astype(np.float32); m = rgb.sum(-1) > 0
    return np.where(m, rgb.mean(-1), 0.0), m


SNAP_MAX_CELLS = 8                                  # 8 cm: the lidar's pose is 2-3 cm off at the median, 6+ in its tail
                                                    # (27 Sep, walk 70/3: at 4 cm half the locks sat on the search's edge);
                                                    # well inside the 48 cm the tiles repeat at
SNAP_TURNS_DEG = (-2.0, -1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0)   # its heading (at 1 deg, a third of the locks' edge)
SNAP_MIN_SCORE, SNAP_MIN_LEAD = 0.30, 0.08          # a lock: a good match, clearly better than any other placement


def ncc_surfaces(F, Fm, M, Mm, maxs, turns_deg, sigma=1.0):
    """the masked correlation of F's edges with M's, at every shift within +-maxs cells, for each turn of M (deg):
    [(turn, surface (2 maxs + 1)^2)], or None when there is too little of both to match. surface[i, j] compares F with
    M shifted: M[r + (i - maxs), c + (j - maxs)] lies under F[r, c]; -2 where too little overlaps"""
    from scipy import ndimage as ndi
    from scipy.signal import fftconvolve
    if Fm.sum() < 2000 or Mm.sum() < 2000: return None
    Fe, Fo = _edges(F, Fm, sigma); H, W = F.shape; out = []
    for td in turns_deg:
        if td:
            Mr = ndi.rotate(np.where(Mm, M, 0.0), td, reshape=False, order=1)
            Mmr = ndi.rotate(Mm.astype(float), td, reshape=False, order=0) > 0.5
        else: Mr, Mmr = M, Mm
        Me, Mo = _edges(Mr, Mmr, sigma)
        fm, mm = Fo.astype(float), Mo.astype(float); A, B = Fe * fm, Me * mm
        corr = lambda a, b: fftconvolve(b, a[::-1, ::-1], mode="full")[H - 1 - maxs:H + maxs, W - 1 - maxs:W + maxs]
        n = corr(fm, mm); sab = corr(A, B); sa = corr(A, mm); sb = corr(fm, B); saa = corr(A * Fe, mm); sbb = corr(fm, B * Me)
        n_ = np.maximum(n, 1.0); num = sab - sa * sb / n_
        den = np.sqrt(np.maximum(saa - sa ** 2 / n_, 0) * np.maximum(sbb - sb ** 2 / n_, 0))
        out.append((td, np.where((n > 1500) & (den > 1e-6), num / np.maximum(den, 1e-9), -2.0)))
    return out


def peak_information(c, k):
    """how sharply the surface c peaks at k, direction by direction: the 2x2 curvature (per cell^2, rows then cols) of a
    quadratic fitted over the 5x5 cells around it, made non-negative (a flat or saddle direction: zero, it tells
    nothing). Along wood grain or a stripe the peak is a ridge: firm across it, nothing along it"""
    r0, c0 = k; R = []; Y = []
    for dr in range(-2, 3):
        for dc in range(-2, 3):
            i, j = r0 + dr, c0 + dc
            if 0 <= i < c.shape[0] and 0 <= j < c.shape[1] and c[i, j] > -1.5:
                R.append([1, dr, dc, 0.5 * dr * dr, dr * dc, 0.5 * dc * dc]); Y.append(c[i, j])
    if len(Y) < 6: return np.zeros((2, 2))
    q = np.linalg.lstsq(np.array(R, float), np.array(Y, float), rcond=None)[0]
    Hs = np.array([[q[3], q[4]], [q[4], q[5]]]); w, V = np.linalg.eigh(-Hs)
    return (V * np.clip(w, 0, None)) @ V.T


def surface_belief(c, tau):
    """where the surface c says the match lies, as a belief over every placement: weight exp((c - best) / tau) (a
    softmax; tau in NCC units). Returns (mean (rows, cols) in cells, 2x2 covariance in cells^2). One sharp peak: a small
    covariance; a ridge (wood grain, a stripe): long along it; two near-equal peaks (the next tile over): wide, so the
    match counts for little. Cells with too little overlap (-2) carry nothing"""
    ok = c > -1.5
    if not ok.any(): return None
    w = np.where(ok, np.exp((c - c[ok].max()) / tau), 0.0); w /= w.sum()
    R, C = np.mgrid[:c.shape[0], :c.shape[1]].astype(float)
    mu = np.array([(w * R).sum(), (w * C).sum()]); dR, dC = R - mu[0], C - mu[1]
    cov = np.array([[(w * dR * dR).sum(), (w * dR * dC).sum()], [(w * dR * dC).sum(), (w * dC * dC).sum()]])
    return mu, cov


def snap(F, Fm, M, Mm, maxs=SNAP_MAX_CELLS, turns_deg=SNAP_TURNS_DEG, sigma=1.0, lead_cells=2, turn_edge_locks=False):
    """how far a frame's floor patch F (seen where Fm) sits from the reference M (where Mm) ([KkQ]; the robot
    should see for itself that it is, say, two centimetres off): the turn about the patch's centre and the shift (rows, cols) that
    best lay the reference's edges on the patch's, by masked correlation over every placement. Returns (dyaw rad, rows,
    cols, score, lead), lead being how much better the best placement is than the best one 2+ cells from it; or None
    when there is too little of both to match. A lock is score >= SNAP_MIN_SCORE and lead >= SNAP_MIN_LEAD: carpet,
    whose pile looks different every night, gives no clear best and is compared unsnapped"""
    S = ncc_surfaces(F, Fm, M, Mm, maxs, turns_deg, sigma)
    if S is None: return None
    best = None
    for td, c in S:
        k = np.unravel_index(np.argmax(c), c.shape)
        if best is None or c[k] > best[0]: best = (float(c[k]), td, k, c)
    sc, td, k, c = best
    if sc <= -1.0: return None
    if k[0] in (0, c.shape[0] - 1) or k[1] in (0, c.shape[1] - 1) or (td in (turns_deg[0], turns_deg[-1]) and not turn_edge_locks):
        return np.radians(td), float(k[0] - maxs), float(k[1] - maxs), sc, 0.0   # on the search's edge: no peak, no lock
    rr, cc = np.mgrid[:c.shape[0], :c.shape[1]]; far = np.hypot(rr - k[0], cc - k[1]) >= lead_cells   # "clearly best": over
                                                    # what lies lead_cells away (at 1 cm, peaks are a few cells broad)
    lead = sc - float(c[far].max()) if far.any() else 0.0
    # c[i, j] compares F with M shifted: M[r + (i - maxs), c + (j - maxs)] lies under F[r, c]
    return np.radians(td), float(k[0] - maxs), float(k[1] - maxs), sc, lead
