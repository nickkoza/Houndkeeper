#!/usr/bin/env python3
"""Floor-mapping precision bench ([d8d]'s floor comparison: a precise floor map
removes many false identifications): map one house's empty floor twice, from two
different routes, and measure how well the two floor pictures agree. Two maps of an unchanged floor should match;
wherever they don't, the walk-through sees change that isn't there (the rug's pattern and the carpet-wood seam, 26 Sep).

Standalone and fast: the house scene alone, cameras placed along the routes (no robot, no DimOS), Filament colour and
MuJoCo depth (to skip floor hidden behind furniture). Each floor cell keeps its sharpest view: the smallest footprint.
Error sources are switched in one at a time: pose noise (the robot's estimate of where the camera was), the camera
(the head camera, or the stowed wrist camera looking down), and the lamp (on for one map, off for the other).
Usage: bench_mapping.py --seed 9916 [--camera head|wrist] [--pose-noise 0.02,0.5,0.5] [--lamp-flip] (dimos venv)"""
import argparse, json, math, os, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent; HK = HERE.parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src")); from houndkeeper import workspace   # noqa: E402  (decision [cy4])
DIMOS = workspace.dimos()
RES, HX, HY = 0.01, 8.4, 4.9                                   # the floor picture's grid, as the walk-through's
W_, H_ = int(2 * HX / RES), int(2 * HY / RES)
CAMERAS = {   # where each camera sits on the robot (forward, up, m), how far it looks down (deg), its lens and image
    "head": dict(fwd=0.30, up=0.29, pitch=0.0, fovy=51.8, size=(640, 360)),   # params/robot.yaml; the sim's walking height
    "wrist": dict(fwd=0.25, up=0.59, pitch=41.0, fovy=65.0, size=(640, 480)),  # the stowed arm (measured in the arm sim, 26 Sep)
    # other wrist poses the arm reaches (sim/arm kinematics, 26 Sep; the wrist camera moves only if that
    # gives substantially better results): the wrist tilted down, and the arm lifted (its mass up to 10 cm higher)
    "wrist56": dict(fwd=0.26, up=0.54, pitch=56.0, fovy=65.0, size=(640, 480)),    # wrist -0.25 rad
    "wrist70": dict(fwd=0.26, up=0.50, pitch=70.0, fovy=65.0, size=(640, 480)),    # wrist -0.50 rad
    "lift41": dict(fwd=0.32, up=0.74, pitch=41.0, fovy=65.0, size=(640, 480)),     # joints 2/3 -0.9/+0.9
    "lift59": dict(fwd=0.33, up=0.69, pitch=59.0, fovy=65.0, size=(640, 480)),     # and the wrist -0.3
}
EDGES = {   # the fixed transitions measured to the millimetre ([FHA]): carpet meets the hard floor along these lines
    # exactly (make_house.CARPET, drawn at the floor texture's 3 mm): (axis the line is at, where, span along it)
    "seam_x": ("x", -2.100, (-2.0, 1.5)), "seam_y": ("y", -2.100, (-7.8, -2.2))}
ACCEPT = (0.35, 0.03)           # a correction is taken when the frame matches the map at least this well, and this
                                # much better than where it was believed to be (est.)
BAND, FINE = 0.06, 0.002        # each edge mapped in a band this far either side, on a grid this fine (m)
AREA = (-7.8, -3.8, -3.2, 1.6)                                 # the living room: carpet, the rug, the seam to the wood
RUG = dict(x=-4.6, y=-0.2, w=2.2, h=1.5); SEAM_X = -8.4 + 17.5 * 0.35


def route(lane_offset, reverse, step=0.10, lanes=0.5):
    """a lawnmower over AREA: (x, y, yaw) every `step` m along lanes `lanes` apart, starting `lane_offset` in"""
    x0, y0, x1, y1 = AREA; out = []
    for k, y in enumerate(np.arange(y0 + lane_offset, y1, lanes)):
        xs = np.arange(x0, x1, step); east = (k % 2 == 0) != reverse
        for x in (xs if east else xs[::-1]): out.append((x, y, 0.0 if east else math.pi))
    return out


BENCH_TAG = "_bench"      # the bench's own copy of the house: the nights running beside it rewrite theirs (27 Sep: a
                          # bench run read a scene a queued night had just regenerated flat)


def house(seed):
    """build the bench's private copy of house `seed` as the nights have it (indoor, photographed floors, night light,
    chairs, a pet's bowls, relief), once per seed; returns the scene's path"""
    import subprocess
    path = DIMOS / "data" / "mujoco_sim" / f"scene_nights{BENCH_TAG}_s{seed}_n1.xml"
    if not getattr(house, "done", {}).get(seed):
        subprocess.run([sys.executable, str(HERE / "make_nights.py"), "--seed", str(seed), "--nights", "1", "--per-night", "0", "--indoor",
                        "--photo-floors", "--regime", "lit", "--movables", "--pets", "--tag", BENCH_TAG], check=True, capture_output=True, cwd=HERE)
        house.done = {**getattr(house, "done", {}), seed: True}
    return path


def load(seed, lamp):
    import mujoco
    sys.path.insert(0, str(DIMOS)); from dimos.simulation.mujoco.model import get_assets
    spec = mujoco.MjSpec.from_string(house(seed).read_text(), assets=get_assets())
    spec.worldbody.add_camera(name="bench", pos=[0, 0, 1], fovy=45)
    m = spec.compile(); d = mujoco.MjData(m)
    for i in range(m.nlight):
        if m.light(i).name.startswith("lamp"): m.light_active[i] = int(lamp)
    return m, d


def cam_pose(cam, x, y, yaw, p):
    """the camera's position and axes (x right, y up, looking along -z) on a robot at (x, y, yaw), looking p rad down"""
    c = CAMERAS[cam]; f = np.array([math.cos(yaw) * math.cos(p), math.sin(yaw) * math.cos(p), -math.sin(p)])
    r = np.array([math.sin(yaw), -math.cos(yaw), 0.0]); u = np.cross(r, f)
    return np.array([x + c["fwd"] * math.cos(yaw), y + c["fwd"] * math.sin(yaw), c["up"]]), np.stack([r, u, -f], 1)


DELAY = None         # (mean s, jitter s, speed m/s): frames stamped late while walking (--delay)
MOUNT_ERR = None     # (pitch deg, yaw deg, pos m [3]): the camera truly sits off where it's believed to (--mount-error)


def place(m, d, cam, x, y, yaw, noise=None, rng=None, true=False):
    """the camera where the robot at (x, y, yaw) carries it; noise (xy m, yaw deg, pitch deg): where it's believed to be.
    true: where it really is (the mount's own error on top). Returns the believed pose as (x, y, yaw, pitch rad)"""
    import mujoco
    c = CAMERAS[cam]; p = math.radians(c["pitch"])
    if true and MOUNT_ERR:
        C, R = cam_pose(cam, x, y, yaw + math.radians(MOUNT_ERR[1]), p + math.radians(MOUNT_ERR[0]))
        C = C + np.array([math.cos(yaw) * MOUNT_ERR[2][0] - math.sin(yaw) * MOUNT_ERR[2][1], math.sin(yaw) * MOUNT_ERR[2][0] + math.cos(yaw) * MOUNT_ERR[2][1], MOUNT_ERR[2][2]])
    else: C, R = cam_pose(cam, x, y, yaw, p)
    cid = m.camera("bench").id; m.cam_pos[cid] = C; q = np.zeros(4); mujoco.mju_mat2Quat(q, R.ravel()); m.cam_quat[cid] = q
    m.cam_fovy[cid] = c["fovy"]; mujoco.mj_forward(m, d)
    if noise is None: return (x, y, yaw, p)
    dx, dy = rng.normal(0, noise[0], 2)
    return (x + dx, y + dy, yaw + math.radians(rng.normal(0, noise[1])), p + math.radians(rng.normal(0, noise[2])))


def _shift_corr(F, Fm, M, Mm, maxs):
    """the shift (rows, cols) by which patch F (mask Fm) best matches map window M (mask Mm), F[i] ~ M[i + shift], by
    correlation normalised over where both have data (Padfield's masked NCC, all shifts at once by FFT), to a fraction
    of a cell (a parabola through the peak)"""
    from scipy.signal import fftconvolve
    H, W = F.shape; fm, mm = Fm.astype(float), Mm.astype(float); A, B = F * fm, M * mm
    corr = lambda a, b: fftconvolve(b, a[::-1, ::-1], mode="full")[H - 1 - maxs:H + maxs, W - 1 - maxs:W + maxs]
    n = corr(fm, mm); sab = corr(A, B); sa = corr(A, mm); sb = corr(fm, B); saa = corr(A * F, mm); sbb = corr(fm, B * M)
    n_ = np.maximum(n, 1.0)
    num = sab - sa * sb / n_; den = np.sqrt(np.maximum(saa - sa ** 2 / n_, 0) * np.maximum(sbb - sb ** 2 / n_, 0))
    c = np.where((n > 400) & (den > 1e-6), num / np.maximum(den, 1e-9), -2.0)
    k = np.unravel_index(np.argmax(c), c.shape); best = c[k]; dr, dc = k[0] - maxs, k[1] - maxs
    def sub(m_, c0, p_):
        d_ = m_ - 2 * c0 + p_; return 0.0 if d_ == 0 else float(np.clip(0.5 * (m_ - p_) / d_, -0.5, 0.5))
    fr = sub(c[k[0] - 1, k[1]], best, c[k[0] + 1, k[1]]) if 0 < k[0] < c.shape[0] - 1 else 0.0
    fc = sub(c[k[0], k[1] - 1], best, c[k[0], k[1] + 1]) if 0 < k[1] < c.shape[1] - 1 else 0.0
    return float(best), dr + fr, dc + fc, float(c[maxs, maxs])


REG_FEATURE = "edges"        # what frames are matched on: "edges" (how strongly brightness changes, blurred) or "intensity"


def register(cam, belief, rays, gray, sample, mean_map, H0):
    """the pose correction that best lays this frame's floor onto the map so far, coarse to fine: the frame's floor is
    projected to its own patch of cells for each candidate tilt and heading, and slid over the map (both blurred 3 cm,
    then 7 mm) for the position. Scored by correlation where both have floor. A single sharp pass locked onto false
    peaks of the carpet's pile and drifted (rug correlation 0.95 -> 0.06, 26 Sep)"""
    from scipy.ndimage import gaussian_filter
    x0, y0, yaw0, p0 = belief; rs = rays[sample]; gs = gray[sample]
    def patch(dyaw, dp):
        C, R = cam_pose(cam, x0, y0, yaw0 + dyaw, p0 + dp)
        dw = rs @ R.T; ok = dw[:, 2] < -1e-3; t = np.where(ok, -C[2] / np.where(ok, dw[:, 2], -1), np.inf)
        P = C[None, :2] + dw[:, :2] * t[:, None]; ok &= t < 2.5
        c = np.floor((P[:, 0] + HX) / RES).astype(int); r = np.floor((HY - P[:, 1]) / RES).astype(int)
        ok &= (c >= 0) & (c < W_) & (r >= 0) & (r < H_)
        return r[ok], c[ok], gs[ok]
    r, c, _ = patch(0.0, 0.0)
    if len(r) < 500: return (0.0, 0.0, 0.0, 0.0)
    pad = 8; r0, r1, c0, c1 = max(r.min() - pad, 0), min(r.max() + pad, H_), max(c.min() - pad, 0), min(c.max() + pad, W_)
    Mw = mean_map[r0:r1, c0:c1]; Mm = np.isfinite(Mw)
    if Mm.sum() < 2000: return (0.0, 0.0, 0.0, 0.0)            # little mapped under it yet: as believed
    M0 = np.where(Mm, Mw, 0.0)
    def evaluate(dyaw, dp, sigma, maxs):
        r, c, g = patch(dyaw, dp); F = np.zeros(Mw.shape); n = np.zeros(Mw.shape)
        k = (r >= r0) & (r < r1) & (c >= c0) & (c < c1); np.add.at(F, (r[k] - r0, c[k] - c0), g[k]); np.add.at(n, (r[k] - r0, c[k] - c0), 1)
        Fm = n > 0; F = np.where(Fm, F / np.maximum(n, 1), 0.0)
        def blur(X, mask):
            num = gaussian_filter(np.where(mask, X, 0.0), sigma); den = gaussian_filter(mask.astype(float), sigma)
            return np.where(den > 0.3, num / np.maximum(den, 1e-6), 0.0), den > 0.3
        Fb, Fbm = blur(F, Fm); Mb, Mbm = blur(M0, Mm)
        if REG_FEATURE == "edges":                                 # matched on edges: the rug, seams and legs decide the
            Fb = np.hypot(*np.gradient(Fb)); Mb = np.hypot(*np.gradient(Mb))   # fit, not the carpet's pile or its shading
            Fbm = Fbm & ~(Fbm ^ np.roll(Fbm, 1, 0)) & ~(Fbm ^ np.roll(Fbm, 1, 1))   # (not the patch's own border)
        return _shift_corr(Fb, Fbm, Mb, Mbm, maxs)
    best = None; d = math.radians(0.5)
    for dyaw in (-2 * d, -d, 0.0, d, 2 * d):                     # coarse: 3 cm blur, +-6 cm
        for dp in (-2 * d, -d, 0.0, d, 2 * d):
            sc, dr, dc, _ = evaluate(dyaw, dp, 3.0, 6)
            if best is None or sc > best[0]: best = (sc, dyaw, dp, dr, dc)
    _, byaw, bp, _, _ = best; best = None; d2 = math.radians(0.125)
    for dyaw in (byaw - 2 * d2, byaw - d2, byaw, byaw + d2, byaw + 2 * d2):   # fine: 7 mm blur, +-2 cm
        for dp in (bp - 2 * d2, bp - d2, bp, bp + d2, bp + 2 * d2):
            sc, dr, dc, _ = evaluate(dyaw, dp, 0.7, 2 + 4)
            if best is None or sc > best[0]: best = (sc, dyaw, dp, dr, dc)
    sc, dyaw, dp, dr, dc = best
    stay = evaluate(0.0, 0.0, 0.7, 2 + 4)[3]                    # the frame where it's believed to be
    if sc < ACCEPT[0] or sc - stay < ACCEPT[1]:                 # moved only on clear evidence: on dark, plain carpet the
        return (0.0, 0.0, 0.0, 0.0)                             # match has no peak (heading trades against a sideways
                                                                # shift), and a frame moved on a guess corrupts the map
                                                                # every later frame is laid onto (26 Sep: 12 -> 45 mm)
    # the frame's floor at cell i is the map's at i + (dr, dc): move the camera by that (cells run +x; rows run -y)
    return (dc * RES, -dr * RES, dyaw, dp)


def grids():
    """the house at RES, and a fine band around each edge: (name, left x, top y, res, width, height)"""
    out = [("house", -HX, HY, RES, W_, H_)]
    for name, (ax, at, (s0, s1)) in EDGES.items():
        if ax == "x": out.append((name, at - BAND, s1, FINE, int(2 * BAND / FINE), int((s1 - s0) / FINE)))
        else: out.append((name, s0, at + BAND, FINE, int((s1 - s0) / FINE), int(2 * BAND / FINE)))
    return out


def build_map(m, d, cam, poses, noise, seed, fixed_ev=False, average=True, blend=False, reg=False, acc_in=None, ref=None, drift=None):
    """the floor pictures from these poses (the house, and each edge's fine band): per cell the colour of its sharpest
    (smallest-footprint) view, every pixel landing in it averaged"""
    import mujoco
    sys.path.insert(0, str(HK / "src")); from houndkeeper import filament_cameras as fc
    w, h = CAMERAS[cam]["size"]; f = (h / 2) / math.tan(math.radians(CAMERAS[cam]["fovy"]) / 2)
    uu, vv = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    rays = np.stack([(uu - w / 2) / f, -(vv - h / 2) / f, -np.ones_like(uu)], -1).reshape(-1, 3)
    depth = mujoco.Renderer(m, h, w); depth.enable_depth_rendering()
    G = grids(); maps = {g[0]: (np.zeros((g[5], g[4], 3), np.float32), np.full((g[5], g[4]), np.inf, np.float32)) for g in G}
    acc = acc_in or ({g[0]: (np.zeros((g[5], g[4], 3), np.float64), np.zeros((g[5], g[4]), np.float64)) for g in G} if blend else None)
    rng = np.random.default_rng(seed); ev = None; diag = []
    walk = np.zeros(3); last = None                                # odometry's drift: a random walk in x, y and heading
    drng = np.random.default_rng(seed + 1000)                      # that grows with distance walked (pitch stays: the IMU)
    fc._BY_SIZE.clear()
    for k, (x, y, yaw) in enumerate(poses):
        place(m, d, cam, x, y, yaw, true=True)                       # render from where the camera really is...
        img = fc.render(m, d, "bench", w, h).astype(np.float32).reshape(-1, 3)
        if fixed_ev: fc.AE_RATE = 0.0                                 # metered once, then held: exposure out of the question
        depth.update_scene(d, "bench"); z = depth.render().reshape(-1)
        belief = place(m, d, cam, x, y, yaw, noise, rng) if noise else place(m, d, cam, x, y, yaw)   # ...project from belief
        if DELAY:                                                    # stamped late: the pose looked up is where the robot is a
            dl = DELAY[0] + drng.normal(0, DELAY[1]); nx, ny, nyaw = poses[min(k + 1, len(poses) - 1)]   # moment later
            stp = math.hypot(nx - x, ny - y); dt_step = max(stp, 1e-3) / DELAY[2]   # at walking speed DELAY[2]
            turn = (nyaw - yaw + math.pi) % (2 * math.pi) - math.pi
            belief = (belief[0] + (nx - x) * dl / dt_step, belief[1] + (ny - y) * dl / dt_step, belief[2] + turn * dl / dt_step, belief[3])
        if drift:
            step = 0.0 if last is None else math.hypot(x - last[0], y - last[1]); last = (x, y)
            walk += drng.normal(0, 1, 3) * np.sqrt(step) * np.array([drift[0], drift[0], math.radians(drift[1])])
            belief = (belief[0] + walk[0], belief[1] + walk[1], belief[2] + walk[2], belief[3])
        if reg:                                                      # laid onto the map so far ([FHA])
            if ref is not None: mean_map = ref                        # the baseline: a complete map from earlier nights
            else: s_, w_ = acc["house"]; mean_map = np.where(w_ > 0, s_.mean(-1) / np.maximum(w_, 1e-12), np.nan)
            Cb, Rb = cam_pose(cam, *belief); dwb = rays @ Rb.T                 # floor only: pixels whose depth agrees with the
            tb = np.where(dwb[:, 2] < -1e-3, -Cb[2] / np.minimum(dwb[:, 2], -1e-3), np.inf)   # floor at the believed pose (the
            floor = np.isfinite(tb) & (np.abs(z - tb) < 0.08 * tb) & (tb < 2.5)             # lidar's job on the robot). Walls
            gray = img.mean(-1); sample = np.nonzero(floor & (np.arange(len(rays)) % 7 == 0) & (gray > 3))[0]   # projected
                                                                                  # onto the floor had made nonsense (26 Sep)
            corr = register(cam, belief, rays, gray, sample, mean_map, None)
            truth = (x, y, yaw, math.radians(CAMERAS[cam]["pitch"])); before = np.subtract(belief, truth)
            belief = (belief[0] + corr[0], belief[1] + corr[1], belief[2] + corr[2], belief[3] + corr[3])
            diag.append((before, np.subtract(belief, truth), any(corr)))
        C, R = cam_pose(cam, *belief)
        dw = rays @ R.T; ok = dw[:, 2] < -1e-3
        t = np.where(ok, -C[2] / np.where(ok, dw[:, 2], -1), np.inf)
        visible = ok & (z >= t * 0.97) & (t < 3.0)                 # floor, not furniture in front of it; within 3 m
        P = C[None, :2] + dw[:, :2] * t[:, None]
        foot = t * np.linalg.norm(dw, axis=1) / f / np.maximum(-dw[:, 2] / np.linalg.norm(dw, axis=1), 1e-3)   # footprint (m)
        for name, x0, y1, res, gw, gh in G:
            rgb, best = maps[name]
            c = np.floor((P[:, 0] - x0) / res).astype(int); r = np.floor((y1 - P[:, 1]) / res).astype(int)
            idx = np.nonzero(visible & (c >= 0) & (c < gw) & (r >= 0) & (r < gh))[0]
            if not len(idx): continue
            key = r[idx] * gw + c[idx]; cells, inv, n = np.unique(key, return_inverse=True, return_counts=True)
            if average:                                            # every pixel landing in a cell, averaged: one
                col = np.stack([np.bincount(inv, img[idx, k]) for k in range(3)], 1) / n[:, None]   # pixel alone aliased
            else:                                                  # the carpet's pile and the rug's edges (26 Sep)
                col = np.zeros((len(cells), 3)); col[inv] = img[idx]
            fp = np.bincount(inv, foot[idx]) / n; rr, cc = cells // gw, cells % gw
            if blend:                                              # every view, weighted by how sharp it is: one
                wgt = 1.0 / fp ** 2                                # frame's pose error averages out with the others'
                acc[name][0][rr, cc] += col * wgt[:, None]; acc[name][1][rr, cc] += wgt
                best[rr, cc] = np.minimum(best[rr, cc], fp); continue
            better = fp < best[rr, cc]                             # the cell's sharpest frame
            rgb[rr[better], cc[better]] = col[better]; best[rr[better], cc[better]] = fp[better]
    if diag:                                                       # how much of each frame's error alignment removed
        b_ = np.array([d_[0] for d_ in diag if d_[2]]); a_ = np.array([d_[1] for d_ in diag if d_[2]])
        if len(b_):
            f_ = lambda e: f"xy {np.sqrt((e[:, :2] ** 2).sum(1)).mean() * 1000:.1f} mm, yaw {np.degrees(np.abs(e[:, 2])).mean():.2f} deg, pitch {np.degrees(np.abs(e[:, 3])).mean():.2f} deg"
            print(f"  aligned {len(b_)} of {len(diag)} frames: error before {f_(b_)}; after {f_(a_)}")
    if blend:
        for name, (rgb, best) in maps.items():
            s_, w_ = acc[name]; seen = w_ > 0; rgb[seen] = s_[seen] / w_[seen, None]; best[seen & ~np.isfinite(best)] = 1.0
    build_map.acc = acc
    return maps


def edge_signal(rgb):
    """what an edge between carpet and hard floor is found in: red minus blue. The wood is orange and the carpets grey or
    navy, so it steps cleanly at the seam, while the wood's dark planks, which pulled a brightness fit by up to 2-3 cm
    beside the seam (and made a straight edge look jogged, 26 Sep), barely move it"""
    rgb = np.asarray(rgb, float); return rgb[..., 0] - rgb[..., 2]


def steepest(p, c, win):
    """where profile p changes fastest near index c (within win), to a fraction of a cell (a parabola through the peak):
    the edge. A shading ramp across it barely moves this; it moved the halfway crossing by up to 20 mm (26 Sep)"""
    g = np.abs(np.convolve(np.gradient(p), np.ones(3) / 3, "same")); lo, hi = max(c - win, 1), min(c + win, len(p) - 2)
    if hi <= lo: return None
    j = lo + int(np.argmax(g[lo:hi]))
    a, b_, c_ = g[j - 1], g[j], g[j + 1]; den = a - 2 * b_ + c_
    return j + (0.5 * (a - c_) / den if den != 0 else 0.0)


def fit_edge(p, c, win):
    """the edge in profile p near index c (within win), to a fraction of a cell: a blurred step on a shading ramp fitted
    to it (least squares), its centre. Robust to both at once: the halfway crossing was pulled up to 20 mm by shading,
    the steepest point wandered on a blurred edge (27 mm p95 under pose error, 26 Sep)"""
    from scipy.optimize import curve_fit
    from scipy.special import erf
    j0 = steepest(p, c, win)
    if j0 is None: return None
    lo_, hi_ = max(int(j0) - win, 0), min(int(j0) + win + 1, len(p)); x = np.arange(lo_, hi_, dtype=float); y = p[lo_:hi_]
    if len(x) < 8: return None
    model = lambda x, a, b, h, x0, s: a + b * (x - j0) + h * 0.5 * (1 + erf((x - x0) / (np.sqrt(2) * np.abs(s) + 1e-6)))
    try:
        (a, b, h, x0, s_), _ = curve_fit(model, x, y, p0=[y[:3].mean(), 0.0, y[-3:].mean() - y[:3].mean(), j0, 2.0], maxfev=2000)
    except Exception:
        return None
    return x0 if abs(x0 - j0) < win and abs(s_) < win else None


def true_line(m, d, name, rows=5):
    """where the edge really is as drawn, at each point edge_at measures (every rows x FINE along it; nan where it can't be
    told): looking straight down on it from 0.4 m (MuJoCo's renderer; Filament agrees within 0.3 mm). The drawn edge
    isn't the nominal line: the texture's 3 mm pixels put it ~2.5 mm off, and one stretch sits 13 mm further off (26 Sep:
    a single reference for the whole edge read that jog as mapping error)"""
    import mujoco
    ax, at, (s0, s1) = EDGES[name]; W, H, fovy, h = 640, 360, 30.0, 0.4; f = (H / 2) / math.tan(math.radians(fovy) / 2)
    n = int((s1 - s0) / FINE) // rows; along = (np.arange(n) * rows + rows / 2) * FINE
    pts = s0 + along if ax == "y" else s1 - along                # the band's columns run +x; its rows run down (-y)
    act, amb, dif = m.light_active.copy(), m.vis.headlight.ambient.copy(), m.vis.headlight.diffuse.copy()
    m.light_active[:] = 0; m.vis.headlight.ambient[:] = 1.0; m.vis.headlight.diffuse[:] = 0.0   # flat, even light: the
    r = mujoco.Renderer(m, H, W); cid = m.camera("bench").id; out = np.full(n, np.nan)       # texture alone (a shadow's
    half = (W / 2 - 10) / f * h if ax == "y" else (H / 2 - 10) / f * h
    for c_ in np.arange(s0 + half, s1 + half, 2 * half):
        C = np.array([at, c_, h]) if ax == "x" else np.array([c_, at, h])
        m.cam_pos[cid] = C; m.cam_quat[cid] = [1, 0, 0, 0]; m.cam_fovy[cid] = fovy; mujoco.mj_forward(m, d)   # image x = +x, up = +y
        r.update_scene(d, "bench"); g = edge_signal(r.render())
        for k in np.nonzero(np.abs(pts - c_) <= half)[0]:
            if ax == "y": u = int(W / 2 + (pts[k] - C[0]) / h * f); prof = g[:, max(u - 3, 0):u + 4].mean(1); ctr = H / 2
            else: v = int(H / 2 - (pts[k] - C[1]) / h * f); prof = g[max(v - 3, 0):v + 4].mean(0); ctr = W / 2
            c = int(ctr); lo, hi = prof[:c - 30].mean(), prof[c + 30:].mean()
            if abs(hi - lo) < 8: continue
            j = fit_edge(prof, c, int(0.03 / h * f))
            if j is None: continue
            off = (j + 0.5 - ctr) / f * h
            out[k] = off if ax == "x" else -off                  # from the nominal line; image rows run down (-y)
    r.close(); m.light_active[:] = act; m.vis.headlight.ambient[:] = amb; m.vis.headlight.diffuse[:] = dif   # gradient had
    return out                                                   # made a 13 mm "jog" of a straight edge, 26 Sep)


def edge_at(rgb, best, name, rows=5):
    """where the edge is, every `rows` x FINE along it: the point its brightness crosses halfway from the carpet's to the
    hard floor's (sub-cell, from the band's profile across it), or nan where it wasn't seen or the two sides look alike"""
    ax = EDGES[name][0]; g = edge_signal(rgb); ok = np.isfinite(best)
    if ax == "y": g, ok = g.T, ok.T                              # profiles run across the edge: columns of the band
    n = g.shape[0] // rows; out = np.full(n, np.nan)
    for k in range(n):
        prof = g[k * rows:(k + 1) * rows]; okk = ok[k * rows:(k + 1) * rows]
        if okk.mean() < 0.8: continue
        p = np.nanmean(np.where(okk, prof, np.nan), 0); m = len(p) // 2; q = int(0.02 / FINE)
        lo, hi = np.nanmean(p[:m - q]), np.nanmean(p[m + q:])
        if not np.isfinite(lo + hi) or abs(hi - lo) < 8: continue
        if not np.all(np.isfinite(p)): p = np.interp(np.arange(len(p)), np.nonzero(np.isfinite(p))[0], p[np.isfinite(p)])
        j = fit_edge(p, m, int(0.03 / FINE))
        if j is None: continue
        pos = (j + 0.5) * FINE - BAND                            # from the band's middle, i.e. from the nominal line
        out[k] = pos if ax == "x" else -pos                      # the y band's rows run downward
    return out


def compare(a, fa, b, fb):
    """how well two maps of the same floor agree, by region: patch correlation, and the share of cells that differ by more
    than 15% of their brightness (what the walk-through could read as change)"""
    both = np.isfinite(fa) & np.isfinite(fb); A, B = a.mean(-1), b.mean(-1)
    g = np.median(A[both & (B > 5)] / B[both & (B > 5)]); B = B * g        # one exposure for both (the walk-through
                                                                          # matches brightness too: floor_mosaic2 --gain)
    out = {}
    for r0 in range(0, H_ - 32, 32):
        for c0 in range(0, W_ - 32, 32):
            m = both[r0:r0 + 32, c0:c0 + 32]
            if m.mean() < 0.9: continue
            pa, pb = A[r0:r0 + 32, c0:c0 + 32], B[r0:r0 + 32, c0:c0 + 32]
            x, y = -HX + (c0 + 16) * RES, HY - (r0 + 16) * RES
            kind = "rug" if abs(x - RUG["x"]) < RUG["w"] / 2 and abs(y - RUG["y"]) < RUG["h"] / 2 else \
                   ("seam" if abs(x - SEAM_X) < 0.3 else ("carpet" if x < SEAM_X else "wood"))
            cc = np.corrcoef(pa.ravel(), pb.ravel())[0, 1] if pa.std() > 1 and pb.std() > 1 else np.nan
            diff = np.mean(np.abs(pa - pb) > 0.15 * np.maximum((pa + pb) / 2, 20))
            out.setdefault(kind, []).append((cc, diff, (fa[r0:r0 + 32, c0:c0 + 32][m].mean() + fb[r0:r0 + 32, c0:c0 + 32][m].mean()) / 2))
    return {k: dict(patches=len(v), corr=float(np.nanmedian([x[0] for x in v])), differ=float(np.mean([x[1] for x in v])),
                    footprint_mm=float(np.median([x[2] for x in v]) * 1000)) for k, v in out.items()}


def compare_pictures(seed, pa, pb):
    """the night pipeline's own floor pictures (floor_mosaic2.py, 1 cm) of the same empty house on two nights, scored as
    the bench scores its maps: cell agreement by region, and each edge placed (fitted across 1 cm cells) against the other
    night's and against the drawn edge ([FHA] measured where it matters: on the nights)"""
    A, B = np.load(pa), np.load(pb)
    fa = np.where(np.isfinite(A["dist"].astype(np.float32)), 0.005, np.inf).astype(np.float32); fb = np.where(np.isfinite(B["dist"].astype(np.float32)), 0.005, np.inf).astype(np.float32)
    res = compare(A["rgb"].astype(float), fa, B["rgb"].astype(float), fb)
    for k in ("rug", "seam", "carpet", "wood"):
        if k in res: print(f"pictures {k:6s} patches {res[k]['patches']:4d}  corr {res[k]['corr']:.2f}  cells differing {res[k]['differ']:.1%}")
    m, d = load(seed, lamp=True)
    for name, (ax, at, (s0, s1)) in EDGES.items():
        tl = true_line(m, d, name, rows=5)                           # every 1 cm along the edge, from its nominal line
        n = len(tl); along = (np.arange(n) * 5 + 2.5) * FINE
        pos = s0 + along if ax == "y" else s1 - along
        out = []
        for P in (A, B):
            g = edge_signal(P["rgb"]); ok = np.isfinite(P["dist"].astype(np.float32)); e = np.full(n, np.nan)
            for k, sp in enumerate(pos):
                if ax == "x":
                    r = int((HY - sp) / RES); c0 = int((at - BAND - (-HX)) / RES); prof, okp = g[r, c0:c0 + 13], ok[r, c0:c0 + 13]
                else:
                    c = int((sp + HX) / RES); r0 = int((HY - (at + BAND)) / RES); prof, okp = g[r0:r0 + 13, c], ok[r0:r0 + 13, c]
                if okp.mean() < 0.9: continue
                j = fit_edge(np.where(okp, prof, np.nanmean(prof)), 6, 4)
                if j is None: continue
                off = (j + 0.5) * RES - BAND; e[k] = off if ax == "x" else -off
            out.append(e)
        ea, eb = out; both = np.isfinite(ea) & np.isfinite(eb)
        if both.sum() < 5: print(f"pictures {name}: edge seen in {both.sum()} places only"); continue
        dab = np.abs(ea - eb)[both] * 1000; dtr = np.concatenate([np.abs(e - tl)[np.isfinite(e) & np.isfinite(tl)] for e in out]) * 1000
        print(f"pictures {name}: {both.sum()} points; nights agree to {np.median(dab):.1f} mm median, {np.percentile(dab, 95):.1f} mm p95; "
              f"vs the drawn edge {np.median(dtr):.1f} mm median, {np.percentile(dtr, 95):.1f} mm p95")


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--seed", type=int, default=9916); ap.add_argument("--camera", default="head", choices=list(CAMERAS))
    ap.add_argument("--pose-noise", default=None, help="xy m, yaw deg, pitch deg: the error in where each frame is believed taken")
    ap.add_argument("--lamp-flip", action="store_true", help="the lamp on for one map and off for the other")
    ap.add_argument("--step", type=float, default=0.10); ap.add_argument("--save", default=None)
    ap.add_argument("--drift", default=None, help="odometry drift per root metre walked: xy m, heading deg (a random walk; est.)")
    ap.add_argument("--ref-passes", type=int, default=0, help="map A is a baseline blended from this many passes (nights); map B, tonight, is laid onto it")
    ap.add_argument("--register", action="store_true", help="lay each frame onto the map so far before adding it (implies --blend)")
    ap.add_argument("--blend", action="store_true", help="each cell every view of it, weighted by sharpness (not the sharpest alone)")
    ap.add_argument("--one-pixel", action="store_true", help="each cell one pixel's colour, not the mean of all landing in it")
    ap.add_argument("--fixed-ev", action="store_true", help="hold the exposure after the first frame (both maps alike)")
    ap.add_argument("--toggles", default=None, help="Filament settings overridden, as JSON (e.g. to switch an effect off)")
    ap.add_argument("--mount-error", action="store_true", help="the head camera truly sits off its believed mount (params go2_air.head_camera_mount_error)")
    ap.add_argument("--delay", default=None, help="frames stamped late: mean s, jitter s, walking speed m/s (e.g. 0.08,0.02,0.3)")
    ap.add_argument("--pictures", nargs=2, default=None, help="score two of the night pipeline's floor pictures (.npz) instead")
    a = ap.parse_args()
    global MOUNT_ERR, DELAY
    if a.mount_error:
        sys.path.insert(0, str(HK / "src")); from houndkeeper import params as _P
        e = _P.get("go2_air.head_camera_mount_error"); MOUNT_ERR = (e["pitch_deg"], e["yaw_deg"], e["pos_m"])
    DELAY = [float(v) for v in a.delay.split(",")] if a.delay else None
    noise = [float(v) for v in a.pose_noise.split(",")] if a.pose_noise else None
    if a.pictures:
        os.environ.setdefault("MUJOCO_GL", "egl"); return compare_pictures(a.seed, *a.pictures)
    drift = [float(v) for v in a.drift.split(",")] if a.drift else None
    os.environ.setdefault("MUJOFIL_BACKEND", "vulkan"); os.environ.setdefault("MUJOCO_GL", "egl")
    sys.path.insert(0, str(HK / "src")); from houndkeeper import filament_cameras as fc
    if a.toggles: fc.TOGGLES.update(json.loads(a.toggles))
    rate = fc.AE_RATE
    maps = []
    if a.ref_passes:                                               # a baseline from several nights, tonight laid onto it
        acc = None; lanes = [(0.0, False), (0.25, True), (0.125, False), (0.375, True), (0.06, True), (0.31, False)]
        for k in range(a.ref_passes):
            fc.AE_RATE = rate; m, d = load(a.seed, lamp=True)
            base = build_map(m, d, a.camera, route(*lanes[k % len(lanes)], a.step), noise, seed=100 + k, fixed_ev=a.fixed_ev, blend=True, acc_in=acc, drift=drift)
            acc = build_map.acc
        maps.append(base); rgb, best = base["house"]; ref = np.where(np.isfinite(best), rgb.mean(-1), np.nan)
        fc.AE_RATE = rate; m, d = load(a.seed, lamp=not a.lamp_flip)
        maps.append(build_map(m, d, a.camera, route(0.19, True, a.step), noise, seed=7, fixed_ev=a.fixed_ev, blend=True, reg=a.register, ref=ref, drift=drift))
    for k, (off, rev) in enumerate(((0.0, False), (0.25, True)) if not a.ref_passes else ()):
        m, d = load(a.seed, lamp=(k == 0) or not a.lamp_flip)
        fc.AE_RATE = rate
        maps.append(build_map(m, d, a.camera, route(off, rev, a.step), noise, seed=k, fixed_ev=a.fixed_ev, average=not a.one_pixel, blend=a.blend or a.register, reg=a.register, drift=drift))
    res = compare(*maps[0]["house"], *maps[1]["house"])
    x0, y0, x1, y1 = AREA; c0, c1 = int((x0 + HX) / RES), int((x1 + HX) / RES); r0, r1 = int((HY - y1) / RES), int((HY - y0) / RES)
    cov = [float(np.isfinite(mp["house"][1][r0:r1, c0:c1]).mean()) for mp in maps]   # a steeper camera sees less per pass
    print(f"{a.camera:6s} coverage of the living room per map: {cov[0]:.0%} / {cov[1]:.0%}")
    m, d = load(a.seed, lamp=True)
    for name in EDGES:
        ea, eb = edge_at(*maps[0][name], name), edge_at(*maps[1][name], name); both = np.isfinite(ea) & np.isfinite(eb)
        if both.sum() < 5: print(f"{name}: edge seen in {both.sum()} places only"); continue
        tl = true_line(m, d, name)                                  # the drawn edge at each point, from the nominal line (m)
        dab = np.abs(ea - eb)[both] * 1000
        dtr = np.concatenate([np.abs(e - tl)[np.isfinite(e) & np.isfinite(tl)] for e in (ea, eb)]) * 1000
        print(f"{a.camera:6s} {name}: {both.sum()} points; maps agree to {np.median(dab):.1f} mm median, {np.percentile(dab, 95):.1f} mm p95; "
              f"vs the drawn edge {np.median(dtr):.1f} mm median, {np.percentile(dtr, 95):.1f} mm p95 ({len(dtr)} points)")
    tag = f"{a.camera}{f' ref{a.ref_passes}' if a.ref_passes else ''} noise={a.pose_noise or 0}{f' drift={a.drift}' if a.drift else ''} lamp_flip={a.lamp_flip} ev={'fixed' if a.fixed_ev else 'auto'} {'1px' if a.one_pixel else 'avg'}{'+blend' if a.blend else ''}{'+register' if a.register else ''} {a.toggles or ''}"
    for k in ("rug", "seam", "carpet", "wood"):
        if k in res: print(f"{tag:40s} {k:6s} patches {res[k]['patches']:4d}  corr {res[k]['corr']:.2f}  cells differing {res[k]['differ']:.1%}  sharpest view {res[k]['footprint_mm']:.1f} mm")
    if a.save: np.savez_compressed(a.save, **{f"{k}_{n}_{i}": v for k, mp in enumerate(maps) for n, (rgb, best) in mp.items() for i, v in enumerate((rgb.astype(np.uint8), best))})


if __name__ == "__main__":
    main()
