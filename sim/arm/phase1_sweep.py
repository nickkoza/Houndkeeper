#!/usr/bin/env python3
"""Phase 1 in the arm sim: sweep a 3 x 3 m carpet patch in lawnmower rows (arm stowed), lay head-camera frames onto
the floor at 5 mm, and compare tonight's floor picture with a clean baseline sweep using the frozen detector's own
features (detect_changes.feats / tolerant_diff, same thresholds). Pose comes from the sim (stands in for SLAM)."""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import sys
from pathlib import Path
import numpy as np, mujoco
from scipy import ndimage as ndi
sys.path.insert(0, str(_HK / "sim" / "arm")); sys.path.insert(0, str(_HK / "sim" / "house"))
import detect_changes as dc
HERE = Path(__file__).resolve().parent; RES, HALF = 0.005, 1.4; N = int(2 * HALF / RES)
ROWS = [-1.1, -0.5, 0.1, 0.7]; X0, X1 = -1.2, 1.1
gx = -HALF + (np.arange(N) + 0.5) * RES; gy = HALF - (np.arange(N) + 0.5) * RES; GX, GY = np.meshgrid(gx, gy)
P = np.stack([GX.ravel(), GY.ravel(), np.zeros(N * N)], 1)

def sweep(S, every=0.2):     # camera frame every 0.2 s of sim time (was 0.1; still heavy overlap at 0.4 m/s)
    """drive the rows with the walking policy; returns (rgb, dist) floor picture"""
    m, d = S.m, S.d; H, W = 360, 640; r = mujoco.Renderer(m, H, W); rd = mujoco.Renderer(m, H, W); rd.enable_depth_rendering(); cid = m.camera("head_camera").id
    tall = np.zeros(N * N, np.int32)                               # hits on something standing above the floor (depth, as lidar would)
    f = (H / 2) / np.tan(np.radians(m.cam_fovy[cid]) / 2); rgb = np.zeros((N * N, 3), np.uint8); best = np.full(N * N, np.inf)
    nxt = [S.t]
    def grab(m_, d_):
        if S.t < nxt[0]: return
        nxt[0] = S.t + every; r.update_scene(d_, "head_camera"); img = r.render(); rd.update_scene(d_, "head_camera"); dep = rd.render()
        c = d_.cam_xpos[cid]; R = d_.cam_xmat[cid].reshape(3, 3); pc = (P - c) @ R; z = -pc[:, 2]
        ok = z > 0.05; u = W / 2 + f * pc[:, 0] / np.where(ok, z, 1); v = H / 2 - f * pc[:, 1] / np.where(ok, z, 1)
        ok &= (u >= 0) & (u < W - 1) & (v >= 0) & (v < H - 1); dist = np.linalg.norm(P - c, axis=1); ok &= dist < 2.0
        ui, vi = u.astype(int), v.astype(int); zi = np.where(ok, dep[np.clip(vi, 0, H - 1), np.clip(ui, 0, W - 1)], np.inf)
        raised = ok & (zi < z - 0.02)                                  # this pixel hits something before reaching the floor
        ok &= ~raised                                                  # don't paint the floor with it (height-aware, like the lidar mask)
        vv, uu = np.nonzero(dep < 2.0); zz = dep[vv, uu]
        pts = c + (np.stack([(uu - W / 2) / f * zz, -(vv - H / 2) / f * zz, -zz], 1) @ R.T)
        up = (pts[:, 2] > 0.015) & (pts[:, 2] < 0.5) & (np.abs(pts[:, 0]) < HALF) & (np.abs(pts[:, 1]) < HALF)
        ci = ((HALF - pts[up, 1]) / RES).astype(int) * N + ((pts[up, 0] + HALF) / RES).astype(int); np.add.at(tall, ci, 1)
        better = ok & (dist < best); idx = np.nonzero(better)[0]
        rgb[idx] = img[v[idx].astype(int), u[idx].astype(int)]; best[idx] = dist[idx]
    S.on_step = grab
    for k, y in enumerate(ROWS):
        xa, xb = (X0, X1) if k % 2 == 0 else (X1, X0)
        S.walk_to(np.array([xa, y]), 0.0 if xb > xa else np.pi, tol=0.08, max_s=20, settle=False)
        S.walk_to(np.array([xb, y]), 0.0 if xb > xa else np.pi, tol=0.08, max_s=25, settle=False)
    S.on_step = None
    return rgb.reshape(N, N, 3), best.reshape(N, N), tall.reshape(N, N)

def detect(base, tonight):
    (rb, db, tb), (rn, dn, tn) = base, tonight
    Vb, Vn = np.isfinite(db), np.isfinite(dn)
    cn, bn = dc.feats(rn, Vn); cb, bb = dc.feats(rb, Vb); dC, br = dc.tolerant_diff(cn, bn, cb, bb)
    region = ndi.binary_erosion(Vn & Vb, iterations=6)
    cand = region & ((dC > dc.TC) | (br > dc.TB))
    newtall = (tn >= 3) & ~ndi.binary_dilation(tb >= 3, iterations=8)          # stands up tonight, not in the baseline (within 4 cm)
    cand = ndi.binary_opening(cand | ndi.binary_dilation(newtall, iterations=2), iterations=1)
    lab, nb = ndi.label(cand)
    if nb == 0: return None, cand, region
    sizes = ndi.sum(cand, lab, range(1, nb + 1)); i = int(np.argmax(sizes)) + 1; rr, cc = np.nonzero(lab == i)
    return {"x": float(gx[cc].mean()), "y": float(gy[rr].mean()), "area_cm2": float(sizes[i - 1] * RES * RES * 1e4), "n_blobs": int(nb),
            "other_blobs_over_4cm2": int((sizes * RES * RES * 1e4 >= 4).sum() - 1)}, cand, region

def detect_all(base, tonight, min_cm2=20.0):
    """every new thing, not just the biggest: list of {x, y, area_cm2}"""
    _, cand, region = detect(base, tonight)
    lab, nb = ndi.label(cand); out = []
    for i in range(1, nb + 1):
        rr, cc = np.nonzero(lab == i); a = len(rr) * RES * RES * 1e4
        if a >= min_cm2: out.append({"x": float(gx[cc].mean()), "y": float(gy[rr].mean()), "area_cm2": round(a, 1)})
    return out, cand
