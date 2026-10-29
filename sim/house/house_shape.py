#!/usr/bin/env python3
"""The house's shape held by its lidar ([KkQ], [FHA]): the lidar, combined with
odometry, keeps the map anchored to real positions so it does not bend. Between loop closures the pose graph
(house_frame.py) trusts leg odometry, whose slow heading drift bends the map (8-18 cm off a straight one, 1 Oct). A lidar
scan is a rigid ruler several metres across: here every pair of keyframes that see the same walls is matched, not only
revisits, and one weighted solve places every keyframe:
  - each match (point-to-line ICP between the two scans as currently placed) is weighed by what it pins: its
    information J'J / sd^2, so a wall holds across itself and not along it, a corner both ways;
  - matches that disagree with the rest are down-weighted (Huber), and a match that would move a keyframe further than
    odometry could have drifted between them is dropped;
  - consecutive keyframes of a night are tied as odometry allows (it drifts ~3 cm and ~1 deg a metre walked);
  - matched again after each pass, as the stitch is (stitch_floor.py), four passes.
It starts from the pose graph's corrections (--init), or odometry's frame. Writes each night's corrections (times, 3x3
SE(2) world <- believed) for floor_mosaic2.py --corrections. Only the robot's own lidar and odometry go in.
Usage: house_shape.py <night1.db> <night2.db> --out <dir> [--init <dir with corrections_<n>.npz>]   (dimos venv)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))
import argparse, math, time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
from dimos.memory.store.sqlite import SqliteStore
from houndkeeper.localize import Localizer, scans_from, structure, level, normals, _voxel, LIDAR_SD
import robust_ls as rl

KEY_M, KEY_DEG, STACK = 0.25, 5.0, 3        # a keyframe each 25 cm or 5 deg, its scan with the next two stacked (denser)
NEAR_M, PER_KEY = 3.0, 12                   # pairs: keyframes within 3 m, the nearest 12 of each (in space and time spread)
SEQ_SD = (0.005, 0.03, math.radians(0.1), math.radians(1.0))   # odometry between consecutive keyframes: m + m/m, rad + rad/m
DRIFT = (0.10, 0.05, math.radians(2.0), math.radians(1.5))     # how far apart two keyframes' corrections may be: m + m per
HUBER = 0.02                                # m walked between them, rad + rad per m (odometry's drift, with room)
MIN_PTS, MAX_RMS = 150, 0.04                # a match: enough wall seen in both, and a tight fit
_K = None


def poses(st):
    """the robot's believed position over time (world <- base_link), from the recording's transforms"""
    ts, xy = [], []
    for o in st.stream("tf"):
        for t in getattr(o.data, "transforms", [o.data]):
            if (t.frame_id, t.child_frame_id) == ("world", "base_link"):
                M = t.to_matrix(); ts.append(getattr(t, "ts", None) or o.ts); xy.append((M[0, 3], M[1, 3], math.atan2(M[1, 0], M[0, 0])))
    o_ = np.argsort(ts); return np.array(ts)[o_], np.array(xy)[o_]


def keyframes(db, night):
    """[(night, time, believed x, y, heading, structure points in the believed frame, travelled m)]"""
    st = SqliteStore(path=db); st.start(); S = scans_from(st); pt, pp = poses(st); st.stop()
    out, last, trav, prev = [], None, 0.0, None
    for k, (t, p) in enumerate(S):
        x, y = np.interp(t, pt, pp[:, 0]), np.interp(t, pt, pp[:, 1]); h = np.interp(t, pt, np.unwrap(pp[:, 2]))
        if prev is not None: trav += math.hypot(x - prev[0], y - prev[1])
        prev = (x, y)
        if last is not None and math.hypot(x - last[0], y - last[1]) < KEY_M and abs(h - last[2]) < math.radians(KEY_DEG): continue
        pts = np.vstack([structure(level(q)) for _t, q in S[k:k + STACK]])
        if len(pts) < MIN_PTS: continue
        out.append((night, t, x, y, h, _voxel(pts, 0.03), trav)); last = (x, y, h)
    return out


def icp_info(src, dst, T0, iters=20, max_d=0.3):
    """point-to-line ICP laying src onto dst from T0: (T, information 3x3 for (tx, ty, turn about the origin), rms, n)"""
    tree = cKDTree(dst); nrm = normals(dst, tree); T = T0.copy(); H = None; rms, n = 1.0, 0
    for _ in range(iters):
        p = src @ T[:2, :2].T + T[:2, 2]; d, j = tree.query(p, distance_upper_bound=max_d); ok = np.isfinite(d)
        if ok.sum() < MIN_PTS: return None
        keep = ok & (d <= np.quantile(d[ok], 0.8)); a, q, nn = p[keep], dst[j[keep]], nrm[j[keep]]
        res = np.einsum("ij,ij->i", nn, a - q); J = np.c_[nn, nn[:, 0] * -a[:, 1] + nn[:, 1] * a[:, 0]]
        H = J.T @ J; dx = -np.linalg.solve(H + 1e-9 * np.eye(3), J.T @ res)
        c, s = math.cos(dx[2]), math.sin(dx[2]); T = np.array([[c, -s, dx[0]], [s, c, dx[1]], [0, 0, 1.0]]) @ T
        rms, n = float(np.sqrt(np.mean(res ** 2))), int(keep.sum())
        if abs(dx[2]) < 1e-6 and math.hypot(dx[0], dx[1]) < 1e-5: break
    return T, H / max(rms, LIDAR_SD) ** 2, rms, n


def _init(K):
    global _K; _K = K


def pair_match(pair):
    """keyframe j's scan laid on keyframe i's, both as currently corrected: the leftover correction j needs, at the
    pair's midpoint (shift x, y and turn), its information there, rms, n; or None"""
    i, j = pair; Pi, Pj = _K[i], _K[j]
    r = icp_info(Pj, Pi, np.eye(3))
    if r is None: return None
    T, I, rms, n = r
    if rms > MAX_RMS: return None
    return i, j, T, I, rms, n


def shift_at(T, at):
    """how far the SE(2) T (about the world's origin) moves the point `at`: a match's shift felt at its keyframe"""
    return float(np.hypot(*(T[:2, :2] @ at + T[:2, 2] - at)))


def drift_ok(T, at, dd):
    """a match odometry could have drifted into over dd m walked: its shift at the keyframe (`at`) and its turn within
    DRIFT. About the origin instead, a 1 deg turn at a corner (7, 4) m read as 14 cm and was dropped (critique, 3 Oct)"""
    return shift_at(T, at) <= DRIFT[0] + DRIFT[1] * dd and abs(math.atan2(T[1, 0], T[0, 0])) <= DRIFT[2] + DRIFT[3] * dd


def se2(x, y, th):
    c, s = math.cos(th), math.sin(th); return np.array([[c, -s, x], [s, c, y], [0, 0, 1.0]])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("dbs", nargs="+"); ap.add_argument("--out", required=True)
    ap.add_argument("--init", default=None); ap.add_argument("--passes", type=int, default=4); ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--texture", default=None, help="the floor's texture springs (stitch_floor.py's springs.npz, its views painted with --init): one solve with the lidar's")
    ap.add_argument("--texture-gain", type=float, default=1.0, help="the texture springs' stiffness, times their own")
    ap.add_argument("--solver", default="irls", choices=["irls", "ls", "gtsam"], help="irls (default): Huber by reweighting lsqr, three a pass; ls: one scipy least_squares solve a pass with a true robust loss; gtsam: a GTSAM factor graph a pass, Levenberg-Marquardt, robust noise models, and how firmly each keyframe is pinned (marginals.npz) ([2Ke])")
    ap.add_argument("--loss", default="huber", choices=["huber", "soft_l1", "cauchy"], help="--solver ls (huber, soft_l1) or gtsam (huber, cauchy): the matches' robust loss, its scale measured from the matches")
    a = ap.parse_args(); t0 = time.time(); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    KF = [k for n, db in enumerate(a.dbs, 1) for k in keyframes(db, n)]; N = len(KF)
    night = np.array([k[0] for k in KF]); ts = np.array([k[1] for k in KF]); trav = np.array([k[6] for k in KF])
    C = []                                                   # each keyframe's correction (world <- believed), from the init
    for n in sorted(set(night)):
        p = Path(a.init) / f"corrections_{n}.npz" if a.init else None
        loc = Localizer(times=list(np.load(p)["times"]), Ts=list(np.load(p)["Ts"])) if p and p.exists() else None
        C += [loc.correction(t) if loc else np.eye(3) for t in ts[night == n]]
    C = np.array(C); B = np.array([[k[2], k[3]] for k in KF])            # believed positions
    C0 = C.copy(); TX = None; hist = [C.copy()]; solve_s = 0.0
    if a.texture:                                            # each image's spring pulls on the keyframe it was taken nearest to
        z = np.load(a.texture); vk = []
        for t_, n_ in zip(z["ts"], z["night"]):
            idx = np.nonzero(night == n_)[0]; vk.append(int(idx[np.argmin(np.abs(ts[idx] - t_))]) if len(idx) else -1)
        vk = np.array(vk); TX = [(vk[i], vk[j], o, d, I * a.texture_gain, it * a.texture_gain)
                                 for i, j, o, d, I, it in zip(z["i"], z["j"], z["o"], z["d"], z["Ixy"], z["Ith"]) if vk[i] >= 0 and vk[j] >= 0 and vk[i] != vk[j]]
        print(f"texture springs: {len(TX)} of {len(z['i'])} between different keyframes", flush=True)
    print(f"{N} keyframes from {len(a.dbs)} nights ({time.time() - t0:.0f} s)", flush=True)
    for it in range(a.passes):
        P = np.array([c[:2, :2] @ b + c[:2, 2] for c, b in zip(C, B)])   # where each keyframe stands now
        pts = [k[5] @ c[:2, :2].T + c[:2, 2] for k, c in zip(KF, C)]
        pairs = set()
        for i in range(N):
            d = np.hypot(*(P - P[i]).T); ok = d < NEAR_M; ok[i] = False
            for j in np.nonzero(ok)[0][np.argsort(d[ok])][:PER_KEY]: pairs.add((min(i, j), max(i, j)))
        with ProcessPoolExecutor(a.workers, initializer=_init, initargs=(pts,)) as ex:
            meas = [m for m in ex.map(pair_match, sorted(pairs), chunksize=32) if m is not None]
        kept = []                                            # a match beyond what odometry could have drifted: a false one
        for i, j, T, I, rms, n in meas:
            dd = abs(trav[j] - trav[i]) if night[i] == night[j] else 50.0
            if drift_ok(T, P[j], dd): kept.append((i, j, T, I))   # felt at P[j], as the solve below takes it
        w = np.ones(len(kept)); perp = lambda v: np.array([-v[1], v[0]])
        S = rl.Rows()                                        # the pass's system, built once; a match's rows are its group
        for g_, (i, j, T, I) in enumerate(kept):            # j's increment minus i's should be the leftover T, felt at P[j]
            o = P[j]; lever = perp(o)                        # T: about the origin -> shift at o, turn
            sx, sy = T[:2, :2] @ o + T[:2, 2] - o; th = math.atan2(T[1, 0], T[0, 0])
            Jo = np.array([[1, 0, -o[1]], [0, 1, o[0]], [0, 0, 1.0]]); Io = np.linalg.inv(Jo).T @ I @ np.linalg.inv(Jo)   # info at o
            wv, V = np.linalg.eigh(Io); L = (V * np.sqrt(np.clip(wv, 0, None))) @ V.T
            pi_, pj = perp(o - P[i]), perp(o - P[j]); b = np.array([sx, sy, th])
            terms = [[(3 * j, 1.0), (3 * j + 2, pj[0]), (3 * i, -1.0), (3 * i + 2, -pi_[0])],
                     [(3 * j + 1, 1.0), (3 * j + 2, pj[1]), (3 * i + 1, -1.0), (3 * i + 2, -pi_[1])],
                     [(3 * j + 2, 1.0), (3 * i + 2, -1.0)]]
            for rr in range(3):
                S.eq([(c_, v_ * L[rr, k]) for k in range(3) for c_, v_ in terms[k]], float(L[rr] @ b), 1.0, g_)
        for ki, kj, o0, dm, Ixy, Ith in (TX or []):         # texture: view j must have moved by dm (since --init) to lie on i
            Di, Dj = C[ki] @ np.linalg.inv(C0[ki]), C[kj] @ np.linalg.inv(C0[kj])
            oi, oj = Di[:2, :2] @ o0 + Di[:2, 2], Dj[:2, :2] @ o0 + Dj[:2, 2]   # where that floor point is now, as each
            done = oj - oi                                   # image carries it: the spring's stretch left is dm - done
            tdone = math.atan2(Dj[1, 0], Dj[0, 0]) - math.atan2(Di[1, 0], Di[0, 0])
            b = np.array([dm[0] - done[0], dm[1] - done[1]]); o = oj
            pi_, pj = perp(o - P[ki]), perp(o - P[kj]); wv, V = np.linalg.eigh(Ixy); L = (V * np.sqrt(np.clip(wv, 0, None))) @ V.T
            tt = [[(3 * kj, 1.0), (3 * kj + 2, pj[0]), (3 * ki, -1.0), (3 * ki + 2, -pi_[0])],
                  [(3 * kj + 1, 1.0), (3 * kj + 2, pj[1]), (3 * ki + 1, -1.0), (3 * ki + 2, -pi_[1])]]
            for rr in range(2):
                if L[rr, 0] or L[rr, 1]: S.eq([(c_, v_ * L[rr, k]) for k in range(2) for c_, v_ in tt[k]], float(L[rr] @ b), 1.0)
            if Ith > 0: S.eq([(3 * kj + 2, 1.0), (3 * ki + 2, -1.0)], float(dm[2] - tdone), math.sqrt(Ith))
        for i in range(N - 1):                               # odometry between a night's consecutive keyframes
            if night[i] != night[i + 1]: continue
            dd = abs(trav[i + 1] - trav[i]); wt, wr = 1 / (SEQ_SD[0] + SEQ_SD[1] * dd), 1 / (SEQ_SD[2] + SEQ_SD[3] * dd)
            pi_, pj = perp(P[i + 1] - P[i]), np.zeros(2)
            for k in (0, 1): S.eq([(3 * (i + 1) + k, 1.0), (3 * i + k, -1.0), (3 * i + 2, -pi_[k])], 0.0, wt)
            S.eq([(3 * (i + 1) + 2, 1.0), (3 * i + 2, -1.0)], 0.0, wr)
        for k in range(3): S.eq([(k, 1.0)], 0.0, 1e4)       # the first keyframe holds the frame
        for i in range(N):
            for k in range(3): S.eq([(3 * i + k, 1.0)], 0.0, 1.0 / (0.5 if k < 2 else 0.2))
        A, bvec, groups = S.system(3 * N)

        def disagree(x):                                     # each match's disagreement left, in m
            res = []
            for (i, j, T, I) in kept:
                o = P[j]; sx, sy = T[:2, :2] @ o + T[:2, 2] - o
                d_ = (x[3 * j:3 * j + 2] + x[3 * j + 2] * perp(o - P[j])) - (x[3 * i:3 * i + 2] + x[3 * i + 2] * perp(o - P[i]))
                res.append(math.hypot(d_[0] - sx, d_[1] - sy))
            return np.array(res)
        t_s = time.time()
        if a.solver == "irls":                               # the old path: Huber by reweighting, a match's rows scaled by
            for rw in range(3):                              # its weight (its squared residual by the weight squared:
                x = rl.lsqr_weighted(A, bvec, groups, w)     # critique #12)
                res = disagree(x); w = np.minimum(1.0, HUBER / np.maximum(res, 1e-9))
        else:                                                # one robust solve, the loss's scale measured ([2Ke])
            last = it == a.passes - 1
            x, info = (rl.solve_robust(A, bvec, groups, loss=a.loss) if a.solver == "ls" else
                       rl.solve_gtsam(A, bvec, groups, loss=a.loss, marginals=last)); res = disagree(x)
            print(f"  solve ({a.solver} {a.loss}): scale {info['f_scale']:.2f} whitened ({info['sd']:.2f} sd measured), {info['nfev']} evaluations", flush=True)
            if "cov" in info:                                # how firmly the last pass holds each keyframe
                sxy, sth = rl.pinned(info["cov"]); worst = np.argsort(-sxy)[:5]
                np.savez(out / "marginals.npz", times=ts, night=night, P=P, sd_xy=sxy, sd_th=sth)
                print(f"  pinned: keyframes' position sd {np.median(sxy) * 1000:.1f} mm median, {np.percentile(sxy, 95) * 1000:.1f} p95; loosest "
                      + ", ".join(f"n{night[i]} t{ts[i]:.0f} {sxy[i] * 1000:.1f} mm" for i in worst), flush=True)
        solve_s += time.time() - t_s
        for i in range(N):                                   # each increment: a turn about where the keyframe stands, and a shift
            th = x[3 * i + 2]; R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            D = np.eye(3); D[:2, :2] = R; D[:2, 2] = P[i] - R @ P[i] + x[3 * i:3 * i + 2]; C[i] = D @ C[i]
        hist.append(C.copy())                                # every pass's positions, kept: the settling can be shown later
        springs = [(i, j, float(np.sqrt(max(np.linalg.eigvalsh(I[:2, :2]).min(), 0)))) for i, j, T, I in kept]   # and its springs:
                                                                                                             # (i, j, stiffness)
        sh = np.hypot(x[0::3], x[1::3]) * 1000
        print(f"pass {it + 1}: {len(meas)} of {len(pairs)} pairs matched, {len(kept)} kept; disagree {np.median(res) * 1000:.0f} mm "
              f"median, {np.percentile(res, 95) * 1000:.0f} p95; keyframes moved {np.median(sh):.0f} mm median, {sh.max():.0f} max", flush=True)
    for n in sorted(set(night)):
        idx = np.nonzero(night == n)[0]; np.savez(out / f"corrections_{n}.npz", times=ts[idx], Ts=C[idx])
    np.savez_compressed(out / "settle.npz", passes=np.array(hist), times=ts, night=night, believed=B,   # for an animation of
                        pts=np.vstack([k[5] for k in KF]), pts_start=np.cumsum([0] + [len(k[5]) for k in KF]),   # the springs settling
                        springs=np.array(springs, float))
    import json as _json                                     # its health, for the pipeline report ([63R])
    (out / "summary.json").write_text(_json.dumps({"keyframes": N, "pairs": len(pairs), "matched": len(meas), "kept": len(kept),
                                                   "disagree_mm_median": round(float(np.median(res)) * 1000, 1),
                                                   "moved_last_pass_mm_median": round(float(np.median(sh)), 1)}))
    print(f"solve time {solve_s:.2f} s ({a.solver})")
    print(f"house shape: {N} keyframes, {time.time() - t0:.0f} s -> {out}/corrections_<n>.npz")


if __name__ == "__main__":
    main()
