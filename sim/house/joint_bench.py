#!/usr/bin/env python3
"""[JzW] measured: the one joint pose graph (joint_graph.py) against the three-solve chain (house_frame.py -> house_shape.py
-> stitch_floor.py) on a tuning house's two baseline nights, each arm's poses graded against the sim's true path (the
bench's alone: no arm ever reads it). Arms:
  frame   DimOS's pose graph alone (house_frame.py's run, the house's own settings: frame_pgo.args);
  shape   the frame straightened by every lidar scan pair (house_shape.py: what today's pictures are built on);
  chain   the shape, then the floor stitch (stitch_floor.py --near over the views as the shape placed them), composed;
  lidar1  the joint graph's back end over the chain's own sensors (odometry, lidar pairs, loops): one solve, no floor;
  joint   the joint graph: the same plus every floor-view lock.
Every arm starts from the same DimOS pose graph and its loops, at the house's own settings (frame_pgo.args: house 72's
repair). joint_default takes DimOS's loops at its default acceptance (house_frame.PGO_SETTINGS), false ones included
(72): can the robust kernels outvote them? joint_cauchy (loops under a Cauchy kernel) and joint_noloop (no DimOS loops:
the lidar pairs alone tie revisits) are the variants.
--plant is the positive control ([G9q]'s 1% was a weak control, not a null): a known error put in the robot's own
measurements, which every arm then starts from: bend:N:F0:F1:DEG (night N's odometry heading drifts DEG degrees,
evenly by distance walked, from F0 to F1 of its time, and stays off), shift:N:F0:F1:DX:DY (night N's odometry jumps
DX, DY m for that block and back: a slip and its return), loop:F:G:DX:DY:DEG (a false loop closure added to DimOS's pose
graph, from the keyframe at F of the stream to the one at G, its offset wrong by DX, DY m and DEG). Each arm's planted
output is compared with its own clean output: the share of the plant it took back.
Writes only under --scratch. Usage: joint_bench.py <seed> --scratch <dir> [--plant SPEC] [--arms ...] (dimos venv; never
a hold-out house)"""
import os as _os; _os.environ.setdefault("OMP_NUM_THREADS", "1")   # one thread for DimOS's ICP, so a run repeats ([fxk]);
# set before anything loads Open3D or a BLAS (house 71, 8 Oct: the parallel sums' order changed loops run to run)
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))
import argparse, json, math, os, sys, tempfile, time
from pathlib import Path

import numpy as np

import joint_graph as jg

HK = Path(__file__).resolve().parents[2]; SIM = HK / "sim" / "house"
HOLDOUT = {78, 80, 81, *range(100, 120)}                       # never these ([bnO]); 78 is validation
ARMS = ("frame", "shape", "chain", "lidar1", "joint")   # and the variants joint_cauchy, joint_noloop, joint_default (--arms)
MOUNT = HK / "data" / "robot" / "head_mount.json"   # the head camera's mount the views and floor maps are painted with (--mount)
RELIEF = None        # the floor's relief per house (--relief: a path with {seed}, camgeo.py selfcal's), or None: the plane
LAG_FILE = None      # the robot's frame lag the views and floor maps are painted with (--lag-file), or None: the stored one
VERIFY = "either"    # DimOS's loops verified before they enter its graph (--verify: loop_verify.MODES, default either,
                     # [fxk]), or None (--verify none): as DimOS ships
PAINT_FROM = None    # another bench's scratch whose views (and the frame they were painted in) are used, not painted again


def painter_args(seed):
    """the painters' --lag-file and --relief for this house, where given (none: the robot's stored lag, the plane)"""
    out = ["--lag-file", str(LAG_FILE)] if LAG_FILE else []
    if RELIEF:
        p = Path(str(RELIEF).format(seed=seed))
        if not p.exists(): raise SystemExit(f"no floor relief for house {seed}: {p}")
        out += ["--relief", str(p)]
    return out


def run_dir(seed): return SIM / "runs" / f"nights_s{seed}"


def recordings(run):
    rec = dict(l.split()[:2] for l in (run / "seed_recordings.txt").read_text().splitlines() if l.strip())
    return [rec["1"], rec["2"]]


def house_pgo(run):
    import house_frame as hf
    args = (run / "frame_pgo.args").read_text().split() if (run / "frame_pgo.args").exists() else []
    return dict(hf.PGO_SETTINGS, **{k: float(v) for k, v in (x.split("=") for x in args)}), args


# --- plants --------------------------------------------------------------------------------------------------------

def _interp_E(t_arr, E_arr):
    from houndkeeper.localize import Localizer
    L = Localizer(times=list(t_arr), Ts=list(E_arr)); return L.correction


def plant_bend(track, f0, f1, deg):
    """odometry whose heading drifts deg degrees, evenly by distance walked, between f0 and f1 of the night's time, and
    stays off: re-integrated from the odometry's own steps. E(t) = O'(t) O(t)^-1"""
    t, xyh = track.t, track.xyh; t0, t1 = t[0] + f0 * (t[-1] - t[0]), t[0] + f1 * (t[-1] - t[0])
    d = np.r_[0, np.hypot(*np.diff(xyh[:, :2], axis=0).T)]; inb = (t >= t0) & (t <= t1); L = d[inb].sum()
    rate = math.radians(deg) / max(L, 1e-6)
    O = [jg.se2(*p) for p in xyh]; Op = [O[0]]
    for k in range(1, len(O)):
        step = jg.inv(O[k - 1]) @ O[k]; extra = rate * d[k] if inb[k] else 0.0
        Op.append(Op[-1] @ jg.se2(0, 0, extra) @ step)
    E = np.array([a @ jg.inv(b) for a, b in zip(Op, O)]); keep = np.r_[True, np.diff(t) > 1e-6]
    return _interp_E(t[keep], E[keep])


def plant_shift(track, f0, f1, dx, dy):
    t = track.t; t0, t1 = t[0] + f0 * (t[-1] - t[0]), t[0] + f1 * (t[-1] - t[0]); S = jg.se2(dx, dy, 0.0)
    return lambda tt: S if t0 <= tt <= t1 else np.eye(3)


def parse_plant(spec, tracks):
    """(odometry plants {night: E}, bad loop spec or None, description)"""
    w = spec.split(":"); kind = w[0]
    if kind == "bend":
        n = int(w[1]); return {n: plant_bend(tracks[n], float(w[2]), float(w[3]), float(w[4]))}, None, spec
    if kind == "shift":
        n = int(w[1]); return {n: plant_shift(tracks[n], float(w[2]), float(w[3]), float(w[4]), float(w[5]))}, None, spec
    if kind == "loop":
        return {}, (float(w[1]), float(w[2]), (float(w[3]), float(w[4]), math.radians(float(w[5])))), spec
    raise SystemExit(f"--plant {spec!r}: bend, shift or loop")


# --- the chain -----------------------------------------------------------------------------------------------------

def run_shape(dbs, frame_dir, out, tracks, cache, workers):
    """house_shape.py as it runs in the pipeline, its keyframes from the cache and moved with any planted odometry"""
    import house_shape as hs
    def kf(db, n):
        out_ = []
        for k in jg.keyframes_cached(db, n, cache):
            E = tracks[n].E(k[1]) if tracks[n].E is not None else np.eye(3)
            x, y, h = jg.params(E @ jg.se2(k[2], k[3], k[4]))
            out_.append((n, k[1], x, y, h, k[5] @ E[:2, :2].T + E[:2, 2], k[6]))
        return out_
    old, argv = hs.keyframes, sys.argv
    hs.keyframes = kf; sys.argv = ["house_shape.py", *dbs, "--out", str(out), "--init", str(frame_dir), "--workers", str(workers)]
    try: hs.main()
    finally: hs.keyframes, sys.argv = old, argv


def run_stitch(prob, shape_dir, out, workers):
    """stitch_floor.py --near over the views moved to where the shape solve put them; its corrections composed onto the
    shape's (the chain's third solve)"""
    import stitch_floor as sf, confirm_stream as cs
    vdir = Path(out) / "views"; vdir.mkdir(parents=True, exist_ok=True)
    shp = {n: jg.localizer(shape_dir, n) for n in (1, 2)}
    views, ts, night = prob.views
    for n in (1, 2):
        idx = np.nonzero(night == n)[0]; sls, pats, C, F, Ts = [], [], [], [], []
        for k in idx:
            v = views[k]; t = ts[k]
            M = shp[n].correction(t) @ prob.tracks[n].at(t) @ jg.inv(prob.painted[n].correction(t) @ prob.tracks[n].clean(t))
            import floor_refine as fr
            o = fr.centre(v.box); d = M[:2, :2] @ o + M[:2, 2] - o; pl = v.placed(np.array([d[0], d[1], math.atan2(M[1, 0], M[0, 0])]))
            sls.append(pl.box); pats.append(pl.rgb); C.append(M[:2, :2] @ prob.views_meta["C"][k] + M[:2, 2]); F.append(M[:2, :2] @ prob.views_meta["F"][k])
        cs.Views(C, F, sls, pats, ts[idx]).save(vdir / f"views_n{n}.npz")
    argv = sys.argv; sys.argv = ["stitch_floor.py", str(vdir), "--out", str(Path(out) / "stitch"), "--near", "--workers", str(workers)]
    try: sf.main()
    finally: sys.argv = argv
    for f in vdir.glob("*.npz"): f.unlink()                    # 100 MB a night: not kept
    st = {n: jg.localizer(Path(out) / "stitch", n) for n in (1, 2)}
    for n in (1, 2):
        t = np.unique(np.r_[shp[n].times, st[n].times])
        np.savez(Path(out) / f"corrections_{n}.npz", times=t, Ts=np.array([st[n].correction(x) @ shp[n].correction(x) for x in t]))


# --- grading -------------------------------------------------------------------------------------------------------

def traj_of(run, seed, n, scratch):
    """the sim's path file for night n, snapshotted under the scratch dir on first use (a run dir's files can be
    rewritten by a later sweep: house 72's night 1 was, 5 Oct 2026, 13:51-14:02)"""
    import shutil
    snap = Path(scratch) / f"s{seed}" / "truth" / f"nights_s{seed}_n{n}.traj.csv"
    if not snap.exists():
        snap.parent.mkdir(parents=True, exist_ok=True); shutil.copy2(run / f"nights_s{seed}_n{n}.traj.csv", snap)
    return snap


def truth(run, seed, tracks, scratch):
    """the true path at the traj's samples inside each night's recording: {night: (t, true x, y, yaw)}"""
    out = {}
    for n in (1, 2):
        T = np.genfromtxt(traj_of(run, seed, n, scratch), delimiter=",", names=True)
        ok = (T["wall_t"] > tracks[n].t[0]) & (T["wall_t"] < tracks[n].t[-1])
        out[n] = (T["wall_t"][ok], np.c_[T["true_x"][ok], T["true_y"][ok]], np.unwrap(T["true_yaw"][ok]))
    return out


def poses(corr_dir, tracks, tr):
    """an arm's poses at the truth's samples: {night: (xy, yaw)}"""
    out = {}
    for n in (1, 2):
        L = jg.localizer(corr_dir, n); P = [L.correction(t) @ tracks[n].at(t) for t in tr[n][0]]
        out[n] = (np.array([p[:2, 2] for p in P]), np.unwrap([math.atan2(p[1, 0], p[0, 0]) for p in P]))
    return out


def grade(est, tr):
    """pose error against the truth, one rigid fit for both nights (they share the house's frame): position and
    heading, mean and p95; and the nights' agreement (each night's own fit, the two fits' difference along night 2)"""
    import grade_localization as gl
    A = np.vstack([est[n][0] for n in (1, 2)]); B = np.vstack([tr[n][1] for n in (1, 2)]); R, t, th = gl.rigid_fit(A, B)
    e = np.hypot(*(A @ R.T + t - B).T)
    dh = np.concatenate([est[n][1] + th - tr[n][2] for n in (1, 2)]); dh = (dh + np.pi) % (2 * np.pi) - np.pi
    dh = np.degrees(np.abs((dh - math.atan2(np.sin(dh).mean(), np.cos(dh).mean()) + np.pi) % (2 * np.pi) - np.pi))
    fits = {n: gl.rigid_fit(est[n][0], tr[n][1]) for n in (1, 2)}
    p2 = est[2][0]; a = p2 @ fits[1][0].T + fits[1][1]; b = p2 @ fits[2][0].T + fits[2][1]
    return {"pos_mm_mean": round(float(e.mean()) * 1000, 1), "pos_mm_p95": round(float(np.percentile(e, 95)) * 1000, 1),
            "head_deg_mean": round(float(dh.mean()), 3), "head_deg_p95": round(float(np.percentile(dh, 95)), 3),
            "nights_mm_mean": round(float(np.hypot(*(a - b).T).mean()) * 1000, 1), "samples": int(len(e))}


def recovery(planted, clean, tracks_p, tracks_c, tr):
    """how much of a planted error an arm took back: its planted poses against its own clean ones, over the samples the
    plant moved by 1 cm or more, against how far the plant moved them"""
    rem, put, rem_h, put_h = [], [], [], []
    for n in (1, 2):
        for k, t in enumerate(tr[n][0]):
            Op, Oc = tracks_p[n].at(t), tracks_c[n].at(t); m = np.hypot(*(Op[:2, 2] - Oc[:2, 2]))
            put.append(m); rem.append(np.hypot(*(planted[n][0][k] - clean[n][0][k])))
            put_h.append(abs(jg.params(Op @ jg.inv(Oc))[2])); rem_h.append(abs(planted[n][1][k] - clean[n][1][k]))
    put, rem = np.array(put), np.array(rem); sel = put >= 0.01
    out = {"moved_mm_mean_all": round(float(rem.mean()) * 1000, 1), "moved_mm_p95_all": round(float(np.percentile(rem, 95)) * 1000, 1)}
    if sel.any():
        out.update(planted_mm_mean=round(float(put[sel].mean()) * 1000, 1), left_mm_mean=round(float(rem[sel].mean()) * 1000, 1),
                   recovered=round(1 - float(rem[sel].mean() / put[sel].mean()), 3), samples=int(sel.sum()))
    return out


def audit(nodes, X, fams, tracks, tr_dense):
    """the front end judged by the truth (the bench's alone): each constraint's measurement against the true relative
    pose of its two poses; a wrong one is 5 cm or 2 deg off. And whether the back end outvoted it (its whitened
    residual at the solution past the kernel's k)"""
    import gtsam
    def true_at(n, t):
        tt, xy, yaw = tr_dense[n]; return jg.se2(np.interp(t, tt, xy[:, 0]), np.interp(t, tt, xy[:, 1]), np.interp(t, tt, yaw))
    out = {}
    for name, fs in fams.items():
        err, wrong, outvoted, wrong_out = [], 0, 0, 0
        for a, b, Z, I in fs:
            Ta, Tb = true_at(nodes.night[a], nodes.t[a]), true_at(nodes.night[b], nodes.t[b])
            e = np.array(gtsam.Pose2.Logmap(gtsam.Pose2(*jg.params(Z)).between(gtsam.Pose2(*jg.params(jg.inv(Ta) @ Tb)))))
            w = np.hypot(e[0], e[1]) > 0.05 or abs(e[2]) > math.radians(2.0); err.append(np.hypot(e[0], e[1]))
            r = np.array(gtsam.Pose2.Logmap(gtsam.Pose2(*jg.params(Z)).between(gtsam.Pose2(*jg.params(jg.inv(X[a]) @ X[b])))))
            o = math.sqrt(max(float(r @ I @ r), 0)) > jg.HUBER_K[name]
            wrong += w; outvoted += o; wrong_out += (w and o)
        out[name] = {"n": len(fs), "err_mm_median": round(float(np.median(err)) * 1000, 1) if err else None,
                     "wrong": int(wrong), "past_k": int(outvoted), "wrong_and_past_k": int(wrong_out)}
    return out


def audit_floor_by_facing(nodes, fams, tr_dense):
    """[4t7]: the last pass's floor constraints judged by the truth (the bench's alone), by the two views' facing
    difference: each band's count and median error (mm)"""
    import gtsam
    fl = fams.get("floor") or []; meta = getattr(jg.floor_factors, "meta", [])
    if not fl or len(meta) != len(fl): return None
    def true_at(n, t):
        tt, xy, yaw = tr_dense[n]; return jg.se2(np.interp(t, tt, xy[:, 0]), np.interp(t, tt, xy[:, 1]), np.interp(t, tt, yaw))
    rows = []
    for (a, b, Z, I), m in zip(fl, meta):
        Ta, Tb = true_at(nodes.night[a], nodes.t[a]), true_at(nodes.night[b], nodes.t[b])
        e = np.array(gtsam.Pose2.Logmap(gtsam.Pose2(*jg.params(Z)).between(gtsam.Pose2(*jg.params(jg.inv(Ta) @ Tb)))))
        rows.append((m[8], math.hypot(e[0], e[1]) * 1000, 1000 / math.sqrt(max(I[0, 0], 1e-12))))
    r = np.array(rows); out = {}
    for lo, hi in zip(jg.DIR_BANDS[:-1], jg.DIR_BANDS[1:]):
        s_ = (r[:, 0] >= lo) & (r[:, 0] < hi)
        if s_.any(): out[f"{int(lo)}-{int(hi)}"] = {"n": int(s_.sum()), "err_mm_median": round(float(np.median(r[s_, 1])), 1),
                                                    "sd_mm_median": round(float(np.median(r[s_, 2])), 1)}
    return out


def bench(seed, scratch, plant=None, arms=ARMS, workers=4, log=print):
    if seed in HOLDOUT: raise SystemExit(f"house {seed} is a hold-out or validation house: never measured here")
    run = run_dir(seed); dbs = recordings(run); cache = Path(scratch) / "cache"; cache.mkdir(parents=True, exist_ok=True)
    tag = "clean" if not plant else plant.replace(":", "_").replace("-", "m"); S = Path(scratch) / f"s{seed}" / tag; S.mkdir(parents=True, exist_ok=True)
    prob = jg.load_problem(dbs, None, None, None, cache)
    clean_tracks = {n: jg.Track(t.t, t.xyh) for n, t in prob.tracks.items()}
    plants, bad, desc = parse_plant(plant, prob.tracks) if plant else ({}, None, "clean")
    for n, E in plants.items(): prob.tracks[n].E = E
    settings, args = house_pgo(run); timing = {}; res = {"seed": seed, "plant": desc, "house_pgo": args, "verify": VERIFY}
    C0 = Path(scratch) / f"s{seed}" / "clean"; fpaint = C0 / ("frame_default" if args else "frame"); vfiles = [C0 / "views_n1.npz", C0 / "views_n2.npz"]
    if PAINT_FROM:                                            # the views as painted elsewhere (another bench), not again
        P0 = Path(PAINT_FROM) / f"s{seed}" / "clean"; fpaint = P0 / ("frame_default" if args else "frame"); vfiles = [P0 / "views_n1.npz", P0 / "views_n2.npz"]
    verifier = None
    if VERIFY:
        import loop_verify as lv
        vv = None
        if all(f.exists() for f in vfiles):
            jg.attach_views(prob, vfiles, fpaint); vw, vts, vn = prob.views; vv = (vw, np.asarray(vts), np.asarray(vn), prob.painted, prob.views_meta)
        verifier = lv.LoopVerifier(prob.tracks, prob.kf, vv, VERIFY)
    def pgo(d, st):
        if (d / "loops.npz").exists(): return
        bl = None; t0 = time.time()
        if verifier is not None: verifier.record.clear()
        if bad is not None:                                   # the keyframe count is the clean run's
            z = np.load(Path(scratch) / f"s{seed}" / "clean" / ("frame" if d.name == "frame" else "frame_default") / "loops.npz")
            nk = len(z["keyframe_ts"]); bl = (int(bad[0] * nk), int(bad[1] * nk), bad[2])
        with tempfile.TemporaryDirectory() as td:
            corr, loops, kts, _ = jg.pgo_with_loops(dbs, st, td, plants=plants, bad_loop=bl, posed_cache=cache, verifier=verifier)
        jg.save_frame(d, corr, loops, kts, dbs); timing[d.name] = round(time.time() - t0, 1)
        if verifier is not None:
            (d / "verify.json").write_text(json.dumps({"mode": VERIFY, "planted_rejected": getattr(jg.pgo_with_loops, "planted_rejected", None),
                                                       "rejected": len(jg.pgo_with_loops.rejected), "candidates": verifier.record}, indent=1))
        log(f"s{seed} {tag}: {d.name}: {len(kts)} keyframes, {len(loops)} loops" + (f", {len(jg.pgo_with_loops.rejected)} refused by the verifier" if verifier else "") + f" ({timing[d.name]} s)")
    pgo(S / "frame", settings)
    import house_frame as hf
    fdef = S / "frame" if not args else S / "frame_default"
    if args: pgo(fdef, hf.PGO_SETTINGS)
    need_views = not set(arms) <= {"frame", "shape"}            # DimOS's graph and the lidar shape use no camera
    for n in ((1, 2) if need_views else ()):                  # the floor's views, today's builder and the robot's mount,
        v = vfiles[n - 1]                                     # painted where DimOS's pose graph put the clean nights
        if not v.exists():
            t0 = time.time(); env = dict(os.environ, PYTHONPATH=f"{HK / 'src'}:{os.environ.get('PYTHONPATH', '')}")
            import subprocess
            r = subprocess.run([sys.executable, "baseline_views.py", str(run), str(n), dbs[n - 1], "--stack", "4", "--mount", str(MOUNT),
                                "--traj", str(traj_of(run, seed, n, scratch)), "--corrections", str(fpaint / f"corrections_{n}.npz"), "--out", str(v)] + painter_args(seed),
                               cwd=SIM, env=env, capture_output=True, text=True)
            if r.returncode: raise SystemExit(f"baseline_views failed: {r.stderr[-800:]}")
            timing[f"views_n{n}"] = round(time.time() - t0, 1)
    if need_views: jg.attach_views(prob, vfiles, fpaint)
    if "shape" in arms or "chain" in arms:
        if not (S / "shape" / "corrections_2.npz").exists():
            t0 = time.time(); run_shape(dbs, S / "frame", S / "shape", prob.tracks, cache, workers); timing["shape"] = round(time.time() - t0, 1)
    if "chain" in arms and not (S / "chain" / "corrections_2.npz").exists():
        t0 = time.time(); run_stitch(prob, S / "shape", S / "chain", workers); timing["stitch"] = round(time.time() - t0, 1)
    tr = truth(run, seed, clean_tracks, scratch); tr_dense = {n: tr[n] for n in (1, 2)}
    VARIANTS = {"lidar1": (False, {}, True, False), "joint": (True, {}, True, False),        # (floor, kernels, loops, DimOS's
                "joint_cauchy": (True, {"loop": "cauchy"}, True, False), "joint_noloop": (True, {}, False, False),   # default
                "joint_default": (True, {}, True, True),          # loop acceptance): back-end and front-end variants;
                "joint_odo": (True, {}, False, None),             # joint_odo: no DimOS at all, started from odometry;
                "floor_only": (True, {}, False, False),
                "joint_dir": (True, {}, True, False)}             # joint_dir: [4t7]'s directional front end           # floor_only: odometry and the floor alone (no lidar, no
                                                                  # loops): the floor factors' own positive control
    for arm, (floor, kern, use_loops, at_default) in VARIANTS.items():
        if arm not in arms or (S / arm / "corrections_2.npz").exists(): continue
        if at_default and not args: continue                  # the same as "joint" where the house has no settings of its own
        fd = fdef if at_default else S / "frame"
        from houndkeeper.localize import Localizer
        (S / arm).mkdir(exist_ok=True); t0 = time.time(); keep_k = dict(jg.KERNEL)
        jg.KERNEL.update(kern); prob.loops = jg.load_loops(fd) if use_loops else []
        keep_front = jg.FLOOR_FRONT; jg.FLOOR_FRONT = "dir" if arm.endswith("_dir") else keep_front
        start = {n: Localizer() for n in (1, 2)} if at_default is None else {n: jg.localizer(fd, n) for n in (1, 2)}
        try: nodes, X, hist = jg.run(prob, start, workers=workers, floor=floor, lidar=arm != "floor_only",
                                     log=lambda m: log(f"s{seed} {tag} {arm}: {m}"))
        finally: jg.KERNEL.clear(); jg.KERNEL.update(keep_k); jg.FLOOR_FRONT = keep_front
        timing[arm] = round(time.time() - t0, 1)
        for n, (t, T) in jg.corrections(prob, nodes, X).items(): np.savez(S / arm / f"corrections_{n}.npz", times=t, Ts=T)
        (S / arm / "summary.json").write_text(json.dumps({"passes": hist, "seconds": timing[arm],
                                                          "audit": audit(nodes, X, jg.run.families, prob.tracks, tr_dense),
                                                          "floor_by_facing": audit_floor_by_facing(nodes, jg.run.families, tr_dense),
                                                          "directional": getattr(jg.floor_factors, "record", None)}, indent=1))
    old = json.loads((S / "result.json").read_text()) if (S / "result.json").exists() else {}
    res["timing_s"] = dict(old.get("timing_s", {}), **timing); res["arms"] = {}
    for arm in arms:
        d = S / arm
        if not (d / "corrections_2.npz").exists(): continue
        est = poses(d, prob.tracks, tr); r = grade(est, tr)
        if plant:
            cd = Path(scratch) / f"s{seed}" / "clean" / arm
            if (cd / "corrections_2.npz").exists(): r["recovery"] = recovery(est, poses(cd, clean_tracks, tr), prob.tracks, clean_tracks, tr)
        if (d / "summary.json").exists() and arm.startswith(("lidar1", "joint")): r["audit"] = json.loads((d / "summary.json").read_text()).get("audit")
        res["arms"][arm] = r
    odo = {n: (np.array([clean_tracks[n].at(t)[:2, 2] for t in tr[n][0]]), np.array([jg.params(clean_tracks[n].at(t))[2] for t in tr[n][0]])) for n in (1, 2)}
    res["arms"]["odometry"] = grade(odo, tr)
    (S / "result.json").write_text(json.dumps(res, indent=1))
    for arm, r in res["arms"].items():
        rc = r.get("recovery", {})
        log(f"s{seed} {tag} {arm:9s} pos {r['pos_mm_mean']:6.1f} / {r['pos_mm_p95']:6.1f} mm, head {r['head_deg_mean']:.2f} / {r['head_deg_p95']:.2f} deg, "
            f"nights {r['nights_mm_mean']:.1f} mm" + (f"; recovered {rc.get('recovered')} (planted {rc.get('planted_mm_mean')} mm, left {rc.get('left_mm_mean')}), moved {rc['moved_mm_mean_all']} mm" if rc else ""))
    return res


def floor_metrics(seed, scratch, arms=("shape", "joint"), log=print):
    """the floor map from each arm's corrections ([FHA]): both nights' pictures built by today's builder with the robot's
    calibration (floor_refine_bench's), scored by the floor map error (floor_fidelity --error, logged under the scratch
    dir), the placement against the true floor (paired against the first arm) and the nights' disagreement"""
    import floor_refine_bench as frb, floor_agreement as fa, floor_fidelity as ff
    run = run_dir(seed); dbs = recordings(run); cam = frb.camera(run); S = Path(scratch) / f"s{seed}" / "clean"
    traj = [traj_of(run, seed, k, scratch) for k in (1, 2)]
    env = dict(os.environ, PYTHONPATH=f"{HK / 'src'}:{os.environ.get('PYTHONPATH', '')}", OMP_NUM_THREADS="2", OPENBLAS_NUM_THREADS="2")
    out = json.loads((S / "floor.json").read_text()) if (S / "floor.json").exists() else {}; place = {}
    flog = S / "floor_map_error.jsonl"
    for arm in arms:
        d = S / arm; r = {}
        for k in (1, 2):
            pic = d / f"mosaic_n{k}.npz"
            if not pic.exists():
                t0 = time.time()
                frb.sh([sys.executable, "floor_mosaic2.py", dbs[k - 1], pic, "--gain", "--dt", "robot", "--traj", traj[k - 1], "--mount", MOUNT,
                        "--views", 15, "--pct", 25, "--camera", cam, "--corrections", d / f"corrections_{k}.npz"] + painter_args(seed), str(pic) + ".log", env)
                r[f"build_s_n{k}"] = round(time.time() - t0, 1)
            frb.sh([sys.executable, "floor_fidelity.py", pic, "--seed", seed, "--night", k, "--traj", traj[k - 1], "--corrections",
                    d / f"corrections_{k}.npz", "--error", run, "--log", flog, "--built", f"JzW {arm}"], d / f"fidelity_n{k}.log", env)
            r[f"error_n{k}"] = [json.loads(l) for l in flog.read_text().splitlines()][-1]["error"]
            place[arm, k] = ff.placement_of(pic, seed, k, traj[k - 1], d / f"corrections_{k}.npz", run)
            r[f"placement_n{k}"] = ff.placement_summary(place[arm, k], place.get((arms[0], k)) if arm != arms[0] else None)
        ag, _ = fa.agreement(run, seed, d / "mosaic_n1.npz", d / "mosaic_n2.npz")
        r["disagree_open"], r["disagree_all"] = ag["open"][0], ag["all"][0]
        out[arm] = r
        log(f"s{seed} floor {arm}: error {r['error_n1']:.2%} / {r['error_n2']:.2%}, disagree open {r['disagree_open']:.2%}, all {r['disagree_all']:.2%}, "
            f"placement {r['placement_n1'].get('paired_mm')} / {r['placement_n2'].get('paired_mm')} mm paired")
    from houndkeeper import calibrate as _cal
    out["builder"] = {p.name: _cal.file_sha(p) for p in frb.BUILDER if p.exists()}
    (S / "floor.json").write_text(json.dumps(out, indent=1)); return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("seeds", type=int, nargs="+"); ap.add_argument("--scratch", required=True)
    ap.add_argument("--plant", action="append", default=[None]); ap.add_argument("--arms", nargs="+", default=list(ARMS))
    ap.add_argument("--workers", type=int, default=4); ap.add_argument("--no-clean", action="store_true")
    ap.add_argument("--floor", nargs="*", default=None, help="also build and score the floor map from these arms (default shape joint)")
    ap.add_argument("--mount", default=None, help="the head camera's mount to paint the views and floor maps with (default the robot's, data/robot/head_mount.json)")
    ap.add_argument("--relief", default=None, help="the floor's relief per house, a path with {seed} (camgeo.py selfcal); default the plane z = 0")
    ap.add_argument("--lag-file", default=None, help="the frame lag to paint the views and floor maps with (default the robot's stored lag)")
    ap.add_argument("--verify", default="either", choices=["lidar", "floor", "either", "log", "none"], help="DimOS's loops verified before they enter its graph (loop_verify.py; [fxk]: on by default, the lidar then the floor; none: DimOS as it ships)")
    ap.add_argument("--paint-from", default=None, help="another bench's scratch, whose views (and the frame they were painted in) are used, not painted again")
    a = ap.parse_args(); sys.path.insert(0, str(SIM))
    global MOUNT, RELIEF, LAG_FILE, VERIFY, PAINT_FROM
    VERIFY, PAINT_FROM = (None if a.verify == "none" else a.verify), a.paint_from
    if a.mount: MOUNT = Path(a.mount)
    RELIEF, LAG_FILE = a.relief, a.lag_file
    plants = [p for p in a.plant if p is not None] if a.no_clean else a.plant
    for s in a.seeds:
        for p in plants:
            try: bench(s, a.scratch, p, a.arms, a.workers, log=lambda m: print(m, flush=True))
            except SystemExit as e: print(f"s{s}: {e}", flush=True)
        if a.floor is not None:
            try: floor_metrics(s, a.scratch, tuple(a.floor) or ("shape", "joint"), log=lambda m: print(m, flush=True))
            except SystemExit as e: print(f"s{s} floor: {e}", flush=True)


if __name__ == "__main__":
    main()
