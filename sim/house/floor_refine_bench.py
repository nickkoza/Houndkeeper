#!/usr/bin/env python3
"""[G9q] measured: per tuning house, the stitch as it stands (its pictures placed by the lidar's shape solve) against
the same plus floor_refine.py's passes, by the floor map error ([nv5], floor_fidelity.py --error, logged to the scratch
directory, never the project's log), the map's placement against the true floor (floor_fidelity.placement: [nv5]'s
error forgives 2 cm, so it cannot see the few mm the refine moves), and the nights' disagreement (floor_agreement.py:
the open floor with the sim's truth, and the whole judged floor without it). An analysis tool over finished runs: it reads the run dir and its
recordings and writes only under --scratch. Every picture of both arms is rebuilt with today's builder and the robot's
calibration (data/robot: mount, and the stored lag for each recording's own clock, from its path file, [55G]), so the
two arms differ only in their corrections.
--plant is the bench's positive control: a known error planted in the shape solve's corrections (a block of one night's
time moved by a shift and a turn about the path there), which both arms then start from. The springs arm must read
worse for it (by placement: the floor map error barely sees it, decision [qLu]), and the refine is asked to take it out.
Usage: floor_refine_bench.py <seed> [<seed> ...] --scratch <dir> [--jobs 2] [--plant 1:0.40-0.55:0.05,0,1.0]
       (dimos venv; never a hold-out house)"""
import argparse, json, os, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HK = Path(__file__).resolve().parents[2]; SIM = HK / "sim" / "house"; PY = sys.executable
MOUNT = HK / "data" / "robot" / "head_mount.json"
HOLDOUT = {80, 81, *range(100, 120)}                       # never these ([bnO]); 78 only labelled as validation


def run_dir(seed): return SIM / "runs" / f"nights_s{seed}"


def recordings(run):
    rec = dict(l.split()[:2] for l in (run / "seed_recordings.txt").read_text().splitlines() if l.strip())
    return rec["1"], rec["2"]


def camera(run):
    """the camera the stitch built this house's pictures from: its stamp's second word (v5/v7 'both' or 'head')"""
    w = (run / "stitch.args").read_text().split()
    return w[1] if len(w) > 1 and w[1] in ("head", "wrist", "both") else "head"


BUILDER = [SIM / "floor_mosaic2.py", SIM / "floor_sample.py", HK / "src" / "houndkeeper" / "rec_poses.py"]


def fresh(out):
    """a picture already built; which builder built each house's pictures is kept in its result (BUILDER's hashes)"""
    return out.exists()


def sh(cmd, log, env=None):
    """run a step, its output to log; a failed step stops the house ([SdA])"""
    with open(log, "w") as f:
        r = subprocess.run([str(c) for c in cmd], stdout=f, stderr=subprocess.STDOUT, cwd=SIM, env=env)
    if r.returncode: raise SystemExit(f"{cmd[1]} failed ({r.returncode}): see {log}")


def parse_plant(spec):
    """'night:f0-f1:dx,dy,dth_deg' -> (night, f0, f1, dx m, dy m, dth deg): the block from f0 to f1 of the night's time"""
    n, fr_, mv = spec.split(":"); f0, f1 = (float(v) for v in fr_.split("-")); dx, dy, dth = (float(v) for v in mv.split(","))
    if not (int(n) in (1, 2) and 0 <= f0 < f1 <= 1): raise SystemExit(f"--plant {spec!r}: night 1 or 2, 0 <= f0 < f1 <= 1")
    return int(n), f0, f1, dx, dy, dth


def plant(times, Ts, path_t, path_xy, f0, f1, dx, dy, dth_deg):
    """corrections (times, Ts 3x3) with a known error planted: between f0 and f1 of their span each correction is moved
    by (dx, dy) and turned dth_deg about the corrected path's mean there (path_t, path_xy: the robot's own path in
    odometry), and kept elsewhere; knots at the block's edges make it a step. (times, Ts)"""
    import math
    import numpy as np
    from houndkeeper.localize import Localizer
    L = Localizer(times=list(times), Ts=list(Ts)); t0 = times[0] + f0 * (times[-1] - times[0]); t1 = times[0] + f1 * (times[-1] - times[0])
    k = (path_t >= t0) & (path_t <= t1)
    if not k.any(): raise SystemExit("--plant: the block holds none of the path")
    o = np.mean([L.correction(t)[:2, :2] @ p + L.correction(t)[:2, 2] for t, p in zip(path_t[k], path_xy[k])], 0)
    th = math.radians(dth_deg); c, s_ = math.cos(th), math.sin(th); R = np.array([[c, -s_], [s_, c]])
    M = np.eye(3); M[:2, :2] = R; M[:2, 2] = o - R @ o + [dx, dy]
    T = np.unique(np.r_[times, t0 - 1e-3, t0, t1, t1 + 1e-3])
    return T, np.array([M @ L.correction(t) if t0 <= t <= t1 else L.correction(t) for t in T])


def plant_dir(run, S, plants, traj):
    """the shape solve's corrections with the plants in them, written under the scratch dir: the arms' starting point"""
    import numpy as np
    d = S / "plant"; d.mkdir(exist_ok=True)
    for k in (1, 2):
        z = np.load(run / "shape" / f"corrections_{k}.npz"); times, Ts = z["times"], z["Ts"]
        T_ = np.genfromtxt(traj[k - 1], delimiter=",", names=True); pt, pxy = T_["wall_t"], np.c_[T_["odom_x"], T_["odom_y"]]
        for n, f0, f1, dx, dy, dth in plants:
            if n == k: times, Ts = plant(times, Ts, pt, pxy, f0, f1, dx, dy, dth)
        np.savez(d / f"corrections_{k}.npz", times=times, Ts=Ts)
    return d


def bench(seed, scratch, jobs, plants=()):
    if seed in HOLDOUT: raise SystemExit(f"house {seed} is a hold-out: never measured here")
    run = run_dir(seed); S = Path(scratch) / f"s{seed}"; (S / "base").mkdir(parents=True, exist_ok=True); (S / "refine").mkdir(exist_ok=True)
    dbs = recordings(run); cam = camera(run); traj = [run / f"nights_s{seed}_n{k}.traj.csv" for k in (1, 2)]
    shape = plant_dir(run, S, plants, traj) if plants else run / "shape"   # the springs arm's corrections
    env = dict(os.environ, PYTHONPATH=f"{HK / 'src'}:{os.environ.get('PYTHONPATH', '')}", OMP_NUM_THREADS="2",
               OPENBLAS_NUM_THREADS="2", MKL_NUM_THREADS="2")
    t0 = time.time(); say = lambda m: print(f"s{seed} [{time.time() - t0:.0f} s] {m}", flush=True)
    def mosaic(k, corr, out):
        if fresh(out): return
        sh([PY, "floor_mosaic2.py", dbs[k - 1], out, "--gain", "--dt", "robot", "--traj", traj[k - 1], "--mount", MOUNT,
            "--views", 15, "--pct", 25, "--camera", cam, "--corrections", corr], str(out) + ".log", env)
    def views(k):
        out = S / "base" / f"views_n{k}.npz"
        if out.exists(): return
        sh([PY, "baseline_views.py", run, k, dbs[k - 1], "--stack", 4, "--mount", MOUNT, "--traj", traj[k - 1],
            "--corrections", shape / f"corrections_{k}.npz", "--out", out], str(out) + ".log", env)
    with ThreadPoolExecutor(jobs) as ex:                     # the stitch's views and pictures, rebuilt
        list(ex.map(lambda f: f(), [lambda k=k: views(k) for k in (1, 2)] +
                    [lambda k=k: mosaic(k, shape / f"corrections_{k}.npz", S / "base" / f"mosaic_n{k}.npz") for k in (1, 2)]))
    say(f"stitch's views and pictures rebuilt ({cam})")
    if not (S / "refine" / "summary.json").exists():
        sh([PY, "floor_refine.py", "--views", S / "base" / "views_n1.npz", S / "base" / "views_n2.npz", "--dbs", *dbs,
            "--init", shape, "--occ", S / "base" / "mosaic_n1.npz", S / "base" / "mosaic_n2.npz",
            "--out", S / "refine", "--workers", 3], S / "refine" / "refine.log", env)
    say("refined")
    with ThreadPoolExecutor(jobs) as ex:
        list(ex.map(lambda k: mosaic(k, S / "refine" / f"corrections_{k}.npz", S / "refine" / f"mosaic_n{k}.npz"), (1, 2)))
    say("refined pictures built")
    import floor_agreement as fa, floor_fidelity as ff
    flog = S / "floor_map_error.jsonl"; res = {"seed": seed, "camera": cam, "validation": seed == 78, "plants": [list(p) for p in plants]}
    place = {}
    for arm, cdir in (("springs", shape), ("refined", S / "refine")):
        r = {}
        for k in (1, 2):
            pic = S / ("base" if arm == "springs" else "refine") / f"mosaic_n{k}.npz"
            sh([PY, "floor_fidelity.py", pic, "--seed", seed, "--night", k, "--traj", traj[k - 1], "--corrections",
                cdir / f"corrections_{k}.npz", "--error", run, "--log", flog, "--built", f"G9q {arm}"], S / f"fidelity_{arm}_n{k}.log", env)
            r[f"error_n{k}"] = [json.loads(l) for l in flog.read_text().splitlines()][-1]["error"]
            place[arm, k] = ff.placement_of(pic, seed, k, traj[k - 1], cdir / f"corrections_{k}.npz", run)   # [nv5] forgives 2 cm
            r[f"placement_n{k}"] = ff.placement_summary(place[arm, k], place.get(("springs", k)) if arm == "refined" else None)
        d = S / ("base" if arm == "springs" else "refine")
        ag, _ = fa.agreement(run, seed, d / "mosaic_n1.npz", d / "mosaic_n2.npz")
        r["disagree_open"], r["disagree_all"] = ag["open"][0], ag["all"][0]
        res[arm] = r
    from houndkeeper import calibrate as _cal
    res["builder"] = {p.name: _cal.file_sha(p) for p in BUILDER if p.exists()}
    res["refine"] = json.loads((S / "refine" / "summary.json").read_text())
    (S / "result.json").write_text(json.dumps(res, indent=1))
    b, f = res["springs"], res["refined"]
    say(f"error n1 {b['error_n1']:.2%} -> {f['error_n1']:.2%}, n2 {b['error_n2']:.2%} -> {f['error_n2']:.2%}; nights disagree "
        f"(open) {b['disagree_open']:.2%} -> {f['disagree_open']:.2%}, (all) {b['disagree_all']:.2%} -> {f['disagree_all']:.2%}; "
        + "; ".join(f"placement n{k} {b[f'placement_n{k}'].get('abs_mm_mean')} -> {f[f'placement_n{k}'].get('abs_mm_mean')} mm "
                    f"(paired {f[f'placement_n{k}'].get('paired_mm')} mm over {f[f'placement_n{k}'].get('paired_patches')} patches)" for k in (1, 2)))
    return res


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("seeds", type=int, nargs="+"); ap.add_argument("--scratch", required=True)
    ap.add_argument("--jobs", type=int, default=2, help="pictures and views built at once")
    ap.add_argument("--plant", action="append", default=[], metavar="N:F0-F1:DX,DY,DTH",
                    help="the positive control: plant a known error in night N's corrections, the block from F0 to F1 of its time moved DX, DY m and turned DTH deg")
    a = ap.parse_args(); sys.path.insert(0, str(SIM)); sys.path.insert(0, str(HK / "src"))
    plants = [parse_plant(p) for p in a.plant]
    for s in a.seeds:
        try: bench(s, a.scratch, a.jobs, plants)
        except SystemExit as e: print(f"s{s}: {e}", flush=True)


if __name__ == "__main__":
    main()
