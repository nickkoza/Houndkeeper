#!/usr/bin/env python3
"""The head camera's floor geometry ([4t7], [JzW]; the mount's calibration, [gJq]): why the floor seen from a camera sits
2-4 cm toward it, and the calibration that takes it off at its source.

A frame's floor is laid on the floor plane (z = 0) from the camera's believed pose: the body's pose at the frame's
stamp less the robot's frame lag, the head camera's place on the body (DimOS's published mount) and the calibrated
turn (head_mount.json). Each of these, if wrong, moves the floor along the line of sight by its own law of range r,
for a camera at height h:
  pitch  (the camera turned down by p rad more than believed):  dr = -p (r^2 + h^2) / h    (grows as r^2)
  height (the camera dh higher than believed), or a range scale: dr = +dh r / h             (grows as r)
  forward place (the camera df further forward):                  dr = +df                  (constant)
  the floor's own height under the patch (a carpet's z):        dr = -z r / h             (as a lower camera)
(dr: the true floor's range less where the believed geometry lays it; positive = the picture sits toward the camera).

The robot's own self-calibration (the views alone, no truth) and the house's relief are in floor_selfcal.py, on the
robot's side; this file keeps the sim-only diagnosis and the offline steps over tuning houses.

Subcommands (sim/house, dimos venv):
  truth <seed> --night n --out <json>    analysis only (reads the sim's truth: the true path, the true floor texture,
        the render's planted mount error, the floor's relief): each frame laid with the true body pose and the robot's
        believed camera geometry, registered on the true floor texture in range bands, and each candidate cause's
        predicted offset over the same cells. Never a hold-out house.
  measure <views.npz ...> --out <json>   the robot's own: the facing-apart offset (dirfloor's radial model) in range
        bands, from the views alone, with an optional geometry change applied to every view first (--dpitch, --dup,
        --dfwd: the positive control and the solve's trial).
  selfcal <views.npz ...> --out <json>   one house's self-calibration (floor_selfcal.selfcal), with plants and fixes.
  propose <selfcal json ...> --out-mount <json> --out-lag <json>   the robot's calibration from many houses' solves,
        written to new files (a proposal; never into data/robot/).
  calibrate <selfcal json ...>   the robot's calibration written ([gJq], decision [yFg]): the one writer of the head
        camera's mount and the floor rows' lag in data/robot/, from tuning houses only (hold-outs and the validation
        house refused before anything is read); both files copied to dated files beside them first, and each new file
        records the previous values, when, and from what."""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import argparse, json, math, time
from pathlib import Path

import numpy as np

from floor_selfcal import (BANDS, TILE_M, TILE_MIN_CELLS, REACH, SELFCAL, _register_cells, regeom, heights, consensus,   # the
                           view_rates, lag_moves, tile_registrations, solve_tiles, along_by_range, floor_level,          # robot's
                           relief_grid, selfcal, views_stamp)                                                            # own

HERE = Path(__file__).resolve().parent; HK = HERE.parents[1]
VALIDATION = {78}   # the validation house: never tuning data, never calibration data


def _holdouts():
    """the houses never measured or calibrated on: the generator's hold-out registry and the validation house"""
    import make_house
    return set(make_house.HOLDOUT) | VALIDATION


KINDS = ("hard", "carpet", "rug")


# --- geometry -----------------------------------------------------------------------------------------------------

def predict_dr(r, h, dpitch=0.0, dup=0.0, dfwd=0.0, zfloor=0.0):
    """how far the floor at range r sits toward the camera (m, the law above) for a camera at height h whose believed
    geometry is off by: dpitch (rad, the camera truly turned down more than believed), dup (m, truly higher), dfwd (m,
    truly further forward), with the floor truly zfloor above the plane the projection assumes"""
    r = np.asarray(r, float)
    return -dpitch * (r ** 2 + h ** 2) / h + dup * r / h + dfwd - zfloor * r / h


def floor_points(C, R, d_cam, z=0.0):
    """where rays (camera frame directions d_cam, n x 3) from a camera at C (world <- optical R) meet the plane at
    height z (n x 3; NaN where a ray never comes down)"""
    w = d_cam @ R.T; s = (z - C[2]) / np.where(w[:, 2] < -1e-6, w[:, 2], np.nan)
    return C + w * np.asarray(s)[:, None]


def true_mount(run):
    """analysis only: the head camera's planted mount error ([nAH] 55b) as the render stamp records it, in the terms
    calibrate.mount uses: (turn in the optical frame, place in the body frame (forward, left, up) m). sim_hook turns
    the MuJoCo camera (x right, y up, z back) about x by -pitch, y by +yaw, z by +roll; in the optical frame (y down,
    z forward) that is _rot(-pitch, -yaw, -roll)"""
    from houndkeeper import calibrate as cal
    st = json.loads((Path(run) / "render_n1.json").read_text().split("} scene=")[0] + "}") if "scene=" in (Path(run) / "render_n1.json").read_text() \
        else json.loads((Path(run) / "render_n1.json").read_text())
    e = (st.get("imperfections") or {}).get("mount")
    if not e: return np.eye(3), np.zeros(3), None
    R = cal._rot(math.radians(-e["pitch_deg"]), math.radians(-e["yaw_deg"]), math.radians(-e["roll_deg"]))
    return R, np.array(e["pos_m"], float), e


def mount_angles(R):
    """(pitch, yaw, roll) deg of a small turn R = _rot(p, y, r) (Rx Ry Rz)"""
    y = math.asin(np.clip(R[0, 2], -1, 1)); p = math.atan2(-R[1, 2], R[2, 2]); r = math.atan2(-R[0, 1], R[0, 0])
    return math.degrees(p), math.degrees(y), math.degrees(r)


def euler_zyx(R):
    """(yaw, pitch, roll) rad of a world <- body rotation (ZYX)"""
    return math.atan2(R[1, 0], R[0, 0]), math.asin(-np.clip(R[2, 0], -1, 1)), math.atan2(R[2, 1], R[2, 2])


def rot_zyx(yaw, pitch, roll):
    cy, sy, cp, sp, cr, sr = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


# --- truth (analysis only) -----------------------------------------------------------------------------------------

def truth_main(a):
    import confirm_stream as cs, floor_fidelity as ff, dirfloor as dfl
    from dimos.memory.store.sqlite import SqliteStore
    from floor_mosaic2 import arr, frame_lags, LAG_FILE, clock_of
    from houndkeeper import calibrate as cal
    from houndkeeper.lens import Rectifier
    from detect_changes2 import RES, HX, HY
    if a.seed in _holdouts(): raise SystemExit("a hold-out house: never measured")
    run = HERE / "runs" / f"nights_s{a.seed}"
    dbs = {int(l.split()[0]): l.split()[1] for l in (run / "seed_recordings.txt").read_text().splitlines() if l.strip()}
    db = dbs[a.night]; traj = run / f"nights_s{a.seed}_n{a.night}.traj.csv"
    Rt, offt, err = true_mount(run)
    mj = json.loads(Path(a.mount).read_text()); Rs, offs = cal.mount(mj)
    if a.plant_pitch or a.plant_up:                                  # a known error planted in the believed geometry
        Rs = Rs @ cal._rot(math.radians(-a.plant_pitch), 0.0, 0.0); offs = offs - np.array([0.0, 0.0, a.plant_up])
    print(f"true mount (optical turn) {np.round(mount_angles(Rt), 3)} deg, place {offt} m; believed {np.round(mount_angles(Rs), 3)} deg, place {offs}", flush=True)
    T = np.genfromtxt(traj, delimiter=",", names=True)
    tt, tx, ty, tyaw = T["wall_t"], T["true_x"], T["true_y"], np.unwrap(T["true_yaw"])
    base, Vb, band, hmap, _ = cs.baseline(run, a.night + 1) if (run / "mosaic_n1.npz").exists() and a.night >= 1 else (None,) * 5
    st = SqliteStore(path=db); st.start()
    ci = next(iter(st.stream("camera_info"))).data; rect = Rectifier.from_camera_info(ci)
    pose, T_bo = cs.poses(st)
    lag, note = frame_lags(st, "robot", ["color_image"], LAG_FILE, check=False, clock=clock_of(db, "auto", str(traj)))["color_image"]
    if a.lag is not None: lag = a.lag
    print(f"lag {lag:.3f} s ({note}); true frame delay {a.true_delay} s", flush=True)
    painter = cs.Confirmer(base, Vb, band, hmap, rect.K, (ci.width, ci.height), argparse.Namespace(**cs.CONFIRM))
    tex = ff.truth(a.seed, a.night, Vb.shape)                       # the true floor, the sim's frame
    B = ff.structure(tex.mean(-1), np.ones(Vb.shape, bool)); S = a.reach
    K, edge = dfl.kind_map(a.seed, run, (0, Vb.shape[0], 0, Vb.shape[1]))
    tr = json.loads((run / f"nights_s{a.seed}_n1.truth.json").read_text()); rel = tr.get("relief_m") or {}
    zc = float(rel.get("carpet", 0.0)); zr = float(rel.get("rug", 0.0))
    rug_on_carpet = False
    if tr.get("rug"):
        rx, ry = tr["rug"]["x"], tr["rug"]["y"]
        from baselines import nights as nn
        rug_on_carpet = any(x0 <= rx <= x1 and y0 <= ry <= y1 for x0, x1, y0, y1 in nn.carpet_rects(a.seed))
    Zf = np.where(K == 1, zc, np.where(K == 2, zr + (zc if rug_on_carpet else 0.0), 0.0)).astype(np.float32)
    Bpad = np.pad(B, S, mode="constant")
    def true_body(t):
        if t < tt[0] or t > tt[-1]: return None
        M = pose(t)
        if M is None: return None
        _, p_, r_ = euler_zyx(M[:3, :3]); Mt = np.eye(4)
        Mt[:3, :3] = rot_zyx(float(np.interp(t, tt, tyaw)), p_, r_); Mt[:3, 3] = [np.interp(t, tt, tx), np.interp(t, tt, ty), M[2, 3]]
        return Mt
    rows = []; t0 = time.time(); kept = 0
    for k, o in enumerate(st.stream("color_image")):
        if k % a.every: continue
        Mb = true_body(o.ts - lag); Mtr = true_body(o.ts - a.true_delay)
        wz = math.degrees((np.interp(o.ts - lag + 0.05, tt, tyaw) - np.interp(o.ts - lag - 0.05, tt, tyaw)) / 0.1)   # turning, deg/s
        if Mb is None or Mtr is None: continue
        Tb = Mb @ T_bo; Tb[:3, :3] = Tb[:3, :3] @ Rs; Tb[:3, 3] += Mb[:3, :3] @ offs
        p = painter.paint(rect(arr(o.data)), Tb)
        if p is None: continue
        sl, pr, pc_, dd, patch, V = p
        g = patch.astype(np.float32).mean(-1); A = ff.structure(g, V)
        r0, c0 = sl[0].start, sl[1].start; Rr, Cc = pr + r0, pc_ + c0
        f = Tb[:3, 2][:2]; f = f / max(np.linalg.norm(f), 1e-9); left = np.array([-f[1], f[0]])
        Cb = Tb[:3, 3]; Rb = Tb[:3, :3]
        X = -HX + (Cc + 0.5) * RES; Y = HY - (Rr + 0.5) * RES; Pb = np.c_[X, Y, np.zeros(len(X))]
        dcam = (Pb - Cb) @ Rb                                        # each cell's line of sight, camera frame
        # each candidate cause on its own: the true geometry for that cause, everything else as believed
        Tt = Mb @ T_bo; Rmt = Tt[:3, :3] @ Rt; Cmt = Tt[:3, 3] + Mb[:3, :3] @ offt
        Tl = Mtr @ T_bo; Rl = Tl[:3, :3] @ Rs; Cl = Tl[:3, 3] + Mtr[:3, :3] @ offs
        Rall = Tl[:3, :3] @ Rt; Call = Tl[:3, 3] + Mtr[:3, :3] @ offt
        zf = Zf[Rr, Cc]
        cand = {"turn": floor_points(Cb, Rmt, dcam), "place": floor_points(Cb + Mb[:3, :3] @ (offt - offs), Rb, dcam),
                "relief": _relief_points(Cb, Rb, dcam, zf), "lag": floor_points(Cl, Rl, dcam),
                "all": _relief_points(Call, Rall, dcam, zf)}
        for bi, (lo, hi) in enumerate(BANDS):
            inb = (dd >= lo) & (dd < hi) & ~edge[Rr, Cc]
            for kk in range(3):
                m = inb & (K[Rr, Cc] == kk)
                if m.sum() < a.min_cells: continue
                rr_, cc_ = Rr[m], Cc[m]
                if B[rr_, cc_].std() < 0.3: continue
                reg = _register_cells(A, pr[m], pc_[m], Bpad, rr_, cc_, S, step=2 if S > 10 else 1)
                if reg is None or reg[2] < a.min_corr: continue
                di, dj, pk = reg; e = np.array([-dj * RES, di * RES])   # the picture's floor less the true floor (world m)
                row = {"k": k, "band": bi, "kind": kk, "n": int(m.sum()), "peak": round(pk, 3), "h": round(float(Cb[2] - zf[m].mean()), 4), "turn_dps": round(wz, 1),
                       "meas": [float(-(e @ f)), float(e @ left)]}
                for nm, Pt in cand.items():
                    ev = (Pb[m, :2] - Pt[m, :2]); ev = np.nanmean(ev, 0)
                    row[nm] = [float(-(ev @ f)), float(ev @ left)]
                rows.append(row)
        kept += 1
        if a.limit and kept >= a.limit: break
        if kept % 50 == 0: print(f"  {kept} frames, {len(rows)} band registrations, {time.time() - t0:.0f} s", flush=True)
    st.stop()
    out = summarize(rows); out.update(seed=a.seed, night=a.night, lag_s=lag, true_delay_s=a.true_delay, frames=kept, mount_file=str(a.mount),
                                      believed_mount_deg=mount_angles(Rs), believed_place_m=offs.tolist(), true_mount_deg=mount_angles(Rt),
                                      true_place_m=offt.tolist(), plant={"pitch_deg": a.plant_pitch, "up_m": a.plant_up}, relief_m={"carpet": zc, "rug": zr, "rug_on_carpet": rug_on_carpet})
    Path(a.out).write_text(json.dumps(out, indent=1)); np.save(Path(a.out).with_suffix(".rows.npy"), np.array(rows, dtype=object), allow_pickle=True)
    print_summary(out)


def _relief_points(C, R, dcam, zf):
    w = dcam @ R.T; s = (zf - C[2]) / np.where(w[:, 2] < -1e-6, w[:, 2], np.nan); return C + w * s[:, None]


def summarize(rows):
    out = {"bands": [list(b) for b in BANDS], "by": {}}
    for kk, kn in enumerate(KINDS):
        for bi in range(len(BANDS)):
            rs = [r for r in rows if r["kind"] == kk and r["band"] == bi]
            if len(rs) < 5: continue
            d = {"n": len(rs), "h_m": round(float(np.median([r["h"] for r in rs])), 4)}
            for key in ("meas", "turn", "place", "relief", "lag", "all"):
                v = np.array([r[key] for r in rs], float)
                d[key] = [round(float(np.nanmedian(v[:, 0])) * 1000, 1), round(float(np.nanmedian(v[:, 1])) * 1000, 1)]
            d["meas_iqr_mm"] = [round(float(np.subtract(*np.percentile(np.array([r["meas"][0] for r in rs]), [75, 25]))) * 1000, 1)]
            out["by"][f"{kn} {BANDS[bi][0]}-{BANDS[bi][1]}"] = d
    return out


def print_summary(out):
    print("along / across (mm; along + = the picture's floor toward the camera, across + = to the camera's left)")
    print(f"{'kind, band (m)':<20}{'n':>5}  {'measured':>14}  {'turn':>12}  {'place':>12}  {'relief':>12}  {'lag':>12}  {'all':>12}")
    for k, d in out["by"].items():
        f = lambda v: f"{v[0]:+6.1f}/{v[1]:+5.1f}"
        print(f"{k:<20}{d['n']:>5}  {f(d['meas']):>14}  {f(d['turn']):>12}  {f(d['place']):>12}  {f(d['relief']):>12}  {f(d['lag']):>12}  {f(d['all']):>12}")


# --- the robot's own measure: the facing-apart offset by range, from its views -------------------------------------

MEASURE_BANDS = ((0.5, 1.1), (1.1, 1.7), (1.7, 2.5))   # wider than BANDS: a direction bin needs 3 views a cell in a band


def in_band(views, lo, hi):
    """the views with only their cells lo-hi m from the camera's foot"""
    import confirm_stream as cs
    from detect_changes2 import RES, HX, HY
    P = []
    for k, (p, (r0, r1, c0, c1)) in enumerate(zip(views.patches, views.sls)):
        rr, cc = np.mgrid[r0:r1, c0:c1]; d = np.hypot(-HX + (cc + 0.5) * RES - views.C[k][0], HY - (rr + 0.5) * RES - views.C[k][1])
        P.append(np.where(((d >= lo) & (d < hi))[..., None], p, 0).astype(np.uint8))
    return cs.Views(views.C, views.F, views.sls, P, views.ts, views.Ts)


MEASURE = dict(nb=8, min_views=2, min_cells=1500)   # looser than dirfloor's (3 views, 3000 cells): a band holds fewer views


def measure(views, bands=MEASURE_BANDS, nb=None, log=None, min_views=None, min_cells=None):
    """the facing-apart offset in each range band, the robot's own (dirfloor's radial model over the views' cells in
    that band): [{band, along m (the floor toward its camera), across m (to the camera's left), pairs}], None where too
    few bin pairs register"""
    import dirfloor as dfl
    nb = nb or MEASURE["nb"]; mv = min_views or MEASURE["min_views"]; mc = min_cells or MEASURE["min_cells"]
    out = []; keep = dfl.MIN_VIEWS
    for lo, hi in bands:
        m = dfl.DirFloor.build(in_band(views, lo, hi), nb, grey_only=True)
        dfl.MIN_VIEWS = mv
        try: B = m.bias(min_cells=mc)
        finally: dfl.MIN_VIEWS = keep
        rd = m.radial(B)
        pairs = sum(1 for (x, y), v in B.items() if x < y and v[2] >= 3)
        out.append({"band": [lo, hi], "r_m": None, "along": None if rd is None else rd[1][0], "across": None if rd is None else -rd[1][1],
                    "pairs": pairs})
        out[-1]["r_m"] = _band_range(views, lo, hi)
        if log: log(f"  band {lo}-{hi} m (r {out[-1]['r_m']:.2f}): along {1000 * (out[-1]['along'] or float('nan')):+.1f} mm, across {1000 * (out[-1]['across'] or float('nan')):+.1f} mm, {pairs} bin pairs")
        del m
    return out


def _band_range(views, lo, hi):
    """the mean range of the views' cells in a band (m)"""
    from detect_changes2 import RES, HX, HY
    s = n = 0.0
    for k, (p, (r0, r1, c0, c1)) in enumerate(zip(views.patches[::7], views.sls[::7])):
        k = k * 7; V = p.any(-1); rr, cc = np.nonzero(V)
        d = np.hypot(-HX + (cc + c0 + 0.5) * RES - views.C[k][0], HY - (rr + r0 + 0.5) * RES - views.C[k][1]); d = d[(d >= lo) & (d < hi)]
        s += d.sum(); n += len(d)
    return s / max(n, 1)


def measure_main(a):
    import confirm_stream as cs
    v = cs.Views.load(a.views); h = heights(v); t0 = time.time()
    print(f"{len(v.C)} views, camera height {h:.3f} m", flush=True)
    if a.dpitch or a.dup or a.dfwd:
        v = regeom(v, math.radians(a.dpitch), a.dup, a.dfwd); print(f"geometry changed: pitch {a.dpitch:+.3f} deg, up {a.dup:+.4f} m, forward {a.dfwd:+.4f} m ({time.time() - t0:.0f} s)", flush=True)
    bands = tuple(tuple(b) for b in a.bands) if a.bands else MEASURE_BANDS
    m = measure(v, bands, a.nb, print, a.min_views, a.min_cells); print(f"({time.time() - t0:.0f} s)")
    Path(a.out).write_text(json.dumps({"views": a.views, "h_m": h, "applied": {"dpitch_deg": a.dpitch, "dup_m": a.dup, "dfwd_m": a.dfwd},
                                       "bands": m}, indent=1))


# --- self-calibration from the views against their own consensus ---------------------------------------------------


def replaced(view_files, painted, poses):
    """the views placed by other poses: each view moved by poses(t) painted(t)^-1 (the night's corrections, house <-
    odometry, at its time). view_files night 1 first; painted, poses: directories with corrections_<n>.npz"""
    import confirm_stream as cs
    from houndkeeper.localize import Localizer
    L = lambda d, n: (lambda z: Localizer(times=list(z["times"]), Ts=list(z["Ts"])))(np.load(Path(d) / f"corrections_{n}.npz"))
    moves = []
    for n, f in enumerate(view_files, 1):
        v = cs.Views.load([f]); a, b = L(painted, n), L(poses, n)
        moves += [b.correction(t) @ np.linalg.inv(a.correction(t)) for t in v.ts]
    return regeom(cs.Views.load(view_files), moves=moves)


def selfcal_main(a):
    import confirm_stream as cs
    t0 = time.time()
    if a.poses:
        v = replaced(a.views, a.painted, a.poses); print(f"views placed by {a.poses} (painted at {a.painted}), {time.time() - t0:.0f} s", flush=True)
    else: v = cs.Views.load(a.views)
    plant = (math.radians(a.plant_pitch), a.plant_up, a.plant_lag) if (a.plant_pitch or a.plant_up or a.plant_lag) else None
    if plant: print(f"planted first: every view's camera taken as turned down {a.plant_pitch:+.3f} deg more and {a.plant_up * 1000:+.1f} mm higher", flush=True)
    fixed = None
    if a.fixed:                                                     # the robot's calibration, from `propose`'s mount file
        fx = json.loads(Path(a.fixed).read_text())["selfcal"]["median_change"]
        fixed = (math.radians(fx["dpitch_deg"]), math.radians(fx["dyaw_deg"]), fx["dup_m"], fx["dlag_s"])
        if a.fixed_zero: fixed = (0.0, 0.0, 0.0, 0.0)              # the views already painted with it: relief only
        print(f"the robot's calibration held: {[round(x, 5) for x in fixed]}", flush=True)
    r = selfcal(v, rounds=a.rounds, gain=a.gain, free=tuple(a.free), every=a.every, log=lambda m: print(f"{m} ({time.time() - t0:.0f} s)", flush=True), plant=plant, fixed=fixed)
    rel = r.pop("relief")
    if rel is None: rel = np.zeros((int(round(2 * 4.9 / 0.01)), int(round(2 * 8.4 / 0.01))), np.float32)   # flat: none raised
    np.save(Path(a.out).with_suffix(".relief.npy"), rel.astype(np.float32))
    r.update(views=a.views, painted_with=views_stamp(a.views), poses=a.poses, painted=a.painted, plant={"pitch_deg": a.plant_pitch, "up_m": a.plant_up, "lag_s": a.plant_lag}, relief_file=str(Path(a.out).with_suffix(".relief.npy")))
    Path(a.out).write_text(json.dumps(r, indent=1))
    print(f"self-calibration (add to the belief): pitch {r['dpitch_deg']:+.3f} deg (down +), yaw {r['dyaw_deg']:+.3f} deg (left +), up {r['dup_m'] * 1000:+.1f} mm, "
          f"floor lag {r['dlag_s'] * 1000:+.1f} ms (the frames truly later; take it off the lag); "
          f"{sum(1 for z in r['relief_tiles'].values() if z > 0)} raised tiles ({time.time() - t0:.0f} s)")


UP_PCT = 50


def robot_change(runs, min_regs=2000):
    """the robot's calibration change from many houses' self-calibrations (each house's views painted with the same
    mount and lag): the median over the houses with enough floor (min_regs registrations). Returns (median change
    {dpitch_deg, dyaw_deg, dup_m, dlag_s}, inter-quartile range, houses kept, houses whose views saw the lag)"""
    ok = [r for r in runs if r["rounds"][-1]["registrations"] >= min_regs]
    if not ok: raise SystemExit("no house with enough floor")
    med = {k: float(np.median([r[k] for r in ok])) for k in ("dpitch_deg", "dyaw_deg", "dup_m", "dlag_s")}
    seen_lag = [r for r in ok if r["rounds"][0]["seen"].get("dlag_s", 0.0) != 0.0]   # the lag is seen only where the views
    if seen_lag: med["dlag_s"] = float(np.median([r["dlag_s"] for r in seen_lag]))   # are near enough in time for a turning
    # rate (view_rates: neighbours within 1 s; houses 82-93 keep a view every 1.4-2.2 s): elsewhere its column is empty
    med["dup_m"] = float(np.percentile([r["dup_m"] for r in ok], UP_PCT))   # a house's floor level is its highest broad
    # level; where carpet covers most of a house that level is the carpet's, and its height reads low by the pile (sim:
    # 8-20 mm, houses 72 and 73), while a house with little floor texture scatters either way: the median of the houses
    iqr = {k: float(np.subtract(*np.percentile([r[k] for r in ok], [75, 25]))) for k in med}
    return med, iqr, ok, seen_lag


def changed_mount(m, med):
    """a mount file's dict with the change applied (the camera's pitch, yaw and height; roll and place kept)"""
    return {"pitch_deg": round(m["pitch_deg"] - med["dpitch_deg"], 3), "yaw_deg": round(m["yaw_deg"] - med["dyaw_deg"], 3),
            "roll_deg": m.get("roll_deg", 0.0), "fwd_m": m.get("fwd_m", 0.0), "left_m": m.get("left_m", 0.0),
            "up_m": round(m.get("up_m", 0.0) + med["dup_m"], 4), "from_selfcal": True}


def propose_main(a):
    """the robot's calibration from many houses' self-calibrations: the median change applied to the stored mount and
    the stored lag; written to new files, never over the robot's own"""
    runs = [json.loads(Path(f).read_text()) for f in a.selfcal]
    med, iqr, ok, seen_lag = robot_change(runs, a.min_regs)
    m = json.loads(Path(a.mount).read_text())
    out = dict(changed_mount(m, med), selfcal={"from": a.mount, "houses": len(ok), "of": len(runs), "lag_houses": len(seen_lag),
                                                "median_change": med, "iqr": iqr, "files": [str(f) for f in a.selfcal]})
    Path(a.out_mount).write_text(json.dumps(out, indent=1))
    lag = json.loads(Path(a.lag).read_text())
    for clk, e in (lag.get("clocks") or {}).items():          # the floor's rows: the stored lag less the measured offset,
        if clk not in a.lag_clocks: continue                  # on the clocks it was measured on
        st = e.get("streams", {}).get("color_image")
        if st and st.get("lag_s") is not None: st["lag_s"] = round(st["lag_s"] - med["dlag_s"], 4)
        if e.get("lag_s") is not None: e["lag_s"] = round(e["lag_s"] - med["dlag_s"], 4)
        e["floor_rows_offset_s"] = round(-med["dlag_s"], 4)
    Path(a.out_lag).write_text(json.dumps(lag, indent=1))
    print(f"{len(ok)} of {len(runs)} houses; median change pitch {med['dpitch_deg']:+.3f} deg (iqr {iqr['dpitch_deg']:.3f}), yaw {med['dyaw_deg']:+.3f} "
          f"({iqr['dyaw_deg']:.3f}), up {med['dup_m'] * 1000:+.1f} mm ({iqr['dup_m'] * 1000:.1f}), lag {med['dlag_s'] * 1000:+.1f} ms ({iqr['dlag_s'] * 1000:.1f})")
    print(f"mount -> {a.out_mount}: pitch {out['pitch_deg']} yaw {out['yaw_deg']} roll {out['roll_deg']} up {out['up_m']} m; lag -> {a.out_lag}")


def seed_of(path, d):
    """the house a self-calibration was solved for: from its file's name (s<seed>...) and its views' paths
    (.../s<seed>/... or nights_s<seed>), which must agree; a SystemExit when neither names one"""
    import re
    found = set()
    m = re.match(r"s(\d+)(?:_|\.|$)", Path(path).name)
    if m: found.add(int(m.group(1)))
    for v in d.get("views") or []:
        for m in re.finditer(r"(?:^|/)(?:nights_)?s(\d+)(?:/|_n)", str(v)): found.add(int(m.group(1)))
    if len(found) != 1: raise SystemExit(f"{path}: cannot tell which house it was solved for ({sorted(found) or 'none named'})")
    return found.pop()


def write_calibration(selfcal_files, robot_dir, base_mount=None, base_lag=None, lag_clocks=("wall",), min_regs=2000,
                      now=None, log=print, dry_run=False):
    """the robot's calibration written ([gJq], decision [yFg]): the one writer of data/robot/head_mount.json (the
    mount's pitch, yaw and height) and of the floor rows' offset in data/robot/frame_lag.json. Every self-calibration
    must be a tuning house's (hold-outs and the validation house refused before any is read further) and solved on
    views painted with base_mount and base_lag (their stamps checked where they carry one; given explicitly where they
    do not). Both robot files are copied to dated files beside them first; each new file records the previous values,
    when, and from what. Returns {mount, lag, backups}"""
    import shutil
    from houndkeeper import calibrate as cal
    robot = Path(robot_dir); mount_p, lag_p = robot / "head_mount.json", robot / "frame_lag.json"
    docs = [(f, json.loads(Path(f).read_text())) for f in selfcal_files]
    seeds = {str(f): seed_of(f, d) for f, d in docs}
    bad = sorted(s for s in seeds.values() if s in _holdouts())
    if bad: raise SystemExit(f"hold-out or validation houses {bad} are never calibration data: nothing written")
    explicit = base_mount is not None and base_lag is not None
    base_mount = Path(base_mount or mount_p); base_lag = Path(base_lag or lag_p)
    want = {"mount_sha": cal.file_sha(base_mount), "lag_sha": cal.file_sha(base_lag)}
    for f, d in docs:
        pw = d.get("painted_with")
        if pw is None and not explicit:
            raise SystemExit(f"{f} does not say what its views were painted with: give --base-mount and --base-lag")
        if pw is not None and any(pw.get(k) != v for k, v in want.items()):
            raise SystemExit(f"{f}'s views were painted with {pw}, not the base calibration {want}: nothing written")
    import hashlib
    files_sha = hashlib.sha256("".join(sorted(cal.file_sha(f) for f, _ in docs)).encode()).hexdigest()[:12]
    cur_m = json.loads(mount_p.read_text()) if mount_p.exists() else {}
    prev_src = (cur_m.get("calibration") or {}).get("source") or {}
    if prev_src.get("selfcal_sha") == files_sha and prev_src.get("base_mount_sha") == want["mount_sha"]:
        raise SystemExit("the robot's calibration was already written from these self-calibrations on this base: nothing written")
    med, iqr, ok, seen_lag = robot_change([d for _, d in docs], min_regs)
    now = now or time.strftime("%Y-%m-%dT%H:%M:%S%z")
    stamp = now[:19].replace("-", "").replace(":", "").replace("T", "_")
    src = {"step": "camgeo.py calibrate", "selfcal": [str(f) for f, _ in docs], "selfcal_sha": files_sha,
           "seeds": sorted(set(seeds.values())), "houses_used": len(ok), "of": len(docs), "lag_houses": len(seen_lag),
           "min_registrations": min_regs, "base_mount": str(base_mount), "base_mount_sha": want["mount_sha"],
           "base_lag": str(base_lag), "base_lag_sha": want["lag_sha"], "median_change": med, "iqr": iqr}
    base_m = json.loads(base_mount.read_text())
    keys = ("pitch_deg", "yaw_deg", "roll_deg", "fwd_m", "left_m", "up_m")
    new_m = dict(changed_mount(base_m, med), selfcal={"from": str(base_mount), "houses": len(ok), "of": len(docs),   # (selfcal
                 "lag_houses": len(seen_lag), "median_change": med, "iqr": iqr, "files": src["selfcal"]})        #  --fixed reads it)
    new_m["calibration"] = {"written": now, "source": src, "previous": {k: cur_m[k] for k in keys if k in cur_m},
                            "previous_sha": cal.file_sha(mount_p)}
    lag = json.loads(lag_p.read_text()); base_l = json.loads(base_lag.read_text())
    for clk in lag_clocks:
        e = (lag.get("clocks") or {}).get(clk)
        if e is None: raise SystemExit(f"the robot's lag file has no {clk} clock to give the floor rows' offset")
        be = cal.lag_entry(base_l, clk, str(base_lag))
        floor = cal.floor_lag(be, "color_image") - med["dlag_s"]           # the base's floor lag, less what the views found
        frame = float(((e.get("streams") or {}).get("color_image") or {}).get("lag_s", e["lag_s"]))
        e["floor_rows"] = {"floor_lag_s": round(floor, 4), "written": now, "step": src["step"], "selfcal": src["selfcal"],
                           "lag_houses": len(seen_lag), "median_change_s": med["dlag_s"], "iqr_s": iqr["dlag_s"],
                           "previous_offset_s": e.get("floor_rows_offset_s"), "base_lag_sha": want["lag_sha"]}
        e["floor_rows_offset_s"] = round(floor - frame, 4)
    out = {"mount": new_m, "lag": lag, "backups": []}
    if dry_run: return out
    for p in (mount_p, lag_p):
        if p.exists():
            b = p.with_name(f"{p.stem}.{stamp}{p.suffix}")
            if b.exists(): raise SystemExit(f"{b} exists: nothing written")
            shutil.copy2(p, b); out["backups"].append(str(b)); log(f"backed up {p.name} -> {b}")
    new_m["calibration"]["backups"] = out["backups"]
    for clk in lag_clocks: lag["clocks"][clk]["floor_rows"]["backup"] = next((b for b in out["backups"] if "frame_lag" in b), None)
    mount_p.write_text(json.dumps(new_m, indent=1)); lag_p.write_text(json.dumps(lag, indent=1))
    log(f"head mount: pitch {cur_m.get('pitch_deg')} -> {new_m['pitch_deg']} deg, yaw {cur_m.get('yaw_deg')} -> {new_m['yaw_deg']} deg, "
        f"roll {new_m['roll_deg']}, up {cur_m.get('up_m', 0.0)} -> {new_m['up_m']} m ({len(ok)} of {len(docs)} houses) -> {mount_p}")
    for clk in lag_clocks:
        e = lag["clocks"][clk]; log(f"floor rows' lag ({clk} clock): {e['floor_rows']['floor_lag_s']} s (offset {e['floor_rows_offset_s']:+.4f} s on the stored "
                                    f"{e['lag_s']:.4f} s; {len(seen_lag)} houses saw it) -> {lag_p}")
    return out


def calibrate_main(a):
    write_calibration(a.selfcal, a.robot_dir, a.base_mount, a.base_lag, tuple(a.lag_clocks), a.min_regs, dry_run=a.dry_run)


def main(argv=None):
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="cmd", required=True)
    t = sub.add_parser("truth"); t.add_argument("seed", type=int); t.add_argument("--night", type=int, default=1); t.add_argument("--out", required=True)
    t.add_argument("--mount", default=str(HK / "data" / "robot" / "head_mount.json")); t.add_argument("--every", type=int, default=8)
    t.add_argument("--limit", type=int, default=0); t.add_argument("--reach", type=int, default=8); t.add_argument("--min-cells", type=int, default=1500)
    t.add_argument("--min-corr", type=float, default=0.3); t.add_argument("--lag", type=float, default=None)
    t.add_argument("--true-delay", type=float, default=0.15, help="the sim's mean frame delay (the render stamp's frame_delay)")
    t.add_argument("--plant-pitch", type=float, default=0.0, help="positive control: the believed camera turned down this much more (deg)")
    t.add_argument("--plant-up", type=float, default=0.0, help="positive control: the believed camera this much lower (m)")
    for nm in ("measure",):
        m = sub.add_parser(nm); m.add_argument("views", nargs="+"); m.add_argument("--out", required=True)
        m.add_argument("--nb", type=int, default=None); m.add_argument("--min-views", type=int, default=None); m.add_argument("--min-cells", type=int, default=None)
        m.add_argument("--bands", type=float, nargs=2, action="append", default=None, help="a range band lo hi (m); repeat")
        if nm == "measure":
            m.add_argument("--dpitch", type=float, default=0.0, help="the believed camera turned down this much more first (deg)")
            m.add_argument("--dup", type=float, default=0.0, help="the believed camera this much higher first (m)")
            m.add_argument("--dfwd", type=float, default=0.0, help="the believed camera this much further forward first (m)")
    sc = sub.add_parser("selfcal"); sc.add_argument("views", nargs="+"); sc.add_argument("--out", required=True)
    sc.add_argument("--every", type=int, default=1); sc.add_argument("--free", nargs="+", default=list(SELFCAL["free"]), choices=["pitch", "fwd", "yaw", "lag"])
    sc.add_argument("--rounds", type=int, default=SELFCAL["rounds"]); sc.add_argument("--gain", type=float, default=SELFCAL["gain"])
    sc.add_argument("--plant-pitch", type=float, default=0.0, help="positive control: every view's camera taken as turned down this much more first (deg)")
    sc.add_argument("--plant-up", type=float, default=0.0, help="positive control: every view's camera taken as this much higher first (m)")
    sc.add_argument("--plant-lag", type=float, default=0.0, help="positive control: every view laid as its camera stood this much later (s)")
    sc.add_argument("--fixed", default=None, help="hold the robot's calibration (a mount file from `propose`): solve the floor's relief only, at the robot's floor level")
    sc.add_argument("--fixed-zero", action="store_true", help="--fixed, the views already painted with it: no change applied")
    sc.add_argument("--poses", default=None, help="place the views by these poses first (a directory with corrections_<n>.npz, e.g. the lidar shape solve's)")
    sc.add_argument("--painted", default=None, help="--poses: the corrections the views were painted at")
    pr = sub.add_parser("propose"); pr.add_argument("selfcal", nargs="+"); pr.add_argument("--mount", default=str(HK / "data" / "robot" / "head_mount.json"))
    pr.add_argument("--lag", default=str(HK / "data" / "robot" / "frame_lag.json")); pr.add_argument("--min-regs", type=int, default=2000)
    pr.add_argument("--lag-clocks", nargs="+", default=["wall"], help="the clocks whose lag takes the floor rows' offset (the houses' own)")
    pr.add_argument("--out-mount", required=True); pr.add_argument("--out-lag", required=True)
    ca = sub.add_parser("calibrate"); ca.add_argument("selfcal", nargs="+"); ca.add_argument("--robot-dir", default=str(HK / "data" / "robot"))
    ca.add_argument("--base-mount", default=None, help="the mount the self-calibrations' views were painted with (default: the robot's; needed when they do not say)")
    ca.add_argument("--base-lag", default=None, help="the lag file the views were painted with (default: the robot's; needed when they do not say)")
    ca.add_argument("--lag-clocks", nargs="+", default=["wall"], help="the clocks the houses' views were stamped on: their floor rows' lag is written")
    ca.add_argument("--min-regs", type=int, default=2000); ca.add_argument("--dry-run", action="store_true", help="print what would be written; write nothing")
    a = ap.parse_args(argv)
    if a.cmd == "calibrate": calibrate_main(a)
    if a.cmd == "propose": propose_main(a)
    if a.cmd == "selfcal": selfcal_main(a)
    if a.cmd == "truth": truth_main(a)
    elif a.cmd == "measure": measure_main(a)


if __name__ == "__main__":
    main()
