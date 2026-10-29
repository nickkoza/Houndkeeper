#!/usr/bin/env python3
"""Floor mosaic v2: height-aware visibility, median of the 3 closest views, interpolated poses.

A floor cell is painted from a frame only if the line of sight from the camera to it clears everything
in DimOS's own 3D map of the night (global_map, as a 2 cm height map). Tall things block the floor behind
them instead of smearing across it. Each cell keeps its 3 closest views; the output is their median.
Usage: floor_mosaic2.py <memory.db> <out.npz> [--every 4] [--rmax 2.5]   (dimos venv python)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper.lens
import argparse, time
import numpy as np
from scipy import ndimage as ndi
from dimos.memory.store.sqlite import SqliteStore
import floor_sample

HX, HY, RES = 8.4, 4.9, 0.01
W, H = int(2 * HX / RES), int(2 * HY / RES)
HRES, K = 0.02, 3
MIN_FRAMES, MIN_FLOOR_M2 = 20, 10.0          # a baseline picture: placed frames and floor seen, at least ([YuI], decision [NvG])
from houndkeeper.rec_poses import POSE_GAP_S   # a frame's pose is interpolated only between poses this close, on the recording's own clock: the
                   # robot's since [3Pv] (sim time in a sim; odometry every 35 ms at any speed). A recording stamped on the
                   # wall clock by a sim at r x real time needs 0.2 / r (--pose-gap): house 93 at 0.16x lost most frames


def baseline_refusal(used, floor_m2, min_frames=MIN_FRAMES, min_floor_m2=MIN_FLOOR_M2):
    """why a picture of `used` placed frames over `floor_m2` of floor is not a baseline, or None ([YuI]; 0: no limit).
    On disk (2 Oct): every picture used had 44+ frames and 20+ m2; the broken ones 1 frame (3.3 m2) and 7 (14.8 m2)"""
    if used >= min_frames and floor_m2 >= min_floor_m2: return None
    return (f"too little of the night for a floor picture: {used} frame{'s' if used != 1 else ''} placed, {floor_m2:.1f} m2 of "
            f"floor seen (needs {min_frames} frames and {min_floor_m2:g} m2)")


def arr(img):
    for c in (getattr(img, "data", None), img):
        try:
            a = np.asarray(c)
            if a.ndim == 3: return a
        except Exception: pass
    return img.to_numpy()


def wrist_extrinsic(iw, ih):
    """trunk -> the wrist camera's optical frame (z forward, x right, y down) with the arm stowed, from the arm sim's own
    model at the stow pose (the same arm and mount DimOS's sim carries: sim_hook adds it with go1_arm.add_arm). Looks
    41 deg down from 59 cm: several times finer floor than the head camera (the mapping bench, 26 Sep)"""
    import sys as _s, mujoco
    from pathlib import Path as _P
    _s.path.insert(0, str(_P(__file__).resolve().parents[1] / "arm")); import go1_arm as ga, payload_test as pt, legs
    m = ga.build(True, pt.STOW, pick=True); d = mujoco.MjData(m); legs.start_standing(m, d, 0.30); mujoco.mj_forward(m, d)
    cid = m.camera("arm_wrist").id; tr = m.body("trunk").id
    Rt, pt_ = d.xmat[tr].reshape(3, 3), d.xpos[tr]; Rc, pc = d.cam_xmat[cid].reshape(3, 3), d.cam_xpos[cid]
    T = np.eye(4); T[:3, :3] = Rt.T @ Rc @ np.diag([1.0, -1.0, -1.0]); T[:3, 3] = Rt.T @ (pc - pt_)   # MuJoCo looks down -z, y up
    return T


def wrist_robot_mask(iw, ih):
    """the wrist image's pixels that show the robot itself (the jaws, below the camera): fixed while the arm is stowed.
    From a segmentation render of the arm sim's model at the stow pose (an empty scene: everything but the floor is the
    robot), grown 4 px. Unmasked, the jaws were painted onto the floor along the whole route (26 Sep)"""
    import sys as _s, mujoco
    from pathlib import Path as _P
    from scipy import ndimage as _ndi
    _s.path.insert(0, str(_P(__file__).resolve().parents[1] / "arm")); import go1_arm as ga, payload_test as pt, legs
    m = ga.build(True, pt.STOW, pick=True); ga.near_clip(m); d = mujoco.MjData(m); legs.start_standing(m, d, 0.30); mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, ih, iw); r.enable_segmentation_rendering(); r.update_scene(d, "arm_wrist"); seg = r.render()[..., 0]; r.close()
    floor = [g for g in range(m.ngeom) if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE]
    robot = (seg >= 0) & ~np.isin(seg, floor)
    return _ndi.binary_dilation(robot, iterations=4)


def wrist_intrinsics(iw, ih):
    """the wrist camera's pinhole (fx, fy, cx, cy) at this image size, from its vertical field of view in the arm model;
    in OpenCV's convention (integers at pixel centres, [AKY]): a centred render's centre is half a pixel short of w / 2"""
    import sys as _s, math
    from pathlib import Path as _P
    _s.path.insert(0, str(_P(__file__).resolve().parents[1] / "arm")); import go1_arm as ga, payload_test as pt, mujoco
    m = ga.build(True, pt.STOW, pick=True); f = (ih / 2) / math.tan(math.radians(float(m.cam_fovy[m.camera("arm_wrist").id])) / 2)
    return f, f, iw / 2 - 0.5, ih / 2 - 0.5


def trimmed_mean(rgbk, valid, lo, hi):
    """each cell the mean of its seen views once the darkest lo% and the brightest (100 - hi)% of them, by brightness, are
    left out (rounded down, at least one view kept). rgbk (K, H, W, 3) uint8, valid (K, H, W); a cell nobody saw is 0.
    Taking one view whole per cell (--pct) leaves each cell's own frame's noise in it, which on dark tile reads as colour
    speckle; the mean averages that away, and the trim keeps a glint seen from only a few places out (3 Oct 2026)"""
    lum = np.where(valid, rgbk.astype(np.float32).mean(-1), np.inf)
    rank = np.argsort(np.argsort(lum, axis=0), axis=0)        # each view's brightness rank in its cell (unseen: last)
    nv = valid.sum(0)
    n_lo = np.floor(lo / 100 * nv).astype(int); n_hi = np.floor((100 - hi) / 100 * nv).astype(int)
    n_lo = np.minimum(n_lo, np.maximum(nv - 1, 0)); top = np.maximum(nv - n_hi, n_lo + 1)   # never trimmed to nothing
    keep = valid & (rank >= n_lo[None]) & (rank < top[None])
    w = keep.astype(np.float32)[..., None]
    return np.clip((rgbk.astype(np.float32) * w).sum(0) / np.maximum(w.sum(0), 1) + 0.5, 0, 255).astype(np.uint8)


def lag_from_turns(st, stream="color_image"):
    """how late a recording's frames are stamped against its odometry ([uKM] 2): the image's turning (calibrate.frame_lag)
    against the odometry's, over the night's first 1,500 frames: (lag s, correlation); 0 if the night never turned"""
    d = lag_detail_from_turns(st, stream); return (d["lag_s"] or 0.0), d["corr"]


def lag_detail_from_turns(st, stream="color_image", max_lag=0.3):
    """lag_from_turns with what the robot's lag gate needs (calibrate.frame_lag_detail; [gJq]); max_lag: the search's
    reach (s), wider for day one's measurement, whose lag is not known yet"""
    from houndkeeper import calibrate as _cal
    fr = [(o.ts, arr(o.data).mean(-1)) for _k, o in zip(range(1500), st.stream(stream))]
    yt, yw = [], []
    for o in st.stream("odom"):
        q = o.data.orientation; yt.append(o.ts); yw.append(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
    yt = np.array(yt); o_ = np.argsort(yt); yt = yt[o_]; yw = np.unwrap(np.array(yw)[o_]); u = np.r_[True, np.diff(yt) > 1e-6]
    return _cal.frame_lag_detail(fr, yt[u], yw[u], max_lag=max_lag)


LAG_FILE = _pl.Path(__file__).resolve().parents[2] / "data" / "robot" / "frame_lag.json"   # the robot's ([gJq])


def picture_lag_fields(cams, lag_file, mount_file):
    """what a picture records of the calibration it was placed with ([gJq]): lag_s is the head camera's lag, whichever
    camera ran last (the live tidy reads it as the head's), wrist_lag_s the wrist's; the lag file's and mount's hashes"""
    from houndkeeper import calibrate as _cal
    f = {}
    for cam in cams: f["lag_s" if cam["stream"] == "color_image" else "wrist_lag_s"] = np.float32(-cam["dt"])
    f["lag_sha"] = np.str_(_cal.file_sha(lag_file) if lag_file else ""); f["mount_sha"] = np.str_(_cal.file_sha(mount_file))
    return f


def clock_of(db, clock="auto", traj=None):
    """the clock a recording was stamped on ([55G]): clock 'wall' or 'robot' as given; 'auto' from the sim's path file
    (traj, or the one the runs list for db: calibrate.recording_clock), a SystemExit when that's unknown"""
    from houndkeeper import calibrate as _cal
    if clock in _cal.CLOCKS: return clock
    if clock != "auto": raise SystemExit(f"--clock must be auto or one of {_cal.CLOCKS}, not {clock!r}")
    try: return _cal.recording_clock(db=db, traj=traj)
    except ValueError as e: raise SystemExit(str(e))


def add_clock_args(ap):
    """--clock and --traj: the clock a recording was stamped on, for the robot's stored lag on that clock ([55G])"""
    ap.add_argument("--clock", default="auto", help="the clock the recording was stamped on: 'robot' (since [3Pv]; a real robot's own), 'wall' (sims before it), or 'auto' (default): from the sim's path file (--traj, or the one its run lists)")
    ap.add_argument("--traj", default=None, help="--clock auto: the sim's path file (traj.csv) for this recording; only its clock columns are read")


def frame_lags(st, dt, streams, lag_file=LAG_FILE, check=True, clock=None):
    """each camera's frame lag for a replay of a recording ([gJq], decision [gsJ]): dt 'robot' ('auto' the same): the
    robot's stored lag for that camera on the recording's clock (calibrate.camera_lags; [55G]: refused when the file
    has none for it), tonight's estimate from its turning only checking it (check); 'night': tonight's estimate stands;
    a number: that lag (s), every camera. {stream: (lag s, note)}; a frame is placed where its camera was that much
    earlier"""
    from houndkeeper import calibrate as _cal
    import json as _json
    if str(dt) not in ("robot", "auto", "night"): return {s_: (float(dt), "given") for s_ in streams}
    mode = "robot" if dt == "auto" else dt
    stored = _json.load(open(lag_file)) if mode == "robot" else None
    est = (lambda s_: lag_detail_from_turns(st, s_)) if mode == "night" or check else None
    return _cal.camera_lags(mode, streams, stored, est, clock=clock)


def main(argv=None, pose_hook=None):
    """the picture from a recording; pose_hook wraps the pose lookup (t -> world<-base) before the lidar's corrections:
    the robot never sets one (true_path.py's offline ablation does)"""
    ap = argparse.ArgumentParser(); ap.add_argument("db"); ap.add_argument("out")
    ap.add_argument("--every", type=int, default=4); ap.add_argument("--rmax", type=float, default=2.5)
    ap.add_argument("--closest", action="store_true", help="keep the single closest view instead of the 3-view median")
    ap.add_argument("--hmap", default=None, help="fused multi-night height map (.npy, 2 cm) to combine with tonight's")
    ap.add_argument("--part", default=None, choices=["1", "2"], help="only the first or second half of the night's frames (two maps under one light)")
    ap.add_argument("--camera", default="head", choices=["head", "wrist", "both"], help="the head camera, the stowed arm's wrist camera ([FHA]), or both into one stack (glints fall in different places from two heights and pitches)")
    ap.add_argument("--mount", default=None, help="the head camera's calibrated mount (.json from calibrate_mount.py)")
    ap.add_argument("--relief", default="auto", help="the floor's relief (.npy on the floor grid, floor_selfcal.py relief): 'auto' (default): the house's own (relief.npy beside --out) when painting in its frame (--corrections it names), else the plane z = 0, said; 'none': the plane; or a path")
    ap.add_argument("--dt", default="robot", help="the frames' stamp lag ([uKM] 2, [gJq]): 'robot' (default; 'auto' the same): the robot's stored lag (--lag-file), tonight's own estimate from its turning only checked against it; 'night': tonight's estimate stands; or a number: that lag (s), each frame placed where its camera was that much earlier (negative: later)")
    ap.add_argument("--lag-file", default=str(LAG_FILE), help="the robot's frame lag (.json from calibrate_lag.py)")
    ap.add_argument("--no-lag-check", action="store_true", help="--dt robot: skip tonight's estimate (the check)")
    add_clock_args(ap)
    ap.add_argument("--corrections", default=None, help="this night's pose corrections (.npz: times, Ts 3x3), e.g. from the loop closure over nights ([KkQ])")
    ap.add_argument("--house", default=None, help="the house's lidar map (.npy): frames placed against it ([KkQ]); missing: this night builds it there")
    ap.add_argument("--gain", action="store_true", help="match each frame's brightness to the picture so far (auto-exposed cameras)")
    ap.add_argument("--views", type=int, default=3, help="the closest views each cell keeps")
    ap.add_argument("--flatten", type=float, default=0.0, help="each frame's own light divided out before it paints (cm: the blur of its brightness over the floor it sees, its illumination estimate); 0: off")
    ap.add_argument("--pct", type=float, default=50.0, help="which of a cell's views, by brightness, it takes (50: the middle one; lower keeps glints and reflections out, as they only add light)")
    ap.add_argument("--combine", default="rank", choices=["rank", "trimmed"], help="how a cell's views become its colour: rank (default): one view whole, as --pct/--closest say; trimmed: the mean of its views once the darkest and brightest shares (--trim) are left out")
    ap.add_argument("--trim", type=float, nargs=2, default=[10.0, 90.0], metavar=("LO", "HI"), help="--combine trimmed: keep the views ranked LO..HI%% by brightness")
    ap.add_argument("--min-frames", type=int, default=MIN_FRAMES, help="refuse a picture placed from fewer frames ([YuI]; 0: no limit)")
    ap.add_argument("--min-floor-m2", type=float, default=MIN_FLOOR_M2, help="refuse a picture that saw less floor (m2; 0: no limit)")
    ap.add_argument("--pose-gap", type=float, default=POSE_GAP_S, help="the widest gap between two poses a frame's pose is interpolated across (s, the recording's clock)")
    a = ap.parse_args(argv); t0 = time.time(); K = a.views
    if a.combine == "trimmed" and (a.closest or a.pct != 50.0): ap.error("--combine trimmed takes no --closest or --pct")
    if not 0 <= a.trim[0] < a.trim[1] <= 100: ap.error("--trim LO HI: 0 <= LO < HI <= 100")
    st = SqliteStore(path=a.db); st.start()
    ci = next(iter(st.stream("camera_info"))).data
    from houndkeeper.lens import Rectifier
    rect = Rectifier.from_camera_info(ci); fx, fy, cx, cy = rect.K; iw, ih = ci.width, ci.height   # the head camera's lens, straightened ([kJ6])
    from houndkeeper.rec_poses import RecordedPoses
    pose = RecordedPoses(st, a.pose_gap); T_bc, T_co = pose.mount()   # world<-base at a frame's own time ([Feg])
    head_cam = dict(stream="color_image", T_bc=T_bc, T_co=T_co, K=(fx, fy, cx, cy), size=(iw, ih), rect=rect, self_mask=None)
    def wrist_cam():                                         # the stowed arm's camera: fixed on the trunk while walking
        o = next(iter(st.stream("arm_color_image")), None)   # its own image size (the head's camera_info is the head's)
        ww, wh = (arr(o.data).shape[1], arr(o.data).shape[0]) if o is not None else (iw, ih)
        return dict(stream="arm_color_image", T_bc=wrist_extrinsic(ww, wh), T_co=np.eye(4), K=wrist_intrinsics(ww, wh),
                    size=(ww, wh), rect=None, self_mask=wrist_robot_mask(ww, wh))   # a pinhole (est.)
    cams = [head_cam] if a.camera == "head" else [wrist_cam()] if a.camera == "wrist" else [head_cam, wrist_cam()]
    if a.camera == "wrist":
        T_bc, T_co, (fx, fy, cx, cy) = cams[0]["T_bc"], cams[0]["T_co"], cams[0]["K"]; rect = None; self_mask = cams[0]["self_mask"]
    else: self_mask = None
    if pose_hook: pose = pose_hook(pose)
    mode = "robot" if a.dt == "auto" else a.dt                # the camera's clock against the odometry's: the robot's ([gJq]),
    clock = clock_of(a.db, a.clock, a.traj) if mode == "robot" else None   # for the clock the recording was stamped on ([55G])
    lags = frame_lags(st, a.dt, [c["stream"] for c in cams], a.lag_file, check=not a.no_lag_check, clock=clock)
    for cam in cams:
        lag, note = lags[cam["stream"]]; cam["dt"] = -lag
        print(f"{cam['stream']}: frame lag {lag:.3f} s, {note}: poses looked up that much earlier", flush=True)
    lag_fields = picture_lag_fields(cams, a.lag_file if mode == "robot" else None, a.mount)
    mount, mount_off = np.eye(3), np.zeros(3)
    if a.mount and a.camera != "wrist":                       # the head camera truly sits a little turned ([KkQ], calibrate.py)
        import json as _json; from houndkeeper import calibrate as _cal; import math as _m
        mount, mount_off = _cal.mount(_json.loads(open(a.mount).read()))   # its turn and its place
    for cam in cams: cam["mount"] = (mount, mount_off) if cam["stream"] == "color_image" else (np.eye(3), np.zeros(3))
    if a.house or a.corrections:                              # the lidar places each frame in the house's frame ([KkQ])
        from houndkeeper.localize import Localizer, scans_from
        from pathlib import Path as _P
        if a.corrections:
            z_ = np.load(a.corrections); loc = Localizer(times=list(z_["times"]), Ts=list(z_["Ts"]), fits=[(0.0, 0)] * len(z_["times"])); hp = _P(a.corrections)
        else:
            hp = _P(a.house); loc = Localizer(np.load(hp) if hp.exists() else None).run(scans_from(st))
            if not hp.exists(): np.save(hp, loc.house)
            np.savez(str(a.out).removesuffix(".npz") + ".corrections.npz", times=np.array(loc.times), Ts=np.array(loc.Ts))   # for tall_layer.py
        rms = np.array([f[0] for f in loc.fits[1:]]) if len(loc.fits) > 1 else np.zeros(1)
        print(f"localized {len(loc.times)} scans to the house map ({'built tonight' if not hp.exists() else hp.name}): fit rms median {np.median(rms) * 1000:.0f} mm", flush=True)
        _pose0 = pose; pose = lambda t: (lambda M: None if M is None else loc.apply(M, t))(_pose0(t))
        _scans = scans_from(st)                                  # tonight's 3D map rebuilt from the scans, each placed as corrected
        from houndkeeper.localize import level                # heights from each scan's own floor (the body tilts as it walks)
        house_pts = np.vstack([np.c_[(lambda C: p[:, :2] @ C[:2, :2].T + C[:2, 2])(loc.correction(t)), p[:, 2]] for t, p in ((t, level(p)) for t, p in _scans) if len(p)])
        house_pts = house_pts[house_pts[:, 2] < 0.60]         # as tall_layer: a table top or a chair's back is above the camera's
                                                              # rays to the floor and hides none of it (27 Sep: they speckled the picture)
    # DimOS's 3D map of the night -> height map (max z per 2 cm cell)
    pts = house_pts if (a.house or a.corrections) else list(st.stream("global_map"))[-1].data.as_numpy()
    pts = np.asarray(pts[0] if isinstance(pts, tuple) else pts)
    hw, hh = int(2 * HX / HRES), int(2 * HY / HRES)
    hj = ((pts[:, 0] + HX) / HRES).astype(int); hi = ((HY - pts[:, 1]) / HRES).astype(int)
    ok = (hj >= 0) & (hj < hw) & (hi >= 0) & (hi < hh)
    tall = ok & (pts[:, 2] > 0.06)                            # floor-level noise and low items don't block the view
    hmap = np.zeros((hh, hw), np.float32); np.maximum.at(hmap, (hi[tall], hj[tall]), pts[tall, 2].astype(np.float32))
    cnt = np.zeros((hh, hw), np.int32); np.add.at(cnt, (hi[tall], hj[tall]), 1)
    hmap[cnt < 3] = 0                                         # a lone noise point isn't an obstacle
    if a.house or a.corrections:                              # every raw scan, not DimOS's filtered map: over a night the
        alln = np.zeros((hh, hw), np.int32); np.add.at(alln, (hi[ok], hj[ok]), 1)   # floor's own returns (range sd 2 cm,
        hmap[cnt < 0.3 * alln] = 0                            # [nAH] JaG) put 3+ points above 6 cm in most cells; a thing
        tall_house = hmap > 0.10                              # that stands has most of its cell's points up (27 Sep: the
                                                              # count alone carved streaks through the baseline pictures)
    if a.hmap:                                                # plus everything the house map has confirmed
        fused = np.load(a.hmap); h2, w2 = min(fused.shape[0], hh), min(fused.shape[1], hw)
        hmap[:h2, :w2] = np.maximum(hmap[:h2, :w2], np.where(fused[:h2, :w2] > 0.10, fused[:h2, :w2], 0))
    hmap = ndi.maximum_filter(hmap, size=3)                  # a little conservative
    cms = list(st.stream("global_costmap")); cm = cms[-1].data
    occ = cm.grid >= 90; cres = cm.resolution; cox, coy = cm.origin.position.x, cm.origin.position.y
    rgbk = np.zeros((K, H, W, 3), np.uint8); dk = np.full((K, H, W), np.inf, np.float32)
    gx = -HX + (np.arange(W) + .5) * RES; gy = HY - (np.arange(H) + .5) * RES
    from houndkeeper import floor_relief as _frl                # the floor where it stands ([yFg]): the house's relief when
    rl = getattr(a, "relief", "auto"); run_dir = _pl.Path(a.out).resolve().parent   # painting in its frame, else the plane
    relief, rpath, rnote = (_frl.resolve(rl, run_dir, a.corrections) if a.corrections else _frl.resolve_odometry(rl, run_dir))
    print(f"floor: {rnote}", flush=True)
    lag_fields["relief_sha"] = np.str_(_frl._sha(rpath) if rpath else "")   # the calibration the picture was painted with ([Q9k])
    used = 0; raw_lv = gain_lv = 0.0                         # the painted pixels' level as the camera gave it and as matched
    for cam in cams:                                         # [both]: every view of either camera in the one stack, each
      stream = cam["stream"]; T_bc, T_co = cam["T_bc"], cam["T_co"]; fx, fy, cx, cy = cam["K"]; iw, ih = cam["size"]   # cell
      rect, self_mask, (mount, mount_off), dt = cam["rect"], cam["self_mask"], cam["mount"], cam["dt"]   # keeping its nearest
      nfr = sum(1 for _ in st.stream(stream)) if a.part else 0                                   # (both
      for k, o in enumerate(st.stream(stream)):                                                  # cameras are valid sources)
          if k % a.every: continue
          if a.part and ((k < nfr // 2) != (a.part == "1")): continue
          Twb = pose(o.ts + dt)
          if Twb is None: continue
          T = Twb @ T_bc @ T_co; R, C = T[:3, :3] @ mount, T[:3, 3] + Twb[:3, :3] @ mount_off   # the head camera's calibrated mount ([KkQ])
          c0, c1 = np.searchsorted(gx, C[0] - a.rmax), np.searchsorted(gx, C[0] + a.rmax)
          r0, r1 = np.searchsorted(-gy, -(C[1] + a.rmax)), np.searchsorted(-gy, -(C[1] - a.rmax))
          if c1 <= c0 or r1 <= r0: continue
          X, Y = np.meshgrid(gx[c0:c1], gy[r0:r1])
          Zf = relief[r0:r1, c0:c1] if relief is not None else 0.0      # the floor where it stands ([4t7]; default the plane)
          P = np.stack([X - C[0], Y - C[1], np.full_like(X, -C[2]) + Zf], -1); pc = P @ R; z = pc[..., 2]; okm = z > 0.1
          u = fx * pc[..., 0] / np.where(okm, z, 1) + cx; v = fy * pc[..., 1] / np.where(okm, z, 1) + cy
          d = np.hypot(P[..., 0], P[..., 1])
          okm &= (u >= 0) & (u < iw - 1) & (v >= 0) & (v < ih - 1) & (d < a.rmax) & (d < dk[K - 1, r0:r1, c0:c1])
          if self_mask is not None:                            # not the robot's own jaws
              okm &= ~self_mask[np.clip(np.rint(v).astype(int), 0, ih - 1), np.clip(np.rint(u).astype(int), 0, iw - 1)]
          if not okm.any(): continue
          rr, cc = np.nonzero(okm); tx, ty, dd = X[rr, cc], Y[rr, cc], d[rr, cc]; vis = np.ones(len(rr), bool)
          for f in np.linspace(0.03, 0.97, 24):                # line of sight vs the height map
              sx, sy = tx + (C[0] - tx) * f, ty + (C[1] - ty) * f; zr = C[2] * f
              near = dd * f < 0.04                              # the cell itself (lets short items paint in place)
              hi_ = ((HY - sy) / HRES).astype(int); hj_ = ((sx + HX) / HRES).astype(int)
              inb = (hi_ >= 0) & (hi_ < hh) & (hj_ >= 0) & (hj_ < hw)
              hgt = np.zeros(len(rr), np.float32); hgt[inb] = hmap[hi_[inb], hj_[inb]]
              vis &= ~((hgt > zr + 0.01) & ~near)
          rr, cc, dd = rr[vis], cc[vis], dd[vis]
          img = arr(o.data) if rect is None else rect(arr(o.data)); slant = np.hypot(dd, C[2])               # each cell the mean of the pixels it covers ([FHA]:
          px = np.clip(floor_sample.sample(floor_sample.pyramid(img), u[rr, cc], v[rr, cc],   # one nearest pixel aliased and
                       floor_sample.px_per_cell(RES, slant, fy, C[2])), 0, 255).astype(np.uint8)   # sat half a pixel off)
          R_, C_ = r0 + rr, c0 + cc
          if a.flatten > 0:                                    # this frame's light, from this frame alone: its brightness over
              hh_, ww_ = r1 - r0, c1 - c0                      # the floor it sees, blurred far wider than the floor's pattern,
              L_ = np.zeros((hh_, ww_), np.float32); M_ = np.zeros((hh_, ww_), np.float32)   # is the light falling on it; the
              L_[rr, cc] = px.astype(np.float32).mean(-1); M_[rr, cc] = 1.0                  # floor's own colour is what's left
              sg = a.flatten / (RES * 100); den = np.maximum(ndi.gaussian_filter(M_, sg), 1e-3)
              illum = ndi.gaussian_filter(L_, sg) / den
              lv = float(np.median(L_[rr, cc])) or 1.0
              px = np.clip(px.astype(np.float32) * (lv / np.maximum(illum[rr, cc], 1.0))[:, None], 0, 255).astype(np.uint8)
          if a.gain:                                           # the camera exposes each frame on its own (auto-exposure): scale
              seen = np.isfinite(dk[0, R_, C_])                # this frame to match what is already painted where they overlap,
              if seen.sum() > 200:                             # so the picture has no seams where its source frames change
                  ref = rgbk[0, R_[seen], C_[seen]].astype(np.float32).mean(-1); mine = px[seen].astype(np.float32).mean(-1)
                  g = float(np.clip(np.median(ref) / max(np.median(mine), 1.0), 0.25, 4.0))
                  raw_lv += float(px.mean()) * len(px)
                  px = np.clip(px.astype(np.float32) * g, 0, 255).astype(np.uint8); gain_lv += float(px.mean()) * len(px)
          for kk in range(K):                                  # insert into the K closest views
              better = dd < dk[kk, R_, C_]
              if not better.any(): continue
              Rb, Cb = R_[better], C_[better]
              for j in range(K - 1, kk, -1):
                  dk[j, Rb, Cb] = dk[j - 1, Rb, Cb]; rgbk[j, Rb, Cb] = rgbk[j - 1, Rb, Cb]
              dk[kk, Rb, Cb] = dd[better]; rgbk[kk, Rb, Cb] = px[better]
              keep = ~better; R_, C_, dd, px = R_[keep], C_[keep], dd[keep], px[keep]
              if not len(dd): break
          used += 1
    st.stop()
    if used == 0:                                            # a recording with no usable frames is a broken night, not an
        raise SystemExit(f"{a.db}: no {a.camera} frames placed (no frames recorded, or none with a pose): not a floor picture")   # empty floor
    why = baseline_refusal(used, float(np.isfinite(dk[0]).sum()) * RES * RES, a.min_frames, a.min_floor_m2)
    if why: raise SystemExit(f"{a.db}: {why}: not a baseline")   # a sliver judged as the floor ([YuI]; 2 Oct: 1 frame, 2%)
    valid = np.isfinite(dk)
    stack = np.where(valid[..., None], rgbk.astype(np.float32), np.nan)
    if a.combine == "trimmed": rgb = trimmed_mean(rgbk, valid, *a.trim)
    elif a.closest: rgb = rgbk[0]
    elif a.pct == 50.0: rgb = np.nan_to_num(np.nanmedian(stack, axis=0)).astype(np.uint8)
    else:                                                    # the view at that rank of brightness, whole (its colour kept)
        lum = np.where(valid, rgbk.astype(np.float32).mean(-1), np.inf); order = np.argsort(lum, axis=0)
        nv = valid.sum(0); pick = np.clip(np.floor(a.pct / 100 * (nv - 1) + 0.5).astype(int), 0, K - 1)
        idx = np.take_along_axis(order, pick[None], 0)[0]
        rgb = np.take_along_axis(rgbk, idx[None, ..., None].repeat(3, -1), 0)[0]
    if a.gain and gain_lv > 0:                               # matching frame to frame keeps the seams away but walks the
        rgb = np.clip(rgb.astype(np.float32) * (raw_lv / gain_lv), 0, 255).astype(np.uint8)   # level off (27 Sep: to half the
                                                             # camera's, where 8-bit steps and noise swamp the floor's colour)
    Xg, Yg = np.meshgrid(gx, gy); gi = ((Yg - coy) / cres).astype(int); gj = ((Xg - cox) / cres).astype(int)
    inb = (gi >= 0) & (gi < occ.shape[0]) & (gj >= 0) & (gj < occ.shape[1])
    occ1 = np.zeros((H, W), bool); occ1[inb] = occ[gi[inb], gj[inb]]
    if a.house or a.corrections:                              # the costmap is in the night's odometry frame; the house
        ti = np.clip(((HY - Yg) / HRES).astype(int), 0, hh - 1); tj = np.clip(((Xg + HX) / HRES).astype(int), 0, hw - 1)
        occ1 = ndi.binary_dilation(tall_house, iterations=2)[ti, tj]   # frame's structures are its own lidar's (+4 cm, the
                                                              # costmap's lethal margin, est.)
    np.savez_compressed(a.out, rgb=rgb, dist=dk[0].astype(np.float16), occ=occ1, **lag_fields)   # the lag, for the live
                                                             # confirmer (tidy_module), which judges frames as they arrive
    print(f"frames used {used}; floor observed {np.isfinite(dk[0]).mean()*100:.0f}%; 3 views for {valid.all(0).sum()/max(valid[0].sum(),1)*100:.0f}% of it; {time.time()-t0:.0f}s")


if __name__ == "__main__":
    main()
