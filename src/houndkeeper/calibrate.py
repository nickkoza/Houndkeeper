"""The head camera's true mount, from its own frames ([KkQ]; the imperfection it answers is [nAH] 55b): a camera
looking almost level sees floor 2-2.5 m out only ~7 deg below the horizon, so a mount off by 0.6 deg moves those floor
points ~20 cm, while points ~1 m out barely move (the error grows with the square of the range). So the floor as the
near views see it is the reference, and the mount correction is the small turn (pitch, yaw, roll) of the camera that
lays the far views' floor onto it. No target, no measurement: any recording of the robot walking its house.

A frame is (image straightened by lens.head, camera pose world <- optical 4x4, (fx, fy, cx, cy)). Colours are compared
as gradients (edges and texture), so lighting and exposure don't matter."""
import math

import numpy as np

RES = {"range": 0.04, "heading": 0.02}   # the comparison grid (m): the far views resolve no finer than 4 cm along their
                                    # range (at 1 cm, noise; at 4 cm a clean pitch peak); yaw shifts views ~2 cm, so 2 cm (27 Sep)
NEAR, FAR = (0.55, 1.2), (1.6, 2.6)  # ranges from the camera's foot (m)


def _rot(pitch, yaw, roll):
    """a small turn of the camera in its own optical frame (x right, y down, z forward): pitch about x (down +),
    yaw about y (right +), roll about z"""
    cp, sp, cy_, sy, cr, sr = math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll)
    Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]]); Ry = np.array([[cy_, 0, sy], [0, 1, 0], [-sy, 0, cy_]])
    Rz = np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return Rx @ Ry @ Rz


def mount(mj):
    """a calibrated mount (head_mount.json's dict, or None): (its turn, as _rot; how far the camera truly sits from where
    the robot's model puts it, in the body's frame: forward, left, up, m). The place is tuned as a constant (1 Oct 2026:
    the camera saw every wall meet the floor ~2 cm short of the lidar's, the same at every range and bearing and standing
    still; a static error is treated as tuning)"""
    if not mj: return np.eye(3), np.zeros(3)
    R = _rot(*(math.radians(mj.get(k, 0.0)) for k in ("pitch_deg", "yaw_deg", "roll_deg")))
    return R, np.array([mj.get("fwd_m", 0.0), mj.get("left_m", 0.0), mj.get("up_m", 0.0)])


def camera_place(C, Twb, off):
    """the camera's position C (world) moved by the mount's place offset (body frame), the body posed by Twb"""
    return np.asarray(C, float) + Twb[:3, :3] @ off


def _project(img, T, K, corr, rng, step=2, res=0.04):
    """floor cells (i, j) and grey values of one frame's pixels whose ground point lies in `rng` metres from the
    camera's foot, with the camera turned by `corr` (3x3, optical frame)"""
    fx, fy, cx, cy = K; H, W = img.shape[:2]
    v, u = np.mgrid[H // 3:H:step, 0:W:step]                              # the lower two thirds: the floor
    d = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1).reshape(-1, 3)
    R = T[:3, :3] @ corr; C = T[:3, 3]; w = d @ R.T
    ok = w[:, 2] < -1e-3; s = -C[2] / np.where(ok, w[:, 2], -1.0); P = C + w * s[:, None]
    r = np.hypot(P[:, 0] - C[0], P[:, 1] - C[1]); ok &= (r >= rng[0]) & (r < rng[1])
    g = img.astype(np.float32).mean(-1)[v.ravel(), u.ravel()]
    return np.floor(P[ok, 0] / res).astype(np.int64), np.floor(P[ok, 1] / res).astype(np.int64), g[ok]




def _dense(cells, box):
    """mean grey per floor cell on a dense grid over box (i0, j0, ni, nj), NaN where unseen"""
    i, j, g = cells; i0, j0, ni, nj = box; ii, jj = i - i0, j - j0
    ok = (ii >= 0) & (ii < ni) & (jj >= 0) & (jj < nj); k = ii[ok] * nj + jj[ok]
    s = np.bincount(k, g[ok], ni * nj); n = np.bincount(k, None, ni * nj)
    return np.where(n > 0, s / np.maximum(n, 1), np.nan).reshape(ni, nj)


def _detail(m, res):
    """each cell's value less its 20 cm neighbourhood's mean (lighting and exposure removed; NaN where unseen)"""
    from scipy import ndimage as ndi
    seen = np.isfinite(m); f = np.where(seen, m, 0.0); w = seen.astype(float); s_ = 0.2 / res
    return np.where(seen, m - ndi.gaussian_filter(f, s_) / np.maximum(ndi.gaussian_filter(w, s_), 1e-6), np.nan)


def _heading(T):
    f = T[:3, 2]; return math.atan2(f[1], f[0])                              # the optical axis (z), seen from above


def agreement(frames, corr, by="range") -> float:
    """how well two sets of views of the same floor agree once the camera is turned by corr (correlation of detail,
    over shared cells). by="range": the far views against the near ones (pitch moves far points most); by="heading":
    views looking one way against views looking the other (a yaw or roll error shifts them in opposite directions)"""
    res = RES[by]
    if by == "range":
        near = [_project(im, T, K, corr, NEAR, res=res) for im, T, K in frames]; far = [_project(im, T, K, corr, FAR, res=res) for im, T, K in frames]
    else:
        hs = np.array([_heading(T) for _, T, _ in frames]); ref = math.atan2(np.sin(2 * hs).mean(), np.cos(2 * hs).mean()) / 2
        one = np.cos(hs - ref) >= 0                                           # the lanes' two directions
        both = (NEAR[0], FAR[1])
        near = [_project(im, T, K, corr, both, res=res) for (im, T, K), o in zip(frames, one) if o]
        far = [_project(im, T, K, corr, both, res=res) for (im, T, K), o in zip(frames, one) if not o]
        if not near or not far: return -1.0
    ci = np.concatenate([c[0] for c in near]); cj = np.concatenate([c[1] for c in near])
    box = (int(ci.min()) - 2, int(cj.min()) - 2, int(ci.max() - ci.min()) + 5, int(cj.max() - cj.min()) + 5)
    cat = lambda cs: tuple(np.concatenate([c[k] for c in cs]) for k in range(3))
    a, b = _detail(_dense(cat(near), box), res), _detail(_dense(cat(far), box), res)
    ok = np.isfinite(a) & np.isfinite(b)
    if ok.sum() < 300: return -1.0
    x, y = a[ok] - a[ok].mean(), b[ok] - b[ok].mean()
    return float((x @ y) / (np.linalg.norm(x) * np.linalg.norm(y) + 1e-9))


def mount_correction(frames, span_deg=1.5, step_deg=0.25, fine_deg=0.05, rounds=2):
    """(pitch, yaw, roll) in degrees and each axis's agreement: pitch from the far views against the near ones, yaw and
    roll from the views looking one way against those looking the other; one axis at a time (coarse then fine), twice
    round (pitch dominates: it moves far points most)"""
    best = [0.0, 0.0, 0.0]; scores = [agreement(frames, np.eye(3), "range"), *[agreement(frames, np.eye(3), "heading")] * 2]
    for _ in range(rounds):
        for ax in (0, 1, 2):
            for st, span in ((step_deg, span_deg if ax < 2 else 1.0), (fine_deg, step_deg)):
                c0 = best[ax]; scores[ax] = agreement(frames, _rot(*(math.radians(t) for t in best)), "range" if ax == 0 else "heading")
                vs = np.arange(-span, span + 1e-9, st) + c0; scs = []
                for v in vs:
                    trial = list(best); trial[ax] = float(v)
                    scs.append(agreement(frames, _rot(*(math.radians(t) for t in trial)), "range" if ax == 0 else "heading"))
                k = int(np.argmax(scs))
                if scs[k] > scores[ax]:
                    v = vs[k]
                    if 0 < k < len(vs) - 1:                                   # the peak between samples: a parabola through three
                        a_, b_, c_ = scs[k - 1], scs[k], scs[k + 1]; den = a_ - 2 * b_ + c_
                        if den < 0: v = v + st * 0.5 * (a_ - c_) / den
                    best[ax] = float(v); scores[ax] = scs[k]
    return tuple(best), tuple(scores)


def frame_lag(frames, yaw_times, yaw, max_lag=0.3, step=0.005):
    """how late the camera's frames are stamped against the odometry (s), from the robot's own turning ([uKM] 2, as
    camera-IMU temporal calibration does): each pair of consecutive frames' sideways image shift, from phase correlation
    of their upper halves (walls, furniture: far, so a turn moves them, walking hardly does), gives the image's turn
    rate; the lag that best lines it up with the odometry's turn rate is the frames' lag. frames: [(t stamp, grey
    image)] in order; yaw_times, yaw: the odometry's heading (unwrapped, rad) over time. Returns (lag s, correlation)"""
    d = frame_lag_detail(frames, yaw_times, yaw, max_lag, step)
    return d["lag_s"], d["corr"]


TURN_RATE = 0.1   # rad/s: a frame pair taken while the odometry turned at least this fast is a turning pair


def frame_lag_detail(frames, yaw_times, yaw, max_lag=0.3, step=0.005):
    """frame_lag with what the gate needs ([gJq], decision [gsJ]): {lag_s (None under 50 pairs), corr, pairs (frame
    pairs measured), turning (of them, taken while the robot turned), edge (the best lag on the search's edge),
    max_lag}"""
    import cv2
    ts, rates = [], []
    for (t0, a), (t1, b) in zip(frames[:-1], frames[1:]):
        dt = t1 - t0
        if not 0.02 < dt < 0.3: continue
        h = a.shape[0] // 2; win = cv2.createHanningWindow((a.shape[1], h), cv2.CV_32F)
        (dx, _), resp = cv2.phaseCorrelate(a[:h].astype(np.float32), b[:h].astype(np.float32), win)
        if resp < 0.05: continue
        ts.append((t0 + t1) / 2); rates.append(dx / dt)                         # px/s: turning left, the scene slides right
    ts, rates = np.array(ts), np.array(rates)
    out = dict(lag_s=None, corr=0.0, pairs=int(len(ts)), turning=0, edge=False, max_lag=float(max_lag))
    if len(ts) < 50: return out
    yr = np.gradient(yaw, yaw_times)
    out["turning"] = int((np.abs(np.interp(ts, yaw_times, yr)) >= TURN_RATE).sum())
    best = (None, -1.0)
    for lag in np.arange(-max_lag, max_lag + 1e-9, step):
        o = np.interp(ts - lag, yaw_times, yr)                                 # the odometry's turn rate when the frame was taken
        c = np.corrcoef(rates, o)[0, 1] if np.std(o) > 0 and np.std(rates) > 0 else -1.0
        if c > best[1]: best = (float(lag), float(c))
    out["lag_s"], out["corr"] = best if best[0] is not None else (None, 0.0)
    out["edge"] = best[0] is not None and abs(abs(best[0]) - max_lag) < step / 2
    return out


# The robot's camera calibration ([gJq], decision [gsJ]): the frame lag and the head mount are stored once for the robot
LAG_GATE = dict(min_corr=0.7, min_turning=150, lo=-0.1, hi=0.3)   # lo below 0: on the robot's clock the frames' lag is
                                    # near 0 and a night's estimate scatters either side of it ([gsJ])
LAG_CHECK_S = 0.040   # a night's estimate this far from the robot's is warned about
CLOCKS = ("wall", "robot")


def lag_refusal(est, gate=LAG_GATE):
    """why a night's lag estimate (frame_lag_detail's dict) is no evidence of the robot's lag, or None"""
    lag = est.get("lag_s")
    if lag is None: return f"too few frame pairs measured ({est.get('pairs', 0)})"
    if est.get("edge"): return f"on the search's edge ({lag:+.3f} s)"
    if est.get("corr", 0.0) < gate["min_corr"]: return f"correlation {est.get('corr', 0.0):.2f} under {gate['min_corr']}"
    if est.get("turning", 0) < gate["min_turning"]: return f"{est.get('turning', 0)} turning frame pairs (needs {gate['min_turning']})"
    if not gate["lo"] <= lag <= gate["hi"]: return f"{lag:+.3f} s outside {gate['lo']}-{gate['hi']} s"
    return None


def robot_lag(estimates, gate=LAG_GATE):
    """the robot's frame lag from many nights' estimates: the median of those the gate keeps. Returns (lag s or None,
    [(index, refusal or None)])"""
    why = [(i, lag_refusal(e, gate)) for i, e in enumerate(estimates)]
    kept = [estimates[i]["lag_s"] for i, r in why if r is None]
    return (float(np.median(kept)) if kept else None), why


def file_sha(path):
    """a calibration file's identity, recorded by every run that reads it (first 12 hex of its sha256; '' if missing)"""
    import hashlib, os
    return hashlib.sha256(open(path, "rb").read()).hexdigest()[:12] if path and os.path.exists(path) else ""


def lag_entry(d, clock, where="the lag file"):
    """one clock's entry of the robot's stored lag ([gJq], decision [55G]): d is frame_lag.json's dict, per clock
    ({"clocks": {clock: entry}}) or, as written before, a single entry naming its clock. The entry, with its clock; a
    ValueError when the file has no lag for that clock (never another clock's in its place)"""
    if clock not in CLOCKS: raise ValueError(f"clock must be one of {CLOCKS}, not {clock!r}")
    if "clocks" in d:
        e = (d.get("clocks") or {}).get(clock)
        if not e or e.get("lag_s") is None:
            raise ValueError(f"{where} has no frame lag for the {clock} clock (it has: {sorted(d.get('clocks') or {})})")
        return dict(e, clock=clock)
    if d.get("clock") not in CLOCKS: raise ValueError(f"{where}: clock must be one of {CLOCKS}, not {d.get('clock')!r}")
    if d["clock"] != clock: raise ValueError(f"{where} has no frame lag for the {clock} clock (only the {d['clock']} clock's)")
    return d


FLOOR_ROWS_KEYS = ("floor_rows_offset_s", "floor_rows")   # written by the floor self-calibration's step, kept by the lag's


def load_lag(path, clock):
    """the robot's stored frame lag (data/robot/frame_lag.json) for a recording stamped on `clock` ([gJq], [55G]): the
    entry's dict, with lag_s, clock and streams; a ValueError when the file has none for that clock"""
    import json
    return lag_entry(json.load(open(path)), clock, str(path))


def store_lag(path, clock, entry):
    """write one clock's entry into the robot's lag file, keeping the other clocks' ([55G]); a file in the single-clock
    form is carried into the per-clock form"""
    import json, os
    if clock not in CLOCKS: raise ValueError(f"clock must be one of {CLOCKS}, not {clock!r}")
    d = json.load(open(path)) if os.path.exists(path) else {}
    if "clocks" not in d: d = {"clocks": {d["clock"]: {k: v for k, v in d.items() if k != "clock"}}} if d.get("clock") in CLOCKS else {"clocks": {}}
    keep = {k: v for k, v in (d["clocks"].get(clock) or {}).items() if k in FLOOR_ROWS_KEYS}   # the floor rows' offset is
    d["clocks"][clock] = {**keep, **{k: v for k, v in entry.items() if k != "clock"}}            # its own step's ([yFg])
    open(path, "w").write(json.dumps(d, indent=1))
    return d


def _traj_clock(traj):
    """the clock a sim path file's recording was stamped on: since the robot's clock ([3Pv], decision [Rpj]) the file
    carries host_t, the host's own clock, beside wall_t, the clock the messages were stamped on; wall_t then follows
    sim_t (robot) unless the sim ran without DIMOS_SIM_CLOCK, when it follows host_t (wall). No host_t: wall. Reads only
    the clock columns, never the true_* ones"""
    import csv
    with open(traj) as fh:
        rows = csv.reader(fh); head = next(rows, [])
        if "host_t" not in head: return "wall"
        iw, isim, ih = head.index("wall_t"), head.index("sim_t"), head.index("host_t")
        w, s, h = [], [], []
        for k, r in enumerate(rows):
            if k >= 2000: break
            try: w.append(float(r[iw])); s.append(float(r[isim])); h.append(float(r[ih]))
            except (IndexError, ValueError): continue
    if len(w) < 2: raise ValueError(f"{traj}: too few rows to tell its clock")
    w, s, h = np.array(w), np.array(s), np.array(h)
    return "robot" if np.ptp(w - s) <= np.ptp(w - h) else "wall"


def _traj_of(db, roots):
    """a recording's path file, where the runs list their recordings: a sweep run's seed_recordings.txt (night n ->
    <run>/<run>_n<n>.traj.csv) or a tidy run's recording.txt (<run>/traj.csv); None when no run lists it"""
    import os
    from pathlib import Path
    db = os.path.realpath(str(db))
    for root in roots:
        root = Path(root)
        if not root.is_dir(): continue
        for lst in list(root.glob("*/seed_recordings.txt")) + list(root.glob("*/recording.txt")):
            try: lines = lst.read_text().splitlines()
            except OSError: continue
            for line in lines:
                parts = line.split(maxsplit=1)
                if not parts: continue
                n, p = (parts[0], parts[1]) if len(parts) == 2 else (None, parts[0])
                if os.path.realpath(p.strip()) != db: continue
                t = lst.parent / (f"{lst.parent.name}_n{n}.traj.csv" if n is not None else "traj.csv")
                if t.exists(): return t
    return None


def recording_clock(db=None, traj=None, roots=None):
    """the clock a recording was stamped on, 'wall' or 'robot' ([gJq], decision [55G]), from the sim's path file (traj,
    or the one the runs list for db). A recording no run lists, a real robot's among them, is a ValueError: its clock
    is given, never guessed"""
    from pathlib import Path
    if traj is None and db is not None:
        hk = Path(__file__).resolve().parents[2]
        traj = _traj_of(db, roots if roots is not None else (hk / "sim" / "house" / "runs", hk / "data" / "runs" / "dimos_tidy"))
    if traj is None:
        raise ValueError(f"the clock {db or 'this recording'} was stamped on is unknown (no sim path file lists it): "
                         f"give it (--clock robot for a real robot's recording, whose own clock is the wall clock)")
    return _traj_clock(traj)


def check_lag(stored, est, tol=LAG_CHECK_S, gate=LAG_GATE):
    """a night's own estimate held against the robot's stored lag: a warning to print, or None when they agree"""
    r = lag_refusal(est, gate)
    if r: return f"tonight's lag estimate is no check ({r}); the robot's {stored:.3f} s kept"
    if abs(est["lag_s"] - stored) > tol:
        return (f"tonight's lag estimate {est['lag_s']:.3f} s differs from the robot's {stored:.3f} s by "
                f"{abs(est['lag_s'] - stored) * 1000:.0f} ms (over {tol * 1000:.0f}); the robot's kept")
    return None


def camera_lags(mode, streams, stored, estimate, tol=LAG_CHECK_S, clock=None):
    """each camera's frame lag for a floor picture ([gJq]): mode 'robot' (the robot's stored lag for the recording's
    clock ([55G]), stored: the lag file's dict, refused when it has none for that clock; tonight's estimate only checks
    it), 'night' (tonight's estimate stands, as asked), or a number: the lag itself (s), each frame placed that
    much earlier, as frame_lags takes it. estimate(stream) -> frame_lag_detail's
    dict. Returns {stream: (lag s, note)}"""
    out = {}
    if mode == "robot":
        if clock is None: raise ValueError("the robot's stored lag needs the recording's clock")
        stored = lag_entry(stored, clock)
    for s in streams:
        if mode == "night":
            e = estimate(s); out[s] = (e["lag_s"] or 0.0, f"tonight's estimate (correlation {e['corr']:.2f}), as asked"); continue
        if mode != "robot":                                   # a number is the lag itself, as every tool's --dt takes it:
            out[s] = (float(mode), "given"); continue          # each frame placed that much earlier
        per = (stored.get("streams") or {}).get(s) or {}
        lag = float(per.get("lag_s", stored["lag_s"])); src = "the robot's" + ("" if "lag_s" in per else " head camera's")
        w = check_lag(lag, estimate(s), tol) if estimate else None   # tonight's turning sees the frame's upper half: checked
        fl = floor_lag(stored, s)                                    # against the frame's lag, the floor laid by its rows' own
        rows = f", floor rows {fl:.4f} s ({stored['floor_rows_offset_s'] * 1000:+.1f} ms)" if fl != lag else ""
        said = f"; check: {w}" if w else ("; tonight's estimate agrees" if estimate else "; not checked tonight")
        out[s] = (fl, f"{src} stored lag ({stored['clock']} clock){rows}{said}")
    return out


# The floor rows' own lag ([yFg]): a rolling-shutter camera reads the floor (the frame's lower part) last, so the floor
# is placed with the stored lag less the floor self-calibration's offset; the stored lag itself stays the frame's (its
# upper half, as the turning estimator measures it, and as the per-night check compares)
FLOOR_STREAMS = ("color_image",)   # the head camera's: the only one whose floor rows were calibrated


def floor_lag(entry, stream="color_image"):
    """the lag a camera's floor pixels are placed with ([yFg]): one clock's entry of the lag file (lag_entry's dict),
    the stream's stored lag plus the clock's floor_rows_offset_s for the head camera (0 where none is stored)"""
    per = (entry.get("streams") or {}).get(stream) or {}
    lag = float(per.get("lag_s", entry["lag_s"]))
    return lag + float(entry.get("floor_rows_offset_s", 0.0) or 0.0) if stream in FLOOR_STREAMS else lag


# What a floor product was painted with ([yFg]): the mount, the lag file and the relief, each by its hash, kept in the
# product so one made with another calibration is never compared with tonight's frames
STAMP_KEYS = ("mount_sha", "lag_sha", "relief_sha")


def stamp(mount_file=None, lag_file=None, relief_file=None) -> dict:
    """the calibration a product is painted with: each file's hash ('' for none: no mount, a given lag, the plane)"""
    return {"mount_sha": file_sha(mount_file), "lag_sha": file_sha(lag_file), "relief_sha": file_sha(relief_file)}


def stamp_of(path) -> dict | None:
    """a saved product's calibration stamp (a picture's or views' .npz), or None when it was made before the stamps
    carried the relief (it cannot be checked)"""
    import os
    if not os.path.exists(path): return None
    z = np.load(path)
    if not all(k in z.files for k in STAMP_KEYS): return None
    return {k: str(z[k]) for k in STAMP_KEYS}


def stale(product, expected, name="the product") -> list:
    """why a product's stamp (stamp_of) is not the calibration expected (stamp), [] when it is"""
    if product is None: return [f"{name} carries no calibration stamp (made before [yFg]'s stamps)"]
    what = {"mount_sha": "mount", "lag_sha": "lag file", "relief_sha": "relief"}
    return [f"{name} was painted with another {what[k]} ({product[k] or 'none'}, not {expected[k] or 'none'})"
            for k in STAMP_KEYS if product.get(k, "") != expected.get(k, "")]


# The house's map version ([dmT], decision [fxk]): the first word of the stitch's stamp (stitch_baselines.sh's
# MAP_VERSION, its run's stitch.args). v9: every loop closure verified before it entered the house's frame (11 Oct
# 2026). A map made before is refused, so tonight is never compared with a frame that unverified loops may have bent
MAP_VERSION = "v9"


def stale_map(run_dir) -> list:
    """why a run's house map was made by an older stitch than MAP_VERSION ([dmT]), [] when it is current or the run has
    no house map yet (no shape solve or floor frame: a house before its second baseline night)"""
    import os
    if not any(os.path.exists(os.path.join(str(run_dir), n)) for n in ("shape", "floor_frame.json")): return []
    p = os.path.join(str(run_dir), "stitch.args")
    if not os.path.exists(p): return [f"the house's map carries no stitch stamp (made again by stitch_baselines.sh {MAP_VERSION})"]
    words = open(p).read().split()
    have = words[0] if words else "none"
    if have != MAP_VERSION:
        return [f"the house's map was made by stitch {have}, before its loop closures were verified (stitch {MAP_VERSION})"]
    return []


def stale_products(run_dir, expected, names=("mosaic_n1.npz", "mosaic_n2.npz", "views_n1.npz", "views_n2.npz")) -> list:
    """every reason a run's baseline products were painted with a calibration other than `expected` (stamp); a
    product the run lacks is no reason"""
    import os
    out = []
    for n in names:
        p = os.path.join(str(run_dir), n)
        if os.path.exists(p): out += stale(stamp_of(p), expected, n)
    return out
