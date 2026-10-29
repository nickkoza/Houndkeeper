#!/usr/bin/env python3
"""Measurements from a recording ([4hm]): what the robot's own recording can tell about the parameters the sims
assume, so each becomes `source: measured` in params/robot.yaml. Each command reads one DimOS recording (memory.db)
and prints what it found; with --write it updates the parameter (value, source measured, the date and how).

  stream  <db>                         frames that never arrived: single drops, stalls (go2_air.video_stream)
  lag     <db>                         the camera frames' stamps against odometry, from the robot's turning
                                       (go2_air.frame_delay_s, relative to odometry: see timing.md)
  lidar   <db> [--box x0 x1 y0 y1 z0 z1]
                                       points a second as DimOS receives them; with --box, a flat surface's range
                                       spread (go2_air.lidar.range_sd) and its returns per scan (dropouts.md)
  odom    <db> --truth T               odometry against the true path: per-night scale error, drift per root metre
                                       (go2_air.odom_drift). T: the sim's traj.csv, or a taped walk (odometry.md)
  noise   <db>                         the head camera's frame-to-frame noise while the robot stands still, against
                                       brightness (go2_air.camera_imperfections shot_noise, read_noise, effective)
  power   <db>                         the battery stream (houndkeeper.battery) against odometry: mean power walking
                                       and standing, checked against the charge used (go2_air.walk_power_W,
                                       stand_power_W; power_and_pose.md 1)
  compression <db> [--sim <sim db>] [--sheet out.png]
                                       how flattened and blocky the dark floor is, standing and walking; with --sim,
                                       beside the sim's frames re-encoded at q50 and q30, and a side-by-side picture
                                       (camera.md 3, go2_air.camera_imperfections jpeg_*; day one's band, [GWP])
  mount   <check.json>...              the head camera's mount night to night: each night's nightly check
                                       (floor_selfcal.py check) side by side, flagged past the band ([GWP])
  (lag --expect S, odom and noise --sim also say pass, outside or fails against day one's bands: DAY_ONE_BANDS)
  gap    <real db> [--sim <sim db> | --reference R] [--box ...] [--truth T] [--out report.md]
                                       the gap report ([KbT]): every analysis above that applies on the real recording,
                                       beside the sim's value and the file's, and which parameter to tune or write
  simref  <sim run dir>...             the sim's side ([esI]): the analyses on existing sim nights, written to
                                       real/calibration/sim_reference.json (the gap report's default sim values)

Two kinds of answer. Some are the parameter itself, read straight off the real recording (the stream's drops and
stalls, a wall's range spread, odometry against a taped walk): --write puts those in the file. The others are what the
parameter does once the camera's own processing and DimOS have had it (the frames' effective noise, the points DimOS
passes on): those are measured the same way on a sim night and a real night (--sim), and the sim's parameter is tuned
until the two agree (sim-to-real-to-sim, [V3o] [7Xn]). The analyses are checked on known inputs (tests/test_measure.py)
and on a sim night against the sim's own settings (README.md).
Usage: measure.py <command> <memory.db> [--sim <sim memory.db>] [options] [--write]   (dimos venv)"""
import argparse, json, math, re, sys, time
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[2]
PARAMS = HK / "params" / "robot.yaml"
STALL_S = 0.3                     # a gap this long is a stall, not a dropped frame or two


def store(db):
    from dimos.memory.store.sqlite import SqliteStore
    st = SqliteStore(path=str(db)); st.start(); return st


def stamps(st, stream):
    return np.sort(np.array([o.ts for o in st.stream(stream)], float))


def frame_gaps(ts, stall_s=STALL_S):
    """from frame arrival times: the nominal period, the share of frames dropped singly (in gaps under stall_s), and
    the stalls (gaps of stall_s or more) a minute and their mean length"""
    d = np.diff(ts); period = float(np.median(d)); span = ts[-1] - ts[0]
    short = d[d < stall_s]; missed = np.clip(np.round(short / period) - 1, 0, None).sum()
    stalls = d[d >= stall_s]
    expected = span / period
    return dict(period_s=round(period, 4), drop=round(float(missed / expected), 4),
                stall_per_min=round(len(stalls) / (span / 60), 2), stall_s=round(float(stalls.mean()), 2) if len(stalls) else 0.0,
                frames=len(ts), minutes=round(span / 60, 2))


def odom_track(st):
    t, x, y, yaw = [], [], [], []
    for o in st.stream("odom"):
        d = o.data; q = d.orientation; t.append(o.ts); x.append(d.position.x); y.append(d.position.y)
        yaw.append(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
    o_ = np.argsort(t); t = np.array(t)[o_]; u = np.r_[True, np.diff(t) > 1e-6]
    return t[u], np.array(x)[o_][u], np.array(y)[o_][u], np.unwrap(np.array(yaw)[o_][u])


def odom_vs_truth(t, x, y, yaw, T):
    """odometry against the true path (both world-framed, the odometry's start aligned to the truth's): the per-night
    scale error (odometry's distance over the true one, less 1), the end-point drift after aligning the start, and that
    drift per root metre walked (the sim's model: position noise ~ sd per sqrt(m))"""
    tx, ty, tyaw = (np.interp(t, T["wall_t"], T[k]) for k in ("true_x", "true_y", "true_yaw"))
    tyaw = np.interp(t, T["wall_t"], np.unwrap(T["true_yaw"]))
    step = lambda a, b: np.hypot(np.diff(a), np.diff(b))
    moving = step(tx, ty) > 2e-4
    d_true = step(tx, ty)[moving].sum(); d_odom = step(x, y)[moving].sum()
    th = tyaw[0] - yaw[0]; c, s = math.cos(th), math.sin(th)                # the odometry's start laid on the truth's
    ox = tx[0] + c * (x - x[0]) - s * (y - y[0]); oy = ty[0] + s * (x - x[0]) + c * (y - y[0])
    err = np.hypot(ox - tx, oy - ty); dyaw = np.degrees((yaw + th) - tyaw)
    walked = np.r_[0, np.cumsum(step(tx, ty))]
    k = walked > 1.0
    return dict(walked_m=round(float(d_true), 2), scale=round(float(d_odom / d_true - 1), 4),
                end_m=round(float(err[-1]), 3), end_yaw_deg=round(float(dyaw[-1] - dyaw[0]), 2),
                xy_per_root_m=round(float(np.median(err[k] / np.sqrt(walked[k]))), 4) if k.any() else None,
                yaw_deg_per_root_m=round(float(np.median(np.abs(dyaw[k] - dyaw[0]) / np.sqrt(walked[k]))), 3) if k.any() else None)


def lidar_rate(st):
    L = list(st.stream("lidar")); ts = np.array([o.ts for o in L]); n = np.array([len(o.data.as_numpy()[0]) for o in L])
    return dict(messages_per_s=round(float(1 / np.median(np.diff(np.sort(ts)))), 2),
                points_per_s=int(n.sum() / (ts.max() - ts.min())), points_per_message=int(np.median(n))), L


def plane_spread(L, box, t0=None, t1=None):
    """a flat surface's points inside box (world frame), each scan's own plane fit: the range spread about it (its
    residual sd, across scans) and the returns a scan puts in the box"""
    x0, x1, y0, y1, z0, z1 = box; res, cnt = [], []
    for o in L:
        if (t0 and o.ts < t0) or (t1 and o.ts > t1): continue
        p = np.asarray(o.data.as_numpy()[0], float)
        m = (p[:, 0] > x0) & (p[:, 0] < x1) & (p[:, 1] > y0) & (p[:, 1] < y1) & (p[:, 2] > z0) & (p[:, 2] < z1)
        q = p[m]; cnt.append(len(q))
        if len(q) < 30: continue
        c = q.mean(0); _, _, vt = np.linalg.svd(q - c, full_matrices=False); res.append((q - c) @ vt[2])
    if not res: return dict(returns_per_scan=float(np.median(cnt)) if cnt else 0.0, range_sd=None)
    r = np.concatenate(res)
    return dict(returns_per_scan=float(np.median(cnt)), range_sd=round(float(np.std(r)), 4), scans=len(res))


def still_spans(t, x, y, yaw, min_s=2.0):
    """the spans the robot stood still (odometry moving under 2 mm and 0.2 deg between messages) of min_s or more"""
    mv = (np.r_[0, np.hypot(np.diff(x), np.diff(y))] > 2e-3) | (np.r_[0, np.abs(np.diff(yaw))] > math.radians(0.2))
    out, s = [], None
    for i, m in enumerate(mv):
        if not m and s is None: s = i
        if (m or i == len(mv) - 1) and s is not None:
            if t[i] - t[s] >= min_s: out.append((t[s] + 0.3, t[i] - 0.3))
            s = None
    return out


def camera_noise(st, spans, max_frames=40):
    """frame-to-frame noise while standing still: per pixel, the variance of the difference of consecutive frames
    (twice the noise variance, the scene cancelling), binned by brightness; fit var = shot * level + read^2 (levels
    0..1). What survives the camera's own noise reduction and compression: the effective noise"""
    from floor_mosaic2 import arr
    lv, vv = [], []
    for t0, t1 in spans:
        fr = [arr(o.data).astype(np.float32).mean(-1) / 255 for o in st.stream("color_image") if t0 <= o.ts <= t1][:max_frames]
        for a, b in zip(fr, fr[1:]):
            d = (a - b).ravel(); m = ((a + b) / 2).ravel(); lv.append(m); vv.append(d * d / 2)
    if not lv: return None
    lv, vv = np.concatenate(lv), np.concatenate(vv)
    edges = np.linspace(0.05, 0.9, 18); L, V = [], []
    for a, b in zip(edges, edges[1:]):
        k = (lv >= a) & (lv < b)
        if k.sum() > 5000: L.append(lv[k].mean()); V.append(np.median(vv[k]) / 0.4549)   # median of chi2(1) -> mean
    if len(L) < 3: return None
    shot, read2 = np.polyfit(L, V, 1)
    return dict(shot_noise=round(float(max(shot, 0)), 6), read_noise=round(float(math.sqrt(max(read2, 0))), 5), levels=len(L))


def battery_track(st):
    """the recording's battery stream (src/houndkeeper/battery_module.py): time, charge (%), voltage (V), current (A,
    negative discharging) and power drawn (W), time-ordered"""
    B = sorted(st.stream("battery"), key=lambda o: o.ts)
    return (np.array([o.ts for o in B], float), *(np.array([getattr(o.data, k) for o in B], float) for k in ("soc", "voltage", "current", "power")))


def power_split(bt, soc, P, odom, battery_Wh, min_s=60):
    """mean power walking (odometry moving over the second around the sample: 5 cm or 5 deg) and standing (inside a
    still span, still_spans), each from the battery's samples, the moments between the two left out; and the charge's cross-check: (start % - end %) of battery_Wh over
    the hours between, against the mean power over the same samples (the two should agree within ~10%)"""
    ot, x, y, yaw = odom; still = np.zeros(len(bt), bool)
    for t0, t1 in still_spans(*odom): still |= (bt >= t0) & (bt <= t1)
    walked = np.r_[0, np.cumsum(np.hypot(np.diff(x), np.diff(y)))]; turned = np.r_[0, np.cumsum(np.abs(np.diff(yaw)))]
    over = lambda c: np.interp(bt + 0.5, ot, c) - np.interp(bt - 0.5, ot, c)
    walk = (bt - 0.5 >= ot[0]) & (bt + 0.5 <= ot[-1]) & ~still & ((over(walked) > 0.05) | (over(turned) > math.radians(5)))
    mean = lambda k: round(float(P[k].mean()), 1) if k.sum() * np.median(np.diff(bt)) >= min_s else None
    hours = (bt[-1] - bt[0]) / 3600; charge = (soc[0] - soc[-1]) / 100 * battery_Wh / hours
    drawn = float(np.sum((P[1:] + P[:-1]) / 2 * np.diff(bt)) / (bt[-1] - bt[0]))
    return dict(walk_power_W=mean(walk), walk_s=round(float(walk.sum() * np.median(np.diff(bt)))), stand_power_W=mean(still),
                stand_s=round(float(still.sum() * np.median(np.diff(bt)))), hours=round(hours, 3), soc_start=float(soc[0]), soc_end=float(soc[-1]),
                mean_power_W=round(drawn, 1), charge_power_W=round(float(charge), 1), charge_vs_mean=round(float(charge / drawn - 1), 3) if drawn else None)


def set_param(path, value, how, params=None, add_after=None):
    """write a measurement into params/robot.yaml: path 'group.key' or 'group.key.sub' (one field of a dict value);
    source becomes measured and the note says when and how. One line per parameter, as the file keeps them. A key the
    group lacks is added on its own line after add_after's (when given), its value lined up with that line's"""
    params = PARAMS if params is None else params                       # read at the call, so a redirect of PARAMS holds
    parts = path.split("."); group, key, sub = parts[0], parts[1], (parts[2] if len(parts) > 2 else None)
    lines = params.read_text().splitlines(); g = None; anchor = None
    for i, ln in enumerate(lines):
        if g is not None and add_after and re.match(rf"^\s+{add_after}:\s", ln): anchor = i
        if re.match(rf"^{group}:\s*$", ln): g = i; continue
        if g is not None and re.match(r"^\S", ln): g = None
        if g is not None and re.match(rf"^\s+{key}:\s", ln):
            import yaml
            ent = yaml.safe_load(ln.strip())[key]
            if sub: ent["value"][sub] = value
            else: ent["value"] = value
            ent["source"] = "measured"
            ent["note"] = f"measured {time.strftime('%Y-%m-%d')}: {how}" + (f" (was: {ent.get('note', '')})" if ent.get("note") else "")
            indent = re.match(r"^(\s*)", ln).group(1)
            lines[i] = f"{indent}{key}: " + json.dumps(ent, ensure_ascii=False).replace('"value"', "value").replace('"source"', "source").replace('"note"', "note")
            params.write_text("\n".join(lines) + "\n"); return True
    if anchor is None or sub: raise KeyError(path)
    a = lines[anchor]; indent = re.match(r"^(\s*)", a).group(1)
    lines.insert(anchor + 1, f"{indent}{key}:".ljust(max(a.index("{"), len(indent) + len(key) + 2)) + "{value: " + json.dumps(value) + ", source: measured, note: "
                 + json.dumps(f"measured {time.strftime('%Y-%m-%d')}: {how}", ensure_ascii=False) + "}")
    params.write_text("\n".join(lines) + "\n"); return True


# ---- day one's pass bands ([4hm], [GWP]) -------------------------------------------------------------------------------
# How far each of the five measurements the launch false alarms lean on most may sit from what the sim assumed before
# the sim's launch numbers stop holding. From the sensitivity study (docs/research/sensitivity-2026-10-13.md: six
# tuning nights replayed live, the lead 39/48 found at 11.2 false alarms a night): `ok` is the largest step that cost no
# more than the replays' own spread (about 1 a night), `fails` the smallest step that cost more. Between the two is
# untested and reads as outside: the sim is never kinder than real ([x3f]).
DAY_ONE_BANDS = {
    "mount": dict(what="head camera pitch, night against night or against its calibration (deg)", ok=0.1, fails=0.5,
                  cost="0.5 deg: 19.0 false alarms a night against 11.2 (1 deg: 20.0)",
                  ok_from="the nightly check's tolerance (floor_selfcal.CHECK_TOL, [9tQ]); no step under 0.5 deg was run"),
    "lag": dict(what="frames' lag behind odometry, off the stored lag (s)", ok=0.02, fails=0.05,
                cost="50 ms: 17.5 false alarms a night (150 ms: 19.7)",
                ok_from="measure.py's agreement for the lag (ATOL); the nightly check warns past 10 ms; no step under 50 ms was run"),
    "drift": dict(what="odometry drift over the file's estimate (ratio; heading and position per root metre)", ok=1.0, fails=2.0,
                  cost="x2: 15.8 false alarms a night (x4: 16.5)", ok_from="the estimate itself (the nominal)"),
    "compression": dict(what="the dark floor's look beside the sim's frames re-encoded at this JPEG quality (judged by eye: compression_compare)", ok=50, fails=30,
                        cost="q30 every frame: 15.3 false alarms a night (q50: 11.0, q75: 11.0)",
                        ok_from="the q50 re-encode, which cost nothing"),
    "noise": dict(what="effective noise sd, real over sim (ratio)", ok=4.0, fails=None,
                  cost="x4: 12.3 false alarms a night (+1.1, within the spread); nothing larger was run",
                  ok_from="the x4 step"),
}


def band_verdict(name, value):
    """a day-one measurement against its pass band (DAY_ONE_BANDS): 'pass', 'fails' (at or past the step the study saw
    raise false alarms) or 'outside' (past the band, short of that step: untested, so treated as outside)"""
    b = DAY_ONE_BANDS[name]; v = abs(value)
    if v <= b["ok"] + 1e-12: return "pass", f"{name}: {value:g} within {b['ok']:g} ({b['ok_from']})"
    if b["fails"] is not None and v >= b["fails"] - 1e-12: return "fails", f"{name}: {value:g} at or past {b['fails']:g}: {b['cost']}"
    return "outside", (f"{name}: {value:g} past {b['ok']:g}" + (f", short of {b['fails']:g} (untested between: treat as outside; {b['cost']})"
                                                                 if b["fails"] is not None else f" (untested past it; {b['cost']})"))


def mount_series(checks, band=None):
    """the mount night to night ([GWP]): each night's nightly check (floor_selfcal.py check on that night's views alone,
    its JSON) gives the floor's disagreement with the robot's calibration in pitch, yaw and height; a night past the band
    says the calibration is off, and a night whose disagreement differs from the first judged night's by more than the
    band says the head moved between them (a knock, a loose bracket). checks: [(name, check dict)] in night order.
    band: per component, default the nightly check's own tolerance (CHECK_TOL; pitch's 0.1 deg is DAY_ONE_BANDS')"""
    if band is None:
        if str(HK / "sim" / "house") not in sys.path: sys.path.insert(0, str(HK / "sim" / "house"))
        from floor_selfcal import CHECK_TOL
        band = {k: CHECK_TOL[k] for k in ("pitch_deg", "yaw_deg", "up_m")}
    unit = {"pitch_deg": (1, "deg"), "yaw_deg": (1, "deg"), "up_m": (1000, "mm")}
    nights, off, moved, ref = [], [], [], None
    for name, d in checks:
        ch = d["change"]; judged = bool(d.get("judged", True)); row = dict(night=name, judged=judged, **{k: ch.get(k) for k in band})
        nights.append(row)
        if not judged: continue
        for k, t in band.items():
            f, u = unit[k]
            if abs(ch[k]) > t: off.append(f"{name}: {k.split('_')[0]} {ch[k] * f:+.3g} {u} from the robot's calibration (band {t * f:g} {u})")
            if ref is not None and abs(ch[k] - ref[1][k]) > t:
                moved.append(f"{name} against {ref[0]}: {k.split('_')[0]} moved {(ch[k] - ref[1][k]) * f:+.3g} {u} (band {t * f:g} {u})")
        if ref is None: ref = (name, ch)
    judged = [r for r in nights if r["judged"]]
    verdict = "not judged" if len(judged) < 2 else ("outside" if off or moved else "pass")
    if len(judged) < 2:
        why = f"{len(judged)} of {len(nights)} nights judged (too little floor in the others): two are needed to say it stayed put"
    else:
        why = "; ".join(moved + off) or f"the mount held within {', '.join(f'{t * unit[k][0]:g} {unit[k][1]} {k.split(chr(95))[0]}' for k, t in band.items())} over {len(judged)} nights"
    return dict(verdict=verdict, note=why, nights=nights, moved=moved, off=off, band=band)


COMP_TILE = 64              # the dark floor is read in tiles this size, on the 16-pixel grid (a JPEG block is 8, an H.264 macroblock 16)
COMP_DARK = (0.03, 0.25)    # a dark floor's tiles: mean grey (0-1) in this band, on both sides alike (under it, crushed to black)


def _grey(a):
    a = np.asarray(a, np.float32); return (a.mean(-1) if a.ndim == 3 else a) / 255


def dark_tiles(g, tile=COMP_TILE, band=COMP_DARK, floor_from=0.5):
    """the dark floor's tiles in one grey frame (0-1): its lower part, from floor_from of the height down, cut on the
    16-pixel grid so the code blocks' edges fall at the same offsets in every tile, each tile's mean grey in band (the
    same band on the real and the sim side, so the two are read at like brightness)"""
    H, W = g.shape; y0 = -(-int(H * floor_from) // 16) * 16; out = []
    for y in range(y0, H - tile + 1, tile):
        for x in range(0, W - tile + 1, tile):
            t = g[y:y + tile, x:x + tile]
            if band[0] <= t.mean() <= band[1]: out.append(t)
    return out


_n8 = np.arange(8); _DCT8 = np.sqrt(2 / 8) * np.cos(np.pi * (2 * _n8[None] + 1) * _n8[:, None] / 16); _DCT8[0] /= np.sqrt(2)
COMP_FLAT = 1.0             # an 8x8 block's frequency under this (grey levels) counts as zeroed by the compression


def _tile_measures(t):
    """one dark tile (0-1, on the grid): its 8x8 blocks' frequencies (the orthonormal DCT, grey levels) and the step
    across the blocks' edges against the step inside them"""
    n = t.shape[0] // 8; B = (t * 255).reshape(n, 8, n, 8).transpose(0, 2, 1, 3)
    ac = np.einsum("ij,abjk,lk->abil", _DCT8, B, _DCT8).reshape(-1, 64)[:, 1:]
    dx = np.abs(np.diff(t, axis=1)) * 255; dy = np.abs(np.diff(t, axis=0)) * 255
    edge = np.r_[dx[:, 7::8].ravel(), dy[7::8].ravel()].mean()
    inside = np.r_[np.delete(dx, np.s_[7::8], axis=1).ravel(), np.delete(dy, np.s_[7::8], axis=0).ravel()].mean()
    return (np.abs(ac) < COMP_FLAT).mean(), np.abs(ac).mean(), edge - inside


def compression_measures(frames, max_frames=60):
    """how compressed the dark floor looks ([4hm], [GWP], camera.md 3), over the floor's dark tiles (dark_tiles):
    flat, the share of the 8x8 blocks' frequencies (all but the mean) under one grey level, which compression zeroes
    and grain fills (rises from about 0.2-0.4 uncompressed toward 1 as the quality falls); ac, their mean size (grey
    levels: the fine texture and grain left); blockiness, the step across the blocks' edges less the step inside (grey
    levels; a dark floor compressed hard goes flat both sides of an edge, so it fades again, and H.264's deblocking hides
    it: shown, not judged on). Pooled, and flat per frame (median and 90th percentile: the stream's dips). frames: RGB
    or grey arrays (uint8)"""
    if len(frames) > max_frames: frames = [frames[int(i)] for i in np.linspace(0, len(frames) - 1, max_frames)]
    per, rows, level = [], [], []
    for f in frames:
        T = dark_tiles(_grey(f))
        if not T: continue
        m = np.array([_tile_measures(t) for t in T]); rows.append(m); per.append(m[:, 0].mean())
        level.append(float(np.mean([t.mean() for t in T])))
    if not per: return None
    m = np.concatenate(rows).mean(0)
    return dict(flat=round(float(m[0]), 4), flat_p50=round(float(np.median(per)), 4), flat_p90=round(float(np.percentile(per, 90)), 4),
                ac=round(float(m[1]), 3), blockiness=round(float(m[2]), 3), dark_level=round(float(np.median(level)), 4), frames=len(per))


def reencode(frames, quality):
    """frames passed through JPEG at quality (the sensitivity study's `jpegQ` variant: a sim frame compressed again)"""
    import cv2
    out = []
    for f in frames:
        ok, buf = cv2.imencode(".jpg", np.asarray(f), [cv2.IMWRITE_JPEG_QUALITY, int(quality)]); out.append(cv2.imdecode(buf, cv2.IMREAD_UNCHANGED))
    return out


def compression_sheet(rows, path, n=6):
    """a side-by-side of the dark floor (camera.md 3): one row of up to n dark tiles from each set of frames (one tile
    a frame, the same place in each row's frames when they are the same frames re-encoded), stretched to the tiles' own
    range and enlarged 3x, so the blocks and flattening show at phone size. rows: [(label, frames)]; the floor only,
    no person can be in it, and it stays in the house"""
    import cv2
    strips = []
    for _, frames in rows:
        T = []
        for f in frames:
            d = dark_tiles(_grey(f))
            if d: T.append(d[len(d) // 2])
            if len(T) == n: break
        strips.append(np.concatenate(T + [np.zeros((COMP_TILE, COMP_TILE), np.float32)] * (n - len(T)), 1))
    im = np.concatenate(strips, 0); lo, hi = np.percentile(im, (1, 99)); im = np.clip((im - lo) / max(hi - lo, 1e-3), 0, 1)
    cv2.imwrite(str(path), cv2.resize((im * 255).astype(np.uint8), None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
    return [r[0] for r in rows]


def compression_compare(real_frames, sim_frames, sheet=None):
    """the real stream's dark floor beside the sim's, and the sim's re-encoded at day one's qualities (DAY_ONE_BANDS
    compression: q50 cost nothing, q30 cost a third more false alarms; the study's own `jpeg50` and `jpeg30` variants),
    the measures of each and, with sheet, the side-by-side picture ([GWP]). No automatic verdict: on the sim's own dark
    floor (house 96, 14 Oct 2026) re-encoding at q50 or q30 moves these measures by a few per cent, less than two
    houses' floors at the same quality differ, so a real stream is placed between them by eye on the sheet, the numbers
    beside it, against a sim night of a like floor; a real floor flatter or blockier than the sim's at q30 by more than
    that is the clear fail"""
    b = DAY_ONE_BANDS["compression"]
    S50, S30 = reencode(sim_frames, b["ok"]), reencode(sim_frames, b["fails"])
    out = dict(real=compression_measures(real_frames), sim=compression_measures(sim_frames),
               **{f"sim_q{b['ok']}": compression_measures(S50), f"sim_q{b['fails']}": compression_measures(S30)})
    if sheet: out["sheet"] = dict(path=str(sheet), rows=compression_sheet([("real", real_frames), ("sim", sim_frames),
                                                                           (f"sim q{b['ok']}", S50), (f"sim q{b['fails']}", S30)], sheet))
    return out


def frames_by_motion(st, n=60, speed=0.1):
    """the head camera's frames standing still (still_spans) and walking (odometry over speed m/s), up to n of each,
    spread evenly over the recording"""
    from floor_mosaic2 import arr
    t, x, y, yaw = odom_track(st); spans = still_spans(t, x, y, yaw)
    v = np.r_[0, np.hypot(np.diff(x), np.diff(y)) / np.maximum(np.diff(t), 1e-3)]
    still_ts, walk_ts = [], []
    obs = list(st.stream("color_image"))
    for i, o in enumerate(obs):
        if any(a <= o.ts <= b for a, b in spans): still_ts.append(i)
        elif t[0] <= o.ts <= t[-1] and v[min(np.searchsorted(t, o.ts), len(v) - 1)] > speed: walk_ts.append(i)
    pick = lambda ix: [arr(obs[ix[int(j)]].data) for j in np.linspace(0, len(ix) - 1, min(n, len(ix)))] if ix else []
    return pick(still_ts), pick(walk_ts)


# ---- the gap report ([KbT]) and the sim's side of the matched measurements ([esI]) ----------------------------------
SIM_REFERENCE = HK / "real" / "calibration" / "sim_reference.json"
def _hold_out():
    """hold-out houses: never measured here (results only, their own harness); make_house's registry, the one list
    (sim-house-nights [ejh]), with the first two as a floor should the generator be absent"""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim" / "house"))
    try:
        import make_house
        return set(make_house.HOLDOUT)
    except ImportError:
        return {80, 81}


HOLD_OUT = _hold_out()
TOL = 0.10                          # sim and real agree within 10% (camera.md: tune until the ratio is within 10% of 1)
WALL_STAMPED = "sim frames and scans are stamped on the wall clock while the sim runs below real time: not comparable"


DAY_ONE_MAX_LAG = 1.0   # s: day one's search reaches past the nightly check's (0.3 s); a camera over the robot's
                        # link may well be later than that, and must be measured, not cut off at the edge


def day_one_lag(st):
    """the frames' lag behind odometry from turning on the spot, searched out to DAY_ONE_MAX_LAG and held to the robot's
    lag gate (calibrate.lag_refusal, its range widened to the search): {lag_s, corr, pairs, turning, refused (why no
    lag was measured, or None: never a 0 standing in for a failed measurement), past_nightly (a lag the nightly check
    cannot see, or None)}"""
    from floor_mosaic2 import lag_detail_from_turns
    from houndkeeper import calibrate as cal
    d = lag_detail_from_turns(st, max_lag=DAY_ONE_MAX_LAG)
    d["refused"] = cal.lag_refusal(d, dict(cal.LAG_GATE, lo=-DAY_ONE_MAX_LAG, hi=DAY_ONE_MAX_LAG))
    hi = cal.LAG_GATE["hi"]
    d["past_nightly"] = (None if d["refused"] or not (d["lag_s"] < cal.LAG_GATE["lo"] or d["lag_s"] > hi) else
                         f"{d['lag_s']:+.3f} s is outside the nightly check's {cal.LAG_GATE['lo']}..{hi} s: widen "
                         "calibrate.LAG_GATE and frame_lag_detail's max_lag before trusting the nightly check")
    return d


def analyse(st, box=None, truth=None):
    """every analysis that applies to one recording, flattened to {quantity: value}; one that can't run (no still
    span, no battery stream, no --box or --truth) leaves its quantities out and says why under 'skipped'"""
    if str(HK / "sim" / "house") not in sys.path: sys.path.insert(0, str(HK / "sim" / "house"))   # floor_mosaic2 (frames, lag)
    if str(HK / "src") not in sys.path: sys.path.insert(0, str(HK / "src"))                       # houndkeeper (calibrate)
    out, skipped = {}, {}
    def run(name, f):
        try: f()
        except Exception as e: skipped[name] = f"{type(e).__name__}: {e}"[:160]
    def stream_():
        r = frame_gaps(stamps(st, "color_image")); out.update({f"stream.{k}": r[k] for k in ("period_s", "drop", "stall_per_min", "stall_s", "minutes")})
    def lag_():
        d = day_one_lag(st); out["lag.correlation"] = round(float(d["corr"]), 3)
        if d["refused"]: skipped["lag"] = d["refused"]; return
        out["lag.frame_lag_vs_odom_s"] = round(float(d["lag_s"]), 3)
    def lidar_():
        r, L = lidar_rate(st); out.update({f"lidar.{k}": v for k, v in r.items()})
        if box is None: skipped["wall"] = "no --box round a flat wall"; return
        w = plane_spread(L, box)
        x0, x1, y0, y1, z0, z1 = box; seen = []                             # returns a scan, over the scans that see the wall
        for o in L:
            p = np.asarray(o.data.as_numpy()[0], float)
            n = int(((p[:, 0] > x0) & (p[:, 0] < x1) & (p[:, 1] > y0) & (p[:, 1] < y1) & (p[:, 2] > z0) & (p[:, 2] < z1)).sum())
            if n >= 30: seen.append(n)
        if seen: out["wall.returns_per_scan"] = float(np.median(seen))
        if w["range_sd"] is not None: out["wall.range_sd"] = w["range_sd"]
        else: skipped["wall"] = "under 30 returns a scan in the box"
    def noise_():
        sp = still_spans(*odom_track(st)); r = camera_noise(st, sp)
        if r is None: skipped["noise"] = f"no usable still span ({len(sp)} found)"; return
        out["noise.read_noise"] = r["read_noise"]; out["noise.shot_noise"] = r["shot_noise"]
    def odom_():
        if truth is None: skipped["odom"] = "no --truth (a taped walk, or the sim's traj.csv)"; return
        T = np.genfromtxt(truth, delimiter=",", names=True) if isinstance(truth, (str, Path)) else truth
        r = odom_vs_truth(*odom_track(st), T); out.update({f"odom.{k}": r[k] for k in ("walked_m", "scale", "xy_per_root_m", "yaw_deg_per_root_m")})
    def power_():
        import yaml
        try: bt, soc, _, _, P = battery_track(st)
        except Exception: bt = []                                           # DimOS: no such stream in this recording
        if len(bt) < 2: skipped["power"] = "no battery stream"; return
        wh = yaml.safe_load(PARAMS.read_text())["go2_air"]["battery_Wh"]["value"]
        r = power_split(bt, soc, P, odom_track(st), wh)
        for k in ("walk_power_W", "stand_power_W"):
            if r[k] is not None: out[f"power.{k}"] = r[k]
    for name, f in (("stream", stream_), ("lag", lag_), ("lidar", lidar_), ("noise", noise_), ("odom", odom_), ("power", power_)): run(name, f)
    return dict(values=out, skipped=skipped)


def go2_params(params=None):
    import yaml
    return {k: v for k, v in yaml.safe_load((PARAMS if params is None else params).read_text())["go2_air"].items()}


def _pv(P, path):
    """a parameter's value by 'key' or 'key.sub' (None if the file lacks it)"""
    k, _, sub = path.partition("."); e = P.get(k)
    if e is None: return None
    v = e["value"]; return v.get(sub) if sub else v


# Each quantity the gap report knows: (key, label, kind, parameter, what the parameter's value is for this quantity)
#   direct:  the real value IS the parameter (or a piece of it): write it
#   matched: sim and real measured the same way; tune the sim's parameter until they agree
#   info:    shown for context, no parameter of its own
QUANTITIES = [
    ("stream.period_s", "frame period (s)", "info", None, None),
    ("stream.drop", "frames dropped singly (share)", "direct", "video_stream.drop", lambda P: _pv(P, "video_stream.drop")),
    ("stream.stall_per_min", "stream stalls a minute", "direct", "video_stream.stall_per_min", lambda P: _pv(P, "video_stream.stall_per_min")),
    ("stream.stall_s", "stall length (s)", "direct", "video_stream.stall_s", lambda P: _pv(P, "video_stream.stall_s")),
    ("lag.frame_lag_vs_odom_s", "frames' lag behind odometry (s)", "direct", "frame_delay_s.mean",
     lambda P: round(_pv(P, "frame_delay_s.mean") - _pv(P, "odom_delay_s.mean"), 3) if P.get("frame_delay_s") and P.get("odom_delay_s") else None),
    ("lidar.points_per_message", "lidar points a message (DimOS)", "matched", "lidar.points_per_s", lambda P: _pv(P, "lidar.points_per_s")),
    ("lidar.points_per_s", "lidar points a second (DimOS)", "info", None, None),
    ("lidar.messages_per_s", "lidar messages a second (DimOS)", "info", None, None),
    ("wall.range_sd", "flat wall's range spread (m)", "direct", "lidar.range_sd", lambda P: _pv(P, "lidar.range_sd")),
    ("wall.returns_per_scan", "wall returns a scan (scans that see it)", "info", None, None),
    ("noise.read_noise", "camera effective read noise (sd, 0-1)", "matched", "camera_imperfections.read_noise", lambda P: _pv(P, "camera_imperfections.read_noise")),
    ("noise.shot_noise", "camera effective shot noise (var/level)", "matched", "camera_imperfections.shot_noise", lambda P: _pv(P, "camera_imperfections.shot_noise")),
    ("odom.scale", "odometry scale error (one walk)", "direct", "odom_drift.scale_sd", lambda P: _pv(P, "odom_drift.scale_sd")),
    ("odom.xy_per_root_m", "odometry position drift (m per root m)", "matched", "odom_drift.xy_sd", lambda P: _pv(P, "odom_drift.xy_sd")),
    ("odom.yaw_deg_per_root_m", "heading drift (deg per root m)", "direct", "odom_drift.yaw_sd_deg", lambda P: _pv(P, "odom_drift.yaw_sd_deg")),
    ("power.walk_power_W", "power walking (W)", "direct", "walk_power_W", lambda P: _pv(P, "walk_power_W")),
    ("power.stand_power_W", "power standing (W)", "direct", "stand_power_W", lambda P: _pv(P, "stand_power_W")),
]
WALL_CLOCK = {"stream.period_s", "stream.drop", "stream.stall_per_min", "stream.stall_s", "lidar.points_per_s", "lidar.messages_per_s",
              "lag.frame_lag_vs_odom_s"}       # sim values these come from are stamped on the wall clock
WRITE_HOW = {"video_stream": "measure.py stream <real.db> --write", "lidar.range_sd": "measure.py lidar <real.db> --box ... --write",
             "frame_delay_s.mean": "by hand: odom_delay_s.mean + this lag, after timing.md 1-3",
             "odom_drift": "by hand, after several walks (odometry.md)", "walk_power_W": "measure.py power <real.db> --write",
             "stand_power_W": "measure.py power <real.db> --write"}


ATOL = {"noise.shot_noise": 2e-5,       # under this the fit can't tell shot noise from none (sd ~0.6 grey levels at mid-grey)
        "noise.read_noise": 5e-4, "lag.frame_lag_vs_odom_s": 0.02, "wall.range_sd": 0.002, "stream.drop": 0.005}


def _agree(a, b, tol=TOL, atol=1e-9):
    return a is not None and b is not None and (abs(a - b) <= tol * max(abs(a), abs(b)) or abs(a - b) <= atol)


def gap_rows(real, sim, P):
    """the gap report's rows: for each quantity the real recording gave, the sim's value (a sim recording's, or the
    reference's), the file's value, the gap and what to do: a matched quantity names the sim parameter to raise or
    lower and by what factor (the same factor applied to the sim should close it: the analyses scale linearly with
    it); a direct one names the parameter to write and the command that writes it"""
    rows = []
    for key, label, kind, param, fileval in QUANTITIES:
        r, s = real.get(key), sim.get(key); pv = fileval(P) if fileval else None
        if r is None: continue
        row = dict(quantity=key, label=label, kind=kind, real=r, sim=s, param=param, param_value=pv, gap=None, action="",
                   flag=("sim value: " + WALL_STAMPED) if key in WALL_CLOCK and s is not None else "")
        if kind == "matched":
            if s is None: row["action"] = "no sim value: run the same measurement on a sim night of the scene"
            elif _agree(r, s, atol=ATOL.get(key, 1e-9)):
                row["gap"] = f"x{r / s:.2f}" if s else f"{r - s:+.3g}"
                row["action"] = f"agree (within {TOL:.0%}" + (f" or {ATOL[key]:g}" if key in ATOL else "") + f"): leave go2_air.{param}"
            elif s < ATOL.get(key, 0) or s == 0:
                row["gap"] = f"{r - s:+.3g} (sim ~0)"; row["action"] = f"raise go2_air.{param}: the sim shows next to none; re-fit after raising it"
            else:
                k = r / s; row["gap"] = f"x{k:.2f}"           # read noise is an sd, shot noise a variance: each scales as measured
                to = f" (to ~{pv * k:.4g})" if isinstance(pv, (int, float)) else ""
                row["action"] = f"{'raise' if k > 1 else 'lower'} go2_air.{param} x{k:.2f}{to}, re-run the sim night, measure again"
                if key.startswith("noise."): row["action"] += " (or isp_noise_keep, which sets what survives the camera's noise reduction)"
                if key == "odom.xy_per_root_m": row["action"] += " (the heading's drift adds to it: settle yaw_sd_deg first; the sim's comes from the same kind of walk)"
        elif kind == "direct":
            ref = pv
            if key == "odom.scale": row["gap"] = f"{r:+.3f} (one draw; sd {pv})" if pv is not None else None
            elif ref is not None: row["gap"] = f"{r - ref:+.4g}" + (f" ({(r - ref) / ref:+.0%})" if ref else "")
            how = next((v for k_, v in WRITE_HOW.items() if param.startswith(k_)), "by hand")
            same = key != "odom.scale" and _agree(r, ref, atol=ATOL.get(key, 1e-9))
            row["action"] = (f"write go2_air.{param} = {r:g}" if key not in ("odom.scale", "lag.frame_lag_vs_odom_s") else
                             f"combine into go2_air.{param}" if key == "odom.scale" else f"lag {r:g} s: set go2_air.{param}") + f" ({how})" + \
                            (" (agrees with the file within 10%: the source becomes measured)" if same else "")
            if key == "lag.frame_lag_vs_odom_s" and real.get("lag.correlation") is not None and real["lag.correlation"] < 0.7:
                row["action"] = f"unreliable (turning correlation {real['lag.correlation']}, under 0.7): turn on the spot again (timing.md 2)"
        rows.append(row)
    return rows


def _fmt(v):
    if v is None: return "-"
    if isinstance(v, float): return f"{v:.4g}"
    return str(v)


def gap_markdown(rows, real_name, sim_name, skipped=None, notes=()):
    out = [f"# Sim-to-real gap report ([KbT])", "", f"Real: `{real_name}`  ", f"Sim: {sim_name}  ",
           f"Made {time.strftime('%Y-%m-%d %H:%M')} by `real/calibration/measure.py gap`; parameters from `params/robot.yaml`.", "",
           "| Quantity | Kind | Sim | Real | File | Gap | Parameter: what to do |", "|---|---|---|---|---|---|---|"]
    for r in rows:
        sim = _fmt(r["sim"]) + (" *" if r["flag"] else "")
        out.append(f"| {r['label']} | {r['kind']} | {sim} | {_fmt(r['real'])} | {_fmt(r['param_value'])} | {r['gap'] or '-'} | {r['action'] or '-'} |")
    if any(r["flag"] for r in rows): out += ["", f"\\* {WALL_STAMPED} (compare the real stream with the file instead)."]
    out += ["", "Direct: the real value is the parameter (write it). Matched: measured the same way on both; tune the sim's parameter "
            f"until they agree within {TOL:.0%} (camera.md, lidar.md)."]
    if skipped: out += ["", "Not measured on the real recording: " + "; ".join(f"{k} ({v})" for k, v in skipped.items()) + "."]
    out += ["", *notes]
    return "\n".join(out) + "\n"


def sim_wall_box(seed, dock_x):
    """a flat, plain wall near the dock in a sim house's world frame (the robot stands there at the start of a night):
    the demo room's east wall it faces from the dock ([qEP], house 95); in the tuning houses, the south outer wall
    beside the dock (mirrored plans put the dock on the east)"""
    if seed == 95: return (-2.60, -2.30, -0.8, 0.6, 0.35, 1.4)
    sx = 1 if dock_x > 0 else -1
    return tuple(sorted((sx * 5.0, sx * 7.5))) + (-4.70, -4.40, 0.35, 1.4)


def sim_recordings(run_dir, nights=(1, 2)):
    """a sim run directory's recordings: (seed, night, memory.db, traj.csv) from its seed_recordings.txt"""
    run_dir = Path(run_dir).resolve(); seed = int(re.search(r"_s(\d+)", run_dir.name).group(1))
    if seed in HOLD_OUT: raise SystemExit(f"house {seed} is a hold-out: not measured here")
    out = []
    for ln in (run_dir / "seed_recordings.txt").read_text().split("\n"):
        if not ln.strip(): continue
        n, db = ln.split(None, 1); n = int(n)
        if n in nights: out.append((seed, n, db.strip(), run_dir / f"{run_dir.name}_n{n}.traj.csv"))
    return sorted(out, key=lambda r: r[1])


def sim_speed(traj):
    T = np.genfromtxt(traj, delimiter=",", names=True)
    return round(float((T["sim_t"][-1] - T["sim_t"][0]) / (T["wall_t"][-1] - T["wall_t"][0])), 3)


def _measure_sim(rec):
    seed, night, db, traj = rec
    sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))
    st = store(db); t, x, *_ = odom_track(st); box = sim_wall_box(seed, x[0])
    a = analyse(st, box=box, truth=str(traj) if Path(traj).exists() else None)
    m = re.search(r"(\d{8})-(\d{6})", db)
    return dict(house=seed, night=night, db=db, recorded=f"{m.group(1)[:4]}-{m.group(1)[4:6]}-{m.group(1)[6:]} {m.group(2)[:2]}:{m.group(2)[2:4]}" if m else None,
                traj=str(Path(traj).relative_to(HK)) if Path(traj).exists() else None, wall_box=list(box),
                sim_speed=sim_speed(traj) if Path(traj).exists() else None, **a)


def sim_reference(run_dirs, nights=(1, 2), workers=4):
    """the sim's values of every quantity the gap report compares, from existing sim nights: each recording's own and,
    per quantity, the median and range over them, with the parameters in force (the file's values) when made"""
    recs = [r for d in run_dirs for r in sim_recordings(d, nights)]
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(min(workers, len(recs))) as ex: per = list(ex.map(_measure_sim, recs))
    keys = sorted({k for p in per for k in p["values"]}); summary = {}
    for k in keys:
        v = [p["values"][k] for p in per if p["values"].get(k) is not None]
        summary[k] = dict(median=round(float(np.median(v)), 6), min=float(min(v)), max=float(max(v)), n=len(v),
                          **({"note": WALL_STAMPED} if k in WALL_CLOCK else {}))
    P = go2_params(); used = sorted({q[3].split(".")[0] for q in QUANTITIES if q[3]} | {"odom_delay_s"})
    return dict(made=time.strftime("%Y-%m-%d"), by="real/calibration/measure.py simref " + " ".join(str(Path(d)) for d in run_dirs),
                about="The sim's side of the matched measurements ([esI], [KbT]): measure.py's analyses on existing sim nights. "
                      "The gap report (measure.py gap) reads summary.<quantity>.median as the sim's value.",
                caveats=[WALL_STAMPED + ": stream.*, lag and the lidar's per-second rates are per wall second; sim_speed is the sim "
                         "seconds a wall second each night ran at",
                         "noise.*: effective noise after the sim's own noise reduction and JPEG, from the still spans at the dock at a night's start",
                         "odom.*: against the sim's own truth (traj.csv), one draw of odom_drift per night",
                         "wall.*: the demo room's east wall (house 95), the south outer wall by the dock elsewhere (sim_wall_box)"],
                params={k: dict(value=P[k]["value"], source=P[k].get("source")) for k in used if k in P},
                recordings=per, summary=summary)


def gap_main(argv):
    ap = argparse.ArgumentParser(prog="measure.py gap", description="the sim-to-real gap report ([KbT]): every analysis that applies on "
                                 "a real recording, beside the sim's value and params/robot.yaml's, and what to adjust")
    ap.add_argument("db", help="the real recording (memory.db)")
    g = ap.add_mutually_exclusive_group(); g.add_argument("--sim", help="a sim recording of the same scene, measured now")
    g.add_argument("--reference", help=f"the sim's reference values (default {SIM_REFERENCE.relative_to(HK)})")
    ap.add_argument("--box", type=float, nargs=6, help="a flat wall in the real recording's map frame (x0 x1 y0 y1 z0 z1, m)")
    ap.add_argument("--sim-box", type=float, nargs=6, help="with --sim: the same wall in the sim's frame")
    ap.add_argument("--truth", help="odometry's true path for the real recording (a taped walk, odometry.md)")
    ap.add_argument("--sim-truth", help="with --sim: the sim's traj.csv")
    ap.add_argument("--out", help="write the markdown report here too"); ap.add_argument("--json", help="write the rows as JSON here")
    a = ap.parse_args(argv); sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))
    real = analyse(store(a.db), box=a.box, truth=a.truth); notes = []
    if a.sim:
        sv = analyse(store(a.sim), box=a.sim_box, truth=a.sim_truth)["values"]; sim_name = f"`{a.sim}` (measured now)"
    else:
        ref = Path(a.reference) if a.reference else SIM_REFERENCE
        R = json.loads(ref.read_text()); sv = {k: v["median"] for k, v in R["summary"].items()}
        sim_name = f"`{ref}` (median of {len(R['recordings'])} sim nights, made {R['made']})"
        notes.append("The reference's parameters in force differ from the file's now: " + ", ".join(
            k for k, v in R["params"].items() if go2_params().get(k, {}).get("value") != v["value"]) + ": re-make it (measure.py simref)."
            if any(go2_params().get(k, {}).get("value") != v["value"] for k, v in R["params"].items()) else
            "The reference was made with the parameters the file holds now.")
    rows = gap_rows(real["values"], sv, go2_params())
    md = gap_markdown(rows, a.db, sim_name, real["skipped"], notes); print(md, end="")
    if a.out: Path(a.out).write_text(md)
    if a.json: Path(a.json).write_text(json.dumps(dict(real=a.db, sim=sim_name, rows=rows, skipped=real["skipped"]), indent=1))
    return rows


def simref_main(argv):
    ap = argparse.ArgumentParser(prog="measure.py simref", description="the sim's reference values for the gap report, from existing sim nights")
    ap.add_argument("run_dirs", nargs="+", help="sim run directories (sim/house/runs/nights_s<seed>), each with seed_recordings.txt")
    ap.add_argument("--nights", type=int, nargs="+", default=[1, 2]); ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--out", default=str(SIM_REFERENCE))
    a = ap.parse_args(argv); R = sim_reference(a.run_dirs, tuple(a.nights), a.workers)
    Path(a.out).write_text(json.dumps(R, indent=1) + "\n")
    for k, v in R["summary"].items(): print(f"{k:28s} {v['median']:<12g} [{v['min']:g} .. {v['max']:g}] n={v['n']}")


def mount_main(argv):
    ap = argparse.ArgumentParser(prog="measure.py mount", description="the head camera's mount night to night ([GWP]): the nightly checks "
                                 "(floor_selfcal.py check, one per night's views) side by side, flagged past the band")
    ap.add_argument("checks", nargs="+", help="each night's check JSON, in night order")
    ap.add_argument("--json", help="write the result here too")
    a = ap.parse_args(argv)
    r = mount_series([(f"{Path(f).parent.name}/{Path(f).stem}", json.loads(Path(f).read_text())) for f in a.checks])
    for n in r["nights"]:
        print(f"{n['night']:40s} " + ("not judged (too little floor)" if not n["judged"] else
              f"pitch {n['pitch_deg']:+.3f} deg  yaw {n['yaw_deg']:+.3f} deg  up {n['up_m'] * 1000:+.1f} mm"))
    print(f"day one ([GWP]): {r['verdict']}: {r['note']}")
    if a.json: Path(a.json).write_text(json.dumps(r, indent=1))
    return r


def main():
    if len(sys.argv) > 1 and sys.argv[1] in ("gap", "simref"):                # the gap report and the sim's reference ([KbT])
        return (gap_main if sys.argv[1] == "gap" else simref_main)(sys.argv[2:])
    if len(sys.argv) > 1 and sys.argv[1] == "mount":                          # the mount night to night ([GWP])
        return mount_main(sys.argv[2:])
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0]); ap.add_argument("command", choices=["stream", "lag", "lidar", "odom", "noise", "power", "compression"])
    ap.add_argument("db"); ap.add_argument("--box", type=float, nargs=6, default=None)
    ap.add_argument("--truth", default=None, help="odom: the true path (traj.csv: wall_t, true_x, true_y, true_yaw)")
    ap.add_argument("--write", action="store_true", help="update params/robot.yaml with what was found (direct measurements only)")
    ap.add_argument("--sim", default=None, help="a sim recording of the same scene: the same measurement on it, side by side (the gap)")
    ap.add_argument("--expect", type=float, default=None, help="lag: the stored lag (s) the floor pictures use, judged against day one's band ([GWP])")
    ap.add_argument("--frames", type=int, default=60, help="compression: frames read, standing and walking each")
    ap.add_argument("--sheet", default=None, help="compression: the side-by-side picture (PNG; _standing and _walking added), kept in the house (real/local/)")
    a = ap.parse_args(); sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))
    st = store(a.db); how = f"real/calibration/measure.py {a.command} on {Path(a.db).parent.name}"
    sim = store(a.sim) if a.sim else None
    if a.command == "stream":
        r = frame_gaps(stamps(st, "color_image")); print(json.dumps(r))
        if sim: print("sim:", json.dumps(frame_gaps(stamps(sim, "color_image"))), "(the sim's frames are stamped on the wall clock while it runs below real time: compare the real stream with the file, not with this)")
        if a.write: set_param("go2_air.video_stream", {k: r[k] for k in ("drop", "stall_per_min", "stall_s")}, how)
    elif a.command == "lag":
        d = day_one_lag(st); lag, c = d["lag_s"], d["corr"]
        print(json.dumps(dict(frame_lag_vs_odom_s=None if lag is None else round(lag, 3), correlation=round(float(c), 3),
                              pairs=d.get("pairs"), turning=d.get("turning"))))
        if d["refused"]: print(f"no lag measured: {d['refused']}; turn on the spot again, slowly both ways"); sys.exit(1)
        if d["past_nightly"]: print(f"note: {d['past_nightly']}")
        if a.expect is not None:
            v, why = band_verdict("lag", lag - a.expect); print(f"day one ([GWP]): {v}: {why}")
        if a.write: print("frame_delay_s is the camera's lag plus odometry's own (timing.md): write it by hand")
    elif a.command == "lidar":
        r, L = lidar_rate(st)
        if a.box: r.update(plane_spread(L, a.box))
        print(json.dumps(r))
        if sim:
            rs, Ls = lidar_rate(sim)
            if a.box: rs.update(plane_spread(Ls, a.box))
            print("sim:", json.dumps(rs))
        if a.write and r.get("range_sd") is not None: set_param("go2_air.lidar.range_sd", r["range_sd"], how + f" --box {a.box}")
    elif a.command == "odom":
        T = np.genfromtxt(a.truth, delimiter=",", names=True); r = odom_vs_truth(*odom_track(st), T); print(json.dumps(r))
        P = go2_params()
        for k, pk in (("yaw_deg_per_root_m", "odom_drift.yaw_sd_deg"), ("xy_per_root_m", "odom_drift.xy_sd")):
            if r.get(k) is not None and _pv(P, pk):
                v, why = band_verdict("drift", r[k] / _pv(P, pk)); print(f"day one ([GWP]): {k} x{r[k] / _pv(P, pk):.2f} of the file's {_pv(P, pk)}: {v}: {why}")
        if a.write: print("odom_drift: combine several walks (odometry.md), then write by hand")
    elif a.command == "noise":
        spans = still_spans(*odom_track(st)); r = camera_noise(st, spans)
        print(json.dumps(dict(still_spans=len(spans), **(r or {}))))
        if sim and r:
            rs = camera_noise(sim, still_spans(*odom_track(sim))); print("sim:", json.dumps(rs))
            if rs and rs["read_noise"]:
                k = r["read_noise"] / rs["read_noise"]
                print(f"real/sim effective noise {k:.2f}: scale camera_imperfections read_noise and shot_noise by {k * k:.2f} "
                      f"(variance) and re-run the sim night, until the two agree (camera.md)")
                v, why = band_verdict("noise", k); print(f"day one ([GWP]): {v}: {why}")
    elif a.command == "compression":
        still, walk = frames_by_motion(st, a.frames)
        print(json.dumps(dict(still=compression_measures(still) if still else None, walking=compression_measures(walk) if walk else None)))
        if sim:
            s_still, s_walk = frames_by_motion(sim, a.frames)
            for name, R, S in (("standing", still, s_still), ("walking", walk, s_walk)):
                if not (R and S): print(f"{name}: no frames on one side"); continue
                sheet = Path(a.sheet).with_name(f"{Path(a.sheet).stem}_{name}.png") if a.sheet else None
                print(name, json.dumps(compression_compare(R, S, sheet)))
            print("day one ([GWP]): judged by eye on the sheet, the numbers beside it (camera.md 3): pass if the real rows look no "
                  "flatter or blockier than the sim's at q50, fails at the sim's q30 or worse")
        else: print("give --sim <a sim night's memory.db over a like dark floor> (and --sheet out.png) to set it beside the sim's frames at q50 and q30 (camera.md 3)")
    elif a.command == "power":
        import yaml
        wh = yaml.safe_load(PARAMS.read_text())["go2_air"]["battery_Wh"]["value"]
        bt, soc, _, _, P = battery_track(st); ot = odom_track(st)
        if len(bt) < 2: sys.exit("no battery stream in this recording: run with houndkeeper.battery on the real robot (power_and_pose.md 1)")
        r = power_split(bt, soc, P, ot, wh); print(json.dumps(r))
        if r["charge_vs_mean"] is not None and abs(r["charge_vs_mean"]) > 0.1:
            print(f"the charge used and the measured power disagree by {r['charge_vs_mean']:+.0%}: check before writing (power_and_pose.md 1)")
        if a.write:
            if r["walk_power_W"] is not None: set_param("go2_air.walk_power_W", r["walk_power_W"], how + f" ({r['walk_s']} s walking)")
            if r["stand_power_W"] is not None: set_param("go2_air.stand_power_W", r["stand_power_W"], how + f" ({r['stand_s']} s standing)", add_after="walk_power_W")


if __name__ == "__main__":
    main()
