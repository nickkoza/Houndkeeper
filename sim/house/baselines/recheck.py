#!/usr/bin/env python3
"""The floor comparison loosened, a local recognizer restoring precision ([3hK]): the curve of found and false
positives a night, each sensitivity of the floor comparison (floor_arm.py --sensitivity) against each local recognizer
([En1], recognizers.py), scored by the one placement and scorer ([o0E]), on all frames and on the frontier subset.

crops:   every candidate the floor comparison proposed (any sensitivity), the frames of the night's head-camera stream
         that saw it: its floor point (odometry's frame, where the robot would walk to) projected through each frame's
         pose and the fisheye's calibration (place.project, as the first arms place boxes), the VIEWS nearest views
         from RANGE_M, at least SEP_S apart, inside the image by MARGIN_PX; a square around the point, sized for about
         SPAN_M of floor at that range and raised a little (a toy stands up from its point), resized to CROP_PX. The
         robot has these frames: it walked past the spot. Nothing here reads the truth.
verify:  one recognizer's verdict on every crop (its score, label), its time a crop and GPU memory.
score:   a candidate is kept when a recognizer's best verdict over its crops reaches the threshold (a candidate no
         frame saw is kept: the robot cannot check it, so it stops for it); the threshold chosen per pair on s70_t1
         alone by F1 ([XEi]); the kept candidates scored as the floor arm is (table.score_arm), all frames and the
         frontier subset. Writes curve.json and curve.md.
Usage: recheck.py crops <out dir> <floor_*.json ...>
       recheck.py verify <out dir> --model NAME [--variant V]
       recheck.py score <manifest.json> <out dir> --floor <floor_*.json ...> --verdicts <verdicts_*.json ...> [--subset frontier_set.json]
       recheck.py union <manifest.json> <out dir> --pair SENS:RECOGNIZER ... --detector NAME=FILE@THRESHOLD ... --subset frontier_set.json
union:   post hoc (chosen after the curve was seen, 4 Oct 2026; not a pre-set arm): the floor comparison checked by a
         recognizer at its curve threshold, united with a local detector's own finds on the frame set (its threshold
         from table.py), scored the same way on all frames and the subset. Writes union.json.
(dimos venv python; verify on the GPU)"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent; HOUSE = HERE.parent
sys.path.insert(0, str(HOUSE)); sys.path.insert(0, str(HOUSE.parents[1] / "src"))

import numpy as np

VIEWS = 3                # crops a candidate gets
RANGE_M = (0.6, 2.0)     # views from this near to this far (along the floor), nearest first
SEP_S = 1.0              # the views at least this far apart in time (different frames, a little parallax)
MARGIN_PX = 16           # the point at least this far inside the image
SPAN_M = 0.4             # a crop covers about this much floor across at the point's range
CROP_MIN, CROP_MAX = 96, 280   # crop side (px, the full frame's) at its smallest and largest
RAISE = 0.15             # the crop's centre this share of its side above the point
EVERY = 2                # the stream's frames looked at (every 2nd)
GRID_M = 0.05            # candidates within this grid cell share their crops (sensitivities propose the same spots)
THRESHOLDS = [0.0, 0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50, 1.0]


def key(night, x, y):
    """a candidate's crop key: its night and its floor point on GRID_M"""
    return f"{night}:{round(x / GRID_M)}:{round(y / GRID_M)}"


def crop_box(u, v, rng, fx, w, h):
    """the crop around pixel (u, v) of a point rng m off: (x0, y0, side), inside a w x h image"""
    s = int(np.clip(fx * SPAN_M / max(rng, 0.1), CROP_MIN, min(CROP_MAX, h)))
    x0 = int(np.clip(round(u - s / 2), 0, w - s)); y0 = int(np.clip(round(v - s / 2 - RAISE * s), 0, h - s))
    return x0, y0, s


def pick_views(cands, poses, w, h, K, D, model):
    """for each candidate (x, y) the up-to-VIEWS frames (index into poses: [(k, ts, T_world_cam)]) that see it best:
    [(pose index, u, v, range)], nearest first, SEP_S apart"""
    from baselines import place as pl
    out = []
    for x, y in cands:
        seen = []
        for i, (k, ts, T) in enumerate(poses):
            C = T[:3, 3]; r = math.hypot(x - C[0], y - C[1])
            if not RANGE_M[0] <= r <= RANGE_M[1]: continue
            uv = pl.project((x, y), T, K, D, model)
            if uv is None or not (MARGIN_PX <= uv[0] < w - MARGIN_PX and MARGIN_PX <= uv[1] < h - MARGIN_PX): continue
            seen.append((r, i, uv))
        chosen = []
        for r, i, uv in sorted(seen):
            if all(abs(poses[i][1] - poses[j][1]) >= SEP_S for _, j, _ in chosen): chosen.append((r, i, uv))
            if len(chosen) == VIEWS: break
        out.append([(i, uv[0], uv[1], r) for r, i, uv in chosen])
    return out


def night_crops(n, cands, out):
    """the night's crops for its candidates (keys -> (x, y)): {key: [crop entries]}"""
    import cv2
    from dimos.memory.store.sqlite import SqliteStore
    import floor_mosaic2 as fm
    from baselines import frames as fr
    from baselines import recognizers as rc
    st = SqliteStore(path=n["db"]); st.start()
    pose, T_bo, lag, clock, ci = fr.head_camera(st, n["run_dir"], n["db"])
    K = np.array(ci.K, float).reshape(3, 3); D = [float(v) for v in (ci.D or [])]; model = ci.distortion_model or ""
    poses = []
    for k, o in enumerate(st.stream("color_image")):
        if k % EVERY: continue
        P = pose(o.ts - lag)
        if P is not None: poses.append((k, float(o.ts), P @ T_bo))
    w, h = int(ci.width), int(ci.height); keys = list(cands)
    views = pick_views([cands[k_] for k_ in keys], poses, w, h, K, D, model)
    want = {}
    for k_, vs in zip(keys, views):
        for j, (i, u, v, r) in enumerate(vs): want.setdefault(poses[i][0], []).append((k_, j, u, v, r))
    res = {k_: [] for k_ in keys}; d = Path(out) / "crops" / n["id"]; d.mkdir(parents=True, exist_ok=True)
    for k, o in enumerate(st.stream("color_image")):
        if k not in want: continue
        img = fm.arr(o.data)
        for k_, j, u, v, r in want[k]:
            x0, y0, s = crop_box(u, v, r, K[0, 0], w, h)
            c = cv2.resize(np.ascontiguousarray(img[y0:y0 + s, x0:x0 + s]), (rc.CROP_PX, rc.CROP_PX), interpolation=cv2.INTER_LINEAR)
            sc_ = rc.CROP_PX / s; uv = [round((u - x0) * sc_, 1), round((v - y0) * sc_, 1)]
            p = d / f"{k_.split(':', 1)[1].replace(':', '_')}_{j}.png"
            cv2.imwrite(str(p), cv2.cvtColor(c, cv2.COLOR_RGB2BGR))
            res[k_].append({"id": f"{k_}#{j}", "png": str(p), "index": k, "ts": float(o.ts), "range_m": round(r, 2),
                            "uv": uv, "box_full": [x0, y0, x0 + s, y0 + s]})
    st.stop()
    return res, len(poses)


def cmd_crops(a):
    from baselines import nights as nn
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    cands = {}
    for path in a.floor:
        F = json.load(open(path))
        for nid, v in F["nights"].items():
            for p in v["points"]: cands.setdefault(nid, {})[key(nid, p["x"], p["y"])] = (p["x"], p["y"])
    index = {"views": VIEWS, "range_m": RANGE_M, "sep_s": SEP_S, "span_m": SPAN_M, "every": EVERY, "grid_m": GRID_M,
             "floor_files": a.floor, "nights": {}}
    for n in nn.night_set():
        c = cands.get(n["id"], {})
        if not c: continue
        t0 = time.time(); res, npose = night_crops(n, c, out)
        index["nights"][n["id"]] = res
        print(f"{n['id']}: {len(c)} candidates, {sum(len(v) for v in res.values())} crops ({sum(not v for v in res.values())} "
              f"with no view), {npose} posed frames; {time.time() - t0:.0f} s", flush=True)
    json.dump(index, open(out / "crops.json", "w"))


def cmd_verify(a):
    import cv2, torch
    from baselines import recognizers as rc
    out = Path(a.out); index = json.load(open(out / "crops.json"))
    rec = rc.make(a.model, a.variant)
    torch.cuda.reset_peak_memory_stats(); t0 = time.time(); rec.load(); t_load = time.time() - t0
    verdicts, times = {}, []
    crops = [c for night in index["nights"].values() for cs in night.values() for c in cs]
    for k, c in enumerate(crops):
        img = cv2.cvtColor(cv2.imread(c["png"]), cv2.COLOR_BGR2RGB)
        if k == 0: rec.verify(img, c["uv"])
        v, dt = rc.timed(rec.verify, img, c["uv"]); times.append(dt); verdicts[c["id"]] = v
        if k % 200 == 0: print(f"{k}/{len(crops)} crops, {dt * 1000:.0f} ms", flush=True)
    name = rec.name
    res = {"recognizer": name, "model": rec.model, "variant": a.variant, "load_s": round(t_load, 1),
           "ms_per_crop_median": round(float(np.median(times)) * 1000, 1) if times else None, **rc.gpu_stats(), "verdicts": verdicts}
    json.dump(res, open(out / f"verdicts_{name}.json", "w"))
    print(f"{name}: {len(crops)} crops, {res['ms_per_crop_median']} ms a crop (median), {sum(v['score'] > 0 for v in verdicts.values())} "
          f"with something; GPU peak {res.get('gpu_peak_alloc_mb')} MB")


def filtered(F, index, V, thr):
    """the floor arm's file with only the candidates the recognizer keeps at thr (best verdict over its crops)"""
    nights = {}
    for nid, v in F["nights"].items():
        crops = index["nights"].get(nid, {}); keep = []
        for p in v["points"]:
            cs = crops.get(key(nid, p["x"], p["y"]), [])
            best = max((V[c["id"]]["score"] for c in cs if c["id"] in V), default=None)
            if best is None or best >= thr: keep.append(dict(p, score=1.0, recog=best))
        nights[nid] = dict(v, points=keep)
    return dict(F, nights=nights)


def cmd_score(a):
    from baselines import table as tb
    out = Path(a.out); index = json.load(open(out / "crops.json"))
    M = json.load(open(a.manifest)); ctx = tb.context()
    view = None
    if a.subset:
        ids = {e["id"] for e in json.load(open(a.subset))["frames"]}; view = {}
        for f in M["frames"]:
            if f["id"] in ids: view.setdefault(f["night"], []).append(f)
    rows = []
    def row(sens, rname, thr, F, sweep=None, cost=None):
        full = tb.score_arm(F, 0.0, ctx); s = tb.summary(f"{sens} + {rname}", F, 0.0, full)
        r = {"sensitivity": sens, "recognizer": rname, "threshold": thr, "full": s, "sweep_s70_t1": sweep, "cost": cost,
             "candidates_a_night": round(sum(len(v["points"]) for v in F["nights"].values()) / len(F["nights"]), 1)}
        wall = {k: v for k, v in full.items() if k != "s96_t1"}
        r["five_wall_clock"] = {"found": sum(x["found"] for x in wall.values()), "toys": sum(x["n"] for x in wall.values()),
                                "fp_mean": round(sum(x["fp"] for x in wall.values()) / max(len(wall), 1), 1)}
        if view is not None: r["subset"] = tb.summary(f"{sens} + {rname} [frontier subset]", F, 0.0, tb.score_arm(F, 0.0, ctx, view=view))
        rows.append(r); return r
    Vs = [json.load(open(p)) for p in a.verdicts]
    for path in a.floor:
        F = json.load(open(path)); sens = F.get("sensitivity", "today")
        row(sens, "none", None, F)
        for V in Vs:
            one = {tb.THRESHOLD_NIGHT: ctx[tb.THRESHOLD_NIGHT]}; sweep = []
            for t in THRESHOLDS:
                r_ = tb.score_arm(filtered(F, index, V["verdicts"], t), 0.0, one)[tb.THRESHOLD_NIGHT]
                sweep.append((t, r_["found"], r_["n"], r_["fp"], round(tb.f1(r_), 3)))
            thr = max(sweep, key=lambda s: (s[4], s[0]))[0]
            cost = {k: V.get(k) for k in ("model", "ms_per_crop_median", "gpu_peak_alloc_mb", "gpu_peak_reserved_mb", "load_s")}
            row(sens, V["recognizer"], thr, filtered(F, index, V["verdicts"], thr), sweep, cost)
    json.dump({"rows": rows, "views": VIEWS, "range_m": RANGE_M, "thresholds": THRESHOLDS}, open(out / "curve.json", "w"), indent=1)
    (out / "curve.md").write_text(curve_md(rows)); print(curve_md(rows))


def cmd_union(a):
    from baselines import table as tb
    out = Path(a.out); index = json.load(open(out / "crops.json")); M = json.load(open(a.manifest)); ctx = tb.context()
    ids = {e["id"] for e in json.load(open(a.subset))["frames"]}; view = {}
    for f in M["frames"]:
        if f["id"] in ids: view.setdefault(f["night"], []).append(f)
    thr = {(r["sensitivity"], r["recognizer"]): r["threshold"] for r in json.load(open(out / "curve.json"))["rows"]}
    tot = lambda r: {"found": sum(x["found"] for x in r.values()), "toys": sum(x["n"] for x in r.values()),
                     "fp_mean": round(sum(x["fp"] for x in r.values()) / len(r), 1), "fp_per_night": {k: x["fp"] for k, x in r.items()}}
    def plus(F, pts): return dict(F, nights={k: dict(v, points=v["points"] + [dict(p, score=1.0) for p in pts.get(k, [])]) for k, v in F["nights"].items()})
    rows = []
    for pair in a.pair:
        sens, rec = pair.split(":")
        kept = filtered(json.load(open(out / "out" / f"floor_{sens}.json")), index, json.load(open(out / f"verdicts_{rec}.json"))["verdicts"], thr[(sens, rec)])
        for d in a.detector:
            name, rest = d.split("=", 1); path, t = rest.rsplit("@", 1); D = json.load(open(path)); t = float(t)
            r = {"sensitivity": sens, "recognizer": rec, "threshold": thr[(sens, rec)], "detector": name, "detector_threshold": t,
                 "full": tot(tb.score_arm(plus(kept, tb.placed_points(D, t)), 0.0, ctx)),
                 "subset": tot(tb.score_arm(plus(kept, tb.placed_points(tb.subset_arm(D, ids), t)), 0.0, ctx, view=view))}
            rows.append(r); s, f = r["subset"], r["full"]
            print(f"{sens} + {rec} ({r['threshold']}) | {name} ({t}) | subset {s['found']}/{s['toys']}, {s['fp_mean']} a night | all {f['found']}/{f['toys']}, {f['fp_mean']}")
    json.dump({"post_hoc": True, "rows": rows}, open(out / "union.json", "w"), indent=1)


def curve_md(rows):
    """the curve as a table: each sensitivity x recognizer, found and false positives a night, subset and all frames"""
    L = ["| sensitivity | recognizer | threshold (s70_t1) | candidates a night | subset found | subset FP a night | all found | all FP a night | five wall-clock nights found | FP a night |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        s, f, w = r.get("subset"), r["full"], r["five_wall_clock"]
        L.append(f"| {r['sensitivity']} | {r['recognizer']} | {r['threshold'] if r['threshold'] is not None else '-'} | {r['candidates_a_night']} | "
                 + (f"{s['found']}/{s['toys']} ({100 * s['found'] / max(s['toys'], 1):.0f}%) | {s['fp_mean']} | " if s else "- | - | ")
                 + f"{f['found']}/{f['toys']} ({100 * f['found'] / max(f['toys'], 1):.0f}%) | {f['fp_mean']} | {w['found']}/{w['toys']} | {w['fp_mean']} |")
    return "\n".join(L) + "\n"


def main(argv=None):
    from baselines import recognizers as rc
    ap = argparse.ArgumentParser(); sp = ap.add_subparsers(dest="cmd", required=True)
    c = sp.add_parser("crops"); c.add_argument("out"); c.add_argument("floor", nargs="+")
    v = sp.add_parser("verify"); v.add_argument("out"); v.add_argument("--model", required=True, choices=list(rc.RECOGNIZERS)); v.add_argument("--variant", default=None)
    s = sp.add_parser("score"); s.add_argument("manifest"); s.add_argument("out"); s.add_argument("--floor", nargs="+", required=True)
    s.add_argument("--verdicts", nargs="*", default=[]); s.add_argument("--subset", default=None)
    u = sp.add_parser("union"); u.add_argument("manifest"); u.add_argument("out"); u.add_argument("--pair", nargs="+", required=True)
    u.add_argument("--detector", nargs="+", required=True); u.add_argument("--subset", required=True)
    a = ap.parse_args(argv)
    {"crops": cmd_crops, "verify": cmd_verify, "score": cmd_score, "union": cmd_union}[a.cmd](a)


if __name__ == "__main__":
    main()
