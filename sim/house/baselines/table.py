#!/usr/bin/env python3
"""The tradeoff table ([JQk], [tWE]): every arm's output through the one placement and scorer ([o0E], [oR6]).
A detector's file (owl_arm.py, frontier.py load) has boxes per frame: each kept box placed on the floor (place.py),
the points carried into the sim's frame (scoring only), merged across frames and scored. The floor comparison's file
(floor_arm.py) has points already. A detector with scores gets its threshold chosen on THRESHOLD_NIGHT alone (the
best F1 of toys found against candidates near a toy, ties to the higher threshold) and applied to every night.
Writes table.json and table.md, and crops of the failures (misses, false positives) under <out>/crops.
The local recognizers ([En1], recog_arm.py) are detectors like OWLv2 here; --curve appends the loosened floor
comparison checked by each of them ([3hK], recheck.py score).
Usage: table.py <manifest.json> <out dir> <arm file> [<arm file> ...] [--subset frontier_set.json] [--curve curve.json]
(dimos venv python)"""
import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent; HOUSE = HERE.parent
sys.path.insert(0, str(HOUSE)); sys.path.insert(0, str(HOUSE.parents[1] / "src"))

from baselines import nights as nn, place as pl, prompts as pr, score as sc   # noqa: E402

THRESHOLD_NIGHT = "s70_t1"          # house 70, the main tuning house: the only night a threshold is chosen on
THRESHOLDS = [0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50]
CROP = (160, 120)


def placed_points(arm, thr, rmax=sc.RMAX_M):
    """{night: [points in the odometry's frame]} from an arm's file at score threshold thr"""
    out = {}
    if arm["arm"] == "floor":
        for k, v in arm["nights"].items(): out[k] = [p for p in v["points"] if p["score"] >= thr]
        return out
    for fr in arm["frames"]:
        f = fr["frame"]; out.setdefault(f["night"], [])
        for d in fr["detections"]:
            if d.get("score", 1.0) < thr: continue
            p = pl.place(d, f, rmax=rmax)
            if p is not None: out[f["night"]].append(p)
    return out


def seen_by(frames, xy_true, to_true, rmax=sc.RMAX_M):
    """whether any of these frames sees a sim-frame floor point within rmax"""
    return any(pl.sees(f, to_true.inverse(xy_true[0], xy_true[1], f["ts"]), rmax) is not None for f in frames)


def subset_arm(arm, ids):
    """a detector's file cut to the frames named (the floor comparison's is returned whole: it has no frames)"""
    if arm["arm"] == "floor": return arm
    return dict(arm, frames=[fr for fr in arm["frames"] if fr["frame"]["id"] in ids])


def score_arm(arm, thr, ctx, min_frames=1, view=None):
    """each night's result at threshold thr: points carried into the sim's frame, merged, scored. view ({night: frames}):
    score only on the floor those frames see (a frame subset shared by every arm): the toys any of them sees, and the
    candidates that lie where one of them looks"""
    pts = placed_points(arm, thr); res = {}
    for nid, c in ctx.items():
        P = []
        for p in pts.get(nid, []):
            x, y = c["to_true"](p["x"], p["y"], p["ts"]); P.append(dict(p, x=x, y=y, x_odom=p["x"], y_odom=p["y"]))
        cands = [m for m in sc.merge(P) if m["frames"] >= min_frames]
        toys = c["toys"]
        if view is not None:
            fs = view.get(nid, [])
            toys = [t for t in toys if any(seen_by(fs, xy, c["to_true"]) for xy in [(t["x"], t["y"])] + list(t.get("also", [])))]
            cands = [m for m in cands if seen_by(fs, (m["x"], m["y"]), c["to_true"])]
        res[nid] = sc.score_night(cands, toys, c["stains"])
    return res


def f1(r):
    """F1 of toys found (recall) against candidates near a toy (precision)"""
    rec = r["found"] / max(r["n"], 1); prec = (r["cands"] - r["fp"]) / r["cands"] if r["cands"] else 0.0
    return 0.0 if rec + prec == 0 else 2 * rec * prec / (rec + prec)


def choose_threshold(arm, ctx):
    """the threshold with the best F1 on THRESHOLD_NIGHT alone (ties: the higher), and the sweep it came from"""
    one = {THRESHOLD_NIGHT: ctx[THRESHOLD_NIGHT]}; sweep = []
    for t in THRESHOLDS:
        r = score_arm(arm, t, one)[THRESHOLD_NIGHT]; sweep.append((t, r["found"], r["n"], r["fp"], round(f1(r), 3)))
    best = max(sweep, key=lambda s: (s[4], s[0]))
    return best[0], sweep


def crop(frame, uv, path):
    """a CROP-sized cut of the frame's image around pixel uv, saved"""
    import cv2
    img = cv2.imread(frame["png"]) if frame.get("png") else None
    if img is None: return None
    w, h = CROP; x0 = int(min(max(uv[0] - w / 2, 0), img.shape[1] - w)); y0 = int(min(max(uv[1] - h / 2, 0), img.shape[0] - h))
    c = img[y0:y0 + h, x0:x0 + w].copy(); cv2.circle(c, (int(uv[0] - x0), int(uv[1] - y0)), 10, (0, 0, 255), 1)
    Path(path).parent.mkdir(parents=True, exist_ok=True); cv2.imwrite(str(path), c); return str(path)


def nearest_view(frames, xy_true, to_true, rmax=sc.RMAX_M):
    """the frame of the set seeing a sim-frame floor point closest (within rmax), and its pixel; or (None, None)"""
    best = (None, None, 1e9)
    for f in frames:
        ox, oy = to_true.inverse(xy_true[0], xy_true[1], f["ts"]); uv = pl.project((ox, oy), f["T_world_cam"], f["K"], f.get("D"), f.get("model", ""))
        if uv is None or not (0 <= uv[0] < f["width"] and 0 <= uv[1] < f["height"]): continue
        C = f["T_world_cam"]; r = math.hypot(ox - C[0][3], oy - C[1][3])
        if r < min(best[2], rmax): best = (f, uv, r)
    return best[0], best[1]


def failures(name, res, ctx, frames_by, out, k=4):
    """crops of up to k misses and k false positives per night"""
    paths = []
    for nid, r in res.items():
        for t in [t for t in r["toys"] if not t["found"]][:k]:
            f, uv = nearest_view(frames_by.get(nid, []), (t["x"], t["y"]), ctx[nid]["to_true"])
            if f: paths.append(("miss", nid, t["name"], crop(f, uv, out / "crops" / name / f"{nid}_miss_{t['id']}.png")))
        for i, c in enumerate(sorted(r["fp_cands"], key=lambda c: -c["score"])[:k]):
            m = max(c["members"], key=lambda m: m.get("score", 1.0))
            f = next((x for x in frames_by.get(nid, []) if x["id"] == m.get("frame")), None)
            uv = pl.bottom_centre(m["box"]) if f and m.get("box") else None
            if uv is None: f, uv = nearest_view(frames_by.get(nid, []), (c["x"], c["y"]), ctx[nid]["to_true"])
            if f: paths.append(("false positive", nid, "/".join(c["labels"]), crop(f, uv, out / "crops" / name / f"{nid}_fp{i}.png")))
    return paths


def summary(name, arm, thr, res, sweep=None):
    nights = list(res.values()); N = len(nights)
    s = {"arm": name, "threshold": thr, "found": sum(r["found"] for r in nights), "toys": sum(r["n"] for r in nights),
         "by_tier": sc.tally(nights, "tier"), "by_floor": sc.tally(nights, "floor"),
         "by_named": {("named" if k else "unknown"): v for k, v in sc.tally(nights, "named").items()},
         "fp_per_night": {k: r["fp"] for k, r in res.items()}, "fp_mean": round(sum(r["fp"] for r in nights) / N, 1),
         "fp_on_stain": sum(r["fp_on_stain"] for r in nights), "fp_near_toy": sum(r["fp_near_toy"] for r in nights),
         "per_night_found": {k: f"{r['found']}/{r['n']}" for k, r in res.items()},
         "missed": {k: [t["name"] for t in r["toys"] if not t["found"]] for k, r in res.items()}}
    if sweep: s["threshold_sweep_" + THRESHOLD_NIGHT] = sweep
    if arm["arm"] == "floor":
        v = arm["nights"].values(); s["ms_per_frame"] = sorted(x["ms_per_frame_median"] for x in v)[len(arm["nights"]) // 2]
        s["frames_per_night"] = round(sum(x["frames"] for x in v) / len(arm["nights"]))
        s["s_per_night"] = round(sum(x["wall_s"] for x in v) / len(arm["nights"]), 1)
        s["compute"] = "CPU only (one core per night's replay); the baselines' floor pictures and views, no prompts, no network"
    else:
        fpn = {}
        for fr in arm["frames"]: fpn[fr["frame"]["night"]] = fpn.get(fr["frame"]["night"], 0) + 1
        s["frames_per_night"] = round(sum(fpn.values()) / max(len(fpn), 1))
        if "ms_per_frame_median" in arm:
            s["ms_per_frame"] = arm["ms_per_frame_median"]; s["s_per_night"] = round(arm["ms_per_frame_median"] * s["frames_per_night"] / 1000, 1)
            s["compute"] = (f"GPU ({arm.get('gpu')}), peak {arm.get('gpu_peak_alloc_mb')} MB allocated, {arm.get('gpu_peak_reserved_mb')} MB "
                            f"reserved; {arm.get('model', '')}; {len(arm['prompts'])} prompt(s); weights local, no network")
        else:
            s["compute"] = "hosted model through Claude Code agents (subscription); a network; the prompt"
    return s


def md(rows, crops):
    L = ["| arm | threshold | found | easy | medium | hard | carpet | hard floor | rug | named kinds | unknown kinds | false positives a night (mean; per night) | on a stain | near a toy | ms a frame | frames a night | s a night | compute |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    g = lambda d, k: f"{d[k][0]}/{d[k][1]}" if k in d else "-"
    for s in rows:
        L.append(f"| {s['arm']} | {s['threshold']} | {s['found']}/{s['toys']} | {g(s['by_tier'], 'easy')} | {g(s['by_tier'], 'medium')} | "
                 f"{g(s['by_tier'], 'hard')} | {g(s['by_floor'], 'carpet')} | {g(s['by_floor'], 'hard')} | {g(s['by_floor'], 'rug')} | "
                 f"{g(s['by_named'], 'named')} | {g(s['by_named'], 'unknown')} | {s['fp_mean']} ({', '.join(str(v) for v in s['fp_per_night'].values())}) | "
                 f"{s['fp_on_stain']} | {s['fp_near_toy']} | {s.get('ms_per_frame', '-')} | {s['frames_per_night']} | {s.get('s_per_night', '-')} | {s['compute']} |")
    L.append(""); L.append("Failure examples (crops):")
    for name, ps in crops.items():
        L.append(f"- {name}: " + "; ".join(f"{kind} {nid} {what}: {p}" for kind, nid, what, p in ps[:8]))
    return "\n".join(L) + "\n"


def context():
    """each night's truth (scoring only) and its odometry -> sim frame map"""
    ctx = {}
    for n in nn.night_set():
        toys, stains = nn.truth_toys(n, pr.named)
        ctx[n["id"]] = {"toys": toys, "stains": stains, "to_true": pl.odom_to_true(n["traj"])}
    return ctx


def main(argv=None):
    ap = argparse.ArgumentParser(); ap.add_argument("manifest"); ap.add_argument("out"); ap.add_argument("arms", nargs="+")
    ap.add_argument("--no-crops", action="store_true")
    ap.add_argument("--subset", default=None, help="frontier_set.json (frontier.py prepare): every arm also scored on exactly its frames")
    ap.add_argument("--curve", default=None, help="curve.json (recheck.py score): the loosened floor comparison x local recognizer curve ([3hK]), appended")
    a = ap.parse_args(argv); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    M = json.load(open(a.manifest)); frames_by = {}
    for f in M["frames"]: frames_by.setdefault(f["night"], []).append(f)
    ctx = context(); rows, crops = [], {}
    ids, view = None, None
    if a.subset:
        ids = {e["id"] for e in json.load(open(a.subset))["frames"]}; view = {}
        for f in M["frames"]:
            if f["id"] in ids: view.setdefault(f["night"], []).append(f)
    for path in a.arms:
        arm = json.load(open(path)); name = arm["arm"]
        scored = any(d.get("score", 1.0) < 1.0 for fr in arm.get("frames", []) for d in fr["detections"])
        if arm["arm"] == "floor":
            sens = arm.get("sensitivity", "today"); tag = "" if sens == "today" else f", {sens}"   # several sensitivities ([3hK])
            for nm, thr in ((f"floor (confirmed or unsure{tag})", 0.0), (f"floor (confirmed only{tag})", 1.0)):
                res = score_arm(arm, thr, ctx); rows.append(summary(nm, arm, thr, res))
                if thr == 0.0 and not a.no_crops: crops[nm] = failures("floor" if sens == "today" else f"floor_{sens}", res, ctx, frames_by, out)
                if view is not None: rows.append(summary(nm + " [frontier subset]", arm, thr, score_arm(arm, thr, ctx, view=view)))
            continue
        thr, sweep = choose_threshold(arm, ctx) if scored else (0.0, None)
        if name != "frontier":
            res = score_arm(arm, thr, ctx); rows.append(summary(name, arm, thr, res, sweep))
            if not a.no_crops: crops[name] = failures(name, res, ctx, frames_by, out)
            if scored:
                res2 = score_arm(arm, thr, ctx, min_frames=2); rows.append(summary(name + " (seen in 2+ frames)", arm, thr, res2))
        if view is not None and arm["frames"] and all(fr["frame"].get("png") for fr in arm["frames"][:50]):   # the frame
            sub = subset_arm(arm, ids); res = score_arm(sub, thr, ctx, view=view)                       # set's own only
            rows.append(summary(name + " [frontier subset]", sub, thr, res))
            if name == "frontier" and not a.no_crops: crops[name] = failures(name, res, ctx, frames_by, out)
    json.dump({"rows": rows, "crops": crops, "toys": {k: [{kk: t[kk] for kk in ("id", "name", "tier", "floor", "named", "area_cm2")} for t in c["toys"]] for k, c in ctx.items()}},
              open(out / "table.json", "w"), indent=1)
    text = md(rows, crops)
    if a.curve:
        from baselines import recheck
        text += "\nThe floor comparison at each sensitivity, each candidate checked by a local recognizer ([3hK]):\n\n" + recheck.curve_md(json.load(open(a.curve))["rows"])
    (out / "table.md").write_text(text); print(text)
    for s in rows:
        if "threshold_sweep_" + THRESHOLD_NIGHT in s: print(s["arm"], "sweep on", THRESHOLD_NIGHT, "(t, found, toys, fp, F1):", s["threshold_sweep_" + THRESHOLD_NIGHT])


if __name__ == "__main__":
    main()
