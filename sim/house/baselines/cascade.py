#!/usr/bin/env python3
"""The near-floor grid proposes, a light local check on each candidate trims ([bFx], [3hK], [Btx]): the floor
comparison's saved replays (floor_diag.py replay, any grid) as stage 1, and stage 2 a check of each candidate on crops
of the frames that saw it (recheck.py's crops and verify) or a cheap multi-view score; each check's threshold chosen on
tuning nights by recall first ([Btx]: the strictest that keeps found at its floor), then judged on fresh nights.

crops:    every candidate (confirmed or unsure, never floor-sized) of a replay directory's nights, cropped from the
          night's head-camera stream as recheck.night_crops does, the night's baselines (head mount) from --runs; plus
          each candidate's own features (its layer, footprint and cells). Nothing here reads the truth.
verify:   one recognizer on every crop (recheck.cmd_verify).
mv:       the two multi-view scores, no model: mv_votes, the highest share over the candidate's cells of the views
          that saw the cell and found it changed (the comparison's own hit / seen, its layer's grid); mv_contrast, the
          lowest over the candidate's crops of the Lab colour difference between the crop's centre (the candidate's
          footprint) and the ring around it, 0 with fewer than MV_MIN_VIEWS crops (a change seen from one side only).
score:    each check's candidate score (a recognizer's best verdict over the crops; a candidate no frame saw kept) swept
          through the shared placement and scorer (table.score_arm, floor_diag.summarise); --choose picks each check's
          threshold, the strictest keeping found at least --min-found; --apply scores at given thresholds (fresh nights).
Usage: cascade.py crops <out> <replay dir> --runs <baselines dir>
       cascade.py verify <out> --model NAME
       cascade.py detect <out> --model NAME          (every box kept; scored as NAME@growG)
       cascade.py mv <out> <replay dir>
       cascade.py score <out> <replay dir> --checks NAME ... (--choose --min-found 45 | --apply NAME=THR ...)
(dimos venv python; verify on the GPU)"""
import argparse
import glob
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent; HOUSE = HERE.parent
sys.path.insert(0, str(HOUSE)); sys.path.insert(0, str(HOUSE.parents[1] / "src"))

import numpy as np

SENS = "loose1"               # the replays' file label (floor_diag replay's sensitivity argument)
RES, HX, HY = 0.01, 8.4, 4.9  # the comparison's 1 cm grid (floor_diag)
KEEP = ("confirmed", "unsure")
MV_MIN_VIEWS = 2              # mv_contrast: a candidate seen in fewer crops scores 0
RING = 2.0                    # mv_contrast: the ring reaches this many footprint radii
FOOT_CM = (1.5, 10.0)         # mv_contrast: the footprint's radius (cm) clipped to this
KEPT = 1e9                    # the score of a candidate no frame saw (always kept: the robot stops for it)


def decisions(d):
    """{night: [(decision, its index)]} of a replay directory, the candidates only"""
    out = {}
    from baselines import floor_diag as fd
    for p in sorted(glob.glob(f"{d}/s*_t*_{SENS}.json")):
        D = fd.reportable(json.load(open(p)), p)                # [LQs]: never from a float16 cache
        out[D["night"]] = [(r, i) for i, r in enumerate(D["decisions"]) if r["decision"] in KEEP and r.get("kind") != "floor_change"]
    return out


def pick_views_by_time(cands, poses, w, h, K, D, model):
    """recheck.pick_views with the views ordered by how near in time they are to the candidate's decision, not by range
    (cands: (x, y, ts)): odometry drifts between the moment the comparison placed the candidate and a view taken long
    before or after, and the crop then misses the thing (the cascade round, 12 Oct 2026, detection-baselines [80w])"""
    from baselines import place as pl, recheck as rk
    out = []
    for x, y, t0 in cands:
        seen = []
        for i, (k, ts, T) in enumerate(poses):
            C = T[:3, 3]; r = math.hypot(x - C[0], y - C[1])
            if not rk.RANGE_M[0] <= r <= rk.RANGE_M[1]: continue
            uv = pl.project((x, y), T, K, D, model)
            if uv is None or not (rk.MARGIN_PX <= uv[0] < w - rk.MARGIN_PX and rk.MARGIN_PX <= uv[1] < h - rk.MARGIN_PX): continue
            seen.append((abs(ts - t0), r, i, uv))
        chosen = []
        for _, r, i, uv in sorted(seen):
            if all(abs(poses[i][1] - poses[j][1]) >= rk.SEP_S for _, j, _ in chosen): chosen.append((r, i, uv))
            if len(chosen) == rk.VIEWS: break
        out.append([(i, uv[0], uv[1], r) for r, i, uv in chosen])
    return out


def cmd_crops(a):
    from baselines import floor_diag as fd, recheck as rk
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    if a.by_time: rk.pick_views = pick_views_by_time
    nights = {n["id"]: n for n in fd.nights_hr()}
    index = {"views": rk.VIEWS, "range_m": rk.RANGE_M, "sep_s": rk.SEP_S, "span_m": rk.SPAN_M, "every": rk.EVERY,
             "grid_m": rk.GRID_M, "by_time": a.by_time, "replays": str(a.replays), "runs": a.runs, "nights": {}, "features": {}}
    for nid, ds in decisions(a.replays).items():
        n = dict(nights[nid], run_dir=str(Path(a.runs) / Path(nights[nid]["run_dir"]).name))
        cands, feats = {}, {}
        for r, _ in ds:
            k = rk.key(nid, r["x_robot"], r["y_robot"])
            if not a.by_time: cands[k] = (r["x_robot"], r["y_robot"])
            elif k not in cands or r["ts"] < cands[k][2]: cands[k] = (r["x_robot"], r["y_robot"], r["ts"])   # the first decision
            f = feats.setdefault(k, {"area_cm2": 0.0, "layers": []})
            f["area_cm2"] = max(f["area_cm2"], float(r.get("area_cm2") or 0.0)); f["layers"] = sorted(set(f["layers"]) | {r.get("layer", "1cm")})
        t0 = time.time(); res, npose = rk.night_crops(n, cands, out)
        index["nights"][nid] = res; index["features"][nid] = feats
        print(f"{nid}: {len(cands)} candidates, {sum(len(v) for v in res.values())} crops ({sum(not v for v in res.values())} "
              f"with no view), {npose} posed frames; {time.time() - t0:.0f} s", flush=True)
    json.dump(index, open(out / "crops.json", "w"))


def cmd_verify(a):
    from baselines import recheck as rk
    rk.cmd_verify(argparse.Namespace(out=a.out, model=a.model, variant=None))


def cmd_detect(a):
    """every detection of a recognizer on every crop, kept whole (boxes, scores, labels), so a check's acceptance rule
    (how far from the candidate's point a box may lie, GROWS) is chosen after one model run"""
    import cv2, torch
    from baselines import recognizers as rc
    out = Path(a.out); index = json.load(open(out / "crops.json")); rec = rc.make(a.model); rec.load(); res, times = {}, []
    crops = [c for night in index["nights"].values() for cs in night.values() for c in cs]
    for k, c in enumerate(crops):
        img = cv2.cvtColor(cv2.imread(c["png"]), cv2.COLOR_BGR2RGB)
        if k == 0: rec.detect(img)
        ds, dt = rc.timed(rec.detect, img); times.append(dt); res[c["id"]] = ds
    json.dump({"recognizer": rec.name, "model": rec.model, "ms_per_crop_median": round(float(np.median(times)) * 1000, 1),
               **rc.gpu_stats(), "detections": res}, open(out / f"detections_{rec.name}.json", "w"))
    print(f"{rec.name}: {len(crops)} crops, {np.median(times) * 1000:.1f} ms a crop (median)")


def votes(r, Z, Zn):
    """the highest hit / seen over a decision's cells, on its layer's grid"""
    if r.get("layer") == "near" and Zn is not None: hit, seen, res = Zn["hit"], Zn["seen"], float(Zn["res"])
    else: hit, seen, res = Z["hit"], Z["seen"], RES
    best = 0.0
    for x, y in r.get("cells", []):
        i, j = int((HY - y) / res), int((x + HX) / res)
        if 0 <= i < seen.shape[0] and 0 <= j < seen.shape[1] and seen[i, j] > 0: best = max(best, float(hit[i, j]) / float(seen[i, j]))
    return best


def contrast(crop, uv, r_px):
    """the Lab distance between the mean colour within r_px of uv and the mean of the ring out to RING * r_px"""
    import cv2
    lab = cv2.cvtColor(crop, cv2.COLOR_RGB2LAB).astype(np.float32); lab[..., 0] *= 100 / 255; lab[..., 1:] -= 128
    yy, xx = np.mgrid[:crop.shape[0], :crop.shape[1]]; d = np.hypot(xx - uv[0], yy - uv[1])
    c, ring = d <= r_px, (d > r_px) & (d <= RING * r_px)
    if c.sum() < 4 or ring.sum() < 4: return 0.0
    return float(np.linalg.norm(lab[c].mean(0) - lab[ring].mean(0)))


def cmd_mv(a):
    import cv2
    from baselines import recheck as rk, recognizers as rc
    out = Path(a.out); index = json.load(open(out / "crops.json"))
    vt, ct, times = {}, {}, []
    for nid, ds in decisions(a.replays).items():
        p = Path(a.replays) / f"{nid}_{SENS}"; Z = np.load(f"{p}.npz")
        Zn = np.load(f"{p}.near.npz") if Path(f"{p}.near.npz").exists() else None
        for r, _ in ds:
            k = rk.key(nid, r["x_robot"], r["y_robot"]); vt[k] = max(vt.get(k, 0.0), votes(r, Z, Zn))
        crops, feats = index["nights"].get(nid, {}), index["features"].get(nid, {})
        for k, cs in crops.items():
            t0 = time.perf_counter(); rad = np.clip(math.sqrt(feats[k]["area_cm2"] / math.pi), *FOOT_CM) / 100; vals = []
            for c in cs:
                img = cv2.cvtColor(cv2.imread(c["png"]), cv2.COLOR_BGR2RGB); side = c["box_full"][2] - c["box_full"][0]
                fx = side / (rk.SPAN_M / max(c["range_m"], 0.1)) if side else 1.0          # the crop's px a metre at its range
                vals.append(contrast(img, c["uv"], rad * fx * rc.CROP_PX / side))
            ct[k] = min(vals) if len(vals) >= MV_MIN_VIEWS else (0.0 if vals else None)
            times.append(time.perf_counter() - t0)
    json.dump({"check": "mv_votes", "scores": vt}, open(out / "scores_mv_votes.json", "w"))
    json.dump({"check": "mv_contrast", "scores": ct, "ms_per_candidate_median": round(float(np.median(times)) * 1000, 2)},
              open(out / "scores_mv_contrast.json", "w"))
    print(f"mv: {len(vt)} candidates' votes, {len(ct)} contrasts, {np.median(times) * 1000:.2f} ms a candidate (median, PNG read included)")


def check_scores(out, name):
    """{key: score or None (no frame saw it)} for a check: the multi-view files, or a recognizer's best verdict"""
    p = Path(out) / f"scores_{name}.json"
    if p.exists(): return json.load(open(p))["scores"]
    index = json.load(open(Path(out) / "crops.json"))
    if "@grow" in name:                                       # NAME@growG: the whole detections, a box counting when,
        from baselines import recognizers as rc               # grown by G of the crop's side, it holds the point
        base, g = name.split("@grow"); D = json.load(open(Path(out) / f"detections_{base}.json"))["detections"]
        cs_ = {c["id"]: c for night in index["nights"].values() for cs in night.values() for c in cs}
        V = {i: {"score": max((d["score"] for d in ds if rc.holds(d["box"], cs_[i]["uv"], rc.CROP_PX, float(g))), default=0.0)}
             for i, ds in D.items()}
    else: V = json.load(open(Path(out) / f"verdicts_{name}.json"))["verdicts"]
    return {k: max((V[c["id"]]["score"] for c in cs if c["id"] in V), default=None)
            for night in index["nights"].values() for k, cs in night.items()}


def arm(replays, scores):
    """the replays' candidates as a floor arm, each scored by the check (KEPT where no frame saw it); scores None: stage
    1 alone, each scored as the floor arm scores its decisions (floor_arm.SCORE, which orders the scorer's merge)"""
    from baselines import floor_arm as fa, recheck as rk
    nights = {}
    for nid, ds in decisions(replays).items():
        pts = []
        for r, i in ds:
            s = fa.SCORE[r["decision"]] if scores is None else scores.get(rk.key(nid, r["x_robot"], r["y_robot"]))
            pts.append({"x": r["x_robot"], "y": r["y_robot"], "ts": r["ts"], "score": KEPT if s is None else float(s),
                        "label": r["decision"], "layer": r.get("layer", "1cm"), "frame": f"decision{i}"})
        nights[nid] = {"points": pts}
    return {"arm": "floor", "sensitivity": SENS, "nights": nights}


def at(A, thr, ctx):
    from baselines import floor_diag as fd, table as tb
    s = fd.summarise(tb.score_arm(A, thr, ctx)); s["kept_a_night"] = round(sum(sum(p["score"] >= thr for p in v["points"]) for v in A["nights"].values()) / len(A["nights"]), 1)
    return s


def choose(curve, min_found):
    """the strictest threshold whose found reaches min_found (none: the loosest, i.e. keep all)"""
    ok = [c for c in curve if c["found"] >= min_found]
    return max(ok, key=lambda c: c["thr"])["thr"] if ok else min(c["thr"] for c in curve)


def cmd_score(a):
    from baselines import curves as cv, floor_diag as fd
    A0 = arm(a.replays, None); ctx = fd.context(set(A0["nights"])); res = {"stage1": at(A0, -1e9, ctx)}
    print(f"stage 1: {res['stage1']['found']}/{res['stage1']['n']} at {res['stage1']['fp_per_night']} a night")
    apply = dict(x.split("=") for x in (a.apply or []))
    for name in a.checks:
        A = arm(a.replays, check_scores(a.out, name)); ths = sorted({p["score"] for v in A["nights"].values() for p in v["points"]} | {-1e9})
        curve = cv.sweep(A, ctx, thresholds=ths); ap = cv.average_precision(curve)
        thr = choose(curve, a.min_found) if a.choose else float(apply[name])
        s = at(A, thr, ctx); s.update(check=name, threshold=thr, ap=round(ap, 3), curve=curve)
        res[name] = s
        print(f"{name:16s} thr {thr:.4g}: found {s['found']}/{s['n']} (main {s['main'][0]}/{s['main'][1]}, bricks {s['tiny'][0]}/{s['tiny'][1]}) "
              f"FP/n {s['fp_per_night']} kept/n {s['kept_a_night']} AP {ap:.3f} | {s['floor']} | {s['fp']}", flush=True)
    json.dump(res, open(Path(a.out) / f"score_{a.label}.json", "w"), indent=1)


def main(argv=None):
    ap = argparse.ArgumentParser(); sp = ap.add_subparsers(dest="cmd", required=True)
    c = sp.add_parser("crops"); c.add_argument("out"); c.add_argument("replays"); c.add_argument("--runs", required=True)
    c.add_argument("--by-time", action="store_true", help="the views nearest in time to the candidate's decision")
    v = sp.add_parser("verify"); v.add_argument("out"); v.add_argument("--model", required=True)
    dt = sp.add_parser("detect"); dt.add_argument("out"); dt.add_argument("--model", required=True)
    m = sp.add_parser("mv"); m.add_argument("out"); m.add_argument("replays")
    s = sp.add_parser("score"); s.add_argument("out"); s.add_argument("replays"); s.add_argument("--checks", nargs="+", required=True)
    s.add_argument("--choose", action="store_true"); s.add_argument("--min-found", type=int, default=45)
    s.add_argument("--apply", nargs="*"); s.add_argument("--label", default="tuning")
    a = ap.parse_args(argv)
    {"crops": cmd_crops, "verify": cmd_verify, "detect": cmd_detect, "mv": cmd_mv, "score": cmd_score}[a.cmd](a)


if __name__ == "__main__":
    main()
