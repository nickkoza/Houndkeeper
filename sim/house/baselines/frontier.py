#!/usr/bin/env python3
"""Arm 3, the frontier model ([WD9], [L9V], [DZr]), prepared here and run by Claude Code agents on the project's
subscription: nothing in this file calls a model. The subscription's quota is not free, so the frontier sees a subset.

prepare: from the full frame set, the frontier set (SUBSET frames, a subset of the full set): stratified toward hard
frames (a hard-tier toy in view, a toy on a dark or patterned hard floor, a toy at the image's edge: partly in view),
with some easy ones and some with nothing on the floor in view, spread over the nights. Strata come from the sim's
truth (sim only: which frames to ask about, never what the model is told). A pilot batch of PILOT frames covering every
night and floor goes first; the rest in batches of about BATCH. Each frame is written as a JPEG (quality 85) scaled to
at most LONG_SIDE px on its long side, its scale recorded. The fixed instruction (prompts.FRONTIER) has the sent size.
An agent reads each frame of its batch and writes answers/batch_NN.json:
{"model": ..., "frames": {"<frame jpg path>": [{"label": ..., "box": [x0, y0, x1, y1]}, ...]}}.
load: those answers turned into the same detections file the local detector writes (owl_arm.py), boxes mapped back to
the full frame's pixels by the recorded scale, each scored 1.0 (the model gives no score), for the shared placement
and scorer ([o0E]); plus the call log [6hv] asks for (when, model, frames). A frame no answer covers is counted.
Usage: frontier.py prepare <manifest.json> <dir>
       frontier.py load <dir> <out.json>          (dir: prepare's; answers in <dir>/answers)"""
import argparse
import json
import re
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent; HOUSE = HERE.parent
sys.path.insert(0, str(HOUSE)); sys.path.insert(0, str(HOUSE.parents[1] / "src"))

PILOT, BATCH, SUBSET, LONG_SIDE, JPEG_Q = 20, 20, 70, 768, 85
SHARES = {"hard": 0.6, "easy": 0.25, "empty": 0.15}      # of the frontier set, by stratum
DARK_OR_PATTERNED = {70, 71, 96}   # houses whose hard floor is the slate tile (70), a patterned tile (71) or a dark
                                   # chequered tile (96), as their frames show
EDGE_PX = 40                       # a toy this near the image's edge is only partly in view


def batches(frames, size=BATCH):
    """the frames cut into batches of at most `size`, as near equal as they come, in order"""
    k = max(1, -(-len(frames) // size)); step = len(frames) / k
    return [frames[round(i * step):round((i + 1) * step)] for i in range(k)]


def stratum(frame, toys, to_true, rmax=2.5):
    """scoring side only: 'hard', 'easy' or 'empty' for a frame, from the toys it sees (within rmax) and why"""
    from baselines import place as pl
    seen, why = [], set()
    for t in toys:
        for x, y in [(t["x"], t["y"])] + list(t.get("also", [])):
            uv = pl.sees(frame, to_true.inverse(x, y, frame["ts"]), rmax)
            if uv is None: continue
            seen.append(t["id"])
            if t["tier"] == "hard": why.add("hard tier")
            if t["floor"] == "hard" and frame["seed"] in DARK_OR_PATTERNED: why.add("dark or patterned floor")
            if min(uv[0], uv[1], frame["width"] - uv[0], frame["height"] - uv[1]) < EDGE_PX: why.add("at the edge")
            break
    return ("empty" if not seen else "hard" if why else "easy"), sorted(set(seen)), sorted(why)


def choose(frames, strata, n=SUBSET, shares=SHARES, seed=0):
    """the frontier set: each stratum's share of n, taken round-robin over the nights (seeded, so it repeats); a
    stratum short of its share leaves the rest to the others, hard first"""
    import random
    rng = random.Random(seed); by = {}
    for f in frames: by.setdefault(strata[f["id"]][0], {}).setdefault(f["night"], []).append(f)
    for s in by.values():
        for v in s.values(): rng.shuffle(v)
    want = {s: round(n * p) for s, p in shares.items()}; out = []
    def take(s, k):
        got = []; pools = by.get(s, {})
        while len(got) < k and any(pools.values()):
            for night in sorted(pools):
                if pools[night] and len(got) < k: got.append(pools[night].pop())
        return got
    for s in ("hard", "easy", "empty"): out += take(s, want[s])
    for s in ("hard", "easy", "empty"): out += take(s, n - len(out))
    return sorted(out, key=lambda f: (f["night"], f["ts"]))


def pilot(chosen, strata, k=PILOT):
    """k of the chosen frames covering every night and every floor kind seen: round-robin over (night, stratum)"""
    groups = {}
    for f in chosen: groups.setdefault((f["night"], strata[f["id"]][0]), []).append(f)
    out = []
    while len(out) < k and any(groups.values()):
        for key in sorted(groups):
            if groups[key] and len(out) < k: out.append(groups[key].pop(len(groups[key]) // 2))
    return sorted(out, key=lambda f: (f["night"], f["ts"]))


def write_jpeg(png, jpg, long_side=LONG_SIDE, q=JPEG_Q):
    """the frame as a JPEG no longer than long_side px; its scale (sent px per full-frame px)"""
    import cv2
    img = cv2.imread(png); s = min(1.0, long_side / max(img.shape[:2]))
    if s < 1.0: img = cv2.resize(img, (round(img.shape[1] * s), round(img.shape[0] * s)), interpolation=cv2.INTER_AREA)
    Path(jpg).parent.mkdir(parents=True, exist_ok=True); cv2.imwrite(str(jpg), img, [cv2.IMWRITE_JPEG_QUALITY, q])
    return s, img.shape[1], img.shape[0]


def prepare(manifest, d, ctx=None):
    """the frontier set, its pilot and batches, the JPEGs and the prompt under d; ctx: table.context() (truth, scoring
    side) for the strata"""
    from baselines import prompts as pr
    if ctx is None:
        from baselines import table; ctx = table.context()
    M = json.load(open(manifest)); d = Path(d); (d / "answers").mkdir(parents=True, exist_ok=True)
    strata = {f["id"]: stratum(f, ctx[f["night"]]["toys"], ctx[f["night"]]["to_true"]) for f in M["frames"]}
    chosen = choose(M["frames"], strata, n=SUBSET); pil = pilot(chosen, strata, PILOT); ids = {f["id"] for f in pil}
    rest = [f for f in chosen if f["id"] not in ids]
    entries, sizes = [], set()
    for f in chosen:
        jpg = d / "frames" / f["night"] / (Path(f["png"]).stem + ".jpg")
        s, w, h = write_jpeg(f["png"], jpg); sizes.add((w, h))
        entries.append({"id": f["id"], "night": f["night"], "png": f["png"], "jpg": str(jpg), "scale": s, "sent_size": [w, h],
                        "stratum": strata[f["id"]][0], "toys_in_view": strata[f["id"]][1], "why": strata[f["id"]][2],
                        "pilot": f["id"] in ids})
    if len(sizes) != 1: raise SystemExit(f"frames sent at several sizes {sizes}: one prompt per size needed")
    w, h = sizes.pop(); (d / "prompt.txt").write_text(pr.FRONTIER.format(width=w, height=h) + "\n")
    jpg_of = {e["id"]: e["jpg"] for e in entries}; files = []
    for i, b in enumerate([pil] + batches(rest)):
        p = d / f"batch_{i:02d}.txt"; p.write_text("".join(jpg_of[f["id"]] + "\n" for f in b)); files.append(str(p))
    json.dump({"manifest": str(manifest), "frames": entries, "batches": files, "pilot_batch": files[0],
               "strata": {s: sum(e["stratum"] == s for e in entries) for s in SHARES}}, open(d / "frontier_set.json", "w"), indent=1)
    print(f"frontier set {len(entries)} frames ({ {s: sum(e['stratum'] == s for e in entries) for s in SHARES} }), pilot "
          f"{len(pil)} in {files[0]}, then {len(files) - 1} batches; sent at {w} x {h} (scale {entries[0]['scale']:.3f}); "
          f"prompt {d / 'prompt.txt'}; answers to {d / 'answers'}/batch_NN.json")
    return files


def parse_answer(v, scale=1.0):
    """one frame's answer: a list of {label, box} in the full frame's pixels (the sent box / scale), from a list or
    from text holding one (JSON only was asked; a code fence or a sentence around it is forgiven, a malformed item is
    dropped and counted)"""
    if isinstance(v, str):
        m = re.search(r"\[.*\]", v, re.S)
        try: v = json.loads(m.group(0)) if m else []
        except json.JSONDecodeError: return [], 1
    good, bad = [], 0
    for it in v if isinstance(v, list) else []:
        try:
            b = [float(x) / scale for x in it["box"]]
            if len(b) != 4: raise ValueError
            good.append({"label": str(it.get("label", "")), "box": [min(b[0], b[2]), min(b[1], b[3]), max(b[0], b[2]), max(b[1], b[3])], "score": 1.0})
        except (KeyError, TypeError, ValueError): bad += 1
    return good, bad


def load(d, out):
    d = Path(d); S = json.load(open(d / "frontier_set.json")); M = json.load(open(S["manifest"]))
    full = {f["id"]: f for f in M["frames"]}; by = {}
    for e in S["frames"]:
        for k in (e["jpg"], Path(e["jpg"]).name, e["id"], e["png"], Path(e["png"]).name): by[k] = e
    got, bad, log = {}, 0, []
    for p in sorted((d / "answers").glob("batch_*.json")):
        A = json.load(open(p)); fr = A.get("frames", A)
        log.append({"batch": p.name, "written": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(p.stat().st_mtime)),
                    "model": A.get("model", "unknown"), "frames": len(fr), "cost": "subscription quota (no API charge)"})
        for k, v in fr.items():
            e = by.get(k) or by.get(Path(k).name)
            if e is None: bad += 1; continue
            dets, b = parse_answer(v, e["scale"]); bad += b; got[e["id"]] = dets
    frames = [{"frame": full[e["id"]], "detections": got[e["id"]]} for e in S["frames"] if e["id"] in got]
    res = {"arm": "frontier", "model": sorted({l["model"] for l in log}), "frames": frames,
           "subset": [e["id"] for e in S["frames"]], "missing_frames": len(S["frames"]) - len(frames), "malformed": bad, "call_log": log}
    json.dump(res, open(out, "w"))
    print(f"{len(frames)} frames answered, {res['missing_frames']} without an answer, {bad} malformed items; {out}")
    return res


def main(argv=None):
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("prepare"); p.add_argument("manifest"); p.add_argument("dir")
    l = sub.add_parser("load"); l.add_argument("dir"); l.add_argument("out")
    a = ap.parse_args(argv)
    if a.cmd == "prepare": prepare(a.manifest, a.dir)
    else: load(a.dir, a.out)


if __name__ == "__main__":
    main()
