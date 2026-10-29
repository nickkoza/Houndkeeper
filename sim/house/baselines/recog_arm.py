#!/usr/bin/env python3
"""The local recognizers as detectors ([En1]): each recognizer (recognizers.RECOGNIZERS) on the frame set, the same
frames as the first arms, its every box at or above recognizers.FLOOR_SCORE kept with its score, written in owl_arm's
form so table.py places and scores it the same way ([o0E]) and chooses its threshold on s70_t1 alone. Records the time
per frame (median, after a warm-up frame), the load time and the peak GPU memory. --nights limits the frames to some
nights (s70_t1: choosing a recognizer's variant, its phrase or task, before the full run).
Usage: recog_arm.py <manifest.json> <out.json> --model NAME [--variant V] [--nights s70_t1 ...]   (dimos venv python; the GPU)"""
import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent; HOUSE = HERE.parent
sys.path.insert(0, str(HOUSE)); sys.path.insert(0, str(HOUSE.parents[1] / "src"))


def main(argv=None):
    import cv2, numpy as np, torch
    from baselines import prompts as pr, recognizers as rc
    ap = argparse.ArgumentParser(); ap.add_argument("manifest"); ap.add_argument("out")
    ap.add_argument("--model", required=True, choices=list(rc.RECOGNIZERS)); ap.add_argument("--variant", default=None)
    ap.add_argument("--nights", nargs="*", default=None)
    a = ap.parse_args(argv); M = json.load(open(a.manifest))
    frames = [f for f in M["frames"] if not a.nights or f["night"] in a.nights]
    rec = rc.make(a.model, a.variant)
    torch.cuda.reset_peak_memory_stats(); t0 = time.time(); rec.load(); t_load = time.time() - t0
    out, times = [], []
    for k, f in enumerate(frames):
        rgb = cv2.cvtColor(cv2.imread(f["png"]), cv2.COLOR_BGR2RGB)
        if k == 0: rec.detect(rgb, f["ts"])                       # warm-up (kernels, compile), not timed
        ds, dt = rc.timed(rec.detect, rgb, f["ts"]); times.append(dt)
        out.append({"frame": f, "detections": ds})
        if k % 50 == 0: print(f"{k} frames, {dt * 1000:.0f} ms, {len(ds)} boxes", flush=True)
    name = rec.name + (f"/{a.variant}" if a.variant and a.model.startswith("moondream") else "")
    res = {"arm": name, "recognizer": a.model, "variant": a.variant, "model": rec.model,
           "prompts": pr.PROMPTS if a.model.startswith(("owlv2", "yoloe_prompt")) else [getattr(rec, "phrase", a.variant or "prompt-free")],
           "floor_score": rc.FLOOR_SCORE, "load_s": round(t_load, 1),
           "ms_per_frame_median": round(float(np.median(times)) * 1000, 1), "ms_per_frame_mean": round(float(np.mean(times)) * 1000, 1),
           **rc.gpu_stats(), "frames": out}
    json.dump(res, open(a.out, "w"))
    print(f"{name} ({rec.model}): {len(out)} frames, {res['ms_per_frame_median']} ms a frame (median), "
          f"GPU peak {res.get('gpu_peak_alloc_mb')} MB allocated / {res.get('gpu_peak_reserved_mb')} MB reserved; load {res['load_s']} s")


if __name__ == "__main__":
    main()
