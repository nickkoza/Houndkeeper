#!/usr/bin/env python3
"""Screen real footage for people before any frame of it is used: kids stay out of visual data (standing rule).
Runs OWLv2 (local, in DimOS) on every Nth frame of each clip for people and body parts, at a deliberately low cut-off,
and marks a margin either side of every hit as unusable. Writes <out>/<clip>.json: the hits (time, score, label) and the
clear stretches. Nothing is shown or saved as an image: flagged frames are never looked at, only skipped.
Usage: screen_people.py <out dir> <clip>...   (dimos venv python)"""
import json, subprocess, sys
from fractions import Fraction
from pathlib import Path
import numpy as np

QUERIES = ["a person", "a child", "a human face", "a human hand", "a human arm", "a human leg", "a foot"]
THR = 0.10          # low on purpose: a false hit costs a second of footage, a miss puts a person in the data
EVERY = 3           # frames: 10 a second at 30 fps
MARGIN_S = 1.0      # unusable either side of a hit
W, H = 960, 540     # frames scaled to this for the detector (OWLv2 looks at 960 px)


def frames(path, every):
    """(index, RGB frame at W x H) for every `every`-th frame, decoded by ffmpeg (no audio)"""
    fps = float(Fraction(subprocess.check_output(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                                        "stream=r_frame_rate", "-of", "csv=p=0", str(path)], text=True).strip()))
    p = subprocess.Popen(["ffmpeg", "-v", "error", "-i", str(path), "-an", "-vf", f"select=not(mod(n\\,{every})),scale={W}:{H}",
                          "-vsync", "0", "-f", "rawvideo", "-pix_fmt", "rgb24", "-"], stdout=subprocess.PIPE)
    k = 0
    while True:
        buf = p.stdout.read(W * H * 3)
        if len(buf) < W * H * 3: break
        yield k * every / fps, np.frombuffer(buf, np.uint8).reshape(H, W, 3); k += 1
    p.wait()


def people_in(im):
    """the detector's hits for people and body parts in one RGB image: [(box, score, label)]; empty means clear"""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm")); import wrist_refine as wr
    return wr.owl_boxes(im, QUERIES, thr=THR)


def screen(path):
    hits, t_end = [], 0.0
    for t, im in frames(path, EVERY):
        t_end = t
        for box, score, label in people_in(im):
            hits.append({"t": round(t, 2), "score": round(score, 3), "label": label, "box": [round(v) for v in box]})
    bad = sorted({round(h["t"], 2) for h in hits})
    clear, start = [], 0.0
    for t in bad:                                         # the stretches at least MARGIN_S from every hit
        if t - MARGIN_S > start + 0.5: clear.append([round(start, 2), round(t - MARGIN_S, 2)])
        start = max(start, t + MARGIN_S)
    if t_end > start + 0.5: clear.append([round(start, 2), round(t_end, 2)])
    return {"clip": path.name, "duration_s": round(t_end, 2), "queries": QUERIES, "threshold": THR, "every": EVERY,
            "margin_s": MARGIN_S, "hits": hits, "clear": clear, "clear_s": round(sum(b - a for a, b in clear), 1)}


def main():
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "houndkeeper"))
    import gpu_share; gpu_share.share()                  # shared with other standalone jobs; a night goes alone ([jlr])
    out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
    for c in sys.argv[2:]:
        r = screen(Path(c)); json.dump(r, open(out / f"{Path(c).stem}.json", "w"), indent=1)
        labels = {}
        for h in r["hits"]: labels[h["label"]] = labels.get(h["label"], 0) + 1
        print(f"{r['clip']}: {r['duration_s']} s, {len(r['hits'])} hits {labels}, clear {r['clear_s']} s in {len(r['clear'])} stretches")


if __name__ == "__main__":
    main()
