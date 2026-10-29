#!/usr/bin/env python3
"""Look-closer naming step: a local VLM (Moondream2 via DimOS's wrapper, cached locally, GPU) names what is in a
wrist-camera image. Nothing leaves the machine. Usage: look_vlm.py <image> [<image> ...]"""
import os, sys, time
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src")); from houndkeeper import workspace   # noqa: E402  (decision [cy4])
sys.path.insert(0, str(workspace.dimos()))   # the DimOS checkout beside this repo
import numpy as np
from PIL import Image
from dimos.models.vl.moondream import MoondreamVlModel
Q = ("This is a close-up from a robot's wrist camera looking down at a floor. What object is lying on the floor? "
     "Answer with a short noun phrase, or 'nothing' if it is just floor.")
def main(paths):
    t0 = time.time(); vlm = MoondreamVlModel(); out = {}
    for p in paths:
        img = np.ascontiguousarray(np.asarray(Image.open(p).convert("RGB"))[..., ::-1]); t = time.time(); ans = vlm.query(img, Q).strip()   # wrapper expects BGR numpy
        out[p] = ans; print(f"{p}: {ans!r} ({time.time() - t:.1f} s)", flush=True)
    print(f"model load + {len(paths)} queries: {time.time() - t0:.0f} s")
    return out
if __name__ == "__main__": main(sys.argv[1:])
