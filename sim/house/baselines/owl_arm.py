#!/usr/bin/env python3
"""Arm 2, local detection ([WD9], [XEi]): OWLv2 through DimOS's wrapper (dimos.perception.detection.detectors.owlv2)
on the frame set, with the fixed prompt set (prompts.PROMPTS). Every box at or above FLOOR_SCORE is kept with its score,
so the threshold is chosen afterwards on one night alone (table.py) and applied to all. Records the time per frame and
the GPU memory. --dense runs it on every 4th frame of each night's stream instead (the floor comparison's own stride),
the frames held in memory only.
Usage: owl_arm.py <manifest.json> <out.json> [--dense]   (dimos venv python; the GPU)"""
import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent; HOUSE = HERE.parent
sys.path.insert(0, str(HOUSE)); sys.path.insert(0, str(HOUSE.parents[1] / "src"))

FLOOR_SCORE = 0.05       # the lowest score kept (the wrapper's own default threshold is 0.1)
DENSE_EVERY = 4          # confirm_stream's --every


def detector():
    from dimos.perception.detection.detectors.owlv2 import Owlv2Detector
    return Owlv2Detector()


def detect(det, rgb, ts, prompts, floor_score=FLOOR_SCORE):
    """one frame's detections: [{label, box, score}]"""
    from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
    r = det.query_detections(Image.from_numpy(rgb, format=ImageFormat.RGB, ts=ts), prompts, threshold=floor_score)
    return [{"label": d.name, "box": [round(float(v), 1) for v in d.bbox], "score": round(float(d.confidence), 4)}
            for d in r.detections]


def dense_frames(manifest):
    """each night's every DENSE_EVERY-th frame with a pose, as manifest entries plus the image (not saved)"""
    from dimos.memory.store.sqlite import SqliteStore
    import floor_mosaic2 as fm
    from baselines import frames as fr, nights as nn
    for n in nn.night_set():
        st = SqliteStore(path=n["db"]); st.start()
        pose, T_bo, lag, clock, ci = fr.head_camera(st, n["run_dir"], n["db"])
        import numpy as np
        K = np.array(ci.K, float).reshape(3, 3).tolist(); D = [float(v) for v in (ci.D or [])]
        for k, o in enumerate(st.stream("color_image")):
            if k % DENSE_EVERY: continue
            P = pose(o.ts - lag)
            if P is None: continue
            img = fm.arr(o.data)
            yield {"id": f"{n['id']}/{o.ts:.3f}", "night": n["id"], "seed": n["seed"], "ts": float(o.ts), "index": k,
                   "width": int(img.shape[1]), "height": int(img.shape[0]), "K": K, "D": D, "model": ci.distortion_model or "",
                   "T_world_cam": (P @ T_bo).tolist(), "lag_s": float(lag), "clock": clock, "frame": "odometry"}, img
        st.stop()


def main(argv=None):
    import cv2, torch
    from baselines import prompts as pr
    ap = argparse.ArgumentParser(); ap.add_argument("manifest"); ap.add_argument("out"); ap.add_argument("--dense", action="store_true")
    a = ap.parse_args(argv); M = json.load(open(a.manifest))
    det = detector(); t_load = time.time(); det._model; det._processor; t_load = time.time() - t_load
    torch.cuda.reset_peak_memory_stats()
    src = dense_frames(M) if a.dense else ((f, cv2.cvtColor(cv2.imread(f["png"]), cv2.COLOR_BGR2RGB)) for f in M["frames"])
    out, times = [], []
    for k, (f, rgb) in enumerate(src):
        if k == 0: detect(det, rgb, f["ts"], pr.PROMPTS)      # warm-up (CUDA kernels), not timed
        torch.cuda.synchronize(); t0 = time.time()
        ds = detect(det, rgb, f["ts"], pr.PROMPTS)
        torch.cuda.synchronize(); times.append(time.time() - t0)
        out.append({"frame": {kk: v for kk, v in f.items()}, "detections": ds})
        if k % 100 == 0: print(f"{k} frames, {times[-1] * 1000:.0f} ms", flush=True)
    import numpy as np
    res = {"arm": "owlv2" + ("_dense" if a.dense else ""), "model": det.config.model_name, "prompts": pr.PROMPTS,
           "floor_score": FLOOR_SCORE, "device": str(det.config.device), "gpu": torch.cuda.get_device_name(0),
           "load_s": round(t_load, 1), "ms_per_frame_median": round(float(np.median(times)) * 1000, 1),
           "ms_per_frame_mean": round(float(np.mean(times)) * 1000, 1),
           "gpu_peak_alloc_mb": round(torch.cuda.max_memory_allocated() / 2 ** 20),
           "gpu_peak_reserved_mb": round(torch.cuda.max_memory_reserved() / 2 ** 20), "frames": out}
    json.dump(res, open(a.out, "w"))
    print(f"{len(out)} frames: {res['ms_per_frame_median']} ms a frame (median), GPU peak {res['gpu_peak_alloc_mb']} MB "
          f"allocated / {res['gpu_peak_reserved_mb']} MB reserved; load {res['load_s']} s")


if __name__ == "__main__":
    main()
