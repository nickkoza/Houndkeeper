#!/usr/bin/env python3
"""The frame set ([8VL], [WD9]): from each night's head-camera stream, PER_NIGHT frames evenly spaced over the frames
that have a pose, the same for every arm. Each saved as the camera delivered it (the fisheye frame, PNG) with a
manifest: the night, its time on the recording's clock, the camera's pose then (world <- optical, odometry's frame:
the pose looked up the robot's stored lag earlier on that clock, [gJq], [55G]; the head camera's calibrated mount, as
the live tidy places its frames), the lens (K, D, equidistant: the recording's camera_info) and the image size.
Sim recordings only ([6hv]).
Usage: frames.py <out dir> [--per-night 45]   (dimos venv python)"""
import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent; HOUSE = HERE.parent
sys.path.insert(0, str(HOUSE)); sys.path.insert(0, str(HOUSE.parents[1] / "src"))

import numpy as np

PER_NIGHT = 45


def even_pick(n_valid, k):
    """k indices spread evenly over n_valid (all of them when there are no more than k)"""
    if n_valid <= k: return list(range(n_valid))
    return sorted({int(round(i)) for i in np.linspace(0, n_valid - 1, k)})


def head_camera(st, run_dir, db):
    """(pose(t) -> world<-base, T_base<-optical with the calibrated mount, lag s, clock, camera_info) for a recording"""
    import confirm_stream as cs, floor_mosaic2 as fm
    from houndkeeper import calibrate as cal
    pose, T_bo = cs.poses(st)
    mp = Path(run_dir) / "head_mount.json"
    if mp.exists():
        Rm, off = cal.mount(json.loads(mp.read_text())); T_bo = T_bo.copy()
        T_bo[:3, :3] = T_bo[:3, :3] @ Rm; T_bo[:3, 3] = T_bo[:3, 3] + off
    clock = fm.clock_of(db, "auto")
    lag = fm.frame_lags(st, "robot", ["color_image"], fm.LAG_FILE, check=False, clock=clock)["color_image"][0]
    ci = next(iter(st.stream("camera_info"))).data
    return pose, T_bo, lag, clock, ci


def sample_night(n, out, per_night=PER_NIGHT):
    """the night's frames saved under out/<night>/<ts>.png; their manifest entries"""
    import cv2
    from dimos.memory.store.sqlite import SqliteStore
    import floor_mosaic2 as fm
    st = SqliteStore(path=n["db"]); st.start()
    pose, T_bo, lag, clock, ci = head_camera(st, n["run_dir"], n["db"])
    valid = [k for k, o in enumerate(st.stream("color_image")) if pose(o.ts - lag) is not None]
    pick = {valid[i] for i in even_pick(len(valid), per_night)}
    d = Path(out) / n["id"]; d.mkdir(parents=True, exist_ok=True); entries = []
    K = np.array(ci.K, float).reshape(3, 3).tolist(); D = [float(v) for v in (ci.D or [])]
    for k, o in enumerate(st.stream("color_image")):
        if k not in pick: continue
        img = fm.arr(o.data); T = pose(o.ts - lag) @ T_bo
        p = d / f"{o.ts:.3f}.png"; cv2.imwrite(str(p), cv2.cvtColor(np.ascontiguousarray(img), cv2.COLOR_RGB2BGR))
        entries.append({"id": f"{n['id']}/{o.ts:.3f}", "night": n["id"], "seed": n["seed"], "ts": float(o.ts), "index": k,
                        "png": str(p), "width": int(img.shape[1]), "height": int(img.shape[0]), "K": K, "D": D,
                        "model": ci.distortion_model or "", "T_world_cam": T.tolist(), "lag_s": float(lag), "clock": clock,
                        "frame": "odometry"})
    st.stop()
    return entries, len(valid)


def main(argv=None):
    from baselines import nights as nn
    ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("--per-night", type=int, default=PER_NIGHT)
    a = ap.parse_args(argv); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    allf, info = [], {}
    for n in nn.night_set():
        e, nv = sample_night(n, out, a.per_night); allf += e
        info[n["id"]] = {"frames": len(e), "frames_with_pose": nv, "db": n["db"], "lag_s": e[0]["lag_s"] if e else None,
                         "clock": e[0]["clock"] if e else None}
        print(f"{n['id']}: {len(e)} of {nv} frames with a pose ({e[0]['clock']} clock, lag {e[0]['lag_s']:.3f} s)", flush=True)
    json.dump({"per_night": a.per_night, "nights": info, "frames": allf}, open(out / "manifest.json", "w"), indent=1)
    print(f"{len(allf)} frames; manifest {out / 'manifest.json'}")


if __name__ == "__main__":
    main()
