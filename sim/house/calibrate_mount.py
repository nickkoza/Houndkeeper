#!/usr/bin/env python3
"""The head camera's mount, calibrated from a recording ([KkQ], calibrate.py): the night's head frames, straightened by
the lens and placed by the lidar against the house map, then the small turn of the camera that makes far views of the
floor agree with near ones (pitch) and views from opposite directions agree (yaw, roll). Writes the correction for
everything that places head-camera pixels (floor_mosaic2.py --mount, the confirmer). Frames are placed with the
robot's stored frame lag ([gJq]).
Usage: calibrate_mount.py <memory.db> --house house.npy --out head_mount.json [--every 6] (dimos venv)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import argparse, json, time
from pathlib import Path

import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from floor_mosaic2 import arr
from houndkeeper import calibrate as cal
from houndkeeper.lens import Rectifier
from houndkeeper.localize import Localizer, scans_from
import confirm_stream as cs


def frames_from(db, house, every=6, limit=150, corrections=None, dt="robot", pose_hook=None, lag_file=None, check=True,
                clock="auto", traj=None):
    """the night's head frames, straightened, each with the camera's pose when it was taken: dt "robot" (default;
    "auto" the same) takes the robot's stored frame lag off first ([gJq]; tonight's estimate from its turns only checks
    it), "night" tonight's estimate, a number that lag (s) ([uKM] 2; 80 ms turning at 0.5 rad/s is 2.3 deg, which the
    fit would otherwise read as the mount). clock: the recording's ('auto': from its path file, [55G]), whose stored
    lag is taken. pose_hook wraps the pose lookup; the robot never sets one"""
    st = SqliteStore(path=db); st.start()
    from floor_mosaic2 import frame_lags, LAG_FILE, clock_of
    clk = clock_of(db, clock, traj) if str(dt) in ("robot", "auto") else None
    lag, note = frame_lags(st, dt, ["color_image"], lag_file or LAG_FILE, check, clock=clk)["color_image"]
    print(f"color_image: frame lag {lag:.3f} s, {note}", flush=True)
    ci = next(iter(st.stream("camera_info"))).data; rect = Rectifier.from_camera_info(ci)
    pose, T_bo = cs.poses(st)
    if pose_hook: pose = pose_hook(pose)
    if corrections:                                           # poses from the loop closure over nights
        z = np.load(corrections); loc = Localizer(times=list(z["times"]), Ts=list(z["Ts"]))
    else: loc = Localizer(np.load(house)).run(scans_from(st)) if house else None
    out = []
    for k, o in enumerate(st.stream("color_image")):
        if k % every: continue
        M = pose(o.ts - lag)
        if M is None: continue
        if loc is not None: M = loc.apply(M, o.ts - lag)
        out.append((rect(arr(o.data)), M @ T_bo, rect.K))
        if len(out) >= limit: break
    st.stop(); return out


def main(argv=None, pose_hook=None):
    """the mount from one recording; pose_hook as frames_from's (true_path.py's offline ablation sets it)"""
    ap = argparse.ArgumentParser(); ap.add_argument("db"); ap.add_argument("--house", default=None); ap.add_argument("--out", required=True)
    ap.add_argument("--corrections", default=None); ap.add_argument("--every", type=int, default=6); ap.add_argument("--limit", type=int, default=150)
    ap.add_argument("--dt", default="robot", help="the frames' stamp lag ([gJq]): 'robot' (default; 'auto' the same): the robot's stored lag (--lag-file), tonight's estimate from its turns only checked against it; 'night': tonight's estimate stands; or a number: that lag (s)")
    ap.add_argument("--lag-file", default=None, help="the robot's frame lag (.json from calibrate_lag.py; default data/robot/frame_lag.json)")
    ap.add_argument("--no-lag-check", action="store_true", help="--dt robot: skip tonight's estimate (the check)")
    from floor_mosaic2 import add_clock_args; add_clock_args(ap)
    a = ap.parse_args(argv); t0 = time.time()
    frames = frames_from(a.db, a.house, a.every, a.limit, a.corrections, a.dt, pose_hook, a.lag_file, not a.no_lag_check,
                         a.clock, a.traj)
    (p, y, r), scores = cal.mount_correction(frames)
    Path(a.out).write_text(json.dumps({"pitch_deg": round(p, 3), "yaw_deg": round(y, 3), "roll_deg": round(r, 3),
                                       "agreement": [round(s, 3) for s in scores], "frames": len(frames), "from": a.db}, indent=1))
    print(f"head camera mount: pitch {p:+.2f} yaw {y:+.2f} roll {r:+.2f} deg (camera frame; agreement {np.round(scores, 3)}), "
          f"{len(frames)} frames, {time.time() - t0:.0f} s -> {a.out}")


if __name__ == "__main__":
    main()
