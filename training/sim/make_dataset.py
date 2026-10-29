#!/usr/bin/env python3
"""Sim demonstrations -> a LeRobot v3 dataset, through DimOS's own dataset writer ([soS]). Only successful episodes
(the item was off the floor when the demonstration ended). Feature names follow what LeRobot's ACT expects:
observation.images.wrist, observation.state (6 arm joints + gripper), action (the same 7, as targets). 10 Hz.
Usage: make_dataset.py <demo dir> <dataset out dir>"""
import glob, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src")); from houndkeeper import workspace   # noqa: E402  (decision [cy4])
sys.path.insert(0, str(workspace.dimos()))   # the DimOS checkout beside this repo
from dimos.imitation.dataprep.core import OutputConfig, Sample
from dimos.imitation.dataprep.formats.lerobot.writer import write

src, out = Path(sys.argv[1]), Path(sys.argv[2]); FPS = 10.0
eps = sorted(glob.glob(str(src / "ep_*.npz")))
def samples():
    n_ok = 0
    for f in eps:
        z = np.load(f, allow_pickle=True)
        if not bool(z["success"]) or len(z["img"]) < 10: continue
        n_ok += 1; eid = Path(f).stem
        for t in range(len(z["img"])):
            yield Sample(ts=t / FPS, episode_id=eid, observation={"wrist": z["img"][t], "state": z["state"][t]},
                         action={"joints": z["action"][t]}, task_label="pick up the item on the floor")
    print(f"{n_ok} successful episodes of {len(eps)}", flush=True)
root = write(samples(), OutputConfig(format="lerobot", path=out, metadata={"fps": FPS, "robot_type": "go2_maker_arm_sim"}))
print("dataset:", root)
