#!/usr/bin/env python3
"""Front-zone reach by jaw direction ([ZMc], [Hwc]): for each 2.5 cm floor cell in front of the lying robot, the share
of jaw closing directions (every 30 deg) the leaning grasp can reach there (Sim.ik_lean: gripper within the stated lean,
jaws level, no arm contact with the robot or the floor), at the grasp height the pick routine uses. An item's narrow
direction is wherever it landed, so the robot should park items where every direction is reachable
(cluster_stops.front_offset reads this map). Kinematics only. Usage: reach_dirs.py [lean deg] [out.json]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import json, sys
import numpy as np
sys.path.insert(0, str(_HK / "sim" / "arm"))
import vslice as vs, legs

LEAN = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
OUT = sys.argv[2] if len(sys.argv) > 2 else f"reach_dirs_tilt{int(LEAN)}.json"
GRASP_Z = 0.035                  # grasp point height with the fingertips 3 mm up (pick.TIP_CLEAR, tips ~3.4 cm past it)
S = vs.Sim("sock", np.array([0.0, 0.0]), watch=False, with_obj=False); S.gains(120, 3); S.legs_to(legs.LIE, 2.0); S.step(60)
d = S.d; R = d.xmat[S.tr].reshape(3, 3); c = d.xpos[S.tr].copy()
dirs = np.radians(np.arange(0, 180, 30))
xs = np.round(np.arange(0.36, 0.6201, 0.025), 3); ys = np.round(np.arange(-0.25, 0.1501, 0.025), 3)
share = np.zeros((len(ys), len(xs)))
for i, y in enumerate(ys):
    for j, x in enumerate(xs):
        p = c + R @ np.array([x, y, 0.0]); p[2] = GRASP_Z; ok = 0
        for a in dirs:
            q, e, _ = S.ik_lean(p, R[:2, :2] @ np.array([np.cos(a), np.sin(a)]), LEAN); ok += q is not None and e < 0.01
        share[i, j] = ok / len(dirs)
    print(f"y {y:+.3f}: " + " ".join(f"{s_:.1f}" for s_ in share[i]), flush=True)
json.dump({"xs": xs.tolist(), "ys": ys.tolist(), "share": share.round(3).tolist(), "lean_deg": LEAN, "grasp_z_m": GRASP_Z,
           "directions_deg": np.degrees(dirs).tolist(),
           "note": "lying; share of jaw directions reachable by Sim.ik_lean at the grasp height; robot frame, metres"},
          open(vs.HERE / OUT, "w"))
print("saved", OUT, "cells with every direction:", int((share == 1).sum()), "of", share.size, flush=True)
