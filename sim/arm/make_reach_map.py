#!/usr/bin/env python3
"""Floor reach while lying: straight-down grasp 2 cm above the floor, IK + collision check (robot body, legs, tray, arm itself)."""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import sys, json
import numpy as np
sys.path.insert(0, str(_HK / "sim" / "arm")); import vslice as vs, legs
S = vs.Sim("toy", np.array([4.0, 4.0])); S.gains(120, 3); S.legs_to(legs.LIE, 2.0); S.step(200)
d = S.d; R = d.xmat[S.tr].reshape(3, 3); c = d.xpos[S.tr].copy()
xs = np.round(np.arange(-0.40, 0.81, 0.05), 2); ys = np.round(np.arange(-0.60, 0.61, 0.05), 2); M = np.zeros((len(ys), len(xs)), np.uint8)
for i, y in enumerate(ys):
    for j, x in enumerate(xs):
        if abs(x) < 0.36 and abs(y) < 0.17: continue
        p = c + R @ np.array([x, y, 0]); p[2] = 0.02; q, e = S.ik(p, R[:, 1][:2], seeds=8)
        if e < 0.012 and S._hits(q) == 0: M[i, j] = 1
json.dump({"xs": xs.tolist(), "ys": ys.tolist(), "reach": M.tolist(), "mount_x_m": float(vs.ga.MOUNT[0]), "note": "lying, mount from params/robot.yaml, turned 180 deg (factory fold forward), straight-down grasp 2 cm above floor, collision-checked incl. self"}, open(vs.HERE / "reach_lying.json", "w"))
print("reachable", int(M.sum()), "cells,", round(M.sum() * 0.0025, 3), "m2")
for i in range(len(ys)): print(f"{ys[i]:+.2f} " + "".join("#" if (abs(xs[j]) < 0.36 and abs(ys[i]) < 0.17) else ("o" if M[i, j] else ".") for j in range(len(xs))))
