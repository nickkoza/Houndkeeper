#!/usr/bin/env python3
"""Lying floor reach when the gripper may lean off vertical (reach spec [RHS]). For each floor cell (2.5 cm grid): can
the grasp point get 2 cm above the floor with the gripper within `tilt` degrees of straight down, jaws closing in any
direction, and no part of the arm touching the robot's body, legs, bin, itself or the floor? Kinematics only.
Usage: reach_tilt.py <tilt deg> [out.json]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import json, sys
import numpy as np, mujoco
sys.path.insert(0, str(_HK / "sim" / "arm"))
import vslice as vs, legs

TILT = float(sys.argv[1]); OUT = sys.argv[2] if len(sys.argv) > 2 else f"reach_lying_tilt{int(TILT)}.json"
S = vs.Sim("sock", np.array([0.0, 0.0]), watch=False, with_obj=False); S.gains(120, 3); S.legs_to(legs.LIE, 2.0); S.step(60)
m, d, k = S.m, S.d, S.kin; gc = m.body("arm_grasp_center").id; l7 = S.l7
R = d.xmat[S.tr].reshape(3, 3); c = d.xpos[S.tr].copy(); down = np.array([0, 0, -1.0]); tmax = np.radians(TILT)
arm_b = {i for i in range(m.nbody) if (mujoco.mj_id2name(m, 1, i) or "").startswith("arm_")}
jaw_b = {m.body("arm_gripper_left").id, m.body("arm_gripper_right").id, gc}

def floor_hit():
    """any arm part other than the fingertips touching the floor (the fingers may touch it at the grasp)"""
    for con in k.contact[:k.ncon]:
        b1, b2 = m.geom_bodyid[con.geom1], m.geom_bodyid[con.geom2]
        if (b1 == 0 and b2 in arm_b and b2 not in jaw_b) or (b2 == 0 and b1 in arm_b and b1 not in jaw_b): return True
    return False

def reach(p, seeds=8, iters=150):
    rng = np.random.default_rng(0)
    for s in range(seeds):
        q = d.ctrl[12:18].copy() if s == 0 else rng.uniform(S.rng[:, 0], S.rng[:, 1])
        for _ in range(iters):
            k.qpos[:] = d.qpos; k.qpos[S.aq] = q; mujoco.mj_kinematics(m, k); mujoco.mj_comPos(m, k)
            a = k.xpos[gc] - k.xpos[l7]; a /= np.linalg.norm(a); ang = np.arccos(np.clip(a @ down, -1, 1))
            ep = p - k.xpos[gc]; w = np.zeros(3)
            if ang > tmax:                                    # lean back toward vertical only by what is over the limit
                ax = np.cross(a, down); n = np.linalg.norm(ax)
                if n > 1e-9: w = ax / n * (ang - tmax)
            if np.linalg.norm(ep) < 0.005 and ang <= tmax + np.radians(2): break
            Jp = np.zeros((3, m.nv)); Jr = np.zeros((3, m.nv)); mujoco.mj_jacBody(m, k, Jp, None, gc); mujoco.mj_jacBody(m, k, None, Jr, l7)
            J = np.vstack([Jp[:, S.ad], 0.3 * Jr[:, S.ad]]); e = np.r_[ep, 0.3 * w]
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 2e-3 * np.eye(6), e), S.rng[:, 0], S.rng[:, 1])
        if np.linalg.norm(ep) < 0.01 and ang <= tmax + np.radians(2):
            if S._hits(q) == 0:
                k.qpos[:] = d.qpos; k.qpos[S.aq] = q; mujoco.mj_fwdPosition(m, k)
                if not floor_hit(): return True
    return False

xs = np.round(np.arange(-0.40, 0.8001, 0.025), 3); ys = np.round(np.arange(-0.60, 0.6001, 0.025), 3)
M = np.zeros((len(ys), len(xs)), np.uint8)
for i, y in enumerate(ys):
    for j, x in enumerate(xs):
        if abs(x) < 0.36 and abs(y) < 0.17: continue             # under the body
        p = c + R @ np.array([x, y, 0]); p[2] = 0.02; M[i, j] = reach(p)
json.dump({"xs": xs.tolist(), "ys": ys.tolist(), "reach": M.tolist(), "tilt_deg": TILT, "mount_x_m": float(vs.ga.MOUNT[0]),
           "note": "lying, grasp point 2 cm above floor, gripper within tilt of vertical, jaws any direction, no arm contact with body/legs/bin/itself/floor"},
          open(vs.HERE / OUT, "w"))
print("TILT", TILT, "area m2", round(M.sum() * 0.025 ** 2, 3), flush=True)
