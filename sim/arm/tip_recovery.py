#!/usr/bin/env python3
"""How far can the robot lean and still recover? For each posture, direction and arm load: start the robot leaned by an
angle (legs holding the posture, arm holding its pose), let go for 3 s, and see whether it comes back level or falls.
Bisect to the tipping angle. Viewer on by default. Usage: tip_recovery.py [--headless]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import argparse, json, sys
import numpy as np, mujoco
sys.path.insert(0, str(_HK / "sim" / "arm"))
import vslice as vs, legs, payload_test as pt, hud as hudm, go1_arm as ga
DIRS = {"left": (1, 0), "right": (-1, 0), "forward": (0, 1), "back": (0, -1)}    # roll or pitch axis sign
def tilt_of(d, tr): R = d.xmat[tr].reshape(3, 3); return float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
def trial(S, posture_q, arm_q, pay, dirn, ang, settle):
    m, d = S.m, S.d; gc = m.body("arm_grasp_center").id; m.body_mass[gc] = pay
    d.qpos[:] = settle["qpos"]; d.qvel[:] = 0; d.ctrl[:12] = posture_q; d.ctrl[12:18] = arm_q; d.qpos[S.aq] = arm_q
    a = np.radians(ang); rx, ry = DIRS[dirn]; ax = np.array([ry, rx, 0.0]) if False else np.array([rx * 1.0, ry * 1.0, 0.0])
    ax = np.array([1.0, 0, 0]) if dirn in ("left", "right") else np.array([0, 1.0, 0]); sgn = -1 if dirn in ("left", "forward") else 1
    q = np.zeros(4); mujoco.mju_axisAngle2Quat(q, ax, sgn * a); d.qpos[3:7] = q
    mujoco.mj_forward(m, d); d.qpos[2] += max(0.0, 0.02 - min(d.geom_xpos[i][2] - m.geom_rbound[i] * 0 for i in range(m.ngeom) if m.geom_bodyid[i] != 0 and m.geom_contype[i]))  # lift clear of the floor
    mujoco.mj_forward(m, d); worst = 0.0
    for k in range(600):                                           # 3 s
        ga.step(m, d); S.t += 0.005
        if k % 7 == 0: S._watch()
    final = tilt_of(d, S.tr); m.body_mass[gc] = 0.0
    return final < 15.0, final                                     # recovered = back within 15 deg of level
def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--headless", action="store_true"); a = ap.parse_args()
    S = vs.Sim("sock", np.array([30.0, 30.0]), watch=not a.headless, hud=hudm.Hud()); m, d = S.m, S.d; S.gains(120, 3); S.speed = 4.0
    out = []
    for posture, pq in (("crouch", legs.CROUCH), ("lie", legs.LIE)):
        S.legs_to(pq, 2.0); S.step(100)
        mount = d.xpos[m.body("arm_base_link").id].copy(); R = d.xmat[S.tr].reshape(3, 3); c = d.xpos[S.tr].copy()
        arm_poses = {"stowed": (np.array(pt.STOW[:6]), 0.0)}
        for side, y in (("reach left", 0.45), ("reach right", -0.45)):
            tgt = c + R @ np.array([vs.ga.MOUNT[0], y, 0]); tgt[2] = 0.10; q, e = S.ik_pos(tgt, seeds=8)
            if e < 0.02: arm_poses[side + " 1.5 kg"] = (q, 1.5)
        settle = {"qpos": d.qpos.copy()}
        for pname, (aq, pay) in arm_poses.items():
            for dirn in DIRS:
                lo, hi = 0.0, 80.0; ok, _ = trial(S, pq, aq, pay, dirn, hi, settle)
                if ok: out.append({"posture": posture, "arm": pname, "lean": dirn, "tipping_deg": ">80"}); continue
                ok0, f0 = trial(S, pq, aq, pay, dirn, 0.0, settle)
                if not ok0:
                    out.append({"posture": posture, "arm": pname, "lean": dirn, "tipping_deg": 0.0, "note": "falls even from level"}); print(json.dumps(out[-1]), flush=True); continue
                for _ in range(7):                                    # bisect to ~0.6 deg
                    mid = (lo + hi) / 2; S.say(f"{posture}, arm {pname}: leaning {dirn} {mid:.1f} deg")
                    ok, _ = trial(S, pq, aq, pay, dirn, mid, settle)
                    lo, hi = (mid, hi) if ok else (lo, mid)
                out.append({"posture": posture, "arm": pname, "lean": dirn, "tipping_deg": round(lo, 1)}); print(json.dumps(out[-1]), flush=True)
        d.qpos[:] = settle["qpos"]; d.qvel[:] = 0; mujoco.mj_forward(m, d); S.legs_to(legs.STAND, 2.0)
    for posture in ("crouch", "lie"):
        rows = [r for r in out if r["posture"] == posture and isinstance(r["tipping_deg"], float)]
        if rows: w = min(rows, key=lambda r: r["tipping_deg"]); print(f"TIPPING {posture}: worst case {w['tipping_deg']} deg (arm {w['arm']}, leaning {w['lean']})", flush=True)
    (vs.HERE / "tip_recovery.json").write_text(json.dumps(out, indent=1))
    import os; sys.stdout.flush(); os._exit(0)
if __name__ == "__main__": main()
