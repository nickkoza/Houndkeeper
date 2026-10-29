#!/usr/bin/env python3
"""Is a standing look safe? ([BPL]'s standing envelope, for the close look's cheaper first step,
used only once balance, the main risk, is confirmed not to be an issue).
The robot stands; the arm aims the wrist camera down at a spot on the floor 0.5-1.0 m ahead and up to 35 cm aside,
from at least 0.5 m up (a wide view: the find's position is 13-30 cm uncertain). For each aiming pose:
  - the tipping angle, standing, in each lean direction (the same bisection as tip_recovery.py's crouched and lying);
  - the arm moving there from its stow at the arm's speed limit while the legs hold the stand: the trunk's largest tilt.
Compared with the robot standing with the arm stowed. Viewer on by default.
Usage: standing_look.py [--headless]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import argparse, json, math, sys
import numpy as np, mujoco
from scipy.optimize import minimize
sys.path.insert(0, str(_HK / "sim" / "arm"))
import vslice as vs, legs, payload_test as pt, hud as hudm, go1_arm as ga
from tip_recovery import trial, tilt_of, DIRS
sys.path.insert(0, str(_HK / "src")); from houndkeeper import params

MIN_CAM_UP = 0.50                    # m: high enough that the view covers the find's position error
TARGETS = [(f, s) for f in (0.5, 0.75, 1.0) for s in (-0.35, 0.0, 0.35)]   # (ahead, aside) of the trunk centre, m


def aim(S, target_w, q0, min_up=MIN_CAM_UP, up_deg=0.0):
    """arm joints that point the wrist camera at target_w from at least min_up high, near q0: straight at it, or with
    the target up_deg above the image's centre (the jaws fill the image's lower middle, and a find on their tip is hidden
    behind them, 27 Sep)"""
    m, kd = S.m, S.kin; cid = m.camera("arm_wrist").id; lo, hi = S.rng[:, 0], S.rng[:, 1]
    ca, sa = np.cos(np.radians(up_deg)), np.sin(np.radians(up_deg))
    def want(kd):                                                 # where the target should lie from the camera
        X = kd.cam_xmat[cid].reshape(3, 3); return -X[:, 2] * ca + X[:, 1] * sa   # forward, turned toward the image's up
    def cost(q):
        kd.qpos[:] = S.d.qpos; kd.qpos[S.aq] = np.clip(q, lo, hi); mujoco.mj_kinematics(m, kd); mujoco.mj_camlight(m, kd)
        c = kd.cam_xpos[cid]; fwd = want(kd); to = target_w - c; to /= np.linalg.norm(to)
        return (1 - fwd @ to) * 50 + max(0.0, min_up - c[2]) ** 2 * 400 + 0.002 * np.sum((q - q0) ** 2)
    best = None
    for k in range(6):                                            # a few starts: the aim is not convex
        seed = np.clip(q0 + np.random.default_rng(k).normal(0, 0.5, 6) * (k > 0), lo, hi)
        r = minimize(cost, seed, method="Powell", options={"maxiter": 3000, "xtol": 1e-4, "ftol": 1e-7})
        if best is None or r.fun < best.fun: best = r
    q = np.clip(best.x, lo, hi); kd.qpos[S.aq] = q; mujoco.mj_kinematics(m, kd); mujoco.mj_camlight(m, kd)
    c = kd.cam_xpos[cid]; fwd = want(kd); to = (target_w - c) / np.linalg.norm(target_w - c)
    return q, float(np.degrees(np.arccos(np.clip(fwd @ to, -1, 1)))), float(c[2])


def move_arm(S, q_from, q_to, speed, hold=1.0):
    """the arm from q_from to q_to at `speed` rad/s (its slowest joint's pace), legs holding the stand: largest trunk tilt"""
    m, d = S.m, S.d; T = max(float(np.max(np.abs(q_to - q_from))) / speed, 0.2); n = int((T + hold) / 0.005); worst = 0.0
    for k in range(n):
        f = min(1.0, k * 0.005 / T); d.ctrl[12:18] = q_from + (q_to - q_from) * f
        ga.step(m, d); S.t += 0.005; worst = max(worst, tilt_of(d, S.tr))
        if k % 7 == 0: S._watch()
    return worst


def tipping(S, arm_q, settle):
    out = {}
    for dirn in DIRS:
        lo, hi = 0.0, 80.0; ok, _ = trial(S, legs.STAND, arm_q, 0.0, dirn, hi, settle)
        if ok: out[dirn] = ">80"; continue
        if not trial(S, legs.STAND, arm_q, 0.0, dirn, 0.0, settle)[0]: out[dirn] = 0.0; continue
        for _ in range(7):
            mid = (lo + hi) / 2; ok, _ = trial(S, legs.STAND, arm_q, 0.0, dirn, mid, settle); lo, hi = (mid, hi) if ok else (lo, mid)
        out[dirn] = round(lo, 1)
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--headless", action="store_true"); a = ap.parse_args()
    S = vs.Sim("sock", np.array([30.0, 30.0]), watch=not a.headless, hud=hudm.Hud()); m, d = S.m, S.d; S.gains(120, 3); S.speed = 4.0
    S.legs_to(legs.STAND, 2.0); S.step(200); settle = {"qpos": d.qpos.copy()}
    stow = np.array(pt.STOW[:6]); speed = params.get("maker_arm.joint_speed_limit_rad_s")
    R = d.xmat[S.tr].reshape(3, 3); c = d.xpos[S.tr].copy(); rows = []
    base = tipping(S, stow, settle); print(json.dumps({"pose": "stowed", "tipping_deg": base}), flush=True)
    rows.append({"pose": "stowed", "tipping_deg": base})
    q_prev = stow
    for ahead, aside in TARGETS:
        tgt = c + R @ np.array([ahead, aside, 0.0]); tgt[2] = 0.0
        q, miss_deg, cam_up = aim(S, tgt, q_prev)
        d.qpos[:] = settle["qpos"]; d.qvel[:] = 0; d.ctrl[:12] = legs.STAND; d.qpos[S.aq] = stow; d.ctrl[12:18] = stow; mujoco.mj_forward(m, d)
        S.say(f"standing look at {ahead:.2f} m ahead, {aside:+.2f} m aside")
        moving = move_arm(S, stow, q, speed)
        tip = tipping(S, q, settle)
        row = {"pose": f"look {ahead:.2f} ahead {aside:+.2f} aside", "aim_miss_deg": round(miss_deg, 1), "camera_up_m": round(cam_up, 2),
               "moving_tilt_deg": round(moving, 2), "tipping_deg": tip, "q": [round(v, 3) for v in q]}
        rows.append(row); print(json.dumps(row), flush=True)
    num = lambda t: min(80.0 if v == ">80" else float(v) for v in t.values())
    worst = min(rows[1:], key=lambda r: num(r["tipping_deg"]))
    print(f"STANDING stowed: tipping {num(base):.1f} deg worst direction; standing looks: worst {num(worst['tipping_deg']):.1f} deg ({worst['pose']}); "
          f"largest tilt while the arm moved {max(r['moving_tilt_deg'] for r in rows[1:]):.2f} deg")
    json.dump(rows, open(str(_HK / "sim" / "suite" / "last_standing_look.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
