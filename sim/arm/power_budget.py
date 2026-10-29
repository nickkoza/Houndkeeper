#!/usr/bin/env python3
"""Arm battery budget from the sim + RobStride's published motor data (RS00/RS02 manuals, 2026-07-13).
Per step: joint torque tau and speed w from the sim. Per motor: current I = |tau|/Kt (Arms);
bus power = mechanical |tau*w| (motoring only; no regen credit) + copper loss 3*I_phase^2*R + standby 48 V x 18 mA.
Winding resistance is NOT published: bracketed low/high (est.). Picks use the true object pose (this measures power,
not perception). Items: same 5 x 6 x 5 cm block at different masses, then the sock."""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import json, sys
import numpy as np, mujoco
sys.path.insert(0, str(_HK / "sim" / "arm")); import vslice as vs, legs, payload_test as pt, go1_arm as ga
KT = [1.48, 1.22, 1.22, 1.48, 1.48, 1.48]          # N·m/Arms: J1 RS00, J2-J3 RS02, J4-J6 RS00 (maker_arm_v1.yaml)
R_LOW, R_HIGH = 0.15, 0.6                            # ohm per phase (est.; not in the manuals)
STANDBY_W = 7 * 48 * 0.018                            # 7 motors at the manual's <=18 mA standby
class Meter:
    def __init__(self, S): self.S = S; self.E = {"mech": 0.0, "cu_lo": 0.0, "cu_hi": 0.0}; self.t = 0.0; self.peak_tau = np.zeros(6)
    def __call__(self, m, d):
        dt = m.opt.timestep; tau = d.actuator_force[12:18]; w = np.array([d.qvel[a] for a in self.S.ad])
        i_rms = np.abs(tau) / np.array(KT); self.peak_tau = np.maximum(self.peak_tau, np.abs(tau))
        self.E["mech"] += np.maximum(tau * w, 0).sum() * dt
        self.E["cu_lo"] += (3 * (i_rms ** 2) * R_LOW).sum() * dt; self.E["cu_hi"] += (3 * (i_rms ** 2) * R_HIGH).sum() * dt; self.t += dt
    def wh(self):
        base = (self.E["mech"] + STANDBY_W * self.t) / 3600
        return round(base + self.E["cu_lo"] / 3600, 4), round(base + self.E["cu_hi"] / 3600, 4)
def pick(mass, obj="toy"):
    vs.OBJS[obj] = dict(vs.OBJS[obj]); S = vs.Sim(obj, np.array([0.1, 0.34])); m, d = S.m, S.d
    m.body_mass[S.ob] = mass; m.body_inertia[S.ob] *= mass / 0.04
    S.gains(120, 3); S.legs_to(legs.LIE, 2.0); S.step(100)
    M = Meter(S); S.on_step = M; o = d.xpos[S.ob].copy(); sa = d.xmat[S.ob].reshape(3, 3)[:, 1][:2]
    mount = d.xpos[m.body("arm_base_link").id].copy(); qr, _ = S.ik(mount + [0, 0, 0.45], sa); S.arm_to_safe(qr, 1.8)
    q, _ = S.ik(o + [0, 0, 0.22], sa); S.arm_to(q, 2.0); S.step(200); S.grip_to(0.05)
    q, _ = S.ik(o + [0, 0, 0.08], sa); S.arm_to(q, 1.5); q, _ = S.ik(o + [0, 0, 0.012], sa); S.arm_to(q, 1.2)
    S.grip_to(0.0, 1.0); S.step(100); z0 = d.xpos[S.ob][2]
    q, _ = S.ik(o + [0, 0, 0.18], sa); S.arm_to(q, 2.5); S.step(60); lifted = d.xpos[S.ob][2] - z0 > 0.05
    tray = d.xpos[m.body("tray").id].copy(); Rt = d.xmat[S.tr].reshape(3, 3)
    q, _ = S.ik(tray + [0, 0, 0.18], sa); S.arm_to(q, 2.5); q, _ = S.ik(tray + [0, 0, 0.10], Rt[:, 0][:2]); S.arm_to(q, 2.0)   # lower into the tray before letting go
    S.grip_to(0.05, 0.8); S.step(200); rel = Rt.T @ (d.xpos[S.ob] - tray); in_tray = bool(abs(rel[0]) < 0.15 and abs(rel[1]) < 0.11 and -0.01 < rel[2] < 0.08)
    q, _ = S.ik(tray + [0, 0, 0.32], Rt[:, 0][:2]); S.arm_to(q, 3.0); S.arm_to_safe(qr, 3.5); S.arm_to_safe(np.array(pt.STOW[:6]), 3.5, grip=0.0)   # slow: full stretch over the tray saturated the shoulder
    S.on_step = None
    return {"mass_kg": mass, "lifted": bool(lifted), "in_tray": in_tray, "pick_seconds": round(M.t, 1), "Wh_low_high": M.wh(),
            "peak_torque_Nm": [round(float(v), 1) for v in M.peak_tau], "torque_limit_Nm": ga.TAU[:6]}
def hold_walk(secs=20.0):
    S = vs.Sim("toy", np.array([5.0, 5.0])); M = Meter(S); S.on_step = M
    S.walk_to(np.array([3.0, 0.0]), 0.0, max_s=secs, settle=False); S.on_step = None
    lo, hi = M.wh(); return {"walk_seconds": round(M.t, 1), "W_low_high": (round(lo * 3600 / M.t, 1), round(hi * 3600 / M.t, 1)), "peak_torque_Nm": [round(float(v), 1) for v in M.peak_tau]}
if __name__ == "__main__":
    out = {"hold_while_walking": hold_walk()}; print(json.dumps(out["hold_while_walking"]), flush=True)
    out["picks"] = []
    for mass in (0.04, 0.15, 0.4, 0.8, 1.2, 1.5):
        r = pick(mass); out["picks"].append(r); print(json.dumps(r), flush=True)
    (vs.HERE / "power_budget.json").write_text(json.dumps(out, indent=1))
