#!/usr/bin/env python3
"""Tip safety for the Go2-with-Maker-Arm (sim Go1 body). Two parts:
  static  : robot lying or crouched, arm at every reachable grasp-point target around it (floor to high), holding
            0 / 0.5 / 1.5 kg. Margin = how far the whole robot's centre of mass sits inside the feet's support polygon
            (convex hull of foot contact points). Kinematic, fast.
  dynamic : from the raised 'ready' pose, move to targets at increasing joint speed (physics, viewer on) and record
            the worst trunk tilt; find the highest joint speed that stays under TILT_OK.
Output: tip_envelope.json (safe targets per posture/payload, max safe joint speed) for the planner to enforce.
Usage: tip_bench.py [--dynamic-only] [--headless] [--dyn-payloads 0.0,1.5] [--out PATH]
  --dynamic-only writes no static rows and no safe reach: give it --out, never the envelope itself"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import argparse, json, sys, time
import numpy as np, mujoco
from scipy.spatial import ConvexHull, Delaunay
sys.path.insert(0, str(_HK / "sim" / "arm"))
import vslice as vs, legs, go1_arm as ga, payload_test as pt, hud as hudm
TILT_OK = 8.0            # deg: more than this while the arm moves counts as unsafe (est.; lying pose rocks ~2-4 deg normally)
PAYLOADS = (0.0, 0.5, 1.5)
def support_margin(m, d, S):
    """distance (m) from the whole-robot CoM (x,y) to the edge of the foot-contact support polygon; negative = outside"""
    feet = np.array([d.site_xpos[m.site(n).id][:2] for n in ("FR", "FL", "RR", "RL")])
    for c in d.contact[:d.ncon]:                             # lying: calves/thighs also touch the floor and support the body
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if 0 in (b1, b2) and (mujoco.mj_id2name(m, 1, b1 if b2 == 0 else b2) or "").split("_")[0] in ("FR", "FL", "RR", "RL", "trunk"):
            feet = np.vstack([feet, c.pos[:2]])
    hull = ConvexHull(feet); com = d.subtree_com[S.tr][:2]
    eq = hull.equations                                        # a*x + b*y + c <= 0 inside
    return float(-np.max(eq[:, :2] @ com + eq[:, 2]))
def static(S, posture):
    m, d = S.m, S.d; rows = []; tr = S.tr
    R = d.xmat[tr].reshape(3, 3); c = d.xpos[tr].copy()
    item = m.body(S.m.body("sock").name).id if False else None
    for pay in PAYLOADS:
        gc = m.body("arm_grasp_center").id; m0 = m.body_mass[gc]; m.body_mass[gc] = pay      # payload carried at the grasp point
        for r in (0.30, 0.45, 0.60):
            for ang in range(0, 360, 30):
                for z in (0.02, 0.20, 0.45):
                    a = np.radians(ang); tgt = c + R @ np.array([ga.MOUNT[0] + r * np.cos(a), r * np.sin(a), 0]); tgt[2] = z
                    q, e = S.ik_pos(tgt, seeds=6)
                    if e > 0.02: continue
                    d.qpos[S.aq] = q; mujoco.mj_forward(m, d)
                    rows.append({"posture": posture, "payload_kg": pay, "r": r, "ang": ang, "z": z, "margin_cm": round(100 * support_margin(m, d, S), 1)})
        m.body_mass[gc] = m0
    return rows
def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--dynamic-only", action="store_true"); ap.add_argument("--headless", action="store_true")
    ap.add_argument("--dyn-payloads", default="0.0,1.5", help="kg held at the gripper for the dynamic swings, comma-separated")
    ap.add_argument("--out", default=None, help="where to write the envelope (default: tip_envelope.json beside this script)"); a = ap.parse_args()
    dyn_payloads = tuple(float(x) for x in a.dyn_payloads.split(","))
    S = vs.Sim("sock", np.array([30.0, 30.0]), watch=not a.headless, hud=hudm.Hud()); m, d = S.m, S.d
    out = {"tilt_ok_deg": TILT_OK, "static": [], "dynamic": []}
    S.gains(120, 3)
    for posture, target in (("crouch", legs.CROUCH), ("lie", legs.LIE)):
        S.state("LIE DOWN" if posture == "lie" else "STAND UP"); S.say(f"posture: {posture}"); S.legs_to(target, 2.0); S.step(80)
        if not a.dynamic_only:
            save = d.qpos.copy(); rows = static(S, posture); d.qpos[:] = save; mujoco.mj_forward(m, d); out["static"] += rows
            for pay in PAYLOADS:
                rr = [x for x in rows if x["payload_kg"] == pay]; bad = [x for x in rr if x["margin_cm"] < 3.0]
                print(f"STATIC {posture} payload {pay} kg: {len(rr)} reachable targets; worst margin {min(x['margin_cm'] for x in rr):.1f} cm; "
                      f"{len(bad)} under 3 cm margin" + (f" (e.g. r={bad[0]['r']} ang={bad[0]['ang']} z={bad[0]['z']})" if bad else ""), flush=True)
        # dynamic: raised pose <-> far-out side targets, faster each round
        mount = d.xpos[m.body("arm_base_link").id].copy(); q_up, _ = S.ik(mount + [0, 0, 0.45], [1, 0])
        R = d.xmat[S.tr].reshape(3, 3); c = d.xpos[S.tr].copy()
        tgts = [c + R @ np.array([ga.MOUNT[0], 0.55, 0]), c + R @ np.array([ga.MOUNT[0], -0.55, 0]), c + R @ np.array([0.60, 0.0, 0])]
        for t_ in tgts: t_[2] = 0.05
        S.ARM_VMAX = 99; S.arm_to(q_up, 3.0); S.step(100); gc = m.body("arm_grasp_center").id
        for pay in dyn_payloads:                              # default: empty arm, and the arm's rated 1.5 kg held at the gripper
          m.body_mass[gc] = pay; m.body_inertia[gc] = [1e-4, 1e-4, 1e-4]; S.say(f"{posture}: payload {pay} kg at the gripper"); S.step(100)
          for vmax in (0.4, 0.6, 0.8, 1.0, 1.4, 2.0, 3.0):
            worst = 0.0; S.ARM_VMAX = vmax; moved = 0; tipped = False
            for tg in tgts:
                q, e = S.ik_pos(tg, seeds=6)
                if e > 0.02: continue
                S.say(f"{posture}: arm swing at up to {vmax} rad/s"); S.max_tilt = 0.0; S.arm_to(q, 0.1); S.step(60); S.arm_to(q_up, 0.1); S.step(80)
                worst = max(worst, S.max_tilt); moved += 1
                if d.xpos[S.tr][2] < 0.05 or worst > 45: tipped = True; break
            res_ = None if moved == 0 else round(worst, 1)
            out["dynamic"].append({"posture": posture, "payload_kg": pay, "vmax_rad_s": vmax, "worst_tilt_deg": res_, "tipped": tipped})
            print(f"DYNAMIC {posture} payload {pay} kg, max joint speed {vmax} rad/s: " + ("no data (no reachable targets)" if res_ is None else f"worst trunk tilt {worst:.1f} deg{' - TIPPED' if tipped else ''}"), flush=True)
            if tipped or res_ is None:
                S.say("tipped over - resetting posture"); m.body_mass[gc] = 0; S.ARM_VMAX = 0.6; S.arm_to(q_up, 3.0)
                S.legs_to(legs.STAND, 2.0); d.qpos[3:7] = [1, 0, 0, 0]; d.qpos[2] = 0.30; d.qvel[:] = 0; mujoco.mj_forward(m, d); S.legs_to(target, 2.0); S.step(100); break
          safe_v = None                                        # fastest speed such that it AND every slower one stayed under TILT_OK
          for x in sorted([x for x in out["dynamic"] if x["posture"] == posture and x["payload_kg"] == pay], key=lambda x: x["vmax_rad_s"]):
              if x["worst_tilt_deg"] is None or x["tipped"] or x["worst_tilt_deg"] > TILT_OK: break
              safe_v = x["vmax_rad_s"]
          out.setdefault("safe_speed", {})[f"{posture}_{pay}"] = safe_v
          print(f"SAFE {posture} payload {pay} kg: max joint speed {safe_v if safe_v is not None else 'NONE (unsafe even at 0.4 rad/s)'} rad/s (tilt <= {TILT_OK} deg)", flush=True)
        m.body_mass[gc] = 0.0
        S.ARM_VMAX = 0.6; S.arm_to_safe(np.array(pt.STOW[:6]), 3.0); S.legs_to(legs.STAND, 2.5); S.step(100)
    env = {}                                              # safe reach: furthest r where every direction/height keeps >= 3 cm CoM margin
    for posture in ("crouch", "lie"):
        for pay in PAYLOADS:
            rows = [x for x in out["static"] if x["posture"] == posture and x["payload_kg"] == pay]; ok_r = None
            for r in sorted({x["r"] for x in rows}):
                if all(x["margin_cm"] >= 3.0 for x in rows if x["r"] == r): ok_r = r
                else: break
            env[f"{posture}_{pay}"] = ok_r
            if rows: print(f"REACH {posture} payload {pay} kg: safe out to r = {ok_r} m from the mount in every direction (3 cm CoM margin)", flush=True)
    out["safe_reach_m"] = env
    (Path(a.out) if a.out else vs.HERE / "tip_envelope.json").write_text(json.dumps(out, indent=1))
    import os; sys.stdout.flush(); os._exit(0)
if __name__ == "__main__": main()
