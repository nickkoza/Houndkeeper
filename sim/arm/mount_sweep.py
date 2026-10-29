#!/usr/bin/env python3
"""Arm mount placement ([Gy7]): try the Maker Arm at several positions along the robot's back (folded gripper forward)
and score each the same way: floor reach lying, tipping point lying/crouched, worst CoM margin with the rated 1.5 kg,
loaded walking, head-camera view blocked by the folded arm, and back length left for the bin.
Body: the sim's Go1 stand-in (re-run on the Go2 model before fabricating). Viewer on by default.
Usage: mount_sweep.py [--positions -0.05 0 0.05 0.10 0.15 0.20] [--headless]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import argparse, json, sys
import numpy as np, mujoco
sys.path.insert(0, str(_HK / "sim" / "arm"))
import vslice as vs, legs, payload_test as pt, go1_arm as ga, hud as hudm, tip_recovery as tr, tip_bench as tb
BODY_REAR, BODY_TOP = -0.132, 0.058 + 0.036          # Go1 body shell rear edge; mount plate top above trunk centre
def reach_area(S):
    d = S.d; R = d.xmat[S.tr].reshape(3, 3); c = d.xpos[S.tr].copy(); n = 0
    for y in np.arange(-0.6, 0.61, 0.1):
        for x in np.arange(-0.5, 0.81, 0.1):
            if abs(x) < 0.36 and abs(y) < 0.17: continue
            p = c + R @ np.array([x, y, 0]); p[2] = 0.02; q, e = S.ik(p, R[:, 1][:2], seeds=4)
            n += (e < 0.012 and S._hits(q) == 0)
    return round(n * 0.01, 2)
def cam_blocked(S, cam):
    m, d = S.m, S.d; r = mujoco.Renderer(m, 120, 160); r.enable_segmentation_rendering(); r.update_scene(d, cam); seg = r.render()[..., 0]; r.close()
    armg = [i for i in range(m.ngeom) if (mujoco.mj_id2name(m, 1, m.geom_bodyid[i]) or "").startswith("arm_")]
    return round(100 * float(np.isin(seg, armg).mean()), 1)
def tipping(S, pq, settle):
    out = {}
    for dirn in ("left", "right", "forward", "back"):
        lo, hi = 0.0, 80.0
        ok, _ = tr.trial(S, pq, np.array(pt.STOW[:6]), 0.0, dirn, hi, settle)
        if ok: out[dirn] = 80.0; continue
        for _ in range(6):
            mid = (lo + hi) / 2; ok, _ = tr.trial(S, pq, np.array(pt.STOW[:6]), 0.0, dirn, mid, settle); lo, hi = (mid, hi) if ok else (lo, mid)
        out[dirn] = round(lo, 1)
    return out
def walk(S):
    m, d = S.m, S.d; S.gains(35, 0); S.pol = ga.LegPolicy(m); worst = 0.0; track = []
    for dur, cmd, name in pt.SCRIPT:
        S.pol.cmd = np.array(cmd, np.float32); vx = []
        for k in range(int(dur / 0.005)):
            S.step(); R = d.xmat[S.tr].reshape(3, 3)
            if k * 0.005 > 1.0: worst = max(worst, np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))); vx.append((R.T @ d.qvel[:3])[0])
        if abs(cmd[0]) > 0.1: track.append(np.mean(vx) / cmd[0])
    S.pol.cmd[:] = 0; S.step(100); S.pol = None
    return round(worst, 1), round(float(np.mean(track)), 2)
def bin_room(S, mx):
    m, d = S.m, S.d; R = d.xmat[S.tr].reshape(3, 3); c = d.xpos[S.tr]
    length = (mx - 0.05) - BODY_REAR                                  # from the body's rear edge to the arm base's rear edge
    armb = [b for b in range(m.nbody) if (mujoco.mj_id2name(m, 1, b) or "").startswith("arm_") and mujoco.mj_id2name(m, 1, b) not in ("arm_base_link", "arm_link_002")]; zs = []
    for i in range(m.ngeom):
        if m.geom_bodyid[i] in armb and m.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
            mid = m.geom_dataid[i]; V = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            L = ((d.geom_xmat[i].reshape(3, 3) @ V.T).T + d.geom_xpos[i] - c) @ R
            sel = (L[:, 0] > BODY_REAR) & (L[:, 0] < mx - 0.05) & (np.abs(L[:, 1]) < 0.11); zs += list(L[sel, 2])
    clear = (min(zs) - BODY_TOP) if zs else 0.30                      # headroom above the plate under the folded arm
    return round(length * 100, 1), round(clear * 100, 1)
def upright(S, mx):
    """robot standing on its walking controller: table/counter reach, arm swing with load, walking while carrying"""
    m, d = S.m, S.d; S.gains(35, 0); S.pol = ga.LegPolicy(m); S.pol.cmd[:] = 0; S.step(200)
    R = d.xmat[S.tr].reshape(3, 3); c = d.xpos[S.tr].copy(); res = {}
    for side, v in (("front", np.array([0.55, 0.0])), ("side", np.array([mx, 0.45]))):   # a table edge ~25 cm past the nose, or beside
        best = None
        for h in (0.5, 0.6, 0.75, 0.9, 1.0):
            t_ = c + R @ np.array([v[0], v[1], 0]); t_[2] = h; q, e = S.ik_pos(t_, seeds=6)
            if e < 0.015: best = h
        res[f"reach_up_{side}_m"] = best
    q_up, _ = S.ik_pos(c + R @ np.array([0.45, 0, 0]) + np.array([0, 0, 0.75 - c[2]]), seeds=6)       # arm out in front at table height
    gc = m.body("arm_grasp_center").id
    for pay in (0.5, 1.5):
        m.body_mass[gc] = pay; S.max_tilt = 0.0; S.arm_to(q_up, 0.1); S.step(200); S.arm_to(np.array(pt.STOW[:6]), 0.1); S.step(200)
        res[f"stand_swing_tilt_{pay}kg_deg"] = round(S.max_tilt, 1)
    m.body_mass[gc] = 0.5; q_carry, _ = S.ik_pos(c + R @ np.array([mx + 0.30, 0, 0]) + np.array([0, 0, 0.62 - c[2]]), seeds=6)
    S.arm_to(q_carry, 0.1); S.step(100); worst = 0.0; vx = []
    for cmd, dur in (((0.3, 0, 0), 4.0), ((0, 0, 0.6), 3.0), ((0.3, 0, 0.4), 3.0)):
        S.pol.cmd = np.array(cmd, np.float32)
        for k in range(int(dur / 0.005)):
            S.step(); Rr = d.xmat[S.tr].reshape(3, 3); worst = max(worst, np.degrees(np.arccos(np.clip(Rr[2, 2], -1, 1))))
            if cmd[0] > 0 and k * 0.005 > 1: vx.append((Rr.T @ d.qvel[:3])[0] / cmd[0])
    res["carry_walk_tilt_deg"] = round(worst, 1); res["carry_walk_speed_ratio"] = round(float(np.mean(vx)), 2); res["carry_walk_fell"] = bool(d.xpos[S.tr][2] < 0.12)
    m.body_mass[gc] = 0.0; S.pol.cmd[:] = 0; S.arm_to(np.array(pt.STOW[:6]), 0.1); S.step(200); S.pol = None
    return res
def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--positions", type=float, nargs="+", default=[-0.05, 0.0, 0.05, 0.10, 0.15, 0.20])
    ap.add_argument("--headless", action="store_true"); a = ap.parse_args(); rows = []
    for mx in a.positions:
        mount = (mx, 0.0, ga.MOUNT[2]); S = vs.Sim("sock", np.array([30.0, 30.0]), watch=not a.headless, hud=hudm.Hud(), mount=mount); m, d = S.m, S.d; S.speed = 4.0
        r = {"mount_x_m": mx}; S.say(f"mount {mx * 100:+.0f} cm: checking the folded arm")
        mujoco.mj_forward(m, d); bad = [c for c in d.contact[:d.ncon] if (mujoco.mj_id2name(m, 1, m.geom_bodyid[c.geom1]) or "").startswith("arm_") != (mujoco.mj_id2name(m, 1, m.geom_bodyid[c.geom2]) or "").startswith("arm_")]
        if bad: r["unusable"] = "folded arm collides with the robot"; rows.append(r); print(json.dumps(r), flush=True); S.viewer and S.viewer.close(); continue
        r["head_cam_blocked_pct"] = cam_blocked(S, "head_camera"); r["lidar_front_blocked_pct"] = cam_blocked(S, "lidar_front_camera")
        r["bin_length_cm"], r["bin_headroom_cm"] = bin_room(S, mx)
        S.say(f"mount {mx * 100:+.0f} cm: walking loaded"); r["walk_worst_tilt_deg"], r["walk_speed_ratio"] = walk(S)
        S.say(f"mount {mx * 100:+.0f} cm: upright use (table reach, arm swing with load, walking while carrying)"); r.update(upright(S, mx))
        S.gains(120, 3)
        for posture, pq in (("crouch", legs.CROUCH), ("lie", legs.LIE)):
            S.say(f"mount {mx * 100:+.0f} cm: {posture}, tipping point"); S.legs_to(pq, 2.0); S.step(80); settle = {"qpos": d.qpos.copy()}
            tp = tipping(S, pq, settle); r[f"tip_{posture}_deg"] = min(tp.values()); r[f"tip_{posture}_by_dir"] = tp
            d.qpos[:] = settle["qpos"]; d.qvel[:] = 0; mujoco.mj_forward(m, d)
            gc = m.body("arm_grasp_center").id; m.body_mass[gc] = 1.5; margins = []
            R = d.xmat[S.tr].reshape(3, 3); c = d.xpos[S.tr].copy(); save = d.qpos.copy()
            for rr in (0.3, 0.45, 0.6):
                for ang in range(0, 360, 45):
                    t_ = c + R @ np.array([mx + rr * np.cos(np.radians(ang)), rr * np.sin(np.radians(ang)), 0]); t_[2] = 0.05
                    q, e = S.ik_pos(t_, seeds=4)
                    if e < 0.02: d.qpos[S.aq] = q; mujoco.mj_forward(m, d); margins.append(tb.support_margin(m, d, S))
            m.body_mass[gc] = 0.0; d.qpos[:] = save; mujoco.mj_forward(m, d)
            r[f"com_margin_1p5kg_{posture}_cm"] = round(100 * min(margins), 1) if margins else None
            if posture == "lie": S.say(f"mount {mx * 100:+.0f} cm: floor reach lying"); r["reach_m2"] = reach_area(S)
        rows.append(r); print(json.dumps(r), flush=True)
        if S.viewer is not None: S.viewer.close()
    ok = [r for r in rows if "unusable" not in r]
    print("\nMOUNT floor picking: reach m2 | tipping lying, worst dir (fwd/back) deg | walk tilt deg, speed | lidar blocked % | bin length, headroom cm")
    for r in rows:
        if "unusable" in r: print(f"MOUNT {r['mount_x_m'] * 100:+4.0f} cm: UNUSABLE ({r['unusable']})"); continue
        tl = r["tip_lie_by_dir"]; print(f"MOUNT {r['mount_x_m'] * 100:+4.0f} cm: {r['reach_m2']:.2f} | {r['tip_lie_deg']} ({tl['forward']}/{tl['back']}) | "
              f"{r['walk_worst_tilt_deg']}, {r['walk_speed_ratio']} | {r['lidar_front_blocked_pct']} | {r['bin_length_cm']}, {r['bin_headroom_cm']}", flush=True)
    print("MOUNT upright use: reach up front/side m | standing swing tilt 0.5/1.5 kg deg | carry-walk tilt deg, speed, fell | tipping crouched worst deg")
    for r in ok:
        print(f"MOUNT {r['mount_x_m'] * 100:+4.0f} cm: {r['reach_up_front_m']}/{r['reach_up_side_m']} | {r['stand_swing_tilt_0.5kg_deg']}/{r['stand_swing_tilt_1.5kg_deg']} | "
              f"{r['carry_walk_tilt_deg']}, {r['carry_walk_speed_ratio']}, {r['carry_walk_fell']} | {r['tip_crouch_deg']}", flush=True)
    if ok:
        def rank(key, rev):
            vals = sorted(ok, key=lambda r: (r[key] if r[key] is not None else -1), reverse=rev); return {id(r): k for k, r in enumerate(vals)}
        fl = [rank("reach_m2", True), rank("bin_length_cm", True), rank("walk_worst_tilt_deg", False), rank("lidar_front_blocked_pct", False)]
        up = [rank("reach_up_front_m", True), rank("stand_swing_tilt_1.5kg_deg", False), rank("carry_walk_tilt_deg", False), rank("tip_crouch_deg", True)]
        sf = {id(r): sum(x[id(r)] for x in fl) for r in ok}; su = {id(r): sum(x[id(r)] for x in up) for r in ok}
        bf = min(ok, key=lambda r: sf[id(r)]); bu = min(ok, key=lambda r: su[id(r)]); bb = min(ok, key=lambda r: sf[id(r)] + su[id(r)])
        print(f"MOUNT best for floor picking: {bf['mount_x_m'] * 100:+.0f} cm; best for upright use: {bu['mount_x_m'] * 100:+.0f} cm; best overall (equal weight): {bb['mount_x_m'] * 100:+.0f} cm (rank sums, equal weights; see the tables for what each gives up)", flush=True)
    (vs.HERE / "mount_sweep.json").write_text(json.dumps({"body": "Go1 stand-in (DimOS sim)", "rows": rows}, indent=1))
    import os; sys.stdout.flush(); os._exit(0)
if __name__ == "__main__": main()
