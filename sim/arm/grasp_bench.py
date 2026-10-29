#!/usr/bin/env python3
"""Tight loop for making the scripted grasp reliable: robot lies down once, then items appear one at a time at random
spots inside the arm's reach band with random rotation. Each trial: look closer (starting from a guess ~5 cm off, like
Phase 1), refine x2, grip across the narrowest side, lift, carry, release over the bin. No sweep, no walking, no VLM.
Viewer on by default. Usage: grasp_bench.py [--trials 20] [--speed 2] [--headless] [--seed 0]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import argparse, json, sys, time
import numpy as np, mujoco
sys.path.insert(0, str(_HK / "sim" / "arm"))
import vslice as vs, legs, payload_test as pt, wrist_refine as wr, cluster_stops as cs, hud as hudm, items, pick
KINDS = {  # name: (shape, half extents, mass range kg, rgba)
    "sock": ("box", [0.09, 0.035, 0.008], (0.03, 0.06), [0.25, 0.35, 0.75, 1]),
    "block": ("box", [0.03, 0.03, 0.03], (0.03, 0.15), [0.9, 0.2, 0.2, 1]),
    "brick": ("box", [0.05, 0.025, 0.02], (0.05, 0.3), [0.95, 0.75, 0.1, 1]),
    "puck": ("cylinder", [0.035, 0.035, 0.015], (0.03, 0.12), [0.2, 0.7, 0.3, 1])}
PARK = np.array([30.0, 30.0]); LOOK_H = 0.18                 # look from 18 cm: fingertips clear of any item we would pick
IN_VIEW = 0.08                                               # camera this far short of the item, toward the robot (jaws fill the lower image)
FRONT_MIN_X = 0.36                                           # robot frame: a front item lies wholly beyond this (the reach maps' body box)
FLAG_TOL = 0.15                                              # a flag is at most 10 cm off (look-closer [hwo]); a blob further than this from it is something else
def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--trials", type=int, default=20); ap.add_argument("--speed", type=float, default=2.0)
    ap.add_argument("--headless", action="store_true"); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--items", default="all", help="comma list from the item library, or 'all' (movable items)")
    ap.add_argument("--zone", default="all", choices=["all", "front"], help="front: items only where a head-on approach leaves them ([ZMc])")
    ap.add_argument("--park-sigma", type=float, default=0.04, help="front zone: parking error (m, s.d.) around the front target")
    ap.add_argument("--max-lean", type=float, default=None, help="deg the grasp may lean off vertical ([Hwc]); default pick.MAX_LEAN"); a = ap.parse_args()
    if a.max_lean is not None: pick.MAX_LEAN = a.max_lean
    rng = np.random.default_rng(a.seed)
    names = ([n for n in items.ITEMS if items.ITEMS[n][0] != "cloth"] if a.items == "all" else
             [n for n in items.ITEMS if items.ITEMS[n][0] != "cloth" and items.ITEMS[n][3] != "auto"] if a.items == "core" else a.items.split(","))   # core: the set before the clutter was added   # cloth: no single body to place yet
    park = {n: [PARK[0] + 10 + 1.0 * i, PARK[1] + 10, 0.3] for i, n in enumerate(names)}
    S = vs.Sim("sock", PARK, watch=not a.headless, with_obj=False, hud=hudm.Hud(), floor_tex=vs.HERE / "carpet_patch.png", local_floor=True,
               lib=[(n, n, park[n]) for n in names]); m, d = S.m, S.d; S.speed = a.speed
    bodies = {k: m.body(k).id for k in names}
    jadr = {k: m.jnt_qposadr[m.body_jntadr[b]] for k, b in bodies.items()}; dadr = {k: m.jnt_dofadr[m.body_jntadr[b]] for k, b in bodies.items()}
    S.state("LIE DOWN"); S.gains(120, 3); S.legs_to(legs.LIE, 2.0); S.step(60)
    mount = d.xpos[m.body("arm_base_link").id].copy(); q_ready = S.ready_pose(); S.arm_to_safe(q_ready, 1.5)
    q_bin = S.bin_pose(); S.say(f"bin release pose: {'found, ' + str(S._bin_info) if q_bin is not None else 'NONE'}")
    lie_q = d.qpos[:19].copy(); cells = cs.OFFS                                                # reach cells (robot frame, 5 cm margin)
    if a.zone == "front":                                                                      # the front zone of the reach map for this lean
        lean_map = min((0, 15, 30, 45), key=lambda t: abs(t - pick.MAX_LEAN)); RM = json.load(open(vs.HERE / f"reach_lying_tilt{lean_map}.json"))
        cells = np.array([(x, y) for i, y in enumerate(RM["ys"]) for j, x in enumerate(RM["xs"]) if RM["reach"][i][j] and x >= 0.36])
        front_target = cs.front_offset(pick.MAX_LEAN, a.park_sigma); S.say(f"front target {front_target}, parking scatter {a.park_sigma * 100:.0f} cm")
    def reset_robot(): d.qpos[:19] = lie_q; d.qvel[:18] = 0; d.ctrl[:12] = legs.LIE; mujoco.mj_forward(m, d)   # the lying body creeps: reset
    results = []
    for t in range(a.trials):
        kind = names[rng.integers(len(names))]; b = bodies[kind]; items.set_mass(m, kind, rng); expect, why = items.ITEMS[kind][3], items.ITEMS[kind][4]
        reset_robot(); Rr = d.xmat[S.tr].reshape(3, 3); base = d.xpos[S.tr][:2].copy()
        if a.zone == "front": cx, cy = front_target + rng.normal(0, a.park_sigma, 2)       # where a head-on approach leaves it, parking scatter
        else: cx, cy = cells[rng.integers(len(cells))] + rng.uniform(-0.0125, 0.0125, 2)
        xy = base + Rr[:2, :2] @ np.array([cx, cy])
        centre = lambda: items.item_points(m, d, b)[:, :2].mean(0)              # where the item is: the middle of its geometry
        fwd = Rr[:2, 0]                                                          # away from the robot
        near = lambda sh: ((items.item_points(m, d, b)[:, :2] + sh - base) @ Rr[:2, :2])[:, 0].min()   # nearest point, robot frame
        qr = rng.normal(size=4); qr /= np.linalg.norm(qr)                       # dropped from 12 cm in any orientation (a rope: any
        if items.ITEMS[kind][0] == "rope": h_ = rng.uniform(0, np.pi); qr = np.array([np.cos(h_), 0, 0, np.sin(h_)])   # turn, flat)
        d.qpos[jadr[kind]:jadr[kind] + 7] = [xy[0], xy[1], 0.12, *qr]; d.qvel[dadr[kind]:dadr[kind] + 6] = 0
        mujoco.mj_forward(m, d); d.qpos[jadr[kind]:jadr[kind] + 2] += xy - centre(); mujoco.mj_forward(m, d)   # its middle on the spot
        if a.zone == "front" and near(0.0) < FRONT_MIN_X:                         # a big item sits further out: the robot parks
            d.qpos[jadr[kind]:jadr[kind] + 2] += fwd * (FRONT_MIN_X - near(0.0) + 0.01); mujoco.mj_forward(m, d)   # with it in front
        S.step(200); shift = xy - centre()                                      # it settles as it lands; then its middle (not its
        if a.zone == "front" and near(shift) < FRONT_MIN_X: shift = shift + fwd * (FRONT_MIN_X - near(shift) + 0.01)   # root) goes
                                                                                # back on the spot, or as near as the robot allows
        d.qpos[jadr[kind]:jadr[kind] + 2] += shift; d.qpos[jadr[kind] + 2] += 0.01   # else drop it again (never slide it into the
        mujoco.mj_forward(m, d); S.step(60)                                      # robot: that knocked the robot over)
        reset_robot(); S.step(20)                                                # whatever placing it did, the robot starts lying still
        S.ob = b; true0 = centre(); S.max_tilt = 0.0; S._tip_logged = False
        blowups0 = int(d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number)       # the solver resetting itself mid-trial
        pos_robot = (Rr[:2, :2].T @ (true0 - base)).round(3).tolist(); tw = items.true_width(items.item_points(m, d, b))[0]   # for analysis
        expect, why = items.judge(m, d, kind)                                   # scanned items: judged as they landed
        guess = true0 + rng.normal(0, 0.05, 2); r = {"trial": t, "kind": kind, "expect": expect, "why": why, "mass_g": round(m.body_mass[b] * 1000), "fail": None}
        r["floor"] = S.new_floor(a.seed * 1000 + t, focus_xy=true0); r["pos_robot"] = pos_robot; r["true_w_cm"] = round(tw * 100, 1)
        S.say(f"trial {t + 1}/{a.trials}: {kind}, {r['mass_g']} g on {r['floor']}")
        ref, notes = pick.look_closer(S, guess); r["look"] = notes
        if ref is None: r["fail"] = "not seen"
        else:
            r["refine_err_cm"] = round(float(np.linalg.norm(ref["xy"] - centre())) * 100, 1); r["grip_width_cm"] = round(ref["grip_width_cm"], 1)
            r["label"] = ref.get("label"); z0 = d.xpos[b][2]
            fail, held = pick.pick_with_tries(S, ref, q_ready, q_bin, relook=lambda xy: pick.look_closer(S, xy))   # [m62]
            r["fail"] = fail; r["robot_says"] = fail or "binned"; r["tries"] = getattr(S, "tries", 1); S.tries = 1
            r["contact_stop"] = getattr(S, "contact_stop", None); S.contact_stop = None
            r["weighed_g"] = None if getattr(S, "weighed_kg", None) is None else round(S.weighed_kg * 1000); r["true_g"] = round(float(m.body_mass[m.body_rootid == b].sum()) * 1000); S.weighed_kg = None
            r["held_gap_mm"] = None if held is None else round(held * 2000, 1); r["track_err_cm"] = getattr(S, "track_err_cm", None); S.track_err_cm = None
            tray = d.xpos[m.body("tray").id]; Rt = d.xmat[S.tr].reshape(3, 3); rel = Rt.T @ (np.r_[centre(), items.item_points(m, d, b)[:, 2].min()] - tray)   # its middle, its bottom
            in_bin = bool(abs(rel[0]) < 0.15 and abs(rel[1]) < 0.11 and -0.01 < rel[2] < 0.12)   # scored by where it really is
            r["end_rel_bin_cm"] = [round(float(v) * 100, 1) for v in rel]                     # where it ended: its middle along and across
                                                                                             # the bin from its centre, its bottom above the tray ([oZb])
            if in_bin: r["fail"] = None
            elif fail is None: r["fail"] = "missed bin"
        r["lean_deg"] = getattr(S, "lean_used_deg", None); S.lean_used_deg = None
        if ref is not None: pick.annotate_last(S, item=kind, truth_in_bin=bool(r["fail"] is None), floor=str(r.get("floor")))
        r["max_tilt_deg"] = round(S.max_tilt, 1)
        if int(d.warning[mujoco.mjtWarning.mjWARN_BADQACC].number) > blowups0:  # the physics blew up and MuJoCo reset the state:
            r["fail"] = "sim unstable"; r["expect"] = "excluded"; S.max_tilt = 0.0  # not the robot's doing; set aside, not scored
            d.qpos[0:2] = base; d.qpos[2] = 0.13; d.qpos[3:7] = [1, 0, 0, 0]; d.qpos[7:19] = legs.LIE; d.qvel[:] = 0; d.ctrl[:12] = legs.LIE
            d.qpos[S.aq] = q_ready; d.ctrl[12:18] = q_ready; mujoco.mj_forward(m, d); S.step(200); lie_q = d.qpos[:19].copy()
        if S.max_tilt > 25:
            r["fail"] = "TIPPED OVER"                                   # put the robot back down lying, arm raised, and carry on
            d.qpos[0:2] = base; d.qpos[2] = 0.13; d.qpos[3:7] = [1, 0, 0, 0]; d.qpos[7:19] = legs.LIE; d.qvel[:] = 0; d.ctrl[:12] = legs.LIE
            d.qpos[S.aq] = q_ready; d.ctrl[12:18] = q_ready; mujoco.mj_forward(m, d); S.step(200); lie_q = d.qpos[:19].copy()
        r["ok"] = r["fail"] is None; results.append(r)
        r["correct"] = r["ok"] if expect == "bin" else r["fail"] not in (None, "dropped lifting", "missed bin", "lost on the way to the bin (may be in it)", "TIPPED OVER")   # review items: right when left where they were
        n_ok = sum(x["ok"] for x in results); S.say(f"trial {t + 1}: {'OK' if r['ok'] else 'FAIL - ' + r['fail']}   ({n_ok}/{len(results)} ok)")
        print(json.dumps(r), flush=True)
        d.qpos[jadr[kind]:jadr[kind] + 3] = park[kind]; d.qvel[dadr[kind]:dadr[kind] + 6] = 0; mujoco.mj_forward(m, d)
        pick.stow(S, q_ready)
    n = len(results); ok = sum(x["ok"] for x in results); from collections import Counter
    nb = [x for x in results if x["expect"] == "bin"]; nr = [x for x in results if x["expect"] == "review"]
    print(f"OUTCOMES pickable {sum(x['ok'] for x in nb)}/{len(nb)} binned; review items {sum(x['correct'] for x in nr)}/{len(nr)} correctly left", flush=True)
    print(f"SUMMARY {ok}/{n} ok; failures: {dict(Counter(x['fail'] for x in results if x['fail']))}; by kind: "
          + ", ".join(f"{k} {sum(x['ok'] for x in results if x['kind'] == k)}/{sum(x['kind'] == k for x in results)}" for k in names), flush=True)
    (vs.HERE / "grasp_bench.json").write_text(json.dumps(results, indent=1))
    import os; sys.stdout.flush(); os._exit(0)                               # skip GL teardown (it segfaults with the viewer open)
if __name__ == "__main__": main()
