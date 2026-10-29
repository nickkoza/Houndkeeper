#!/usr/bin/env python3
"""Several items, one head-on stop per find ([S7o]; grouping finds into one stop is backlogged). Sweep twice, detect
every new thing, then nearest first: walk up facing it, park it at the front target (cluster_stops.front_offset),
lie down, look closer, name it with the local VLM, pick it with the shared pick routine (sim/arm/pick.py, as on DimOS),
stow, stand, next. HUD: state machine, power, battery (battery_model estimates).
Usage: vslice_multi.py [--watch] [--option direct|boost|own] [--cradle]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import argparse, json, sys, time
import numpy as np, mujoco
sys.path.insert(0, str(_HK / "sim" / "arm"))
import vslice as vs, phase1_sweep as p1, legs, payload_test as pt, cluster_stops as cs, hud as hudm, pick
ITEMS = [("sock", (0.35, -0.20)), ("toy", (0.62, -0.24)), ("sock", (-0.55, 0.42))]
def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--headless", action="store_true"); ap.add_argument("--option", default="direct")
    ap.add_argument("--cradle", action="store_true"); ap.add_argument("--fresh-baseline", action="store_true"); ap.add_argument("--video", default=None); a = ap.parse_args()
    tex = p1.HERE / "carpet_patch.png"; start = (-1.3, -1.3, 0.0); res = {"items": []}
    cache = vs.HERE / "_baseline_sweep.npz"                      # the clean-floor sweep is identical every run: sweep once, reuse
    if cache.exists() and not a.fresh_baseline:
        z = np.load(cache); base = (z["rgb"], z["dist"], z["tall"]); print("baseline: reusing cached clean-floor sweep", flush=True)
    else:
        B = vs.Sim(ITEMS[0][0], np.array(ITEMS[0][1]), floor_tex=tex, with_obj=False, start=start, watch=not a.headless); B.speed = 3.0
        B.hud = hudm.Hud(a.option, a.cradle); B.state("SWEEP"); B.say("Baseline: sweep the clean floor (last night's picture)."); base = p1.sweep(B)
        if B.viewer is not None: B.viewer.close()
        np.savez_compressed(cache, rgb=base[0], dist=base[1], tall=base[2])
    H = hudm.Hud(a.option, a.cradle)
    S = vs.Sim(ITEMS[0][0], np.array(ITEMS[0][1]), video=a.video, floor_tex=tex, start=start, watch=not a.headless, extra=ITEMS[1:], hud=H); m, d = S.m, S.d
    truth0 = [d.xpos[b][:2].copy() for b in S.obs]
    S.state("SWEEP"); S.say("Sweep in rows, arm stowed; build tonight's floor picture."); S.speed = 3.0
    night = p1.sweep(S); S.speed = 1.0; S._w0 = None
    S.state("DETECT"); flags, cand = p1.detect_all(base, night); S.say(f"{len(flags)} new things flagged."); S.step(200)
    S.state("PLAN STOPS")
    fxy = np.array([(f["x"], f["y"]) for f in flags]); off = cs.front_offset(pick.MAX_LEAN) - [cs.LIE_SETTLE_M, 0]   # where a find should end up (the body settles back lying)
    def free(x, y, th, skip):                             # inside the patch, and not lying on top of any other flagged item
        if abs(x) > 1.25 or abs(y) > 1.25: return False
        c_, s_ = np.cos(th), np.sin(th); dd = np.delete(fxy, skip, 0) - [x, y]; u = c_ * dd[:, 0] + s_ * dd[:, 1]; v = -s_ * dd[:, 0] + c_ * dd[:, 1]
        return not np.any((np.abs(u) < cs.FOOT[0] + 0.05) & (np.abs(v) < cs.FOOT[1] + 0.05))
    def stop_for(fi, frm):                                # head-on: facing the find, parked so it lies at the front target ([S7o])
        base_th = np.arctan2(*(fxy[fi] - frm)[::-1])
        for dth in (0.0, 0.35, -0.35, 0.7, -0.7):         # turn a little if the straight approach would stand on another find
            th = base_th + dth; Rz = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); xy = fxy[fi] - Rz @ off
            if free(xy[0], xy[1], th, fi): return xy, th
        return None
    order, here = [], d.qpos[0:2].copy(); left = list(range(len(flags)))
    while left:                                           # nearest find first
        fi = min(left, key=lambda i: np.linalg.norm(fxy[i] - here)); left.remove(fi); order.append(fi); here = fxy[fi]
    S.say(f"{len(flags)} flags -> {len(flags)} stops, one per find, each approached head-on."); S.step(200)
    res["flags"] = len(flags); res["stops"] = 0; res["unreachable"] = 0
    handled = []                                          # objects dealt with tonight: never twice ([XfI])
    for k, fi in enumerate(order):
        f = flags[fi]; st = stop_for(fi, d.qpos[0:2].copy())
        if st is None: res["unreachable"] += 1; res["items"].append({"flag": fi, "human_review": True, "why": "no free stop in front of it"}); continue
        res["stops"] += 1; S.state("WALK TO STOP"); S.say(f"Stop {k + 1}/{len(order)}: head-on to flag {fi}.")
        S.walk_to(st[0], st[1]); S.state("LIE DOWN"); S.legs_to(legs.LIE, 2.0)
        q_ready = S.ready_pose(); S.arm_to_safe(q_ready, 1.8); q_bin = S.bin_pose()
        ref, notes = pick.look_closer(S, np.array([f["x"], f["y"]]))                  # the shared pick routine, as on DimOS
        looks = [cv for cv, rf in getattr(S, "last_looks", []) if rf is not None]
        name = "nothing" if ref is None else ref.get("label", "something")      # the detector that located it named it ([JKS])
        S.say(f"Looked closer: {name}"); item = {"flag": fi, "name": name, "look": notes}
        same = None if ref is None else pick.same_as(ref["xy"], handled)
        if same is not None: item["outcome"] = f"same as flag {same}"; res["items"].append(item)
        elif ref is None or name.lower().startswith("nothing"): res["items"].append(item)
        else:
            S.ob = min(S.obs, key=lambda b: np.linalg.norm(d.xpos[b][:2] - ref["xy"]))
            fail, _ = pick.pick_with_tries(S, ref, q_ready, q_bin, relook=lambda xy: pick.look_closer(S, xy))   # [m62]
            item["outcome"] = fail or "binned"; item["tries"] = getattr(S, "tries", 1); handled.append((fi, ref["xy"]))
            if fail and not fail.startswith("lost"): item["human_review"] = True; S.say(f"{fail}: flag for human review.")
            res["items"].append(item)
        S.state("STOW"); S.arm_to_safe(np.array(pt.STOW[:6]), 3.5, grip=0.0); S.state("STAND UP"); S.legs_to(legs.STAND, 2.5)
    S.state("DONE"); S.walk_to(d.qpos[0:2] + [0.5, 0.0], 0.0, max_s=4)
    Rt = d.xmat[S.tr].reshape(3, 3); tray = d.xpos[m.body("tray").id]
    for b, t0 in zip(S.obs, truth0):
        rel = Rt.T @ (d.xpos[b] - tray); res["items"].append({"body": mujoco.mj_id2name(m, 1, b), "start_xy": [round(float(v), 2) for v in t0],
            "in_tray_at_end": bool(abs(rel[0]) < 0.15 and abs(rel[1]) < 0.11 and -0.01 < rel[2] < 0.10)})
    res["air_pack_used_Wh"] = round(H.used_air, 2); res["air_pack_pct_left"] = round(100 * (1 - H.used_air / H.air_Wh), 2)
    if H.arm_Wh: res["arm_pack_used_Wh"] = round(H.used_arm, 2)
    res["sim_minutes"] = round(S.t / 60, 1); res["max_tilt_deg"] = round(S.max_tilt, 1)
    print(json.dumps(res)); (vs.HERE / "vslice_multi.json").write_text(json.dumps({"result": res, "log": S.log}, indent=1))
    if a.video: vs.write_video(S, a.video)
if __name__ == "__main__": main()
