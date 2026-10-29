#!/usr/bin/env python3
"""Small, fast sim for the grip-width estimate ([ZMc]): items settle where a head-on approach leaves them, the robot
runs only look-closer (no grasp), and every look (image, mask, camera pose) is saved with the item's true width across
the jaws' height band and its true height. Width estimators are then compared offline on the saved looks, in seconds.
Usage: width_bench.py [--trials 40] [--seed 0] [--items a,b] [--headless] [--out data/eval/width_looks.pkl]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import argparse, pickle, sys
import numpy as np, mujoco
sys.path.insert(0, str(_HK / "sim" / "arm"))
import vslice as vs, legs, cluster_stops as cs, hud as hudm, items, pick, bench_io

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--trials", type=int, default=40); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--items", default="all"); ap.add_argument("--headless", action="store_true")
    ap.add_argument("--out", default=str(_HK / "data" / "eval" / "width_looks.pkl")); a = ap.parse_args()
    rng = np.random.default_rng(a.seed)
    names = [n for n in items.ITEMS if items.ITEMS[n][0] != "cloth"] if a.items == "all" else a.items.split(",")
    PARK = np.array([30.0, 30.0]); park = {n: [PARK[0] + 10 + 1.0 * i, PARK[1] + 10, 0.3] for i, n in enumerate(names)}
    S = vs.Sim("sock", PARK, watch=not a.headless, with_obj=False, hud=hudm.Hud(), floor_tex=vs.HERE / "carpet_patch.png", local_floor=True,
               lib=[(n, n, park[n]) for n in names]); m, d = S.m, S.d; S.speed = 4.0
    bodies = {k: m.body(k).id for k in names}
    jadr = {k: m.jnt_qposadr[m.body_jntadr[b]] for k, b in bodies.items()}; dadr = {k: m.jnt_dofadr[m.body_jntadr[b]] for k, b in bodies.items()}
    S.gains(120, 3); S.legs_to(legs.LIE, 2.0); S.step(60)
    mount = d.xpos[m.body("arm_base_link").id].copy(); q_ready = S.ready_pose(); S.arm_to_safe(q_ready, 1.5)
    lie_q = d.qpos[:19].copy(); front = cs.front_offset(pick.MAX_LEAN); out = []; bench_io.start(a.out)
    for t in range(a.trials):
        kind = names[t % len(names)] if a.items != "all" else names[rng.integers(len(names))]; b = bodies[kind]
        d.qpos[:19] = lie_q; d.qvel[:18] = 0; d.ctrl[:12] = legs.LIE; mujoco.mj_forward(m, d)
        Rr = d.xmat[S.tr].reshape(3, 3); xy = d.xpos[S.tr][:2] + Rr[:2, :2] @ (front + rng.normal(0, 0.04, 2))
        qr = rng.normal(size=4); qr /= np.linalg.norm(qr)
        d.qpos[jadr[kind]:jadr[kind] + 7] = [xy[0], xy[1], 0.12, *qr]; d.qvel[dadr[kind]:dadr[kind] + 6] = 0
        mujoco.mj_forward(m, d); S.step(200); floor = S.new_floor(a.seed * 1000 + t, focus_xy=d.xpos[b][:2])
        w, h = items.true_width(items.item_points(m, d, b)); true_xy = d.xpos[b][:2].copy()   # the library's one truth
        ref, notes = pick.look_closer(S, true_xy + rng.normal(0, 0.05, 2))
        item_g = np.flatnonzero(m.body_rootid[m.geom_bodyid] == b)          # the item's own pixels in each look (the sim's truth)
        looks = [({k: cv[k] for k in ("img", "robot", "c", "R", "fovy", "aim", "ir_frames", "img_r", "baseline") if k in cv} | ({"truth": np.isin(cv["seg"], item_g)} if "seg" in cv else {}),
                  None if rf is None else {k: rf[k] for k in ("xy", "mask", "grip_width_cm", "label")})
                 for cv, rf in getattr(S, "last_looks", [])]
        r = {"trial": t, "kind": kind, "floor": floor, "true_w": w, "true_h": h, "true_xy": true_xy, "notes": notes, "looks": looks,
             "tier": items.grasp_tier(kind, d.xquat[b].copy()),                  # [QyZ], as it landed: what the verdict should be
             "ref": None if ref is None else {"xy": ref["xy"], "z_est": ref.get("z_est"), "grip_width_cm": ref["grip_width_cm"],
                                              **{k: ref.get(k) for k in ("fits_jaws", "length_cm", "breadth_cm", "width_rule")}}}
        out.append(r); bench_io.append(a.out, r)
        est = "-" if ref is None else f"{ref['grip_width_cm']:.1f}"
        size = "-" if ref is None else f"{ref.get('length_cm') or 0:.0f}x{ref.get('breadth_cm') or 0:.0f} fits_jaws {ref.get('fits_jaws')}"
        print(f"trial {t}: {kind:13s} {r['tier']:7s} true w {w * 100:4.1f} h {h * 100:4.1f} cm, estimate {est} cm, {size}", flush=True)
        d.qpos[jadr[kind]:jadr[kind] + 3] = park[kind]; d.qvel[dadr[kind]:dadr[kind] + 6] = 0; mujoco.mj_forward(m, d)
        pick.stow(S, q_ready)
    import os; sys.stdout.flush(); os._exit(0)


if __name__ == "__main__": main()
