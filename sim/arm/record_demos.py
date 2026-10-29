#!/usr/bin/env python3
"""Record pick demonstrations for the training pipeline ([cLC], [qMA]). A privileged demonstrator (the scripted grasp,
told the item's true mask by the simulator) picks items from the library on generated floors; what is recorded is only
what the robot has: wrist images, arm joint states and the commanded joint targets, 10 Hz, from the moment look-closer
hands over (camera over the item) until the item is lifted. Viewer on by default.
Usage: record_demos.py --episodes 60 --seed 100 [--headless]"""
import argparse, json, sys, time
from pathlib import Path
import numpy as np, mujoco
sys.path.insert(0, str(Path(__file__).resolve().parent))
import vslice as vs, legs, items, pick, wrist_refine as wr, cluster_stops as cs, hud as hudm
OUT = Path(__file__).resolve().parents[2] / "data" / "demos"; IMG_W, IMG_H = 128, 96
import os; os.environ["HK_PICK_RECORD"] = "0"                    # this recorder keeps its own (privileged-teacher) episodes
def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--episodes", type=int, default=60); ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--headless", action="store_true"); ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--lib", type=int, default=18, help="items loaded in this run, sampled by seed from the training items that fit the jaws")
    a = ap.parse_args()
    import gpu_share; gpu_share.share()                              # shared with other standalone sims; a night goes alone ([jlr])
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True); rng = np.random.default_rng(a.seed)
    pool = items.grippable("train")                                  # every training item the jaws fit ([nuQ]: never held out)
    names = sorted(np.random.default_rng(a.seed + 1).choice(pool, min(a.lib, len(pool)), replace=False))   # this run's share:
                                                                     # all in one model costs GPU texture memory per renderer
    park = {n: [40.0 + 1.0 * i, 40.0, 0.3] for i, n in enumerate(names)}
    S = vs.Sim("sock", np.array([30.0, 30.0]), watch=not a.headless, with_obj=False, hud=hudm.Hud(), floor_tex=vs.HERE / "carpet_patch.png",
               local_floor=True, lib=[(n, n, park[n]) for n in names]); m, d = S.m, S.d; S.speed = 50.0
    lib_b = {n: m.body(n).id for n in names}
    def park_it(n):                                                  # parked items float out of the way: no contacts to solve
        b_ = lib_b[n]; j_ = m.jnt_qposadr[m.body_jntadr[b_]]; dd = m.jnt_dofadr[m.body_jntadr[b_]]
        m.body_gravcomp[b_] = 1.0; d.qpos[j_:j_ + 7] = park[n] + [1, 0, 0, 0]; d.qvel[dd:dd + 6] = 0
    for n in names: park_it(n)
    mujoco.mj_forward(m, d)
    S.state("LIE DOWN"); S.gains(120, 3); S.legs_to(legs.LIE, 2.0); S.step(60)
    mount = d.xpos[m.body("arm_base_link").id].copy(); q_ready = S.ready_pose(); S.arm_to_safe(q_ready, 1.5); q_bin = S.bin_pose()
    lie_q, lie_v = d.qpos[:19].copy(), d.qvel[:18].copy()           # the lying robot creeps a little with every arm move:
    def reset_robot():                                              # put it back each episode so reach stays where the map says
        d.qpos[:19] = lie_q; d.qvel[:18] = 0; d.ctrl[:12] = legs.LIE; mujoco.mj_forward(m, d)
    seg = mujoco.Renderer(m, 480, 640); seg.enable_segmentation_rendering()
    gl = m.jnt_qposadr[m.joint("arm_gripper_left_joint").id]; kept = 0
    for ep in range(a.episodes):
        n = names[rng.integers(len(names))]; b = m.body(n).id; ja = m.jnt_qposadr[m.body_jntadr[b]]; da = m.jnt_dofadr[m.body_jntadr[b]]
        reset_robot(); Rr = d.xmat[S.tr].reshape(3, 3); base = d.xpos[S.tr][:2].copy()
        items.set_mass(m, n, rng); m.body_gravcomp[b] = 0.0
        cx, cy = cs.OFFS[rng.integers(len(cs.OFFS))] + rng.normal(0, 0.01, 2); qr = rng.normal(size=4); qr /= np.linalg.norm(qr)
        xy = base + Rr[:2, :2] @ np.array([cx, cy]); d.qpos[ja:ja + 7] = [xy[0], xy[1], 0.12, *qr]; d.qvel[da:da + 6] = 0
        fl = S.new_floor(a.seed * 1000 + ep, focus_xy=xy)                        # a rug's edge, if any, near the item ([N4z])
        mujoco.mj_forward(m, d); S.step(200); true = d.xpos[b][:2].copy()
        out_dir = true - d.qpos[:2]; out_dir /= np.linalg.norm(out_dir)
        q = S.look_down(true - out_dir * pick.IN_VIEW, out_dir, h_cam=pick.LOOK_H + 0.14)       # where look-closer hands over
        if q is None: print(json.dumps({"ep": ep, "item": n, "skipped": "no look pose"}), flush=True); park_it(n); pick.stow(S, q_ready); continue
        S.arm_to(q, 1.5); S.step(40); v = S.capture(filament=False)   # the teacher reads the true outline, not the image
        seg.update_scene(d, "arm_wrist"); sg = seg.render()[..., 0]
        mask = np.isin(sg, [g for g in range(m.ngeom) if m.body_rootid[m.geom_bodyid[g]] == m.body_rootid[b]])
        ref = wr.ref_from_mask(v, mask)
        if ref is None or not ref["fits_jaws"]:
            print(json.dumps({"ep": ep, "item": n, "skipped": "not in view" if ref is None else f"too wide ({ref['grip_width_cm']:.1f} cm)"}), flush=True)
            park_it(n); mujoco.mj_forward(m, d); pick.stow(S, q_ready); continue
        frames = {"img": [], "state": [], "action": []}; stop = {"v": False}
        def rec(S_):
            if stop["v"]: return
            frames["img"].append(S_.small_image())                   # the policy camera, with this episode's floor
            frames["state"].append(np.r_[S_.d.qpos[S_.aq], S_.d.qpos[gl]].astype(np.float32)); frames["action"].append(S_.d.ctrl[12:19].astype(np.float32).copy())
        orig_state = S.state
        def state(name):
            if name == "INTO BIN": stop["v"] = True; stop["z"] = float(d.xpos[b][2])   # the demonstration ends once the item is lifted
            orig_state(name)
        S.state = state; S.rec = rec
        fail, held = pick.grasp_to_bin(S, ref, q_ready, q_bin)
        S.rec = None; S.state = orig_state
        lifted = bool(stop.get("z", 0.0) > 0.08)                   # truth: was it off the floor when the demonstration ended?
        np.savez_compressed(out / f"ep_{a.seed}_{ep:04d}.npz", img=np.array(frames["img"]), state=np.array(frames["state"]), action=np.array(frames["action"]),
                            item=n, floor=fl, success=lifted, fail=str(fail))
        kept += lifted; print(json.dumps({"ep": ep, "item": n, "floor": fl, "frames": len(frames["img"]), "success": lifted, "fail": fail}), flush=True)
        park_it(n); mujoco.mj_forward(m, d); pick.stow(S, q_ready)
    print(f"DEMOS {kept}/{a.episodes} successful episodes recorded in {out}", flush=True)
    import os; os._exit(0)
if __name__ == "__main__": main()
