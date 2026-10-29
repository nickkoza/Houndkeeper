#!/usr/bin/env python3
"""Score a trained pick policy against the scripted grasp on held-out seeds ([qMA]). Each episode starts where
look-closer hands over (camera over the item), exactly as the demonstrations did; then either the policy drives the arm
(action chunks, n_action_steps executed at 10 Hz) or the scripted grasp runs with its own perception (look closer from
the wrist camera, no privileged information). Success: the item is off the floor (> 8 cm) at the end. Sim truth scores.
Usage: eval_policy.py --checkpoint <path> --episodes 30 --seed 900 [--headless] [--which policy,scripted] [--floors heldout]"""
import argparse, json, pickle, subprocess, sys
from pathlib import Path
import numpy as np, mujoco
sys.path.insert(0, str(Path(__file__).resolve().parent))
import vslice as vs, legs, items, pick, wrist_refine as wr, cluster_stops as cs, hud as hudm
HK = Path(__file__).resolve().parents[2]
def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--checkpoint", required=False); ap.add_argument("--episodes", type=int, default=30)
    ap.add_argument("--seed", type=int, default=900); ap.add_argument("--headless", action="store_true"); ap.add_argument("--which", default="policy,scripted")
    ap.add_argument("--steps", type=int, default=110); ap.add_argument("--query-every", type=int, default=10)
    ap.add_argument("--action-space", default="joint", choices=["joint", "ee"])
    ap.add_argument("--items", default="train", choices=["train", "heldout"], help="items to judge on ([nuQ]): heldout never trained on")
    ap.add_argument("--floors", default="train", choices=["train", "heldout"], help="photographed floors to judge on ([N4z]): heldout never trained on")
    ap.add_argument("--grey-floor-policy", action="store_true", help="policy camera keeps the placeholder floor (how demos before 25 Sep 03:40 were recorded)")
    a = ap.parse_args(); which = a.which.split(",")
    import floors; floors.SPLIT = a.floors                         # which photographed floors this run judges on
    import gpu_share; gpu_share.share()                              # shared with other standalone sims; a night goes alone ([jlr])
    srv = None
    if "policy" in which:
        client = pick.PolicyClient(a.checkpoint); srv = client.p; print("policy", client.info, flush=True)
    pool = items.grippable(a.items)                                  # held-out items only when asked for ([nuQ])
    names = sorted(np.random.default_rng(a.seed + 1).choice(pool, min(18, len(pool)), replace=False)); rng = np.random.default_rng(a.seed)
    park = {n: [40.0 + 1.0 * i, 40.0, 0.3] for i, n in enumerate(names)}
    S = vs.Sim("sock", np.array([30.0, 30.0]), watch=not a.headless, with_obj=False, hud=hudm.Hud(), floor_tex=vs.HERE / "carpet_patch.png",
               local_floor=True, lib=[(n, n, park[n]) for n in names]); m, d = S.m, S.d; S.speed = 50.0
    if a.grey_floor_policy: S._small = mujoco.Renderer(m, 96, 128); S._small_frozen = True   # made before any floor: grey, never updated
    def park_it(n):
        b_ = m.body(n).id; j_ = m.jnt_qposadr[m.body_jntadr[b_]]; dd = m.jnt_dofadr[m.body_jntadr[b_]]
        m.body_gravcomp[b_] = 1.0; d.qpos[j_:j_ + 7] = park[n] + [1, 0, 0, 0]; d.qvel[dd:dd + 6] = 0
    for n in names: park_it(n)
    mujoco.mj_forward(m, d); S.gains(120, 3); S.legs_to(legs.LIE, 2.0); S.step(60)
    mount = d.xpos[m.body("arm_base_link").id].copy(); q_ready = S.ready_pose(); S.arm_to_safe(q_ready, 1.5); q_bin = S.bin_pose()
    lie_q = d.qpos[:19].copy(); small = mujoco.Renderer(m, 96, 128)
    def reset_robot(): d.qpos[:19] = lie_q; d.qvel[:18] = 0; d.ctrl[:12] = legs.LIE; mujoco.mj_forward(m, d)
    gl = m.jnt_qposadr[m.joint("arm_gripper_left_joint").id]; res = {w: [] for w in which}
    for ep in range(a.episodes):
        n = names[rng.integers(len(names))]; b = m.body(n).id; ja = m.jnt_qposadr[m.body_jntadr[b]]; da = m.jnt_dofadr[m.body_jntadr[b]]
        mass = rng.uniform(*items.ITEMS[n][2]); floor_seed = a.seed * 1000 + ep
        cx, cy = cs.OFFS[rng.integers(len(cs.OFFS))] + rng.normal(0, 0.01, 2); qr = rng.normal(size=4); qr /= np.linalg.norm(qr)
        for w in which:                                                    # same item, pose, floor and start for each controller
            reset_robot(); Rr = d.xmat[S.tr].reshape(3, 3); base = d.xpos[S.tr][:2].copy()
            m.body_mass[b] = mass; m.body_gravcomp[b] = 0.0
            xy = base + Rr[:2, :2] @ np.array([cx, cy]); S.new_floor(floor_seed, focus_xy=xy); d.qpos[ja:ja + 7] = [xy[0], xy[1], 0.12, *qr]; d.qvel[da:da + 6] = 0
            mujoco.mj_forward(m, d); S.step(200); true = d.xpos[b][:2].copy(); od = true - d.qpos[:2]; od /= np.linalg.norm(od)
            q = S.look_down(true - od * pick.IN_VIEW, od, h_cam=pick.LOOK_H + 0.14)
            if q is None: res[w].append(None); park_it(n); continue
            S.arm_to(q, 1.5); S.step(40)
            closest = [9.0]
            if w == "policy":
                cmd0 = S.command
                def command(q6, jaw, _c=cmd0):
                    _c(q6, jaw); closest[0] = min(closest[0], float(np.linalg.norm(S.grasp_point()[:2] - d.xpos[b][:2])))
                S.command = command
                pick.policy_pick(S, client, ee=(a.action_space == "ee"), steps=a.steps, query_every=a.query_every)
                S.command = cmd0
            else:
                g = np.array([0.0, 0.0]); v = S.capture(); v["aim"] = true + rng.normal(0, 0.03, 2)
                ref, notes = pick.look_closer(S, v["aim"])
                if ref is not None:
                    orig = S.state; stop = {}
                    def state(nm, o=orig):
                        if nm == "INTO BIN": stop["z"] = float(d.xpos[b][2]); raise StopIteration
                        o(nm)
                    S.state = state
                    try: pick.grasp_to_bin(S, ref, q_ready, q_bin)
                    except StopIteration: pass
                    S.state = orig
            ok = bool(d.xpos[b][2] > 0.08); res[w].append(ok)
            print(json.dumps({"ep": ep, "item": n, "who": w, "lifted": ok, "closest_cm": round(closest[0] * 100, 1), "jaw_end": round(float(d.qpos[gl]), 4),
                              "grasp_z_end_cm": round(float(S.grasp_point()[2]) * 100, 1)}), flush=True)
            park_it(n); mujoco.mj_forward(m, d); pick.stow(S, q_ready)
    for w in which:
        r = [x for x in res[w] if x is not None]
        print(f"EVAL {w}: {sum(r)}/{len(r)} lifted ({100 * sum(r) / max(len(r), 1):.0f}%) on seeds {a.seed}+, {len(res[w]) - len(r)} without a start pose", flush=True)
    if srv: srv.stdin.close()
    import os; os._exit(0)
if __name__ == "__main__":
    import os, traceback
    try: main()
    except Exception: traceback.print_exc(); sys.stdout.flush(); os._exit(1)      # never linger holding GPU memory
