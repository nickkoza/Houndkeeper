#!/usr/bin/env python3
"""Tight loop for picking on DimOS's own sim: one toy beside the robot, lie down, run the shared pick routine
(sim/arm/pick.py) through DimOS (arm adapter, arm camera, odometry), save every wrist capture. Viewer on."""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import json, os, sys, time, pickle
import numpy as np
sys.path.insert(0, str(_HK / "sim" / "dimos")); import env as hk_env
ITEM = sys.argv[1] if len(sys.argv) > 1 else "triceratops"; OFF = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
OUT = sys.argv[3] if len(sys.argv) > 3 else "/tmp/pick_bringup"; os.makedirs(OUT, exist_ok=True)
START = [float(v) for v in (sys.argv[4] if len(sys.argv) > 4 else "-1.0,1.0").split(",")]
sys.path.insert(0, str(_HK / "sim" / "arm")); import cluster_stops as _cs
OFF_XY = _cs.front_offset(30)                                                 # where a head-on approach leaves an item ([S7o]) (from the reach map)
hk_env.apply(json.dumps([[ITEM, ITEM, [START[0] + float(OFF_XY[0]) + 0.035, START[1] + float(OFF_XY[1]), 0.1]]]))   # +3.5 cm: lying settles the base back                   # beside the robot's left front, in lying reach
os.environ["DIMOS_SIM_TRUTH_FILE"] = f"{OUT}/truth.json"
sys.path.insert(0, str(_HK / "sim" / "arm")); sys.path.insert(0, str(_HK / "src"))
from dimos.core.global_config import GlobalConfig
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.hardware.manipulators.sim.adapter import ShmMujocoAdapter
from houndkeeper.dimos_robot import DimosRobot
import pick
conn = MujocoConnection(GlobalConfig(simulation="mujoco", robot_model="unitree_go2", mujoco_start_pos=f"{START[0]}, {START[1]}")); conn.start(); t0 = time.time()
say = lambda *a: print(f"[{time.time() - t0:6.1f}s]", *a, flush=True)
def odom():
    o = conn.get_odom_message()
    return None if o is None else (np.array([o.position.x, o.position.y, o.position.z]), np.array([o.orientation.w, o.orientation.x, o.orientation.y, o.orientation.z]))
last = {}
def img():
    f = conn.get_arm_video_frame()
    if f is not None: last["f"] = f
    return last.get("f")
time.sleep(5); conn.liedown(); time.sleep(5)
arm = ShmMujocoAdapter(dof=7, address=hk_env.ARM_KEY); arm.connect(); arm.write_enable(True)
import threading, math
_state = {"s": "start"}
def _yawlog():
    lastp = None
    while True:
        o = odom()
        if o is not None:
            p, q = o; w, x, y, z = q; yw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
            print(f"   yaw {yw:6.1f} xy {np.round(p[:2], 3)} during {_state['s']}", flush=True)
        time.sleep(1.0)
S = DimosRobot(arm, img, odom, say=lambda *a: (_state.__setitem__("s", " ".join(map(str, a))), say(*a))); caps = []
threading.Thread(target=_yawlog, daemon=True).start()
cap0 = S.capture; S.capture = lambda: (caps.append(cap0()), caps[-1])[1]
q_ready, q_bin = S.ready_and_bin(); S.waypoints = [q_ready]; S.arm_to_safe(q_ready, 2.0)
truth = json.load(open(f"{OUT}/truth.json"))["bodies"][ITEM]; say("item truth", np.round(truth, 3))
rng = np.random.default_rng(0); guess = np.array(truth[:2]) + rng.normal(0, OFF / 1.4, 2); say("flag", np.round(guess, 3))
ref, notes = pick.look_closer(S, guess); say("look", notes, None if ref is None else (np.round(ref["xy"], 3), ref.get("label"), round(ref["grip_width_cm"], 1)))
if ref is not None:                                                          # how far look-closer put the item from where it is
    t_now = np.array(json.load(open(f"{OUT}/truth.json"))["bodies"][ITEM][:2]); say("look-closer error (cm):", round(float(np.linalg.norm(ref["xy"] - t_now)) * 100, 1))
if ref is not None and os.environ.get("HK_PICK_POLICY"):                    # learned pick from the handover pose, scripted carry
    client = pick.PolicyClient(os.environ["HK_PICK_POLICY"]); say("policy", client.info)
    q = pick.handover_pose(S, ref["xy"]); S.arm_to(q, 1.5); S.step(40)
    jaw = pick.policy_pick(S, client, ee=False); say("policy pick done, jaw", round(jaw, 4), "item z", json.load(open(f"{OUT}/truth.json"))["bodies"][ITEM][2])
    say("carry:", pick.carry_to_bin(S, q_ready, q_bin) or "binned (robot's judgement)"); client.close()
elif ref is not None:
    trace = []                                                              # the body's pose when the grasp is planned and run
    def _pose(tag):
        S.sync(); trace.append((tag, np.round(S.d.qpos[0:3], 4).tolist(), round(float(np.degrees(2 * np.arctan2(S.d.qpos[6], S.d.qpos[3]))), 2)))
    _ikl, _at = S.ik_lean, S.arm_to
    S.ik_lean = lambda *a, **k: (_pose("plan"), _ikl(*a, **k))[1]
    S.arm_to = lambda q, secs, grip=None: (_at(q, secs, grip), _pose("moved"))[0]
    fail, held = pick.grasp_to_bin(S, ref, q_ready, q_bin); S.ik_lean, S.arm_to = _ikl, _at
    for t in trace[:14]: say("  body", *t)
    say("pick:", fail or "binned (robot's judgement)", "held", held, "fingertip error at grip (cm)", getattr(S, "track_err_cm", None), "cmd vs measured joints", np.round(np.array(S._cmd[:6]) - np.array(S._read()[:6]), 3))
pick.stow(S, q_ready); time.sleep(3)
say("truth now", json.load(open(f"{OUT}/truth.json"))["bodies"][ITEM])
pickle.dump([{k: v for k, v in c.items()} for c in caps], open(f"{OUT}/caps.pkl", "wb"))
conn.stop(); os._exit(0)
