#!/usr/bin/env python3
"""Tight loop for the arm in DimOS's own sim ([RJi]): start DimOS's MuJoCo connection with the Maker Arm hook, then
lie down, drive the arm through DimOS's sim-arm adapter, read the arm camera, stand up and walk. Viewer on (DimOS's)."""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import os, sys, time
import numpy as np
sys.path.insert(0, str(_HK / "src")); sys.path.insert(0, str(_HK / "sim" / "dimos"))
import env as hk_env
hk_env.apply()
from dimos.core.global_config import GlobalConfig
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.hardware.manipulators.sim.adapter import ShmMujocoAdapter
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3

cfg = GlobalConfig(simulation="mujoco", robot_model="unitree_go2", mujoco_start_pos="0.0, 0.0")
conn = MujocoConnection(cfg); conn.start(); t0 = time.time()
def z():
    o = conn.get_odom_message(); return None if o is None else round(float(o.position.z), 3)
def say(*a): print(f"[{time.time() - t0:6.1f}s]", *a, flush=True)
time.sleep(4); say("started; body z", z())
conn.liedown(); time.sleep(4); say("liedown -> body z", z())
arm = ShmMujocoAdapter(dof=7, address=hk_env.ARM_KEY); say("arm connect", arm.connect(), "dof", arm.get_dof())
q0 = arm.read_joint_positions(); say("arm joints", np.round(q0, 2))
arm.write_enable(True); target = list(q0[:7]); target[1] += 0.4; target[2] -= 0.4; target[6] = 0.03
say("command accepted", arm.write_joint_positions(target)); time.sleep(3); say("after move", np.round(arm.read_joint_positions(), 2), "target", np.round(target, 2))
f = conn.get_arm_video_frame(); time.sleep(0.5); f = conn.get_arm_video_frame() if f is None else f
say("arm camera frame", None if f is None else f.shape)
if f is not None:
    from PIL import Image; Image.fromarray(f).save(os.environ.get("HK_OUT", "/tmp/arm_cam.png")); say("saved arm camera frame")
arm.write_joint_positions(list(q0[:7])); time.sleep(3)
conn.standup(); time.sleep(5); say("standup -> body z", z())
conn.move(Twist(linear=Vector3(0.4, 0, 0), angular=Vector3(0, 0, 0)), duration=3.0); time.sleep(4)
o = conn.get_odom_message(); say("walked to", None if o is None else (round(o.position.x, 2), round(o.position.y, 2), round(o.position.z, 2)))
arm.disconnect(); conn.stop(); say("done")
os._exit(0)
