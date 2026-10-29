"""The robot as the pick routine (sim/arm/pick.py) sees it, backed by DimOS: arm motions go through DimOS's
manipulator adapter, wrist images come from the arm camera stream, the body pose from odometry. A kinematic copy of the
robot (the same Maker Arm definition the standalone sims use) does IK, look poses, collision checks and the wrist
camera's pose and self-mask, exactly as the real robot would from its URDF and joint angles."""
import os
import sys
import threading
from pathlib import Path

import numpy as np

from dimos.utils import clock   # the robot's clock: sim time in the sim ([3Pv])

ARM_SIM = Path(__file__).resolve().parents[2] / "sim" / "arm"
sys.path.insert(0, str(ARM_SIM))


class DimosRobot:
    """Standalone Sim for the kinematics; arm_to / grip_to / step / capture act on DimOS."""

    RATE_HZ = 50.0

    def __init__(self, adapter, get_image, get_odom, say=print, posture="lie"):
        import legs
        import vslice as vs

        self.K = vs.Sim("sock", np.array([30.0, 30.0]), with_obj=False, watch=False)   # kinematics only; never stepped
        self.m, self.d = self.K.m, self.K.d
        self.adapter = adapter; self.get_image = get_image; self.get_odom = get_odom; self.say = say
        self.legs = {"lie": legs.LIE, "crouch": legs.CROUCH, "stand": legs.STAND}[posture]
        self.waypoints = []; self.max_tilt = 0.0; self.speed = 1.0; self.rec = None
        self.source = "sim-dimos" if os.environ.get("DIMOS_SIM_MODEL_HOOK") else "real"   # where recordings are filed
        lim = self.adapter.get_limits(); self._g_lo, self._g_hi = lim.position_lower[-1], lim.position_upper[-1]
        self._cmd = np.array(self._read(), float)                                   # 6 arm joints + gripper (MJCF joint value)
        self._lock = threading.Lock()
        self.sync()

    # --- DimOS's gripper coordinate runs opposite to the MJCF joint; the pick routine works in joint values ---
    def _read(self):
        js = list(self.adapter.read_joint_positions()); js[6] = self._g_lo + self._g_hi - js[6]; return js

    def _write(self, cmd):
        c = list(cmd); c[6] = self._g_lo + self._g_hi - c[6]; self.adapter.write_joint_positions(c)

    # --- keep the kinematic copy matched to DimOS ---
    def sync(self):
        import mujoco
        odom = self.get_odom()
        if odom is not None: self._last_odom = odom                            # sources may return None between messages
        if getattr(self, "_last_odom", None) is None: raise RuntimeError("no odometry yet")
        p, q = self._last_odom                                                  # position, quaternion (w, x, y, z)
        self.d.qpos[0:3] = p; self.d.qpos[3:7] = q
        self.d.qpos[7:19] = self.legs
        js = np.array(self._read(), float)
        self.d.qpos[self.K.aq] = js[:6]
        gl = self.m.jnt_qposadr[self.m.joint("arm_gripper_left_joint").id]; gr = self.m.jnt_qposadr[self.m.joint("arm_gripper_right_joint").id]
        self.d.qpos[gl] = js[6]; self.d.qpos[gr] = js[6]
        self.d.ctrl[12:18] = self._cmd[:6]; self.d.ctrl[18] = self._cmd[6]
        mujoco.mj_forward(self.m, self.d)
        R = self.d.xmat[self.K.tr].reshape(3, 3); self.max_tilt = max(self.max_tilt, float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))))

    # --- what pick.py uses ---
    def look_down(self, xy, out, h_cam=0.32): self.sync(); return self.K.look_down(xy, out, h_cam=h_cam)
    def ik(self, pos, jaw_dir=None):
        self.sync(); q, e = self.K.ik(pos, jaw_dir)
        if e > 0.01: self.say(f"ik error {e * 100:.1f} cm for {np.round(pos, 3)} (base {np.round(self.d.qpos[:3], 3)}, yaw quat {np.round(self.d.qpos[3:7], 3)})")
        return q, e
    def base_xy(self): self.sync(); return self.d.qpos[0:2].copy()
    def ik_lean(self, pos, jaw_dir, max_lean_deg=30.0): self.sync(); self.K.d.ctrl[12:18] = self._cmd[:6]; return self.K.ik_lean(pos, jaw_dir, max_lean_deg)
    def ik_pose(self, pos, R_l7, q_seed): self.sync(); return self.K.ik_pose(pos, R_l7, q_seed)
    def gripper_frame(self, q): self.sync(); return self.K.gripper_frame(q)
    def _hits(self, q): self.sync(); return self.K._hits(q)
    def floor_hit(self, q): self.sync(); return self.K.floor_hit(q)
    def finger_low(self, q): self.sync(); return self.K.finger_low(q)
    def joint_efforts(self): return np.array(self.adapter.read_joint_efforts()[:6], float)   # motor torques (Nm)
    SETTLE_SAMPLES, AVERAGE_SAMPLES, STILL_RAD_S = 3, 12, 0.01

    def _samples(self, n, timeout, still=None):
        """the next n joint states the arm publishes (each new one: the arm's own clock, [Feg], not the wall's), as
        (velocities, efforts); with still, n in a row whose joints all move slower than it"""
        out, last, t0 = [], None, clock.now()
        while len(out) < n and clock.now() - t0 < timeout:
            e = np.array(self.adapter.read_joint_efforts()[:6], float); key = e.tobytes()
            if key != last:
                last = key; v = np.array(self.adapter.read_joint_velocities()[:6], float)
                if still is not None and np.max(np.abs(v)) >= still: out = []
                else: out.append((v, e))
            clock.sleep(0.005)
        return out

    def settled_efforts(self, timeout=8.0):
        """the joint torques once the arm has stopped moving, averaged over AVERAGE_SAMPLES of the arm's own readings:
        the arm publishes one instant per sim frame (35 ms), and the printed frame's flex rings at ~10 Hz ([nAH] z7P), so
        a few readings timed by the wall clock (the sim at 0.28x) caught it mid-swing, and a pony weighed 2.6 kg (28 Sep);
        still earlier, read while catching up with its target, a 0.2 kg shoe weighed 3.5 kg (26 Sep)"""
        self._samples(self.SETTLE_SAMPLES, timeout, still=self.STILL_RAD_S)
        got = self._samples(self.AVERAGE_SAMPLES, timeout)
        return np.mean([e for _, e in got], axis=0) if got else self.joint_efforts()

    def held_mass(self, dtau, q): self.sync(); return self.K.held_mass(dtau, q)

    def set_payload(self, kg):
        """the held item's mass (kg) on the arm's gravity feed-forward ([cqG], [qaS]): in DimOS's sim, through the
        per-launch payload file the sim process reads each frame (sim_hook.viewer); on the real robot, the adapter's own"""
        import go1_arm
        path = os.environ.get(go1_arm.PAYLOAD_ENV)
        if path: go1_arm.write_payload(kg, path)
        if hasattr(getattr(self, "adapter", None), "set_payload"): self.adapter.set_payload(max(0.0, float(kg)))   # the guard's payload too ([qpO])
    def bin_clear_z(self, hang): self.sync(); return self.K.bin_clear_z(hang)
    def over_bin(self, hang): self.sync(); return self.K.over_bin(hang)
    def state(self, name): self.say(f"state: {name}")
    def gripper(self):                         # half the gap between the gripping faces: the jaw reading less any pad
        import jaws; return float(self._read()[6]) - jaws.pad_m()
    def grasp_point(self): self.sync(); return self.d.xpos[self.m.body("arm_grasp_center").id].copy()
    # --- what pick.policy_pick needs ---
    def small_image(self):
        """the newest arm-camera frame as the policy saw it in training: centre-cropped to 4:3 (DimOS renders 16:9 at
        the same vertical field of view) and resized to 128x96"""
        import cv2
        img = self.get_image(); H, W = img.shape[:2]; w4 = int(H * 4 / 3); x0 = (W - w4) // 2
        return cv2.resize(np.ascontiguousarray(img[:, x0:x0 + w4]), (128, 96), interpolation=cv2.INTER_AREA)
    def joint_state(self): return np.array(self._read(), float)
    def kin_model(self): self.sync(); return self.K
    def target(self): return self._cmd.copy()
    def command(self, q6, jaw):
        c = self._cmd.copy(); c[:6] = q6; c[6] = jaw; self._write(c); self._cmd = c; clock.sleep(0.1)   # kept once sent ([WkC])
        if getattr(self, "rec", None) is not None: self.rec(self)

    def _guard(self): return getattr(self.adapter, "guard", None)   # arm_guard.GuardedArm's, where the adapter has one

    def refuses(self, q):
        """why the arm guard would refuse pose q now ([G48], [FWp], [qpO]), or None"""
        g = self._guard(); return None if g is None else g.refuses(q)

    def _profile(self, q0, q, secs, grip=None):
        """the commands of a smooth move from q0 to q at the arm's speed limit (the guard's, where it is lower)"""
        q1 = q0.copy(); q1[:6] = q
        if grip is not None: q1[6] = grip
        g = self._guard(); gv = None if g is None else g.vmax()      # none known: the guard refuses the first step
        v = self.K.ARM_VMAX if gv is None else min(self.K.ARM_VMAX, gv)
        secs = max(secs, float(np.abs(q1[:6] - q0[:6]).max()) / v * 1.6)
        n = max(1, int(secs * self.RATE_HZ))
        return [(1 - a) * q0 + a * q1 for a in (0.5 - 0.5 * np.cos(np.pi * k / n) for k in range(1, n + 1))]

    def arm_to(self, q, secs, grip=None):
        """smooth move of the arm (and optionally the gripper) to q, streamed to DimOS's adapter; a move the guard would
        refuse anywhere along it is refused before it starts"""
        cmds = self._profile(self._cmd.copy(), q, secs, grip)
        if (g := self._guard()) is not None: g.check_path(cmds)
        for k, c in enumerate(cmds, 1):                # the command kept only once the guard let it through: one refused
            self._write(c); self._cmd = c; clock.sleep(1.0 / self.RATE_HZ)   # partway holds the arm at the last sent ([WkC])
            if k % 5 == 0 and getattr(self, "rec", None) is not None: self.rec(self)   # training data at 10 Hz ([95i])
        self.sync()

    def grip_to(self, g, secs=0.8): self.arm_to(self._cmd[:6].copy(), secs, grip=g)

    def arm_to_safe(self, q, secs, grip=None):
        self.sync(); self.K.waypoints = self.waypoints
        path = self.K.plan(self._cmd[:6].copy(), q)
        if (g := self._guard()) is not None:                  # the whole path judged before the first segment moves
            q0 = self._cmd.copy()
            for w in path: cmds = self._profile(q0, w, secs / len(path) * 1.2, grip); g.check_path(cmds); q0 = cmds[-1]
        for w in path: self.arm_to(w, secs / len(path) * 1.2, grip)

    def step(self, n=1): clock.sleep(n * 0.005); self.sync()

    SETTLE_FRAMES, SETTLE_SHARE, SETTLE_MAX_S = 3, 0.03, 8.0

    def _settled_image(self):
        """the newest wrist frame once the camera's auto-exposure has settled: SETTLE_FRAMES new frames in a row whose
        brightness changed by under SETTLE_SHARE. Swung from the stow (looking at the dark under the body) to the floor,
        the camera's first frames are washed out and grainy while its exposure comes down (27 Sep: most standing looks);
        a real camera does the same, so the robot waits, as a person lets their eyes adjust"""
        img = prev = None; stable = 0; t0 = clock.now()
        while clock.now() - t0 < self.SETTLE_MAX_S:
            im = self.get_image()
            if im is None: clock.sleep(0.05); continue
            if prev is not None and np.array_equal(im[::16, ::16], prev[::16, ::16]): clock.sleep(0.03); continue   # no new frame yet
            if prev is not None:
                m0, m1 = float(prev[::8, ::8].mean()), float(im[::8, ::8].mean())
                stable = stable + 1 if abs(m1 - m0) <= self.SETTLE_SHARE * max(m0, 1.0) else 0
            prev = img = im
            if stable >= self.SETTLE_FRAMES: break
        return img

    def capture(self):
        """the newest wrist frame taken after the arm settled, with the camera pose and robot self-mask from kinematics"""
        import mujoco
        import wrist_refine as wr
        clock.sleep(0.3); self.sync()
        img = self._settled_image()
        H, W = img.shape[:2]
        key = ("dimos", H, W)
        if wr._SEG.get("dkey") != key:
            g_ = self.m.vis.global_; g_.offwidth = max(int(g_.offwidth), W); g_.offheight = max(int(g_.offheight), H)   # a frame
            wr._SEG["dseg"] = mujoco.Renderer(self.m, H, W); wr._SEG["dseg"].enable_segmentation_rendering(); wr._SEG["dkey"] = key   # at 1280 x 720 ([7cU])
            root = self.m.body_rootid[self.m.body("trunk").id]
            wr._SEG["drobot"] = np.array([i for i in range(self.m.ngeom) if self.m.body_rootid[self.m.geom_bodyid[i]] == root])
        wr._SEG["dseg"].update_scene(self.d, "arm_wrist"); seg = wr._SEG["dseg"].render()[..., 0]
        cid = self.m.camera("arm_wrist").id
        return {"img": img, "robot": np.isin(seg, wr._SEG["drobot"]), "c": self.d.cam_xpos[cid].copy(),
                "R": self.d.cam_xmat[cid].reshape(3, 3).copy(), "fovy": float(self.m.cam_fovy[cid])}

    # --- stop poses (computed on the kinematic copy, lying) ---
    def ready_and_bin(self):
        self.sync()
        mount = self.d.xpos[self.m.body("arm_base_link").id].copy()
        q_ready = self.K.ready_pose()
        return q_ready, self.K.bin_pose()
