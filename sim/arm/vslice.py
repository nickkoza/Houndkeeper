#!/usr/bin/env python3
"""Vertical slice in sim: walk to a parking spot beside a flagged object, lie down, look closer with the wrist camera,
grasp straight down, drop it in the tray on the robot's back, stow, stand up, walk off. Records frames for video.
Scripted (no learning). Sim Go1 body, not a Go2 Air. Usage: vslice.py [--obj toy|sock] [--video out.mp4]"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import argparse, os, sys, json
from pathlib import Path
import numpy as np, mujoco
def _cross(a, b): return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])   # np.cross is slow on 3-vectors
sys.path.insert(0, str(_HK / "sim" / "arm")); import go1_arm as ga, payload_test as pt, legs, jaws as jw
sys.path.insert(0, str(_HK / "src" / "houndkeeper"))
HERE = Path(__file__).resolve().parent
OBJS = {"toy": {"name": "toy", "half": [0.03, 0.03, 0.03], "mass": 0.04, "rgba": [0.9, 0.2, 0.2, 1]},
        "sock": {"name": "sock", "half": [0.09, 0.035, 0.008], "mass": 0.04, "rgba": [0.25, 0.35, 0.75, 1]}}
PARK_REL = np.array([0.10, 0.34])          # object beside the robot, clear of the front leg when lying (still inside the grasp band, FINDINGS)

PATCH_AHEAD = 0.3                # the wrist-resolution floor patch is centred this far ahead of the body (items sit ~0.47 m ahead)


class Sim:
    def __init__(self, obj, obj_xy, video=None, floor_tex=None, with_obj=True, start=(0.0, 0.0, 0.0), watch=False, extra=(), hud=None, mount=None, local_floor=False, lib=()):
        o = dict(OBJS[obj]); o["pos"] = [obj_xy[0], obj_xy[1], o["half"][2] + 0.001]
        more = []
        for k_, (kind_, xy_) in enumerate(extra):
            e_ = dict(OBJS[kind_]); e_["name"] = f"{kind_}_{k_ + 2}"; e_["pos"] = [xy_[0], xy_[1], e_["half"][2] + 0.001]; more.append(e_)
        self.m = ga.build(True, pt.STOW, pick=True, objects=([o] + more) if with_obj else [], floor_tex=floor_tex, mount=mount, local_floor=local_floor, lib=lib); self.m.opt.timestep = ga.CTRL_DT / ga.SUBSTEPS
        self.m.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC      # held items crept out of the jaws with pyramidal friction (grasp spec [Udt])
        ga.near_clip(self.m)                                   # the wrist camera sees what's just in front of it
        self.d = mujoco.MjData(self.m)
        if not with_obj: obj = "trunk"
        self.obj = obj; self.ob = self.m.body(obj).id; self.tr = self.m.body("trunk").id; self.gc = self.m.body("arm_grasp_center").id
        self.l7 = self.m.body("arm_link_007").id
        self.aq = [self.m.jnt_qposadr[self.m.joint("arm_" + j).id] for j in ga.ARM_JOINTS[:6]]
        self.ad = [self.m.jnt_dofadr[self.m.joint("arm_" + j).id] for j in ga.ARM_JOINTS[:6]]
        self.rng = np.array([self.m.jnt_range[self.m.joint("arm_" + j).id] for j in ga.ARM_JOINTS[:6]])
        self.kin = mujoco.MjData(self.m); self.frames = []; self.video = video; self.log = []; self.t = 0.0
        self.pol = None; self.max_tilt = 0.0
        if video:
            self.ren = mujoco.Renderer(self.m, 480, 640); self.wren = mujoco.Renderer(self.m, 240, 320)
            self.cam = mujoco.MjvCamera(); self.cam.type = mujoco.mjtCamera.mjCAMERA_FREE; self.cam.distance = 1.6; self.cam.elevation = -28; self.cam.azimuth = 135
        self.viewer = None; self.speed = 1.0; self._w0 = None; self.hud = hud; self.source = "sim-standalone"   # where pick recordings are filed
        self.obs = [self.m.body(n).id for n in ([o["name"]] + [e_["name"] for e_ in more])] if with_obj else []
        self.snap_every = 7; self.inset = None; self.on_step = None; legs.start_standing(self.m, self.d, 0.30, at=start); self.caption = ""
        if start != (0.0, 0.0, 0.0):
            self.d.qpos[0:2] = start[:2]; self.d.qpos[3:7] = [np.cos(start[2] / 2), 0, 0, np.sin(start[2] / 2)]; mujoco.mj_forward(self.m, self.d)
            ga.step(self.m, self.d, 200)
        if watch:                                                     # live window, paced to real time (speed x)
            import mujoco.viewer as _mv, time as _t; self._time = _t
            self.viewer = _mv.launch_passive(self.m, self.d, show_left_ui=False, show_right_ui=False)
            self.viewer.cam.distance = 2.2; self.viewer.cam.elevation = -30; self.viewer.cam.azimuth = 135
    def gains(self, kp, kd):
        for a in range(12): self.m.actuator_gainprm[a, 0] = kp; self.m.actuator_biasprm[a, 1] = -kp; self.m.actuator_biasprm[a, 2] = -kd
    def step(self, n=1):
        for _ in range(n):
            if self.pol is not None and int(round(self.t / 0.005)) % 4 == 0: self.pol.act(self.m, self.d)
            ga.step(self.m, self.d); self.t += 0.005
            if getattr(self, "rec", None) is not None and int(round(self.t / 0.005)) % 20 == 0: self.rec(self)   # demonstration recording, 10 Hz
            R = self.d.xmat[self.tr].reshape(3, 3); tl_ = np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))); self.max_tilt = max(self.max_tilt, tl_)
            if tl_ > 25 and not getattr(self, "_tip_logged", False): self._tip_logged = True; self._tip_report()
            if self.video and int(round(self.t / 0.005)) % self.snap_every == 0: self.snap()        # ~28 fps (3x during sweeps)
            if self.on_step: self.on_step(self.m, self.d)
            if self.hud: self.hud.tick(self, 0.005)
            self._watch()
    def _tip_report(self):
        m, d = self.m, self.d; armb = {b for b in range(m.nbody) if (mujoco.mj_id2name(m, 1, b) or "").startswith("arm_")}
        cont = sorted({(mujoco.mj_id2name(m, 1, m.geom_bodyid[c.geom1]), mujoco.mj_id2name(m, 1, m.geom_bodyid[c.geom2])) for c in d.contact[:d.ncon]
                       if (m.geom_bodyid[c.geom1] in armb) or (m.geom_bodyid[c.geom2] in armb)})
        gc = d.xpos[m.body("arm_grasp_center").id]; rel = d.xmat[self.tr].reshape(3, 3).T @ (gc - d.xpos[self.tr])
        print(f"   TIP ALARM t={self.t:.1f}s state={getattr(self.hud, 'state', '-')}: trunk tilted >25 deg; grasp point rel trunk {np.round(rel, 2)}; "
              f"arm torques {np.round(d.actuator_force[12:18], 1)}; arm contacts {cont}", flush=True)
    def _watch(self):
        if int(round(self.t / 0.005)) % 2000 == 0: print(f"   heartbeat sim t={self.t:.0f}s state={getattr(self.hud, 'state', '-')}", flush=True)
        if self.viewer is None or int(round(self.t / 0.005)) % 7: return
        if self._w0 is None: self._w0 = (self._time.time(), self.t)
        now = self._time.time()                                   # draw at most 20 times a wall-clock second: the viewer stays on
        lag = (self.t - self._w0[1]) / self.speed - (now - self._w0[0])   # without slowing the physics down to its frame rate
        if lag > 0: self._time.sleep(lag)
        if now - getattr(self, "_last_draw", 0.0) < 0.05: return
        self._last_draw = now
        self.viewer.cam.lookat[:] = self.d.xpos[self.tr]; self.viewer.cam.lookat[2] = 0.2
        if self.hud is not None and now - getattr(self, "_last_hud", 0.0) > 0.25:
            self._last_hud = now
            vp = self.viewer.viewport; W_, H_ = (vp.width, vp.height) if vp is not None else (1280, 720)
            self.viewer.set_figures(self.hud.figures(W_, H_)); self.viewer.set_texts(self.hud.texts())
            if getattr(self, "wren", None) is None: self.wren = mujoco.Renderer(self.m, 240, 320)
            self.wren.update_scene(self.d, "arm_wrist"); imgs = [(mujoco.MjrRect(W_ - 330, H_ - 240 - 250, 320, 240), self.wren.render().copy())]   # under the graph (no flip: camera is right side up now)
            if getattr(self, "depth_inset", None) is not None:   # the stereo pair's last height map, beside it ([A8K])
                imgs.append((mujoco.MjrRect(10, 10, 320, 180), self.depth_inset))   # lower left: off the robot and the state machine
            try: self.viewer.set_images(imgs)
            except Exception: self.viewer.set_images(imgs[0])
        with self.viewer.lock(): self.viewer.sync()
    def say(self, text):
        self.caption = text; self.log.append((round(self.t, 2), text)); print(f"[{self.t:6.2f}s] {text}", flush=True)
        if self.hud is not None: self.hud.note = text
        elif self.viewer is not None:
            try: self.viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, text, ""))
            except Exception: pass
    def state(self, name):
        if self.hud is not None: self.hud.state = name
    def snap(self):
        self.cam.lookat[:] = self.d.xpos[self.tr] * 0.6 + self.d.xpos[self.ob] * 0.4; self.cam.lookat[2] = 0.15
        self.ren.update_scene(self.d, self.cam); img = self.ren.render().copy()
        self.wren.update_scene(self.d, "arm_wrist"); w = self.wren.render()
        img[8:8 + 240 // 2, -8 - 320 // 2:-8] = w[::2, ::2]
        if self.inset is not None:                                   # tonight's floor picture with the flag, bottom-left
            fl, cand, tgt = self.inset; th = np.asarray(fl)[::5, ::5].copy(); cm = cand[::5, ::5]; th[cm] = [255, 0, 255]
            img[450 - th.shape[0] - 6:450 - 6, 8:8 + th.shape[1]] = th
        self.frames.append((img, self.caption, self.snap_every > 7))
    # --- base ---
    def walk_to(self, goal_xy, goal_yaw, tol=0.05, max_s=30, settle=True):      # 5 cm: finer only inches around (look-closer absorbs ~10 cm)
        self.gains(35, 0)
        if self.pol is None: self.pol = ga.LegPolicy(self.m)
        t0 = self.t
        while self.t - t0 < max_s:
            x, y = self.d.qpos[0:2]; q = self.d.qpos[3:7]; yaw = np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
            e = goal_xy - np.array([x, y]); c, s = np.cos(yaw), np.sin(yaw); eb = np.array([c * e[0] + s * e[1], -s * e[0] + c * e[1]])
            ey = (goal_yaw - yaw + np.pi) % (2 * np.pi) - np.pi
            if np.linalg.norm(e) < tol and (abs(ey) < 0.10 or not settle): break
            if not settle:                                           # sweeping: steer toward the goal while still walking (U-turn arcs, no spin in place)
                ey = (np.arctan2(e[1], e[0]) - yaw + np.pi) % (2 * np.pi) - np.pi
                dist = np.linalg.norm(e); fwd = 0.45 * max(np.cos(ey), 0.25 if dist > 0.5 else 0.0)   # near the waypoint: turn on the spot, don't orbit it
                self.pol.cmd = np.array([np.clip(fwd, 0, 0.45), 0.0, np.clip(2.0 * ey, -1.0, 1.0)], np.float32)
            else:
                self.pol.cmd = np.array([np.clip(1.2 * eb[0], -0.4, 0.5), np.clip(1.2 * eb[1], -0.3, 0.3), np.clip(1.5 * ey, -0.6, 0.6)], np.float32)
            self.step(4)
        if not settle: return np.linalg.norm(goal_xy - self.d.qpos[0:2])
        self.pol.cmd[:] = 0; self.step(80)                           # stop and settle 0.4 s
        # hand the legs from the walking policy to posture control: hold the current leg angles, then stiffen
        self.pol = None; self.d.ctrl[:12] = self.d.qpos[7:19]; self.gains(120, 3)
        return np.linalg.norm(goal_xy - self.d.qpos[0:2])
    def legs_to(self, target, secs): legs.move_legs(self.m, self.d, target, secs, lambda m, d: self._tick())
    def _tick(self):
        self.t += 0.005; R = self.d.xmat[self.tr].reshape(3, 3); self.max_tilt = max(self.max_tilt, np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))))
        if self.video and int(round(self.t / 0.005)) % 7 == 0: self.snap()
        if self.on_step: self.on_step(self.m, self.d)
        if self.hud: self.hud.tick(self, 0.005)
        self._watch()
    # --- arm ---
    def ik(self, pos, jaw_dir=None, seeds=10, signed=False):
        """grasp point at pos, gripper pointing straight down, jaws closing along jaw_dir (world xy). Kinematics only."""
        best = (9, None); R_ = np.random.default_rng(len(self.log)); good = []
        for s in range(seeds):
            self.kin.qpos[:] = self.d.qpos; q = self.d.ctrl[12:18].copy() if s == 0 else R_.uniform(self.rng[:, 0], self.rng[:, 1])
            for it in range(200):
                self.kin.qpos[self.aq] = q; mujoco.mj_kinematics(self.m, self.kin); mujoco.mj_comPos(self.m, self.kin)
                Rl = self.kin.xmat[self.l7].reshape(3, 3)
                approach = self.kin.xpos[self.gc] - self.kin.xpos[self.l7]; approach /= np.linalg.norm(approach)
                w = _cross(approach, [0, 0, -1.0])
                if jaw_dir is not None:
                    jaw = Rl[:, 1]; jd = np.array([jaw_dir[0], jaw_dir[1], 0.0]); jd /= np.linalg.norm(jd)
                    if jaw @ jd < 0 and not signed: jd = -jd          # signed: the wrist camera side matters (it sits on the -jaw side)
                    w = w + 0.5 * _cross(jaw, jd)
                e = np.hstack([pos - self.kin.xpos[self.gc], 0.3 * w])
                if np.linalg.norm(e[:3]) < 0.004 and np.linalg.norm(w) < 0.03: break
                Jp = np.zeros((3, self.m.nv)); Jr = np.zeros((3, self.m.nv))
                mujoco.mj_jacBody(self.m, self.kin, Jp, None, self.gc); mujoco.mj_jacBody(self.m, self.kin, None, Jr, self.l7)
                J = np.vstack([Jp[:, self.ad], 0.3 * Jr[:, self.ad]])
                q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 2e-3 * np.eye(6), e), self.rng[:, 0], self.rng[:, 1])
            err = np.linalg.norm(e[:3]) + 0.05 * np.linalg.norm(w)
            if err < best[0]: best = (err, q.copy())
            if err < 0.01: good.append(q.copy())
            if err < 0.01 and (s == 0 or len(good) >= 2): break   # converged from where the arm is, or enough options to choose from
        if good: return self._nearest(good), best[0]
        return best[1], best[0]
    MOVE_W = np.array([3.0, 2.0, 2.0, 1.0, 1.0, 1.0])      # joint-motion cost: swinging the base or shoulder moves the whole arm
    def _nearest(self, qs):
        """of several solutions, the one needing the least (weighted) motion from the arm's current target: no swinging
        the arm half way round the robot (in a house, into furniture) when a nearby solution exists"""
        cur = self.d.ctrl[12:18]; return min(qs, key=lambda q: float(np.sum(self.MOVE_W * np.abs(q - cur))))
    def new_floor(self, seed, kind=None, focus_xy=None):
        """a fresh wrist-resolution floor under the robot (grasp spec [N4z]); uploaded to the viewer and every camera"""
        import floors, wrist_refine as wr_
        if getattr(self, "_small", None) is None and not getattr(self, "_small_frozen", False): self._small = mujoco.Renderer(self.m, 96, 128)
        rs = [getattr(self, "ren", None), getattr(self, "wren", None), wr_._SEG.get("rgb"),        # every camera must get the new floor:
              None if getattr(self, "_small_frozen", False) else getattr(self, "_small", None)]  # a renderer keeps the textures it was made with
        wr_._SEG["viewer"] = self.viewer
        R = self.d.xmat[self.tr].reshape(3, 3); centre = self.d.xpos[self.tr][:2] + R[:2, 0] * PATCH_AHEAD   # the patch covers where
        self.relight(seed, centre)                                          # every new floor gets the episode's lighting too
        return floors.apply(self.m, seed, centre, renderers=rs, viewer=self.viewer, kind=kind, focus_xy=focus_xy)
    def relight(self, seed, centre):
        """the episode's lighting, drawn from seed as the house nights draw theirs ([QMt]): night (half the episodes) is a
        warm ceiling light over the work area and often a lamp; day is a window's glow from one side and, a third of
        the time, the sun. The scene's own spotlight goes off; the renderers are rebuilt for the new light."""
        m = self.m; r = np.random.default_rng(seed * 7 + 5); c = np.array([centre[0], centre[1]])
        lid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, n)
        m.light_active[:] = 0
        def put(name, pos, d, rgb, k, n):
            for i in range(n):
                j = lid(f"{name}_{i}"); m.light_active[j] = 1; m.light_diffuse[j] = np.asarray(rgb) * k / n
                m.light_pos[j] = pos + np.array([np.cos(2 * np.pi * i / n), np.sin(2 * np.pi * i / n), 0]) * 0.1 * (n > 1)
                m.light_dir[j] = d / np.linalg.norm(d)
        warm, cool = np.array([1.0, 0.78, 0.55]), np.array([0.92, 0.96, 1.0])
        if r.random() < 0.5:                                                 # night
            put("ceiling", np.r_[c + r.uniform(-1.5, 1.5, 2), 2.35], np.array([0, 0, -1.0]), warm, r.uniform(0.5, 0.9) * 2.2, 3)
            if r.random() < 0.6: put("lamp", np.r_[c + r.uniform(-2.5, 2.5, 2), 1.6], np.array([0, 0, -1.0]), warm, 0.9, 2)
            m.vis.headlight.ambient[:] = warm * r.uniform(0.06, 0.16)
        else:                                                                # day
            a = r.uniform(0, 2 * np.pi); p = c + 2.5 * np.array([np.cos(a), np.sin(a)])    # a window's glow from one side
            put("window", np.r_[p, 1.6], np.r_[c - p, -1.2], cool, r.uniform(0.5, 1.0) * 1.4, 2)
            if r.random() < 0.35:
                az, el = r.uniform(0, 2 * np.pi), np.radians(r.uniform(25, 55))
                j = lid("sun_0"); m.light_active[j] = 1; m.light_diffuse[j] = r.uniform(0.7, 1.0) * np.ones(3)
                m.light_dir[j] = [np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), -np.sin(el)]
            m.vis.headlight.ambient[:] = cool * r.uniform(0.25, 0.45)
        import filament_cameras as fc; fc.reset(m)   # items are, not the body
    def look_down(self, xy, out, h_cam=0.32, seeds=10):
        """arm pose that puts the wrist CAMERA h_cam above floor point xy, looking straight down, image-up pointing away
        from the robot (out). Top-down view: true outlines, fingers off to the robot side of the frame."""
        m = self.m; cid = m.camera("arm_wrist").id; tgt = np.array([xy[0], xy[1], h_cam]); yt = np.array([out[0], out[1], 0.0]); zt = np.array([0, 0, 1.0])
        best = (9, None); R_ = np.random.default_rng(5); good = []
        for s_ in range(seeds):
            q = self.d.ctrl[12:18].copy() if s_ == 0 else R_.uniform(self.rng[:, 0], self.rng[:, 1])
            for it in range(250):
                self.kin.qpos[:] = self.d.qpos; self.kin.qpos[self.aq] = q; mujoco.mj_kinematics(m, self.kin); mujoco.mj_comPos(m, self.kin); mujoco.mj_camlight(m, self.kin)
                c = self.kin.cam_xpos[cid]; Rc = self.kin.cam_xmat[cid].reshape(3, 3)
                ep = tgt - c; er = 0.5 * (_cross(Rc[:, 2], zt) + _cross(Rc[:, 1], yt))
                if np.linalg.norm(ep) < 0.004 and np.linalg.norm(er) < 0.02: break
                Jp = np.zeros((3, m.nv)); Jr = np.zeros((3, m.nv)); mujoco.mj_jac(m, self.kin, Jp, Jr, c, self.l7)
                J = np.vstack([Jp[:, self.ad], 0.3 * Jr[:, self.ad]]); e = np.hstack([ep, 0.3 * er])
                q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 2e-3 * np.eye(6), e), self.rng[:, 0], self.rng[:, 1])
            err = np.linalg.norm(ep) + 0.05 * np.linalg.norm(er)
            ok = self._hits(q) == 0
            if err < best[0] and ok: best = (err, q.copy())
            if err < 0.01 and ok: good.append(q.copy())
            if err < 0.01 and ok and (s_ == 0 or len(good) >= 3): break
        self._look_err = best[0]
        return self._nearest(good) if good else best[1]
    def look_at(self, xy, h, out, iters=3):
        """arm pose whose wrist-camera view centre lands on floor point xy, grasp point h above the floor, gripper down,
        camera on the robot's side of the wrist looking outward (out = unit vector from robot to item)."""
        m = self.m; cid = m.camera("arm_wrist").id; jd = np.array([-out[1], out[0]]); tgt = np.array(xy, float).copy(); q = None
        for _ in range(iters):
            q, _ = self.ik(np.array([tgt[0], tgt[1], h]), jd, signed=True)
            self.kin.qpos[:] = self.d.qpos; self.kin.qpos[self.aq] = q; mujoco.mj_kinematics(m, self.kin); mujoco.mj_camlight(m, self.kin)
            c = self.kin.cam_xpos[cid]; fwd = -self.kin.cam_xmat[cid].reshape(3, 3)[:, 2]
            if fwd[2] >= -1e-3: break
            hit = c + fwd * (-c[2] / fwd[2]); tgt = tgt - (hit[:2] - np.asarray(xy)); self._look_miss = float(np.linalg.norm(hit[:2] - np.asarray(xy)))
        return q
    def ik_lean(self, pos, jaw_dir, max_lean_deg=30.0, seeds=16):
        """grasp point at pos with the gripper within max_lean of straight down, jaws level and closing along jaw_dir
        (either sign), no arm contact with the robot or the floor. Returns (q, err, lean_deg); least motion among options."""
        m = self.m; k = self.kin; down = np.array([0, 0, -1.0]); tmax = np.radians(max_lean_deg)
        jd = np.array([jaw_dir[0], jaw_dir[1], 0.0]); jd /= np.linalg.norm(jd); R_ = np.random.default_rng(len(self.log)); good = []; best = (9, None, 0.0)
        for s_ in range(seeds):
            q = self.d.ctrl[12:18].copy() if s_ == 0 else R_.uniform(self.rng[:, 0], self.rng[:, 1])
            for it in range(200):
                k.qpos[:] = self.d.qpos; k.qpos[self.aq] = q; mujoco.mj_kinematics(m, k); mujoco.mj_comPos(m, k)
                Rl = k.xmat[self.l7].reshape(3, 3); a = k.xpos[self.gc] - k.xpos[self.l7]; a /= np.linalg.norm(a)
                ang = np.arccos(np.clip(a @ down, -1, 1)); w = np.zeros(3)
                if ang > tmax:
                    ax = _cross(a, down); n = np.linalg.norm(ax)
                    if n > 1e-9: w = w + ax / n * (ang - tmax)
                jaw = Rl[:, 1]; jt = jd if jaw @ jd >= 0 else -jd; w = w + 0.5 * _cross(jaw, jt)   # level jaws across the narrow side
                ep = pos - k.xpos[self.gc]
                if np.linalg.norm(ep) < 0.004 and np.linalg.norm(w) < 0.03: break
                Jp = np.zeros((3, m.nv)); Jr = np.zeros((3, m.nv)); mujoco.mj_jacBody(m, k, Jp, None, self.gc); mujoco.mj_jacBody(m, k, None, Jr, self.l7)
                J = np.vstack([Jp[:, self.ad], 0.3 * Jr[:, self.ad]]); e = np.hstack([ep, 0.3 * w])
                q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 2e-3 * np.eye(6), e), self.rng[:, 0], self.rng[:, 1])
            err = np.linalg.norm(ep) + 0.05 * np.linalg.norm(w)
            if err < 0.01 and self._hits(q) == 0 and not self.floor_hit(q): good.append((q.copy(), float(np.degrees(ang))))
            elif err < best[0]: best = (err, q.copy(), float(np.degrees(ang)))
            if len(good) >= 2 or (good and s_ == 0): break
        if good:
            q, lean = min(good, key=lambda g: float(np.sum(self.MOVE_W * np.abs(g[0] - self.d.ctrl[12:18])))); return q, 0.0, lean
        return best[1], best[0], best[2]
    def ik_pose(self, pos, R_l7, q_seed, iters=150):
        """grasp point at pos with link 7 held at orientation R_l7 (to move along the gripper's own axis); (q, err)"""
        m = self.m; k = self.kin; q = np.array(q_seed, float)
        for it in range(iters):
            k.qpos[:] = self.d.qpos; k.qpos[self.aq] = q; mujoco.mj_kinematics(m, k); mujoco.mj_comPos(m, k)
            ep = pos - k.xpos[self.gc]; Rc = k.xmat[self.l7].reshape(3, 3)
            er = 0.5 * (_cross(Rc[:, 0], R_l7[:, 0]) + _cross(Rc[:, 1], R_l7[:, 1]) + _cross(Rc[:, 2], R_l7[:, 2]))
            if np.linalg.norm(ep) < 0.002 and np.linalg.norm(er) < 0.01: break
            Jp = np.zeros((3, m.nv)); Jr = np.zeros((3, m.nv)); mujoco.mj_jacBody(m, k, Jp, None, self.gc); mujoco.mj_jacBody(m, k, None, Jr, self.l7)
            J = np.vstack([Jp[:, self.ad], 0.3 * Jr[:, self.ad]])
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), np.hstack([ep, 0.3 * er])), self.rng[:, 0], self.rng[:, 1])
        return q, float(np.linalg.norm(ep))
    def gripper_frame(self, q):
        """(approach direction, link-7 orientation) at arm pose q"""
        k = self.kin; k.qpos[:] = self.d.qpos; k.qpos[self.aq] = q; mujoco.mj_kinematics(self.m, k)
        a = k.xpos[self.gc] - k.xpos[self.l7]; return a / np.linalg.norm(a), k.xmat[self.l7].reshape(3, 3).copy()
    def floor_hit(self, q):
        """any arm part except the jaws touching the floor at pose q (the jaws may touch it at the grasp)"""
        m = self.m; k = self.kin; k.qpos[:] = self.d.qpos; k.qpos[self.aq] = q; mujoco.mj_fwdPosition(m, k)
        if not hasattr(self, "_arm_nojaw"):
            jaws = {m.body("arm_gripper_left").id, m.body("arm_gripper_right").id, self.gc}
            self._arm_nojaw = {i for i in range(m.nbody) if (mujoco.mj_id2name(m, 1, i) or "").startswith("arm_") and i not in jaws}
        for con in k.contact[:k.ncon]:
            b1, b2 = m.geom_bodyid[con.geom1], m.geom_bodyid[con.geom2]
            if (b1 == 0 and b2 in self._arm_nojaw) or (b2 == 0 and b1 in self._arm_nojaw): return True
        return False
    def ready_pose(self, max_lean=30.0, h_cam=0.5):
        """where the arm waits and passes through: the wrist camera raised over the front target, looking down, so the
        first look is a short move straight down (the old "straight up over the mount" was out of reach, and the solver
        settled 34 cm to the robot's right, so every pick swung right and back)"""
        import cluster_stops as cs_
        R = self.d.xmat[self.tr].reshape(3, 3); out = R[:2, 0]; tgt = self.d.xpos[self.tr][:2] + R[:2, :2] @ cs_.front_offset(max_lean)
        q = self.look_down(tgt - out * 0.08, out, h_cam=h_cam)
        if q is None: q, _ = self.ik(self.d.xpos[self.m.body("arm_base_link").id] + [0.2, 0, 0.35], [1, 0])
        return q
    def joint_efforts(self):
        """the six arm joints' actuator torques now (Nm): what the real arm reports as motor torque"""
        return self.d.actuator_force[12:18].copy()
    def settled_efforts(self, max_steps=400):
        """the joint torques once the arm has stopped moving, averaged over a few steps (as DimosRobot's): a weight
        read while the arm still moves includes the torque that moves it"""
        dof = [self.m.jnt_dofadr[self.m.actuator_trnid[12 + i, 0]] for i in range(6)]
        for _ in range(max_steps // 10):
            if np.max(np.abs(self.d.qvel[dof])) < 0.01: break
            self.step(10)
        out = []
        for _ in range(5): self.step(2); out.append(self.joint_efforts())
        return np.mean(out, axis=0)
    def set_payload(self, kg):
        """the held item's mass (kg) on the arm's gravity feed-forward, from the next control step ([cqG], [qaS])"""
        return ga.set_payload(self.m, kg)
    def held_mass(self, dtau, q):
        """the weight in the jaws (kg) from how much the joint torques rose at pose q: the least-squares fit of a
        vertical load at the grasp point"""
        m = self.m; k = self.kin; k.qpos[:] = self.d.qpos; k.qpos[self.aq] = q; mujoco.mj_kinematics(m, k); mujoco.mj_comPos(m, k)
        Jp = np.zeros((3, m.nv)); mujoco.mj_jacBody(m, k, Jp, None, self.gc); jz = Jp[2, self.ad]
        return float(jz @ np.asarray(dtau) / (9.81 * (jz @ jz) + 1e-12))
    def finger_low(self, q):
        """height of the lowest point of either finger at pose q (m): the tips sit ~3.4 cm beyond the grasp point"""
        m = self.m; k = self.kin; k.qpos[:] = self.d.qpos; k.qpos[self.aq] = q; mujoco.mj_kinematics(m, k)
        if not hasattr(self, "_finger_v"):
            fb = {m.body("arm_gripper_left").id, m.body("arm_gripper_right").id}; self._finger_v = []
            for i in range(m.ngeom):
                if m.geom_bodyid[i] in fb and m.geom_type[i] == mujoco.mjtGeom.mjGEOM_MESH:
                    mid = m.geom_dataid[i]; self._finger_v.append((i, m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].copy()))
        return float(min((k.geom_xpos[i] + v @ k.geom_xmat[i].reshape(3, 3).T)[:, 2].min() for i, v in self._finger_v))
    def ik_pos(self, pos, seeds=24):
        """grasp point at pos, any gripper orientation, collision-free (for dropping into the bin)"""
        best = (9, None); R_ = np.random.default_rng(3); m = self.m
        for s in range(seeds):
            q = self.d.ctrl[12:18].copy() if s == 0 else R_.uniform(self.rng[:, 0], self.rng[:, 1])
            for it in range(200):
                self.kin.qpos[:] = self.d.qpos; self.kin.qpos[self.aq] = q; mujoco.mj_kinematics(m, self.kin); mujoco.mj_comPos(m, self.kin)
                e = pos - self.kin.xpos[self.gc]
                if np.linalg.norm(e) < 0.004: break
                J = np.zeros((3, m.nv)); mujoco.mj_jacBody(m, self.kin, J, None, self.gc); Jq = J[:, self.ad]
                q = np.clip(q + Jq.T @ np.linalg.solve(Jq @ Jq.T + 1e-3 * np.eye(3), e), self.rng[:, 0], self.rng[:, 1])
            err = np.linalg.norm(e)
            if err < 0.01 and self._hits(q) == 0: return q, err
            if err < best[0]: best = (err, q.copy())
        return best[1], best[0]
    def _hits(self, q):
        """arm geoms touching the robot body, legs or tray at arm pose q (kinematic check, held object ignored)"""
        m = self.m; self.kin.qpos[:] = self.d.qpos; self.kin.qpos[self.aq] = q; mujoco.mj_fwdPosition(m, self.kin)
        if not hasattr(self, "_armb"):
            self._armb = {b for b in range(m.nbody) if (mujoco.mj_id2name(m, 1, b) or "").startswith("arm_")}
            root = m.body_rootid[m.body("trunk").id]            # the robot's own bodies only: never the items, the floor or props
            self._robot = {b for b in range(m.nbody) if b not in self._armb and m.body_rootid[b] == root}
        n = 0
        for c in self.kin.contact[:self.kin.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if (b1 in self._armb and b2 in self._robot) or (b2 in self._armb and b1 in self._robot) or (b1 in self._armb and b2 in self._armb): n += 1
        return n
    def _seg_free(self, qa, qb, n=40): return all(self._hits(qa + (qb - qa) * k / n) == 0 for k in range(1, n + 1))
    BIN_RELEASE = os.environ.get("HK_BIN_RELEASE", "corner")   # sim-grasp [oZb]: corner (the first found), centre, near
    BIN_LEAN_MAX = 40.0  # deg: the most a release pose may lean off straight down ([oZb])
    BIN_NEAR_M = 0.08    # m: "near" takes the least-leaning pose within this of the bin's middle ([oZb])
    def bin_pose(self):
        """one release pose for every stop (the lying posture is always the same). Computed once, then reused.
        corner: the lowest gripper-down, collision-free grasp-point position over the bin footprint, else the least-tilted
        one the IK happened on (on the lying Go1 that is 10 cm forward, 5 cm aside, tilted 39 deg: a 28 cm shoe let go
        there lands across the rim, 29 Sep). centre / near ([oZb]): of the points over the bin reached with the jaws across
        the robot (so a long item gripped across its width hangs along the bin's long side) within BIN_LEAN_MAX of
        straight down, the one nearest the bin's middle, then the least lean (centre), or the least lean within
        BIN_NEAR_M of the middle (near); then the lowest"""
        if getattr(self, "_q_bin", None) is not None: return self._q_bin
        m, d = self.m, self.d; tray = d.xpos[m.body("tray").id].copy(); Rt = d.xmat[self.tr].reshape(3, 3); best = None
        if self.BIN_RELEASE in ("centre", "near"):
            cands = []
            for h in (0.10, 0.14, 0.18, 0.22):
                for dx in (0.10, 0.05, 0.0, -0.05):
                    for dy in (0.0, 0.05, -0.05):
                        for lim in (5.0, 10.0, 20.0, 30.0, self.BIN_LEAN_MAX):
                            q, e, lean = self.ik_lean(tray + Rt @ np.array([dx, dy, h]), Rt[:, 1][:2], lim)
                            if q is not None and e < 0.01 and self._hits(q) == 0:
                                cands.append((float(np.hypot(dx, dy)), float(lean), h, dx, dy, q)); break
            if self.BIN_RELEASE == "near": cands = [c for c in cands if c[0] <= self.BIN_NEAR_M + 1e-9] or cands
            key = (lambda c: (round(c[0], 3), round(c[1] / 5), c[2])) if self.BIN_RELEASE == "centre" else (lambda c: (round(c[1] / 5), c[0], c[2]))
            if cands:
                dist, lean, h, dx, dy, q = min(cands, key=key)
                self._q_bin = q; self._bin_info = (h, dx, dy, lean); return q
        for h in (0.16, 0.20, 0.24, 0.28, 0.32, 0.36):
            for dx in (0.10, 0.05, 0.0, -0.05):
                for dy in (0.0, 0.04, -0.04):
                    q, e = self.ik(tray + Rt @ np.array([dx, dy, h]), Rt[:, 1][:2], seeds=10)
                    if e < 0.01 and self._hits(q) == 0: best = (h, dx, dy, q); break
                if best: break
            if best: break
        if best is None:                                  # no gripper-down pose reaches the bin: least-tilted collision-free one
            cands = []
            for h in (0.10, 0.14, 0.18, 0.22):
                for dx in (0.10, 0.05, 0.0, -0.05):
                    for dy in (0.0, 0.05, -0.05):
                        q, e = self.ik_pos(tray + Rt @ np.array([dx, dy, h]), seeds=12)
                        if e < 0.01 and self._hits(q) == 0:
                            self.kin.qpos[:] = d.qpos; self.kin.qpos[self.aq] = q; mujoco.mj_kinematics(m, self.kin)
                            ap = self.kin.xpos[self.gc] - self.kin.xpos[self.l7]; ap /= np.linalg.norm(ap)
                            cands.append((float(np.degrees(np.arccos(np.clip(-ap[2], -1, 1)))), h, dx, dy, q))
            if cands: tilt, h, dx, dy, q = min(cands, key=lambda c: (c[0], c[1])); best = (h, dx, dy, q, tilt)
        self._q_bin = best[3] if best else None; self._bin_info = (best[0], best[1], best[2], (best[4] if len(best) > 4 else 0.0)) if best else None
        return self._q_bin
    BIN_RIM = 0.06       # m, the bin's walls above the tray's origin (go1_arm: 3 cm half-height walls centred 3 cm up)
    def bin_clear_z(self, hang, margin=0.02):
        """world height the grasp point must be at for an item hanging `hang` m below it to clear the bin's rim"""
        return float(self.d.xpos[self.m.body("tray").id][2]) + self.BIN_RIM + hang + margin
    def over_bin(self, hang, margin=0.02):
        """a gripper-down pose over the bin's release point, high enough that an item hanging `hang` m below the grasp
        point clears the rim; the highest reachable one short of that; None if the release pose already clears it"""
        self.bin_pose()
        if self._bin_info is None: return None
        h, dx, dy, _ = self._bin_info; need = self.BIN_RIM + hang + margin
        if need <= h: return None
        tray = self.d.xpos[self.m.body("tray").id].copy(); Rt = self.d.xmat[self.tr].reshape(3, 3)
        for hh in np.arange(need, h, -0.02):
            q, e = self.ik(tray + Rt @ np.array([dx, dy, hh]), Rt[:, 1][:2], seeds=10)
            if e < 0.01 and self._hits(q) == 0: return q
        return None
    def plan(self, qa, qb, tries=300):
        """straight joint move if collision-free, else one random intermediate pose that clears both halves"""
        if self._seg_free(qa, qb): return [qb]
        for mid in [w for w in getattr(self, "waypoints", []) if w is not None]:          # known-safe poses first (arm raised over the mount)
            if self._hits(mid) == 0 and self._seg_free(qa, mid) and self._seg_free(mid, qb): return [mid, qb]
        R_ = np.random.default_rng(7)
        for t in range(tries):
            mid = (qa + qb) / 2 + R_.normal(0, 0.8, 6); mid = np.clip(mid, self.rng[:, 0], self.rng[:, 1])
            if self._hits(mid) == 0 and self._seg_free(qa, mid) and self._seg_free(mid, qb): return [mid, qb]
        print("   plan: no collision-free path found; moving straight", flush=True); return [qb]
    def arm_to_safe(self, q, secs, grip=None):
        path = self.plan(self.d.ctrl[12:18].copy(), q)
        for w in path: self.arm_to(w, secs / len(path) * 1.2, grip)
    ARM_VMAX = 1.0       # rad/s, every joint: half the lying-down safe speed from ./run tip (2.0 rad/s empty arm, items <= 0.5 kg)
    def arm_to(self, q, secs, grip=None):
        q0 = self.d.ctrl[12:18].copy(); g0 = self.d.ctrl[18]
        secs = max(secs, float(np.abs(np.asarray(q) - q0).max()) / self.ARM_VMAX * 1.6)   # smooth profile peaks ~1.6x the average speed
        n = int(secs / 0.005)
        for k in range(n):
            a = 0.5 - 0.5 * np.cos(np.pi * min(1, k / (0.85 * n)))
            self.d.ctrl[12:18] = (1 - a) * q0 + a * q
            if grip is not None: self.d.ctrl[18] = (1 - a) * g0 + a * grip
            self.step()
    def jaws(self, tag):
        m, d = self.m, self.d; gl = m.jnt_qposadr[m.joint("arm_gripper_left_joint").id]
        touch = sorted(set(mujoco.mj_id2name(m, 1, m.geom_bodyid[c.geom2 if m.geom_bodyid[c.geom1] == self.ob else c.geom1]) or "?" for c in d.contact[:d.ncon] if self.ob in (m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2])))
        print(f"   {tag:18s} jaw gap {2 * d.qpos[gl] * 100:.1f} cm, obj z {d.xpos[self.ob][2]:.3f}, touching {touch}", flush=True)
    def grip_to(self, g, secs=0.8): self.arm_to(self.d.ctrl[12:18].copy(), secs, grip=g)
    # --- what the pick routine (pick.py) needs; the DimOS-backed robot provides the same ---
    def capture(self, filament=None):                 # filament=False: a view only its geometry is used from (the teacher's)
        import wrist_refine as wr_; v = wr_.capture(self.m, self.d, filament=filament)
        if v.get("stereo_h") is not None: self.depth_inset = height_image(v["stereo_h"], v["robot"])
        return v
    def gripper(self):                         # half the gap between the gripping faces (the jaw reading, less any pad)
        return float(self.d.qpos[self.m.jnt_qposadr[self.m.joint("arm_gripper_left_joint").id]]) - jw.pad_m()
    def base_xy(self): return self.d.qpos[0:2].copy()
    def grasp_point(self): return self.d.xpos[self.m.body("arm_grasp_center").id].copy()
    def small_image(self):
        import filament_cameras as fc                                          # the policy's camera: Filament unless HK_RENDERER=classic
        if fc.enabled():                                                       # rendered full size and area-averaged down, as the real
            import cv2                                                         # camera's frame is scaled to 128x96 (rendering 128x96
            big = fc.render(self.m, self.d, "arm_wrist", 640, 480)            # directly aliases carpet and grain into moire); at the
            return cv2.resize(big, (128, 96), interpolation=cv2.INTER_AREA)   # close look's size, so both share one renderer
        if getattr(self, "_small", None) is None: self._small = mujoco.Renderer(self.m, 96, 128)
        import floors                                                          # carpet pile looks different from each side:
        floors.view_from(self.m, self.d.cam_xpos[self.m.camera("arm_wrist").id], renderers=(self._small,))   # the recorded frames too
        self._small.update_scene(self.d, "arm_wrist"); return self._small.render().copy()
    def joint_state(self): return np.r_[self.d.qpos[self.aq], self.gripper()]
    def kin_model(self): return self
    def target(self): return self.d.ctrl[12:19].copy()
    def command(self, q6, jaw): self.d.ctrl[12:18] = q6; self.d.ctrl[18] = jaw; self.step(20)

def height_image(h, robot, top_m=0.10, size=(320, 180)):
    """a stereo height map for the viewer: 0 (blue) to top_m (red) above the floor, black where the match failed, grey the
    robot ([A8K]); a 1 cm Lego piece shows a tenth of the way up"""
    import cv2
    v = np.clip(np.nan_to_num(h, nan=-1.0) / top_m, 0, 1); img = cv2.applyColorMap((v * 255).astype(np.uint8), cv2.COLORMAP_TURBO)[..., ::-1]
    img[~np.isfinite(h)] = 0; img[robot] = 60
    return cv2.resize(np.ascontiguousarray(img), size, interpolation=cv2.INTER_NEAREST)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--obj", default="toy"); ap.add_argument("--video", default=None)
    ap.add_argument("--obj-xy", type=float, nargs=2, default=[1.3, 0.35]); a = ap.parse_args()
    S = Sim(a.obj, np.array(a.obj_xy), a.video); m, d = S.m, S.d; res = {"object": a.obj}
    S.say("Phase 1: the floor picture flagged something new here. Walk over, arm stowed.")
    obj = d.xpos[S.ob][:2].copy(); yaw = 0.0
    park = obj - np.array([np.cos(yaw) * PARK_REL[0] - np.sin(yaw) * PARK_REL[1], np.sin(yaw) * PARK_REL[0] + np.cos(yaw) * PARK_REL[1]])
    res["park_error_m"] = round(float(S.walk_to(park, yaw)), 3)
    S.say("Phase 2: stop and lie down so the arm can reach the floor."); S.legs_to(legs.LIE, 2.0)
    o = d.xpos[S.ob].copy(); Ro = d.xmat[S.ob].reshape(3, 3); short_axis = Ro[:, 1][:2]
    S.say("Phase 3: look closer with the wrist camera.")
    mount = d.xpos[m.body("arm_base_link").id].copy()
    q_ready, _ = S.ik(mount + [0, 0, 0.45], short_axis)                    # arm raised straight up over the mount: safe waypoint
    S.arm_to_safe(q_ready, 1.8)
    q_look, e_look = S.ik(o + [0, 0, 0.22], short_axis); S.arm_to(q_look, 2.0); S.step(100)
    if S.video: 
        from PIL import Image; big = mujoco.Renderer(m, 480, 640); big.update_scene(d, "arm_wrist"); Image.fromarray(big.render()).save(HERE / f"lookcloser_{a.obj}.png")
    res["look_ik_err_cm"] = round(e_look * 100, 1)
    S.say("Phase 4: open, come down, grip."); S.grip_to(0.05)
    q_pre, e_pre = S.ik(o + [0, 0, 0.08], short_axis); S.arm_to(q_pre, 1.5)
    q_g, e_g = S.ik(o + [0, 0, max(0.0, OBJS[a.obj]["half"][2] - 0.01)], short_axis); S.arm_to(q_g, 1.2)
    res["grasp_ik_err_cm"] = round(e_g * 100, 1); S.jaws("at grasp"); S.grip_to(0.0, 1.0); S.step(100); S.jaws("closed")
    S.say("Lift."); q_up, _ = S.ik(o + [0, 0, 0.18], short_axis); S.arm_to(q_up, 1.5); S.step(60)
    res["lifted_cm"] = round(float(d.xpos[S.ob][2] - o[2]) * 100, 1); S.jaws("lifted")
    S.say("Put it in the tray on the back.")
    tray = d.xpos[m.body("tray").id].copy(); Rt = d.xmat[S.tr].reshape(3, 3)
    q_tray0, _ = S.ik(tray + [0, 0, 0.18], short_axis); S.arm_to(q_tray0, 2.5)          # carry over without twisting the wrist
    q_tray, e_tray = S.ik(tray + [0, 0, 0.14], Rt[:, 0][:2])      # then turn slowly in place: long items land crosswise
    S.arm_to(q_tray, 2.0); S.jaws("over tray"); S.grip_to(0.05, 0.8); S.step(400); S.jaws("opened")
    rel = Rt.T @ (d.xpos[S.ob] - tray); res["tray_ik_err_cm"] = round(e_tray * 100, 1); res["drop_rel_tray_cm"] = [round(float(v) * 100, 1) for v in rel]; res["in_tray"] = bool(abs(rel[0]) < 0.15 and abs(rel[1]) < 0.11 and -0.01 < rel[2] < 0.08)
    S.say("Stow, stand up, carry on.")
    q_clear, _ = S.ik(tray + [0, 0, 0.32], Rt[:, 0][:2]); S.arm_to(q_clear, 1.2)             # straight up out of the tray first
    S.arm_to_safe(q_ready, 2.0); S.arm_to_safe(np.array(pt.STOW[:6]), 3.0, grip=0.0)   # collision-checked moves back to stow
    S.jaws("stowed")
    armb = {b for b in range(m.nbody) if (mujoco.mj_id2name(m, 1, b) or "").startswith("arm_")}
    hits = sorted({(mujoco.mj_id2name(m, 1, m.geom_bodyid[c.geom1]), mujoco.mj_id2name(m, 1, m.geom_bodyid[c.geom2])) for c in d.contact[:d.ncon] if (m.geom_bodyid[c.geom1] in armb) != (m.geom_bodyid[c.geom2] in armb)})
    print("   stow joint errors (rad):", np.round(d.qpos[S.aq] - np.array(pt.STOW[:6]), 2), "arm contacts:", hits, "arm torque:", np.round(d.actuator_force[12:18], 1))
    res["stow_err_rad"] = round(float(np.abs(d.qpos[S.aq] - np.array(pt.STOW[:6])).max()), 3)
    S.legs_to(legs.STAND, 2.5); S.walk_to(d.qpos[0:2] + [0.8, 0.0], 0.0, max_s=6)
    Rt = d.xmat[S.tr].reshape(3, 3); rel = Rt.T @ (d.xpos[S.ob] - d.xpos[m.body("tray").id])
    res["still_in_tray_after_walk"] = bool(abs(rel[0]) < 0.15 and abs(rel[1]) < 0.11 and -0.01 < rel[2] < 0.08)
    res["max_tilt_deg"] = round(S.max_tilt, 1); res["sim_seconds"] = round(S.t, 1)
    print(json.dumps(res)); (HERE / f"vslice_{a.obj}.json").write_text(json.dumps({"result": res, "log": S.log}, indent=1))
    if a.video and S.frames: write_video(S, a.video)

def write_video(S, path):
        import subprocess
        from PIL import Image, ImageDraw
        tmp = HERE / "_frames"; tmp.mkdir(exist_ok=True)
        for f in tmp.glob("*.png"): f.unlink()
        for i, (img, cap, fast) in enumerate(S.frames):
            im = Image.fromarray(img); dr = ImageDraw.Draw(im); dr.rectangle([0, 450, 640, 480], fill=(0, 0, 0)); dr.text((10, 458), cap + ("   [3x speed]" if fast else ""), fill=(255, 255, 255))
            dr.text((8, 8), "SIM: Go1 body + Maker Arm URDF, scripted; perception-driven after setup", fill=(40, 40, 40))
            dr.text((640 - 8 - 160, 8 + 122), "wrist camera", fill=(255, 255, 255)); im.save(tmp / f"{i:05d}.png")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", "28", "-i", str(tmp / "%05d.png"), "-pix_fmt", "yuv420p", path], check=True)
        print("video:", path)
if __name__ == "__main__": main()
