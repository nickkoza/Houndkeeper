#!/usr/bin/env python3
"""Sim Go1 (DimOS's stand-in for the Go2) with MakerMods' Maker Arm URDF mounted on its back.
Arm masses/limits from the URDF (3.38 kg); motor gains/torque limits from maker_arm_v1.yaml (RS02 17 Nm, RS00 14 Nm).
Arm geoms are visual/inertial only for now (no collisions): this model is for payload/stability tests."""
import math
import os
import sys
import weakref
from pathlib import Path
import numpy as np, mujoco
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src")); from houndkeeper import workspace   # noqa: E402  (decision [cy4])
DIMOS = workspace.dimos()                       # the DimOS checkout beside this repo
sys.path.insert(0, str(DIMOS))
from dimos.simulation.mujoco import model as dm
from houndkeeper import params as P
import jaws as jw                               # the jaws' geometry (HK_JAWS, pads, widest item)

HERE = Path(__file__).resolve().parent
DATA = DIMOS / "data" / "mujoco_sim"
ARM_JOINTS = ["link_002_joint", "link_003_joint", "link_004_joint", "link_005_joint", "link_006_joint", "link_007_joint", "gripper_left_joint"]
ARMATURE = [0.02, 0.04, 0.04, 0.02, 0.02, 0.02, 0.01]   # est.: RS02/RS00 rotor inertia x gear ratio^2; unpublished
KP = [60, 150, 90, 30, 30, 30, 20]; KD = [4.0, 4.5, 3.0, 2.0, 2.0, 2.0, 0.5]; TAU = [14, 17, 17, 14, 14, 14, 14]
from bin_geom import TRAY_X, TRAY_HALF        # the bin, 30 x 22 cm, 15 cm behind the trunk's centre: one place (bin_geom.py)
ARM_MASS_KG = 3.7                                   # official MakerMods figure, without power supply or e-stop
MOUNT = (P.get("maker_arm.mount_x_m"), 0.0, 0.058 + P.get("maker_arm.mount_plate_m"))   # from params/robot.yaml; 0.058 = Go1 trunk top


JAW_BARE = list(P.get("maker_arm.jaw_friction_bare"))   # sliding, torsional (m), rolling: params/robot.yaml (estimates)


NEAR_CLIP_M = 0.005                            # cameras draw everything from 5 mm out


def near_clip(model):
    """MuJoCo clips each camera at vis.map.znear x the model's extent: 1% of a 26 m house is 26 cm, and the wrist camera,
    18 cm above an item on a close look, saw straight through it and its own jaws (found 25 Sep). Call
    after the extent is final."""
    model.vis.map.znear = NEAR_CLIP_M / max(float(model.stat.extent), 1e-6)


ROOM_LIGHTS = {"ceiling": 3, "lamp": 2, "window": 2, "sun": 1}   # copies of each (soft shadows); at most 8 on at once


def add_room_lights(spec):
    """the standalone sims' room lights, all off until Sim.relight draws the episode's lighting (the house nights'
    regimes: a warm ceiling light and lamp by night, window light and sometimes sun by day)"""
    for name, n in ROOM_LIGHTS.items():
        for i in range(n):
            L = spec.worldbody.add_light(); L.name = f"{name}_{i}"; L.active = False; L.castshadow = True
            if name == "sun": L.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
            else: L.cutoff = 80; L.exponent = 1


def add_jaws(arm, jaws=None):
    """the jaws' gripping surfaces (HK_JAWS): bare fingers keep their own meshes; pads add the printed pad's shape
    (cad/gripper_pads, built from the finger's CAD) to each finger, the left mirrored"""
    jaws = jaws or jw.JAWS
    if jaws == "rubber": return
    for side in ("left", "right"):                 # priority: the jaw's material sets its contacts with items (MuJoCo would
        for g in arm.body(f"gripper_{side}").geoms:    # otherwise take the grippier of the two, i.e. the item's, every time)
            if g.contype: g.friction = list(JAW_BARE); g.priority = 1
    if not jaws.startswith("pads_"): return
    H = jw.pad_file(jaws)
    for side, sy in (("right", 1.0), ("left", -1.0)):
        b = arm.body(f"gripper_{side}")
        for k in sorted(int(n[5:]) for n in H.files if n.startswith("hull_")):
            v = H[f"hull_{k}"] * np.array([1.0, sy, 1.0])
            ms = arm.add_mesh(); ms.name = f"pad_{side}_{k}"; ms.uservert = v.flatten().tolist()
            g = b.add_geom(); g.name = ms.name; g.type = mujoco.mjtGeom.mjGEOM_MESH; g.meshname = ms.name; g.density = 0; g.mass = 0.0
            mu_t = float(os.environ.get("HK_PAD_TORSION", H["mu_t"]))          # the design's own contact values (build_pads.contact:
            g.contype = 2; g.conaffinity = 3; g.condim = 4; g.friction = [float(H["mu"]), mu_t, 0.0001]   # sliding, twist from its
            g.solimp = [0.6, 0.95, max(float(H["give"]), 0.0005), 0.5, 2]       # patch; its give: MuJoCo's contact softens over this depth)
            g.priority = 1
            g.rgba = [0.15, 0.15, 0.17, 1]


WRIST_STEREO = os.environ.get("HK_WRIST_STEREO", "0") == "1"   # two ELP 720p (OV9712) cameras on the mount (waymark 0011 [A8K])
STEREO_BASELINE_M = float(os.environ.get("HK_STEREO_BASELINE", "0.04"))   # 40 mm: outlines swell less than at 50 (waymark 0011, 29 Sep)
STEREO_LENS_MM = float(os.environ.get("HK_STEREO_LENS_MM", "2.31"))  # the Arducam M40210M09S ("2.1 mm") as fitted to its maker's
                                                                     # reference photo (29 Sep); the OV9712's 720 rows are 2.16 mm
STEREO_FOVY = float(np.degrees(2 * np.arctan(0.00108 / (STEREO_LENS_MM / 1000))))
STEREO_FULL = os.environ.get("HK_STEREO_FULL", "1") == "1"   # the close look at the cameras' whole 1280 x 720 (29 Sep)
STEREO_CAL_ERR_DEG = float(os.environ.get("HK_STEREO_CAL_DEG", "0.05"))   # what a checkerboard calibration leaves (est.)
WRIST_LIGHT = os.environ.get("HK_WRIST_LIGHT", "0") == "1"   # a steady white light for the close look (waymark 0010 [J7G])
WRIST_LIGHT_CLASSIC = 0.04    # the classic renderer's diffuse: ~0.45 on a white floor at 30 cm, beside the room lights'
                              # ~0.2-0.5 (the viewer only; Filament takes diffuse x its unit, filament_cameras.UNITS)
WRIST_LIGHT_OFFSET_M = 0.035
WRIST_LIGHT_SPREAD_M = 0.045  # with the pair: the outer LEDs this far either side of the middle (just past each lens)                                # from the lens, on the side away from the jaws (est.: the printed mount)
ARM_BLACK = os.environ.get("HK_ARM_BLACK", "1") == "1"   # waymark 0008 [ipj]; on from round 12, by default in every sim from 29 Sep
PRINTED_RGBA = [0.035, 0.035, 0.04, 1.0]      # black PLA/PETG's albedo (est.); MakerMods: "black 3D-printed structure"
PRINTED_ROUGHNESS = 0.6                       # printed plastic: a soft sheen, not a mirror (est.)


def paint_arm(arm):
    """the arm drawn as the real one looks: its printed structure black and matte (MakerMods' product page, observed
    28 Sep 2026). Its metal (the RobStride actuators' fittings, screws) isn't told apart in the URDF's unnamed parts and
    stays black too (not modelled). The jaw pads are black too: the arm and its jaws are all black"""
    mat = arm.add_material(); mat.name = "printed_black"; mat.rgba = PRINTED_RGBA
    mat.roughness = PRINTED_ROUGHNESS; mat.metallic = 0.0
    for g in arm.geoms:
        g.material = "printed_black"; g.rgba = PRINTED_RGBA[:3] + [g.rgba[3]]   # hidden parts stay hidden


CTRL_DT = 0.005                 # the sims' control step, and the physics steps in it (params sim.physics_dt_s: the printed
SUBSTEPS = int(round(CTRL_DT / P.get("sim.physics_dt_s")))   # frame's flex, [z7P], is stable only that fine, 27 Sep)


def step(m, d, n=1):
    """advance n control steps (CTRL_DT each) of physics: every sim steps through here, never mj_step alone"""
    for _ in range(n):
        update_gravity_ff(m, d)                    # once per control step, as the arm's controller sends it ([cqG])
        for _ in range(SUBSTEPS): mujoco.mj_step(m, d)


# The arm's gravity feed-forward ([cqG], [Y2p]): the DimOS adapter's own code (gravity.py, numpy only, loaded by file
# path from the driver worktree), on a model built from the SDK's URDF with a stated, seeded error ([x3f]), written
# into each arm actuator's bias constant so force = kp (ctrl - q) + ff and the motor's force limit still applies.
GRAVITY_PY = Path(os.environ.get("HK_GRAVITY_PY", str(DIMOS.parent / "dimos-maker-arm-driver/dimos/hardware/manipulators/maker_arm/gravity.py")))
SDK_URDF = Path(os.environ.get("HK_MAKER_ARM_URDF", str(DIMOS.parent / "maker-arm-sdk/urdf/maker_arm/robot.urdf")))
FF_JOINTS = ARM_JOINTS[:6]                                  # the revolute chain; the jaws' slide carries no feed-forward
FF_MOTORS = ["RS02" if t == 17 else "RS00" for t in TAU[:6]]   # by peak torque: RS02 17 N m, RS00 14 N m
G_WORLD = np.array([0.0, 0.0, -9.81])
_GRAVITY = {}                                               # the loaded module, once
_FF = weakref.WeakKeyDictionary()                           # per compiled model: its feed-forward and addresses


def gravity_module():
    """dimos/hardware/manipulators/maker_arm/gravity.py by file path: the same code the arm host runs ([Y2p]), without
    importing DimOS's hardware package (the main DimOS checkout has no maker_arm yet)"""
    if "mod" not in _GRAVITY:
        import importlib.util
        spec_ = importlib.util.spec_from_file_location("maker_arm_gravity", GRAVITY_PY)
        mod = importlib.util.module_from_spec(spec_); sys.modules["maker_arm_gravity"] = mod   # dataclasses look it up
        spec_.loader.exec_module(mod); _GRAVITY["mod"] = mod
    return _GRAVITY["mod"]


def gravity_model_perturbed(err=None):
    """the controller's belief about the arm: the SDK URDF's links, each mass scaled by 1 + N(0, mass_sd) and each centre
    of mass moved by N(0, com_sd_m) per axis, from a fixed seed (params maker_arm.gravity_ff_model_error, estimates)"""
    import dataclasses
    G = gravity_module(); err = err or P.get("maker_arm.gravity_ff_model_error")
    base = G.GravityModel.from_urdf(SDK_URDF, joints=FF_JOINTS)
    rng = np.random.default_rng(int(err["seed"]))
    links = [dataclasses.replace(lk, mass=lk.mass * max(0.2, 1.0 + rng.normal(0.0, err["mass_sd"])),
                                 com=tuple(float(c) for c in np.add(lk.com, rng.normal(0.0, err["com_sd_m"], 3))))
             for lk in base.links]
    return G.GravityModel(links, tip=base.tip)


def gravity_ff(m):
    """the model's feed-forward state, made on first use: GravityFeedForward at the adapter's rated caps, full from the
    start (the sim's arm starts armed), with mass_scale set so the model carries gravity_ff_fraction of the sim arm's
    true moving mass: the share of the weight cancelled ([x3f]); how it is spread over the links is the model's error"""
    st = _FF.get(m)
    if st is None:
        G = gravity_module(); model = gravity_model_perturbed()
        b0 = m.body("arm_link_002").id; true_kg = float(m.body_subtreemass[b0])
        share = P.get("maker_arm.gravity_ff_fraction")
        ff = G.GravityFeedForward(model, [G.RATED_TORQUE_NM[k] for k in FF_MOTORS], scale=1.0,
                                  mass_scale=share * true_kg / model.mass)
        ff.arm(0.0, ramped=True)
        base = m.body("arm_base_link").id; trunk = m.body("trunk").id
        assert m.body_parentid[base] == trunk, "the arm's base hangs off the trunk"
        R_mount = np.zeros(9); mujoco.mju_quat2Mat(R_mount, m.body_quat[base])
        st = dict(ff=ff, model=model, true_kg=true_kg, trunk=trunk, R_mount=R_mount.reshape(3, 3),
                  act=np.array([m.actuator("arm_" + j).id for j in FF_JOINTS]))
        _FF[m] = st
    return st


def update_gravity_ff(m, d=None):
    """write the feed-forward for the commanded arm pose into the arm actuators' bias constant (biasprm[:, 0]), with
    gravity turned into the arm base's frame by the trunk's attitude (the IMU's job on the robot; the frame's flex is
    unseen). No data: the first keyframe's command, upright (seeding a fresh model)."""
    st = _FF.get(m)
    if st is None:
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "arm_" + FF_JOINTS[0]) < 0: return None   # no arm
        st = gravity_ff(m)
    if d is None: R_trunk = np.eye(3); q = m.key_ctrl[0, st["act"]] if m.nkey else np.zeros(len(st["act"]))
    else: R_trunk = d.xmat[st["trunk"]].reshape(3, 3); q = d.ctrl[st["act"]]
    st["ff"].set_gravity((R_trunk @ st["R_mount"]).T @ G_WORLD)
    tau = st["ff"].torques(q, 0.0)
    m.actuator_biasprm[st["act"], 0] = tau
    return tau


# The held item's mass on the feed-forward ([qaS]): set by the pick once weighed, zero once released. DimOS's sim steps
# the physics in another process, so there the pick writes it to a /dev/shm file (named per launch in sim/dimos/env.py)
# and sim_hook's per-frame hook reads it.
PAYLOAD_ENV = "HK_SIM_PAYLOAD"
_PAYLOAD_SEEN = {}                                          # path -> (the file's identity, kg) as last read


def set_payload(m, kg):
    """the held item's mass (kg) on this model's feed-forward, at the grasp point; returns the mass set (None: no arm)"""
    st = _FF.get(m)
    if st is None:
        if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "arm_" + FF_JOINTS[0]) < 0: return None
        st = gravity_ff(m)
    st["ff"].set_payload(max(0.0, float(kg)))
    return st["ff"].payload_kg


def write_payload(kg, path):
    """the held mass for the sim process, written whole and swapped in, so a reader never meets half a file"""
    import json
    with open(f"{path}.tmp", "w") as fh: json.dump({"kg": max(0.0, float(kg))}, fh)
    os.replace(f"{path}.tmp", path)


def payload_from_file(m, path=None):
    """set the model's feed-forward payload from the payload file (path, else $HK_SIM_PAYLOAD): read again only when
    the file has changed, so once a frame it is a stat. No path: untouched (None). No file, or a bad one: nothing held"""
    import json
    path = path or os.environ.get(PAYLOAD_ENV)
    if not path: return None
    try: st_ = os.stat(path); key = (st_.st_ino, st_.st_mtime_ns)
    except OSError: key = None
    seen = _PAYLOAD_SEEN.get(path)
    if seen is not None and seen[0] == key: kg = seen[1]
    else:
        try: kg = float(json.load(open(path))["kg"]) if key is not None else 0.0
        except Exception: kg = 0.0
        _PAYLOAD_SEEN[path] = (key, kg)
    return set_payload(m, kg)


FLEX_MODE_RAD_S = 60.0       # each flex joint's own mode is held at or below this (~10 Hz) by giving it inertia k / w^2: its
                             # stiffness (the droop) is kept, its shake slowed. At its bare inertia the frame's fastest mode was
                             # 223 rad/s, past the 4-5 ms step's limit: walking, the arm went unstable and the robot fell (27 Sep;
                             # a quarter step walked fine). A printed arm's first modes are ~5-20 Hz (est.)
LASH_AS_SPRING = ("link_006", "link_007")
WRIST_LOAD_NM = 0.5          # the wrist's typical load (gripper and camera out on a lever, est.): its play is used up at this


def add_imperfections(spec):
    """What the encoders can't see ([x3f]): the printed frame bending at the mount and along the arm (a spring hinge
    in-plane and one out-of-plane per place, lightly damped as plastic is), and each gearbox's free play after its
    encoder (a hinge on the motor's axis, free within +-backlash, hard stops at the ends). Passive joints, no
    actuator; a kinematic copy leaves them at zero, which is the nominal arm the robot believes it has."""
    flex = P.get("maker_arm.frame_flex_Nm_per_rad"); lash = P.get("maker_arm.backlash_deg")
    only = os.environ.get("HK_ARM_IMPERFECT", "flex,lash").split(",")      # all on by default ([x3f]); for isolating one
    flex = flex if "flex" in only else {}; lash = lash if "lash" in only else {}
    names = {j.name for j in spec.joints}
    for body, k in flex.items():
        b = spec.body("arm_" + body)
        out = [1, 0, 0] if body == "base_link" else [0, 1, 0]   # across the link, never about its length (x: torsion,
        for tag, axis in (("in", [0, 0, 1]), ("out", out)):       # grams of inertia, numerically unstable); the base's y is up
            j = b.add_joint(); j.name = f"arm_{body}_flex_{tag}"; j.type = mujoco.mjtJoint.mjJNT_HINGE; j.axis = axis
            j.stiffness = [float(k), 0.0, 0.0] if hasattr(j.stiffness, "__len__") else float(k)
            j.armature = k / FLEX_MODE_RAD_S ** 2; j.damping = [0.05 * 2 * math.sqrt(k * j.armature), 0.0, 0.0]   # 5% of critical
    for jn, deg in lash.items():
        if f"arm_{jn}_joint" not in names: continue
        motor = spec.joint(f"arm_{jn}_joint"); b = spec.body("arm_" + jn)
        j = b.add_joint(); j.name = f"arm_{jn}_lash"; j.type = mujoco.mjtJoint.mjJNT_HINGE; j.axis = list(motor.axis)
        r = math.radians(deg); j.damping = [0.2, 0.0, 0.0]; j.armature = 0.01
        if jn in LASH_AS_SPRING:                         # the wrist's joints carry grams: hard stops there were numerically
            j.stiffness = [WRIST_LOAD_NM / r, 0.0, 0.0]  # unstable (4 ms step), so their play is a spring taking it up under load
        else: j.range = [-r, r]; j.limited = 1


def add_arm(spec, arm_q=None, mount=None, pick=True):
    """Everything Maker-Arm-specific, added to a DimOS Go1 MjSpec: the arm on its mount (turned 180 deg, factory fold
    forward), rotor inertia, gains and torque limits, gripper, wrist camera, head collision shell and the bin. Used by
    the standalone sims (build) and by DimOS's own sim through houndkeeper.sim_hook. Returns post(model), which fills
    the arm's part of the keyframes once the model is compiled."""
    arm = mujoco.MjSpec.from_file(str(HERE / "maker_arm.xml"))
    scale = ARM_MASS_KG / sum(b.mass for b in arm.bodies if b.mass > 0)      # URDF says 3.38 kg; MakerMods' official figure is 3.7 kg
    for b in arm.bodies:
        if b.mass > 0: b.mass *= scale; b.inertia = [v * scale for v in b.inertia]
    for g in arm.geoms:
        if pick: g.contype = 2; g.conaffinity = 3; g.condim = 4; g.friction = [1.2, 0.02, 0.001]   # hits floor, robot, objects and itself
        else: g.contype = 0; g.conaffinity = 0
    if pick: add_jaws(arm)
    if ARM_BLACK: paint_arm(arm)
    if pick:                                              # wrist camera on the last link, looking down the gripper (placement est.)
        cam = arm.body("link_007").add_camera(); cam.name = "wrist"; cam.fovy = 65   # FOV est.
        # at the camera module on MakerMods' own bracket (URDF parts 054/056/057; the 4 small parts 058-061 around
        # (0.093, 0, -0.06) taken as the lens mount): 9.3 cm out along link 7 x, i.e. perpendicular to the jaw travel (y).
        # Aimed at the fingertips; image "up" points away from the gripper (the camera sits above it). Aim/FOV are est.
        p_ = np.array([0.093, 0.0, -0.065]); f_ = np.array([0.0, 0.0, -0.20]) - p_; f_ /= np.linalg.norm(f_); p_ = p_ + 0.015 * f_   # lens just outside the housing
        z_ = -f_; x_ = np.cross([1.0, 0, 0], z_); x_ /= np.linalg.norm(x_); y_ = np.cross(z_, x_)
        qc = np.zeros(4); mujoco.mju_mat2Quat(qc, np.column_stack([x_, y_, z_]).flatten()); cam.pos = list(p_); cam.quat = list(qc)
        import ir_camera                                   # the infrared trial (waymark [6hI]): its lens where this one is, the
        if ir_camera.ON:                                   # B0332's view, and four 940 nm LEDs round it on the printed mount
            cam.fovy = ir_camera.FOVY_DEG; ir_camera.add_leds(arm.body("link_007"), p_, x_, y_, f_)
        if WRIST_STEREO:                                   # the pair: this camera the left one, the right one a baseline
            cam.fovy = STEREO_FOVY                         # along the image's x, pointing the same way but for a calibration's
            cr = arm.body("link_007").add_camera(); cr.name = "wrist_r"; cr.fovy = STEREO_FOVY   # residual (it knows it as parallel)
            ang = np.radians(STEREO_CAL_ERR_DEG); ax = np.array([0.3, 1.0, 0.2]); ax /= np.linalg.norm(ax)
            qe = np.r_[np.cos(ang / 2), np.sin(ang / 2) * ax]; qr = np.zeros(4); mujoco.mju_mulQuat(qr, qc, qe)
            cr.pos = list(p_ + STEREO_BASELINE_M * x_ + np.array([0.0001, 0.0, 0.0])); cr.quat = list(qr)
            if STEREO_FULL:                                # the classic renderers (the robot's own pixels) draw at the
                g_ = spec.visual.global_                   # cameras' whole 1280 x 720: MuJoCo's off-screen buffer must hold it
                g_.offwidth = max(int(g_.offwidth), 1280); g_.offheight = max(int(g_.offheight), 720)
        if WRIST_LIGHT:                                    # white LEDs built into the camera holder ([J7G]): with the pair, three
            mid = p_ + (STEREO_BASELINE_M / 2 * x_ if WRIST_STEREO else 0)   # in a row, midway and out past each lens, for an
            row = (-WRIST_LIGHT_SPREAD_M, 0.0, WRIST_LIGHT_SPREAD_M) if WRIST_STEREO else (0.0,)   # even field and soft shadows
            for k, dx in enumerate(row):                   # the one light's current shared among them
                L = arm.body("link_007").add_light(); L.name = "wrist_light" if len(row) == 1 else ("wrist_light_l", "wrist_light", "wrist_light_r")[k]
                L.active = False; L.castshadow = True
                L.pos = list(mid + dx * x_ + WRIST_LIGHT_OFFSET_M * y_); L.dir = list(f_); L.cutoff = 80; L.exponent = 1
                L.diffuse = [WRIST_LIGHT_CLASSIC / len(row)] * 3; L.specular = [0, 0, 0]; L.ambient = [0, 0, 0]
                L.attenuation = [0.0, 0.0, 1.0]            # inverse square (the viewer's classic renderer; Filament its own)
    qx = np.array([0.7071068, 0.7071068, 0, 0]); qz = np.array([0.0, 0, 0, 1.0]); qm = np.zeros(4); mujoco.mju_mulQuat(qm, qz, qx)
    frame = spec.body("trunk").add_frame(pos=list(mount or MOUNT), quat=list(qm))   # arm's +y up -> world +z; turned 180 deg so the factory fold points the gripper forward
    frame.attach_body(arm.body("base_link"), "arm_", "")
    for b_ in spec.bodies:                                   # no MuJoCo gravcomp: the arm's weight is held by its actuators, the
        if b_.name.startswith("arm_"): b_.gravcomp = 0       # adapter's feed-forward (update_gravity_ff, [cqG]) on the bias
    for j in ARM_JOINTS + ["gripper_right_joint"]:           # constant plus the PD springs, which sag by what the model misses
        spec.joint("arm_" + j).actgravcomp = False
    add_imperfections(spec)
    for j, arm_i in zip(ARM_JOINTS, ARMATURE):          # reflected rotor inertia + joint friction (est.; not in the URDF)
        jt = spec.joint("arm_" + j); jt.armature = arm_i; jt.damping = [0.3, 0, 0]
    spec.joint("arm_gripper_right_joint").armature = ARMATURE[-1]; spec.joint("arm_gripper_right_joint").damping = [0.3, 0, 0]
    for j, kp, kd, tau in zip(ARM_JOINTS, KP, KD, TAU):
        a = spec.add_actuator(); a.name = "arm_" + j; a.target = "arm_" + j; a.trntype = mujoco.mjtTrn.mjTRN_JOINT
        a.gaintype = mujoco.mjtGain.mjGAIN_FIXED; a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        a.gainprm[0] = kp; a.biasprm[0:3] = [0, -kp, 0]; a.forcelimited = 1; a.forcerange = [-tau, tau]
        jt = spec.joint("arm_" + j); jt.damping = [float(np.ravel(jt.damping)[0]) + kd, 0, 0]   # the motor's kd (MIT mode, velocity
        # target 0: tau = kp e - kd qdot) as joint damping, which MuJoCo's Euler step takes implicitly: as an actuator bias it is
        # explicit, and once the frame's flex left each motor less inertia to damp, it overshot at the 4-5 ms step (27 Sep)
    if pick:
        a = next(x for x in spec.actuators if x.name == "arm_gripper_left_joint")    # jaw squeeze ~15 N max (est.; firmware sets it via a compliant kp)
        a.gainprm[0] = 3000; a.biasprm[0:3] = [0, -3000, -10]; a.forcerange = [-15, 15]   # stiff enough that any item over ~1 cm gets the full squeeze, as a torque-limited servo does
        for b_ in ("trunk",): 
            for arm_b in ("arm_base_link", "arm_link_002"):          # these sit on the mount plate; the upper arm must not pass through the trunk
                ex = spec.add_exclude(); ex.bodyname1 = b_; ex.bodyname2 = arm_b
        ex = spec.add_exclude(); ex.bodyname1 = "arm_gripper_left"; ex.bodyname2 = "arm_gripper_right"   # closed jaws touch by design
        shell = spec.body("trunk").add_geom(); shell.name = "body_shell"; shell.type = mujoco.mjtGeom.mjGEOM_BOX   # the Go1 collision is two
        shell.pos = [0.0835, 0, 0.006]; shell.size = [0.2155, 0.097, 0.067]; shell.group = 3                   # cylinders to +-12.5 cm; the
        shell.contype = 0; shell.conaffinity = 2; shell.mass = 0; shell.density = 0                             # head (to +30 cm) had none. Hit by
        # the arm only (contype 2): legs and floor ignore it, so walking is unchanged.
        tray = spec.body("trunk").add_body(); tray.name = "tray"; tray.pos = [TRAY_X, 0, 0.058 + 0.005]   # printed bin over the rear back, under the folded arm
        hx, hy = TRAY_HALF
        for sx, sy, px, py in ((hx, hy, 0, 0), (hx, 0.004, 0, hy - 0.004), (hx, 0.004, 0, -hy + 0.004), (0.004, hy, hx - 0.004, 0), (0.004, hy, -hx + 0.004, 0)):
            g = tray.add_geom(); g.type = mujoco.mjtGeom.mjGEOM_BOX; g.size = [sx, sy, 0.004 if sy > 0.01 and sx > 0.01 else 0.03]
            g.pos = [px, py, 0 if (sx > 0.01 and sy > 0.01) else 0.03]; g.rgba = [0.85, 0.55, 0.2, 1]; g.mass = 0.02; g.contype = 1; g.conaffinity = 1
    eq = spec.add_equality(); eq.type = mujoco.mjtEq.mjEQ_JOINT; eq.name1 = "arm_gripper_right_joint"; eq.name2 = "arm_gripper_left_joint"
    q = list(arm_q if arm_q is not None else np.zeros(7)); qfull = q[:7] + [q[6]]         # right jaw follows left
    def post(m):                                     # attach grew the keyframes with zeros; fill in the arm part by address
        for j in range(m.njnt):                      # the arm's flex and play ([nAH]), added after attach, start at rest
            n = m.joint(j).name
            if "_flex_" in n or n.endswith("_lash"): m.key_qpos[:, m.jnt_qposadr[j]] = 0.0
        names = ["arm_" + j for j in ARM_JOINTS] + ["arm_gripper_right_joint"]
        for jn, v in zip(names, qfull): m.key_qpos[:, m.jnt_qposadr[m.joint(jn).id]] = v
        for jn, v in zip(ARM_JOINTS, q[:7]): m.key_ctrl[:, m.actuator("arm_" + jn).id] = v
        update_gravity_ff(m)                         # seeded at the keyframe's command, upright ([cqG])
    return post


def build(with_arm=True, arm_q=None, scene="scene_empty.xml", mount=None, pick=False, objects=(), floor_tex=None, floor_half=1.5, local_floor=False, lib=(), dress=None):
    """pick=True: arm collides (floor, legs, objects), realistic gripper, wrist camera, tray on the back. objects: dicts for build_objects."""
    flat = HERE / f"_flat_{Path(scene).stem}.xml"            # DimOS's include/asset-dict model, resolved to one file
    if not flat.exists():
        xml = dm.get_model_xml("unitree_go1", (DATA / scene).read_text(), add_person=False)   # as our DimOS runs ([UvE])
        mujoco.MjModel.from_xml_string(xml, assets=dm.get_assets()); tmp = HERE / "_tmp.xml"
        mujoco.mj_saveLastXML(str(tmp), mujoco.MjModel.from_xml_string(xml, assets=dm.get_assets()))
        flat.write_text(tmp.read_text()); tmp.unlink()
    spec = mujoco.MjSpec.from_file(str(flat), assets=dm.get_assets())
    person = next((b for b in spec.bodies if b.name == "person"), None)   # DimOS's decorative figure (gone from its scenes
    for g in (person.geoms if person else []): g.contype = 0; g.conaffinity = 0; g.rgba = [0, 0, 0, 0]   # since 26 Sep): at the origin, where the robot spawned
    post = add_arm(spec, arm_q, mount, pick) if with_arm else None
    if pick: add_room_lights(spec)                          # switched on per episode by Sim.relight ([QMt]'s day and night)
    if floor_tex and not local_floor:                        # visual carpet patch (3 x 3 m, 5 mm/px texture) over the physics floor
                                                             # (not with the local floor: it and its surround cover the view; two would flicker)
        t = spec.add_texture(); t.name = "carpet"; t.type = mujoco.mjtTexture.mjTEXTURE_2D; t.file = str(floor_tex)
        mt = spec.add_material(); mt.name = "carpet"; mt.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "carpet"; mt.texrepeat = [1, 1]; mt.texuniform = False
        g = spec.worldbody.add_geom(); g.type = mujoco.mjtGeom.mjGEOM_PLANE; g.size = [floor_half, floor_half, 0.01]; g.pos = [0, 0, 0.0005]
        g.material = "carpet"; g.contype = 0; g.conaffinity = 0

    if lib:                                                  # item library ([QxY]): (name, uid, pos)
        import items
        for name, uid, pos in lib: items.add(spec, name, pos, uid)
    if local_floor:                                          # wrist-camera-resolution floor patch, regenerated per trial (floors.apply)
        import floors; floors.add_to_spec(spec)
    for o in objects:                                        # graspable litter: free bodies with mass and friction
        b_ = spec.worldbody.add_body(); b_.name = o["name"]; b_.pos = o["pos"]; b_.add_freejoint()
        g = b_.add_geom(); g.type = {"box": mujoco.mjtGeom.mjGEOM_BOX, "cylinder": mujoco.mjtGeom.mjGEOM_CYLINDER}[o.get("shape", "box")]
        g.size = o["half"] if o.get("shape", "box") == "box" else [o["half"][0], o["half"][2], 0]; g.mass = o["mass"]; g.rgba = o["rgba"]
        g.friction = [1.0, 0.02, 0.001]; g.condim = 4; g.contype = 1; g.conaffinity = 1
    if lib: spec.memory = 256 * 1024 * 1024                  # many mesh contacts: more constraint arena than the default
    if dress: dress(spec)                                    # a scene dressed around the robot (a set built for a render)
    m = spec.compile()
    if post: post(m)
    for o in objects:                                        # set by name: joints added after attach shift every later address
        a_ = m.jnt_qposadr[m.body(o["name"]).jntadr[0]]; m.key_qpos[:, a_:a_ + 7] = list(o["pos"]) + [1, 0, 0, 0]
    if lib:                                                  # attach leaves added joints zeroed in the keyframes (a zero quaternion is
        robot = m.body_rootid[m.body("trunk").id]            # invalid): every joint not on the robot starts at the model's own default
        nq_of = {mujoco.mjtJoint.mjJNT_FREE: 7, mujoco.mjtJoint.mjJNT_BALL: 4, mujoco.mjtJoint.mjJNT_SLIDE: 1, mujoco.mjtJoint.mjJNT_HINGE: 1}
        for j in range(m.njnt):
            if m.body_rootid[m.jnt_bodyid[j]] != robot:
                a_ = m.jnt_qposadr[j]; n_ = nq_of[m.jnt_type[j]]; m.key_qpos[:, a_:a_ + n_] = m.qpos0[a_:a_ + n_]
    m.opt.timestep = CTRL_DT / SUBSTEPS                        # physics at 1.25 ms (step() runs SUBSTEPS per control step)
    return m


class LegPolicy:
    """DimOS's Go1 ONNX walking policy, fed only the 12 leg joints (DimOS feeds qpos[7:], which would include the arm)."""
    def __init__(self, m):
        import onnxruntime as ort
        self.pol = ort.InferenceSession(str(DATA / "unitree_go1_policy.onnx"), providers=["CPUExecutionProvider"])
        self.default = np.array(m.keyframe("home").qpos[7:19]); self.last = np.zeros(12, np.float32); self.cmd = np.zeros(3, np.float32)
        self.imu = m.site("imu").id
    def act(self, m, d):
        g = d.site_xmat[self.imu].reshape(3, 3).T @ np.array([0, 0, -1])
        obs = np.hstack([d.sensor("local_linvel").data, d.sensor("gyro").data, g, d.qpos[7:19] - self.default, d.qvel[6:18], self.last, self.cmd]).astype(np.float32)
        a = self.pol.run(["continuous_actions"], {"obs": obs.reshape(1, -1)})[0][0]; self.last = a.copy()
        d.ctrl[:12] = a * 0.5 + self.default
