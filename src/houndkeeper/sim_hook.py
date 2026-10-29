"""DimOS sim model hook: puts the Maker Arm (and optionally library items) on DimOS's simulated robot.

DimOS calls a hook named by DIMOS_SIM_MODEL_HOOK ("module:function") with the MjSpec of its robot scene before
compiling, and calls whatever it returns with the compiled model. Set DIMOS_SIM_MODEL_HOOK=houndkeeper.sim_hook:add.

HK_SIM_ITEMS: optional JSON list of [item name, uid, [x, y, z]] placed as library items ([QxY]).
HK_SIM_LOCAL_FLOOR=1: add the wrist-resolution floor patch (sim/arm/floors.py).

DIMOS_SIM_VIEWER_HOOK=houndkeeper.sim_hook:viewer draws our HUD on DimOS's viewer ([5AP]): the tidy loop's state and
latest step (from HUD_FILE, written by the tidy module in its own process), battery and power, as in the standalone
slice, and the robot's head and wrist camera frames bottom-left, the very frames it published (so the insets show
what its perception gets, whichever renderer made them; HK_SIM_CAMERA_INSETS=0 hides them).
HK_SIM_SHADOWSIZE (default 8192): the shadow map size. MuJoCo gives every light one map of this size; the lamp's
cone keeps its shadows crisp, but the sun spreads its map over the whole house (DimOS's scenes leave MuJoCo's 4096)."""
import json
import os
import sys
from pathlib import Path

ARM_SIM = Path(__file__).resolve().parents[2] / "sim" / "arm"
# The adapter address DimOS's sim-arm shared memory is keyed on (an MJCF path; the sim-arm adapter uses it as a name).
ARM_KEY = str(ARM_SIM / "maker_arm.xml")
from houndkeeper.hud_state import HUD_FILE   # noqa: E402  the tidy module's state for the HUD (another process)
ENERGY_FILE = "/dev/shm/houndkeeper_energy.json"   # the battery model's energy per pack, for the night's report ([bOu])
_HUD: dict = {}
INSET_SCALE, INSET_HZ = 2, 10.0              # insets at half the cameras' size, refreshed 10 times a second


def write_energy(h, t: float, path: str = ENERGY_FILE) -> None:
    """the HUD's battery model, per pack, for the run report (sim-slice [bOu]; it was on the HUD only): written whole
    and swapped in, so a reader never meets half a file"""
    left = round(100 * (1 - h.used_air / h.air_Wh), 2)
    e = {"t": round(float(t), 2), "air_used_Wh": round(h.used_air, 3), "air_Wh": h.air_Wh, "air_left_pct": left,
         "arm_used_Wh": round(h.used_arm, 3), "arm_Wh": h.arm_Wh}
    with open(path + ".tmp", "w") as fh: json.dump(e, fh)
    os.replace(path + ".tmp", path)


def monitor_on(xset_q: str) -> bool:
    """from `xset q`'s output: False only when it says the monitor is off, in standby or suspended"""
    for line in xset_q.splitlines():
        if line.strip().startswith("Monitor is"): return line.strip() == "Monitor is On"
    return True


_DISPLAY = {"t": -1e9, "awake": True}


def display_awake(every_s: float = 10.0) -> bool:
    """is the screen on? `xset q`, asked at most every 10 s (a few ms each time); True if it can't be told"""
    import subprocess
    import time
    now = time.time()
    if now - _DISPLAY["t"] >= every_s:
        _DISPLAY["t"] = now
        try: _DISPLAY["awake"] = monitor_on(subprocess.run(["xset", "q"], capture_output=True, text=True, timeout=1.0).stdout)
        except Exception: _DISPLAY["awake"] = True
    return _DISPLAY["awake"]


def draw_overlays(v, draw) -> bool:
    """draw(v) puts the HUD and camera insets on the viewer, unless the screen is asleep ([5AP]): each call waits on
    the viewer's lock, and with the monitor off its render thread holds it through a stalled swap (28 Sep: the loop's
    time in the hook went from 4% to most of it, the sim from 0.47x to under 0.37x)"""
    if not hasattr(v, "set_texts") or not display_awake(): return False   # headless: no window to draw on ([bnO])
    draw(v); return True


def viewer(model, data, v, frames=None):
    """each sim frame: the arm's gravity feed-forward for its command and the held item's mass, from the pick's
    per-launch payload file ([cqG], [qaS]; DimOS steps the physics, so once a frame here); advance the battery and
    power model; four times a second, redraw the HUD"""
    import time
    import numpy as np
    _arm_path()
    import go1_arm
    go1_arm.payload_from_file(model)
    go1_arm.update_gravity_ff(model, data)
    if not _HUD:
        import go1_arm
        import hud as hudm

        class _View:                                   # what the HUD reads from the standalone Sim
            pass
        s = _View(); s.m, s.d = model, data; s.tr = model.body("trunk").id
        s.ad = [model.jnt_dofadr[model.joint("arm_" + j).id] for j in go1_arm.ARM_JOINTS[:6]]
        s.pol = _View(); s.pol.cmd = np.zeros(3)
        _HUD.update(h=hudm.Hud(), s=s, t=data.time, wall=0.0)
        import mujoco                                  # third person: the viewer follows the robot (the standalone sims do)
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING; v.cam.trackbodyid = s.tr
        v.cam.distance = 2.5; v.cam.elevation = -25.0; v.cam.azimuth = 150.0
    h, s = _HUD["h"], _HUD["s"]
    if frames is not None and os.environ.get("HK_SIM_CAMERA_INSETS", "1") != "0":
        draw_overlays(v, lambda v_: _camera_insets(v_, frames, time.time()))
    s.t = data.time; s.pol.cmd = data.qvel[0:3].copy() if np.linalg.norm(data.qvel[0:2]) > 0.05 else np.zeros(3)   # walking
    h.tick(s, max(data.time - _HUD["t"], 0.0)); _HUD["t"] = data.time
    now = time.time()
    if now - _HUD["wall"] < 0.25: return
    _HUD["wall"] = now
    try: write_energy(h, data.time, ENERGY_FILE)       # read at each call: a test or a second sim can point it elsewhere
    except Exception: pass
    try:
        st = json.load(open(HUD_FILE)); h.state = st.get("state", h.state); h.note = st.get("note", "")
    except Exception:
        pass
    def hud(v_):
        vp = v_.viewport; W, H = (vp.width, vp.height) if vp is not None else (1280, 720)
        v_.set_figures(h.figures(W, H)); v_.set_texts(h.texts())
    draw_overlays(v, hud)


def _camera_insets(v, frames, now):
    """the head and wrist cameras' latest published frames, side by side in the viewer's bottom-left corner (the power
    chart is top-right, the battery bottom-right), labelled; nothing re-rendered, so it is exactly what the robot saw"""
    import mujoco
    import numpy as np
    if now - _HUD.get("cam_t", 0.0) < 1.0 / INSET_HZ: return
    from PIL import Image, ImageDraw
    _HUD["cam_t"] = now; x = 10; images = []
    for name in ("head", "wrist"):
        img = frames.get(name)
        if img is None: continue
        pic = Image.fromarray(np.ascontiguousarray(img[::INSET_SCALE, ::INSET_SCALE, :3]))
        ImageDraw.Draw(pic).text((6, 4), f"{name} camera", fill=(255, 230, 0))   # the viewer's one text overlay is the HUD's
        small = np.asarray(pic)
        images.append((mujoco.MjrRect(x, 10, small.shape[1], small.shape[0]), small))
        x += small.shape[1] + 6
    if images: v.set_images(images)


RENDER_VERSION = 4   # bump when anything changes how DimOS's sim renders a scene (lights, shadows, extent, textures)



def _arm_path():
    """the arm sim's modules importable, the path entry added once: viewer() runs every frame, and inserting there grew
    sys.path by one entry a frame for the whole night (3 Oct 2026)"""
    if str(ARM_SIM) not in sys.path: sys.path.insert(0, str(ARM_SIM))

def _near_clip_m():
    _arm_path(); import go1_arm
    return go1_arm.NEAR_CLIP_M


def render_settings() -> dict:
    """what a night's pictures were rendered with. The floor comparison needs tonight and the baseline nights rendered
    alike: on 25 Sep a sharper shadow map (8192 against the baselines' 4096) turned every shadow edge into a find."""
    cams = os.environ.get("DIMOS_SIM_CAMERA_RENDERER", "classic")
    if cams != "classic":
        import importlib
        mod = importlib.import_module(cams.partition(":")[0]); cams += f"@{getattr(mod, 'CALIBRATION', 0)}"
    shadows = 0 if os.environ.get("MUJOCO_SHADOWS") == "off" else int(os.environ.get("HK_SIM_SHADOWSIZE", "8192"))   # classic's
    out = {"version": RENDER_VERSION, "shadowsize": shadows, "near_clip_m": _near_clip_m(),
           "cameras": cams, "head_camera": _head_camera(), "imperfections": _imperfections()}
    from houndkeeper import filament_cameras as _fc          # the mujofil build, always (waymark 0007 [3my]): the default is
    if _fc.mujofil_dir(): out["mujofil"] = os.path.basename(_fc.mujofil_dir().rstrip("/"))   # the fixed build now
    return out


def _imperfections():
    """the world's imperfections the sim was run with ([nAH]): a baseline recorded without them isn't comparable"""
    from houndkeeper import params
    return {"arm": {k: params.get(f"maker_arm.{k}") for k in ("gravity_ff_fraction", "frame_flex_Nm_per_rad", "backlash_deg")}
            if os.environ.get("HK_ARM_IMPERFECT", "flex,lash") != "none" else None,
            "camera": params.get("go2_air.camera_imperfections") if os.environ.get("HK_CAMERA_IMPERFECT", "1") == "1" else None,
            "odom": os.environ.get("DIMOS_SIM_ODOM_DRIFT") or None,
            "mount": params.get("go2_air.head_camera_mount_error") if os.environ.get("HK_MOUNT_ERROR", "1") == "1" else None,
            "frame_delay": os.environ.get("DIMOS_SIM_FRAME_DELAY") or None, "cmd_delay": os.environ.get("DIMOS_SIM_CMD_DELAY") or None,
            "lidar_delay": os.environ.get("DIMOS_SIM_LIDAR_DELAY") or None, "sim_clock": bool(os.environ.get("DIMOS_SIM_CLOCK")),
            "lidar": os.environ.get("DIMOS_SIM_LIDAR_MODEL") or None, "stream": os.environ.get("DIMOS_SIM_STREAM") or None,
            "physics_dt": os.environ.get("DIMOS_SIM_DT") or None}


def _head_camera():
    """the head camera's lens and aim, from params/robot.yaml (one place: the sim and the real robot both follow
    it; they had drifted, the sim's 45 deg against the real lens's ~52, 26 Sep)"""
    from houndkeeper import params
    return {"fovy_deg": params.get("go2_air.head_camera_fovy_deg"), "pitch_deg": params.get("go2_air.head_camera_pitch_deg"),
            "pos_m": params.get("go2_air.head_camera_pos_m"), **({"frame_px": frame_px()} if frame_px() != [640, 360] else {})}
                                                                  # (DimOS's default size leaves the stamp as it was before [7cU])


def frame_px():
    """the frame size the sim renders and publishes (DIMOS_SIM_VIDEO_SIZE, set by sim/dimos/env.py from params
    head_camera_px; DimOS's own 640 x 360 when unset): a baseline rendered at another size isn't comparable"""
    w, _, h = (os.environ.get("DIMOS_SIM_VIDEO_SIZE") or "640x360").lower().partition("x")
    return [int(w), int(h)]


def set_head_camera(model):
    """DimOS's head_camera made the real one: its vertical field of view, pitched down from level by the given angle"""
    import mujoco
    import numpy as np
    cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "head_camera")
    if cid < 0: return
    from houndkeeper import lens
    hc = _head_camera(); model.cam_fovy[cid] = hc["fovy_deg"]
    if lens.fisheye_on():                                         # rendered as a wide pinhole, then warped to the real fisheye
        model.cam_fovy[cid] = lens.render_fovy(*frame_px())       # (filament_cameras): DimOS publishes the lens's calibration
        pub = os.environ.get("DIMOS_SIM_CAMERA_INFO_YAML")
        if pub != str(lens.CALIBRATION):
            raise RuntimeError(f"head camera renders the fisheye of {lens.CALIBRATION} but DimOS publishes {pub}: set "
                               "DIMOS_SIM_CAMERA_INFO_YAML (sim/dimos/env.py)")
    model.cam_pos[cid] = hc["pos_m"]                              # where it sits on the trunk, as DimOS publishes it
    mount = os.environ.get("DIMOS_CAMERA_MOUNT")                  # (base_link -> camera): every floor pixel is placed with
    if mount is None or np.abs(np.array([float(v) for v in mount.split(",")]) - hc["pos_m"]).max() > 1e-6:   # it (26 Sep:
        raise RuntimeError(f"head camera sits at {hc['pos_m']} but DimOS publishes {mount}: set DIMOS_CAMERA_MOUNT "   # 1 cm low)
                           "(sim/dimos/env.py) from params/robot.yaml")
    pub = os.environ.get("DIMOS_SIM_CAMERA_FOVY")                 # what DimOS publishes as the camera's lens (camera_info):
    if not lens.fisheye_on() and (pub is None or abs(float(pub) - hc["fovy_deg"]) > 1e-6):    # every floor pixel is placed with it. They drifted once
        raise RuntimeError(f"head camera renders at {hc['fovy_deg']} deg but DimOS publishes {pub}: set DIMOS_SIM_CAMERA_FOVY "   # (26 Sep:
                           "(sim/dimos/env.py) from params/robot.yaml")   # 51.8 rendered, 45 published: every map since was mis-projected)
    if hc["pitch_deg"]:                                   # about the camera's own x (its right): look down
        q = np.zeros(4); mujoco.mju_axisAngle2Quat(q, np.array([1.0, 0, 0]), -np.radians(hc["pitch_deg"]))
        out = np.zeros(4); mujoco.mju_mulQuat(out, model.cam_quat[cid].copy(), q); model.cam_quat[cid] = out
    if os.environ.get("HK_MOUNT_ERROR", "1") == "1":      # where it truly sits differs from what DimOS publishes by the
        from houndkeeper import params                    # unmeasured miscalibration ([nAH] 55b): fixed, as one robot's is
        e = params.get("go2_air.head_camera_mount_error")
        model.cam_pos[cid] = model.cam_pos[cid] + np.array(e["pos_m"])
        for axis, deg in (([1.0, 0, 0], -e["pitch_deg"]), ([0, 1.0, 0], e["yaw_deg"]), ([0, 0, 1.0], e["roll_deg"])):   # camera
            q = np.zeros(4); mujoco.mju_axisAngle2Quat(q, np.array(axis), np.radians(deg))   # frame: x right, y up, z back
            out = np.zeros(4); mujoco.mju_mulQuat(out, model.cam_quat[cid].copy(), q); model.cam_quat[cid] = out


def _fixed_extent(model):
    """Size the model from the house's fixed parts only. MuJoCo fits the sun's shadow map to the model's extent, so
    anything placed on a night (items, the arm) shifted every blocky shadow edge, and the floor comparison saw the
    edges as changes: a false find at the foot of a wall on 25 Sep (a 3% larger extent moved 655 px of edge in one
    view). Every night of the same house now casts identical shadows."""
    import mujoco
    import numpy as np
    d = mujoco.MjData(model); mujoco.mj_kinematics(model, d)
    fixed = [g for g in range(model.ngeom) if model.body_weldid[model.geom_bodyid[g]] == 0
             and model.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE]
    if not fixed: return
    lo = np.min([d.geom_xpos[g] - model.geom_rbound[g] for g in fixed], axis=0)
    hi = np.max([d.geom_xpos[g] + model.geom_rbound[g] for g in fixed], axis=0)
    model.stat.center[:] = (lo + hi) / 2; model.stat.extent = float(np.max(hi - lo))


def _fixtures(spec) -> list:
    """tonight's fixtures from the night's truth file (make_nights.py), by the scene's name: scene_nights_s70_n3"""
    name = (spec.modelname or "").removeprefix("scene_")
    if not name.startswith("nights_s"): return []
    truth = ARM_SIM.parent / "house" / "runs" / name.rsplit("_n", 1)[0] / f"{name}.truth.json"
    return json.loads(truth.read_text()).get("fixtures") or [] if truth.exists() else []


def add(spec):
    _arm_path()
    import go1_arm
    import payload_test

    post_arm = go1_arm.add_arm(spec, payload_test.STOW, pick=True)
    fixtures = _fixtures(spec)                                  # the house's own floor things tonight: a pet's bowls ([nAH] TJu)
    if fixtures:
        import items
        for fx in fixtures: items.add(spec, fx["name"], [fx["x"], fx["y"], 0.04], fx["uid"])
    lib = json.loads(os.environ.get("HK_SIM_ITEMS", "[]"))
    if lib:
        import items
        for name, uid, pos in lib:
            items.add(spec, name, pos, uid)
        spec.memory = 256 * 1024 * 1024
    if os.environ.get("HK_SIM_LOCAL_FLOOR") == "1":
        import floors
        floors.add_to_spec(spec)

    def post(model):
        import mujoco
        post_arm(model)
        if os.environ.get("HK_RENDER_EXPORT"):             # the Filament render server builds the same scene from this
            try: spec.to_zip(os.environ["HK_RENDER_EXPORT"])   # (it runs MuJoCo 3.9 and can't share this process's model)
            except Exception as e: print(f"[sim_hook] scene export for the render server failed: {e!r}", flush=True)
        model.vis.quality.shadowsize = render_settings()["shadowsize"]   # before any render context exists
        _fixed_extent(model); go1_arm.near_clip(model)          # after the extent: the clip is relative to it
        set_head_camera(model)
        if lib:                                        # items weigh what the real thing would (est. ranges), as in every
            import numpy as np                         # other sim, not the scans' solid density (a 3.4 kg ballet flat)
            rng = np.random.default_rng(int(os.environ.get("HK_SIM_ITEM_SEED", "0")))
            for name, uid, pos in lib:
                items.set_mass(model, uid, rng, name=name)
        if fixtures:                                   # the bowls weigh what real ones do (est. ranges)
            import numpy as np
            rng_f = np.random.default_rng(len(fixtures))
            for fx in fixtures: items.set_mass(model, fx["uid"], rng_f, name=fx["name"])
        if os.environ.get("HK_SIM_PYRAMIDAL") != "1":  # held items crept out of the jaws with the pyramidal friction cone
            model.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC   # (grasp spec [Udt]); HK_SIM_PYRAMIDAL=1 restores DimOS's default
        robot = model.body_rootid[model.body("trunk").id]
        width = {mujoco.mjtJoint.mjJNT_FREE: 7, mujoco.mjtJoint.mjJNT_BALL: 4}
        for j in range(model.njnt):           # added joints start at the model's own default, never a zero quaternion
            if model.body_rootid[model.jnt_bodyid[j]] != robot:
                a = model.jnt_qposadr[j]
                n = width.get(model.jnt_type[j], 1)
                model.key_qpos[:, a:a + n] = model.qpos0[a:a + n]
    return post
