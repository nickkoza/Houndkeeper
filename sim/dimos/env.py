"""Environment for running DimOS's own sim with the Maker Arm on the robot ([RJi]). One place for the variables both
the DimOS sim process and Houndkeeper's modules read."""
import os
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[2]
ARM_KEY = str(HK / "sim" / "arm" / "maker_arm.xml")        # sim-arm adapter address (the shared memory is keyed on it)
ARM_JOINTS = ["arm_link_002_joint", "arm_link_003_joint", "arm_link_004_joint", "arm_link_005_joint", "arm_link_006_joint", "arm_link_007_joint"]
ARM_GRIPPER = "arm_gripper_left_joint"
ARM_CAMERA = "arm_wrist"
# Our settings for DimOS's general sim and costmap options (its defaults are the stock sim's): every run sets them here
# ([UvE]; the fork carries no houndkeeper value, docs/dimos-fork.md rule 6)
SIM_OPTIONS = {
    "DIMOS_SIM_PERSON": "0",                       # no walking figure: the tidy robot works with nobody about
    "DIMOS_SIM_SENSOR_GEOMGROUPS": "0,1,2,4",      # the sensors see the outside walls' boxes, which make_nights puts in
                                                   # group 4 for the viewer to hide ([yyJ])
    "DIMOS_SIM_ARM_CAMERA_IDLE_S": "1.0",          # folded, a wrist frame a second of sim time ([hJ5])
    "DIMOS_SIM_ARM_CAMERA_REST_POSE": ",".join(["0"] * len(ARM_JOINTS)),   # the Maker Arm's factory fold: every joint at 0
    "DIMOS_SIM_ARM_CAMERA_REST_TOL": "0.15",       # rad on every joint, its sag included
    "DIMOS_SIM_POLICY_GPU": "0",                   # the walking policy on CPU: its CUDA workspaces took ~3 GB of the GPU
}
# DimOS's CostMapper (height_cost), set through its environment config source (COSTMAPPER__CONFIG__<FIELD>)
COSTMAP = {
    "can_pass_under": 3.0,   # the pass-under rule off: it sees only a cell's lowest and highest point, so it marked
                             # half-seen furniture free, and walls under a ceiling
    "ignore_above": 2.0,     # points above this dropped first: an indoor house's ceiling (2.4 m) neither blocks the floor
}                            # under it nor turns walls passable ([Xl1])


def costmap_env() -> dict:
    """the environment that sets COSTMAP in DimOS's CostMapper"""
    return {f"COSTMAPPER__CONFIG__{k.upper()}": str(v) for k, v in COSTMAP.items()}


def costmap_exports() -> str:
    """`export` lines for COSTMAP alone (for a script run without apply's exports)"""
    import shlex
    return "\n".join(f"export {k}={shlex.quote(v)}" for k, v in costmap_env().items())


def display_awake() -> bool:
    """the laptop's display is on and unlocked (X's DPMS monitor state, GNOME's screensaver); unknown counts as awake"""
    import subprocess
    env = dict(os.environ, DISPLAY=os.environ.get("DISPLAY", ":1"), XAUTHORITY=os.environ.get("XAUTHORITY", "/run/user/1000/gdm/Xauthority"))
    try:
        out = subprocess.run(["xset", "q"], capture_output=True, text=True, timeout=5, env=env).stdout
        if "Monitor is Off" in out or "Monitor is in Standby" in out or "Monitor is in Suspend" in out: return False
        ss = subprocess.run(["gdbus", "call", "--session", "--dest", "org.gnome.ScreenSaver", "--object-path", "/org/gnome/ScreenSaver",
                             "--method", "org.gnome.ScreenSaver.GetActive"], capture_output=True, text=True, timeout=5, env=env).stdout
        return "true" not in ss
    except Exception:
        return True


def apply(items_json=None):
    """set the variables; returns their names (what a shell script exports: `shell_exports`)"""
    before = dict(os.environ)
    os.environ.update(SIM_OPTIONS); os.environ.update(costmap_env())
    os.environ["DIMOS_SIM_MODEL_HOOK"] = "houndkeeper.sim_hook:add"
    os.environ["DIMOS_SIM_VIEWER_HOOK"] = "houndkeeper.sim_hook:viewer"   # our HUD on DimOS's viewer ([5AP])
    os.environ["DIMOS_SIM_ARM_KEY"] = ARM_KEY
    os.environ["DIMOS_SIM_ARM_JOINTS"] = ",".join(ARM_JOINTS)
    os.environ["DIMOS_SIM_ARM_GRIPPER"] = ARM_GRIPPER
    os.environ["DIMOS_SIM_ARM_CAMERA"] = ARM_CAMERA
    sys.path.insert(0, str(HK / "src")); from houndkeeper import params      # the head camera's lens, as the sim hook sets
    os.environ["DIMOS_SIM_CAMERA_FOVY"] = str(params.get("go2_air.head_camera_fovy_deg"))   # it: DimOS publishes it (camera_info)
    os.environ["DIMOS_CAMERA_MOUNT"] = ",".join(str(v) for v in params.get("go2_air.head_camera_pos_m"))   # and where it sits (tf)
    px = os.environ.get("HK_CAMERA_PX") or "real"       # the frame's size ([7cU], [bFx]): the real camera's by default (params
    w_, h_ = params.get("go2_air.head_camera_px")        # head_camera_px, 1280 x 720); HK_CAMERA_PX=WxH overrides it (640x360,
    os.environ["DIMOS_SIM_VIDEO_SIZE"] = f"{w_}x{h_}" if px == "real" else px   # DimOS's own size, for quick loops only)
    if os.environ.get("HK_ODOM_DRIFT", "1") == "1":     # the robot's pose estimate drifts, as a real one does ([x3f], [nAH] 2Ak)
        gain = float(os.environ.get("HK_ODOM_DRIFT_GAIN", "1"))   # x the drift's spreads: a demo in a small room ([nkM]) walks too
        os.environ["DIMOS_SIM_ODOM_DRIFT"] = ",".join(f"{k}={v * gain}" for k, v in params.get("go2_air.odom_drift").items())   # little to show it
    else: os.environ.pop("DIMOS_SIM_ODOM_DRIFT", None)
    os.environ["DIMOS_SIM_DT"] = str(params.get("sim.physics_dt_s"))   # the arm's flex is stable at 1.25 ms ([z7P]); the
                                                                        # policy still acts every 20 ms
    if os.environ.get("HK_LIDAR_MODEL", "1") == "1":    # the L1's point budget and range noise, not every depth pixel ([nAH] JaG)
        os.environ["DIMOS_SIM_LIDAR_MODEL"] = ",".join(f"{k}={v}" for k, v in {**params.get("go2_air.lidar"), **params.get("go2_air.lidar_dropouts")}.items())
    else: os.environ.pop("DIMOS_SIM_LIDAR_MODEL", None)
    if os.environ.get("HK_STREAM", "1") == "1":         # frames lost and stalled on the way ([nAH] UMH)
        os.environ["DIMOS_SIM_STREAM"] = ",".join(f"{k}={v}" for k, v in {**params.get("go2_air.video_stream"),   # a stall's
                                                  "burst_share": params.get("go2_air.stall_burst_share")}.items())    # frames in a late burst
    else: os.environ.pop("DIMOS_SIM_STREAM", None)
    if os.environ.get("HK_FRAME_DELAY", "1") == "1":    # frames stamped on arrival, late and jittered ([nAH] UMH)
        fd = params.get("go2_air.frame_delay_s"); os.environ["DIMOS_SIM_FRAME_DELAY"] = f"{fd['mean']},{fd['jitter']}"
    else: os.environ.pop("DIMOS_SIM_FRAME_DELAY", None)
    if os.environ.get("HK_ODOM_DELAY", "1") == "1":     # odometry arriving late and jittered, stamped on arrival ([nAH] UMH),
        od = params.get("go2_air.odom_delay_s"); os.environ["DIMOS_SIM_ODOM_DELAY"] = f"{od['mean']},{od['jitter']}"
        os.environ["DIMOS_SIM_LIDAR_DELAY"] = os.environ["DIMOS_SIM_ODOM_DELAY"]   # and the lidar over the same link ([3Pv])
    else:
        os.environ.pop("DIMOS_SIM_ODOM_DELAY", None); os.environ.pop("DIMOS_SIM_LIDAR_DELAY", None)
    if os.environ.get("HK_CMD_DELAY", "1") == "1":      # velocity commands reaching the robot late ([3Pv])
        cd = params.get("go2_air.cmd_delay_s"); os.environ["DIMOS_SIM_CMD_DELAY"] = f"{cd['mean']},{cd['jitter']}"
    else: os.environ.pop("DIMOS_SIM_CMD_DELAY", None)
    # the robot's clock follows sim time, published here for every DimOS process ([3Pv]: the sim the same at any speed):
    # one name per launch (a fresh shell's exports), kept if the caller set one
    os.environ.setdefault("DIMOS_SIM_CLOCK", f"houndkeeper_clock_{os.getpid()}")
    # the held item's mass for the arm's gravity feed-forward, from the pick (DimosRobot.set_payload) to the sim process
    # (sim_hook.viewer) ([qaS]): one file per launch, like the clock, so a launch never reads another's
    os.environ.setdefault("HK_SIM_PAYLOAD", f"/dev/shm/houndkeeper_payload_{os.getpid()}.json")
    if os.environ.get("HK_VIEWER") != "1" and not display_awake():   # nobody is watching: no viewer while
        os.environ["DIMOS_SIM_HEADLESS"] = "1"                        # the display sleeps; a viewer on a sleeping display also
                                                                      # stalled the sim (its frames block; 0.04x real time)
    if os.environ.get("HK_VIEWER") != "1":                    # DimOS's rerun viewer too: ~350 MB of the GPU a night needs for
        os.environ.setdefault("RERUN_OPEN", "none")           # its cameras (the renderer ran out of memory, 18 Oct, [U8i])
    os.environ.setdefault("MUJOCO_GL", "glfw"); os.environ.setdefault("DISPLAY", ":1")
    os.environ.setdefault("XAUTHORITY", "/run/user/1000/gdm/Xauthority")
    if os.environ.get("HK_RENDERER", "filament") == "filament":   # the robot's cameras through Filament (mujofil, built for
        os.environ["DIMOS_SIM_CAMERA_RENDERER"] = "houndkeeper.filament_cameras:make"   # this MuJoCo by tools/build_mujofil.sh);
        os.environ["MUJOCO_SHADOWS"] = "off"                  # the classic renderer draws only the viewer then: its 8192 px shadow
                                                              # maps for every light made each viewer frame so slow that the HUD's
                                                              # waits on it held the sim to 0.04x real time (27 Sep)
    else:                                                         # HK_RENDERER=classic keeps MuJoCo's classic renderer
        os.environ.pop("DIMOS_SIM_CAMERA_RENDERER", None)
        os.environ["HK_FISHEYE"] = "0"                            # (whose frames can't be warped to the fisheye)
        os.environ.pop("MUJOCO_SHADOWS", None)
    from houndkeeper import lens                                  # the head camera's real fisheye ([nAH] kJ6): DimOS publishes
    if lens.fisheye_on(): os.environ["DIMOS_SIM_CAMERA_INFO_YAML"] = str(lens.CALIBRATION)   # its calibration as camera_info
    else: os.environ.pop("DIMOS_SIM_CAMERA_INFO_YAML", None)
    if items_json is not None:
        os.environ["HK_SIM_ITEMS"] = items_json
    sys.path.insert(0, str(HK / "src"))
    return sorted(k for k in set(os.environ) | set(before) if os.environ.get(k) != before.get(k)
                  or k.startswith(("DIMOS_SIM_", "HK_SIM_", "COSTMAPPER__")))


def shell_exports(items_json=None) -> str:
    """`export NAME='value'` lines for every variable apply() sets (or `unset NAME` for one it clears), for scripts to
    eval: one list, so a new variable reaches every script"""
    import shlex
    lines = []
    for k in apply(items_json):
        lines.append(f"export {k}={shlex.quote(os.environ[k])}" if k in os.environ else f"unset {k}")
    return "\n".join(lines)
