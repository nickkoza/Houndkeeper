"""[4hm]: the sim's head camera is the real one as params/robot.yaml records it, one place for the sim and the
real robot. They had drifted: the sim's 45 deg against the real lens's ~52 deg (26 Sep 2026)."""
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
mujoco = pytest.importorskip("mujoco")
XML = '<mujoco><worldbody><body name="trunk"><camera name="head_camera" pos="0.29 0 0.01" xyaxes="0 -1 0 0 0 1"/></body></worldbody></mujoco>'


@pytest.fixture(autouse=True)
def _pinhole(monkeypatch):
    """these tests are of the pinhole path (HK_FISHEYE=0); the fisheye's are marked below"""
    monkeypatch.setenv("HK_FISHEYE", "0")


def _view(model):
    d = mujoco.MjData(model); mujoco.mj_forward(model, d)
    return -d.cam_xmat[0].reshape(3, 3)[:, 2]                     # a camera looks along its -z


def test_the_head_camera_has_the_real_lens(monkeypatch):
    from houndkeeper import params, sim_hook
    monkeypatch.setenv("HK_MOUNT_ERROR", "0")                     # the nominal placement (the error is tested below)
    monkeypatch.setenv("DIMOS_SIM_CAMERA_FOVY", str(params.get("go2_air.head_camera_fovy_deg")))
    monkeypatch.setenv("DIMOS_CAMERA_MOUNT", ",".join(str(v) for v in params.get("go2_air.head_camera_pos_m")))
    m = mujoco.MjModel.from_xml_string(XML); sim_hook.set_head_camera(m)
    assert m.cam_fovy[0] == pytest.approx(params.get("go2_air.head_camera_fovy_deg"))
    assert m.cam_pos[0] == pytest.approx(params.get("go2_air.head_camera_pos_m"))


def test_a_measured_pitch_points_it_down(monkeypatch):
    from houndkeeper import sim_hook
    monkeypatch.setattr(sim_hook, "_head_camera", lambda: {"fovy_deg": 51.8, "pitch_deg": 30.0, "pos_m": [0.29, 0.0, 0.01]})
    monkeypatch.setenv("DIMOS_SIM_CAMERA_FOVY", "51.8"); monkeypatch.setenv("DIMOS_CAMERA_MOUNT", "0.29,0.0,0.01")
    monkeypatch.setenv("HK_MOUNT_ERROR", "0")
    m = mujoco.MjModel.from_xml_string(XML); sim_hook.set_head_camera(m)
    v = _view(m)
    assert np.degrees(np.arcsin(-v[2])) == pytest.approx(30.0, abs=0.01) and v[0] > 0   # down, still facing forward


def test_the_camera_truly_sits_off_where_dimos_publishes_it(monkeypatch):
    """[55b]: the sim's camera carries the unmeasured miscalibration a real mount has; DimOS publishes the nominal"""
    from houndkeeper import params, sim_hook
    monkeypatch.setenv("DIMOS_SIM_CAMERA_FOVY", str(params.get("go2_air.head_camera_fovy_deg")))
    monkeypatch.setenv("DIMOS_CAMERA_MOUNT", ",".join(str(v) for v in params.get("go2_air.head_camera_pos_m")))
    e = params.get("go2_air.head_camera_mount_error")
    monkeypatch.setenv("HK_MOUNT_ERROR", "0"); m0 = mujoco.MjModel.from_xml_string(XML); sim_hook.set_head_camera(m0)
    monkeypatch.setenv("HK_MOUNT_ERROR", "1"); m1 = mujoco.MjModel.from_xml_string(XML); sim_hook.set_head_camera(m1)
    ang = np.degrees(np.arccos(np.clip(_view(m0) @ _view(m1), -1, 1)))
    assert ang == pytest.approx(np.hypot(e["pitch_deg"], e["yaw_deg"]), abs=0.05)
    assert m1.cam_pos[0] - m0.cam_pos[0] == pytest.approx(e["pos_m"])


def test_a_camera_rendered_unlike_its_published_lens_refuses_to_start(monkeypatch):
    """DimOS publishes the lens (camera_info) every floor pixel is placed with: 51.8 deg rendered against 45 published
    mis-projected every floor picture for an evening (26 Sep 2026)"""
    from houndkeeper import sim_hook
    monkeypatch.setenv("DIMOS_SIM_CAMERA_FOVY", "45")
    with pytest.raises(RuntimeError, match="publishes"):
        sim_hook.set_head_camera(mujoco.MjModel.from_xml_string(XML))


def test_a_camera_placed_unlike_its_published_mount_refuses_to_start(monkeypatch):
    """DimOS publishes where the camera sits (tf base_link -> camera): 1 cm low put far floor several cm out (26 Sep 2026)"""
    from houndkeeper import params, sim_hook
    monkeypatch.setenv("DIMOS_SIM_CAMERA_FOVY", str(params.get("go2_air.head_camera_fovy_deg")))
    monkeypatch.setenv("DIMOS_CAMERA_MOUNT", "0.3,0,0")
    with pytest.raises(RuntimeError, match="sits at"):
        sim_hook.set_head_camera(mujoco.MjModel.from_xml_string(XML))


def test_dimos_publishes_the_mount_it_is_given():
    """what DimOS actually publishes as base_link -> camera_link follows DIMOS_CAMERA_MOUNT: a second, hard-coded copy of
    DimOS's placeholder in the tf publisher kept the camera 1 cm low after the constant was fixed (26 Sep 2026)"""
    import os, subprocess, sys
    code = ("from dimos.robot.unitree.go2.connection import GO2Connection as G, PoseStamped, Quaternion, Vector3; "
            "t = G._odom_to_tf(PoseStamped(position=Vector3(0, 0, 0), orientation=Quaternion(0, 0, 0, 1), frame_id='world')); "
            "print([round(v, 4) for v in [x for x in t if x.child_frame_id == 'camera_link'][0].translation])")
    from houndkeeper import workspace
    dimos = workspace.dimos()
    out = subprocess.run([str(dimos / ".venv" / "bin" / "python"), "-c", code], capture_output=True, text=True, cwd=dimos,
                         env={**os.environ, "DIMOS_CAMERA_MOUNT": "0.29,0.0,0.01"}, timeout=120)
    assert out.stdout.strip().splitlines()[-1].replace(" ", "") == "[0.29,0.0,0.01]", out.stdout + out.stderr[-500:]


def test_with_the_fisheye_the_sim_renders_wide_and_publishes_the_real_calibration(monkeypatch):
    """[kJ6]: the head camera is the Go2's fisheye: rendered as a pinhole wide enough to cover it, warped to it, and DimOS
    publishes the calibration it was warped with"""
    from houndkeeper import lens, params, sim_hook
    monkeypatch.setenv("HK_FISHEYE", "1")
    monkeypatch.setenv("DIMOS_CAMERA_MOUNT", ",".join(str(v) for v in params.get("go2_air.head_camera_pos_m")))
    monkeypatch.delenv("DIMOS_SIM_CAMERA_INFO_YAML", raising=False)
    with pytest.raises(RuntimeError, match="publishes"):
        sim_hook.set_head_camera(mujoco.MjModel.from_xml_string(XML))
    monkeypatch.setenv("DIMOS_SIM_CAMERA_INFO_YAML", str(lens.CALIBRATION))
    m = mujoco.MjModel.from_xml_string(XML); sim_hook.set_head_camera(m)
    assert m.cam_fovy[0] == pytest.approx(lens.render_fovy(640, 360)) and m.cam_fovy[0] > 70


def test_a_fisheye_frame_straightened_matches_a_pinhole_at_its_focal_length(monkeypatch):
    """[kJ6]: lens.head straightens the sim's (and the robot's) fisheye frames into the pinhole every consumer uses"""
    monkeypatch.setenv("HK_FISHEYE", "1")
    import math
    import cv2
    from houndkeeper import lens
    fovy, W, H = lens.render_fovy(640, 360), 960, 540
    img = np.dstack([(30 + (np.indices((H, W)).sum(0) // 24) % 2 * 200).astype(np.uint8)] * 3)
    straight = lens.head(640, 360)(lens.to_fisheye(img, 640, 360, fovy))
    fx, fy, cx, cy = lens.head(640, 360).K; f = (H / 2) / math.tan(math.radians(fovy) / 2)
    u, v = np.meshgrid(np.arange(640.0), np.arange(360.0))
    direct = cv2.remap(img, ((u - cx) / fx * f + W / 2 - 0.5).astype(np.float32), ((v - cy) / fy * f + H / 2 - 0.5).astype(np.float32), cv2.INTER_LINEAR)
    assert (straight.sum(-1) > 0).mean() > 0.99
    assert np.abs(straight.astype(int) - direct.astype(int)).mean() < 4


def test_every_consumer_takes_the_head_lens_from_one_place():
    """the robot's kinematic copy carries MuJoCo's stock 45 deg head camera; the live code took its lens from there
    while frames were 51.8 deg, 13% out (27 Sep). Nothing but lens.py and the sim's own setup may read it"""
    import re
    src = (Path(__file__).resolve().parents[1] / "src" / "houndkeeper" / "tidy_module.py").read_text()
    assert not re.search(r"cam_fovy\[[^\]]*\]", src.replace('cam_fovy[cid])}', '')), "tidy_module reads a model's camera lens"


def test_at_the_real_frame_size_the_sim_renders_the_real_calibration_unscaled(monkeypatch):
    """[7cU]: the sim's frames are the real camera's 1280 x 720: the lens is DimOS's calibration as measured (no scaling),
    and the head camera's render covers the fisheye at that size"""
    from houndkeeper import lens, params, sim_hook
    import yaml
    assert params.get("go2_air.head_camera_px") == [1280, 720]
    c = yaml.safe_load(lens.CALIBRATION.read_text()); K, _ = lens.calibration(1280, 720)
    assert [c["image_width"], c["image_height"]] == [1280, 720]
    assert np.allclose(K.ravel(), c["camera_matrix"]["data"])
    monkeypatch.setenv("DIMOS_SIM_VIDEO_SIZE", "1280x720"); assert sim_hook.frame_px() == [1280, 720]
    monkeypatch.setenv("HK_FISHEYE", "1")
    monkeypatch.setenv("DIMOS_CAMERA_MOUNT", ",".join(str(v) for v in params.get("go2_air.head_camera_pos_m")))
    monkeypatch.setenv("DIMOS_SIM_CAMERA_INFO_YAML", str(lens.CALIBRATION))
    m = mujoco.MjModel.from_xml_string(XML); sim_hook.set_head_camera(m)
    assert m.cam_fovy[0] == pytest.approx(lens.render_fovy(1280, 720), abs=1e-6)


def test_env_renders_the_real_cameras_size_unless_overridden(monkeypatch):
    """[bFx], [7cU]: unset, the sim renders the real camera's 1280 x 720 and the render stamp carries the size (a
    baseline at another size is stale); HK_CAMERA_PX=WxH overrides it, and DimOS's 640 x 360 leaves the stamp as it was
    before [7cU], so an old 640 x 360 baseline stays comparable with an opt-in 640 x 360 night"""
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "dimos")); import env
    from houndkeeper import sim_hook
    monkeypatch.delenv("DIMOS_SIM_VIDEO_SIZE", raising=False)     # restored after, whatever apply() leaves
    for unset in (None, ""):
        if unset is None: monkeypatch.delenv("HK_CAMERA_PX", raising=False)
        else: monkeypatch.setenv("HK_CAMERA_PX", unset)
        assert "export DIMOS_SIM_VIDEO_SIZE=1280x720" in env.shell_exports().splitlines()
        assert sim_hook._head_camera()["frame_px"] == [1280, 720]
    monkeypatch.setenv("HK_CAMERA_PX", "real")
    assert "export DIMOS_SIM_VIDEO_SIZE=1280x720" in env.shell_exports().splitlines()
    monkeypatch.setenv("HK_CAMERA_PX", "640x360")
    assert "export DIMOS_SIM_VIDEO_SIZE=640x360" in env.shell_exports().splitlines()
    assert "frame_px" not in sim_hook._head_camera()
