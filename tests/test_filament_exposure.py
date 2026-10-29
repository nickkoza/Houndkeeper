"""[Hsz]: the cameras auto-expose as a real camera does: a dim room and a bright one both settle near mid-grey within a
few frames (a fixed exposure left 53% of a night's frames black, 26 Sep 2026)."""
import os, sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "houndkeeper"))
mujoco = pytest.importorskip("mujoco"); pytest.importorskip("mujofil")
os.environ.setdefault("MUJOCO_GL", "egl")


def _room(diffuse):
    return f"""<mujoco><worldbody>
<light name="ceiling_0" pos="0 0 2.4" dir="0 0 -1" diffuse="{diffuse} {diffuse} {diffuse}" cutoff="80"/>
<geom type="plane" size="3 3 .1" rgba=".55 .5 .45 1"/>
<geom type="box" pos="1.5 0 1" size=".05 3 1" rgba=".8 .8 .78 1"/>
<camera name="head" pos="-1 0 0.35" xyaxes="0 -1 0 0.3 0 1" fovy="65"/></worldbody></mujoco>"""


@pytest.mark.parametrize("diffuse", [0.02, 1.0])
def test_dim_and_bright_rooms_settle_near_mid_grey(diffuse):
    import filament_cameras as fc
    m = mujoco.MjModel.from_xml_string(_room(diffuse)); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    fc._BY_SIZE.clear()
    for _ in range(4): im = fc.render(m, d, "head", 160, 90)
    mean = im.mean() / 255
    assert fc.TARGET / 2 < mean < fc.TARGET * 2, mean


def test_exposure_follows_the_camera_into_a_darker_place():
    """metered once, a camera walking out of the light goes black; it has to keep adjusting"""
    import filament_cameras as fc
    m = mujoco.MjModel.from_xml_string(_room(1.0)); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    fc._BY_SIZE.clear()
    for _ in range(4): fc.render(m, d, "head", 160, 90)
    m.light_diffuse[:] *= 0.01; fc.reset(m)                    # the same camera, now somewhere 100x darker
    for _ in range(60): im = fc.render(m, d, "head", 160, 90)   # 3 s at 20 frames/s: 6.6 stops, eased as a camera does
    assert im.mean() / 255 > fc.TARGET / 2, im.mean()


def test_cameras_share_a_renderer_but_expose_on_their_own():
    """one copy of the scene's textures on the GPU (a copy per camera ran a night out of memory, 26 Sep), and a camera
    facing the lit wall and one facing the dark still each settle near mid-grey"""
    import filament_cameras as fc
    xml = _room(1.0).replace('<camera name="head"', '<camera name="dark" pos="-2.5 0 0.35" xyaxes="0 1 0 -0.3 0 1" fovy="65"/>\n<camera name="head"')
    m = mujoco.MjModel.from_xml_string(xml.replace('cutoff="80"', 'cutoff="25"')); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    fc._BY_SIZE.clear()
    for _ in range(5):
        lit = fc.render(m, d, "head", 160, 90); dark = fc.render(m, d, "dark", 160, 90)
    cams = next(iter(fc._BY_SIZE.values()))
    assert len(cams.rs) == 1
    evs = [c["ev"] for c in cams.cams.values()]
    assert abs(evs[0] - evs[1]) > 0.5                          # they did expose differently
    for im in (lit, dark): assert fc.TARGET / 2 < im.mean() / 255 < fc.TARGET * 2


def test_a_dark_room_does_not_blow_its_pale_patch_to_white():
    """[Hsz], 30 Sep: metering stops short of clipping the frame's brightest few percent (a dark slate floor metered to
    mid-grey blew a beige carpet and a pale table to white)"""
    import filament_cameras as fc
    xml = (_room(1.0).replace('rgba=".55 .5 .45 1"/>', 'rgba=".04 .035 .03 1"/>\n<geom type="box" pos="0.3 0 0.005" size=".12 .15 .005" rgba=".9 .85 .75 1"/>')
           .replace('pos="0 0 2.4" dir="0 0 -1"', 'pos="0.3 0 1.0" dir="0 0 -1"').replace('cutoff="80"', 'cutoff="25"')
           .replace('rgba=".8 .8 .78 1"', 'rgba=".05 .05 .05 1"'))                  # a pale patch under a tight lamp, the rest dark
    m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    fc._BY_SIZE.clear()
    for _ in range(6): im = fc.render(m, d, "head", 160, 90)
    assert (im.max(-1) >= 250).mean() < 0.01                    # under 1% of the frame at white
