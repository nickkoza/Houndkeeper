"""[Hsz]: a camera creeping 1 mm at a time sees a steady room: no bands flickering across far walls, ceiling and floor.
The "high" preset's SSAO banded on large surfaces seen across a long room (it looked like z-fighting:
7% of pixels jumped between frames 1 mm apart at the far end of the house) and the floor comparison reads flicker as change."""
import os, sys
from pathlib import Path

from houndkeeper import workspace
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "houndkeeper"))
mujoco = pytest.importorskip("mujoco"); pytest.importorskip("mujofil")
os.environ.setdefault("MUJOCO_GL", "egl")

DIMOS = workspace.dimos()
HOUSE = DIMOS / "data" / "mujoco_sim" / "scene_nights_s70_n1.xml"          # the night house, made by night_tidy.sh


def test_the_house_does_not_flicker_as_the_camera_creeps():
    """from the far end of house 70, looking the length of it (where the banding showed)"""
    if not HOUSE.exists(): pytest.skip("no house 70 scene (run sim/dimos/night_tidy.sh once)")
    sys.path.insert(0, str(DIMOS)); from dimos.simulation.mujoco.model import get_assets
    import filament_cameras as fc
    spec = mujoco.MjSpec.from_string(HOUSE.read_text(), assets=get_assets())
    spec.worldbody.add_camera(name="probe", pos=[7, 1.5, 0.35], xyaxes=[0, 1, 0, -0.1, 0, 1], fovy=70)
    m = spec.compile(); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "probe")

    def creep(toggles):
        """the share of pixels jumping > 24 levels frame to frame as the camera creeps 1 mm, at a held exposure; and
        whether the same view rendered twice matches"""
        saved = dict(fc.TOGGLES); fc.TOGGLES.update(toggles)
        try:
            cams = fc.FilamentCameras(m, d); cams.render(d, cid)
            r = next(iter(cams.rs.values()))["r"]; r.set_exposure(next(iter(cams.cams.values()))["ev"])
            m.cam_pos[cid, 0] = 7; mujoco.mj_forward(m, d)
            same = [cams._shot(r, d, cid).astype(np.int16) for _ in range(2)]
            frames = []
            for k in range(4):
                m.cam_pos[cid, 0] = 7 - 0.001 * k; mujoco.mj_forward(m, d)
                frames.append(cams._shot(r, d, cid).astype(np.int16))
        finally:
            fc.TOGGLES.clear(); fc.TOGGLES.update(saved)
        return (np.mean([(np.abs(frames[k + 1] - frames[k]).max(-1) > 24).mean() for k in range(3)]),
                (np.abs(same[1] - same[0]).max(-1) > 24).mean())
    # the effects must add no flicker to what the motion itself does: a 1 mm creep moves near carpet ~1 px and its
    # fibres sparkle for real (29 Sep: lit by five fixtures, that alone is ~0.17%); with SSAO off, what's left is only
    # that motion. The half-resolution SSAO of 26 Sep added ~0.11 points here (and 1.09% then, in the darker house)
    with_fx, same_fx = creep({})
    plain, _ = creep({"ssao": False})
    assert same_fx < 0.0001, "the same view rendered twice differs"
    assert with_fx - plain < 0.0005, f"the effects add {with_fx - plain:.2%} of pixels flickering ({with_fx:.2%} against {plain:.2%} without)"
