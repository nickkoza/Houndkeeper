"""Light adds up: with two lamps on, each half of the floor is lit by its own lamp, as with that lamp alone (waymark
0007 [3my]: Filament drew only the nearest lamp, so a house was lit by one lamp at a time and popped as the camera moved).
Runs on the mujofil build HK_MUJOFIL_DIR chooses, or the installed one."""
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "houndkeeper"))
mujoco = pytest.importorskip("mujoco")

XML = """<mujoco><worldbody>
<geom type="plane" size="10 10 0.1" rgba="0.6 0.6 0.6 1"/>
<light name="a" pos="-1.5 0 2" dir="0 0 -1" cutoff="60" diffuse="1 1 1"/>
<light name="b" pos="1.5 0 2" dir="0 0 -1" cutoff="60" diffuse="1 1 1"/>
<camera name="c" pos="-0.2 0 4" xyaxes="1 0 0 0 1 0" fovy="80"/>
</worldbody></mujoco>"""


def _halves(r, m, d, on):
    r.clear_dynamic_lights()
    for i in on: r.add_spot_light(*m.light_pos[i], *m.light_dir[i], 1, 1, 1, 1.5e6, 30.0, 30.0, 60.0, cast_shadows=False)
    r.sync_transforms(m, d); r.sync_camera(m, d, 0); img = np.asarray(r.render().cpu())[::-1, :, :3].astype(float)
    return img[:, :160].mean(), img[:, 160:].mean()


def test_two_lamps_each_light_their_own_half():
    import filament_cameras as fc
    os.environ.setdefault("MUJOFIL_BACKEND", "vulkan"); fc.use_chosen_mujofil()   # (MUJOFIL_BACKEND=gl tries the other backend)
    mujofil = pytest.importorskip("mujofil")
    m = mujoco.MjModel.from_xml_string(XML); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    r = mujofil.WarpRenderer(preset="high", auto_ibl=False, width=320, height=240, exposure=-4.0)
    want = "_warp" if os.environ["MUJOFIL_BACKEND"] == "vulkan" else "_warp_gl"   # mujofil falls back to OpenGL quietly
    assert mujofil._native.__name__.endswith(want), f"asked for {os.environ['MUJOFIL_BACKEND']}, got {mujofil._native.__name__}"
    act = m.light_active.copy(); m.light_active[:] = 0; r.load_model(m); m.light_active[:] = act
    a, b, both = _halves(r, m, d, [0]), _halves(r, m, d, [1]), _halves(r, m, d, [0, 1])
    assert a[0] > a[1] + 3 and b[1] > b[0] + 3                              # each lamp lights its own half
    assert both[0] >= a[0] - 1 and both[1] >= b[1] - 1, f"only one lamp drawn: {a}, {b}, both {both}"
