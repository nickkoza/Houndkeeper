"""[Hsz]: a Filament frame is the classic renderer's frame, the same way up (mujofil hands frames back bottom row first,
checked 25 Sep 2026, mujofil 0.2.6 @ 339a0fa: every Filament frame until then was upside down)."""
import os, sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "houndkeeper"))
mujoco = pytest.importorskip("mujoco"); pytest.importorskip("mujofil")
os.environ.setdefault("MUJOCO_GL", "egl")

XML = """<mujoco><worldbody>
<light name="ceiling_0" pos="0 0 3" dir="0 0 -1" diffuse="1 1 1" cutoff="80"/>
<geom type="plane" size="3 3 .1" rgba=".5 .5 .5 1"/>
<geom type="box" pos="-0.3 0.2 0.05" size=".08 .08 .05" rgba="1 0 0 1"/>
<geom type="box" pos="0.3 -0.2 0.05" size=".08 .08 .05" rgba="0 1 0 1"/>
<camera name="top" pos="0 0 1.5" xyaxes="1 0 0 0 1 0"/>
</worldbody></mujoco>"""


def _centre(im, ch):
    other = [c for c in range(3) if c != ch]
    ys, xs = np.nonzero(im[..., ch].astype(int) - im[..., other].max(-1).astype(int) > 60)
    return np.array([xs.mean(), ys.mean()])


def test_filament_frame_is_upright():
    import filament_cameras as fc
    m = mujoco.MjModel.from_xml_string(XML); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 240, 320); r.update_scene(d, "top"); classic = r.render()
    fil = fc.render(m, d, "top", 320, 240)
    for ch in (0, 1):                                            # the red box up-left, the green one down-right, in both
        assert np.abs(_centre(fil, ch) - _centre(classic, ch)).max() < 6


# [Hsz]: a textured floor is where MuJoCo draws it: the same scale, the same way round, and only as large as the geom
# (mujofil 0.2.6 scaled plane textures by a fixed 0.13 per metre and drew every plane 20 m square, 25 Sep 2026)
QUAD = np.zeros((64, 64, 3), np.uint8)
QUAD[:32, :32] = (220, 30, 30); QUAD[:32, 32:] = (30, 200, 30); QUAD[32:, :32] = (30, 30, 220); QUAD[32:, 32:] = (220, 200, 30)


def _plane_scene(uniform):
    rep = "2 2" if uniform else "1 1"
    return f"""<mujoco><asset><texture name="q" type="2d" width="64" height="64" builtin="flat" rgb1=".5 .5 .5"/>
<material name="q" texture="q" texrepeat="{rep}" texuniform="{str(uniform).lower()}"/></asset><worldbody>
<light name="ceiling_0" pos="0 0 3" dir="0 0 -1" diffuse="1 1 1" cutoff="80"/>
<geom type="plane" pos="0.1 0.05 0" size="0.4 0.25 .1" material="q"/>
<camera name="top" pos="0.1 0.05 1.2" xyaxes="1 0 0 0 1 0" fovy="45"/></worldbody></mujoco>"""


def _classes(im):
    """per pixel: 0 red, 1 green, 2 blue, 3 yellow, -1 none (grey, dark or unsaturated)"""
    f = im.astype(int); r, g, b = f[..., 0], f[..., 1], f[..., 2]
    c = np.full(r.shape, -1)
    c[(r > g + 60) & (r > b + 60)] = 0; c[(g > r + 60) & (g > b + 60)] = 1; c[(b > r + 60) & (b > g + 60)] = 2
    c[(r > b + 60) & (g > b + 60) & (np.abs(r - g) < 60)] = 3
    return c


@pytest.fixture
def fixed_exposure(monkeypatch):
    """these check where the pattern lands, not exposure: a small plane on black would auto-expose to white"""
    import filament_cameras as fc
    monkeypatch.setattr(fc, "EV_RANGE", (5.0, 5.0))


@pytest.mark.parametrize("uniform", [False, True])
def test_filament_plane_texture_matches_classic(uniform, fixed_exposure):
    import filament_cameras as fc
    m = mujoco.MjModel.from_xml_string(_plane_scene(uniform)); d = mujoco.MjData(m)
    a = m.tex_adr[0]; m.tex_data[a:a + QUAD.size] = QUAD.reshape(-1); mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 240, 320); r.update_scene(d, "top"); classic = _classes(r.render())
    fc._BY_SIZE.clear(); fil = _classes(fc.render(m, d, "top", 320, 240))
    seen = classic >= 0
    assert seen.mean() > 0.3                                     # the test sees the pattern at all
    assert (fil[seen] == classic[seen]).mean() > 0.9             # same colours in the same places
    assert (fil[~seen] >= 0).mean() < 0.05                       # and no pattern beyond the plane's edges


def test_filament_follows_a_texture_change(fixed_exposure):
    import filament_cameras as fc
    m = mujoco.MjModel.from_xml_string(_plane_scene(False)); d = mujoco.MjData(m)
    a = m.tex_adr[0]; m.tex_data[a:a + QUAD.size] = QUAD.reshape(-1); mujoco.mj_forward(m, d)
    fc._BY_SIZE.clear(); fc.render(m, d, "top", 320, 240)
    m.tex_data[a:a + QUAD.size] = np.tile(np.array([30, 30, 220], np.uint8), 64 * 64)   # the floor becomes all blue
    fc.texture_changed(m, 0)
    fil = _classes(fc.render(m, d, "top", 320, 240))
    assert (fil == 2).sum() > 0.9 * (fil >= 0).sum()
