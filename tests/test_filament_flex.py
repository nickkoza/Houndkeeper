"""[Hsz]: Filament draws MuJoCo flex bodies (cloth, soft bodies) where MuJoCo's own renderer does, following them as
they move (mujofil drew geoms only: a flex cloth was invisible to the cameras, 26 Sep 2026)."""
import os, sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "houndkeeper"))
mujoco = pytest.importorskip("mujoco"); pytest.importorskip("mujofil")
os.environ.setdefault("MUJOCO_GL", "egl")

XML = """<mujoco><option timestep="0.002"/><worldbody>
<light name="ceiling_0" pos="0 0 3" dir="0 0 -1" diffuse="1 1 1" cutoff="80"/>
<geom type="plane" size="2 2 .1" rgba=".6 .6 .6 1"/>
<geom type="box" pos="0 0 0.1" size=".08 .08 .1" rgba=".3 .3 .9 1"/>
<flexcomp name="towel" type="grid" count="12 12 1" spacing=".025 .025 .025" pos="0 0 0.35" radius=".003" dim="2" rgba=".9 .1 .1 1">
  <edge equality="true"/><contact condim="3" selfcollide="none"/></flexcomp>
<camera name="top" pos="0 0 1.2" xyaxes="1 0 0 0 1 0" fovy="40"/></worldbody></mujoco>"""


def _red(im):
    """red whatever its brightness (each renderer exposes the scene its own way)"""
    f = im.astype(float) + 1; return (f[..., 0] > 12) & (f[..., 0] > 1.8 * f[..., 1]) & (f[..., 0] > 1.8 * f[..., 2])


def test_filament_draws_a_cloth_where_mujoco_does():
    import filament_cameras as fc
    m = mujoco.MjModel.from_xml_string(XML); d = mujoco.MjData(m)
    fc._BY_SIZE.clear(); r = mujoco.Renderer(m, 240, 320)
    for t in (0, 600):                                           # as built, then draped over the box
        for _ in range(t): mujoco.mj_step(m, d)
        mujoco.mj_forward(m, d)
        r.update_scene(d, "top"); c = _red(r.render()); f = _red(fc.render(m, d, "top", 320, 240))
        k = 0.7 if t == 0 else 0.4                               # draped: MuJoCo draws the cloth thickened by its radius
        assert c.sum() > 500 and f.sum() > k * c.sum(), (t, c.sum(), f.sum())   # (3 mm), Filament the bare surface
        assert (c & f).sum() > k * c.sum()                       # the same place
    top_z = d.flexvert_xpos[:, 2].max(); assert top_z < 0.3        # it did fall and drape


def test_a_cloth_lying_flat_on_the_floor_is_seen():
    """a flex rests its radius above what it touches, and MuJoCo draws it that thick: drawn as a bare surface through
    its vertices it would lie in the floor and flicker into it"""
    import filament_cameras as fc
    xml = XML.replace('<geom type="box" pos="0 0 0.1" size=".08 .08 .1" rgba=".3 .3 .9 1"/>', "").replace('pos="0 0 0.35"', 'pos="0 0 0.05"')
    m = mujoco.MjModel.from_xml_string(xml.replace('<option timestep="0.002"/>', '<option timestep="0.005"/>')); d = mujoco.MjData(m)
    for _ in range(400): mujoco.mj_step(m, d)
    mujoco.mj_forward(m, d); fc._BY_SIZE.clear()
    r = mujoco.Renderer(m, 240, 320); r.update_scene(d, "top"); c = _red(r.render()); f = _red(fc.render(m, d, "top", 320, 240))
    assert c.sum() > 2000 and f.sum() > 0.85 * c.sum(), (c.sum(), f.sum())
