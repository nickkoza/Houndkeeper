"""[yyJ]: the house's outside walls are seen through from outside, as its ceiling from above, so the viewer can watch
the robot, while every sensor inside still sees solid walls: the lidar's depth cameras draw the
walls' boxes (group 4, which the viewer hides) and the cameras the planes facing in. Each light is one light: the soft
shadow copies filled the viewer's 8 lights, and lights dropped in and out as the robot moved."""
import os, re, shutil, subprocess, sys
from pathlib import Path

from houndkeeper import workspace
import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")
os.environ.setdefault("MUJOCO_GL", "egl")
HOUSE = Path(__file__).resolve().parents[1] / "sim" / "house"
DIMOS = workspace.dimos()
SEED = 9914                                                       # a seed no timeline uses; its files are removed after


@pytest.fixture(scope="module")
def house():
    out = [HOUSE / "runs" / f"nights_s{SEED}"] + [DIMOS / "data" / "mujoco_sim" / f"scene_nights_s{SEED}_n{n}.xml" for n in (1, 2)]
    try:
        subprocess.run([sys.executable, "make_nights.py", "--seed", str(SEED), "--nights", "1", "--per-night", "0", "--indoor",
                        "--regime", "night"], cwd=HOUSE, check=True, capture_output=True)
        yield out[1].read_text()
    finally:
        shutil.rmtree(out[0], ignore_errors=True)
        for f in out[1:]: f.unlink(missing_ok=True)
        for f in (DIMOS / "data" / "mujoco_sim" / "scene_office1" / "textures").glob(f"house_floor_s{SEED}_*"): f.unlink()


def _model(xml):
    sys.path.insert(0, str(DIMOS)); from dimos.simulation.mujoco.model import get_assets
    spec = mujoco.MjSpec.from_string(xml, assets=get_assets())
    # the long outside wall at the house's south edge, seen square on from 3 m outside and from 1 m inside, below the window sills
    hy = float(re.search(r'name="ceiling" type="plane" pos="0 0 [\d.]+" quat="0 1 0 0" size="[\d.]+ ([\d.]+)', xml).group(1)) - 0.2
    for name, y, fwd in (("outside", -hy - 3.0, 1.0), ("inside", -hy + 1.0, -1.0)):
        spec.worldbody.add_camera(name=name, pos=[2.0, y, 0.5], xyaxes=[fwd, 0, 0, 0, 0, 1], fovy=30)
    m = spec.compile(); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    return m, d, hy


def _south_faces_views(m, d, cam):
    """the colour view (as the viewer draws it) with and without the south wall's face planes"""
    r = mujoco.Renderer(m, 60, 80)
    r.update_scene(d, cam); a = r.render().astype(int)
    south = [g for g in range(m.ngeom) if (m.geom(g).name or "").endswith("_face") and m.geom_pos[g][1] < -3.0]
    rgba, mat = m.geom_rgba[south].copy(), m.geom_matid[south].copy()
    m.geom_rgba[south, 3] = 0; m.geom_matid[south] = -1
    r.update_scene(d, cam); b = r.render().astype(int)
    m.geom_rgba[south], m.geom_matid[south] = rgba, mat; r.close()
    return a, b


def _centre_depth(m, d, cam, groups):
    r = mujoco.Renderer(m, 60, 80); r.enable_depth_rendering()
    opt = mujoco.MjvOption()
    for g in range(6): opt.geomgroup[g] = int(g in groups)
    r.update_scene(d, cam, scene_option=opt); z = r.render(); r.close()
    return float(np.median(z[25:35, 35:45]))


def test_the_viewer_sees_in_through_an_outside_wall(house):
    m, d, hy = _model(house)
    a, b = _south_faces_views(m, d, "outside")         # the same view into the house (the faces dim MuJoCo's own
    ga, gb = a.mean(-1).ravel(), b.mean(-1).ravel()     # lighting of it a little), not a wall
    assert np.corrcoef(ga, gb)[0, 1] > 0.9


def test_from_inside_the_wall_is_solid_to_the_viewer_and_the_lidar(house):
    m, d, hy = _model(house)
    a, b = _south_faces_views(m, d, "inside")                                                 # the viewer: a wall
    assert (np.abs(a - b).max(-1) > 30).mean() > 0.9
    assert _centre_depth(m, d, "inside", groups=(0, 1, 2, 4)) == pytest.approx(0.65, abs=0.03)  # the lidar: the wall's box


def test_each_light_is_one_light(house):
    names = re.findall(r'<light name="([a-z_]+?)_\d+"', house)
    assert len(names) == len(set(names))
