"""Cabinets read as cabinets (sim-house-nights [WTj]): door fronts proud of the carcass with gaps, a handle on each, a
drawer row on the counter, plain panels on the island's seating side, texture coordinates at real scale on every face,
and a finish drawn per house without disturbing the house's other draws."""
import re, sys
from pathlib import Path

import numpy as np

HOUSE = Path(__file__).resolve().parents[1] / "sim" / "house"
sys.path.insert(0, str(HOUSE))


def _pieces():
    import cabinets as cb
    out = {}
    for g in cb.GEOMS:
        name = re.search(r'name="([^"]+)"', g).group(1)
        m = next(a for a in cb.ASSETS if f'name="m_{name}"' in a)
        V = np.array(re.search(r'vertex="([^"]+)"', m).group(1).split(), float).reshape(-1, 3)
        T = np.array(re.search(r'texcoord="([^"]+)"', m).group(1).split(), float).reshape(-1, 2)
        pos = np.array(re.search(r'pos="([^"]+)"', g).group(1).split(), float)
        out[name] = (V + pos, T, re.search(r'material="([^"]+)"', g).group(1), 'contype="1"' in g)
    return out


def test_a_counter_has_doors_a_drawer_row_and_handles_proud_of_its_face():
    import cabinets as cb
    cb.reset(); cb.carcass("c_body", 0.0, 0.0, 2.0, 0.6, 0.1, 0.86); cb.fronts("c", 0.0, 2.0, 0.0, -1, 0.1, 0.86, drawers=True)
    P = _pieces()
    fronts = [n for n in P if "_front" in n]; handles = [n for n in P if "_handle" in n]
    assert len(fronts) == 4 + 2 and len(handles) == len(fronts)            # 4 doors of 50 cm under 2 drawers of 1 m
    for n in fronts:
        V = P[n][0]; assert V[:, 1].max() <= 1e-9 and V[:, 1].min() >= -cb.PROUD - 1e-9   # on the face, standing out -y
        assert P[n][2] == "cabinet" and not P[n][3]                        # the finish; the carcass is what's solid
    xs = sorted((P[n][0][:, 0].min(), P[n][0][:, 0].max()) for n in fronts if P[n][0][:, 2].max() < 0.86 - cb.DRAWER_H)
    assert all(b0 - a1 >= cb.GAP - 1e-9 for (_, a1), (b0, _) in zip(xs, xs[1:]))   # a reveal between neighbouring doors
    for n in handles:
        assert P[n][2] == "handle" and P[n][0][:, 1].max() <= -cb.PROUD + 1e-9  # on the door's face, not inside it
    assert P["c_body"][3]


def test_the_island_seating_side_has_panels_without_handles():
    import cabinets as cb
    cb.reset(); cb.panels("i_seat", 0.0, 1.8, 0.25, -1, 0.0, 0.86)
    P = _pieces()
    assert len([n for n in P if "_front" in n]) == 4 and not [n for n in P if "_handle" in n]


def test_every_face_carries_texture_at_real_scale_the_grain_running_up():
    import cabinets as cb
    cb.reset(); cb.piece("door", 0.2, -0.018, 0.1, 0.7, 0.0, 0.8)
    V, T, _, _ = _pieces()["door"]
    assert len(V) == 24 and len(T) == 24                                   # four corners a face, each its own coordinates
    front = [i for i in range(24) if abs(V[i, 1] + 0.018) < 1e-9][:4]      # the -y face: the door's face
    du = np.ptp(T[front, 0]); dv = np.ptp(T[front, 1])
    assert abs(du - 0.5 * cb.REP) < 1e-6 and abs(dv - 0.7 * cb.REP) < 1e-6  # u across (0.5 m), v up (0.7 m)
    for k in range(6):                                                     # no face stretched to a line
        f = slice(4 * k, 4 * k + 4); assert np.ptp(T[f, 0]) > 0 and np.ptp(T[f, 1]) > 0


def test_the_finish_is_drawn_per_house_and_leaves_the_other_draws_alone():
    import cabinets as cb, random
    assert cb.finish(70) == cb.finish(70)
    assert len({cb.finish(s)[0] for s in range(70, 100)}) >= 4
    r = random.Random(70 * 7907 + 11); before = [r.random() for _ in range(5)]
    cb.finish(70); r = random.Random(70 * 7907 + 11); assert [r.random() for _ in range(5)] == before


def test_the_meshes_compile_and_the_fronts_stand_where_they_were_drawn(tmp_path):
    import cabinets as cb, mujoco
    cb.reset(); cb.carcass("c_body", 0.0, 0.0, 1.0, 0.6, 0.1, 0.86); cb.fronts("c", 0.0, 1.0, 0.0, -1, 0.1, 0.86)
    xml = ('<mujoco><asset><material name="cabinet"/><material name="handle"/>\n' + "\n".join(cb.ASSETS) +
           '</asset><worldbody>\n' + "\n".join(cb.GEOMS) + '</worldbody></mujoco>')
    m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "c_front0")
    lo = np.array([0.0 + cb.GAP / 2, -cb.PROUD, 0.1 + cb.GAP / 2]); hi = np.array([0.5 - cb.GAP / 2, 0.0, 0.86 - cb.GAP / 2])
    assert np.allclose(d.geom_xpos[g], (lo + hi) / 2, atol=1e-4)


def test_the_house_furniture_is_textured_meshes_and_the_cabinets_are_in_it():
    """[jmf], [WTj]: every wood, upholstery and cabinet piece of the indoor furniture is a mesh carrying its own texture
    coordinates (no box stretching a texture up its faces); the cabinets have fronts and handles"""
    import make_house as mh, make_nights as mn, cabinets as cb
    mh.use_layout(70); geoms = mn.furniture_xml()
    textured = [g for g in geoms if re.search(r'material="(wood|upholstery|cabinet|stone)', g) and 'euler=' not in g]
    assert textured and all('type="mesh"' in g for g in textured), [g[:80] for g in textured if 'type="mesh"' not in g][:3]
    names = " ".join(geoms)
    for n in ("kitchen_counter_front", "kitchen_island_front", "kitchen_island_seat_front", "tv_console_front", "_handle"):
        assert n in names, n
    meshes = {re.search(r'mesh="([^"]+)"', g).group(1) for g in geoms if 'type="mesh"' in g}
    assert meshes <= {re.search(r'name="([^"]+)"', a).group(1) for a in cb.ASSETS}   # each has its mesh


def test_the_dining_chairs_and_stool_are_textured_meshes():
    import movables as mv, cabinets as cb
    cb.reset()
    for c in [dict(id="chair_x", x=0, y=0, yaw=0, w=.44, d=.44, h=.46, back_h=.9, rgba="0.5 0.4 0.3 1", kind="chair_w"),
              dict(id="stool", x=0, y=0, yaw=0, w=.4, d=.4, h=.42, back_h=0.0, rgba="0.55 0.52 0.48 1", kind="stool")]:
        x = mv.xml(c)
        assert 'type="box"' not in x and x.count('type="mesh"') >= 5 and 'material="wood_m"' in x
