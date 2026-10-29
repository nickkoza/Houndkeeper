"""[JaG]: the sim's lidar returns what the Go2's L1 does ([x3f]): its point budget, range noise and minimum range, not
every pixel of a depth camera exactly."""
import numpy as np

from dimos.simulation.mujoco.lidar_model import LidarModel


def _wall(n=200):
    """a dense depth camera's view of a wall 2 m ahead: the camera at the origin looking along +x (MuJoCo: down its -z)"""
    y, z = np.meshgrid(np.linspace(-1.5, 1.5, n), np.linspace(-1.5, 1.5, n))
    pts = np.stack([np.full(y.size, 2.0), y.ravel(), z.ravel()], 1)
    cam_mat = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float).T      # columns: right, up, back (-forward)
    return pts, np.zeros(3), cam_mat


def test_it_returns_the_l1s_points_a_second_with_its_noise():
    pts, c, R = _wall(); m = LidarModel("points_per_s=21600,range_sd=0.02,min_range=0.05,seed=1")
    got = m.sample(pts, c, R, 0.5)
    assert abs(len(got) - 10800) < 400                                     # half a second's budget of a dense 40,000
    along = np.linalg.norm(got, axis=1) - 2.0 * np.linalg.norm(got, axis=1) / got[:, 0]   # range less the wall's, per ray
    assert 0.015 < along.std() < 0.025                                     # ~2 cm range noise


def test_nothing_nearer_than_its_minimum_range():
    pts = np.array([[0.03, 0, 0], [0.04, 0.01, 0], [1.0, 0, 0]] * 50, float)
    got = LidarModel("points_per_s=1000000,range_sd=0.0,min_range=0.05,seed=2").sample(pts, np.zeros(3), np.eye(3), 1.0)
    assert (np.linalg.norm(got, axis=1) > 0.05).all()


def test_dark_and_shiny_surfaces_return_little():
    """black fabric absorbs the laser and glass reflects it away: those pixels go missing most of the time"""
    import mujoco
    xml = """<mujoco><asset><material name="tv_screen" rgba="0.05 0.05 0.05 1"/></asset><worldbody>
      <geom name="white" type="box" size="1 1 0.1" rgba="0.8 0.8 0.8 1"/>
      <geom name="black" type="box" size="1 1 0.1" pos="3 0 0" rgba="0.03 0.03 0.03 1"/>
      <geom name="tv" type="box" size="1 1 0.1" pos="6 0 0" material="tv_screen"/></worldbody></mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml)
    lm = LidarModel("dark_albedo=0.10,dark_drop=0.8,shiny_drop=0.7,seed=4"); lm.surfaces(m)
    seg = np.array([[0] * 1000, [1] * 1000, [2] * 1000])
    lost = lm.dropped(seg).mean(1)
    assert lost[0] == 0 and 0.45 < lost[1] < 0.8 and 0.6 < lost[2] < 0.8          # black at 3% albedo: 0.8 x (1 - 0.3)


def test_each_return_knows_the_surface_it_came_from():
    """a ray from the lidar to each sampled point finds the geom it hit (no segmentation render: MuJoCo 3.10's failed
    on the house scene, index past its table, 27 Sep)"""
    import mujoco
    xml = """<mujoco><worldbody><geom name="near" type="box" size="0.1 1 1" pos="2 0 0"/>
      <geom name="far" type="box" size="0.1 1 1" pos="4 3 0"/></worldbody></mujoco>"""
    m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    pts = np.array([[1.9, 0.2, 0.0], [3.9, 3.0, 0.1]])
    got = LidarModel("seed=1").hit_geoms(m, d, np.zeros(3), pts, np.ones(6, np.uint8))
    assert list(got) == [m.geom("near").id, m.geom("far").id]
