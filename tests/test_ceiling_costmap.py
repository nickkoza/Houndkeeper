"""[Xl1] an indoor house's ceiling changes nothing in DimOS's costmap: the floor under it stays free, walls and a table
top stay obstacles, exactly as without a ceiling (DimOS's ignore_above, as sim/dimos/env.py sets it: [UvE])"""
import sys
import time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "dimos"))
import env  # noqa: E402
from dimos.mapping.pointclouds.occupancy import height_cost_occupancy
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def _costmap(ceiling: bool):
    g = np.mgrid[-1:1:0.02, -1:1:0.02].reshape(2, -1).T
    parts = [np.c_[g, np.zeros(len(g))],                                                                    # floor
             np.array([[0.5, y, z] for y in np.arange(-1, 1, 0.02) for z in np.arange(0, 2.4, 0.05)]),        # a wall
             np.array([[x, y, 0.7] for x in np.arange(-0.6, -0.4, 0.02) for y in np.arange(-0.1, 0.1, 0.02)])]   # a table top
    if ceiling: parts.append(np.c_[g, np.full(len(g), 2.4)])
    return height_cost_occupancy(PointCloud2.from_numpy(np.vstack(parts).astype(np.float32), frame_id="world", timestamp=time.time()),
                                 **env.COSTMAP)


def test_a_ceiling_changes_nothing():
    a, b = _costmap(False), _costmap(True)
    assert a.grid.shape == b.grid.shape and (a.grid == b.grid).all()


def test_the_wall_and_table_are_still_obstacles_under_it():
    b = _costmap(True); res, ox, oy = b.resolution, b.origin.position.x, b.origin.position.y
    near = lambda x, y: b.grid[int((y - oy) / res) - 1: int((y - oy) / res) + 2, int((x - ox) / res) - 1: int((x - ox) / res) + 2].max()
    assert near(0.5, 0.3) >= 90 and near(-0.5, 0.1) >= 90
