"""[rUS]: the dog stands where it is told to start. It used to settle at the world origin first and be moved after: in
a furnished house the origin was inside a bench, the dog stood on it, dropped from 0.57 m and landed with its back left
thigh wedged under the tray (the showcase video's limp, 30 Sep 2026)."""
import math, sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "arm")); sys.path.insert(0, str(HK / "src"))
mujoco = pytest.importorskip("mujoco")


def test_the_dog_settles_standing_at_its_start_not_on_what_is_at_the_origin():
    import go1_arm as ga, payload_test as pt, legs
    def bench(spec):                                               # furniture where the origin is
        g = spec.worldbody.add_geom(); g.type = mujoco.mjtGeom.mjGEOM_BOX; g.pos = [0, 0, 0.2]; g.size = [0.5, 0.5, 0.2]
    m = ga.build(True, pt.STOW, pick=True, dress=bench); m.opt.timestep = 0.005; d = mujoco.MjData(m)
    legs.start_standing(m, d, 0.30, at=(-2.0, -3.0, math.radians(100)))
    assert np.allclose(d.qpos[0:2], [-2.0, -3.0], atol=0.10)       # where it was told (settling sways a few cm), on the floor
    assert 0.22 < d.qpos[2] < 0.34                                 # 0.239 m: the legs carry the arm's whole weight now that its
    # gravity feed-forward rides its actuators ([cqG]); MuJoCo's gravcomp also lifted the dog by 29 N (0.253 m, before 3 Oct)
    R = d.xmat[m.body("trunk").id].reshape(3, 3); assert R[2, 2] > math.cos(math.radians(8))   # upright
    thigh, tray = m.body("RL_thigh").id, m.body("tray").id
    assert not any({m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]} == {thigh, tray} for c in d.contact[:d.ncon])
