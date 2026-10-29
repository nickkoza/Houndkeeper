"""[FQ3]: the robot lies down from standing and stands up from lying with the arm stowed and the kit on, without tipping
(sim, the Go1 stand-in on leg PD, as the sims pose it). The kit is a dummy at the budget's full 1.8 kg ([Rrm]), on the
trunk's top at the mount or at the bin's end (est.: the printed parts are not weighed)."""
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "arm")); sys.path.insert(0, str(HK / "src"))
mujoco = pytest.importorskip("mujoco")

TIP_DEG = 22.5      # the lowest measured tipping point: standing, sideways, arm stowed (sim-tip-safety [BuG])


def _cycle(kit_at_x):
    import go1_arm as ga
    import legs
    import payload_test as pt
    from houndkeeper import params as P
    m = ga.build(True, pt.STOW); d = mujoco.MjData(m); tr = m.body("trunk").id
    kit = float(P.get("kit.budget_kg")); M = m.body_mass[tr]; at = np.array([kit_at_x, 0.0, 0.12])
    m.body_ipos[tr] = (M * m.body_ipos[tr] + kit * at) / (M + kit); m.body_mass[tr] = M + kit
    legs.start_standing(m, d); z_stand = float(d.qpos[2]); worst = [0.0]

    def tilt(m_, d_):
        worst[0] = max(worst[0], float(np.degrees(np.arccos(np.clip(d_.xmat[tr].reshape(3, 3)[2, 2], -1, 1)))))
    legs.move_legs(m, d, legs.LIE, 2.0, tilt); legs.move_legs(m, d, legs.LIE, 1.0, tilt); z_lie = float(d.qpos[2])
    legs.move_legs(m, d, legs.STAND, 2.0, tilt); legs.move_legs(m, d, legs.STAND, 1.0, tilt)
    return z_stand, z_lie, float(d.qpos[2]), worst[0]


@pytest.mark.parametrize("kit_at_x", [0.10, -0.15])
def test_lie_down_and_stand_up_loaded_without_tipping(kit_at_x):           # [FQ3]
    z_stand, z_lie, z_end, worst = _cycle(kit_at_x)
    assert z_lie < z_stand - 0.08                                            # it lay down
    assert abs(z_end - z_stand) < 0.02                                       # and stood back up to its height
    assert worst < TIP_DEG / 2                                               # never near tipping
