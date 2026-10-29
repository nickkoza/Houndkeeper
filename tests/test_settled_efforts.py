"""[XCl]'s weighing on DimOS: the torques are averaged over the arm's own readings once it is still, not over wall
time (28 Sep: at 0.28x the sim published one or two frames while the printed frame's flex rang, and a pony weighed 2.6 kg)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from houndkeeper.dimos_robot import DimosRobot  # noqa: E402


class _Arm:
    """publishes a new joint state every few reads: moving at first, then still with the torque ringing +-2 Nm"""
    def __init__(self): self.reads = 0
    def _k(self): return self.reads // 3
    def read_joint_efforts(self):
        self.reads += 1; k = self._k(); return [5.0 + 2.0 * np.sin(k * 2.2)] * 6
    def read_joint_velocities(self): return [0.5 if self._k() < 10 else 0.0] * 6


def test_the_torque_is_averaged_over_the_arms_own_readings_once_still():
    r = DimosRobot.__new__(DimosRobot); r.adapter = _Arm()
    tau = r.settled_efforts(timeout=5.0)
    assert np.allclose(tau, 5.0, atol=0.4)                      # the ring averages out over 12 readings
    assert r.adapter._k() >= 10 + DimosRobot.SETTLE_SAMPLES     # and only after the arm stopped
