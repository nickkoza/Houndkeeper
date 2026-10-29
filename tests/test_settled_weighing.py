"""[XCl]: the pick weighs from joint torques read once the arm is still. Streamed at wall-clock speed in DimOS, the arm
is still catching up just after a move, and its tracking torque read as weight (a 0.2 kg shoe weighed 3.5 kg, 26 Sep)."""
import sys, time
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "houndkeeper"))


class LaggingArm:
    """an arm still moving for `settle_s` after the call: while it moves, a tracking torque adds to the true load"""
    def __init__(self, settle_s=0.5, load=0.3, tracking=5.0):
        self.t0, self.settle_s, self.load, self.tracking = time.time(), settle_s, load, tracking
    def _moving(self): return max(0.0, 1 - (time.time() - self.t0) / self.settle_s)
    def read_joint_positions(self): return [0.1 * (1 - self._moving())] * 6 + [0.0]
    def read_joint_velocities(self): return [0.2 / self.settle_s * (self._moving() > 0)] * 6 + [0.0]   # as the adapter interface
    def read_joint_efforts(self): return [self.load + self.tracking * self._moving()] * 6 + [0.0]


def test_weighs_once_the_arm_is_still():
    import dimos_robot
    robot = object.__new__(dimos_robot.DimosRobot); robot.adapter = LaggingArm()
    tau = robot.settled_efforts(timeout=2.0)
    assert np.allclose(tau, 0.3, atol=0.05), tau                  # the load, not the tracking torque
    assert abs(robot.joint_efforts()[0] - 0.3) < 0.05             # (by now the arm really is still)
    fresh = object.__new__(dimos_robot.DimosRobot); fresh.adapter = LaggingArm()
    assert fresh.joint_efforts()[0] > 1.0                          # what an immediate read gives
