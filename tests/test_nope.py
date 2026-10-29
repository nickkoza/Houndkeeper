"""The nope shake after an empty look (manners [N2T])."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


class _S:
    def __init__(self): self.q = np.array([0.1, -0.2, 0.3, -0.1, 0.05, 0.0]); self.moves = []
    def target(self): return self.q.copy()
    def arm_to(self, q, secs, grip=None):                       # the robot's speed limit, as DimosRobot.arm_to
        q = np.asarray(q, float); self.secs = getattr(self, 'secs', 0.0) + max(secs, float(np.abs(q - self.q).max()) / 1.0 * 1.6)
        self.moves.append(q.copy()); self.q = q.copy()


def test_nope_turns_only_the_shoulder_and_returns():
    from houndkeeper.tidy_module import TidyArm
    arm = TidyArm.__new__(TidyArm); S = _S(); q0 = S.q.copy()
    TidyArm._nope(arm, S)
    M = np.array(S.moves)
    assert len(M) == 5 and np.allclose(M[:, 1:], q0[1:]) and np.ptp(M[:, 0]) >= 0.1 and np.allclose(S.q, q0)
    assert S.secs <= 1.1


def test_nope_can_be_turned_off(monkeypatch):
    from houndkeeper.tidy_module import TidyArm
    arm = TidyArm.__new__(TidyArm); monkeypatch.setattr(TidyArm, "NOPE", False); S = _S()
    TidyArm._nope(arm, S); assert S.moves == []
