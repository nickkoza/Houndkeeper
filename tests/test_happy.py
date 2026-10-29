"""The happy wiggle after an item lands in the bin (manners [Yju]), within every gesture's limits ([ZjA])."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


def test_the_wiggle_swings_both_ways_and_ends_facing_the_same_way():
    from houndkeeper.tidy_module import TidyArm
    plan = TidyArm.wiggle_plan()
    signs = [r > 0 for r, _ in plan]
    assert sum(a != b for a, b in zip(signs, signs[1:])) >= 3                 # a couple of times each way
    assert abs(sum(r * t for r, t in plan)) < 1e-9                            # the swings cancel: facing the way it was
    assert max(abs(r) for r, _ in plan) <= 1.2                               # within what the walking controller follows
    assert sum(t for _, t in plan) <= 2.0                                    # short


class _Pub:
    def __init__(self): self.sent = []
    def publish(self, tw): self.sent.append(tw)


def test_the_wiggle_can_be_turned_off(monkeypatch):
    from houndkeeper.tidy_module import TidyArm
    arm = TidyArm.__new__(TidyArm); arm.cmd_vel = _Pub(); monkeypatch.setattr(TidyArm, "HAPPY", False)
    TidyArm._happy(arm); assert arm.cmd_vel.sent == []


def test_the_wiggle_ends_stopped(monkeypatch):
    from houndkeeper.tidy_module import TidyArm
    import houndkeeper.tidy_module as tm
    monkeypatch.setattr(tm.clock, "sleep", lambda s: None)   # the robot's clock ([3Pv])
    arm = TidyArm.__new__(TidyArm); arm.cmd_vel = _Pub(); monkeypatch.setattr(TidyArm, "HAPPY", True)
    TidyArm._happy(arm)
    assert arm.cmd_vel.sent and arm.cmd_vel.sent[-1].angular.z == 0.0 and arm.cmd_vel.sent[-1].linear.x == 0.0
