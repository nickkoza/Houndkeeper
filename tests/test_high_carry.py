"""The carry allows for as long a hang as any item could have, as high as the arm reaches (sim-grasp [2ho]): a figure
gripped by one leg, carried by its measured length, swung into the robot's side (28 Sep)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))
import pick  # noqa: E402

REACH_Z = 0.55                                     # this fake arm reaches the carry height up to here


class _S:
    def bin_clear_z(self, hang): return 0.40 + hang
    def ik_lean(self, p, jaw, lim): return (("q", round(float(p[2]), 3)), 0.0, 0.0) if p[2] <= REACH_Z + 1e-9 else (None, 1.0, 0.0)


def test_a_short_item_is_still_carried_high(monkeypatch):
    monkeypatch.setattr(pick, "HIGH_CARRY", True)
    q, hang = pick.carry_plan(_S(), {"length_cm": 8.0, "grip_width_cm": 3.0}, np.zeros(3), np.array([1.0, 0.0]), "qu")
    assert hang == pick.HANG_MAX and q != "qu" and REACH_Z - 0.03 <= q[1] <= REACH_Z   # the highest it reaches


def test_as_before_with_the_rule_off(monkeypatch):
    monkeypatch.setattr(pick, "HIGH_CARRY", False)
    q, hang = pick.carry_plan(_S(), {"length_cm": 8.0, "grip_width_cm": 3.0}, np.zeros(3), np.array([1.0, 0.0]), "qu")
    assert abs(hang - 0.08) < 1e-9 and q[1] == 0.48


class _Held:
    def __init__(self): self.asked = None; self.waypoints = []
    def gripper(self): return 0.01
    def over_bin(self, hang): self.asked = hang; return None
    def state(self, s): pass
    def arm_to_safe(self, q, secs): pass
    def grip_to(self, g, secs): pass
    def step(self, n): pass


def test_the_policy_picks_carry_is_as_high(monkeypatch):
    monkeypatch.setattr(pick, "HIGH_CARRY", True)
    S = _Held(); pick.carry_to_bin(S, None, None); assert S.asked == pick.HANG_MAX
