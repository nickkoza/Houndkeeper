"""Let go in the bin, the arm leaves the way it came down, straight up, before any other move (sim-grasp [2ho]): a
train set down against the rim was knocked out by the arm's stow (28 Sep)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))
import pick  # noqa: E402


class _S:
    """an arm lowering 1 cm a step until the tray is met 4 cm down"""
    def __init__(self): self.q = np.array([0.30, 0, 0, 0, 0, 0.0]); self.moves = []; self.z = 0.30   # q[0] is its height
    def target(self): return self.q.copy()
    def gripper_frame(self, q): return np.array([0, 0, -1.0]), np.eye(3)
    def grasp_point(self): return np.array([0.0, 0.0, self.z])
    def joint_efforts(self): return np.full(6, 5.0 if self.z <= 0.26 + 1e-9 else 0.0)
    def ik_pose(self, p, R, seed): q = np.zeros(6); q[0] = p[2]; return q, 0.0
    def _hits(self, q): return 0
    def arm_to(self, q, secs, grip=None): self.q = np.asarray(q, float); self.z = float(q[0]); self.moves.append(("arm", self.z))
    def step(self, n): pass
    def grip_to(self, g, secs): self.moves.append(("grip", g))


def test_let_go_it_rises_the_way_it_came(monkeypatch):
    monkeypatch.setattr(pick, "BIN_EXIT", True)
    S = _S(); pick.release_in_bin(S, max_m=0.12)
    kinds = [m[0] for m in S.moves]; i = kinds.index("grip")
    down, up = [m[1] for m in S.moves[:i]], [m[1] for m in S.moves[i + 1:]]
    assert down == sorted(down, reverse=True) and min(down) <= 0.26 + 1e-9        # lowered until it met the tray
    assert up == sorted(up) and abs(up[-1] - 0.30) < 1e-9                          # then straight back up, to where it began
    assert up == down[::-1][1:] + [0.30]                                            # by the same poses


def test_off_it_lets_go_and_stays(monkeypatch):
    monkeypatch.setattr(pick, "BIN_EXIT", False)
    S = _S(); pick.release_in_bin(S, max_m=0.12)
    assert [m[0] for m in S.moves][-1] == "grip"
