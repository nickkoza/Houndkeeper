"""[G48]: the arm guard judges the base still from odometry at the robot's own stamps, so a burst of readings delivered
late to the tidy (its process busy) is still a still base, as the readings say; and a stop whose first arm move the guard
refused does not try to stow an arm that never left the stow (a refused fold had ended the night, 18 Oct 2026); and
the arm waits for the base to read still for a second without a break, not one lull between the gait's steps."""
import math
import sys
import types
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from houndkeeper import arm_guard as ag  # noqa: E402


def _tidy(now):
    from houndkeeper.tidy_module import TidyArm
    t = TidyArm.__new__(TidyArm); g = ag.ArmGuard(lambda: now[0], posture="lie", reach=lambda q: 0.0)
    t._guard = g; t._live_loc = None; return t, g


def _msg(ts):
    return types.SimpleNamespace(ts=ts, position=types.SimpleNamespace(x=0.0, y=0.0, z=0.3),
                                 orientation=types.SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))


def test_a_late_burst_of_odometry_reads_at_its_own_stamps():
    import houndkeeper.tidy_module as tm
    now = [100.0]; t, g = _tidy(now)
    real = tm.clock.now; tm.clock.now = lambda: now[0]
    try:
        for k in range(25): _on_odom(t, _msg(99.0 + k * 0.04))   # a second of readings, all arriving at 100.0
    finally: tm.clock.now = real
    assert g.base.moving(now[0]) is None                                  # still: arrival times would read "too little"
    late = ag.BaseWatch()
    for k in range(25): late.on_odom(100.0, 0.0, 0.0, 0.0)
    assert "too little" in late.moving(100.0)                             # the planted error: stamped on arrival


def test_a_message_without_a_stamp_falls_back_to_arrival():
    import houndkeeper.tidy_module as tm
    now = [100.0]; t, g = _tidy(now)
    real = tm.clock.now; tm.clock.now = lambda: now[0]
    try:
        m = _msg(None); _on_odom(t, m)
    finally: tm.clock.now = real
    assert g.base._odom[-1][0] == 100.0


def test_a_stamp_ahead_of_the_clock_reads_as_its_arrival():
    """[G48]: a stamp later than the reading arrived is clamped to arrival, so it cannot keep a stalled stream fresh"""
    import houndkeeper.tidy_module as tm
    now = [100.0]; t, g = _tidy(now)
    real = tm.clock.now; tm.clock.now = lambda: now[0]
    try:
        _on_odom(t, _msg(150.0))
    finally: tm.clock.now = real
    assert g.base._odom[-1][0] == 100.0
    now[0] = 102.0; assert "old" in g.base.moving(now[0])                # the stream stopped: stale, not still


def _on_odom(t, msg):
    from houndkeeper.tidy_module import TidyArm
    TidyArm._on_odom(t, msg)


def test_the_stop_folds_the_arm_only_if_it_left_the_stow():
    g = ag.ArmGuard(lambda: 0.0, posture="lie", read_q=lambda: np.zeros(7))
    assert g.stowed()                                                     # refused first move: nothing to stow
    g.read_q = lambda: np.array([0.0, -2.95, 1.3, 1.2, 0.0, 0.0, 0.0]); assert not g.stowed()
    src = (Path(__file__).resolve().parents[1] / "src" / "houndkeeper" / "tidy_module.py").read_text()
    assert "if not self._guard.stowed():" in src and src.index("if not self._guard.stowed():") < src.index("pick.stow(S, q_ready)")


def _gait(g, t0, t1, pulse=True):
    """odometry from t0 to t1 at 25 Hz: the trunk swinging side to side as the gait steps on after a turn (0.08 rad each
    way, once a second: up to 0.5 rad/s), or still"""
    def yaw(t): return 0.08 * math.sin(2 * math.pi * t) if pulse else 0.0
    for k in range(int(round((t1 - t0) * 25))):
        t = t0 + (k + 1) / 25; g.on_odom(t, 0.0, 0.0, yaw(t))


def _settle(now, feed):
    """TidyArm._base_settled on a fake clock: each sleep advances it and feeds odometry for that interval"""
    import houndkeeper.tidy_module as tm
    from houndkeeper.tidy_module import TidyArm
    t, g = _tidy(now); real = (tm.clock.now, tm.clock.sleep)
    def sleep(dt): feed(g, now[0], now[0] + dt); now[0] += dt
    tm.clock.now, tm.clock.sleep = (lambda: now[0]), sleep
    try:
        feed(g, now[0] - 1.0, now[0]); return TidyArm._base_settled(t), now[0]
    finally: tm.clock.now, tm.clock.sleep = real


def test_a_base_stepping_in_pulses_never_reads_settled():
    """[G48]: the lull between a turn's steps reads still for an instant; the arm waits for a second without a break"""
    ok, t = _settle([100.0], lambda g, a, b: _gait(g, a, b, pulse=True))
    assert not ok and t >= 100.0 + 5.0 - 0.11


def test_a_still_base_reads_settled_after_the_hold():
    ok, t = _settle([100.0], lambda g, a, b: _gait(g, a, b, pulse=False))
    assert ok and t < 100.0 + 1.5

