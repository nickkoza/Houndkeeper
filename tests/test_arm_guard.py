"""The arm's one gate (arm-on-board [Bhf]): no deploying while the base moves [G48], moving only stopped and down [FWp],
inside the tip-safety envelope for the weight held [qpO] (sim-tip-safety [uMO]); judged from odometry, the posture it
took and the arm's own readings."""
import sys
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from houndkeeper import arm_guard as ag

ENV = {"safe_reach_m": {"lie_0.0": 0.6, "lie_0.5": 0.6, "lie_1.5": 0.3, "crouch_0.0": 0.6},
       "safe_speed": {"lie_0.0": 3.0, "lie_1.5": None, "crouch_0.0": None}}
OUT = np.array([0.0, -2.95, 1.3, 1.2, 0.0, 0.0, 0.0])      # a floor reach, well off the stow
STOWED = np.zeros(7)


class _Clock:
    def __init__(self): self.t = 100.0
    def __call__(self): return self.t


class _Arm:
    def __init__(self): self.q = STOWED.copy(); self.sent = []
    def read_joint_positions(self): return list(self.q)
    def write_joint_positions(self, q, velocity=1.0): self.sent.append(np.array(q)); self.q = np.array(q); return True
    def write_joint_velocities(self, v): self.sent.append(("vel", v)); return True
    def write_stop(self): return True


def _env(tmp_path, data=ENV):
    p = tmp_path / "env.json"; p.write_text(__import__("json").dumps(data)); return ag.Envelope(p)


def _still(g, clock, x=0.0, secs=1.0, v=0.0):
    """odometry for secs up to now: still, or walking at v m/s"""
    for k in range(int(secs * 50) + 1):
        t = clock.t - secs + k / 50; g.on_odom(t, x + v * (t - clock.t + secs), 0.0, 0.0)


def _guard(tmp_path, posture="lie", reach=lambda q: float(np.abs(q[1]) / 5.0)):
    clock = _Clock(); arm = _Arm()
    g = ag.ArmGuard(clock, envelope=_env(tmp_path), reach=reach, posture=posture)
    return g, ag.GuardedArm(arm, g), arm, clock


def _stream(ga, clock, q0, q1, v=0.5, hz=50):
    """a move at v rad/s, streamed at hz as DimosRobot streams it"""
    n = max(1, int(np.abs(q1 - q0).max() / v * hz))
    for k in range(1, n + 1):
        clock.t += 1 / hz; ga.guard.on_odom(clock.t, 0.0, 0.0, 0.0); ga.write_joint_positions(q0 + (q1 - q0) * k / n)


def test_lying_still_the_arm_moves_out_and_back(tmp_path):                    # [FWp]
    g, ga, arm, clock = _guard(tmp_path); _still(g, clock)
    _stream(ga, clock, STOWED, OUT); _stream(ga, clock, OUT, STOWED)
    assert np.allclose(arm.q, STOWED) and not g.refusals


def test_while_the_base_moves_the_arm_cannot_deploy(tmp_path):                # [G48]
    g, ga, arm, clock = _guard(tmp_path); _still(g, clock, v=0.3)
    with pytest.raises(ag.ArmRefused, match="moving"): ga.write_joint_positions(STOWED + 0.01)
    assert arm.sent == []


def test_a_sway_that_ends_where_it_began_is_not_still():                      # [G48]: the gait after a turn
    """the base turning 0.5 rad/s one way then back inside the window reads moving, though its ends agree"""
    b = ag.BaseWatch()
    for k in range(26):
        t = 100.0 + k / 50; yaw = 0.5 * (t - 100.0) if t <= 100.25 else 0.125 - 0.5 * (t - 100.25)
        b.on_odom(t, 0.0, 0.0, yaw)
    assert b.moving(100.5) and "moving" in b.moving(100.5)
    q = ag.BaseWatch()
    for k in range(26): q.on_odom(100.0 + k / 50, 0.001 * (k % 2), 0.0, 0.0)   # 1 mm of jitter: still
    assert q.moving(100.5) is None


def test_no_fresh_odometry_is_not_still(tmp_path):                            # [G48]: no reading is never "still"
    g, ga, arm, clock = _guard(tmp_path)
    with pytest.raises(ag.ArmRefused, match="no odometry"): ga.write_joint_positions(STOWED + 0.01)
    _still(g, clock); clock.t += 2.0
    with pytest.raises(ag.ArmRefused, match="old"): ga.write_joint_positions(STOWED + 0.01)


def test_standing_only_the_standing_looks_measured_poses_empty(tmp_path):    # [FWp]; sim-tip-safety [BuG]
    g, ga, arm, clock = _guard(tmp_path, posture="stand"); _still(g, clock)
    look = STOWED.copy(); look[[0, 4, 5]] = [0.2, 0.2, 0.1]
    _stream(ga, clock, STOWED, look); _stream(ga, clock, look, STOWED)
    assert g.refuses(OUT) is not None and "standing" in g.refuses(OUT)
    g.carried_kg = 0.2; assert g.refuses(look) is not None


def test_unknown_or_changing_posture_refuses(tmp_path):                     # [FWp]
    for posture in (None, "changing"):
        g, ga, arm, clock = _guard(tmp_path, posture=posture); _still(g, clock)
        with pytest.raises(ag.ArmRefused, match="lying or crouched"): ga.write_joint_positions(STOWED + 0.01)


def test_posture_changes_only_with_the_arm_stowed_and_are_kept(tmp_path):   # [G48], [FWp]
    g, ga, arm, clock = _guard(tmp_path, posture="stand"); _still(g, clock)
    seen = []
    robot = types.SimpleNamespace(liedown=lambda: seen.append(g.posture) or True, standup=lambda: True)
    gp = ag.GuardedPosture(robot, g)
    assert gp.liedown() and g.posture == "lie" and seen == ["changing"]       # the arm held while it happens
    _stream(ga, clock, STOWED, OUT)
    with pytest.raises(ag.ArmRefused, match="arm out"): gp.standup()
    _stream(ga, clock, OUT, STOWED); assert gp.standup() and g.posture == "stand"
    gp_fail = ag.GuardedPosture(types.SimpleNamespace(liedown=lambda: False), g)
    assert not gp_fail.liedown() and g.posture is None                       # a failed change is no known posture


def test_reach_past_the_envelope_for_the_weight_held_is_refused(tmp_path):  # [qpO], [uMO]
    g, ga, arm, clock = _guard(tmp_path); _still(g, clock)
    half = STOWED.copy(); half[1] = -2.0                                     # reach 0.40 m in the fake kinematics
    _stream(ga, clock, STOWED, half)
    g.carried_kg = 0.3; assert g.refuses(half) is None                       # judged as 0.5 kg: 0.6 m
    g.carried_kg = 0.8                                                       # judged as 1.5 kg: 0.3 m
    with pytest.raises(ag.ArmRefused, match="past the envelope"): ga.write_joint_positions(half + [0, 0.01, 0, 0, 0, 0, 0])
    g.carried_kg = 2.0; assert "no tip-safety envelope" in g.refuses(half)    # heavier than anything measured


def test_a_jump_faster_than_the_safe_speed_is_refused(tmp_path):            # [qpO]
    g, ga, arm, clock = _guard(tmp_path); _still(g, clock)
    _stream(ga, clock, STOWED, STOWED + 0.3, v=0.9)                          # under the arm's 1 rad/s
    clock.t += 0.02
    with pytest.raises(ag.ArmRefused, match="rad/s past"): ga.write_joint_positions(STOWED + 0.6)
    clock.t += 10.0; _still(g, clock)                                        # a long wait makes no jump slow
    with pytest.raises(ag.ArmRefused, match="rad/s past"): ga.write_joint_positions(STOWED + 0.6)
    ga.write_joint_positions(STOWED + 0.3)                                   # holding the last command always passes


def test_unmeasured_speed_falls_to_the_global_limit_and_is_said(tmp_path):  # [qpO]: lie at 1.5 kg measured no safe speed
    g, ga, arm, clock = _guard(tmp_path); _still(g, clock); g.carried_kg = 1.0
    assert g.vmax() == ag.ARM_VMAX and any("no measured safe speed" in n for n in g.notes)
    g.posture = "crouch"; g.carried_kg = 0.2                                 # nothing measured at or above: no move
    with pytest.raises(ag.ArmRefused, match="no safe arm speed"): ga.write_joint_positions(STOWED + 0.01)


def test_a_toy_held_gets_the_measured_half_kilo_speed_not_the_fallback():  # [qpO], [uMO], sim-tip-safety [Yns]
    env = ag.Envelope()                                                      # the envelope the robot uses
    for posture in ("lie", "crouch"):
        want = env.data["safe_speed"].get(f"{posture}_0.5")
        assert want is not None, f"{posture} at 0.5 kg measured no safe speed"
        g = ag.ArmGuard(_Clock(), envelope=env, reach=lambda q: 0.0, posture=posture); g.carried_kg = 0.3
        assert g.vmax() == min(want, ag.ARM_VMAX) and not g.notes            # the 0.5 kg row, not "no measured safe speed"


def test_without_an_envelope_only_the_stow(tmp_path):                       # sim-tip-safety's failure behaviour
    clock = _Clock(); arm = _Arm(); arm.q = STOWED + 0.05
    g = ag.ArmGuard(clock, envelope=ag.Envelope(tmp_path / "none.json"), reach=lambda q: 0.0, posture="lie")
    ga = ag.GuardedArm(arm, g); _still(g, clock)
    _stream(ga, clock, STOWED + 0.05, STOWED)
    with pytest.raises(ag.ArmRefused, match="no tip-safety envelope"): ga.write_joint_positions(STOWED + 0.2)


def test_no_command_gets_round_the_guard(tmp_path):                         # [G48], [FWp], [qpO]: one choke point
    g, ga, arm, clock = _guard(tmp_path); _still(g, clock, v=0.3)
    for name, args in (("write_joint_velocities", ([0.1] * 7,)), ("write_cartesian_position", ({"x": 0.3},)),
                       ("write_joint_efforts", ([0.0] * 7,))):
        with pytest.raises(ag.ArmRefused, match="bypasses"): getattr(ga, name)(*args)
    assert arm.sent == [] and ga.write_stop() and ga.read_joint_positions() == list(STOWED)
    ga.set_payload(0.4); assert g.carried_kg == 0.4


def test_base_moving_with_the_arm_out_raises_the_alarm(tmp_path):           # failure behaviour: the base stops
    g, ga, arm, clock = _guard(tmp_path, posture="stand"); g.read_q = arm.read_joint_positions
    _still(g, clock, v=0.3); assert g.base_alarm() is None                   # walking stowed
    arm.q = OUT.copy(); assert "moving" in g.base_alarm()
    g.posture = "lie"; assert g.base_alarm() is None                         # lying, it cannot walk: rocking is not walking


def test_the_tidy_stops_the_base_when_the_arm_is_out(tmp_path):             # failure behaviour, wired in the tidy
    from houndkeeper.tidy_module import TidyArm
    t = TidyArm.__new__(TidyArm); g, ga, arm, clock = _guard(tmp_path, posture="stand"); g.read_q = arm.read_joint_positions
    _still(g, clock, v=0.3); arm.q = OUT.copy()
    pub, cancel = [], []
    t._guard = g; t._live_loc = None; t.cmd_vel = types.SimpleNamespace(publish=pub.append)
    t._navigation = types.SimpleNamespace(cancel_goal=lambda: cancel.append(1) or True)
    msg = types.SimpleNamespace(position=types.SimpleNamespace(x=0.0, y=0.0, z=0.3),
                                orientation=types.SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))
    import houndkeeper.tidy_module as tm
    real = tm.clock.now; tm.clock.now = clock
    try: TidyArm._on_odom(t, msg)
    finally: tm.clock.now = real
    assert cancel and pub and pub[-1].linear.x == 0.0


def test_the_arm_up_sweep_is_refused():                                     # [G48]
    from houndkeeper.tidy_module import TidyArm
    t = TidyArm.__new__(TidyArm); t._status = {"state": "x", "log": []}; t.SWEEP_ARM = "look"
    t._split_inside = lambda r, inset=0.0: (r, []); t._say = lambda s: None
    t.CANDIDATE_CHECK = False
    with pytest.raises(RuntimeError, match="G48"): TidyArm._sweep_and_pick(t, {"route": [[0.0, 0.0]]})


def test_dimos_robot_refuses_a_whole_move_before_it_starts(tmp_path):       # [qpO]: never halfway
    from houndkeeper.dimos_robot import DimosRobot
    g, ga, arm, clock = _guard(tmp_path, reach=lambda q: float(abs(q[0]))); _still(g, clock)
    S = DimosRobot.__new__(DimosRobot); S.adapter = ga; S._g_lo, S._g_hi = 0.0, 0.0
    S.K = types.SimpleNamespace(ARM_VMAX=1.0); S._cmd = STOWED.copy(); S.sync = lambda: None
    far = np.zeros(6); far[0] = 0.9                                          # reach 0.9 m: past the 0.6
    with pytest.raises(ag.ArmRefused): S.arm_to(far, 0.5)
    assert arm.sent == [] and S.refuses(far) is not None and S.refuses(np.zeros(6)) is None


def test_the_robots_arm_model_measures_reach_from_the_mount():              # [qpO]: the envelope's r
    pytest.importorskip("mujoco")
    k = ag.ArmKinematics()
    assert k.reach_m(np.zeros(6)) < 0.25 and k.reach_m(OUT[:6]) > 0.3


def test_the_arm_model_is_built_when_the_arm_is_attached_not_at_the_first_check(monkeypatch):   # [G48], [qpO]
    """built at the first lying check it stalled the tidy for seconds and odometry piled up (18 Oct): built with the arm"""
    made = []
    class _K:
        def __init__(self): made.append(1)
        def reach_m(self, q): return 0.0
    monkeypatch.setattr(ag, "ArmKinematics", _K)
    g = ag.ArmGuard(_Clock(), envelope=ag.Envelope(), posture="stand"); assert made == []   # the dog alone loads nothing
    ag.GuardedArm(_Arm(), g); assert made == [1]
    g.posture = "lie"; g.reach_m(OUT[:6]); assert made == [1]           # no check builds it again
