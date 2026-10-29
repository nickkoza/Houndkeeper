"""[WkC]: the robot never walks or changes posture with the arm out of the stow. A move the arm guard refuses partway
holds the arm where it stopped; once the base reads still the stow is tried again; still refused after a bounded number
of tries, the night stops where it stands with the arm out filed for review, rather than failing with the arm out."""
import sys
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from houndkeeper import arm_guard as ag  # noqa: E402

OUT = np.array([0.0, -2.95, 1.3, 1.2, 0.0, 0.0])


class _Clock:
    def __init__(self): self.t = 100.0
    def now(self): return self.t
    def sleep(self, dt): self.t += dt


@pytest.fixture
def clk(monkeypatch):
    import houndkeeper.tidy_module as tm
    c = _Clock(); monkeypatch.setattr(tm.clock, "now", c.now); monkeypatch.setattr(tm.clock, "sleep", c.sleep)
    return c


class _Arm:
    """an arm whose moves the guard refuses for the first `refuse` tries, partway: half done, then held there"""
    def __init__(self, q, refuse):
        self.q = np.asarray(q, float).copy(); self.refuse = refuse; self.tries = 0; self.sent = []

    def move(self, target):
        self.tries += 1; target = np.asarray(target, float)
        if self.tries <= self.refuse:
            self.q = self.q + 0.5 * (target - self.q); self.sent.append(self.q.copy())
            raise ag.ArmRefused("the base is moving (0.00 m/s, 0.14 rad/s) ([G48], [FWp])")
        self.q = target.copy(); self.sent.append(self.q.copy())


def _tidy(arm, posture="lie"):
    from houndkeeper.tidy_module import TidyArm
    t = TidyArm.__new__(TidyArm); said = []
    t._guard = ag.ArmGuard(lambda: 0.0, posture=posture, reach=lambda q: 0.0, read_q=lambda: arm.q)
    t._say = said.append; t._state = lambda *a, **k: None; t._status = {"state": "x", "log": [], "items": []}
    settled = []; t._base_settled = lambda: settled.append(arm.tries) or True
    return t, said, settled


def test_a_stow_refused_partway_is_held_then_finished_once_the_base_settles(clk):
    arm = _Arm(OUT, refuse=1); t, said, settled = _tidy(arm)
    t._stow(lambda: arm.move(np.zeros(6)))
    assert t._guard.stowed() and arm.tries == 2
    assert settled == [1]                                                # the second try only after the base settled
    assert np.allclose(arm.sent[0], OUT / 2)                             # held where the refusal left it, not snapped
    assert any("held where it is" in s for s in said)


def test_a_stow_refused_every_time_stops_the_night_with_the_arm_out(clk):
    arm = _Arm(OUT, refuse=99); t, said, settled = _tidy(arm)
    with pytest.raises(Exception) as e: t._stow(lambda: arm.move(np.zeros(6)))
    from houndkeeper.tidy_module import ArmStop
    assert isinstance(e.value, ArmStop) and arm.tries == t.STOW_TRIES   # bounded
    assert not t._guard.stowed()


def test_the_night_stops_in_place_with_the_arm_filed_not_failed(clk):
    """a refusal that ends the night is a stop where it stands ([Qd5]'s "stopped"), the arm's pose filed for review"""
    from houndkeeper.tidy_module import ArmStop, TidyArm
    for err in (ArmStop("the arm could not be stowed: the base is moving"), ag.ArmRefused("odometry 1.0 s old ([G48], [FWp])")):
        arm = _Arm(OUT, refuse=0); t, said, _ = _tidy(arm); t._status["items"] = [{"flag": [0.0, 0.0]}]
        pub = []; t.cmd_vel = types.SimpleNamespace(publish=pub.append)
        def loop(): raise err
        t._tidy_loop = loop
        TidyArm._run(t)
        s = t._status
        assert s["state"] == "stopped" and "error" not in s, s
        assert s["arm_out"]["q"] == [round(float(x), 3) for x in OUT] and s["arm_out"]["posture"] == "lie"
        assert s["items"][-1]["outcome"].startswith("review: stopped with the arm out")
        assert pub and pub[-1].linear.x == 0.0                           # the base told to stand still


def test_the_end_of_a_stop_stows_through_the_retry(clk, monkeypatch):
    """the stop's fold after a look that found nothing, refused partway once: retried, the stop finishes standing"""
    from houndkeeper.tidy_module import TidyArm
    q_ready = np.array([0.0, -1.0, 1.0, 0.5, 0.0, 0.0])
    arm = _Arm(np.zeros(6), refuse=0); t, said, settled = _tidy(arm)
    calls = []
    S = types.SimpleNamespace(max_tilt=0.0, waypoints=[], arm_to_safe=lambda q, secs, grip=None: calls.append(q) or arm.move(q))
    pick = types.SimpleNamespace(look_closer=lambda S_, f, wide_tol=None: (None, {"seen": 0}), WIDE_TOL=0.2, CLOSED=0.0,
                                 stow=lambda S_, q: S_.arm_to_safe(q, 2.0))
    t._ctx = {"pick": pick, "wr": None, "S": S, "park": np.zeros(2), "handled": [], "q_ready": q_ready, "q_bin": q_ready}
    t.STANDING_LOOK = False; t._robot = lambda: S; t._nope = lambda S_: None
    t._refine_flag = lambda f, wr: (f, "kept"); t._approach = lambda f, park, walk=True: ("reached", 0.0)
    stood = []; t._posture = types.SimpleNamespace(liedown=lambda: True, standup=lambda: stood.append(t._guard.stowed()) or True)
    real = arm.move
    def move(q):                                                         # the fold's first try refused partway
        if len(calls) == 2 and arm.refuse == 0: arm.refuse = arm.tries + 1
        return real(q)
    arm.move = move
    rep = TidyArm._handle_find(t, 1, np.array([1.0, 0.0]))
    assert rep["outcome"] == "review: not seen at the flag"
    assert stood == [True]                                               # stood up only stowed
    assert t._guard.stowed() and any("held where it is" in s_ for s_ in said)   # refused once, retried, finished


def test_a_standing_look_refused_partway_stows_before_it_walks_on(clk, monkeypatch):
    from houndkeeper.tidy_module import TidyArm
    arm = _Arm(np.zeros(6), refuse=1); t, said, settled = _tidy(arm, posture="stand")
    look = np.array([0.2, 0.0, 0.0, 0.0, 0.2, 0.1])
    S = types.SimpleNamespace(legs=None, K=None, sync=lambda: None, target=lambda: np.zeros(7), refuses=lambda q: None,
                              arm_to=lambda q, secs, grip=None: arm.move(q), waypoints=[])
    S.arm_to_safe = S.arm_to
    for name, mod in (("legs", types.SimpleNamespace(STAND="stand", LIE="lie")),
                      ("standing_look", types.SimpleNamespace(aim=lambda K, f, q0, up_deg=0: (look, 0.0, 1.0), MIN_CAM_UP=0.0)),
                      ("wrist_refine", types.SimpleNamespace())):
        monkeypatch.setitem(sys.modules, name, mod)
    t._robot = lambda: S; t._ctx = {"pick": None}
    t._pose = lambda: (np.array([0.0, 0.0, 0.3]), np.array([1.0, 0.0, 0.0, 0.0])); t._yaw = lambda: 0.0
    t._turn = lambda a, rate=0.6: None; t._go = lambda *a, **k: "reached"
    f, note = TidyArm._standing_look(t, np.array([0.75, 0.0]))
    assert "refused partway" in note and np.allclose(f, [0.75, 0.0])
    assert t._guard.stowed() and S.legs == "lie"                         # stowed again before the sweep walks on


def test_a_move_refused_partway_keeps_the_last_command_sent(monkeypatch):
    """the robot's own record of the arm's command is the last one the guard let through, where the arm holds"""
    import houndkeeper.dimos_robot as dr
    env = types.SimpleNamespace(reach_m=lambda p, kg: 1.0, speed=lambda p, kg: (1.0, "the envelope"))
    c = _Clock(); g = ag.ArmGuard(c.now, envelope=env, posture="lie", reach=lambda q: 0.0)
    for k in range(51): g.on_odom(c.t - 1.0 + k / 50, 0.0, 0.0, 0.0)
    sent = []
    inner = types.SimpleNamespace(read_joint_positions=lambda: [0.0] * 7,
                                  write_joint_positions=lambda q, *a, **k: sent.append(np.array(q)) or True)
    S = dr.DimosRobot.__new__(dr.DimosRobot); S.adapter = ag.GuardedArm(inner, g); S._g_lo, S._g_hi = 0.0, 0.0
    S.K = types.SimpleNamespace(ARM_VMAX=1.0); S._cmd = np.zeros(7); S.sync = lambda: None
    def sleep(dt):                                     # the base starts turning a third of the way through the move
        c.t += dt; g.on_odom(c.t, 0.0, 0.0, 0.5 * max(0.0, c.t - 100.3))
    monkeypatch.setattr(dr.clock, "sleep", sleep)
    with pytest.raises(ag.ArmRefused, match="moving"): S.arm_to(np.array([0.0, -0.8, 0.0, 0.0, 0.0, 0.0]), 1.0)
    assert sent and np.allclose(S.target(), sent[-1])   # held at the last sent, not the refused one


STUCK = np.array([0.097, -0.13, 0.003, 0.015, -0.001, -0.001])   # 18 Oct sim night: the fold come to rest on a binned shoe


class _BlockedArm(_Arm):
    """an arm whose stow runs, nothing refused, and comes to rest at `rest`: something is in the fold's way"""
    def __init__(self, q, rest):
        super().__init__(q, refuse=0); self.rest = np.asarray(rest, float); self.held = []

    def move(self, target):
        self.tries += 1; self.q = self.rest.copy(); self.sent.append(np.asarray(target, float))


class _SlowArm(_Arm):
    """an arm that reaches the stow, nothing in its way, but takes longer than STOW_READ_S to read stowed: still moving"""
    def __init__(self, q, clk, secs):
        super().__init__(q, refuse=0); self.clk = clk; self.secs = secs; self.t0 = None; self.q0 = self.q.copy()

    def move(self, target):
        self.tries += 1; self.target = np.asarray(target, float)
        if self.t0 is None: self.t0 = self.clk.t                         # sent again: the same motion carries on

    def read(self):
        if self.t0 is None: return self.q
        a = min(1.0, (self.clk.t - self.t0) / self.secs); self.q = self.q0 + a * (self.target - self.q0); return self.q


def test_a_blocked_stow_is_held_where_it_rests_and_stops_without_retrying(clk):
    """a stow nothing refused that comes to rest short of the stow is not pressed or repeated: the arm held where it
    rests, the night stopped saying which joints are out"""
    from houndkeeper.tidy_module import ArmStop
    arm = _BlockedArm(OUT, STUCK); t, said, settled = _tidy(arm)
    with pytest.raises(ArmStop, match="blocked") as e:
        t._stow(lambda: arm.move(np.zeros(6)), hold=lambda: arm.held.append(arm.q.copy()))
    assert arm.tries == 1                                                # the same move not tried again
    assert len(arm.held) == 1 and np.allclose(arm.held[0], STUCK)        # held where it came to rest, not pressed at the stow
    assert "joint 2" in str(e.value) and "0.13" in str(e.value)          # which joint is out, and by how much


def test_a_slow_stow_still_moving_is_not_called_blocked(clk):
    """positive control: an arm still on its way at STOW_READ_S is not blocked; it is tried again and reads stowed"""
    arm = _SlowArm(OUT, clk, secs=3.0); t, said, settled = _tidy(arm); t._guard.read_q = arm.read
    t._stow(lambda: arm.move(np.zeros(6)), hold=lambda: pytest.fail("a moving arm was held as blocked"))
    assert t._guard.stowed() and arm.tries == 2
