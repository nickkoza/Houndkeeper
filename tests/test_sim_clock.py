"""[3Pv]: the sim behaves the same at any speed. Every message the robot reads is held until the sim's clock reaches its
capture time plus its delay and stamped on a robot clock that advances with sim time; the robot's side reads that clock.
Everything here runs on a fake sim clock and a fake wall clock: no sim, no GPU."""
import numpy as np
import pytest

from dimos.simulation.mujoco.delay_line import DelayLine
from dimos.simulation.mujoco.mujoco_process import MockController
from dimos.simulation.mujoco.odom_delay import OdomDelay
from dimos.simulation.mujoco.stream_model import StreamModel
from dimos.utils import clock

LOOP_S = 0.035          # the DimOS sim's loop: 7 physics steps of 5 ms of sim time
FRAME_S = 0.05          # 20 frames a sim second (rendered at the first loop past each)
POLL_S = 0.005          # the connection polls its shared memory at 200 Hz of wall time


class Wall:
    def __init__(self, t0=1000.0): self.t = t0
    def __call__(self): return self.t


@pytest.fixture
def fake_wall(monkeypatch, tmp_path):
    w = Wall(); monkeypatch.setattr(clock, "_wall", w); monkeypatch.setattr(clock, "_reader", clock._Reader())
    monkeypatch.setenv(clock.ENV, str(tmp_path / "clock")); clock.set_source(None)
    return w


def test_a_message_is_held_until_the_sim_clock_reaches_capture_plus_delay():
    """[3Pv]"""
    line = DelayLine(0.15, 0.0)
    assert line.push(10.0, "frame") == pytest.approx(10.15)
    assert line.due(10.149) == [] and len(line) == 1                    # not readable early, with a future stamp
    (arrival, sent, msg), = line.due(10.15)
    assert (arrival, sent, msg) == (pytest.approx(10.15), 10.0, "frame")
    assert line.push(11.0, "held", through=11.6) == pytest.approx(11.75)   # held through a stall: its end plus the delay
    assert line.push(11.05, "next") >= 11.75                             # the link keeps order


def test_odometry_is_held_on_the_clock_it_is_given():
    """[3Pv]: the same robot-time delay whatever the clock's pace"""
    d = OdomDelay(0.03, 0.0, seed=1)
    assert d.push(5.0, [1, 0, 0], [1, 0, 0, 0]) is None and d.push(5.029, [2, 0, 0], [1, 0, 0, 0]) is None
    pos, _q, stamp = d.push(5.03, [3, 0, 0], [1, 0, 0, 0])
    assert pos[0] == 1 and stamp == pytest.approx(5.03)


def test_a_stall_delivers_its_held_frames_late_in_a_burst():
    """[3Pv]: a stall's frames come through at its end (a share of them), not all lost"""
    s = StreamModel("drop=0.0,stall_per_min=6,stall_s=0.6,burst_share=0.5,seed=2")
    t = np.arange(0, 600, FRAME_S); arr = [s.arrival(x) for x in t]
    late = [(x, a) for x, a in zip(t, arr) if a is not None and a > x]
    lost = sum(a is None for a in arr)
    assert late and lost                                                 # some held, some lost
    assert 0.3 < len(late) / (len(late) + lost) < 0.7                    # about half of a stall's frames
    assert all(a - x < 5.0 for x, a in late)
    assert StreamModel("drop=0.0,stall_per_min=6,stall_s=0.6,seed=2").arrival(0.0) in (0.0, None)   # no burst_share: lost


def test_stamps_follow_sim_time_at_any_speed(fake_wall):
    """[3Pv]: the published clock is the wall time at the first tick plus the sim time since, whatever the ratio; a
    reset never turns it back"""
    for ratio in (0.16, 1.0, 2.0):
        fake_wall.t = 1000.0; clock._reader = clock._Reader()
        sc = clock.SimClock(clock.clock_path())
        for k in range(200):
            sim_t = k * LOOP_S; fake_wall.t = 1000.0 + sim_t / ratio
            rt = sc.tick(sim_t)
            assert rt == pytest.approx(1000.0 + sim_t) and clock.now() == pytest.approx(rt)
        assert sc.ratio == pytest.approx(ratio, rel=0.05)
        before = clock.now(); fake_wall.t += 0.1; assert sc.tick(0.0) == pytest.approx(before)   # sim time turned back
        sc.close()


def test_without_a_sim_clock_it_is_the_wall_clock(fake_wall, monkeypatch):
    """[3Pv]: on the robot, the robot's clock is the host's"""
    monkeypatch.delenv(clock.ENV); fake_wall.t = 1234.5
    assert clock.now() == 1234.5 and clock.SimClock.from_env().tick(99.0) == 1234.5


def test_a_command_takes_effect_after_its_delay_on_the_sim_clock():
    """[3Pv]: commands reach the robot late (cmd_delay_s), in sim time"""
    class Shm:
        cmd = None
        def read_command(self):
            c, self.cmd = self.cmd, None; return c
    shm = Shm(); t = [0.0]
    mc = MockController(shm, delay=DelayLine(0.05, 0.0), clock=lambda: t[0])
    shm.cmd = (np.array([0.5, 0, 0]), np.zeros(3)); assert mc.get_command()[0] == 0.0
    t[0] = 0.049; assert mc.get_command()[0] == 0.0
    t[0] = 0.05; assert mc.get_command()[0] == pytest.approx(0.5)


def _night(ratio, wall, follow_sim=True, seconds=6.0):
    """A fake sim at `ratio` x real time and the robot's side reading it, in one thread. The sim: 35 ms loops; the
    robot walks at the commanded speed; a frame every 50 ms of sim time showing where the robot truly was, held by the
    frame delay and a stalling stream; odometry each loop, held by its delay; commands held by theirs. The robot's
    side polls the single-slot shared memory every 5 ms of wall time and runs a small controller on the robot's clock:
    walk at 0.5 m/s; stop once a frame shows it past 1.0 m; and if no frame has come for 0.3 s, a stall: count it.
    Returns what the robot's side did, in robot-clock terms."""
    sc = clock.SimClock(clock.clock_path() if follow_sim else None, follow_sim=follow_sim)
    frames, odom = DelayLine(0.15, 0.03, seed=4), OdomDelay(0.03, 0.01, seed=5)
    stream = StreamModel("drop=0.03,stall_per_min=20,stall_s=0.5,burst_share=0.5,seed=6")
    cmd_slot = {"cmd": None}

    class Shm:
        def read_command(self):
            c, cmd_slot["cmd"] = cmd_slot["cmd"], None; return c
    sim_t = 0.0; ctl = MockController(Shm(), delay=DelayLine(0.05, 0.02, seed=7))
    ctl.clock = lambda: sc.at(sim_t)
    slots = {"frame": (0, None), "odom": (0, None)}
    x = 0.0; last_render = -1.0; t0 = wall.t; sc.tick(sim_t)
    seen = {"frame": 0, "odom": 0}; out = {"stamps": [], "stop_at": None, "stop_x": None, "stalls": 0, "odom_n": 0}
    cmd_slot["cmd"] = (np.array([0.5, 0, 0]), np.zeros(3)); last_frame_t = clock.now()
    while sim_t < seconds:
        for _ in range(7):                                               # physics: the policy reads its command each step
            x += float(ctl.get_command()[0]) * 0.005; sim_t += 0.005
        wall.t = t0 + sim_t / ratio
        rt = sc.tick(sim_t)
        got = odom.push(rt, [x, 0, 0], [1, 0, 0, 0])
        if got is not None: slots["odom"] = (slots["odom"][0] + 1, got)
        if not 0 <= sim_t - last_render < FRAME_S:
            last_render = sim_t; through = stream.arrival(rt)
            if through is not None: frames.push(rt, x, through=through)
        for arrival, _sent, shows in frames.due(rt, limit=1):
            slots["frame"] = (slots["frame"][0] + 1, (arrival, shows))
        next_loop = t0 + (sim_t + LOOP_S) / ratio                        # the robot's side, until the next loop
        poll = wall.t
        while poll < next_loop:
            wall.t = poll; now = clock.now() if follow_sim else wall.t
            if slots["odom"][0] > seen["odom"]: seen["odom"] = slots["odom"][0]; out["odom_n"] += 1
            if slots["frame"][0] > seen["frame"]:
                seen["frame"] = slots["frame"][0]; stamp, shows = slots["frame"][1]
                out["stamps"].append(round(stamp - 1000.0, 6)); last_frame_t = now
                if shows > 1.0 and out["stop_at"] is None:
                    out["stop_at"] = round(now - 1000.0, 6); cmd_slot["cmd"] = (np.zeros(3), np.zeros(3))
            elif now - last_frame_t > 0.3:
                out["stalls"] += 1; last_frame_t = now
            poll += POLL_S
    out["stop_x"] = round(x, 6); sc.close()
    return out


def test_the_same_night_at_two_speeds_gives_the_same_outcome(fake_wall):
    """[3Pv]: speed is a cost, never a variable: the same message sequence at 0.16x and at 2x real time gives the
    robot's side identical stamps, decisions and stopping point"""
    runs = []
    for ratio in (0.16, 0.5, 1.0, 2.0):
        fake_wall.t = 1000.0; clock._reader = clock._Reader(); runs.append(_night(ratio, fake_wall))
    assert runs[0]["stop_at"] is not None and runs[0]["stalls"] > 0 and len(runs[0]["stamps"]) > 50
    for r in runs[1:]:
        assert r == runs[0]


def test_on_the_wall_clock_the_outcome_depended_on_speed(fake_wall, monkeypatch):
    """[3Pv]: the check above can fail: held and stamped on the wall clock (DimOS's sim before), the slow night's
    delays cost less sim time, so it stopped sooner"""
    monkeypatch.delenv(clock.ENV)
    fake_wall.t = 1000.0; slow = _night(0.16, fake_wall, follow_sim=False)
    fake_wall.t = 1000.0; real = _night(1.0, fake_wall, follow_sim=False)
    assert slow["stop_x"] < real["stop_x"] - 0.03
