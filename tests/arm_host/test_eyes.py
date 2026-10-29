"""[GTG] the robot's eyes never flash ([0Py]): whatever they are told, start-up and faults included, no step change, and
a full swing from dark to full takes at least 2 seconds."""
import random

import pytest

from arm_host.eyes import Eyes, MIN_SWING_S

DT = 1 / 60


def frames(eyes, seconds):
    return [eyes.step(DT) for _ in range(int(round(seconds / DT)))]


def biggest_step(seq, start):
    prev, worst = start, 0.0
    for f in seq:
        worst = max(worst, max(abs(a - b) for a, b in zip(f, prev))); prev = f
    return worst


def test_minimum_swing_is_two_seconds():
    assert MIN_SWING_S >= 2.0


def test_starts_dark_and_fades_in():
    e = Eyes(); assert e.output == (0.0, 0.0, 0.0)
    e.set(1.0, 1.0, 1.0)
    out = frames(e, 1.0)
    assert max(max(f) for f in out) < 0.75                       # not there within a second
    assert biggest_step(out, (0.0, 0.0, 0.0)) <= DT / MIN_SWING_S + 1e-9


def test_full_swing_takes_two_seconds():
    e = Eyes(); e.set(1.0, 1.0, 1.0)
    out = frames(e, MIN_SWING_S - 0.1)
    assert max(max(f) for f in out) < 1.0


@pytest.mark.parametrize("seed", range(20))
def test_no_command_sequence_makes_a_step(seed):
    rng = random.Random(seed); e = Eyes(); prev = e.output; worst = 0.0
    for _ in range(2000):
        if rng.random() < 0.3:                                   # hammered with commands, colours, nonsense values
            e.set(rng.uniform(-5, 5), rng.uniform(-5, 5), rng.uniform(-5, 5))
        if rng.random() < 0.05: e.fault()
        f = e.step(DT * rng.uniform(0.0, 3.0))                   # and uneven frame times
        assert all(0.0 <= c <= 1.0 for c in f)
        dt_used = e.last_dt
        worst = max(worst, max(abs(a - b) for a, b in zip(f, prev)) / max(dt_used, 1e-9)); prev = f
    assert worst <= 1.0 / MIN_SWING_S + 1e-6


def test_fault_fades_out_not_off():
    e = Eyes(); e.set(1.0, 0.6, 0.2); frames(e, 3.0)
    e.fault()
    out = frames(e, 0.5)
    assert biggest_step(out, (1.0, 0.6, 0.2)) <= DT / MIN_SWING_S + 1e-9
    assert out[-1][0] > 0.5                                       # still fading after half a second


def test_breathing_is_slow():
    e = Eyes(); e.breathe((1.0, 0.55, 0.15), low=0.2, high=0.8, period_s=1.0)   # asked for a fast breath
    assert e.breath_period_s >= 4.0


# ---- red team, 3 Oct 2026 (docs/research/redteam-arm-host-2026-10-03.md) -----------------------------------------------

class FakeSPI:
    def __init__(self): self.frames = []
    def writebytes2(self, b): self.frames.append(bytes(b))


def decode(frame, count=1):
    """the LED bytes back out of an SPI frame (three SPI bits per data bit, after the leading low)"""
    from arm_host.eyes import WS2812
    data = frame[WS2812.LEAD_BYTES:WS2812.LEAD_BYTES + 9 * count]
    bits = int.from_bytes(data, "big"); n = len(data) * 8 // 3; out = []
    for i in range(n):
        out.append(1 if (bits >> (3 * (n - 1 - i))) & 0b111 == 0b110 else 0)
    vals = [int("".join(map(str, out[k:k + 8])), 2) for k in range(0, len(out), 8)]
    return [(vals[i + 1], vals[i], vals[i + 2]) for i in range(0, len(vals), 3)]      # g, r, b on the wire


def driver(tmp_path=None, count=1):
    from arm_host.eyes import WS2812
    clock = [0.0]
    w = WS2812(count=count, state_path=(tmp_path / "eyes.json") if tmp_path else None, clock=lambda: clock[0])
    w.spi = FakeSPI()
    return w, clock


@pytest.mark.parametrize("period", [float("nan"), "nan", float("inf"), -1.0, 0.0])
def test_a_nonsense_breath_period_never_leaves_0_to_1(period):
    """[GTG] a breath period of NaN made the output climb past full forever; on the wire the byte wrapped round, a
    bright-to-dark jump several times a second (a strobe)"""
    e = Eyes(); e.breathe((1.0, 1.0, 1.0), low=0.0, high=1.0, period_s=period)
    for _ in range(60 * 20):
        f = e.step(DT); assert all(0.0 <= c <= 1.0 for c in f)
    assert e.breath_period_s >= 4.0


@pytest.mark.parametrize("colour", [(1.0, 1.0), (1.0,), (), (1.0, 1.0, 1.0, 1.0)])
def test_a_colour_without_three_channels_is_refused(colour):
    e = Eyes()
    with pytest.raises(ValueError): e.breathe(colour)
    assert len(e.step(DT)) == 3


def test_the_led_driver_slew_limits_itself():
    """[GTG] at the lowest layer: whatever show() is handed, the wire changes no faster than the limit"""
    w, clock = driver()
    w.show((0.0, 0.0, 0.0))
    clock[0] += DT; w.show((1.0, 1.0, 1.0))                       # a caller that skipped the Eyes
    assert max(w.last) <= DT / MIN_SWING_S + 1e-9
    clock[0] += 5.0; w.show((1.0, 1.0, 1.0))                      # a stalled loop catches up in steps, not at once
    assert max(w.last) <= (DT + 0.25) / MIN_SWING_S + 1e-9


def test_the_led_driver_clamps_and_never_wraps():
    w, clock = driver()
    for _ in range(60 * 10):
        clock[0] += DT; w.show((50.0, float("nan"), -3.0))
    r, g, b = decode(w.spi.frames[-1])[0]
    assert r == 255 and g == 0 and b == 0
    prev = 0
    for fr in w.spi.frames:
        v = decode(fr)[0][0]; assert v >= prev; prev = v               # only ever up, toward full, never round


def test_a_restarted_driver_carries_on_from_where_the_lights_are(tmp_path):
    """[GTG] a crash leaves the LEDs latched where they were; the next process must fade from there, not jump to dark"""
    w, clock = driver(tmp_path)
    for _ in range(60 * 4):
        clock[0] += DT; w.show((0.8, 0.8, 0.8))
    w2, clock2 = driver(tmp_path)
    w2.show((0.0, 0.0, 0.0))
    assert abs(w2.last[0] - 0.8) < 1e-6                              # the first frame is where the lights were
    clock2[0] += DT; w2.show((0.0, 0.0, 0.0))
    assert 0.8 - w2.last[0] <= DT / MIN_SWING_S + 1e-9


def test_the_latch_gap_suits_every_ws2812():
    """the low after the data must outlast the longest reset time (280 us, WS2812B V5) at 2.4 MHz"""
    from arm_host.eyes import WS2812
    frame = WS2812.frame([(255, 255, 255)])
    trail = len(frame) - WS2812.LEAD_BYTES - 9
    assert trail * 8 / 2.4e6 >= 300e-6 and WS2812.LEAD_BYTES * 8 / 2.4e6 >= 50e-6
    assert WS2812.LEAD_BYTES + 9 * 2 <= 64                           # two LEDs' data inside the SPI FIFO: no gaps in it


def test_breathing_while_commands_arrive_never_crashes_the_loop():
    """a /eyes call between the breath check and its use left the loop unpacking None (the eye thread died)"""
    import threading
    e = Eyes(); stop = threading.Event(); errors = []

    def hammer():
        while not stop.is_set():
            e.breathe((1.0, 0.5, 0.2)); e.set(0.1, 0.1, 0.1)
    t = threading.Thread(target=hammer); t.start()
    try:
        for _ in range(20000):
            try: e.step(DT)
            except Exception as x: errors.append(x); break
    finally:
        stop.set(); t.join()
    assert not errors
