"""Tests written to kill mutants that survived the suite (docs/research/test-quality-2026-10-03.md): the eyes' slew limit
([GTG], principle [0Py]: nothing flashes) when the loop stalls for seconds, when the driver's clock steps back, and
when a colour is infinite."""
from arm_host.eyes import Eyes, MAX_DT, MIN_SWING_S, WS2812


def test_a_stalled_loop_catches_up_over_several_frames_not_one():
    """[GTG] Eyes.step after a 5 s stall moves no further than one longest frame allows"""
    e = Eyes(); e.set(1.0, 1.0, 1.0)
    out = e.step(5.0)
    assert max(out) <= MAX_DT / MIN_SWING_S + 1e-9


def test_a_clock_stepping_back_changes_nothing_on_the_wire():
    """[GTG] the driver's clock going backwards (a clock source swapped, a test clock) is no time at all"""
    class SPI:
        def writebytes2(self, b): pass
    clock = [100.0]; w = WS2812(count=1, clock=lambda: clock[0]); w.spi = SPI()
    w.show((0.0, 0.0, 0.0)); clock[0] = 90.0; w.show((0.0, 0.0, 0.0))
    assert w.last == (0.0, 0.0, 0.0)


def test_an_infinite_colour_is_dark():
    """[GTG] NaN or infinite reads as dark, as the module says, not as full"""
    e = Eyes(); e.set(float("inf"), float("-inf"), float("nan"))
    assert e.target == (0.0, 0.0, 0.0)
