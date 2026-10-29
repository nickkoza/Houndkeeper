"""[4WJ] the host's report: the Pi's under-voltage and throttling flags read right (vcgencmd get_throttled's bits)."""
from arm_host.health import throttle_flags, report


def test_clear():
    f = throttle_flags("throttled=0x0")
    assert not any(f.values())


def test_under_voltage_now_and_since_boot():
    f = throttle_flags("throttled=0x50005")
    assert f["under_voltage_now"] and f["throttled_now"] and f["under_voltage_since_boot"] and f["throttled_since_boot"]
    assert not f["arm_freq_capped_now"] and not f["soft_temp_limit_now"]


def test_report_has_what_the_spec_asks():
    r = report(release="7", services={"hostd": "active"}, cpu_temp_c=51.2, throttled="throttled=0x0", arm="absent",
               speaker="connected")
    for k in ("release", "services", "cpu_temp_c", "throttle", "arm", "speaker", "time"):
        assert k in r


def test_core_clock_fixed_only_when_min_equals_max():
    """[GTG] the eyes' SPI timing follows the core clock: driven only when that clock is pinned"""
    from arm_host.health import core_clock_fixed
    assert core_clock_fixed("core_freq=250\ncore_freq_min=250\n")
    assert not core_clock_fixed("core_freq=400\ncore_freq_min=250\n")          # a Pi 3 left as shipped
    assert not core_clock_fixed(None) and not core_clock_fixed("") and not core_clock_fixed("error=2")
