"""With the monitor off, the viewer's overlays are left alone (sim-slice [5AP]): drawing them waits on the viewer's own
lock, and with the screen asleep that wait took the sim from 0.47x to under 0.37x real time (28 Sep, 12:44 PT, the hook's
share of the loop jumping from 4% to most of it). The battery and power model still advance every frame."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


def test_the_monitor_state_is_read_from_xset():
    from houndkeeper import sim_hook as sh
    assert sh.monitor_on("  DPMS is Enabled\n  Monitor is On\n") is True
    assert sh.monitor_on("  DPMS is Enabled\n  Monitor is Off\n") is False
    assert sh.monitor_on("  Monitor is in Standby\n") is False
    assert sh.monitor_on("") is True                                   # can't tell: draw, as before


def test_asleep_means_no_overlay_calls(monkeypatch):
    from houndkeeper import sim_hook as sh
    calls = []
    class V:
        def set_images(self, *a): calls.append("images")
        def set_figures(self, *a): calls.append("figures")
        def set_texts(self, *a): calls.append("texts")
    monkeypatch.setattr(sh, "display_awake", lambda: False)
    assert sh.draw_overlays(V(), lambda v: (v.set_figures(), v.set_texts())) is False and calls == []
    monkeypatch.setattr(sh, "display_awake", lambda: True)
    assert sh.draw_overlays(V(), lambda v: (v.set_figures(), v.set_texts())) is True and calls == ["figures", "texts"]


def test_the_energy_used_is_written_for_the_report(tmp_path):
    """the night's energy per pack reaches the run report (sim-slice [bOu]); it was on the HUD only"""
    import json
    from houndkeeper import sim_hook as sh
    class H:
        used_air, air_Wh, used_arm, arm_Wh = 12.5, 250.0, 0.0, None
    p = tmp_path / "energy.json"; sh.write_energy(H(), 321.0, str(p))
    e = json.loads(p.read_text())
    assert e == {"t": 321.0, "air_used_Wh": 12.5, "air_Wh": 250.0, "air_left_pct": 95.0, "arm_used_Wh": 0.0, "arm_Wh": None}


def test_a_headless_run_draws_nothing_but_still_counts_energy(monkeypatch):
    """headless (the hold-out, sim-slice [bnO]) the sim still calls the hook, so the battery model runs and the energy
    reaches the score; there is no window, so nothing is drawn"""
    from houndkeeper import sim_hook as sh
    class Headless:                                                  # as the fork's _HeadlessViewer: no drawing methods
        cam = None
    monkeypatch.setattr(sh, "display_awake", lambda: True)
    assert sh.draw_overlays(Headless(), lambda v: v.set_texts()) is False
