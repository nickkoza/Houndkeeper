"""The wrist's extra hardware is on the battery (waymarks 0011 [A8K], 0010 [J7G]: the
battery budget counts the Pi 3B's added processing and the added camera)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))


def test_the_second_camera_all_night_and_the_look_loads_only_while_looking(monkeypatch):
    import go1_arm, hud
    monkeypatch.setattr(go1_arm, "WRIST_STEREO", True); monkeypatch.setattr(go1_arm, "WRIST_LIGHT", True)
    h = hud.Hud(); p = h.p
    h.state = "SWEEP"; assert h.extra_W() == p.stereo_camera_W
    h.state = "LOOK CLOSER: find and name"; assert h.extra_W() == p.stereo_camera_W + p.look_host_extra_W + p.wrist_light_W
    monkeypatch.setattr(go1_arm, "WRIST_STEREO", False); monkeypatch.setattr(go1_arm, "WRIST_LIGHT", False)
    assert h.extra_W() == 0.0                                                  # the stock arm: nothing added
