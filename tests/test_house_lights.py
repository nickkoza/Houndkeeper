"""[QMt]: every light in a house has its real size, for the renderer's own soft shadows, and indirect light is simulated
(the renderer's native soft shadows, and bounce light)."""
import re
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house"))


def _lights(xml):
    return [dict(re.findall(r'(\w+)="([^"]*)"', l)) for l in re.findall(r"<light [^>]*>", xml)]


def test_night_and_day_lights_have_sizes_and_bounce():
    import make_nights as mn, make_house as mh
    mh.use_layout(71)
    night = mn.regime(71, 2, "night"); night.update(tint="bulb", lamp_on=True)
    day = {"regime": "day", "window": 0.8, "ambient": 0.3, "tint": "neutral", "lamp_on": False, "sun_in": True,
           "sun": 0.9, "sun_elev": 40.0, "sun_turn": 0.0, "sun_window": 0}
    windows = [(-5.775, -4.725, 0.0, 1.0), (-8.225, 1.225, 1.0, 0.0)]
    for light in (night, day):
        L = _lights(mn.lights_xml(light, (-6.0, -2.0), windows))
        spots = [l for l in L if l.get("directional") != "true"]
        assert spots and all(float(l.get("bulbradius", 0)) >= 0.04 for l in spots), [l["name"] for l in spots if "bulbradius" not in l]
        assert any("bounce" in l["name"] for l in L)
        assert all(l.get("castshadow") != "false" for l in L)      # a shadowless light misbehaved in mujofil (30 Sep)
