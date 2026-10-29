"""[QMt]: the indoor house's floor is photographed (training floors only), with its relief layers in the scene."""
import sys
from pathlib import Path
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "sim" / "arm"))
floors = pytest.importorskip("floors")
needs_photos = pytest.mark.skipif(not floors.PHOTO_ZIPS.exists(), reason="tools/fetch_floors.sh not run")


@needs_photos
def test_house_floor_uses_training_floors_only(monkeypatch):
    import make_nights as mn
    monkeypatch.setattr(mn, "PX", 0.03)                          # coarse: the choice of floors is what's tested
    held = {r["id"] for r in floors.photos("heldout")}
    for seed in range(8):
        mn._BASE.clear()
        col, nrgb, orm, carpet, ids = mn.photo_base(seed)
        hard, carpet_id = ids.split("+")
        assert hard not in held and carpet_id not in held
        assert col.shape[:2] == nrgb.shape[:2] == orm.shape[:2] == carpet.shape
        assert 0.1 < carpet.mean() < 0.9                         # both kinds of floor are in the house


def test_scene_carries_the_relief_layers():
    import make_nights as mn
    xml = mn.floor_material(("n.png", "orm.png"))
    assert 'role="normal"' in xml and 'role="orm"' in xml and 'colorspace="linear"' in xml
    assert "layer" not in mn.floor_material(None)                # the generated floor stays as it was


def test_night_only_nights_and_a_fixed_ceiling_light():
    """[QMt]: the tidy's nights can all be lit as night (a window-lit floor against a lamp-lit one differs everywhere),
    and a house's ceiling light hangs in one place every night"""
    import make_nights as mn
    for seed in (3, 70):
        ls = [mn.regime(seed, n, "night") for n in range(1, 6)]
        assert all(l["regime"] == "night" for l in ls)
        assert len({tuple(l["ceiling_at"]) for l in ls}) == 1
    assert {mn.regime(70, n)["regime"] for n in range(1, 13)} == {"day", "night"}   # left alone, both still come up
