"""[TF8]: the floor isn't one flat plane ([x3f]): the carpet stands above the hard floor, the rug on it, and the rug
stays where it lies night to night ([yyJ]); the raised parts show exactly the floor texture beneath them."""
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house"))


def test_the_rug_stays_where_it_lies_each_night():
    """[yyJ]: the rug no longer moves between nights"""
    import make_nights as mn
    rug = {"id": "rug", "x": -4.6, "y": -0.2, "w": 2.2, "h": 1.5}
    assert all(mn.rug_tonight(rug, 70, n) == rug for n in range(1, 9))


def test_a_slab_is_textured_by_where_it_lies(tmp_path):
    """its texture coordinates are the floor plane's at the same x, y: the top shows the floor beneath, raised"""
    import make_nights as mn
    f = tmp_path / "s.obj"; corners = [(-1.0, -0.5), (1.0, -0.5), (1.0, 0.5), (-1.0, 0.5)]
    mn.slab_obj(f, corners, 0.0, mn.CARPET_M)
    lines = f.read_text().splitlines()
    v = np.array([[float(x) for x in l.split()[1:]] for l in lines if l.startswith("v ")])
    vt = np.array([[float(x) for x in l.split()[1:]] for l in lines if l.startswith("vt ")])
    assert np.allclose(sorted(set(v[:, 2])), [0.0, mn.CARPET_M])
    for (x, y), (u, w) in zip(corners, vt):
        assert np.isclose(u, (x + mn.HX) / (2 * mn.HX)) and np.isclose(w, (y + mn.HY) / (2 * mn.HY))


def test_the_nightly_runs_build_raised_floors():
    sh = (HK / "sim" / "dimos" / "night_tidy.sh").read_text()
    assert '[ "${RELIEF:-1}" = 1 ] || echo --flat' in sh


def test_the_main_tuning_house_has_complex_high_contrast_tile():
    """house 70 has carpet and a diagonal slate-look tile, house 71 a patterned high-contrast
    tile; the others keep theirs"""
    import make_nights as mn
    sys.path.insert(0, str(HK / "sim" / "arm")); import floors
    assert mn.photo_base(70)[4] == "slate_tile+" + mn.LOW_PILE_CARPET              # slate-look tile, low pile
    assert mn.carpet_m(70) < mn.carpet_m(42)
    assert mn.photo_base(71)[4].split("+")[0] in {r["id"] for r in floors.photos("train", ["tile_pattern"])}
    assert mn.photo_base(42)[4] == "WoodFloor048+Carpet016"


def test_test_nights_are_lit_the_same_every_night():
    """[72t]: testing runs with the testing light on: the ceiling light and the lamp on, the same every night (dim
    nights at high gain were grainy, and a lamp on one night and off the next changed the whole floor's look)"""
    import make_nights as mn
    ls = [mn.regime(70, n, "lit") for n in range(1, 6)]
    assert all(l["lamp_on"] and l["ceiling"] == ls[0]["ceiling"] and l["ambient"] == ls[0]["ambient"] for l in ls)
    assert '${REGIME-lit}' in (HK / "sim" / "dimos" / "night_tidy.sh").read_text()
