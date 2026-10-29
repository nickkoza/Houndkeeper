"""House 70's carpet (sim-house-nights [jmf]): the low-pile photo it is said to be, laid in house 70's carpet hue
(HOUSE70_CARPET_SRGB), its brightness the photo's own."""
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "sim" / "arm"))


def test_house_70_lays_its_carpet_in_its_hue():
    import make_house as mh, make_nights as mn, floors
    mh.use_layout(70); mn._BASE.clear()
    col, _, _, carpet, name = mn.photo_base(70)
    assert name.endswith("+" + mn.LOW_PILE_CARPET)
    got = col[carpet].mean(0, dtype=np.float64); want = np.array(mn.HOUSE70_CARPET_SRGB)
    assert np.allclose(got / got.mean(), want / want.mean(), atol=0.01), got      # house 70's carpet hue
    photo = floors._photo_maps(mn.LOW_PILE_CARPET)[0].reshape(-1, 3).mean(0, dtype=np.float64)
    assert abs(got.mean() - photo.mean()) < 0.03                                  # the low-pile photo's brightness
    dark = floors._photo_maps("Carpet012")[0].reshape(-1, 3).mean(0, dtype=np.float64)
    assert got.mean() > dark.mean() + 0.2                                          # not the dark cut pile it had kept
