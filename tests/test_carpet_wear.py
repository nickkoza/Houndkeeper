"""[bP1]: carpet worn along walked paths: on the carpet only, in a few smooth bands, the same every night."""
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "sim" / "arm"))


def test_wear_lies_on_carpet_in_smooth_bands_and_is_the_houses_own():
    import make_nights as mn
    carpet = np.zeros((400, 600), bool); carpet[50:350, 100:500] = True
    w1 = mn.wear_paths(70, carpet, px=0.01); w2 = mn.wear_paths(70, carpet, px=0.01)
    assert np.array_equal(w1, w2)                                   # the house's own: the same every night
    assert w1[~carpet].max() == 0 and w1[carpet].max() > 0.8        # on the carpet only, and some of it well worn
    worn = (w1 > 0.3)[carpet].mean(); assert 0.05 < worn < 0.5     # a few bands, not the whole carpet
    inner = carpet[:, 1:] & carpet[:, :-1]                          # smooth inside the carpet (it stops at the carpet's edge)
    assert np.abs(np.diff(w1, axis=1))[inner].max() < 0.2
    assert not np.array_equal(w1, mn.wear_paths(71, carpet, px=0.01))   # each house walks its own way
