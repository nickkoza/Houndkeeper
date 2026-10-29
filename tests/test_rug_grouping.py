"""[6Pn]: a new patterned rug is one floor change, not many finds. Blobs within group_m of each other are judged
floor-sized together (a rug broke into toy-sized blobs: 17 stops for nothing on tuning night 70/2, 26 Sep 2026);
a lone toy-sized blob is still a candidate."""
import argparse, sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "house"))
cs = pytest.importorskip("confirm_stream")


def _confirmer(group_m):
    c = cs.Confirmer.__new__(cs.Confirmer)
    c.a = argparse.Namespace(**{**cs.CONFIRM, "group_m": group_m})
    c.res, c.k = cs.RES, 1                     # the 1 cm grid ([Ot0]: the grid is a setting of the comparison)
    c.hit = np.zeros((cs.H, cs.W), np.int16); c.hit_b = np.zeros((cs.NB, cs.H, cs.W), np.int16)
    c.seen_b = np.zeros((cs.NB, cs.H, cs.W), np.int16)
    def mark(r0, c0, n):
        c.hit[r0:r0 + n, c0:c0 + n] = 5; c.hit_b[0, r0:r0 + n, c0:c0 + n] = 5; c.seen_b[0, r0:r0 + n, c0:c0 + n] = 5
    for i in range(10):                        # a rug: 10 x 10 patches of 8 cm, 6 cm apart (1.4 m square, 0.64 m2 changed)
        for j in range(10): mark(100 + 14 * i, 100 + 14 * j, 8)
    mark(400, 600, 8)                          # a toy, 2.5 m from the rug
    return c


def _floor_sized(c):
    return {(round(x, 1), round(y, 1)): group > cs.BIG_PX for _, _, area, x, y, _, group in c.spots()}


def test_rug_fragments_group_into_one_floor_change():
    spots = _floor_sized(_confirmer(0.08))
    assert sum(spots.values()) == 100          # every rug patch judged part of a floor-sized change
    assert list(spots.values()).count(False) == 1   # the toy alone is not


def test_without_grouping_each_fragment_looks_like_a_toy():
    assert not any(_floor_sized(_confirmer(0.0)).values())
