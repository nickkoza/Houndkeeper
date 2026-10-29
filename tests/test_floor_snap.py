"""[KkQ]: a frame's floor patch snaps onto the reference where the floor has edges to lock on (tile, grout, a carpet's
edge), by exactly the offset it is off, and doesn't lock where the floor has none (plain carpet); a toy in the patch
doesn't drag the snap."""
import math
import sys
from pathlib import Path

import numpy as np
from scipy import ndimage as ndi

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from houndkeeper import floor_register as fr  # noqa: E402


def _tiles(h=220, w=220, rng=None):
    """diagonal tiles 48 cm across with 5 mm grout and mottled faces, at 1 cm cells"""
    rng = rng or np.random.default_rng(1)
    r, c = np.mgrid[:h, :w].astype(float); u, v = (r + c) / math.sqrt(2), (r - c) / math.sqrt(2)
    face = ndi.gaussian_filter(rng.normal(0, 1, (h, w)), 3) * 15 + 120 + 30 * ((u // 48 + v // 48) % 2)
    grout = (np.abs((u % 48) - 24) > 23.5) | (np.abs((v % 48) - 24) > 23.5)
    return np.where(grout, 60.0, face)


def test_the_snap_finds_how_far_the_patch_is_off():
    M = _tiles(); Mm = np.ones_like(M, bool)
    F = np.roll(M, (2, -3), (0, 1)); Fm = np.zeros_like(Mm); Fm[10:-10, 10:-10] = True
    s = fr.snap(F, Fm, M, Mm)
    assert s is not None and s[3] >= fr.SNAP_MIN_SCORE and s[4] >= fr.SNAP_MIN_LEAD
    assert abs(s[0]) < 1e-9 and (s[1], s[2]) == (-2.0, 3.0)        # M shifted by (-2, 3) lies under F


def test_a_toy_does_not_drag_the_snap():
    M = _tiles(); Mm = np.ones_like(M, bool)
    F = np.roll(M, (1, 2), (0, 1)).copy(); F[100:130, 90:125] = 240.0     # a bright toy, ~30 x 35 cm
    Fm = np.zeros_like(Mm); Fm[10:-10, 10:-10] = True
    s = fr.snap(F, Fm, M, Mm)
    assert (s[1], s[2]) == (-1.0, -2.0)


def test_plain_carpet_does_not_lock():
    rng = np.random.default_rng(2)
    M = ndi.gaussian_filter(rng.normal(0, 1, (220, 220)), 1) * 6 + 90      # pile: fine, low-contrast, and different
    F = ndi.gaussian_filter(rng.normal(0, 1, (220, 220)), 1) * 6 + 90      # each night
    m = np.ones_like(M, bool); s = fr.snap(F, m, M, m)
    assert s is None or s[3] < fr.SNAP_MIN_SCORE or s[4] < fr.SNAP_MIN_LEAD
