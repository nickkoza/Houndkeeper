"""A re-park aims the grip, not the item's middle, and a grip has a real width (sim-grasp decision on re-parks)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))


def test_the_re_park_aims_the_grip_point():
    from houndkeeper.tidy_module import repark_target
    ref = {"xy": np.array([2.0, 1.0]), "grip_options": [(0.05, np.array([1.0, 0.0]), np.array([0.12, 0.0])),
                                                        (0.08, np.array([0.0, 1.0]), np.array([0.0, 0.0]))]}
    assert np.allclose(repark_target(ref), [2.12, 1.0])                     # the narrowest grip's point, 12 cm off the middle
    assert np.allclose(repark_target({"xy": np.array([2.0, 1.0])}), [2.0, 1.0])   # no options: the item, as before


def test_slivers_are_not_grips():
    import pick
    opts = [(0.001, np.array([1.0, 0.0]), np.zeros(2)), (0.004, np.array([0.0, 1.0]), np.zeros(2)),
            (0.03, np.array([1.0, 1.0]) / np.sqrt(2), np.zeros(2)), (0.2, np.array([1.0, 0.0]), np.zeros(2))]
    kept = pick.real_grips(opts)
    assert [round(o[0], 3) for o in kept] == [0.03]                          # neither a 1-4 mm sliver nor too wide
