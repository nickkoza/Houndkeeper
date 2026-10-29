"""A night's toys: a fixed mix of tiers in proportion to the floor (sim-slice [Afy]), binning being what nights show."""
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "dimos"))


def test_the_test_house_gets_one_big_two_hard_five_easy():
    from house_toys import night_mix, toys_for_floor
    assert toys_for_floor(16.1 * 9.1) == 8
    assert night_mix(8) == {"too_big": 1, "hard": 2, "easy": 5}


def test_the_mix_scales_with_the_floor():
    from house_toys import night_mix, toys_for_floor
    assert toys_for_floor(2 * 16.1 * 9.1) == 16 and night_mix(16) == {"too_big": 2, "hard": 4, "easy": 10}
    assert toys_for_floor(40) == 4 and night_mix(4) == {"too_big": 1, "hard": 1, "easy": 2}   # a flat: still one of each


def test_a_night_is_drawn_in_its_mix_without_repeats():
    from house_toys import pick_night
    names = [f"{t}_{i}" for t in ("big", "hard", "easy") for i in range(6)]
    tier = lambda n: {"big": "too_big", "hard": "hard", "easy": "easy"}[n.split("_")[0]]
    for seed in range(20):
        night = pick_night(np.random.default_rng(seed), names, tier, 8)
        assert Counter(map(tier, night)) == {"too_big": 1, "hard": 2, "easy": 5} and len(set(night)) == 8
