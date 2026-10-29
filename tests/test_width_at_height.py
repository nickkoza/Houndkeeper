"""[pYZ]'s close look: a find measured too wide on its mask laid flat gets a second measure at the height the looks
triangulated it at, and is only too wide if both say so (28 Sep: standing toys, grippable by their tiers, refused)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))
import pick  # noqa: E402


def _look(half_px):
    """a camera 40 cm up looking straight down, seeing a square blob half_px pixels from its centre"""
    m = np.zeros((360, 640), bool); m[180 - half_px:180 + half_px, 320 - half_px:320 + half_px] = True
    return {"mask": m, "_cv": {"c": np.array([0.0, 0.0, 0.40]), "R": np.eye(3), "fovy": 60.0}}


def test_a_standing_toy_too_wide_flat_fits_at_its_height():
    # 50 px half-width at 60 deg fovy: 12.3 cm wide laid on the floor from 40 cm up; at its 15 cm height, 8.0 cm
    ref = {"fits_jaws": False, "z_est": 0.15, "grip_width_cm": 12.3}
    out, note = pick._width_at_height(ref, [_look(50), _look(50)])
    assert out["fits_jaws"] and out["grip_width_cm"] < pick.JAW_MAX * 100 and "fits at its height" in note


def test_a_wide_thing_stays_too_wide():
    ref = {"fits_jaws": False, "z_est": 0.03, "grip_width_cm": 25.0}
    out, note = pick._width_at_height(ref, [_look(170)])
    assert not out["fits_jaws"] and "too wide at its height too" in note


def test_a_toy_with_its_legs_apart_fits_by_one_leg():
    """two parallel legs 5 cm wide, 3 cm apart (a finger fits between), 20 cm long: every strip through the middle
    spans both (13 cm), one leg is 5 cm"""
    from wrist_refine import part_grip_options
    rng = np.random.default_rng(0)
    legs = [np.c_[rng.uniform(-0.10, 0.10, 3000), rng.uniform(y0, y0 + 0.05, 3000)] for y0 in (-0.065, 0.015)]
    P = np.vstack(legs); opts = part_grip_options(P - P.mean(0))
    assert opts and 0.04 < opts[0][0] < 0.06
