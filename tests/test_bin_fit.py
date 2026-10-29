"""What fits the bin (sim-slice [QyZ], look-closer [pYZ]: the 35 x 28 cm cart turtle is too big; shoes,
27 x 10 cm, are the upper end of binnable)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))


def test_the_tier_rule_knows_the_bin():
    import items
    assert items.grasp_tier("melissa_doug_cart_turtle_blo") == "too_big"      # 35 x 28 cm: doesn't fit 30 x 22
    assert items.grasp_tier("toy_taxi") != "too_big"                          # 12.7 x 5.9 cm (its 27 x 27 was the cache mix-up)
    assert items.grasp_tier("reebok_reesculpt_trainer_ii") != "too_big"       # a shoe fits, at the upper end
    assert items.grasp_tier("starstruck_tieks_glittery_go") != "too_big"


def test_the_robot_refuses_what_its_measure_says_wont_fit():
    import pick
    assert not pick.fits_bin({"length_cm": 36.0, "breadth_cm": 28.0})         # the turtle, measured
    assert not pick.fits_bin({"length_cm": 27.0, "breadth_cm": 27.0})         # something 27 cm square
    assert pick.fits_bin({"length_cm": 29.5, "breadth_cm": 11.0})             # a shoe, over-measured a little
    assert pick.fits_bin({"grip_width_cm": 5.0})                               # nothing measured: no refusal


def test_binned_means_over_the_bin():
    import bin_geom as bg
    trunk, quat = np.array([1.0, 2.0, 0.30]), np.array([1.0, 0, 0, 0])          # facing +x
    over = np.array([1.0 + bg.TRAY_X, 2.0, 0.35])                              # the bin's middle
    leaning = over + [bg.TRAY_HALF[0] + 0.03, 0, 0.05]                         # a shoe resting on the back wall
    beside = np.array([1.0 + 0.25, 2.0, 0.35])                                 # on the robot's back, forward of the bin
    low = over - [0, 0, 0.2]                                                   # under it: on the floor
    assert bg.in_bin(over, trunk, quat) and bg.in_bin(leaning, trunk, quat)
    assert not bg.in_bin(beside, trunk, quat) and not bg.in_bin(low, trunk, quat)


def test_a_toys_size_doesnt_depend_on_what_was_measured_before_it():
    """the mesh-surface cache was keyed by id(model): a freed model's address, reused by the next, handed it the last
    toy's points (29 Sep: the taxi 26.6 x 26.5 cm after other toys, 12.7 x 5.9 alone; every round's tiers scrambled)"""
    import items
    items._FOOT.clear(); items._GRIP.clear(); items._SETTLED.clear(); items._TIER.clear()
    seq = ["3m_vinyl_tape_green_1_x_36_y", "car_carrier_train", "elephant", "melissa_doug_cart_turtle_blo",   # the tuning
           "my_little_pony_princess_cele", "pokmon_x_nintendo_3ds_game", "razer_abyssus_ambidextrous_g",   # rounds' toys, in
           "reebok_reesculpt_trainer_ii", "rolling_lion", "school_bus", "starstruck_tieks_glittery_go", "toy_taxi",   # the order
           "toysmith_windem_up_flippin_a", "transformers_age_of_extincti", "triceratops"]                  # that scrambled
    for n in seq: items.footprint_half(n); items.grasp_tier(n)
    got = {n: [round(2 * v * 100, 1) for v in items.footprint_half(n)] for n in ("toy_taxi", "triceratops")}
    assert abs(got["toy_taxi"][0] - 12.7) < 1.0 and abs(got["toy_taxi"][1] - 5.9) < 1.0, got
    assert abs(got["triceratops"][0] - 6.9) < 1.0 and abs(got["triceratops"][1] - 4.6) < 1.0, got


def test_hard_is_the_items_named_hard():
    import items                     # hard: the winged pony, tiny toys, and long ones that tend to rotate
    assert items.grasp_tier("my_little_pony_princess_cele") == "hard"         # too wide but for its legs: awkward
    assert items.grasp_tier("toysmith_windem_up_flippin_a") == "hard"         # 4.9 cm: tiny
    assert items.grasp_tier("car_carrier_train") == "hard"                    # 22 x 3.9 cm: long and slender
    assert items.grasp_tier("ballet_flat") == "easy"                          # 25 x 8 cm, a shoe: rests steady
    assert items.grasp_tier("olive_kids_birdie_lunch_box") == "too_big"       # its handle 8.5 cm: no part with room
    assert items.grasp_tier("pokmon_x_nintendo_3ds_game") == "too_big"        # flat and wide: no part to take


def test_a_thin_part_of_a_wide_body_is_hard_and_a_toy_standing_up_isnt_tiny():
    import items                     # the rule's check on the held-out toys, blind to which (29 Sep)
    assert items.grasp_tier("room_essentials_mug_white_ye") == "hard"         # 4.2 cm handle, 10.2 cm body
    assert items.grasp_tier("turboprop_airplane_with_pilo") == "hard"         # across a wing of a 10 cm body
    assert items.grasp_tier("matte_black_tieks_italian_le") == "easy"         # a shoe's taper, 6.3 of 7.6 cm
    assert items.tier_of(0.04, 0.05, 0.04, 0.09, None, 0.095) == "easy"       # 5 x 4 cm standing 9 cm tall
    assert items.tier_of(0.03, 0.05, 0.035, 0.05, None, 0.095) == "hard"      # 5 cm every way: tiny
