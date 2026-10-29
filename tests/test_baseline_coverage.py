"""[YuI]: a floor picture seen from too little of the night is not a baseline (limits: decision [NvG])."""
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def test_the_broken_nights_on_disk_are_refused_and_says_how_much_they_saw():
    """[YuI], [NvG]: house 91's night of 1 frame over 2% of the grid (3.3 m2), house 92's 7 frames over 9% (14.8 m2)"""
    import floor_mosaic2 as fm
    for used, m2 in ((1, 3.3), (7, 14.8)):
        msg = fm.baseline_refusal(used, m2)
        assert msg and f"{used} frame" in msg and f"{m2:.1f} m2" in msg


def test_the_smallest_good_nights_on_disk_pass():
    """[YuI], [NvG]: house 88's 44 frames over 16% (26 m2); the demo room's 389 frames over 12% (20 m2)"""
    import floor_mosaic2 as fm
    assert fm.baseline_refusal(44, 26.3) is None and fm.baseline_refusal(389, 19.7) is None


def test_too_little_floor_alone_is_refused_and_zero_turns_a_limit_off():
    """[YuI]: many frames over a sliver of floor (a robot that stood still) is refused too; 0 is no limit (--part)"""
    import floor_mosaic2 as fm
    assert fm.baseline_refusal(300, 4.0) and "floor" in fm.baseline_refusal(300, 4.0)
    assert fm.baseline_refusal(1, 3.3, min_frames=0, min_floor_m2=0) is None


def test_the_builder_applies_it_by_default():
    """[YuI]: the limits are the builder's defaults, so every baseline picture is held to them"""
    import floor_mosaic2 as fm
    assert (fm.MIN_FRAMES, fm.MIN_FLOOR_M2) == (20, 10.0)
    src = (HK / "sim" / "house" / "floor_mosaic2.py").read_text()
    assert "baseline_refusal(used," in src and '"--min-frames", type=int, default=MIN_FRAMES' in src
