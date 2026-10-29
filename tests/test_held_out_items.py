"""[nuQ]: held-out items only judge. Every training, demonstration and tuning entry point draws from the training items;
judging asks for the held-out ones by name."""
import os, subprocess, sys, json
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "arm"))


def test_training_and_held_out_items_are_disjoint_and_cover_the_library():
    import items
    train, held = set(items.pool("train")), set(items.pool("heldout"))
    assert held == set(items.HELD_OUT) and not (train & held)
    # every item is in exactly one of: training, held out, or set aside as several objects fused in one scan (sim-items
    # [w8q]: no night or score uses those, whichever side of the split they'd fall on)
    assert not (train | held) & items.SEPARATE_PIECES and train | held | items.SEPARATE_PIECES == set(items.ITEMS)
    assert not set(items.PICKABLE) & held                          # what recorders, benches and tuning draw from
    assert set(items.PICKABLE_HELD_OUT) <= held


def test_house_nights_place_training_items_unless_judging(tmp_path):
    import items
    truth = sorted((HK / "sim" / "house" / "runs").glob("nights_s*/nights_s*_n1.truth.json"))[0]
    def placed(split):
        env = dict(os.environ, HK_ITEMS=split, PYTHONPATH=str(HK / "sim" / "arm") + ":" + str(HK / "sim" / "house"))
        seen = set()
        for seed in range(6):
            out = subprocess.run([sys.executable, str(HK / "sim" / "dimos" / "house_items.py"), str(truth), "4", str(seed)],
                                 env=env, capture_output=True, text=True, check=True).stdout.strip().splitlines()
            seen |= {name for name, _, _ in json.loads(out[-1])}
        return seen
    assert not placed("train") & items.HELD_OUT
    assert placed("heldout") <= items.HELD_OUT


def test_gripper_pool_is_training_items_that_fit_the_jaws():
    import items, jaws
    train, held = items.grippable("train"), items.grippable("heldout")
    assert not set(train) & items.HELD_OUT and set(held) <= items.HELD_OUT
    assert len(train) >= 30                                        # toys and household things, not just the fixed shapes
    widths = {n: 2 * items.footprint_half(n)[1] for n in train[:5]}
    assert all(w <= jaws.max_m() for w in widths.values()), widths
