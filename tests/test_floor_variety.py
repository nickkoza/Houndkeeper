"""[aj8]: the floor-variety tuning houses are laid with their named training floors, every carpet kind among them."""
import os
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "sim" / "arm"))


def test_the_variety_houses_cover_every_training_carpet_and_new_hard_floors():
    import make_nights as mn, floors
    train = {r["id"]: r["kind"] for r in floors.photos("train")}
    carpets = {c for _, c in mn.FLOOR_VARIETY.values()}
    assert carpets == {r["id"] for r in floors.photos("train", ["carpet"])}
    assert all(train[h] in ("wood", "tile", "tile_pattern") for h, _ in mn.FLOOR_VARIETY.values())
    assert len({h for h, _ in mn.FLOOR_VARIETY.values()}) == len(mn.FLOOR_VARIETY) == 12
    assert min(mn.FLOOR_VARIETY) > 81                                   # clear of the hold-out houses (80, 81)
