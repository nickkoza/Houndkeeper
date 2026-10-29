"""No night or score uses a scan of several separate objects fused into one body (sim-items [w8q])."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))


def test_every_scan_in_the_pool_is_one_object():
    import items
    for split in ("train", "heldout"):
        several = [n for n in items.pool(split) if items.ITEMS[n][0] == "gso" and len(items.separate_objects(n)) > 1]
        assert not several, f"{len(several)} {split} scans hold separate objects" + (f": {several}" if split == "train" else "")


def test_the_taxi_is_two_cars():
    import items
    assert len(items.separate_objects("toy_taxi")) == 2 and "toy_taxi" not in items.pool("train")
    assert len(items.separate_objects("my_little_pony_princess_cele")) == 1        # a mane that touches: one object
