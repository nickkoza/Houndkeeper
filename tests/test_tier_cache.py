"""[QyZ]: a test drop's grasp tier, kept on disk, is the tier the measurement gives; any change to what it depends on
measures afresh."""
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "arm")); sys.path.insert(0, str(HK / "sim" / "house"))


def test_a_kept_tier_is_the_measured_one_and_a_changed_item_is_measured_again(tmp_path, monkeypatch):
    import items
    monkeypatch.setattr(items, "TIER_CACHE", tmp_path)
    monkeypatch.setenv("HK_TIER_CACHE", "0"); items._TIER.clear()
    measured = {n: items.grasp_tier(n) for n in ("block", "puck")}
    assert not list(tmp_path.iterdir())                                         # off: nothing kept
    monkeypatch.setenv("HK_TIER_CACHE", "1"); items._TIER.clear()
    assert {n: items.grasp_tier(n) for n in measured} == measured               # measured and kept
    assert len(list(tmp_path.glob("*.json"))) == 2
    items._TIER.clear(); assert {n: items._tier_kept(n) for n in measured} == measured   # read back as measured
    key = items.tier_key("block")
    kind, spec, mass, *rest = items.ITEMS["block"]
    monkeypatch.setitem(items.ITEMS, "block", (kind, dict(spec, half=[2 * h for h in spec["half"]]), mass, *rest))
    assert items.tier_key("block") != key and items._tier_kept("block") is None   # a bigger block: not the kept tier
    items._TIER.clear()
