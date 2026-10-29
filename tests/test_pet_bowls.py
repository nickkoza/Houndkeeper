"""[TJu]: a pet's food and water bowls are part of the house every night, nudged about, and never a thing to pick up
(each bowl on its own)."""
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "sim" / "arm")); sys.path.insert(0, str(HK / "src"))


def test_the_bowls_are_never_in_the_pick_pool():
    import items
    for split in ("train", "heldout"):
        assert not [n for n in items.pool(split) if "dog_bow" in n or "Dog_Bow" in str(items.ITEMS[n][1])]
    assert set(items.FIXTURES) == {"dog_bowl_plastic", "dog_bowl_ceramic"}


def test_the_bowls_are_nudged_between_nights_but_stay_by_the_kitchen():
    """the bowls' home is the house plan's (make_house.layout(seed)["bowls"], [uCP]): by the kitchen counter's end in
    every house, mirrored or not; each night nudges them about that home and no two nights alike"""
    import make_house as mh, make_nights as mn
    was = mh.LAYOUT_SEED
    try:
        for seed in (70, 72, 73, 74, 75, 76, 77):
            mh.use_layout(seed); L = mh.layout(seed)
            (cx0, cy0, cx1, cy1, _, _), = [f for f in L["furniture"] if f[5] == "kitchen_counter"]
            (ax, ay), (bx, by) = mh.cell_to_world(cx0 - 0.5, cy0 - 0.5), mh.cell_to_world(cx1 + 0.5, cy1 + 0.5)
            nights = [mn.pet_bowls_tonight(seed, n) for n in range(1, 9)]
            assert [b["name"] for b in nights[0]] == [n for n, _, _ in L["bowls"]]
            for i, (_, hx, hy) in enumerate(L["bowls"]):
                gap = np.hypot(max(ax - hx, 0, hx - bx), max(by - hy, 0, hy - ay))   # home to the counter's footprint
                assert gap < 1.0, (seed, i, gap)
                xy = np.array([[b[i]["x"], b[i]["y"]] for b in nights])
                assert np.linalg.norm(xy - [hx, hy], axis=1).max() < 3 * mn.PET_NUDGE * 1.5
                assert len({tuple(p) for p in xy}) == len(xy)
    finally:
        mh.use_layout(was)


def test_the_sim_puts_tonights_bowls_in_the_house(tmp_path, monkeypatch):
    import json, mujoco
    from houndkeeper import sim_hook
    import items
    spec = mujoco.MjSpec.from_string('<mujoco model="scene_nights_s1_n2"><worldbody><geom type="plane" size="5 5 .1"/></worldbody></mujoco>')
    runs = tmp_path / "house" / "runs" / "nights_s1"; runs.mkdir(parents=True)
    (runs / "nights_s1_n2.truth.json").write_text(json.dumps({"fixtures": [{"name": "dog_bowl_ceramic", "uid": "pet_bowl_0", "x": 1.0, "y": 2.0, "yaw": 0}]}))
    monkeypatch.setattr(sim_hook, "ARM_SIM", tmp_path / "arm")
    fx = sim_hook._fixtures(spec); assert fx and fx[0]["uid"] == "pet_bowl_0"
    items.add(spec, fx[0]["name"], [1.0, 2.0, 0.04], fx[0]["uid"]); m = spec.compile()
    assert m.body("pet_bowl_0").id >= 0


def test_the_nightly_runs_have_a_pet():
    assert '[ "${PETS:-1}" = 1 ] && echo --pets' in (HK / "sim" / "dimos" / "night_tidy.sh").read_text()
