"""[TJu]: furniture moves between nights ([x3f]; chairs first), baselines included, and the nightly
runs build their houses with it."""
import math
import random
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house"))


def test_chairs_stand_somewhere_else_each_night():
    import make_house as mh, movables as mv
    g = mh.base_grid(); seed = 70
    n1 = mv.place(random.Random(seed * 7919 + 1), g, None)                # as make_nights draws them, night by night
    n2 = mv.place(random.Random(seed * 7919 + 2), g, n1)
    moved = [c for c in n2 if c["moved_m"] and c["moved_m"] > 0.03]
    assert len(moved) >= len(n2) // 2                                     # most have been moved by morning
    assert any(abs(math.remainder(a["yaw"] - b["yaw"], 2 * math.pi)) > 0.05 for a, b in zip(n1, n2))   # some turned


def test_the_nightly_runs_build_houses_with_moving_chairs():
    sh = (HK / "sim" / "dimos" / "night_tidy.sh").read_text()
    assert '[ "${MOVABLES:-1}" = 1 ] && echo --movables' in sh


def test_big_moves_happen_as_often_as_drawn():
    """the move is drawn once per chair; re-drawing it on every retry had made big moves 3% of chairs, not 15%"""
    import collections, make_house as mh, movables as mv
    g = mh.base_grid(); modes = collections.Counter(); prev = None
    for n in range(1, 41):
        prev = mv.place(random.Random(70 * 7919 + n), g, prev); modes.update(c["mode"] for c in prev)
    share = modes["a lot"] / sum(modes.values())
    assert abs(share - (1 - mv.P_TUCK - mv.P_BIT)) < 0.06


def test_toys_are_placed_clear_of_tonights_chairs(tmp_path):
    import json, subprocess, make_house as mh, movables as mv
    chairs = mv.place(random.Random(3), mh.base_grid(), None)
    sx, sy = mh.cell_to_world(*mh.START)
    chairs[0].update(x=sx + 1.5, y=sy)                                    # one chair right where toys go
    t = tmp_path / "t.json"; t.write_text(json.dumps({"start_pos": [sx, sy], "movables": chairs}))
    for seed in range(6):
        out = subprocess.run([sys.executable, str(HK / "sim" / "dimos" / "house_items.py"), str(t), "3", str(seed)],
                             capture_output=True, text=True, check=True).stdout.splitlines()
        for _, _, (x, y, _) in json.loads(out[1]):
            for c in chairs:
                assert math.hypot(x - c["x"], y - c["y"]) > math.hypot(c["w"], c["d"]) / 2


def test_no_two_chairs_ever_stand_on_each_other():
    """[yyJ] (two chairs ended up on top of each other; house 78 night 1: a chair moved a
    lot onto another's home, which then stayed home on top of it): over many houses and nights, no two chairs closer than
    a chair is wide"""
    import random
    import make_house as mh, movables as mv
    for seed in range(70, 90):
        mh.use_layout(seed); g = mh.base_grid(); prev = None
        for n in range(1, 9):
            chairs = mv.place(random.Random(seed * 1000 + n), g, prev); prev = chairs
            for i in range(len(chairs)):
                for j in range(i + 1, len(chairs)):
                    a, b = chairs[i], chairs[j]; gap = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
                    assert gap >= max(a["w"], b["w"]), f"house {seed} night {n}: {a['id']} on {b['id']} ({gap:.2f} m apart)"
