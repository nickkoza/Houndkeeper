"""[yyJ]: the rug is down from the first night, in the baseline; a major floor change (a rug that appears) is a
separate test, not part of toy detection's. A rug new on night 3 made 17 stops for nothing."""
import json, shutil, subprocess, sys
from pathlib import Path

HOUSE = Path(__file__).resolve().parents[1] / "sim" / "house"
SEED = 9913                                                   # a seed no timeline uses; its nights are removed after


def _nights(*extra):
    out = HOUSE / "runs" / f"nights_s{SEED}"
    try:
        subprocess.run([sys.executable, "make_nights.py", "--seed", str(SEED), "--nights", "3", "--per-night", "0", *extra],
                       cwd=HOUSE, check=True, capture_output=True)
        return [json.load(open(out / f"nights_s{SEED}_n{n}.truth.json")) for n in (1, 2, 3)]
    finally:
        shutil.rmtree(out, ignore_errors=True)


def test_rug_is_in_the_baseline():
    t = _nights()
    assert all(n["rug"] for n in t) and not any(n["rug_new_tonight"] for n in t)


def test_a_rug_appearing_is_still_available_as_its_own_test():
    t = _nights("--rug-night", "3")
    assert [bool(n["rug"]) for n in t] == [False, False, True] and t[2]["rug_new_tonight"]
