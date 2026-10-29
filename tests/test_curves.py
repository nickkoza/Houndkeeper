"""[HGT]: arms compared by their curves: the threshold sweep through the shared scorer, precision-recall and FROC
points, and average precision."""
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def _ctx():
    toys = [{"id": f"t{i}", "name": "toy", "x": float(i), "y": 0.0, "reach": 0.3, "tier": "easy", "floor": "carpet",
             "named": True, "area_cm2": 50.0, "also": []} for i in range(4)]
    return {"n1": {"toys": toys, "stains": [], "to_true": lambda x, y, t: (x, y)}}


def _arm(points):
    return {"arm": "floor", "nights": {"n1": {"points": [dict(x=x, y=y, ts=0.0, score=s, frame=f"p{i}") for i, (x, y, s) in enumerate(points)]}}}


def test_the_sweep_walks_recall_up_and_false_positives_up_as_the_threshold_falls():
    """[HGT]: each distinct score is a threshold; lower thresholds keep more candidates"""
    from baselines import curves as cv
    arm = _arm([(0, 0, 0.9), (1, 0, 0.8), (5, 5, 0.7), (2, 0, 0.6), (7, 7, 0.5)])
    c = cv.sweep(arm, _ctx())
    assert [p["thr"] for p in c] == [0.9, 0.8, 0.7, 0.6, 0.5]
    assert [p["found"] for p in c] == [1, 2, 2, 3, 3] and [p["fp"] for p in c] == [0, 0, 1, 1, 2]
    r, p, f = cv.pr_points(c)[-1]
    assert r == 0.75 and abs(p - 3 / 5) < 1e-9 and f == 2.0


def test_average_precision_is_one_for_a_perfect_ranking_and_less_when_a_false_one_ranks_first():
    """[HGT]: AP summarises the whole curve"""
    from baselines import curves as cv
    perfect = cv.sweep(_arm([(i, 0, 1 - 0.1 * i) for i in range(4)]), _ctx())
    assert abs(cv.average_precision(perfect) - 1.0) < 1e-9
    worse = cv.sweep(_arm([(9, 9, 1.0)] + [(i, 0, 0.9 - 0.1 * i) for i in range(4)]), _ctx())
    assert 0.5 < cv.average_precision(worse) < 1.0
