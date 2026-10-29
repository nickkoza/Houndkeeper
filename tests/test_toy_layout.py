"""[O7l]: a night's toys lie where a household leaves them: spread over the walk's floor, now and then a few together,
never all in one patch, the robot's first metre clear."""
import itertools
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "dimos"))


def _nights(**kw):
    import house_items as hi
    clear = lambda p: -7.5 < p[0] < 7.5 and -4.0 < p[1] < 4.0           # an open floor, 15 x 8 m
    start = np.array([-5.5, -2.5]); out = []
    for seed in range(60):
        P = hi.layout(np.random.default_rng(seed), start, clear, 8, **kw)
        assert len(P) == 8
        out.append((start, P))
    return out


def test_toys_spread_over_the_walk_with_a_few_together():
    import house_items as hi
    nights = _nights(**hi.SPREAD)
    spans = [max(np.linalg.norm(a - b) for a, b in itertools.combinations(P, 2)) for _, P in nights]
    nn = [np.median([min(np.linalg.norm(a - b) for b in P if b is not a) for a in P]) for _, P in nights]
    pairs = [any(np.linalg.norm(a - b) < 1.0 for a, b in itertools.combinations(P, 2)) for _, P in nights]
    assert min(spans) >= 4.5 and np.median(spans) >= 5.5                 # never all in one patch (the old layout: 3.6, 4.5)
    assert np.median(nn) >= 1.1                                          # most of them apart (pairs pull the median down)
    assert np.mean(pairs) >= 0.3                                         # a pair or a small cluster, often (a couple together is fine)
    assert all(np.linalg.norm(p - s) >= 1.0 for s, P in nights for p in P)   # the robot's first metre clear
    heap = [max(sum(np.linalg.norm(a - b) < 1.0 for b in P) for a in P) for _, P in nights]
    assert max(heap) <= 3                                                # a pair, a trio at most: never a heap


def test_the_old_layout_heaped_every_toy_by_the_start():
    """what it replaces: 1-3 m from the start, 0.6 m apart (every bench night until 1 Oct: all eight within 3-4 m)"""
    spans = [max(np.linalg.norm(a - b) for a, b in itertools.combinations(P, 2)) for _, P in _nights()]
    assert max(spans) <= 6.0 and np.median(spans) < 5.0
