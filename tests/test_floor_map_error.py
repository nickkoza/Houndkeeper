"""[nv5]: the floor map's error against the actual floor, the funnel's step above detection."""
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def _floor(seed=3, h=300, w=400):
    rng = np.random.default_rng(seed)
    base = np.repeat(np.repeat(rng.uniform(60, 200, (h // 20 + 1, w // 20 + 1, 3)), 20, 0), 20, 1)[:h, :w]   # tiles
    return np.clip(base + rng.normal(0, 4, (h, w, 3)), 0, 255).astype(np.uint8)


def test_a_map_that_is_the_floor_has_no_error_and_one_off_by_a_tile_does():
    import floor_fidelity as ff
    T = _floor(); V = np.ones(T.shape[:2], bool); M = np.zeros(T.shape[:2], bool)
    same = ff.map_error(T, V, T.astype(np.float32), M)
    shifted = ff.map_error(np.roll(T, 10, 1), V, T.astype(np.float32), M)
    assert same < 0.001 and shifted > 0.10


def test_lighting_alone_is_not_error():
    """a smooth lamp pool over the map is not a difference from the floor (the detector's own test, relative colour)"""
    import floor_fidelity as ff
    T = _floor(); V = np.ones(T.shape[:2], bool); M = np.zeros(T.shape[:2], bool)
    r, c = np.mgrid[:T.shape[0], :T.shape[1]]; pool = 0.5 + 0.5 * np.exp(-((r - 150) ** 2 + (c - 200) ** 2) / (2 * 120 ** 2))
    lit = np.clip(T * pool[..., None], 0, 255).astype(np.uint8)
    assert ff.map_error(lit, V, T.astype(np.float32), M) < 0.02


def test_the_edge_of_what_the_map_saw_is_not_error():
    """the map's colour is judged against its own surroundings; the truth's must be too, over the same floor"""
    import floor_fidelity as ff
    T = _floor(); V = np.zeros(T.shape[:2], bool); V[50:250, 60:340] = True; M = np.zeros(T.shape[:2], bool)
    seen = np.where(V[..., None], T, 0).astype(np.uint8)
    assert ff.map_error(seen, V, T.astype(np.float32), M) < 0.001
