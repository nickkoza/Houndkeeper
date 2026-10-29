"""[evl]: a floor picture can average each spot's views (a trimmed mean by brightness) instead of taking one."""
import sys
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))

K, H, W = 15, 40, 60


def _views(seed=1, noise=6.0):
    """a dark, saturated tile (as under the night bulb) seen K times, each view with its own sensor noise"""
    rng = np.random.default_rng(seed)
    tile = np.repeat(np.repeat(rng.uniform(15, 70, (H // 10, W // 10, 3)), 10, 0), 10, 1)
    views = tile[None] + rng.normal(0, noise, (K, H, W, 3))
    return tile, rng, views


def _u8(a):
    return np.clip(a + 0.5, 0, 255).astype(np.uint8)


def test_one_view_speckle_averages_away():
    """each cell has one view whose colour is far off (a JPEG block, a frame's own gain): the trimmed mean lands near the
    floor, far nearer than any one view taken whole does"""
    import floor_mosaic2 as fm
    tile, rng, views = _views()
    odd = rng.integers(0, K, (H, W)); ch = rng.integers(0, 3, (H, W)); sign = rng.choice([-1.0, 1.0], (H, W))
    r, c = np.mgrid[:H, :W]; views[odd, r, c, ch] += 60 * sign
    rgbk = _u8(views); valid = np.ones((K, H, W), bool)
    out = fm.trimmed_mean(rgbk, valid, 10, 90).astype(np.float32)
    one = rgbk[3].astype(np.float32)                                     # a single view, whole
    err_mean, err_one = np.abs(out - tile).mean(), np.abs(one - tile).mean()
    assert err_mean < 3.0 and err_mean < 0.4 * err_one


def test_a_glint_in_one_view_stays_out():
    """a reflection only adds light, and from one place on the route only: the brightest view is left out"""
    import floor_mosaic2 as fm
    tile, rng, views = _views(seed=2, noise=2.0)
    views[7, 10:30, 20:45] += 180                                        # one view's glint over a patch
    rgbk = _u8(views); valid = np.ones((K, H, W), bool)
    glint = (slice(10, 30), slice(20, 45))
    trimmed = fm.trimmed_mean(rgbk, valid, 10, 90).astype(np.float32)
    plain = fm.trimmed_mean(rgbk, valid, 0, 100).astype(np.float32)      # every view: the glint gets in
    assert np.abs(trimmed[glint] - tile[glint]).mean() < 1.5
    assert np.abs(plain[glint] - tile[glint]).mean() > 8.0


def test_cells_with_few_or_no_views():
    """unseen views are ignored, a cell keeps at least one view, and a cell nobody saw stays black"""
    import floor_mosaic2 as fm
    rgbk = np.zeros((K, 2, 2, 3), np.uint8); valid = np.zeros((K, 2, 2), bool)
    rgbk[:, 0, 0] = 200; rgbk[0, 0, 0] = 40; valid[0, 0, 0] = True       # one view seen: it, whatever the others hold
    rgbk[:3, 0, 1] = [[30, 30, 30], [40, 40, 40], [250, 250, 250]]; valid[:3, 0, 1] = True
    out = fm.trimmed_mean(rgbk, valid, 10, 90)
    assert (out[0, 0] == 40).all()
    assert (out[1, 1] == 0).all()
    assert (out[0, 1] >= 30).all() and (out[0, 1] <= 250).all()
