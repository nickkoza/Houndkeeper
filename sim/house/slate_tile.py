"""A floor of large slate-look porcelain tiles laid on the diagonal ([QMt]; 27 Sep 2026: a complex, high-contrast tile design,
seen from above: ~48 cm tiles at 45 deg, each a different brown, rust or grey stone print with pale veins, thin dark
grout, and at some crossings the corners clipped around a small four-square inset of dark and light stone). Built from
the CC0 stone photos (floor_photos.csv, kind stone): each tile a crop of one of them, turned, its tone moved to one of
the palette's (so the stone's own detail stays) and its contrast kept high. Sizes and palette are the project's own
chosen values.

Returns maps in make_nights' PNG layout (row 0 = +y edge): albedo 0-1, normals (world axes), roughness, occlusion."""
import math

import numpy as np

TILE_M, GROUT_M = 0.48, 0.005          # tile side and grout width (est.: 18-20 inch porcelain, 3/16 inch grout)
INSET_M = 0.13                         # the four-square inset's side, at one crossing in four (est.)
PALETTE = [(0.40, 0.31, 0.24), (0.33, 0.28, 0.24), (0.45, 0.33, 0.23), (0.29, 0.26, 0.24), (0.37, 0.32, 0.27),
           (0.47, 0.37, 0.28), (0.31, 0.24, 0.19)]   # tile tones: browns, rust, grey-brown (est. by eye)
GROUT = (0.16, 0.14, 0.12); INSET_DARK, INSET_LIGHT = (0.22, 0.21, 0.20), (0.46, 0.38, 0.29)
SOURCES = ("Rock049", "Rock061", "Rock039", "Rock056", "Travertine005")


def slate_tile(rng, hpx, wpx, px, hx, hy, sources=SOURCES):
    import cv2
    import floors
    r, c = np.mgrid[0:hpx, 0:wpx].astype(np.float32)
    x = (c + 0.5) * px - hx; y = hy - (r + 0.5) * px
    u = (x + y) / math.sqrt(2); v = (y - x) / math.sqrt(2)                   # the tiles' own axes, at 45 deg
    i = np.floor(u / TILE_M).astype(np.int32); j = np.floor(v / TILE_M).astype(np.int32)
    lu = u - i * TILE_M; lv = v - j * TILE_M
    keys = i * 100003 + j; uk, inv = np.unique(keys, return_inverse=True); inv = inv.reshape(hpx, wpx)
    n = len(uk); src = rng.integers(len(sources), size=n); rot = rng.integers(4, size=n)
    tone = np.array(PALETTE, np.float32)[rng.integers(len(PALETTE), size=n)] * rng.uniform(0.9, 1.1, (n, 1)).astype(np.float32)
    col = np.zeros((hpx, wpx, 3), np.float32)
    for s_i, sid in enumerate(sources):
        sel = src[inv] == s_i
        if not sel.any(): continue
        img = floors._photo_maps(sid)[0]; S = img.shape[0]
        size_m = float(next(rw for sp in ("train", "heldout") for rw in floors.photos(sp, ["stone"]) if rw["id"] == sid)["size_m"])
        crop = TILE_M / size_m * S                                           # source pixels one tile spans
        ox = rng.uniform(0, S - crop, n).astype(np.float32); oy = rng.uniform(0, S - crop, n).astype(np.float32)
        a = lu / TILE_M; b = lv / TILE_M; k = rot[inv]
        su = np.where(k == 0, a, np.where(k == 1, 1 - b, np.where(k == 2, 1 - a, b)))
        sv = np.where(k == 0, b, np.where(k == 1, a, np.where(k == 2, 1 - b, 1 - a)))
        mx = (ox[inv] + su * crop).astype(np.float32); my = (oy[inv] + sv * crop).astype(np.float32)
        small = cv2.resize(img, (S // 2, S // 2), interpolation=cv2.INTER_AREA) if crop * 2 < S / 3 else img
        f = small.shape[0] / S
        got = cv2.remap(small, mx * f, my * f, cv2.INTER_AREA if f < 1 else cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
        col[sel] = got[sel]
    sums = np.zeros((n, 3), np.float64); np.add.at(sums, inv.ravel(), col.reshape(-1, 3)); cnt = np.bincount(inv.ravel(), minlength=n)[:, None]
    mean = (sums / np.maximum(cnt, 1)).astype(np.float32)
    lum = lambda a_: a_ @ np.array([0.2126, 0.7152, 0.0722], np.float32)
    d0 = col - mean[inv]; dl = d0 @ np.array([0.2126, 0.7152, 0.0722], np.float32)
    detail = (0.5 * d0 + 0.5 * dl[..., None]) * 1.25                                        # each tile's tone moved to its palette colour; the
                                                                             # stone's own veins and mottling kept, a little stronger,
                                                                             # their colour halved (lichen yellows and blues are rock's)
    col = np.clip(tone[inv] + detail * (lum(tone)[inv] / np.maximum(lum(mean)[inv], 0.05))[..., None], 0, 1)
    grout = np.minimum(np.minimum(lu, TILE_M - lu), np.minimum(lv, TILE_M - lv)) < GROUT_M / 2
    col[grout] = GROUT
    # the insets: at one crossing in four, a square turned to the room's axes over the four corners (they're clipped)
    ci = np.round(u / TILE_M); cj = np.round(v / TILE_M)
    at = (ci % 2 == 0) & (cj % 2 == 0)
    cu, cv = ci * TILE_M, cj * TILE_M; cx = (cu - cv) / math.sqrt(2); cy = (cu + cv) / math.sqrt(2)
    dx, dy = x - cx, y - cy; h = INSET_M / 2
    inset = at & (np.abs(dx) < h + GROUT_M) & (np.abs(dy) < h + GROUT_M)
    inner = at & (np.abs(dx) < h) & (np.abs(dy) < h) & (np.abs(dx) > GROUT_M / 2) & (np.abs(dy) > GROUT_M / 2)
    col[inset] = GROUT
    quad_dark = (dx > 0) == (dy > 0)
    col[inner & quad_dark] = INSET_DARK; col[inner & ~quad_dark] = INSET_LIGHT
    nrm = np.zeros((hpx, wpx, 3), np.float32); nrm[..., 2] = 1.0            # glazed porcelain: flat
    rough = np.where(grout | inset, 0.9, 0.45).astype(np.float32); rough[inner] = 0.6
    occ = np.where(grout | (inset & ~inner), 0.7, 1.0).astype(np.float32)
    return col, nrm, rough, occ                                              # rows already run from +y (y from r above)
