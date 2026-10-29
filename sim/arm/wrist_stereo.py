"""What the wrist stereo pair adds to the close look (look-closer [GMs], waymark 0011 [A8K]): the item whole where the
detector boxed a part of it, and room for the fingers beside a part the jaws would take."""
import numpy as np
from scipy import ndimage as ndi

MIN_H_M = 0.008      # standing above the floor by more than this: part of something
GROW = 1.8           # a standing region this many times the detector's mask: the mask was a part of it
FINGER_M = 0.012     # a finger's landing spot beside a part (est.)
FLOOR_H_M = 0.010    # what counts as floor where a finger lands


def standing(h, robot, min_h=MIN_H_M):
    """the pixels standing more than min_h above the floor (h: heights, NaN unknown), cleaned: specks out, gaps closed,
    holes filled"""
    up = ndi.binary_opening(np.nan_to_num(h, nan=0.0) > min_h, iterations=1)
    return ndi.binary_fill_holes(ndi.binary_closing(up, iterations=3)) & ~robot


def whole(det_mask, stand):
    """the detector's mask, or the standing region holding it when that is GROW times bigger; and whether it grew"""
    lab, _ = ndi.label(stand); ids = lab[det_mask & stand]
    if not len(ids): return det_mask, False
    reg = lab == np.bincount(ids).argmax()
    return (reg | det_mask, True) if reg.sum() > GROW * det_mask.sum() else (det_mask, False)


def fingers_land(g, jaw, w, points, h):
    """room for both fingers beside a part: at the grasp point g (world xy) across width w along jaw, the pair's points
    where each finger comes down are floor, not more of the item"""
    ok = np.isfinite(h); xy = points[ok][:, :2]; hh = h[ok]
    for side in (-1, 1):
        spot = np.asarray(g) + side * (w / 2 + FINGER_M) * np.asarray(jaw)
        near = np.linalg.norm(xy - spot, axis=1) < FINGER_M
        if near.sum() < 5 or np.percentile(hh[near], 80) > FLOOR_H_M: return False
    return True
