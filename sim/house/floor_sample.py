"""Reading a floor cell's colour from a camera frame ([FHA]): the average of the pixels the cell covers, not the one
nearest its centre. A 1 cm cell near the camera spans several pixels; the nearest one alone aliased the carpet's pile and
put a rug's or a seam's edge on either side of the cell from view to view, so two maps of an unchanged floor disagreed
(the mapping bench, 26 Sep 2026: 0.60-0.94 correlation with one pixel, 0.96-0.99 averaged). Mipmapping: the frame at
full, 1/2, 1/4 and 1/8 size, each cell read from the level where a pixel is about its size, bilinearly. One place, so
the floor picture (floor_mosaic2.py) and tonight's frames (confirm_stream.py) read the floor alike."""
import numpy as np
from scipy.ndimage import map_coordinates

LEVELS = 4


def pyramid(img):
    """the frame at full, 1/2, 1/4 and 1/8 size (2 x 2 box averages), as float32"""
    out = [np.asarray(img, np.float32)[..., :3]]
    for _ in range(LEVELS - 1):
        a = out[-1]; h, w = a.shape[0] // 2 * 2, a.shape[1] // 2 * 2
        out.append(a[:h, :w].reshape(h // 2, 2, w // 2, 2, 3).mean((1, 3)))
    return out


def sample(pyr, u, v, px_per_cell):
    """the colour (N x 3, float) of cells centred at image points (u, v), each covering px_per_cell pixels across: read
    from the pyramid level where a cell is about one pixel. (u, v) are in OpenCV's convention, as every K that projects
    onto a frame here (cv2's straightening of the fisheye, ROS's camera_info): integers at pixel centres, (0, 0) the
    first pixel's centre ([AKY]). Read as corner-based, every head-camera cell sat half a pixel up and left"""
    lvl = np.clip(np.floor(np.log2(np.maximum(px_per_cell, 1.0))), 0, LEVELS - 1).astype(int)
    out = np.zeros((len(u), 3), np.float32)
    for L in np.unique(lvl):
        k = lvl == L; s = 2.0 ** L; a = pyr[L]
        rows, cols = (v[k] + 0.5) / s - 0.5, (u[k] + 0.5) / s - 0.5  # a level-L pixel spans s full ones: its centre
                                                                   # at s (j + 1/2) - 1/2 in full-size pixels
        for ch in range(3):
            out[k, ch] = map_coordinates(a[..., ch], [rows, cols], order=1, mode="nearest")
    return out


def px_per_cell(cell_m, slant_m, f_px, height_m):
    """how many pixels a floor cell spans in the image, seen from height_m at this slant distance: across the view
    cell x f / slant, along it that times the sine of the viewing angle (height / slant); their geometric mean"""
    across = cell_m * f_px / np.maximum(slant_m, 1e-3)
    return across * np.sqrt(np.clip(height_m / np.maximum(slant_m, 1e-3), 0.0, 1.0))
