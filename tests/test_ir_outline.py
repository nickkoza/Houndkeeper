"""The infrared trial's outline: flash shadows ring what stands on the floor, a printed pattern nothing (waymark [6hI])."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))


def _scene():
    import ir_camera as ir
    H, W = 200, 300
    floor = 60 + 40 * ((np.arange(W) // 12) % 2)[None, :].repeat(H, 0)     # a striped floor: pattern, not shape
    toy = np.zeros((H, W), bool); toy[80:130, 100:190] = True               # 90 x 50 px standing on it
    frames = [np.full((H, W), 5, np.uint8)]
    for dv in ir.led_image_dirs():                                          # each LED's shadow on the toy's far side
        f = floor.astype(float).copy()
        sh = np.zeros_like(toy)
        for s in range(1, 7):
            sh |= np.roll(toy, (int(round(s * dv[1])), int(round(s * dv[0]))), axis=(0, 1))
        f[sh & ~toy] = 8; f[toy] = 150
        frames.append(np.clip(f + 5, 0, 255).astype(np.uint8))
    return frames, toy


def test_a_part_grows_to_the_whole_toy_and_the_stripes_dont_count():
    import ir_camera as ir
    frames, toy = _scene()
    part = np.zeros_like(toy); part[95:110, 110:125] = True                 # the detector boxed a part of it
    out = ir.outline(frames, part, np.zeros_like(toy))
    assert out is not None
    iou = (out & toy).sum() / (out | toy).sum()
    assert iou > 0.85, iou


def test_nothing_standing_gives_no_outline():
    import ir_camera as ir
    frames, toy = _scene()
    flat = [frames[0]] + [np.full_like(frames[1], 100)] * 4
    seed = np.zeros_like(toy); seed[95:110, 110:125] = True
    assert ir.outline(flat, seed, np.zeros_like(toy)) is None               # it would spread over the whole floor
