"""[d8d]'s standing look: a find too small for the detector at the whole frame's scale is looked for again in a crop
around the flag, enlarged; its box maps back to the frame, and a box over most of the crop (the floor) is dropped."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "arm"))
import wrist_refine as wr  # noqa: E402


def test_a_crop_box_maps_back_to_the_frame(monkeypatch):
    img = np.zeros((360, 640, 3), np.uint8); calls = []
    def fake(im, queries=None, thr=0.15):
        calls.append((im.shape, thr))
        if im.shape[:2] == (360, 639):                       # the crop, enlarged 3x (213 x 120 -> 639 x 360)
            return [((300.0, 150.0, 330.0, 180.0), 0.12, "a toy"), ((10.0, 5.0, 600.0, 350.0), 0.2, "a piece of cloth")]
        return []
    monkeypatch.setattr(wr, "owl_boxes", fake)
    out = wr.owl_boxes_zoomed(img, (304, 109), zooms=(3,))
    assert len(out) == 1 and calls[1][1] == wr.ZOOM_THR
    (x1, y1, x2, y2), sc, lab = out[0]
    assert abs(x1 - (int(np.clip(304 - 213 / 2, 0, 640 - 213)) + 100.0)) < 1e-6 and abs(y1 - (int(np.clip(109 - 60, 0, 240)) + 50.0)) < 1e-6
