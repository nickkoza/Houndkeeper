"""The floor geometry's sign conventions, on synthetic floors ([FHA], [WFI]): the stitch says which way a view must move
to lie on another, and the confirmer's tracker turns and shifts the pose back the way the floor snap found it off.
(First checked synthetically in the pipeline critique of 3 Oct 2026; nothing held them before.)"""
import math
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy import ndimage as ndi

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def _texture():
    rng = np.random.default_rng(0)
    t = ndi.gaussian_filter(rng.random((600, 800)), 2.0); t = (t - t.min()) / (t.max() - t.min())
    return np.clip(t * 255, 0, 255)


def _paint(tex, sx, sy, th_deg, box, res):
    """the floor as a view whose pose is off by (sx, sy) m and a turn th about the box's centre (the texture's) paints it"""
    r0, r1, c0, c1 = box
    img = ndi.rotate(tex, th_deg, reshape=False, order=1) if th_deg else tex
    img = np.roll(img, (int(round(-sy / res)), int(round(sx / res))), (0, 1))
    return np.repeat(img[r0:r1, c0:c1][..., None], 3, -1).astype(np.uint8)


@pytest.mark.parametrize("sx, sy, th", [(0.03, 0.02, 0.0), (-0.05, 0.0, 0.0), (0.0, 0.0, 1.0)])
def test_the_stitch_says_a_view_moves_back_by_what_it_was_off(sx, sy, th):
    """[FHA], [WFI]: view j painted off by (sx, sy) and th: measure says j moves by (-sx, -sy) and -th"""
    import confirm_stream as cs, stitch_floor as sf
    tex = _texture(); box = (150, 450, 250, 550)                          # centred on the texture's centre (300, 400)
    pi_, pj = _paint(tex, 0, 0, 0, box, cs.RES), _paint(tex, sx, sy, th, box, cs.RES)
    sf._V = cs.Views(C=[[0, 0], [0.5, 0]], F=[[1, 0], [1, 0]], sls=[box, box], patches=[pi_, pj], ts=[0.0, 10.0])
    m = sf.measure((0, 1))
    assert m is not None and m[7] > 0.9
    assert abs(m[4] + sx) < 0.003 and abs(m[5] + sy) < 0.003 and abs(math.degrees(m[6]) + th) < 0.1


def _window(cs, n=201):
    r0 = cs.H // 2 - n // 2; c0 = cs.W // 2 - n // 2
    return (slice(r0, r0 + n), slice(c0, c0 + n))


def _centroid_world(cs, img, P):
    r, c = ndi.center_of_mass(np.clip(img, 0, None))
    return np.array([-cs.HX + (P[1].start + c + 0.5) * cs.RES, cs.HY - (P[0].start + r + 0.5) * cs.RES])


def test_the_trackers_turn_undoes_the_snaps():
    """[FHA], [WFI]: ndi.rotate's +td turns the floor anticlockwise (north up); a frame whose floor sits turned by td
    from the reference is turned back by the tracker's step (track_rot_sign -1, the confirmer's default)"""
    import confirm_stream as cs
    P = _window(cs); n = P[0].stop - P[0].start
    ref = np.zeros((n, n), np.float32); ref[n // 2, n // 2 + 30] = 1.0       # a mark 30 cells east of the window's middle
    ref = ndi.gaussian_filter(ref, 2.0)
    mid = np.zeros((n, n), np.float32); mid[n // 2, n // 2] = 1.0; pc = _centroid_world(cs, mid, P)   # the window's middle
    north = np.zeros((n, n), np.float32); north[n // 2 - 30, n // 2] = 1.0
    west = _centroid_world(cs, ndi.rotate(north, 90, reshape=False, order=1), P) - pc
    assert west[0] < -0.25 and abs(west[1]) < 0.01                         # north turned +90 lands west: anticlockwise
    td = 10.0
    frame = ndi.rotate(ref, td, reshape=False, order=1)                   # tonight's floor: the reference turned by td
    seen = _centroid_world(cs, frame, P)
    true = _centroid_world(cs, ref, P)
    S = cs.track_step(td, 0, 0, P, gain=1.0)
    back = S[:2, :2] @ seen + S[:2, 2]
    assert np.hypot(*(back - true)) < 0.005 < np.hypot(*(seen - true))
    S_wrong = cs.track_step(td, 0, 0, P, gain=1.0, rot_sign=1.0)
    assert np.hypot(*(S_wrong[:2, :2] @ seen + S_wrong[:2, 2] - true)) > 1.9 * np.hypot(*(seen - true))   # turned on


def test_the_trackers_shift_undoes_the_snaps():
    """[FHA], [WFI]: the snap rolls the reference by (-dr, -dc) cells onto the frame; the tracker shifts the pose back"""
    import confirm_stream as cs
    P = _window(cs); n = P[0].stop - P[0].start
    ref = np.zeros((n, n), np.float32); ref[n // 2 + 7, n // 2 - 11] = 1.0
    for dr, dc in ((3, -5), (-4, 6)):
        frame = np.roll(ref, (-dr, -dc), (0, 1))
        seen, true = _centroid_world(cs, frame, P), _centroid_world(cs, ref, P)
        S = cs.track_step(0.0, dr, dc, P, gain=1.0)
        assert np.hypot(*(S[:2, :2] @ seen + S[:2, 2] - true)) < 1e-6


def test_the_confirmers_default_tracks_with_the_checked_sign():
    """[WFI]: the default the live robot and the bench take, and the tracker's own fallback, are the checked sign"""
    import confirm_stream as cs, inspect
    assert cs.CONFIRM["track_rot_sign"] == -1.0
    assert inspect.signature(cs.track_step).parameters["rot_sign"].default == -1.0
