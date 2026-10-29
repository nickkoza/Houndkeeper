"""Tests written to kill mutants that survived the suite (docs/research/test-quality-2026-10-03.md): the floor map error [nv5],
the floor pictures' agreement [FHA] and the detector's own test (detect_changes2 feats and tolerant_diff) that both use.
Each test pins one behaviour a plausible one-line bug would break while every earlier test still passed."""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def _floor(seed=3, h=300, w=400, tile=20):
    rng = np.random.default_rng(seed)
    base = np.repeat(np.repeat(rng.uniform(60, 200, (h // tile + 1, w // tile + 1, 3)), tile, 0), tile, 1)[:h, :w]
    return np.clip(base + rng.normal(0, 4, (h, w, 3)), 0, 255).astype(np.uint8)


# ---- the detector's own test --------------------------------------------------------------------------------------

def test_tolerant_diff_forgives_every_misalignment_up_to_the_slack_and_no_more():
    """every shift within +-slack cm, in both directions on both axes, reads as unchanged; 4 cm off does not"""
    from detect_changes2 import feats, tolerant_diff
    T = _floor(); V = np.ones(T.shape[:2], bool); c, r = feats(T, V); inner = (slice(30, -30), slice(30, -30))
    for dy in (-2, -1, 0, 1, 2):
        for dx in (-2, 0, 2):
            cs_, rs_ = np.roll(c, (dy, dx), (0, 1)), np.roll(r, (dy, dx), (0, 1))
            dC, _ = tolerant_diff(cs_, rs_, c, r, slack=2)
            assert dC[inner].max() < 1e-6, (dy, dx)
    dC, _ = tolerant_diff(np.roll(c, 4, 1), np.roll(r, 4, 1), c, r, slack=2)
    assert (dC[inner] > 0.045).mean() > 0.05


def test_tolerant_diff_brightening_is_signed_so_a_shadow_is_not_a_brightening():
    from detect_changes2 import tolerant_diff
    h, w = 60, 60; cb = np.full((h, w, 3), 1 / 3, np.float32); rb = np.ones((h, w), np.float32)
    up = rb.copy(); up[20:40, 20:40] += 0.5; down = rb.copy(); down[20:40, 20:40] -= 0.5
    _, br_up = tolerant_diff(cb, up, cb, rb); _, br_down = tolerant_diff(cb, down, cb, rb)
    assert abs(br_up[30, 30] - 0.5) < 1e-6 and abs(br_down[30, 30] + 0.5) < 1e-6


def test_feats_judge_a_toy_against_the_floor_round_it_not_against_itself():
    """the surroundings are ~25 cm wide: a 12 cm toy keeps nearly all its colour shift; a narrow surround would eat it"""
    from detect_changes2 import feats
    img = np.full((200, 200, 3), 120, np.uint8); toy = img.copy(); toy[94:106, 94:106] = (160, 100, 100)
    V = np.ones((200, 200), bool); c0, _ = feats(img, V); c1, _ = feats(toy, V)
    want = np.linalg.norm(np.array([160, 100, 100]) / 361 - np.array([120, 120, 120]) / 361)
    assert np.linalg.norm(c1[100, 100] - c0[100, 100]) > 0.9 * want


def test_feats_are_blind_to_the_floor_outside_what_was_seen():
    """what lies beyond the seen floor (black in a mosaic, or real floor) changes nothing inside it"""
    from detect_changes2 import feats
    T = _floor(); V = np.zeros(T.shape[:2], bool); V[50:250, 60:340] = True
    black = np.where(V[..., None], T, 0).astype(np.uint8)
    c1, r1 = feats(black, V); c2, r2 = feats(T, V)
    assert np.abs(c1 - c2)[V].max() < 1e-4 and np.abs(r1 - r2)[V].max() < 1e-4


# ---- the floor map error [nv5] ------------------------------------------------------------------------------------

def _none(T): return np.zeros(T.shape[:2], bool)


def test_the_truth_is_judged_over_the_maps_floor_even_beside_a_bright_rug():
    """[nv5] a red rug just beyond what the map saw must not tint the truth's surroundings and read as error"""
    import floor_fidelity as ff
    T = _floor().astype(np.float32); V = np.zeros(T.shape[:2], bool); V[50:250, 60:340] = True
    T[:, :60] = (220, 30, 30); T[:, 340:] = (220, 30, 30)
    seen = np.where(V[..., None], T, 0).astype(np.uint8)
    assert ff.map_error(seen, V, T, _none(T)) < 0.001


def test_what_is_excluded_is_not_judged():
    """[nv5] a wrong patch under the exclusion (rug, moved chairs, the band by the walls) is not error; uncovered it is
    (the exclusion reaches 60 cm past it: a change tints the surroundings the detector judges colour against)"""
    import floor_fidelity as ff
    T = _floor(); V = np.ones(T.shape[:2], bool); bad = T.copy(); bad[100:200, 150:250] = (200, 60, 60)
    ex = _none(T); ex[40:260, 90:310] = True
    assert ff.map_error(bad, V, T.astype(np.float32), ex) < 0.001
    assert ff.map_error(bad, V, T.astype(np.float32), _none(T)) > 0.05


def test_the_maps_ragged_edge_is_not_judged():
    """[nv5] the seen floor's outer 5 cm (where a mosaic smears) is left out"""
    import floor_fidelity as ff
    T = _floor(); V = np.zeros(T.shape[:2], bool); V[50:250, 60:340] = True
    seen = np.where(V[..., None], T, 0).astype(np.uint8)
    ring = V & ~np.pad(np.ones((194, 274), bool), ((53, 53), (63, 63)))
    seen[ring] = (230, 40, 200)
    assert ff.map_error(seen, V, T.astype(np.float32), _none(T)) < 0.001


def test_the_error_threshold_is_the_confirmers():
    """[nv5] a faint tint (colour shift ~1.5x the confirmer's tc) over a patch counts; one at ~0.5x does not"""
    import floor_fidelity as ff, confirm_stream as cs
    from detect_changes2 import feats
    T = _floor(); V = np.ones(T.shape[:2], bool); patch = (slice(110, 190), slice(160, 240)); share = 80 * 80 / V.size
    c0, _ = feats(T, V)

    def tinted(k):
        m = T.astype(np.float32).copy(); m[patch] *= np.array([1 + k, 1 - k / 2, 1 - k / 2]); return np.clip(m, 0, 255).astype(np.uint8)

    def shift(k): c1, _ = feats(tinted(k), V); return float(np.median(np.linalg.norm(c1 - c0, axis=-1)[patch]))
    lo = next(k for k in np.arange(0.01, 1, 0.01) if shift(k) > 0.5 * cs.CONFIRM["tc"])
    hi = next(k for k in np.arange(0.01, 1, 0.01) if shift(k) > 1.5 * cs.CONFIRM["tc"])
    assert ff.map_error(tinted(hi), V, T.astype(np.float32), _none(T)) > 0.5 * share
    assert ff.map_error(tinted(lo), V, T.astype(np.float32), _none(T)) < 0.1 * share


def test_a_map_off_by_more_than_the_slack_is_error_and_within_it_is_not():
    """[nv5] the confirmer's slack (2 cm) and no more: a map 2 cm off is the floor, one 4 cm off is not"""
    import floor_fidelity as ff
    T = _floor(); V = np.ones(T.shape[:2], bool)
    assert ff.map_error(np.roll(T, (2, -2), (0, 1)), V, T.astype(np.float32), _none(T)) < 0.005
    assert ff.map_error(np.roll(T, 4, 1), V, T.astype(np.float32), _none(T)) > 0.02


def _fine_floor(seed=5, h=300, w=400):
    """a floor with detail at a few cm, so a placement can be read to a fraction of a cell"""
    from scipy import ndimage as ndi
    g = ndi.gaussian_filter(np.random.default_rng(seed).normal(0, 1, (h, w)), 2.5)
    return np.repeat(np.clip(128 + 60 * g / g.std(), 0, 255)[..., None], 3, -1).astype(np.float32)


def test_the_maps_placement_reads_a_shift_the_error_forgives():
    """[nv5], [G9q] (decision [qLu]'s control): a map placed 15 mm east and 8 mm south of the true floor is no error by
    the detector's test (2 cm of slack), and its placement reads that shift, patch by patch, to a millimetre or two"""
    import floor_fidelity as ff
    from scipy import ndimage as ndi
    T = _fine_floor(); V = np.ones(T.shape[:2], bool)
    pic = ndi.shift(T, (0.8, 1.5, 0), order=3, mode="nearest")          # rows down = south, cols right = east, in cells
    assert ff.map_error(pic, V, T, _none(T)) < 0.01
    s = ff.placement(pic, V, T, _none(T)); a = np.array(list(s.values()))
    assert len(a) > 20 and np.abs(np.median(a, 0) - [15.0, -8.0]).max() < 1.0 and np.abs(a - [15.0, -8.0]).max() < 3.0
    summ = ff.placement_summary(s, ff.placement(T, V, T, _none(T)))
    assert summ["abs_mm_mean"] == pytest.approx(17.0, abs=1.0) and summ["paired_mm"] == pytest.approx(17.0, abs=1.0)


def test_a_patch_without_a_clear_match_is_left_out_of_the_placement():
    """[nv5] a map whose floor is something else entirely gives (almost) no placement, not a guess"""
    import floor_fidelity as ff
    T = _fine_floor(); V = np.ones(T.shape[:2], bool)
    assert len(ff.placement(_fine_floor(seed=9), V, T, _none(T))) < 0.05 * len(ff.placement(T, V, T, _none(T)))


def _traj(path, odom, true):
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(["wall_t", "odom_x", "odom_y", "true_x", "true_y"])
        for k, ((ox, oy), (tx, ty)) in enumerate(zip(odom, true)): w.writerow([k * 0.1, ox, oy, tx, ty])


def test_the_maps_frame_comes_from_its_path_and_lands_points_where_they_truly_are(tmp_path):
    """[nv5] the rigid move that lays the night's path on its true path, and the picture resampled by it: a mark at a
    picture place appears at R p + t in the sim's frame"""
    import floor_fidelity as ff
    th, t = math.radians(3.0), np.array([0.12, -0.07]); R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
    s = np.linspace(0, 2 * math.pi, 200); odom = np.c_[1.2 * np.cos(s), 0.8 * np.sin(2 * s)]
    _traj(tmp_path / "traj.csv", odom, odom @ R.T + t)
    R_, t_ = ff.frame_from_path(None, tmp_path / "traj.csv")
    assert np.allclose(R_, R, atol=1e-6) and np.allclose(t_, t, atol=1e-6)
    H, W = 300, 400; img = np.zeros((H, W), np.float32); p = np.array([0.5, 0.3])          # a mark 0.5 m right, 0.3 m up
    img[int(round(H / 2 - p[1] / ff.RES - 0.5)), int(round(W / 2 + p[0] / ff.RES - 0.5))] = 1.0
    out = ff.to_true_frame(img, R_, t_, order=0); r, c = np.unravel_index(np.argmax(out), out.shape)
    got = np.array([-W * ff.RES / 2 + (c + 0.5) * ff.RES, H * ff.RES / 2 - (r + 0.5) * ff.RES])
    assert np.linalg.norm(got - (R @ p + t)) < 0.015


def test_a_straight_walk_still_gives_a_turn_not_a_mirror(tmp_path):
    """[nv5] a path along one line leaves the fit's second axis free: the frame must stay a rotation (det +1)"""
    import floor_fidelity as ff
    for walk_deg in (0, 45, 90, 135):                     # (the SVD's free axis comes out mirrored for some of these,
        for deg in range(-30, 31, 3):                     # which ones depending on round-off)
            th = math.radians(deg); R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            u = np.array([math.cos(math.radians(walk_deg)), math.sin(math.radians(walk_deg))])
            odom = np.outer(np.linspace(-2, 2, 100), u)
            _traj(tmp_path / "traj.csv", odom, odom @ R.T + [0.05, 0.02])
            R_, _ = ff.frame_from_path(None, tmp_path / "traj.csv")
            assert np.linalg.det(R_) > 0.99, (walk_deg, deg)


def test_the_score_finds_the_frame_and_each_patchs_own_warp():
    """[FHA] a picture laid 16 cm off the truth is laid back by lay_on (patches then agree with ~0 warp); a picture
    whose patch is 6 cm off locally reports ~6 cm of warp there"""
    import floor_fidelity as ff
    T = _floor(seed=5, h=400, w=480, tile=12); V = np.ones(T.shape[:2], bool)
    s, (td, dr, dc) = ff.score(np.roll(T, (0, 16), (0, 1)).astype(np.float32), V, T.astype(np.float32))
    assert abs(td) < 0.6 and (dr, dc) == (0, -16) and np.median(s[:, 0]) > 0.9 and np.median(s[:, 1]) < 1.5
    s, _ = ff.score(np.roll(T, (0, 6), (0, 1)).astype(np.float32), V, T.astype(np.float32), frame=(np.eye(2), np.zeros(2)))
    assert np.median(s[:, 0]) > 0.9 and abs(np.median(s[:, 1]) - 6) < 1.0


def test_the_structures_mask_covers_each_wall_cell_where_the_sim_built_it():
    """[nv5] the one mask every map is judged with: each of make_house's wall cells, at its own place on the grid"""
    import floor_fidelity as ff, make_house as mh
    shape = (980, 1680); occ = ff.truth_structures(70, shape); g = mh.base_grid()
    cells = [(x, y) for y in range(mh.H) for x in range(mh.W) if g[y][x]]
    for x, y in cells[:: max(1, len(cells) // 60)]:
        cx, cy = mh.cell_to_world(x, y)
        for ox, oy in ((0, 0), (0.12, 0.12), (-0.12, -0.12)):
            r, c = int((shape[0] * ff.RES / 2 - (cy + oy)) / ff.RES), int((cx + ox + shape[1] * ff.RES / 2) / ff.RES)
            if 0 <= r < shape[0] and 0 <= c < shape[1]: assert occ[r, c], (x, y, ox, oy)


# ---- the floor pictures' agreement [FHA] ----------------------------------------------------------------------------

def _pic(path, rgb, V, occ):
    dist = np.where(V, 1.0, np.inf).astype(np.float16); np.savez(path, rgb=rgb, dist=dist, occ=occ)


def test_a_night_with_one_truth_file_has_no_truth(tmp_path):
    import floor_agreement as fa
    (tmp_path / "nights_s95_n1.truth.json").write_text("{}")
    assert not fa.has_truth(tmp_path, 95)


def test_agreement_judges_only_floor_both_saw_away_from_structures_both_mapped(tmp_path):
    """[FHA] the judged floor: seen in both pictures, less 20 cm round structures mapped in both; a change only one
    picture saw, or a structure only one mapped, is not judged"""
    import floor_agreement as fa, confirm_stream as cs
    from scipy import ndimage as ndi
    h, w = 200, 300; T = _floor(h=h, w=w)
    Va = np.ones((h, w), bool); Vb = Va.copy(); Vb[:, 180:] = False                        # b never saw the right part
    wall = np.zeros((h, w), bool); wall[90:110, 20:40] = True                              # mapped by both
    only_a = np.zeros((h, w), bool); only_a[20:30, 150:170] = True                         # mapped by one
    b = T.copy(); b[:, 180:] = 0
    a = T.copy(); a[40:160, 255:290] = (200, 50, 50)                                       # a change b never saw
    _pic(tmp_path / "a.npz", a, Va, wall | only_a); _pic(tmp_path / "b.npz", b, Vb, wall)
    res, _ = fa.agreement(tmp_path, None, tmp_path / "a.npz", tmp_path / "b.npz")
    band = ndi.distance_transform_edt(~wall) * cs.RES < cs.CONFIRM["band_m"]
    assert res["all"][1] == int((Va & Vb & ~band).sum())
    assert res["all"][0] < 0.005


def test_with_the_truth_the_rug_and_moved_chairs_are_split_from_the_open_floor(tmp_path):
    """[FHA] a chair moved between the nights (both its places, +30 cm for its shadow) and the rug (+5 cm) are reported
    apart; a chair that stayed put is open floor"""
    import floor_agreement as fa, confirm_stream as cs
    H, W = int(2 * cs.HY / cs.RES), int(2 * cs.HX / cs.RES)
    chair = dict(id="c1", x=1.0, y=1.0, yaw=0.0, w=0.4, d=0.4); stay = dict(id="c2", x=-3.0, y=-2.0, yaw=0.0, w=0.4, d=0.4)
    rug = dict(x=-1.0, y=2.0, w=1.0, h=0.6)
    for k, x in ((1, 1.0), (2, 2.0)):
        (tmp_path / f"nights_s5_n{k}.truth.json").write_text(json.dumps(dict(movables=[dict(chair, x=x), stay], rug=rug)))
    rugm, chairs = fa.regions(tmp_path, 5, (1, 2), (H, W))
    rc = lambda x, y: (int((cs.HY - y) / cs.RES), int((x + cs.HX) / cs.RES))
    assert chairs[rc(1.0, 1.0)] and chairs[rc(2.0, 1.0)] and chairs[rc(1.0, 1.0 + 0.2 + 0.25)]   # both places, grown
    assert not chairs[rc(-3.0, -2.0)] and not chairs[rc(1.5, 1.0 + 0.2 + 0.4)]
    assert rugm[rc(-1.0, 2.0)] and rugm[rc(-1.0 - 0.5 - 0.03, 2.0)] and not rugm[rc(-1.0 - 0.5 - 0.08, 2.0)]
    assert not rugm[rc(-1.0, -2.0)]                                                         # not mirrored in y
    T = _floor(h=H, w=W, tile=25); V = np.ones((H, W), bool); occ = np.zeros((H, W), bool)
    b = T.copy(); b[chairs] = np.roll(T, 10, 1)[chairs]                                    # the floor differs only by the chairs
    _pic(tmp_path / "a.npz", T, V, occ); _pic(tmp_path / "b.npz", b, V, occ)
    res, _ = fa.agreement(tmp_path, 5, tmp_path / "a.npz", tmp_path / "b.npz")
    assert set(res) == {"all", "open", "rug", "chairs"} and res["open"][0] < 0.002 and res["chairs"][0] > 0.1


def test_a_nights_error_leaves_out_the_band_by_the_walls(tmp_path, monkeypatch):
    """[nv5] error_of: the map laid by its path, judged over the open floor only: wrong colour within the confirmer's
    band of a wall (where the walls' own shading and the mosaic's edge live) is not error, and its area is not judged"""
    import floor_fidelity as ff, floor_agreement as fa, confirm_stream as cs
    from scipy import ndimage as ndi
    T = _floor(); H, W = T.shape[:2]; occ = np.zeros((H, W), bool); occ[120:180, 180:220] = True     # a pillar
    near = ~occ & (ndi.distance_transform_edt(~occ) * ff.RES < 0.1)
    pic = T.copy(); pic[occ] = 30; pic[near] = (200, 40, 40)
    np.savez(tmp_path / "pic.npz", rgb=pic, dist=np.ones((H, W), np.float16))
    s = np.linspace(0, 2 * math.pi, 100); path = np.c_[np.cos(s), 0.5 * np.sin(s)]
    _traj(tmp_path / "traj.csv", path, path)
    monkeypatch.setattr(ff, "truth", lambda seed, night, shape: T.astype(np.float32))
    monkeypatch.setattr(ff, "truth_structures", lambda seed, shape: occ)
    monkeypatch.setattr(fa, "regions", lambda run, seed, nights, shape: (np.zeros(shape, bool), np.zeros(shape, bool)))
    err, m2 = ff.error_of(tmp_path / "pic.npz", 1, 1, tmp_path / "traj.csv", None, tmp_path)
    band = ndi.distance_transform_edt(~occ) * ff.RES < cs.CONFIRM["band_m"]
    assert err < 0.02 and abs(m2 - (~band).sum() * ff.RES ** 2) < 0.15     # (the red still tints the floor just past
                                                                          # the band a little: ~0.5%)
