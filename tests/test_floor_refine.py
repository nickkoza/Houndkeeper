"""[G9q]: each view registered against the floor picture built without it, on synthetic floors (floor_refine.py): the
picture a view is registered against leaves it out; the lidar's constraints keep the house's shape however hard the
images pull; a view never moves further than the bound (no jump of a whole tile on a repeating floor); a pass that makes
the two nights' pictures agree worse is not kept."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))

import floor_refine as fr   # noqa: E402

RES = fr.RES


def texture(h, w, seed=0, blob=6):
    """a random floor with detail at a few cm, grey 0-255"""
    from scipy import ndimage as ndi
    g = ndi.gaussian_filter(np.random.default_rng(seed).normal(0, 1, (h, w)), blob / 3)
    return np.clip(128 + 60 * g / g.std(), 0, 255)


def tiles(h, w, period=30, seed=1):
    """a repeating floor: one 30 cm tile (its own pattern and grout) over and over"""
    t = texture(period, period, seed, blob=3); t[:2, :] = 30; t[:, :2] = 30
    return np.tile(t, (h // period + 2, w // period + 2))[:h, :w]


def box_at(r0, c0, h, w):
    return (r0, r0 + h, c0, c0 + w)


def centre(box):
    return np.array([-fr.HX + (box[2] + box[3]) / 2 * RES, fr.HY - (box[0] + box[1]) / 2 * RES])


# --- leave one out ---------------------------------------------------------------------------------------------------

def test_the_picture_a_view_is_registered_against_leaves_it_out():
    """[G9q] each view's reference is the picture built without it: where only that view saw the floor the reference
    has nothing, where others saw it the reference is theirs alone, and changing the view changes nothing of its own
    reference (but does change the others')"""
    floor = texture(300, 500)
    boxes = [box_at(100, 100, 120, 120), box_at(100, 140, 120, 120), box_at(100, 300, 120, 120)]   # 0 and 1 overlap; 2 alone
    views = [fr.View(np.repeat(floor[b[0]:b[1], b[2]:b[3], None], 3, -1).astype(np.uint8), b, t, 1) for t, b in enumerate(boxes)]
    views[1].rgb[:] = np.clip(views[1].rgb.astype(int) + 40, 0, 255).astype(np.uint8)   # view 1 brighter: the mix is told apart
    F = fr.Floor([v.placed() for v in views])
    box, ref, m = F.loo(2, [2], pad=4)
    assert not m.any(), "view 2 alone saw its floor: without it, nothing"
    box, ref, m = F.loo(0, [0], pad=4)
    g1 = views[1].rgb.mean(-1)
    r, c = 150 - box[0], 180 - box[2]                    # a cell both 0 and 1 saw: the reference is view 1's own value
    assert m[r, c] and abs(ref[r, c] - g1[150 - 100, 180 - 140]) < 1.0
    before = ref.copy()
    views[0].rgb[:] = 0; views[0].rgb[5:-5, 5:-5] = 200     # view 0 changed: its own reference is unchanged
    F2 = fr.Floor([v.placed() for v in views]); _, ref2, m2 = F2.loo(0, [0], pad=4)
    assert np.allclose(ref2[m2], before[m2]) and (m2 == m).all()
    _, r1, m1 = F2.loo(1, [1], pad=4); _, r1b, _ = F.loo(1, [1], pad=4)
    assert not np.allclose(r1[m1], r1b[m1]), "view 1's reference is view 0's, so it changed"


def test_leave_out_takes_the_view_and_its_own_nights_neighbours_in_time():
    """[G9q] decision [ObC]: a view's near neighbours of the same night carry its own pose error, so they go with it;
    the other night's views at the same moment stay in"""
    ts = np.array([0.0, 1.0, 3.0, 9.0, 1.0, 2.0]); night = np.array([1, 1, 1, 1, 2, 2])
    assert sorted(fr.leave_out(1, ts, night, 5.0)) == [0, 1, 2]
    assert sorted(fr.leave_out(1, ts, night, 0.0)) == [1]


def test_registration_finds_how_far_a_view_sits_off_the_floor():
    """[G9q] a view laid 2 cm east, 1 cm south and 0.3 deg turned off the floor is told to move back by as much"""
    floor = texture(400, 400, seed=3)
    box = box_at(140, 140, 120, 120)
    v = fr.View(np.repeat(floor[box[0]:box[1], box[2]:box[3], None], 3, -1).astype(np.uint8), box, 0.0, 1)
    err = np.array([0.02, -0.01, math.radians(0.3)])
    pv = v.placed(err)                                          # the view, misplaced
    ref = np.repeat(floor[..., None], 3, -1).astype(np.uint8)
    F = fr.Floor([fr.View(ref[100:300, 100:300], box_at(100, 100, 200, 200), 5.0, 2).placed()])
    m = fr.measure(pv, F, [], search_cells=6)
    assert m is not None
    assert abs(m.move[0] + 0.02) < 0.004 and abs(m.move[1] - 0.01) < 0.004
    assert abs(math.degrees(m.move[2]) + 0.3) < 0.12


# --- the lidar keeps the shape ----------------------------------------------------------------------------------------

def _line(nk=12, step=0.25):
    P = np.c_[np.arange(nk) * step, np.zeros(nk)]
    return fr.Keyframes(P=P, night=np.ones(nk, int), trav=np.arange(nk) * step, ts=np.arange(nk, dtype=float))


def _views_on(K, pull):
    """a view a little ahead of each keyframe, linked to it; those in `pull` asked to move 4 cm sideways"""
    nv = len(K.P); o = K.P + [0.0, 1.0]
    tgt = np.full((nv, 3), np.nan); tgt[pull] = [0.0, 0.04, 0.0]
    return fr.ViewTerms(o=o, a=np.arange(nv), b=np.arange(nv), f=np.zeros(nv), dd=np.full(nv, 0.05),
                        target=tgt, info=np.full(nv, (1 / 0.005) ** 2), info_th=np.full(nv, 0.0))


def _lidar(K, sd=0.002):
    I = np.diag([1 / sd ** 2, 1 / sd ** 2, 1 / math.radians(0.05) ** 2])
    return [(i, j, I) for i in range(len(K.P)) for j in range(i + 1, len(K.P)) if K.P[j, 0] - K.P[i, 0] < 1.01]


def test_the_lidar_pinned_shape_does_not_bend():
    """[G9q] views in the middle of a straight wall pulled 4 cm sideways: with the lidar's constraints in the solve the
    keyframes stay a straight line (the house's shape), and without them the same pull bends it"""
    K = _line(); pull = [4, 5, 6, 7]; V = _views_on(K, pull)
    X, Y = fr.solve(K, _lidar(K), V)
    bend = lambda X: float(np.ptp(X[:, 1] - np.polyval(np.polyfit(K.P[:, 0], X[:, 1], 1), K.P[:, 0])))
    assert bend(X) < 0.001, f"the lidar's shape bent {bend(X) * 1000:.1f} mm"
    X0, _ = fr.solve(K, [], V)
    assert bend(X0) > 0.005, "the test's pull bends a chain the lidar doesn't hold"


def test_no_image_evidence_moves_nothing():
    """[G9q] the lidar's terms are the shape as solved: with no view measured, the solve leaves every pose where it was"""
    K = _line(); V = _views_on(K, [])
    X, Y = fr.solve(K, _lidar(K), V)
    assert np.abs(X).max() < 1e-9 and np.abs(Y).max() < 1e-9


# --- the move bound ---------------------------------------------------------------------------------------------------

def test_the_move_bound_stops_a_one_tile_jump_on_a_repeating_floor():
    """[G9q] decision [ObC]: on a floor of 30 cm tiles a view whose best match anywhere is one whole tile over (a stain
    seen there) moves at most the bound, to its own tile, never the 30 cm; searched without the bound it jumps"""
    floor = tiles(400, 500)
    ref = floor.copy(); view = floor.copy()
    ref[190:230, 250:290] = 250                                 # a bright stain the reference has in one tile...
    view[190:230, 220:260] = 250                                # ...that the view (one tile west) has in the tile before
    box = box_at(150, 170, 140, 160)
    v = fr.View(np.repeat(view[box[0]:box[1], box[2]:box[3], None], 3, -1).astype(np.uint8), box, 0.0, 1)
    F = fr.Floor([fr.View(np.repeat(ref[50:350, 50:450, None], 3, -1).astype(np.uint8), box_at(50, 50, 300, 400), 9.0, 2).placed()])
    m = fr.measure(v.placed(), F, [], search_cells=fr.cells(fr.MOVE_BOUND_M))
    assert m is None or math.hypot(*m.move[:2]) <= fr.MOVE_BOUND_M + 1e-9
    wide = fr.measure(v.placed(), F, [], search_cells=36, min_lead=0.0)
    assert wide is not None and abs(wide.move[0] - 0.30) < 0.02, "without the bound the stain wins: a one-tile jump"


def test_a_view_already_at_the_bound_cannot_be_moved_past_it():
    """[G9q] the search reaches only what is left of the bound: a view moved 7 of its 8 cm, whose floor lies 3 cm further,
    finds no lock (the match is beyond its reach) rather than moving 10 cm"""
    floor = texture(400, 400, seed=5)
    box = box_at(140, 140, 120, 120)
    v = fr.View(np.repeat(floor[box[0]:box[1], box[2]:box[3], None], 3, -1).astype(np.uint8), box, 0.0, 1)
    F = fr.Floor([fr.View(np.repeat(floor[100:300, 100:300, None], 3, -1).astype(np.uint8), box_at(100, 100, 200, 200), 9.0, 2).placed()])
    now = np.array([0.07, 0.0, 0.0])
    pv = v.placed(now + [0.03, 0.0, 0.0])                     # where it lies: 10 cm east of its floor
    reach = fr.reach_cells(now)
    assert reach == fr.cells(fr.MOVE_BOUND_M - 0.07)
    assert fr.measure(pv, F, [], search_cells=reach) is None
    assert fr.measure(pv, F, [], search_cells=12) is not None   # the floor is there to be found, past the bound


def test_the_solve_never_moves_a_view_past_the_bound():
    K = _line(); V = _views_on(K, [3]); V.target[3] = [0.0, 0.5, 0.0]; V.info[3] = 1e9
    X, Y = fr.solve(K, [], V, tie_scale=1e3)
    assert np.hypot(Y[:, 0], Y[:, 1]).max() <= fr.MOVE_BOUND_M + 1e-9


# --- a pass is kept only if the nights agree better ---------------------------------------------------------------------

def test_a_pass_that_worsens_the_nights_agreement_is_rejected():
    """[G9q] passes kept while the two nights' pictures agree better; the first that agrees worse is thrown away and
    refining stops, keeping the pass before it"""
    shares = {0: 0.10, 1: 0.08, 2: 0.09, 3: 0.01}
    best, hist = fr.refine(0, lambda s: s + 1, lambda s: shares[s], passes=3, min_gain=0.0)
    assert best == 1
    assert [(h["pass"], h["kept"]) for h in hist] == [(0, True), (1, True), (2, False)]


def test_refining_stops_when_the_agreement_stops_improving():
    shares = {0: 0.10, 1: 0.0999, 2: 0.05}
    best, hist = fr.refine(0, lambda s: s + 1, lambda s: shares[s], passes=3, min_gain=0.01)
    assert best == 0 and [h["kept"] for h in hist] == [True, False]


def test_the_agreement_is_the_truth_free_nights_disagreement(tmp_path):
    """[G9q] the pass test is floor_agreement's share of the judged floor reading as changed, without truth"""
    import floor_agreement as fa
    rng = np.random.default_rng(2); h, w = 160, 240
    tl = np.repeat(np.repeat(rng.uniform(60, 200, (h // 20 + 1, w // 20 + 1, 3)), 20, 0), 20, 1)[:h, :w]
    a = np.clip(tl + rng.normal(0, 2, (h, w, 3)), 0, 255).astype(np.uint8); b = np.roll(a, 3, 1)
    occ = np.zeros((h, w), bool); occ[:, :10] = True; V = np.ones((h, w), bool)
    for k, x in (("a", a), ("b", b)):
        np.savez(tmp_path / f"{k}.npz", rgb=x, dist=np.ones((h, w), np.float16), occ=occ)
    want = fa.agreement(tmp_path, None, tmp_path / "a.npz", tmp_path / "b.npz")[0]["all"][0]
    assert fr.agreement_share(a, V, b, V, occ) == pytest.approx(want)


# --- the bench's positive control --------------------------------------------------------------------------------------

def test_the_benchs_plant_moves_only_its_block_by_the_shift_and_turn_given():
    """[G9q] the bench's positive control (floor_refine_bench --plant): between the block's two fractions of the night's
    time each correction is moved and turned about the path's mean there, and outside it nothing moves"""
    import floor_refine_bench as frb
    from houndkeeper.localize import Localizer
    times = np.linspace(0.0, 100.0, 51); Ts = np.repeat(np.eye(3)[None], len(times), 0)
    pt = np.linspace(0.0, 100.0, 201); pxy = np.c_[pt / 10.0, np.zeros_like(pt)]           # 1 m every 10 s along x
    T, M = frb.plant(times, Ts, pt, pxy, 0.4, 0.6, 0.05, -0.02, 1.0)
    L = Localizer(times=list(T), Ts=list(M))
    for t in (0.0, 20.0, 39.9, 60.1, 80.0, 100.0):
        assert np.allclose(L.correction(t), np.eye(3), atol=1e-9)                            # outside the block
    o = np.array([5.0, 0.0])                                                                  # the block's path, its mean
    for t in (40.0, 50.0, 60.0):
        C = L.correction(t)
        assert np.allclose(C[:2, :2] @ o + C[:2, 2], o + [0.05, -0.02], atol=1e-6)
        assert math.degrees(math.atan2(C[1, 0], C[0, 0])) == pytest.approx(1.0)
    assert frb.parse_plant("1:0.40-0.55:0.05,0,1.0") == (1, 0.40, 0.55, 0.05, 0.0, 1.0)
    with pytest.raises(SystemExit): frb.parse_plant("3:0.4-0.5:0,0,0")
