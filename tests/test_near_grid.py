"""The near floor compared on a finer grid ([Ot0]), on synthetic floors: the grid divides the 1 cm one, sizes in cm keep
their size on the floor, the views carry their grid, and a near find beside a 1 cm find is that find."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def test_a_finer_grid_divides_the_1_cm_grid():
    """[Ot0]: 5 mm splits each 1 cm cell 2 x 2; a size that does not divide 1 cm is refused"""
    import confirm_stream as cs
    assert cs.grid(0.01) == (cs.H, cs.W)
    assert cs.grid(0.005) == (2 * cs.H, 2 * cs.W)
    with pytest.raises(SystemExit):
        cs.grid(0.003)
    x = np.arange(6).reshape(2, 3)
    assert cs.upsample(x, 2).shape == (4, 6) and cs.upsample(x, 2)[1, 1] == x[0, 0] and cs.upsample(x, 2)[3, 5] == x[1, 2]


def _confirmer(cs, res, **over):
    a = dict(cs.CONFIRM, view_noise=0.0, fuse=0.0, dark=0.0, min_cm2=float(cs.MIN_PX), small_px=0.0); a.update(over); a = SimpleNamespace(**a)
    H, W = cs.grid(res); z = np.zeros((H, W), bool)
    return cs.Confirmer(np.zeros((H, W, 3), np.uint8), ~z, z, np.zeros((H, W), np.float32),
                        (800.0, 800.0, 640.0, 360.0), (1280, 720), a, res=res)


def _spot(cf, side_m=0.01, n=8):
    """n views of a 60 cm patch with a square change of side side_m in its middle, well over the threshold"""
    k = cf.k; ph = pw = 60 * k; sl = (slice(400 * k, 400 * k + ph), slice(800 * k, 800 * k + pw))
    pr, pc_ = np.nonzero(np.ones((ph, pw), bool)); V = np.ones((ph, pw), bool); s = int(round(side_m / cf.res)); c = ph // 2
    for j in range(n):
        dC = np.zeros((ph, pw), np.float32); dC[c:c + s, c:c + s] = 3 * cf.a.tc
        cf._tc = cf.a.tc; cf._refp = (np.zeros((ph, pw, 3), np.uint8), V)
        cf._judge(sl, pr, pc_, np.full(len(pr), 0.55 + 0.05 * j, np.float32), np.zeros((ph, pw, 3), np.uint8), V, ph, pw,
                  np.zeros(3), None, None, dC, np.zeros((ph, pw), np.float32))


def test_a_1_cm_change_is_a_spot_on_the_5_mm_grid_and_measured_in_cm2():
    """[Ot0]: a 1 cm square under the smallest blob on the 1 cm grid (one cell) is four cells on the 5 mm grid, a spot
    once the smallest blob allowed is under 1 cm2; its area is reported in cm2 on either grid"""
    import confirm_stream as cs
    coarse = _confirmer(cs, 0.01, min_cm2=0.5); _spot(coarse)
    fine = _confirmer(cs, 0.005, min_cm2=0.5); _spot(fine)
    sc, sf = list(coarse.spots()), list(fine.spots())
    assert len(sf) == 1 and sf[0][2] == 4                           # four 5 mm cells
    x, y = sf[0][3], sf[0][4]
    assert abs(x - (-cs.HX + 830 * 0.01)) < 0.01 and abs(y - (cs.HY - 430 * 0.01)) < 0.01   # where the 1 cm grid puts it
    recs = fine.decide(0.0, (x + 0.6, y))
    assert recs and recs[0]["area_cm2"] == 1.0
    big = _confirmer(cs, 0.005); _spot(big)                          # the default smallest blob (4 cm2) holds on the fine grid too
    assert not list(big.spots())
    assert len(sc) <= 1 and (not sc or sc[0][2] <= 1)               # the 1 cm grid: at most one cell


def test_views_carry_their_grid(tmp_path):
    """[Ot0]: views painted on the 5 mm grid say so, load at that grid, and are not mixed with 1 cm views"""
    import confirm_stream as cs
    p = np.full((20, 20, 3), 100, np.uint8)
    cs.Views([[0, 0]], [[1, 0]], [(10, 30, 10, 30)], [p], res=0.005).save(tmp_path / "fine.npz")
    cs.Views([[0, 0]], [[1, 0]], [(10, 30, 10, 30)], [p]).save(tmp_path / "coarse.npz")
    assert cs.Views.load([str(tmp_path / "fine.npz")]).res == 0.005
    assert cs.Views.load([str(tmp_path / "coarse.npz")]).res == cs.RES
    assert "res" not in np.load(tmp_path / "coarse.npz").files     # a 1 cm view set's file is as before
    nc, nr, n = cs.Views.load([str(tmp_path / "fine.npz")]).noise()
    assert n.shape == cs.grid(0.005)
    with pytest.raises(SystemExit):
        cs.Views.load([str(tmp_path / "fine.npz"), str(tmp_path / "coarse.npz")])


def test_a_near_find_beside_a_1_cm_find_is_that_find():
    """[Ot0]: the near layer's find within the merge distance of the 1 cm layer's confirmed or unsure find is not a
    second stop; one apart from it stands; one larger than the near layer keeps is left to the 1 cm layer"""
    import confirm_stream as cs
    fine = _confirmer(cs, 0.005, min_cm2=0.5, hits=2, span=0.1, slop=0); _spot(fine)
    x, y = list(fine.spots())[0][3:5]
    coarse = SimpleNamespace(decided={(0, 0): {"decision": "confirmed", "x": x + 0.2, "y": y}})
    a = SimpleNamespace(near_merge_m=0.3, near_max_cm2=0.0)
    r = cs.near_decide(fine, coarse, 0.0, (x + 0.6, y), a)
    assert r and r[0]["layer"] == "near" and r[0]["decision"] == "not_a_find" and r[0]["why_not"] == "the 1 cm grid's find"
    fine2 = _confirmer(cs, 0.005, min_cm2=0.5, hits=2, span=0.1, slop=0); _spot(fine2)
    coarse.decided = {(0, 0): {"decision": "confirmed", "x": x + 1.0, "y": y}}
    assert cs.near_decide(fine2, coarse, 0.0, (x + 0.6, y), a)[0]["decision"] == "confirmed"
    fine3 = _confirmer(cs, 0.005, min_cm2=0.5, hits=2, span=0.1, slop=0); _spot(fine3)
    r3 = cs.near_decide(fine3, coarse, 0.0, (x + 0.6, y), SimpleNamespace(near_merge_m=0.3, near_max_cm2=0.5))
    assert r3[0]["decision"] == "not_a_find"


def test_the_live_tidy_judges_with_the_benchs_options():
    """[BW3]: the live tidy's options at a named setting are the replay's at it, every one but the night's own files;
    with no setting, the replay's defaults (CONFIRM's where it names them)"""
    import confirm_stream as cs
    files = {"run_dir", "seed", "night", "db", "views", "near_views", "setting"}
    base = ["--run-dir", "r", "--night", "3", "--db", "d"]
    for setting, extra in (("launch", ["--setting", "launch"]), (None, [])):
        bench = vars(cs.parser().parse_args(base + extra)); live = vars(cs.options(setting))
        assert {k: v for k, v in live.items() if k not in files} == {k: v for k, v in bench.items() if k not in files}
    assert cs.options("launch").near_res == 0.005 and cs.options("launch").setting == "launch"
    assert all(getattr(cs.options(), k) == v for k, v in cs.CONFIRM.items())


class _Layer:
    """a stand-in Confirmer: records each frame with the floor lock it was placed by"""
    def __init__(self, k, Cf=None, recs=()):
        self.k, self.Cf, self.frames, self.recs, self.decided = k, np.eye(3) if Cf is None else Cf, [], list(recs), {}
    def frame(self, img, T, K=None, wh=None, mask=None):
        self.frames.append((img, self.Cf.copy())); self.Cf = self.Cf + (1.0 if self.k == 1 else 0.0)   # the 1 cm lock moves
        return "patch"
    def decide(self, t, xy): return [dict(r) for r in self.recs]
    def keepout(self, margin=0.05): return [(self.k, 0.0, margin)]


def test_the_layers_judge_as_the_replay_pairs_them():
    """[BW3], [Ot0]: each frame to the 1 cm layer, then to the near layer placed by the 1 cm layer's lock as it now
    stands; decisions the 1 cm layer's then the near layer's, a near find beside a 1 cm find that find; spots kept
    out from both; the tall layer set on the pair reaches the near layer on its grid; anything else is the 1 cm layer's"""
    import confirm_stream as cs
    cf = _Layer(1, recs=[{"decision": "confirmed", "x": 1.0, "y": 0.0}]); cf.decided = {0: cf.recs[0]}
    cfn = _Layer(2, recs=[{"decision": "confirmed", "x": 1.1, "y": 0.0, "area_cm2": 1.0},
                          {"decision": "confirmed", "x": 3.0, "y": 0.0, "area_cm2": 1.0}])
    L = cs.Layers(cf, cfn, cs.options("launch"))
    assert L.frame("img", np.eye(4)) == "patch" and L.frame("img2", np.eye(4)) == "patch"
    assert [c[0, 0] for _, c in cfn.frames] == [2.0, 3.0]              # the lock after the 1 cm layer's own frame
    r = L.decide(0.0, (0.0, 0.0))
    assert [x.get("layer") for x in r] == [None, "near", "near"]
    assert [x["decision"] for x in r] == ["confirmed", "not_a_find", "confirmed"]
    assert L.keepout() == [(1, 0.0, 0.05), (2, 0.0, 0.05)]
    L.hmap = np.ones((2, 3), np.float32)
    assert cf.hmap.shape == (2, 3) and cfn.hmap.shape == (4, 6)
    L.on_lock = "lock"; assert cf.on_lock == "lock" and not hasattr(cfn, "on_lock") and L.Cf is cf.Cf


def test_a_tidy_night_at_the_launch_setting_runs_the_near_grid(tmp_path):
    """[BW3]: at the launch setting the live tidy's confirmer is the 1 cm layer with the near grid beside it, on the
    baseline nights' own near views (not another night's), with the near layer's own options and each layer's view
    noise as the replay sets them; the house's nights without near views are named"""
    import confirm_stream as cs
    from houndkeeper import tidy_module as tm
    from houndkeeper.live_localize import HouseFramed
    p = np.full((20, 20, 3), 100, np.uint8)
    for k in (1, 2, 9):
        cs.Views([[0, 0]], [[1, 0]], [(10, 30, 10, 30)], [p], res=0.005).save(tmp_path / f"views_near_n{k}.npz")
        cs.Views([[0, 0]], [[1, 0]], [(10, 30, 10, 30)], [p]).save(tmp_path / f"views_n{k}.npz")
    nv, missing = tm.near_views(tmp_path, [1, 2])
    assert nv == [str(tmp_path / "views_near_n1.npz"), str(tmp_path / "views_near_n2.npz")] and missing == []
    assert tm.near_views(tmp_path, [1, 3]) == ([str(tmp_path / "views_near_n1.npz")], [3])
    assert tm.near_views(tmp_path, [4]) == (None, [4])
    a = cs.options("launch"); a.near_views = nv
    H, W = cs.H, cs.W; z = np.zeros((H, W), bool)
    img = np.zeros((720, 1280, 3), np.uint8)
    im = SimpleNamespace(ts=None, to_rgb=lambda: SimpleNamespace(data=img))
    T = np.eye(4); T[2, 3] = 0.3
    me = SimpleNamespace(_head=im, _head_camera=lambda ts: T, _head_lens=lambda raw: (raw, (800.0, 800.0, 640.0, 360.0)),
                         CANDIDATE_CHECK=False, _cf=None, _cst=cs, _relief=None, _run_dir=str(tmp_path), _prev=[1, 2],
                         _live_loc=None, _refresh_sightlines=lambda: None,
                         _cf_args=(np.zeros((H, W, 3), np.uint8), ~z, z, np.zeros((H, W), np.float32), a))
    tm.TidyArm._watch_frame(me)
    cf = me._cf.cf if isinstance(me._cf, HouseFramed) else me._cf
    assert isinstance(cf, cs.Layers) and cf.cf.res == cs.RES and cf.cfn.res == 0.005
    assert cf.cfn.views.res == 0.005 and len(cf.cfn.views.C) == 2 and len(cf.cf.views.C) == 2   # nights 1 and 2 only
    assert cf.cfn.a.rmax == 1.0 and cf.cfn.a.span == 0.3 and cf.cfn.a.min_cm2 == 0.5 and cf.cf.a.span == 0.3
    assert cf.cf.vnoise is not None and cf.cfn.vnoise is not None     # each layer's view noise, as the replay sets it
    me2 = SimpleNamespace(**dict(vars(me), _cf=None, _last_im=None, _cf_args=me._cf_args[:4] + (cs.options(),)))
    tm.TidyArm._watch_frame(me2)                                        # no setting: the 1 cm layer alone, as before
    assert isinstance(me2._cf, HouseFramed) and isinstance(me2._cf.cf, cs.Confirmer)
