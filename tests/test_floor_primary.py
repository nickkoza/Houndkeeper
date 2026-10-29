"""The floor comparison as the primary finder ([H9w]), on synthetic floors: the empty nights' own spread per cell
(Views.noise, behind --view-noise) and the change pooled over views before any threshold (--fuse)."""
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src"))


def test_view_noise_is_the_spread_between_the_empty_nights_views():
    """[H9w]: a cell the views agree on has no spread; one whose brightness moves from view to view (glare) has it"""
    import confirm_stream as cs
    rng = np.random.default_rng(1); box = (100, 160, 200, 260)
    patches = []
    for k in range(4):
        p = np.full((60, 60, 3), 120, np.uint8)
        p[20:30, 20:30] = 120 + 60 * (k % 2)                       # a glint that comes and goes with the view
        patches.append(p)
    V = cs.Views(C=[[0, 0]] * 4, F=[[1, 0]] * 4, sls=[box] * 4, patches=patches)
    nc, nr, n = V.noise(0.0)
    assert n[130, 230] == 4
    assert nr[100 + 25, 200 + 25] > 0.1                             # the glint's cell
    assert nr[100 + 50, 200 + 50] < 0.02 and nc[100 + 50, 200 + 50] < 0.005   # plain floor away from it
    nc3, nr3, _ = cs.Views(C=[[0, 0]] * 2, F=[[1, 0]] * 2, sls=[box] * 2, patches=patches[:2]).noise(0.0)
    assert nr3.max() == 0                                           # fewer than 3 views: no say


def _confirmer(cs, **over):
    a = dict(cs.CONFIRM); a.update(over); a = SimpleNamespace(**a)
    for k, v in (("fuse", 0.0), ("view_noise", 0.0), ("dark", 0.0)): setattr(a, k, getattr(a, k, v))
    z = np.zeros((cs.H, cs.W), bool)
    return cs.Confirmer(np.zeros((cs.H, cs.W, 3), np.uint8), ~z, z, np.zeros((cs.H, cs.W), np.float32),
                        (300.0, 300.0, 320.0, 180.0), (640, 360), a)


def _frames(cf, cs, n=12, level=0.7, seed=0):
    """n views of a 60 x 60 cell patch: a 10 x 10 spot whose colour shift is `level` of the threshold in every view,
    the rest the floor's noise (well under it)"""
    rng = np.random.default_rng(seed); sl = (slice(400, 460), slice(800, 860)); ph = pw = 60
    pr, pc_ = np.nonzero(np.ones((ph, pw), bool)); V = np.ones((ph, pw), bool)
    for k in range(n):
        dC = np.abs(rng.normal(0, 0.15 * cf.a.tc, (ph, pw))).astype(np.float32); dC[25:35, 25:35] = level * cf.a.tc
        dd = np.full(len(pr), 0.6 + 0.12 * k, np.float32); cf._tc = cf.a.tc
        cf._refp = (np.zeros((ph, pw, 3), np.uint8), V)
        cf._judge(sl, pr, pc_, dd, np.zeros((ph, pw, 3), np.uint8), V, ph, pw, np.zeros(3), None, None, dC,
                  np.zeros((ph, pw), np.float32))
    return sl


def test_a_faint_spot_seen_in_every_view_is_found_only_when_views_are_pooled():
    """[H9w]: a change under the per-frame threshold in each view never counts a hit (the vote); pooled over the
    views it stands out of the floor's noise"""
    import confirm_stream as cs
    vote = _confirmer(cs); _frames(vote, cs)
    assert not list(vote.spots())
    pooled = _confirmer(cs, fuse=0.5); _frames(pooled, cs)
    sp = list(pooled.spots())
    assert len(sp) == 1
    _, m, area, x, y, _, _ = sp[0]
    assert 60 <= area <= 140                                       # the 10 x 10 spot, a little closed
    recs = pooled.decide(0.0, (x + 1.0, y))
    assert recs and recs[0]["hits"] >= 4 and recs[0]["span_m"] >= 1.0   # evidence: the views that saw it, over their range


def _lo_confirmer(cs, **over):
    """a Confirmer with the log-odds filter's options at the parser's defaults (and any given)"""
    a = vars(cs.parser().parse_args(["--run-dir", "x", "--seed", "1", "--night", "3", "--db", "x"]))
    a.update(logodds=True); a.update(over); a = SimpleNamespace(**a)
    z = np.zeros((cs.H, cs.W), bool)
    return cs.Confirmer(np.zeros((cs.H, cs.W, 3), np.uint8), ~z, z, np.zeros((cs.H, cs.W), np.float32),
                        (300.0, 300.0, 320.0, 180.0), (640, 360), a)


def test_the_log_odds_sensor_model_weighs_a_change_against_the_cells_own_noise():
    """[H9w]: one frame's evidence is strongly against change where nothing changed, strongly for it where much did,
    and weaker for the same change where the empty nights' views of the cell vary, where the frame saw it from far, and
    where no near baseline view covers it"""
    import confirm_stream as cs
    cf = _lo_confirmer(cs, view_noise=6.0); sl = (slice(0, 1), slice(0, 4)); cf._tc = cf.a.tc
    near = np.full((1, 4), 0.6, np.float32); zero = np.zeros((1, 4), np.float32)
    l0 = cf.log_likelihood(sl, zero, zero, near)
    assert np.allclose(l0, cf.a.lo_min)
    big = np.full((1, 4), 10 * cf.a.tc, np.float32)
    assert np.allclose(cf.log_likelihood(sl, big, zero, near), cf.a.lo_max)
    mid = np.full((1, 4), 0.9 * cf.a.tc, np.float32)                  # 5.4 of the floor's noise units on a quiet cell
    quiet = cf.log_likelihood(sl, mid, zero, near)[0, 0]
    cf.vnoise = (np.full((cs.H, cs.W), 0.1 * cf.a.tc, np.float32), np.zeros((cs.H, cs.W), np.float32))
    noisy = cf.log_likelihood(sl, mid, zero, near)[0, 0]; del cf.vnoise
    far = cf.log_likelihood(sl, mid, zero, np.full((1, 4), 2.4, np.float32))[0, 0]
    cf._near = np.zeros((1, 4), bool); blended = cf.log_likelihood(sl, mid, zero, near)[0, 0]
    assert quiet > 0 and noisy < quiet and far < quiet and blended < quiet


def test_log_odds_keeps_a_steady_change_and_forgets_a_passing_glint():
    """[H9w]: summed over the views, a spot changed in every view climbs to the confirm level (clamped), and one that
    changed in a single view of twelve ends believed unchanged"""
    import confirm_stream as cs
    cf = _lo_confirmer(cs); sl = _frames(cf, cs, level=1.5)
    assert cf.L[sl][30, 30] == cf.a.lo_clamp_max and cf.L[sl][15, 15] == cf.a.lo_clamp_min   # (the patch's 6-cell edge is never judged)
    sp = list(cf.spots()); assert len(sp) == 1
    g = _lo_confirmer(cs); rng = np.random.default_rng(0); sl = (slice(400, 460), slice(800, 860)); V = np.ones((60, 60), bool)
    pr, pc_ = np.nonzero(V)
    for k in range(12):
        dC = np.abs(rng.normal(0, 0.15 * g.a.tc, (60, 60))).astype(np.float32)
        if k == 3: dC[25:35, 25:35] = 3 * g.a.tc                          # a glint in one view
        g._tc = g.a.tc; g._refp = (np.zeros((60, 60, 3), np.uint8), V)
        g._judge(sl, pr, pc_, np.full(len(pr), 1.0, np.float32), np.zeros((60, 60, 3), np.uint8), V, 60, 60, np.zeros(3), None, None,
                 dC, np.zeros((60, 60), np.float32))
    assert g.L[sl][30, 30] < 0 and not list(g.spots())


def test_a_cached_replay_judges_and_decides_as_the_live_one(tmp_path):
    """[H9w]: the evidence replayed from a saved comparison (FrameCache) is the evidence the frames gave"""
    import confirm_stream as cs
    live = _lo_confirmer(cs); live.cache = cs.FrameCache(tmp_path / "c.fc")
    rng = np.random.default_rng(2); sl = (slice(400, 460), slice(800, 860)); V = np.ones((60, 60), bool); pr, pc_ = np.nonzero(V)
    for k in range(8):
        dC = np.abs(rng.normal(0, 0.3 * live.a.tc, (60, 60))).astype(np.float32); dC[25:35, 25:35] = 1.5 * live.a.tc
        dd = np.full(len(pr), 2.0 - 0.15 * k, np.float32); live._tc = live.a.tc; live._refp = (np.zeros((60, 60, 3), np.uint8), V)
        live.cache.add(sl, pr, pc_, dd, np.zeros((60, 60, 3), np.uint8), V, np.zeros(3), dC, np.zeros((60, 60), np.float32), live._tc, live._refp, None)
        live._judge(sl, pr, pc_, dd, np.zeros((60, 60, 3), np.uint8), V, 60, 60, np.zeros(3), None, None, dC, np.zeros((60, 60), np.float32))
        live.cache.tick(100.0 + k, [0.0, 0.0], live.Cf, True, False)
    live.cache.close()
    again = _lo_confirmer(cs, cache_in=str(tmp_path / "c.fc")); recs, used, _, _ = cs.replay_cached(again.a, again)
    assert used == 8 and np.allclose(again.L, live.L, atol=0.02) and (again.hit == live.hit).all()


def test_a_far_view_that_sees_no_change_is_weak_evidence_against_one():
    """[H9w]: with --lo-miss-dist, a frame that finds a cell unchanged from far counts less against it than one from
    near (a small toy covers too few pixels to show far off); evidence for a change is not shrunk"""
    import confirm_stream as cs
    cf = _lo_confirmer(cs, lo_miss_dist=2.0); sl = (slice(0, 1), slice(0, 2)); cf._tc = cf.a.tc
    zero = np.zeros((1, 2), np.float32); big = np.full((1, 2), 10 * cf.a.tc, np.float32)
    near, far = np.full((1, 2), 0.6, np.float32), np.full((1, 2), 2.3, np.float32)
    assert cf.log_likelihood(sl, zero, zero, far)[0, 0] > cf.log_likelihood(sl, zero, zero, near)[0, 0] * 0.5
    assert cf.log_likelihood(sl, zero, zero, far)[0, 0] < 0
    assert cf.log_likelihood(sl, big, zero, far)[0, 0] == cf.a.lo_max


def test_a_cached_replay_is_the_live_one_exactly_at_a_knife_edge(tmp_path):
    """[LQs]: a saved comparison keeps the robot's float32, so a cell a hair over the threshold in the live run is over it
    replayed (float16 rounded such cells either way); an old float16 cache still reads, and says it is float16"""
    import confirm_stream as cs
    live = _lo_confirmer(cs, logodds=False, hits=2, ratio=0.4, span=0.0)
    live.cache = cs.FrameCache(tmp_path / "c.fc", options=cs.cache_options(live.a))
    assert live.cache.precision == "float32"
    sl = (slice(400, 460), slice(800, 860)); V = np.ones((60, 60), bool); pr, pc_ = np.nonzero(V); tc = live.a.tc
    edge = np.float32(tc) * np.float32(1 + 2e-4)                  # over tc in float32, back to tc or under in float16
    assert np.float16(edge) <= np.float16(tc) or np.float32(np.float16(edge)) != edge
    for k in range(6):
        dC = np.zeros((60, 60), np.float32); dC[25:35, 25:35] = edge
        dd = np.full(len(pr), 2.0 - 0.2 * k, np.float32); live._tc = tc; live._refp = (np.zeros((60, 60, 3), np.uint8), V)
        live.cache.add(sl, pr, pc_, dd, np.zeros((60, 60, 3), np.uint8), V, np.zeros(3), dC, np.zeros((60, 60), np.float32), live._tc, live._refp, None)
        live._judge(sl, pr, pc_, dd, np.zeros((60, 60, 3), np.uint8), V, 60, 60, np.zeros(3), None, None, dC, np.zeros((60, 60), np.float32))
        live.cache.tick(100.0 + k, [0.0, 0.0], live.Cf, True, k == 5)
    live.cache.close()
    again = _lo_confirmer(cs, logodds=False, hits=2, ratio=0.4, span=0.0, cache_in=str(tmp_path / "c.fc"))
    recs, used, _, prec = cs.replay_cached(again.a, again)
    assert prec == "float32" and used == 6 and again.a._cache_differs == []     # the same options: a measurement
    other = _lo_confirmer(cs, logodds=False, hits=2, ratio=0.4, span=0.0, tc=0.032, cache_in=str(tmp_path / "c.fc"))
    cs.replay_cached(other.a, other); assert other.a._cache_differs == ["tc"]      # another --tc: a screen, said
    assert (again.hit == live.hit).all() and (again.seen == live.seen).all() and np.array_equal(again.score, live.score)
    old = tmp_path / "old.fc"                                     # a cache written before the header: float16
    import pickle, zlib
    with open(old, "wb") as f:
        z = zlib.compress(pickle.dumps((1.0, np.zeros(2), np.eye(3), None, False), protocol=5), 1); f.write(len(z).to_bytes(8, "little")); f.write(z)
    fc = cs.FrameCache(old, "rb"); assert fc.precision == "float16" and len(list(fc)) == 1


def test_a_reported_number_never_comes_from_a_screen(monkeypatch):
    """[LQs]: the scorers refuse a replay judged from a float16 cache (recorded, or an old replay with --cache-in), or from
    a cache painted with other options than the replay states, unless asked for analysis; painted replays and float32
    caches of the same options pass"""
    import pytest
    from baselines import floor_diag as fd
    monkeypatch.delenv("HK_ALLOW_F16_CACHE", raising=False)
    for D in ({"night": "n", "extra": ["--cache-in", "x.fc"]}, {"night": "n", "precision": "float16"}):
        with pytest.raises(SystemExit): fd.reportable(D)
    monkeypatch.delenv("HK_CACHE_SCREEN", raising=False)
    with pytest.raises(SystemExit): fd.reportable({"night": "n", "precision": "float32", "extra": ["--cache-in", "x"], "cache_differs": ["tc"]})
    assert fd.reportable({"night": "n", "extra": []})
    assert fd.reportable({"night": "n", "precision": "float32", "extra": ["--cache-in", "x"], "cache_differs": []})
    monkeypatch.setenv("HK_CACHE_SCREEN", "1"); assert fd.reportable({"night": "n", "precision": "float16"})
