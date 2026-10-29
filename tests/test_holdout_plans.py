"""Hold-out houses are wholly unlike the tuning houses (sim-house-nights [ejh], [A7G]): a registry says which seeds are
hold-outs; the ones from [ejh] on draw their own plans, which no tuning seed uses and none is close to."""
import os, subprocess, sys, json, hashlib
from pathlib import Path

HOUSE = Path(__file__).resolve().parents[1] / "sim" / "house"
sys.path.insert(0, str(HOUSE))
THRESHOLD = 0.30   # [A7G]: the 5th percentile of tuning-vs-tuning overlaps (3 Oct 2026)


def _tuning(mh):
    return [mh.layout(70)] + [mh.layout(s) for s in range(72, 100) if s not in mh.HOLDOUT and s not in mh.DEMO_PLANS]


def test_registry_keeps_holdouts_apart_from_tuning():
    """[ejh], [A7G]: hold-outs never tuning, demo or preview seeds; only own-plan hold-outs call the generator"""
    import make_house as mh
    assert mh.HOLDOUT_SHARED == {80, 81} and not mh.HOLDOUT_OWN & (set(range(70, 100)) | set(mh.DEMO_PLANS) | set(mh.PREVIEW))
    real = mh.unique_plan; mh.unique_plan = lambda s: (_ for _ in ()).throw(AssertionError(s))
    try:
        for s in [None, 7] + list(range(70, 100)): mh.layout(s)
    finally: mh.unique_plan = real
    assert mh.layout(min(mh.HOLDOUT_OWN)) == real(min(mh.HOLDOUT_OWN))


def test_existing_plans_stay_unchanged():
    """[ejh]: 70, 71, 80, 81, the variants and 95 as they were (hashes taken before the generator existed)"""
    import make_house as mh
    want = json.loads((Path(__file__).parent / "fixtures" / "plan_hashes.json").read_text())
    for k, h in want.items():
        s = None if k == "None" else int(k)
        assert hashlib.sha256(json.dumps(mh.layout(s), sort_keys=True, default=str).encode()).hexdigest()[:16] == h, k


def test_own_plans_are_deterministic():
    """[A7G]: the same seed, the same plan, in another process too"""
    import make_house as mh
    code = "import make_house as mh, json; print(json.dumps(mh.unique_plan(901), sort_keys=True, default=str))"
    got = subprocess.run([sys.executable, "-c", code], cwd=HOUSE, env=dict(os.environ, PYTHONHASHSEED="7"), capture_output=True, text=True).stdout.strip()
    assert got == json.dumps(mh.unique_plan(901), sort_keys=True, default=str) and mh.unique_plan(901) == mh.unique_plan(901)


def test_every_own_plan_is_valid_and_reachable():
    """[ejh], [A7G]: pieces inside the walls and on open floor, names unique, every open cell reachable from the dock"""
    import make_house as mh
    for s in sorted(mh.HOLDOUT_OWN):
        p = mh.layout(s); names = [f[5] for f in p["furniture"]]
        assert len(set(names)) == len(names) and mh._reach_all(p["furniture"], p["stubs"], p["start"]), s
        assert all(1 <= f[0] <= f[2] <= mh.W - 2 and 1 <= f[1] <= f[3] <= mh.H - 2 for f in p["furniture"]), s


def test_own_plans_are_unlike_every_tuning_plan_and_80_81_are_not():
    """[ejh], [A7G]: below the threshold against every tuning plan (mirrors included); 80 and 81 above it"""
    import make_house as mh
    tun = _tuning(mh) + [mh.layout(95)]
    for s in sorted(mh.HOLDOUT_OWN):
        p = mh.layout(s); assert max(mh.plan_overlap(p, t) for t in tun) < THRESHOLD, s
    for s in (80, 81): assert max(mh.plan_overlap(mh.layout(s), t) for t in tun) >= THRESHOLD, s


def test_chairs_follow_a_holdout_plans_own_table_and_rooms():
    """[ejh], [A7G]: in a hold-out's own plan (preview seeds stand in; the real ones stay unseen) the movable chairs are
    homed round its own table, stool and play shelf: tucked ones within a few cm of those homes, those moved a lot inside
    their own room, none on another; every other plan keeps the chairs exactly as before"""
    import math, random
    import make_house as mh, movables as mv
    real = mh.layout; mh.layout = lambda s: mh.unique_plan(s) if s in mh.PREVIEW else real(s)   # a preview seed as a hold-out
    try:
        for seed in (70, 72, 81):
            mh.use_layout(seed); assert mv.chairs() == mv.CHAIRS and mv.lot_box("stool") == mv.LOT_BOX["stool"]
        for seed in sorted(mh.PREVIEW)[:6]:
            mh.use_layout(seed); g = mh.base_grid(); H = mh.CHAIR_HOMES; prev = None
            assert set(H) == {c[0] for c in mv.CHAIRS}
            for n in range(1, 5):
                cs = mv.place(random.Random(seed * 1000 + n), g, prev); prev = cs
                for c in cs:
                    hx, hy, _, (x0, y0, x1, y1) = H[c["id"]]
                    if c["mode"] == "tucked": assert math.hypot(c["x"] - hx, c["y"] - hy) < 0.05, (seed, n, c)
                    if c["mode"] == "a lot": assert x0 <= c["x"] <= x1 and y0 <= c["y"] <= y1, (seed, n, c)
                for i, a in enumerate(cs):
                    for b in cs[i + 1:]: assert math.hypot(a["x"] - b["x"], a["y"] - b["y"]) >= max(a["w"], b["w"])
    finally:
        mh.layout = real; mh.use_layout(None)
