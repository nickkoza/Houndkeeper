"""Each house has its own layout (sim-house-nights [uCP]): the seed draws the floor plan from 72 up; the houses already
run keep theirs; every open cell is reachable from the dock; everything reads the plan from the seed."""
import os, subprocess, sys
from collections import deque
from pathlib import Path

HOUSE = Path(__file__).resolve().parents[1] / "sim" / "house"
sys.path.insert(0, str(HOUSE))


def _reachable(mh, g):
    sx, sy = mh.START; seen = {(sx, sy)}; q = deque([(sx, sy)])
    while q:
        x, y = q.popleft()
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < mh.W and 0 <= ny < mh.H and (nx, ny) not in seen and g[ny][nx] == 0:
                seen.add((nx, ny)); q.append((nx, ny))
    return seen


def test_the_houses_already_run_keep_their_plan():
    import make_house as mh
    for seed in (70, 71, 80, 7):
        mh.use_layout(seed)
        assert mh.FURNITURE == mh.BASE_FURNITURE and mh.STUBS == mh.BASE_STUBS and mh.START == mh.BASE_START
        assert mh.RUG == mh.BASE_RUG and mh.REGIONS == mh.BASE_REGIONS
        assert mh.CARPET == mh.BASE_CARPET and mh.LAMP == mh.BASE_LAMP and mh.BOWLS == mh.BASE_BOWLS


def test_new_houses_each_draw_their_own_plan_the_same_every_time():
    import make_house as mh
    plans = []
    for seed in range(72, 82):
        if seed == 80: continue
        mh.use_layout(seed); plan = (tuple(mh.FURNITURE), tuple(mh.STUBS), mh.START, tuple(sorted(mh.RUG.items())))
        mh.use_layout(seed); assert plan == (tuple(mh.FURNITURE), tuple(mh.STUBS), mh.START, tuple(sorted(mh.RUG.items())))
        plans.append(plan)
    mh.use_layout(70); base = (tuple(mh.FURNITURE), tuple(mh.STUBS), mh.START, tuple(sorted(mh.RUG.items())))
    assert base not in plans and len(set(plans)) == len(plans)
    mh.use_layout(72); flips = []
    for seed in range(72, 100):
        mh.use_layout(seed); flips.append(mh.MIRRORED)
    assert any(flips) and not all(flips)                                    # both ways round among the new houses


def test_every_open_cell_is_reachable_from_the_dock():
    import make_house as mh
    for seed in [70] + list(range(72, 100)):
        mh.use_layout(seed); g = mh.base_grid()
        open_ = {(x, y) for y in range(mh.H) for x in range(mh.W) if g[y][x] == 0}
        if seed in mh.DEMO_PLANS:                                          # a demo room is walled off from the rest of the
            xe = max(s[2] for s in mh.STUBS if s[0] == s[2]); ye = max(s[3] for s in mh.STUBS if s[1] == s[3])   # house:
            open_ = {(x, y) for x, y in open_ if x < xe and y < ye}        # its own floor ([qEP])
        assert g[mh.START[1]][mh.START[0]] == 0 and _reachable(mh, g) == open_, seed
        for x0, y0, x1, y1, h, name in mh.FURNITURE:                       # inside the outer walls
            assert 1 <= x0 <= x1 <= mh.W - 2 and 1 <= y0 <= y1 <= mh.H - 2, (seed, name)


def test_the_house_seed_reaches_everything_that_reads_the_plan():
    code = "import make_house as mh; print(mh.LAYOUT_SEED, mh.START)"
    env = dict(os.environ, HK_HOUSE_SEED="73")
    got = subprocess.run([sys.executable, "-c", code], cwd=HOUSE, env=env, capture_output=True, text=True).stdout.split()
    import make_house as mh
    mh.use_layout(73)
    assert got[0] == "73" and " ".join(got[1:]) == str(mh.START)
