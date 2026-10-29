#!/usr/bin/env python3
"""Items for a tidy run in the house scene ([YWo], [QxY]): library items (scanned toys and the core set) on free floor
1-3 m from the dock, at least 0.5 m clear of walls and furniture and 0.6 m from each other, from a seed. Prints
two lines for tidy_run.sh: the dock position "x, y" and the HK_SIM_ITEMS JSON.
Usage: house_items.py <truth.json> [count|auto] [seed]   (auto: one per 200 sq ft of floor, [Afy])"""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import json, os, sys
import numpy as np
sys.path.insert(0, str(_HK / "sim" / "arm")); sys.path.insert(0, str(_HK / "sim" / "house"))
import items, make_house as mh
from house_toys import pick_night, toys_for_floor
_TIERS: dict = {}
def _tier(name):   # [QyZ]'s tier, once per toy (it settles the toy in a small sim)
    if name not in _TIERS: _TIERS[name] = items.grasp_tier(name)
    return _TIERS[name]


def static_bounds():
    """walls and furniture of the house layout, [x0, x1, y0, y1] in metres (make_house.py's own)"""
    def bounds(x0, y0, x1, y1):
        (ax, ay), (bx, by) = mh.cell_to_world(x0, y0), mh.cell_to_world(x1, y1)
        return [min(ax, bx) - mh.CELL / 2, max(ax, bx) + mh.CELL / 2, min(ay, by) - mh.CELL / 2, max(ay, by) + mh.CELL / 2]
    return [bounds(*r) for r in mh.wall_rects(mh.base_grid())] + [bounds(*f[:4]) for f in mh.FURNITURE]


def layout(rng, start, clear, n, radius=None, near=0.0, far=0.6, near_m=(0.4, 0.9), start_clear=1.0, r_min=1.0, r_max=3.0,
           max_near=2):
    """n toy positions: (x, y) on clear floor. The old layout (radius None): each 1-3 m from the start, 0.6 m apart.
    Spread ([O7l]): anywhere within `radius` of the start, `far` apart; at most `max_near` dropped `near_m` from one
    already down (never by one that is itself part of a pair: a pair, a trio at most, not a heap)"""
    placed, paired, n_near = [], set(), 0
    for _ in range(20000):
        if len(placed) == n: break
        if radius is None:
            a, r = rng.uniform(0, 2 * np.pi), rng.uniform(r_min, r_max); p = start + r * np.array([np.cos(a), np.sin(a)])
            if clear(p) and all(np.linalg.norm(p - q) > far for q in placed): placed.append(p)
            continue
        loners = [k for k in range(len(placed)) if k not in paired]
        by = None
        if loners and n_near < max_near and rng.random() < near:   # now and then dropped by another (a pair, a small cluster)
            by = loners[rng.integers(len(loners))]; a, r = rng.uniform(0, 2 * np.pi), rng.uniform(*near_m)
            p = placed[by] + r * np.array([np.cos(a), np.sin(a)]); gap = near_m[0] - 0.05
        else:                                               # anywhere on the floor the walk covers
            a, r = rng.uniform(0, 2 * np.pi), radius * np.sqrt(rng.uniform()); p = start + r * np.array([np.cos(a), np.sin(a)])
            gap = far
        if np.linalg.norm(p - start) < start_clear or not clear(p): continue
        if not all(np.linalg.norm(p - q) > gap for q in placed): continue
        if by is not None:
            if any(np.linalg.norm(p - q) < near_m[1] + 0.05 for k, q in enumerate(placed) if k != by): continue   # one partner only
            paired |= {by, len(placed)}; n_near += 1
        placed.append(p)
    return placed


WALK_RADIUS = float(os.environ.get("RADIUS", 5.0)) - 0.5   # the night's walk covers this far from the start (night_tidy's --radius)
SPREAD = dict(radius=WALK_RADIUS, near=0.25, far=1.5)   # [O7l]: spread over the walk's floor, most toys 1.5 m+ apart, a quarter
                                                        # dropped 0.4-0.9 m from one already down; the first metre clear


if __name__ == "__main__":
    truth = json.load(open(sys.argv[1]))
    FLOOR_M2 = (mh.W - 2) * (mh.H - 2) * mh.CELL ** 2                      # inside the outer walls: 16.1 x 9.1 m
    n = toys_for_floor(FLOOR_M2) if len(sys.argv) < 3 or sys.argv[2] == "auto" else int(sys.argv[2])
    rng = np.random.default_rng(int(sys.argv[3]) if len(sys.argv) > 3 else 0)
    start = np.array(truth["start_pos"]); static = truth.get("static_bounds") or static_bounds()
    static = static + [[c["x"] - r_, c["x"] + r_, c["y"] - r_, c["y"] + r_]            # tonight's chairs where they stand
                       for c in truth.get("movables") or [] for r_ in [np.hypot(c["w"], c["d"]) / 2]]   # (the truth given is tonight's)
    static = static + [[f["x"] - 0.15, f["x"] + 0.15, f["y"] - 0.15, f["y"] + 0.15] for f in truth.get("fixtures") or []]   # the pet's bowls
    SPLIT = os.environ.get("HK_ITEMS", "train")        # held-out items only when judging ([nuQ]): HK_ITEMS=heldout
    names = [k for k in items.pool(SPLIT) if items.ITEMS[k][3] in ("bin", "auto") and items.ITEMS[k][0] in ("gso", "box", "cylinder")]   # one body each
    B = np.array(static); inside = (B[:, 0].min(), B[:, 1].max(), B[:, 2].min(), B[:, 3].max())   # the outer walls' extent
    def clear(p, margin=0.5):
        """on the house's floor: inside its outer walls, and clear of every wall and piece of furniture"""
        if not (inside[0] + margin < p[0] < inside[1] - margin and inside[2] + margin < p[1] < inside[3] - margin): return False
        return all(not (x0 - margin < p[0] < x1 + margin and y0 - margin < p[1] < y1 + margin) for x0, x1, y0, y1 in static)
    tonight = pick_night(rng, names, _tier, n)                              # the night's mix of tiers ([Afy])
    for far in (SPREAD["far"], 1.2, 1.0, 0.8):                # a small or crowded walk area: closer, rather than a toy short
        pos = layout(rng, start, clear, n, **dict(SPREAD, far=far))   # (house 70, 2 Oct: "room for only 7 of 8")
        if len(pos) == n: break
    placed = list(zip(tonight, pos))
    if len(placed) < n: sys.exit(f"room for only {len(placed)} of {n} toys")
    print(f"{start[0]:.3f}, {start[1]:.3f}")
    print(json.dumps([[k, f"{k}_{i}", [round(float(p[0]), 3), round(float(p[1]), 3), 0.1]] for i, (k, p) in enumerate(placed)]))
