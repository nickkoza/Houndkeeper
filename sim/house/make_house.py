#!/usr/bin/env python3
"""Generate a house-like MuJoCo scene with seeded floor clutter for DimOS.

Open-plan ground floor: living, kitchen/dining and play areas flow together; a few stub walls.
Illustrative only, not the real house layout.
Writes <dimos>/data/mujoco_sim/scene_house_s<seed>.xml and a ground-truth JSON.

Usage: make_house.py --seed 1 --count 30
"""
import argparse, json, math, random
from pathlib import Path

CELL = 0.35                     # metres per cell -> 16.8 m x 9.8 m (~1,770 sq ft; ~2/3 of a 2,700 sq ft house)
W, H = 48, 28
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src")); from houndkeeper import workspace   # noqa: E402  (decision [cy4])
DIMOS_DATA = workspace.dimos() / "data" / "mujoco_sim"
OUT_DIR = Path(__file__).resolve().parent / "runs"

WALL_H = 2.4
# (x0, y0, x1, y1, height_m, label) in cells
FURNITURE = [
    (4, 1, 12, 1, 0.50, "tv_console"),
    (4, 6, 12, 7, 0.85, "sofa"),
    (6, 10, 9, 11, 0.45, "coffee_table"),
    (18, 1, 31, 2, 0.90, "kitchen_counter"),
    (20, 6, 28, 8, 0.90, "kitchen_island"),
    (22, 17, 27, 19, 0.75, "dining_table"),
    (45, 2, 46, 8, 1.20, "play_shelf"),
]
# weighted clutter zones [x0, y0, x1, y1, weight]: play area and living heaviest
REGIONS = [(1, 3, 16, 19, 3), (35, 1, 46, 12, 4), (17, 3, 33, 26, 2), (34, 14, 46, 26, 2), (1, 21, 16, 26, 1)]
START = (3, 24)                  # "Dock" on the page
STUBS = [(34, 0, 34, 4), (34, 11, 34, 13), (34, 13, 40, 13), (0, 20, 6, 20)]   # play-area alcove (1.8 m openings), entry nook
RUG = {"x": -4.6, "y": -0.2, "w": 2.2, "h": 1.5}                                # metres, under the living area
CARPET = [(0, 0, 17, 19), (34, 0, 47, 27)]       # carpeted cells: the living and play areas; the rest is tile
LAMP = (-4.0, 2.0)                               # m: the floor lamp beside the sofa
BOWLS = [("dog_bowl_ceramic", 3.00, 4.36), ("dog_bowl_plastic", 3.32, 4.36)]   # m: food and water, by the wall at the
                                                                             # kitchen counter's end ([nAH] TJu)
BASE_FURNITURE, BASE_REGIONS, BASE_START, BASE_STUBS, BASE_RUG = list(FURNITURE), list(REGIONS), START, list(STUBS), dict(RUG)
BASE_CARPET, BASE_LAMP, BASE_BOWLS = list(CARPET), LAMP, list(BOWLS)
FIXED_PLAN = {70, 71, 80}        # houses run on the one plan before [uCP]: they keep it (and every seed below 72)
# the hold-out registry (sim-house-nights [ejh], [A7G]): which seeds are hold-outs, and which plan each draws
HOLDOUT_SHARED = {80, 81}            # run before [ejh] on the tuning plan (80 the plan itself, 81 a variant): kept as they ran,
                                     # and labelled shared wherever their results are shown
HOLDOUT_OWN = set(range(100, 120))   # from [ejh] on: each draws its own plan (unique_plan); no tuning seed ever does
HOLDOUT = HOLDOUT_SHARED | HOLDOUT_OWN
PREVIEW = range(900, 1000)           # unique_plan drawn for review only, never hold-outs: the hold-outs' own plans stay unseen
MIRRORED, LAYOUT_SEED = False, None
FACING, CHAIR_HOMES = {}, None       # a hold-out's own plan: pieces turned to face their room (quarter turns from facing -y),
                                     # and the chairs' homes round its table ([A7G]); empty for every other plan
# the furniture's groups and how far each may move (cells: dx range, dy range); against a wall means dy 0 or dx 0
GROUPS = [({"tv_console": 0, "sofa": 1, "coffee_table": 1}, (-3, 3), (-1, 1)),   # the living area: the TV stays on its
          ({"kitchen_counter": 0, "kitchen_island": 1}, (-2, 2), (-1, 1)),       # wall (dy only for the others)
          ({"dining_table": 1}, (-3, 3), (-2, 2)),
          ({"play_shelf": 1}, (0, 0), (-1, 3))]


DEMO_PLANS = {   # subsystem-demos [qEP]: the demo room, about 20 x 20 ft (17 x 17 cells), in the house's top-left corner. Its two
                 # inside walls (x = 17, y = 17) are the ones a demo camera outside it looks through, and leaves out
    95: {"furniture": [(2, 1, 10, 2, 0.90, "kitchen_counter"), (8, 7, 11, 9, 0.75, "dining_table"),   # a dining room
                       (1, 8, 1, 8, 1.62, "reading_lamp"),                                    # behind the chair, by the window
                       (1, 12, 1, 16, 1.27, "fireplace"), (1, 8, 3, 10, 1.10, "wing_chair"), (3, 7, 3, 8, 0.58, "side_table")],
                                                     # the fireside where the coffee table stood (the cold
                                                     # open's fireplace and wing chair in the demo room, to tie the films together);
                                                     # the chair pushed back into the corner by the fire, ~5 cm off the window's
                                                     # shutters, so no floor hides behind it (night 1 never saw it)
         "regions": [(2, 2, 15, 15, 1)], "start": (14, 14), "stubs": [(17, 0, 17, 17), (0, 17, 17, 17)],
         "rug": {"x": -3.9, "y": 0.1, "w": 1.6, "h": 1.1}, "carpet": [(0, 7, 5, 16), (6, 12, 16, 16)], "lamp": (-7.83, 1.96), "bowls": [],
         "mirrored": False,   # (the counter two cells deep, ~0.6 m; the rug in the open carpet, not under the table; the carpet
                              # up round the fireside's new corner and back from the table's chairs, which stay on the tile)
         "chairs": [("dchair_n0", -5.25, 2.70, -math.pi / 2), ("dchair_n1", -4.55, 2.70, -math.pi / 2),   # round the dining table,
                    ("dchair_s0", -5.25, 1.15, math.pi / 2), ("dchair_s1", -4.55, 1.15, math.pi / 2)],    # off the walls (two cells
                    # down, three over: the planner keeps a margin round the table and chairs, and found no path through the ~1 m
                    # left by one or two over) so the robot can walk up its east side
                    # (night 1 only crossed the room)
         "dressing": True, "shutters": True, "pendant": True},   # a pendant light over the table   # lived in: upper cabinets and things on every surface (make_nights.demo_dressing)
}


def layout(seed):
    """the floor plan a house seed draws (sim-house-nights [uCP]), as a dict: furniture, regions, start, stubs, rug, carpet,
    lamp, bowls, mirrored. Seeds
    below 72 and those in FIXED_PLAN keep the one plan; from 72 up: the furniture moved in its groups, the stub walls and
    the entry nook moved, the rug with the living area, and half the houses mirrored left to right. Drawn again until
    every open cell is reachable from the dock. A hold-out from [ejh] on draws its own plan (unique_plan)"""
    if seed in DEMO_PLANS: return dict(DEMO_PLANS[seed])      # subsystem-demos [qEP]: a small room seen whole
    if seed in HOLDOUT_OWN: return unique_plan(seed)          # [ejh], [A7G]: never the tuning plan or a variant of it
    if seed is None or seed < 72 or seed in FIXED_PLAN:
        return {"furniture": list(BASE_FURNITURE), "regions": list(BASE_REGIONS), "start": BASE_START, "stubs": list(BASE_STUBS),
                "rug": dict(BASE_RUG), "carpet": list(BASE_CARPET), "lamp": BASE_LAMP, "bowls": list(BASE_BOWLS), "mirrored": False}
    rng = random.Random(seed * 7717 + 3)
    for _ in range(200):
        furn, shift = [], {}
        for members, (dx0, dx1), (dy0, dy1) in GROUPS:
            dx, dy = rng.randint(dx0, dx1), rng.randint(dy0, dy1)
            for f in BASE_FURNITURE:
                if f[5] in members:
                    d = dy * members[f[5]]; furn.append((f[0] + dx, f[1] + d, f[2] + dx, f[3] + d, f[4], f[5])); shift[f[5]] = (dx, d)
        c = 34 + rng.randint(-3, 3); top = rng.randint(3, 5); low = rng.randint(10, 12)
        nook_y = 20 + rng.randint(-2, 2); nook_x = rng.randint(5, 8)
        stubs = [(c, 0, c, top), (c, low, c, 13), (c, 13, c + 6, 13), (0, nook_y, nook_x, nook_y)]
        dx, dy = shift["sofa"]; rug = {"x": BASE_RUG["x"] + dx * CELL, "y": BASE_RUG["y"] - dy * CELL, "w": BASE_RUG["w"], "h": BASE_RUG["h"]}
        lamp = (BASE_LAMP[0] + dx * CELL, BASE_LAMP[1] - dy * CELL)                           # with the living area
        kx = shift["kitchen_counter"][0] * CELL; bowls = [(n, x + kx, y) for n, x, y in BASE_BOWLS]   # the counter's end
        ldx = shift["tv_console"][0]; carpet = [(0, 0, max(0, min(W - 1, 17 + ldx)), 19), (c, 0, W - 1, H - 1)]   # the play
        start, regions = BASE_START, list(BASE_REGIONS)                                      # area's from its stub wall
        mirrored = rng.random() < 0.5
        if mirrored:                                         # left to right: x -> W-1-x (cells), x -> -x (metres)
            fx = lambda x: W - 1 - x
            furn = [(fx(f[2]), f[1], fx(f[0]), f[3], f[4], f[5]) for f in furn]
            stubs = [(fx(x1), y0, fx(x0), y1) for x0, y0, x1, y1 in stubs]
            regions = [(fx(r[2]), r[1], fx(r[0]), r[3], r[4]) for r in regions]
            start = (fx(start[0]), start[1]); rug = dict(rug, x=-rug["x"])
            carpet = [(fx(x1), y0, fx(x0), y1) for x0, y0, x1, y1 in carpet]
            lamp = (-lamp[0], lamp[1]); bowls = [(n, -x, y) for n, x, y in bowls]
        if _valid(furn, stubs, start):
            return {"furniture": furn, "regions": regions, "start": start, "stubs": stubs, "rug": rug, "carpet": carpet,
                    "lamp": lamp, "bowls": bowls, "mirrored": mirrored}
    raise RuntimeError(f"no valid layout for seed {seed}")


def _grid(furn, stubs):
    g = [[0] * W for _ in range(H)]
    def fill(x0, y0, x1, y1, v):
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                g[y][x] = v
    fill(0, 0, W - 1, 0, 1); fill(0, H - 1, W - 1, H - 1, 1); fill(0, 0, 0, H - 1, 1); fill(W - 1, 0, W - 1, H - 1, 1)
    for s in stubs: fill(*s, 1)
    for f in furn: fill(f[0], f[1], f[2], f[3], 2)
    return g


def _valid(furn, stubs, start):
    """inside the outer walls, no piece on a wall or another piece, the dock open, every open cell reachable from it"""
    taken = set()
    for s in stubs: taken |= {(x, y) for y in range(s[1], s[3] + 1) for x in range(s[0], s[2] + 1)}
    for f in furn:
        if not (1 <= f[0] <= f[2] <= W - 2 and 1 <= f[1] <= f[3] <= H - 2): return False
        cells = {(x, y) for y in range(f[1] - 1, f[3] + 2) for x in range(f[0] - 1, f[2] + 2)}   # a cell's gap around it,
        if cells & taken: return False                                                          # walls allowed at its back
        taken |= {(x, y) for y in range(f[1], f[3] + 1) for x in range(f[0], f[2] + 1)}
    return _reach_all(furn, stubs, start)


def _reach_all(furn, stubs, start):
    """the dock open, and every open cell reachable from it"""
    from collections import deque
    g = _grid(furn, stubs)
    if g[start[1]][start[0]]: return False
    seen = {start}; q = deque([start])
    while q:
        x, y = q.popleft()
        for n in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= n[0] < W and 0 <= n[1] < H and n not in seen and g[n[1]][n[0]] == 0: seen.add(n); q.append(n)
    return len(seen) == sum(row.count(0) for row in g)


ROOM_MIN, DOOR_W = 7, (4, 6)     # cells: the narrowest room (2.45 m); a doorway's width (1.4-2.1 m; the variants' openings
                                 # are 1.8 m)
USE_WEIGHT = {"play": 4, "living": 3, "kitchen": 2, "dining": 2, "hall": 1}   # clutter by room, as REGIONS weights the base's


def _rooms(rng):
    """the footprint split into 3-5 rooms (rects of open cells): each split a wall across one room, with one or two
    doorways in it or an open end; a wall never ends in a doorway. Returns rooms, walls (stubs), doorway cells"""
    rooms, walls, door = [(1, 1, W - 2, H - 2)], [], set()
    for _ in range(rng.randint(2, 4)):
        cands, wts = [], []
        for i, (x0, y0, x1, y1) in enumerate(rooms):
            for vert, n in ((True, x1 - x0 + 1), (False, y1 - y0 + 1)):
                for c in range((x0 if vert else y0) + ROOM_MIN, (x1 if vert else y1) - ROOM_MIN + 1):
                    ends = ([(c + k, y) for k in range(-2, 3) for y in (y0 - 1, y1 + 1)] if vert else
                            [(x, c + k) for k in range(-2, 3) for x in (x0 - 1, x1 + 1)])
                    if not door.intersection(ends): cands.append((i, vert, c)); wts.append(n * n)
        if not cands: break
        i, vert, c = rng.choices(cands, wts)[0]; x0, y0, x1, y1 = rooms.pop(i)
        lo, hi = (y0, y1) if vert else (x0, x1); L = hi - lo + 1
        if rng.random() < 0.25:                                        # an open end: the wall stops short of one side
            k = rng.randint(5, max(5, L // 2)); gap = set(range(hi - k + 1, hi + 1) if rng.random() < 0.5 else range(lo, lo + k))
        else:                                                          # one doorway, or two in a long wall
            gap = set()
            for _ in range(2 if L >= 18 and rng.random() < 0.4 else 1):
                for _ in range(20):
                    w = rng.randint(DOOR_W[0], min(DOOR_W[1], L - 2)); s = rng.randint(lo + 1, hi - w)
                    if not gap.intersection(range(s - 3, s + w + 3)): gap |= set(range(s, s + w)); break
        run = [t for t in range(lo, hi + 1) if t not in gap]
        for t in range(lo, hi + 1):
            if t in gap: door.add((c, t) if vert else (t, c))
        segs = []
        for t in run:
            if segs and segs[-1][1] == t - 1: segs[-1][1] = t
            else: segs.append([t, t])
        walls += [(c, a, c, b) if vert else (a, c, b, c) for a, b in segs]
        rooms += [(x0, y0, c - 1, y1), (c + 1, y0, x1, y1)] if vert else [(x0, y0, x1, c - 1), (x0, c + 1, x1, y1)]
    return rooms, walls, door


def _side(room, s):
    """a room seen from one of its walls (s: 0 top, 1 left, 2 bottom, 3 right; also the quarter turns that face a piece
    backed to that wall into the room): its length along the wall, its depth, and (u along, v out from the wall) -> cell"""
    x0, y0, x1, y1 = room; w, h = x1 - x0 + 1, y1 - y0 + 1
    return [(w, h, lambda u, v: (x0 + u, y0 + v)), (h, w, lambda u, v: (x0 + v, y0 + u)),
            (w, h, lambda u, v: (x0 + u, y1 - v)), (h, w, lambda u, v: (x1 - v, y0 + u))][s]


def _side_m(room, s):
    """as _side, in continuous cells (u, v from the wall's face) -> world metres"""
    x0, y0, x1, y1 = room
    f = [lambda u, v: (x0 + u, y0 + v), lambda u, v: (x0 + v, y0 + u), lambda u, v: (x0 + u, y1 + 1 - v),
         lambda u, v: (x1 + 1 - v, y0 + u)][s]
    return lambda u, v: (lambda cx, cy: (round(cx * CELL - W * CELL / 2, 3), round(H * CELL / 2 - cy * CELL, 3)))(*f(u, v))


def _draw_plan(rng):
    """one try at a hold-out's own plan (unique_plan); None when the rooms drawn can't hold the house"""
    rooms, stubs, door = _rooms(rng)
    if len(rooms) < 3: return None
    uses = ["living", "kitchen", "play"] + rng.sample(["dining", "hall"], 2)[:len(rooms) - 3]
    order = list(range(len(rooms))); rng.shuffle(order); use = {u: rooms[order[i]] for i, u in enumerate(uses)}
    g = _grid([], stubs); keep = set()                                # keep: cells no piece may take
    for x, y in sorted(door): keep |= {(x + dx, y + dy) for dx in range(-2, 3) for dy in range(-2, 3)}   # by the doorways
    door_zone = set(keep); furn, facing, homes = [], {}, {}
    inside = lambda x, y: 0 <= x < W and 0 <= y < H
    def free(cells): return all(inside(x, y) and g[y][x] == 0 for x, y in cells)
    def rect(f, u0, v0, u1, v1):
        (ax, ay), (bx, by) = f(u0, v0), f(u1, v1); return (min(ax, bx), min(ay, by), max(ax, bx), max(ay, by))
    def cells(r): return [(x, y) for y in range(r[1], r[3] + 1) for x in range(r[0], r[2] + 1)]
    def put(r, h, name, q=0):
        for x, y in cells(r): g[y][x] = 2
        furn.append((r[0], r[1], r[2], r[3], h, name))
        if q % 4: facing[name] = q % 4
    def can(r, zone=(), back=()):                                     # the piece on open floor no zone holds, its zone
        return (free(cells(r)) and not keep.intersection(cells(r)) and free(zone)    # open, a wall all along its back
                and all(inside(x, y) and g[y][x] == 1 for x, y in back))
    def wall_spots(room, depth, length, front, sides=2):
        """every place a piece `length` x `depth` can stand backed to one of the room's walls, `sides` cells clear at its
        ends and `front` in front: (s, u, f, rect, zone)"""
        out = []
        for s in range(4):
            Lw, D, f = _side(room, s)
            if D < depth + front: continue
            for u in range(sides, Lw - sides - length + 1):
                r = rect(f, u, 0, u + length - 1, depth - 1)
                zone = [f(uu, vv) for uu in range(u - sides, u + length + sides) for vv in range(0, depth + front)
                        if not (u <= uu < u + length and vv < depth)]
                if can(r, zone, [f(uu, -1) for uu in range(u, u + length)]): out.append((s, u, f, r, zone))
        return out
    world = lambda c: cell_to_world(*c)
    def room_box(room, m=0.3):
        (ax, ay), (bx, by) = world(room[:2]), world(room[2:])
        return (round(min(ax, bx) - CELL / 2 + m, 3), round(min(ay, by) - CELL / 2 + m, 3),
                round(max(ax, bx) + CELL / 2 - m, 3), round(max(ay, by) + CELL / 2 - m, 3))

    R = use["living"]; spots = []                                     # the TV backed to a wall, the sofa facing it across
    a, b, c, gc = rng.randint(7, 9), rng.randint(7, 9), rng.randint(3, 4), rng.randint(1, 2)   # a coffee table
    for s in range(4):
        Lw, D, f = _side(R, s)
        for G in range(6, 9):                                         # TV to sofa 2.1-2.8 m; the sofa's back to the far
            behind = D - (3 + G)                                      # wall, or a walkway behind it
            if behind != 0 and behind < 3: continue
            for ut in range(2, Lw - 2 - a + 1):
                us = ut + (a - b) // 2; uc = us + (b - c) // 2; u0, u1 = min(ut, us) - 2, max(ut + a, us + b) + 1
                if u0 < 0 or u1 >= Lw: continue
                tv, sofa = rect(f, ut, 0, ut + a - 1, 0), rect(f, us, 1 + G, us + b - 1, 2 + G)
                table = rect(f, uc, G - gc - 1, uc + c - 1, G - gc)
                own = set(cells(tv)) | set(cells(sofa)) | set(cells(table))
                zone = [f(u, v) for u in range(u0, u1 + 1) for v in range(0, 3 + G + min(behind, 3))]
                zone = [z for z in zone if z not in own]
                back = [f(u, -1) for u in range(ut, ut + a)] + ([f(u, 3 + G) for u in range(us, us + b)] if behind == 0 else [])
                if can(tv, zone, back) and can(sofa) and can(table): spots.append((s, f, tv, sofa, table, zone, us, G))
    if not spots: return None
    s, f, tv, sofa, table, zone, us, G = rng.choice(spots)
    put(tv, 0.50, "tv_console", s); put(sofa, 0.85, "sofa", s + 2); put(table, 0.45, "coffee_table"); keep |= set(zone)
    lamp = tuple(round(v, 3) for v in world(f(rng.choice((us - 1, us + b)), 2 + G)))   # the floor lamp at one end of the sofa

    K = use["kitchen"]; n = rng.randint(10, 14)                       # the counter backed to a wall, room to work in front
    spots = wall_spots(K, 2, n, 3)
    if not spots: return None
    s, u, f, r, zone = rng.choice(spots); put(r, 0.90, "kitchen_counter", s); keep |= set(zone)
    Lw, D, _ = _side(K, s); island = None
    if rng.random() < 0.65:                                           # an island parallel to it, an aisle between, room to
        m, gi = rng.randint(7, min(9, n)), rng.randint(3, 4)          # sit on its far side
        opts = []
        for ui in range(u + (n - m) // 2 - 1, u + (n - m) // 2 + 2):
            ri = rect(f, ui, 2 + gi, ui + m - 1, 4 + gi)
            z = [f(uu, vv) for uu in range(ui - 2, ui + m + 2) for vv in range(gi, 8 + gi) if not (ui <= uu < ui + m and 2 + gi <= vv <= 4 + gi)]
            if 0 <= ui - 2 and ui + m + 1 < Lw and 8 + gi <= D and can(ri, z): opts.append((ui, ri, z))
        if opts:
            ui, ri, z = rng.choice(opts); put(ri, 0.90, "kitchen_island", s); keep |= set(z)
            island = _side_m(K, s)(ui + m / 2, 5 + gi + 0.35 / CELL), (s * math.pi / 2 + math.pi / 2)   # the stool at its seat side
    ends = [e for e, cs in ((-1, [f(u - 1, 0), f(u - 2, 0)]), (1, [f(u + n, 0), f(u + n + 1, 0)]))   # the bowls past one end
            if free(cs) and not door_zone.intersection(cs)]
    if not ends: return None
    e = rng.choice(ends); fm = _side_m(K, s); ue = u if e < 0 else u + n
    bowls = [(name, *fm(ue + e * d / CELL, 0.19 / CELL)) for name, d in (("dog_bowl_ceramic", 0.20), ("dog_bowl_plastic", 0.52))]
    stool = island or (fm(u + 0.5 / CELL if e > 0 else u + n - 0.5 / CELL, 2 + 0.35 / CELL),   # else at the counter, at the
                       s * math.pi / 2 + math.pi / 2)                                         # bowls' other end

    Dn = use.get("dining") or rng.choice([K, R])                      # the table free-standing, room round it for chairs
    tw, td = rng.randint(5, 6), 3; spots = []
    for along_x in (True, False):
        w_, h_ = (tw, td) if along_x else (td, tw)
        for y in range(Dn[1] + 3, Dn[3] - 3 - h_ + 2):
            for x in range(Dn[0] + 3, Dn[2] - 3 - w_ + 2):
                r = (x, y, x + w_ - 1, y + h_ - 1)
                zone = [(xx, yy) for yy in range(y - 3, y + h_ + 3) for xx in range(x - 3, x + w_ + 3) if not (x <= xx < x + w_ and y <= yy < y + h_)]
                if can(r, zone): spots.append((r, zone))
    if not spots: return None
    r, zone = rng.choice(spots); put(r, 0.75, "dining_table"); keep |= set(zone)
    (ax, ay), (bx, by) = world(r[:2]), world(r[2:]); tx0, tx1 = min(ax, bx) - CELL / 2, max(ax, bx) + CELL / 2
    ty0, ty1 = min(ay, by) - CELL / 2, max(ay, by) + CELL / 2; cx, cy = (tx0 + tx1) / 2, (ty0 + ty1) / 2; o = 0.33
    if r[2] - r[0] >= r[3] - r[1]:                                    # as the base plan's six round its table
        seats = {"chair_n1": (tx0 + 0.5, ty1 + o, -math.pi / 2), "chair_n2": (tx1 - 0.5, ty1 + o, -math.pi / 2),
                 "chair_s1": (tx0 + 0.5, ty0 - o, math.pi / 2), "chair_s2": (tx1 - 0.5, ty0 - o, math.pi / 2),
                 "chair_w": (tx0 - o, cy, 0.0), "chair_e": (tx1 + o, cy, math.pi)}
    else:
        seats = {"chair_n1": (tx1 + o, ty1 - 0.5, math.pi), "chair_n2": (tx1 + o, ty0 + 0.5, math.pi),
                 "chair_s1": (tx0 - o, ty1 - 0.5, 0.0), "chair_s2": (tx0 - o, ty0 + 0.5, 0.0),
                 "chair_w": (cx, ty1 + o, -math.pi / 2), "chair_e": (cx, ty0 - o, math.pi / 2)}
    for k, (x, y, yaw) in seats.items(): homes[k] = (round(x, 3), round(y, 3), round(yaw, 4), room_box(Dn))
    homes["stool"] = (*stool[0], round(stool[1], 4), room_box(K))

    P = use["play"]; spots = wall_spots(P, 2, rng.randint(5, 7), 3, sides=1)   # the play shelf backed to a wall
    if not spots: return None
    s, u, f, r, zone = rng.choice(spots); put(r, 1.20, "play_shelf"); keep |= set(zone)
    ln = (r[2] - r[0] + 1) if s % 2 == 0 else (r[3] - r[1] + 1)
    kx, ky = _side_m(P, s)(u + ln / 2, 2 + 0.5 / CELL)               # the kid's chair in front of it, facing it
    homes["kid_chair"] = (kx, ky, round(s * math.pi / 2 + math.pi / 2 + 0.3, 4), room_box(P))

    spots = []                                                        # the dock: backed to a wall, nothing near it
    for room in [use[k] for k in ("hall", "play", "living") if k in use]:
        for x in range(room[0], room[2] + 1):
            for y in range(room[1], room[3] + 1):
                near = [(x + dx, y + dy) for dx in range(-3, 4) for dy in range(-3, 4)]
                if ((x, y) in door_zone or not free([(x + dx, y + dy) for dx in range(-2, 3) for dy in range(-2, 3)])
                        or any(inside(*c) and g[c[1]][c[0]] == 2 for c in near)
                        or not any(inside(*c) and g[c[1]][c[0]] == 1 for c in ((x - 3, y), (x + 3, y), (x, y - 3), (x, y + 3)))): continue
                spots.append((x, y))
        if spots: break
    if not spots: return None
    start = rng.choice(spots)

    rw, rh = round(rng.uniform(2.0, 2.4), 2), round(rng.uniform(1.4, 1.7), 2)   # the rug on open floor by the sofa
    if rng.random() < 0.5: rw, rh = rh, rw
    (lx0, ly0, lx1, ly1), sx, sy = room_box(R, 0.0), *world(start)
    tcx, tcy = [(a_ + b_) / 2 for a_, b_ in zip(world(table[:2]), world(table[2:]))]
    rugs = []
    for x in range(R[0], R[2] + 1):
        for y in range(R[1], R[3] + 1):
            cx, cy = world((x, y))
            ux0, ux1, uy0, uy1 = cx - rw / 2 - 0.1, cx + rw / 2 + 0.1, cy - rh / 2 - 0.1, cy + rh / 2 + 0.1
            if ux0 < lx0 or ux1 > lx1 or uy0 < ly0 or uy1 > ly1 or (abs(cx - sx) < 1.4 + rw / 2 and abs(cy - sy) < 1.4 + rh / 2): continue
            under = [(xx, yy) for xx in range(int((ux0 + W * CELL / 2) / CELL), int((ux1 + W * CELL / 2) / CELL) + 1)
                     for yy in range(int((H * CELL / 2 - uy1) / CELL), int((H * CELL / 2 - uy0) / CELL) + 1)]
            if free(under): rugs.append((math.hypot(cx - tcx, cy - tcy), x, y, cx, cy))
    if not rugs: return None
    _, _, _, cx, cy = rng.choice(sorted(rugs)[:3])
    rug = {"x": round(cx, 3), "y": round(cy, 3), "w": rw, "h": rh}

    carpet = [use[k] for k, p in (("living", 0.9), ("play", 0.8), ("hall", 0.3), ("dining", 0.2)) if k in use and rng.random() < p]
    carpet = carpet or [R]                                            # carpet somewhere, the kitchen always hard floor
    regions = [(*use[k], USE_WEIGHT[k]) for k in uses]
    if not _reach_all(furn, stubs, start): return None
    wide = {(x, y) for y in range(1, H - 1) for x in range(1, W - 1)  # the robot walks into every room (a cell clear all
            if all(g[y + dy][x + dx] == 0 for dx in (-1, 0, 1) for dy in (-1, 0, 1))}   # round it)
    seen, q = {start}, [start]
    while q:
        x, y = q.pop()
        for nb in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if nb in wide and nb not in seen: seen.add(nb); q.append(nb)
    if not all(any(rm[0] <= x <= rm[2] and rm[1] <= y <= rm[3] for x, y in seen) for rm in rooms): return None
    return {"furniture": furn, "regions": regions, "start": start, "stubs": stubs, "rug": rug, "carpet": [tuple(c) for c in carpet],
            "lamp": lamp, "bowls": bowls, "mirrored": False, "facing": facing, "chair_homes": homes}


def unique_plan(seed):
    """a hold-out house's own floor plan (sim-house-nights [ejh], [A7G]), in layout()'s shape (and the pieces' facing and
    the chairs' homes): the footprint split into 3-5 rooms with their walls and doorways, each room given a use, and the
    furniture set as a household sets it: the TV console, counter and play shelf backed to a wall facing the room, the
    sofa facing the TV across a coffee table, an island parallel to the counter, the dining table free-standing with room
    for its chairs; carpet in the living and play rooms, the rug on open floor by the sofa, the lamp at the sofa's end,
    the bowls past the counter's end, the dock backed to a wall with nothing near it. Drawn again until every open cell
    is reachable from the dock and the robot can walk into every room. The same seed, the same plan"""
    rng = random.Random(seed * 104729 + 17)
    for _ in range(2000):
        p = _draw_plan(rng)
        if p: return p
    raise RuntimeError(f"no valid hold-out plan for seed {seed}")


def structure(plan):
    """a plan's fixed structure, for [ejh]'s measure: per cell 1 a wall inside the outer ones, 2 furniture, else 0"""
    g = _grid(plan["furniture"], plan["stubs"])
    return [[0 if x in (0, W - 1) or y in (0, H - 1) else g[y][x] for x in range(W)] for y in range(H)]


def plan_overlap(a, b, shift=6):
    """[ejh]'s measure of how alike two plans are: the share of their structure (inside walls and furniture, kind for kind)
    that coincides at their best alignment, over shifts of up to `shift` cells each way and mirroring either way (Jaccard:
    matching cells over cells either has). 1 is the same plan"""
    import numpy as np
    A, B = np.array(structure(a)), np.array(structure(b)); best = 0.0
    for Bf in (B, B[:, ::-1], B[::-1, :], B[::-1, ::-1]):
        win = np.lib.stride_tricks.sliding_window_view(np.pad(Bf, shift), A.shape)   # every shift of B, against A
        inter, union = ((win == A) & (A > 0)).sum((2, 3)), ((win > 0) | (A > 0)).sum((2, 3))
        best = max(best, float((inter / np.maximum(union, 1)).max()))
    return best


def use_layout(seed):
    """make this house's plan the one every reader of this module sees (FURNITURE, REGIONS, START, STUBS, RUG, CARPET,
    LAMP, BOWLS)"""
    global FURNITURE, REGIONS, START, STUBS, RUG, CARPET, LAMP, BOWLS, MIRRORED, LAYOUT_SEED, FACING, CHAIR_HOMES
    L = layout(seed); LAYOUT_SEED = seed
    FURNITURE, REGIONS, START, STUBS, RUG = L["furniture"], L["regions"], L["start"], L["stubs"], L["rug"]
    CARPET, LAMP, BOWLS, MIRRORED = L["carpet"], L["lamp"], L["bowls"], L["mirrored"]
    FACING, CHAIR_HOMES = L.get("facing", {}), L.get("chair_homes")   # a hold-out's own plan's only ([A7G])


if __import__("os").environ.get("HK_HOUSE_SEED"):    # every script run for a house (night_tidy.sh exports its seed) reads
    use_layout(int(__import__("os").environ["HK_HOUSE_SEED"]))   # that house's plan ([uCP])
HEIGHTS = [0.02, 0.04, 0.06, 0.08, 0.10, 0.15, 0.20, 0.30]   # spread evenly so every height gets samples
COLORS = ["0.85 0.2 0.2 1", "0.2 0.5 0.85 1", "0.95 0.75 0.1 1", "0.2 0.7 0.3 1", "0.6 0.3 0.75 1", "0.95 0.5 0.15 1"]


def base_grid():
    g = [[0] * W for _ in range(H)]
    def fill(x0, y0, x1, y1, v):
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                g[y][x] = v
    fill(0, 0, W - 1, 0, 1); fill(0, H - 1, W - 1, H - 1, 1); fill(0, 0, 0, H - 1, 1); fill(W - 1, 0, W - 1, H - 1, 1)
    for s in STUBS: fill(*s, 1)                        # stub walls only: play-area alcove and an entry nook (this house's)
    for f in FURNITURE:
        fill(f[0], f[1], f[2], f[3], 2)
    return g


def cell_to_world(x, y):
    """Cell (x right, y down) -> world metres, centred on the house."""
    return (x + 0.5) * CELL - W * CELL / 2, H * CELL / 2 - (y + 0.5) * CELL


def wall_rects(g):
    """Merge wall cells into rectangles: horizontal runs, then stack identical runs."""
    runs = {}
    for y in range(H):
        x = 0
        while x < W:
            if g[y][x] == 1:
                x0 = x
                while x < W and g[y][x] == 1:
                    x += 1
                runs.setdefault((x0, x - 1), []).append(y)
            else:
                x += 1
    rects = []
    for (x0, x1), ys in runs.items():
        ys.sort(); start = prev = ys[0]
        for y in ys[1:] + [None]:
            if y is not None and y == prev + 1:
                prev = y; continue
            rects.append((x0, start, x1, prev)); 
            if y is not None: start = prev = y
    return rects


def box_geom(name, x0, y0, x1, y1, h, rgba, material=None):
    (ax, ay), (bx, by) = cell_to_world(x0, y0), cell_to_world(x1, y1)
    cx, cy = (ax + bx) / 2, (ay + by) / 2
    sx, sy = (abs(bx - ax) + CELL) / 2, (abs(by - ay) + CELL) / 2
    mat = f' material="{material}"' if material else f' rgba="{rgba}"'
    return (f'    <geom name="{name}" type="box" pos="{cx:.3f} {cy:.3f} {h/2:.3f}" size="{sx:.3f} {sy:.3f} {h/2:.3f}"'
            f'{mat} contype="1" conaffinity="0"/>')


def clutter(g, seed, count):
    rng = random.Random(seed)
    tw = sum(r[4] for r in REGIONS)
    items, taken, tries = [], set(), 0
    def clear(x, y, r):                      # r cells of margin from walls/furniture/other items
        for yy in range(y - r, y + r + 1):
            for xx in range(x - r, x + r + 1):
                if not (0 <= xx < W and 0 <= yy < H) or g[yy][xx] or (xx, yy) in taken:
                    return False
        return True
    while len(items) < count and tries < 20000:
        tries += 1
        pick = rng.random() * tw
        for reg in REGIONS:
            pick -= reg[4]
            if pick <= 0:
                break
        x, y = rng.randint(reg[0], reg[2]), rng.randint(reg[1], reg[3])
        if abs(x - START[0]) <= 3 and abs(y - START[1]) <= 3:
            continue
        if not clear(x, y, 2):
            continue
        taken.add((x, y))
        i = len(items)
        h = HEIGHTS[i % len(HEIGHTS)]
        shape = "box" if rng.random() < 0.6 else "cylinder"
        wx, wy = cell_to_world(x, y)
        wx += rng.uniform(-0.08, 0.08); wy += rng.uniform(-0.08, 0.08)
        if shape == "box":
            sx, sy = rng.uniform(0.05, 0.15), rng.uniform(0.05, 0.15)
            footprint = [sx, sy]
        else:
            r = rng.uniform(0.04, 0.12); footprint = [r, r]
        items.append({"name": f"clutter_{i:02d}", "shape": shape, "x": round(wx, 3), "y": round(wy, 3),
                      "height_m": h, "half_extent_m": [round(v, 3) for v in footprint],
                      "rgba": COLORS[i % len(COLORS)], "yaw": round(rng.uniform(0, 3.1416), 3)})
    return items


def item_geom(it, solid=False):
    ct = "1" if solid else "0"   # non-solid: seen by depth cameras, but the robot can't trip on it
    x, y, h = it["x"], it["y"], it["height_m"]
    if it["shape"] == "box":
        sx, sy = it["half_extent_m"]
        return (f'    <body name="{it["name"]}" pos="{x} {y} {h/2:.3f}" euler="0 0 {it["yaw"]}">'
                f'<geom type="box" size="{sx} {sy} {h/2:.3f}" rgba="{it["rgba"]}" contype="{ct}" conaffinity="0"/></body>')
    r = it["half_extent_m"][0]
    return (f'    <body name="{it["name"]}" pos="{x} {y} {h/2:.3f}">'
            f'<geom type="cylinder" size="{r} {h/2:.3f}" rgba="{it["rgba"]}" contype="{ct}" conaffinity="0"/></body>')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--count", type=int, default=30)
    ap.add_argument("--solid", action="store_true", help="clutter collides with the robot (it can trip)")
    a = ap.parse_args()
    g = base_grid()
    geoms = [box_geom(f"wall_{i}", *r, WALL_H, None, "wall") for i, r in enumerate(wall_rects(g))]
    geoms += [box_geom(f["label"] if False else f"furn_{f[5]}", f[0], f[1], f[2], f[3], f[4], None, "furniture") for f in FURNITURE]
    items = clutter(g, a.seed, a.count)
    room = f"house_s{a.seed}_n{a.count}" + ("_solid" if a.solid else "")
    xml = f'''<?xml version="1.0" ?>
<mujoco model="scene_{room}">
  <compiler angle="radian"/>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.9 0.92 0.95" rgb2="0.6 0.65 0.7" width="512" height="512"/>
    <texture type="2d" name="floorwood" builtin="checker" rgb1="0.72 0.58 0.42" rgb2="0.68 0.54 0.39" width="512" height="512"/>
    <material name="floorwood" texture="floorwood" texuniform="true" texrepeat="8 8" reflectance="0.05"/>
    <material name="wall" rgba="0.93 0.91 0.87 1"/>
    <material name="furniture" rgba="0.45 0.38 0.32 1"/>
  </asset>
  <worldbody>
    <light name="sun" directional="true" pos="0 0 6" dir="0.2 0.3 -1" diffuse="0.7 0.7 0.7"/>
    <geom name="floor" type="plane" size="15 15 0.1" pos="0 0 0" material="floorwood" contype="1" conaffinity="0" priority="1" friction="1.0"/>
{chr(10).join(geoms)}
{chr(10).join(item_geom(it, a.solid) for it in items)}
  </worldbody>
  <visual>
    <headlight diffuse=".6 .6 .6" ambient=".35 .35 .35" specular="0.1 0.1 0.1"/>
    <global azimuth="120" elevation="-35"/>
  </visual>
</mujoco>
'''
    DIMOS_DATA.joinpath(f"scene_{room}.xml").write_text(xml)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    sx, sy = cell_to_world(*START)
    def bounds(x0, y0, x1, y1):
        (ax, ay), (bx, by) = cell_to_world(x0, y0), cell_to_world(x1, y1)
        return [round(min(ax, bx) - CELL / 2, 3), round(max(ax, bx) + CELL / 2, 3),
                round(min(ay, by) - CELL / 2, 3), round(max(ay, by) + CELL / 2, 3)]
    static = [bounds(*r) for r in wall_rects(g)] + [bounds(*f[:4]) for f in FURNITURE]
    truth = {"room": room, "static_bounds": static, "seed": a.seed, "count": len(items), "cell_m": CELL,
             "start_pos": [round(sx, 3), round(sy, 3)], "items": items}
    OUT_DIR.joinpath(f"{room}.truth.json").write_text(json.dumps(truth, indent=1))
    print(f"room={room} items={len(items)} start={sx:.3f},{sy:.3f}")
    print(f"scene: {DIMOS_DATA}/scene_{room}.xml")


if __name__ == "__main__":
    main()
