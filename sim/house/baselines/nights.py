"""The night set ([8VL]): recorded tuning nights whose night-3 walk-through has toys down and a truth file, never a
hold-out (80, 81, 100-119; 78 is validation). Five from the detection bench's houses6r walks (one toy set each, both
sets used; house 72 left out while its live tidy's misses are looked into) and house 96's walk of 3 Oct (its recording
stamped on the robot's clock). Each night: its recording, its baselines (the floor comparison's run dir), the sim's path
(scoring and the recording's clock) and its truth (scoring only)."""
import json
import math
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[3]
HOLDOUTS = {78, 80, 81, *range(100, 120)}


def _bench(seed, toy_set, tidy):
    b = HK / "data" / "bench" / "houses6r"
    e = next(x for x in json.load(open(b / "nights.json")) if x["seed"] == seed and x["toy_set"] == toy_set)
    return {"id": f"s{seed}_t{toy_set}", "seed": seed, "db": e["db"], "run_dir": str(b / f"nights_s{seed}"),
            "tidy": str(HK / "data" / "runs" / "dimos_tidy" / tidy), "items": e["items"]}


def _s96():
    nt = HK / "data" / "runs" / "night_tidy" / "20261003_200709" / "items.json"
    tidy = HK / "data" / "runs" / "dimos_tidy" / "20261003_200750"
    return {"id": "s96_t1", "seed": 96, "db": (tidy / "recording.txt").read_text().strip(),
            "run_dir": str(HK / "data" / "bench" / "night96" / "nights_s96"), "tidy": str(tidy),   # its baselines as they were
                                                                       # (copied 5 Oct 2026: a new night of house 96 re-sweeps sim/house/runs')
            "items": json.loads(nt.read_text().split(":", 1)[1])}


def night_set():
    """the six nights, each {id, seed, db, run_dir, tidy, items, traj}"""
    out = [_bench(70, 1, "20261001_235547"), _bench(71, 2, "20261002_010214"), _bench(73, 1, "20261002_023644"),
           _bench(76, 2, "20261002_035054"), _bench(77, 1, "20261002_043011"), _s96()]
    for n in out:
        if n["seed"] in HOLDOUTS: raise SystemExit(f"night {n['id']}: a hold-out house")
        n["traj"] = str(Path(n["tidy"]) / "traj.csv")
        rec = Path(n["tidy"]) / "recording.txt"
        if rec.read_text().strip() != n["db"]: raise SystemExit(f"night {n['id']}: its tidy run lists another recording")
    return out


def carpet_rects(seed):
    """the house's carpeted rectangles in world metres (x0, x1, y0, y1), from its own plan ([uCP])"""
    sys.path.insert(0, str(HK / "sim" / "house")); import make_house as mh
    out = []
    for x0, y0, x1, y1 in mh.layout(seed)["carpet"]:
        (ax, ay), (bx, by) = mh.cell_to_world(x0, y0), mh.cell_to_world(x1, y1)
        out.append((min(ax, bx) - mh.CELL / 2, max(ax, bx) + mh.CELL / 2, min(ay, by) - mh.CELL / 2, max(ay, by) + mh.CELL / 2))
    return out


def truth_toys(n, named):
    """scoring only: the night's toys where they were dropped (as score_tidy) and where the walk left them (the tidy
    run's truth.json: the robot bumps some, 0.4-0.9 m in house 73), each with its find reach, its tier, the floor under
    it and whether a specific prompt names its kind (named(name) -> bool)"""
    sys.path.insert(0, str(HK / "sim" / "arm")); import items as _items
    from baselines import score
    t = json.load(open(Path(n["tidy"]) / "truth.json")); B = t["bodies"]
    n3 = json.load(open(Path(n["run_dir"]) / f"nights_s{n['seed']}_n3.truth.json"))
    rects = carpet_rects(n["seed"]); toys = []
    for name, uid, start in n["items"]:
        p, end = start, B.get(uid)
        half = _items.footprint_half(name); area = 4 * half[0] * half[1] * 1e4
        toys.append({"id": uid, "name": name, "x": float(p[0]), "y": float(p[1]), "reach": score.reach(half[0]),
                     "area_cm2": round(area, 1), "tier": score.tier_of(area),
                     "floor": score.floor_kind(p[0], p[1], rects, n3.get("rug")), "named": bool(named(name)),
                     "also": [[float(end[0]), float(end[1])]] if end else []})
    return toys, n3.get("stains", [])
