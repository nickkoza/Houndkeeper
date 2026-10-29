#!/usr/bin/env python3
"""Score a DimOS tidy run against the sim's truth: which items really ended in the bin, which the robot left, and
whether its own report matched. Sim-only (the real robot has no truth file)."""
import json, sys
from pathlib import Path
out = Path(sys.argv[1]); t = json.loads(open(out / "truth.json").read()); rep = open(out / "report.json").read(); rep = json.loads(rep[rep.index("{"):])
items = json.loads(__import__("os").environ.get("HK_SIM_ITEMS", "[]")) or []
B = t["bodies"]; tray = B.get("tray"); trunk = B.get("trunk"); trunk_q = (t.get("quats") or {}).get("trunk")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "arm")); import bin_geom as _bin
def _in_bin(p):   # its centre over the bin, off the floor ([QyZ], 29 Sep; was 35 cm from the trunk's centre)
    return p is not None and trunk is not None and trunk_q is not None and _bin.in_bin(p, trunk, trunk_q)
print("robot trunk", trunk)
for name, uid, start in items:
    p = B.get(uid)
    if p is None: print(f"{uid}: no truth"); continue
    inbin = _in_bin(p)
    print(f"{uid:14s} start {start[:2]} now {[round(v, 2) for v in p]} -> {'IN BIN (on the robot)' if inbin else 'on the floor'}")
print("robot's report:", [(i['flag'], i.get('outcome')) for i in rep.get("items", [])])

# the night's numbers ([bOu]), for success rates across nights (sim/dimos/vslice_summary.py): a find is a stop within
# NEAR_M of an item's start (the head camera places a flag a few cm off a 10 cm toy), or one that binned something; a
# stop that is neither is a stop for nothing
NEAR_M = 0.30
flags = [(i["flag"], str(i.get("outcome"))) for i in rep.get("items", []) if i.get("flag")]
near = lambda f, s: ((f[0] - s[0]) ** 2 + (f[1] - s[1]) ** 2) ** 0.5 < _REACH.get(tuple(s[:2]), NEAR_M)
_REACH = {}                                                             # per item below: NEAR_M, or 15 cm past a long toy's
                                                                        # half-length (its origin can sit at one end)
res = {"items": len(items), "binned": 0, "found": 0, "stops": len(flags),
       "stops_for_nothing": sum(1 for f, o in flags if not any(near(f, st) for _, _, st in items) and not o.startswith(("same as", "binned"))),
       "outcomes": {}, "state": rep.get("state"), "error": rep.get("error"), "t_s": None}
# what each item should get (too big counts as a success when the item really is too big): one whose narrow side fits
# the open jaws is binned; one that doesn't is left and reported for review as too wide. Whether it fits is the item's
# own geometry resting upright (items.footprint_half), never the robot's measurement; calling a fitting item too wide
# is a miscategorisation, counted apart
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "arm")); import items as _items, jaws as _jaws
res.update(binnable=0, too_big=0, too_big_reported=0, wrongly_too_wide=0, handled=0, per_item=[])
for name, uid, start in items: _REACH[tuple(start[:2])] = max(NEAR_M, _items.footprint_half(name)[0] + 0.15)
for name, uid, start in items:
    p = B.get(uid)
    binned_it = _in_bin(p)
    res["binned"] += binned_it
    if binned_it or any(near(f, start) for f, _ in flags): res["found"] += 1
    q = (t.get("rest_quats") or {}).get(uid)                            # as it lay when the night began (DimOS's truth),
    narrow = _items.grip_width_at(name, q) if q else _items.grip_width(name)   # else as it lands in a test drop
    tier = _items.grasp_tier(name, q)                                          # [QyZ], as it lies tonight
    fits = tier != "too_big"
    said_wide = any(near(f, start) and ("too wide" in o or "too big" in o) for f, o in flags)   # reported too big, either way
    res["binnable" if fits else "too_big"] += 1
    if not fits and said_wide: res["too_big_reported"] += 1
    if fits and said_wide and not binned_it: res["wrongly_too_wide"] += 1
    ok = bool(binned_it or (not fits and said_wide)); res["handled"] += ok            # what it binned fitted, whatever we measured
    res.setdefault("tiers", {}).setdefault(tier, [0, 0]); res["tiers"][tier][0] += ok; res["tiers"][tier][1] += 1
    res["per_item"].append({"item": uid, "grip_cm": round(narrow * 100, 1), "tier": tier, "binned": bool(binned_it), "said_too_wide": said_wide, "handled": ok})
for f, o in flags:
    k = "same as another" if o.startswith("same as") else o.split(" (")[0]; res["outcomes"][k] = res["outcomes"].get(k, 0) + 1
log = rep.get("log") or []
if log:                                              # the night's length: its last log line, or its last sweep record if
    try: res["t_s"] = float(str(log[-1]).split("s")[0])   # later (a walk-through logs nothing after its first second: 0.7 s
    except ValueError: pass                          # for a 6 min walk, 3 Oct 2026); both on the robot's clock
ts = [w["t"] for w in rep.get("sweep", []) if isinstance(w, dict) and isinstance(w.get("t"), (int, float))]
if ts and isinstance(rep.get("t0"), (int, float)): res["t_s"] = round(max(res["t_s"] or 0.0, max(ts) - rep["t0"]), 1)
res["waypoints_not_reached"] = [w for w in rep.get("sweep", []) if isinstance(w, dict) and w.get("not_reached")]   # a sweep that
ENERGY = Path("/dev/shm/houndkeeper_energy.json")   # the battery model per pack, as the sim hook last wrote it ([bOu])
try:
    if (out / "energy.json").exists():                                          # the night's own, saved when it was scored
        res["energy"] = json.loads((out / "energy.json").read_text())
    elif abs(ENERGY.stat().st_mtime - (out / "report.json").stat().st_mtime) < 600:   # written as this night ended (the
        res["energy"] = json.loads(ENERGY.read_text()); (out / "energy.json").write_text(ENERGY.read_text())   # file is shared)
except (OSError, ValueError):
    pass
import run_speed                                                                # how fast the night ran ([5SS]): the sim's
res["speed"] = run_speed.of_run(out)                                            # own report, its numbers marked if slow
json.dump(res, open(out / "score.json", "w"), indent=1)                                                                # never walked
print("score:", json.dumps({k: v for k, v in res.items() if k not in ("outcomes", "per_item")}) + run_speed.flag(res["speed"]))
