#!/usr/bin/env python3
"""Score placed clutter against DimOS's live global_costmap. Read-only; safe to run mid-exploration.

Per item: detected (lethal cost within 1 cell of its footprint), missed (area observed, not lethal),
or unobserved (every footprint cell still unknown). Run with the dimos venv python.
Usage: score_clutter.py runs/<room>.truth.json [--wait 15] [--tag t300]
"""
import argparse, collections, json, math, sys, time
from pathlib import Path
import numpy as np
from dimos.core.transport_factory import transport_topic
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh

LETHAL = 100


def grab_costmap(wait_s):
    latest = {}
    bus = Zenoh(); bus.start()
    bus.subscribe(Topic(f"{transport_topic('/global_costmap')}/**"),
                  lambda msg, _t: latest.__setitem__("m", msg) if isinstance(msg, OccupancyGrid) else None)
    t0 = time.time()
    while time.time() - t0 < wait_s and "m" not in latest:
        time.sleep(0.2)
    time.sleep(1.0)                     # take the freshest after the first arrives
    bus.stop()
    return latest.get("m")


def score(cm, items):
    g = cm.grid; res = cm.resolution
    ox, oy = cm.origin.position.x, cm.origin.position.y
    out = []
    for it in items:
        r = max(it["half_extent_m"]) * (math.sqrt(2) if it["shape"] == "box" else 1)   # yaw-safe bound
        x0, x1 = int((it["x"] - r - ox) / res), int((it["x"] + r - ox) / res)
        y0, y1 = int((it["y"] - r - oy) / res), int((it["y"] + r - oy) / res)
        foot = g[max(y0, 0):max(y1 + 1, 0), max(x0, 0):max(x1 + 1, 0)]
        near = g[max(y0 - 1, 0):max(y1 + 2, 0), max(x0 - 1, 0):max(x1 + 2, 0)]
        if foot.size == 0 or np.all(foot == -1):
            status = "unobserved"
        elif near.size and near.max() >= LETHAL:
            status = "detected"
        else:
            status = "missed"
        out.append({**{k: it[k] for k in ("name", "shape", "x", "y", "height_m")},
                    "status": status, "max_cost": int(near.max()) if near.size else None})
    return out


def unexplained(cm, truth, level=50):
    """Cells at >= level cost that aren't within 15 cm of walls/furniture or 35 cm of an item: false positives."""
    g = cm.grid; res = cm.resolution
    ox, oy = cm.origin.position.x, cm.origin.position.y
    ys, xs = np.nonzero(g >= level)
    wx, wy = ox + (xs + .5) * res, oy + (ys + .5) * res
    ok = np.zeros(len(wx), bool)
    for x0, x1, y0, y1 in truth.get("static_bounds", []):
        ok |= (wx >= x0 - .15) & (wx <= x1 + .15) & (wy >= y0 - .15) & (wy <= y1 + .15)
    for it in truth["items"]:
        ok |= np.hypot(wx - it["x"], wy - it["y"]) < .35
    return len(wx), int((~ok).sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("truth"); ap.add_argument("--wait", type=float, default=15); ap.add_argument("--tag", default="")
    a = ap.parse_args()
    truth = json.load(open(a.truth))
    cm = grab_costmap(a.wait)
    if cm is None:
        sys.exit("no global_costmap message received; is the sim running?")
    rows = score(cm, truth["items"])
    # Graded view: DimOS's height_cost map is 0-100, not binary. 100 = planner refuses; 50 = goal-validator limit.
    print(f"costmap {cm.width}x{cm.height} @ {cm.resolution:.3f} m  known={100-cm.unknown_percent:.0f}%")
    print(f"{'height':>7} {'n':>3} {'unseen':>6} {'cost0':>6} {'1-49':>5} {'50-99':>6} {'100':>4} {'median':>7}")
    for h in sorted({r["height_m"] for r in rows}):
        rs = [r for r in rows if r["height_m"] == h]
        seen = [r["max_cost"] for r in rs if r["status"] != "unobserved"]
        b = lambda lo, hi: sum(lo <= c <= hi for c in seen)
        med = f"{sorted(seen)[len(seen)//2]:>7}" if seen else "      -"
        print(f"{h*100:>5.0f}cm {len(rs):>3} {len(rs)-len(seen):>6} {b(0,0):>6} {b(1,49):>5} {b(50,99):>6} {b(100,100):>4} {med}")
    if "static_bounds" in truth:
        for lv in (50, 100):
            n, bad = unexplained(cm, truth, lv)
            print(f"cells >= {lv}: {n}; unexplained (false positives): {bad} ({100*bad/max(n,1):.1f}%)")
    stem = Path(a.truth).with_suffix("").with_suffix("")
    out = Path(f"{stem}.score{('.' + a.tag) if a.tag else ''}.json")
    out.write_text(json.dumps({"costmap": {"w": cm.width, "h": cm.height, "res": cm.resolution,
                   "origin": [cm.origin.position.x, cm.origin.position.y]}, "items": rows}, indent=1))
    np.save(f"{stem}.costmap{('.' + a.tag) if a.tag else ''}.npy", cm.grid)
    print(f"saved {out.name}")


if __name__ == "__main__":
    main()
