#!/usr/bin/env python3
"""The vslice's success rates across DimOS tidy nights ([bOu]): each run's score.json (score_tidy.py) summed.
Usage: vslice_summary.py <dimos_tidy run dir>...   (e.g. data/runs/dimos_tidy/2026092[6]_*)"""
import json, sys
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent)); import run_speed  # noqa: E402
runs = [json.load(open(Path(d) / "score.json")) | {"dir": Path(d).resolve().name} for d in sys.argv[1:] if (Path(d) / "score.json").exists()]
if not runs: sys.exit("no score.json in those runs")
tot = Counter(); out = Counter()
for r in runs:
    tot.update({k: r.get(k, 0) for k in ("items", "binned", "found", "stops", "stops_for_nothing", "binnable", "too_big",
                                          "too_big_reported", "wrongly_too_wide", "handled")}); out.update(r["outcomes"])
    print(f"{r['dir']}: binned {r['binned']}/{r['items']}, found {r['found']}/{r['items']}, stops {r['stops']} "
          f"({r['stops_for_nothing']} for nothing), {r['state']}{' ' + str(r['error'])[:60] if r['error'] else ''}, "
          f"{(r['t_s'] or 0) / 60:.0f} min" + run_speed.flag(r.get("speed")))   # [5SS]
n = len(runs); i = max(tot["items"], 1)
print(f"\n{n} nights, {tot['items']} items: binned {tot['binned']} ({tot['binned'] / i:.0%}), found {tot['found']} ({tot['found'] / i:.0%}), "
      f"picked when found {tot['binned']}/{tot['found']} ({tot['binned'] / max(tot['found'], 1):.0%}); "
      f"stops for nothing {tot['stops_for_nothing'] / n:.1f} a night" + run_speed.flag_many(run_speed.combine([r.get("speed") for r in runs])))
if tot["binnable"] + tot["too_big"]:
    print(f"handled right {tot['handled']}/{tot['items']} ({tot['handled'] / i:.0%}): binned {tot['binned']} of {tot['binnable']} that fit "
          f"the jaws ({tot['binned'] / max(tot['binnable'], 1):.0%}); too big and left for review {tot['too_big_reported']} of {tot['too_big']}; "
          f"called too wide though they fit: {tot['wrongly_too_wide']}")
tiers = {}
for r in runs:
    for k, (a, b) in r.get("tiers", {}).items(): tiers.setdefault(k, [0, 0]); tiers[k][0] += a; tiers[k][1] += b
if tiers: print("by grasp tier ([QyZ]):", ", ".join(f"{k} {a}/{b} ({a / b:.0%})" for k, (a, b) in sorted(tiers.items())))
print("outcomes:", dict(out.most_common()))
