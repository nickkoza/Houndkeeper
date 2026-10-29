#!/usr/bin/env python3
"""Per-night find rates for the items dropped in the sim, from detect_results.json.
Writes runs/nights_s<seed>/metrics.csv and prints a table. Night 1 is the baseline (nothing to compare)."""
import argparse, csv, json
from pathlib import Path

KINDS = ["toy", "lego", "tissue", "string", "sock", "paper"]


def rows(run):
    res = json.loads((run / "detect_results.json").read_text()); out = []
    for n in sorted(res, key=int):
        r = res[n]; pk = r["per_kind"]
        f = sum(v["found"] for v in pk.values()); m = sum(v["missed"] for v in pk.values()); u = sum(v["unobserved"] for v in pk.values())
        row = {"night": int(n), "dropped": f + m + u, "seen_by_sweep": f + m, "found": f,
               "found_of_seen": f"{f/(f+m):.0%}" if f + m else "-", "found_of_dropped": f"{f/(f+m+u):.0%}" if f + m + u else "-",
               "false_alarms": r["false_alarms"], "stains_flagged_as_new": r["stains_flagged"]}
        for k in KINDS:
            v = pk.get(k, {"found": 0, "missed": 0, "unobserved": 0}); row[k] = f"{v['found']}/{v['found']+v['missed']}"
        out.append(row)
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--seed", type=int, default=7); a = ap.parse_args()
    run = Path(__file__).resolve().parent / "runs" / f"nights_s{a.seed}"; rs = rows(run)
    with open(run / "metrics.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rs[0])); w.writeheader(); w.writerows(rs)
    print(" | ".join(rs[0])); [print(" | ".join(str(v) for v in r.values())) for r in rs]
    print(f"saved {run/'metrics.csv'}")


def tier_rates(run, nights):
    """Found rate by difficulty tier over the given nights, counting only items the sweep observed."""
    import collections
    from difficulty import tier
    c = collections.defaultdict(lambda: {"found": 0, "seen": 0, "dropped": 0})
    for n in nights:
        p = run / f"items_n{n}.json"
        if not p.exists(): continue
        truth = json.loads(next(run.glob(f"nights_s*_n{n}.truth.json")).read_text())
        status = {r["id"]: r["status"] for r in json.loads(p.read_text())}
        for it in truth["items"]:
            tr = tier(it, truth["rug"])[0]; st = status.get(it["id"], "unobserved"); c[tr]["dropped"] += 1
            if st != "unobserved": c[tr]["seen"] += 1
            if st == "found": c[tr]["found"] += 1
    return {k: c[k] for k in ("easy", "medium", "hard")}


if __name__ == "__main__" and True:
    import sys
    run = Path(__file__).resolve().parent / "runs" / "nights_s7"
    for label, nights in (("tuning nights 2-4", [2, 3, 4]), ("held-out nights 5-8", [5, 6, 7, 8])):
        r = tier_rates(run, nights)
        if any(v["seen"] for v in r.values()):
            print(label + ": " + ", ".join(f"{k} {v['found']}/{v['seen']} ({v['found']/v['seen']:.0%})" if v["seen"] else f"{k} -" for k, v in r.items()))
