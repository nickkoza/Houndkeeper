#!/usr/bin/env python3
"""A real night scored without the sim's truth ([vO9], [71R], [iDT]; decisions [hOy], [wTj]): the robot's candidate
list (real_tidy.py's candidates.json, in the house's frame) matched by place to the toys the user laid out and marked
on the house's floor map before the night (marks.json), with the sim scorer's own rule for a find (sim/house/baselines/
score.py: a candidate within a toy's reach, 30 cm or 15 cm past half a long toy's length; candidates within 30 cm one).
A candidate near no marked toy is a flag the user judges in the cockpit (review.json): a false alarm, or a real change
(something on the floor that was not laid out, which then counts neither way). The night is final when no flag is
left unjudged. Nights are pooled into the sim's own figures (found and its share, false alarms a night, per floor kind,
the bricks apart: floor_diag.summarise's keys), so the gap report ([KbT]) sets them beside the sim's. The first two
real nights set the light check's threshold ([wTj]): their results say tuning, and the pool reports them apart, never
in the launch number. The files live in the night's own directory ([oWO]); the phone cockpit's Marks and Review pages
write marks.json and review.json there (houndkeeper.night_scoring).

marks.json:   {"night": name, "toys": [{"id", "name", "x", "y", "half_long_m", "brick": bool, "floor": "hard"|"carpet"|"rug"}]}
              (house frame, metres; a night with no toys laid out still counts its false alarms)
review.json:  {candidate id: "false_alarm" | "real_change"}
Usage: score_night.py night <marks.json> <candidates.json> [--review review.json] [--out result.json] [--tuning]
       score_night.py pool <result.json> ...                       (any python 3)"""
import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim" / "house"))
from baselines import score as sc   # noqa: E402  the sim's one scorer ([o0E]): merge, reach and the night's score

FLOORS = ("hard", "carpet", "rug")
JUDGEMENTS = ("false_alarm", "real_change")
TUNING_NIGHTS = 2      # the first real nights, which set the light check's threshold ([wTj]): never in the launch number


def score_night(marks, cands, review=None, tuning=False):
    """one night: each marked toy found or not, each unmatched flag with the user's judgement (or none yet); tuning:
    one of the first TUNING_NIGHTS real nights ([wTj])"""
    review = review or {}
    bad = sorted(set(review.values()) - set(JUDGEMENTS))
    if bad: raise SystemExit(f"review.json: judgements must be {' or '.join(JUDGEMENTS)}, not {', '.join(bad)}")
    toys = [{"id": t["id"], "name": t.get("name", t["id"]), "x": float(t["x"]), "y": float(t["y"]),
             "reach": sc.reach(float(t.get("half_long_m", 0.0))), "brick": bool(t.get("brick", False)),
             "floor": t.get("floor", "hard")} for t in marks.get("toys", [])]
    merged = sc.merge([{"x": c["x"], "y": c["y"], "score": 1.0 if c.get("decision") == "confirmed" else 0.5, "frame": c["id"]}
                       for c in cands])
    r = sc.score_night(merged, toys)
    flags = []
    for c in r["fp_cands"]:
        ids = sorted(m["frame"] for m in c["members"]); js = [review.get(i) for i in ids]
        j = "false_alarm" if "false_alarm" in js else "real_change" if js and all(x == "real_change" for x in js) else None
        flags.append({"ids": ids, "x": round(c["x"], 3), "y": round(c["y"], 3), "judged": j})
    by_floor = {k: [sum(t["found"] for t in r["toys"] if t["floor"] == k and not t["brick"]),
                    sum(t["floor"] == k and not t["brick"] for t in r["toys"])] for k in FLOORS}
    return {"night": marks.get("night"), "tuning": bool(tuning), "laid_out": len(toys) > 0,
            "toys": [{k: t[k] for k in ("id", "name", "x", "y", "floor", "brick", "found")} for t in r["toys"]],
            "found": r["found"], "n": r["n"],
            "main": [sum(t["found"] for t in r["toys"] if not t["brick"]), sum(not t["brick"] for t in r["toys"])],
            "bricks": [sum(t["found"] for t in r["toys"] if t["brick"]), sum(t["brick"] for t in r["toys"])],
            "floor": by_floor, "candidates": len(merged), "flags": flags,
            "false_alarms": sum(f["judged"] == "false_alarm" for f in flags),
            "real_changes": sum(f["judged"] == "real_change" for f in flags),
            "unjudged": sum(f["judged"] is None for f in flags),
            "final": all(f["judged"] is not None for f in flags)}


def wilson(k, n, z=1.96):
    """the 95% interval of a share k/n (Wilson's), (lo, hi); (0, 1) with no toys"""
    if n == 0: return (0.0, 1.0)
    p = k / n; d = 1 + z * z / n; c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (round(c - h, 3), round(c + h, 3))


def figures(results):
    """nights' figures, in the sim's own keys (floor_diag.summarise: found, n, main, tiny, fp_per_night, floor), with the
    found share's interval"""
    laid = [r for r in results if r["laid_out"]]
    found, n = sum(r["found"] for r in laid), sum(r["n"] for r in laid)
    fl = {k: [sum(r["floor"][k][0] for r in laid), sum(r["floor"][k][1] for r in laid)] for k in FLOORS}
    return {"nights": len(results), "laid_out_nights": len(laid), "found": found, "n": n,
            "found_share": round(found / n, 3) if n else None, "found_share_95": wilson(found, n),
            "main": [sum(r["main"][0] for r in laid), sum(r["main"][1] for r in laid)],
            "tiny": [sum(r["bricks"][0] for r in laid), sum(r["bricks"][1] for r in laid)],
            "fp_per_night": round(sum(r["false_alarms"] for r in results) / max(len(results), 1), 1),
            "fp": {str(r["night"]): r["false_alarms"] for r in results},
            "real_changes": sum(r["real_changes"] for r in results), "floor": fl}


def pool(results):
    """nights pooled into the launch number, in the sim's figures; a counted night not yet final is refused, so a number
    never rests on unjudged flags. The tuning nights ([wTj]) are kept apart: their own figures under "tuning", named in
    "tuning_nights", and the pool says why they are not counted"""
    tune = [r for r in results if r.get("tuning")]; count = [r for r in results if not r.get("tuning")]
    open_ = [r["night"] for r in count if not r["final"]]
    if open_: raise SystemExit(f"nights with flags not yet judged: {', '.join(map(str, open_))}")
    out = figures(count)
    out["tuning_nights"] = [str(r["night"]) for r in tune]
    out["tuning"] = dict(figures(tune), unjudged=[str(r["night"]) for r in tune if not r["final"]]) if tune else None
    out["note"] = (f"The first {TUNING_NIGHTS} real nights set the light check's threshold: "
                   f"{', '.join(out['tuning_nights'])} not counted in the launch number.") if tune else ""
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0]); sp = ap.add_subparsers(dest="cmd", required=True)
    a_ = sp.add_parser("night"); a_.add_argument("marks"); a_.add_argument("candidates")
    a_.add_argument("--review", default=None); a_.add_argument("--out", default=None)
    a_.add_argument("--tuning", action="store_true", help=f"one of the first {TUNING_NIGHTS} real nights ([wTj])")
    p_ = sp.add_parser("pool"); p_.add_argument("results", nargs="+")
    a = ap.parse_args(argv)
    if a.cmd == "night":
        C = json.loads(Path(a.candidates).read_text())
        r = score_night(json.loads(Path(a.marks).read_text()), C["candidates"],
                        json.loads(Path(a.review).read_text()) if a.review else None, a.tuning)
        r["checked"] = C.get("checked")
        if a.out: Path(a.out).write_text(json.dumps(r, indent=1))
        print(f"{r['night']}: found {r['found']}/{r['n']} (bricks {r['bricks'][0]}/{r['bricks'][1]}), "
              f"{len(r['flags'])} flags near no toy: {r['false_alarms']} false alarms, {r['real_changes']} real changes, "
              f"{r['unjudged']} to judge" + ("" if r["final"] else " (not final)") + (" (a tuning night)" if r["tuning"] else ""))
    else:
        print(json.dumps(pool([json.loads(Path(p).read_text()) for p in a.results]), indent=1))


if __name__ == "__main__":
    main()
