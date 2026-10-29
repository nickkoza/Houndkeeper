#!/usr/bin/env python3
"""A round of DimOS nights summed the way the hold-out is judged (sim-slice [bnO]): grippable toys by binned, too-big
toys by reported, never mixed (ungrippable toys never count in the grippable metrics);
stops for nothing; night length; and why each grippable toy that wasn't binned was lost.
Usage: round_summary.py 9 10 11 [--dir data/eval]"""
import argparse
import json
from collections import Counter
from pathlib import Path

import run_speed


def summarize(folder, rnd, prefix="r") -> dict:
    """round `rnd`'s nights: the <prefix><rnd>_s*_i*.score.json files in `folder` (prefix h: a hold-out run)"""
    nights = sorted(Path(folder).glob(f"{prefix}{rnd}_s*_i*.score.json"))
    s = {"round": rnd, "nights": len(nights), "grippable": 0, "binned": 0, "too_big": 0, "too_big_reported": 0, "too_big_binned": 0,
         "found": 0, "items": 0, "lost": Counter(), "easy": [0, 0], "hard": [0, 0], "stops_for_nothing": [], "minutes": [], "pack": []}
    speeds = []
    for p in nights:
        sc = json.loads(p.read_text())
        speeds.append(sc.get("speed"))                                        # how fast each night ran ([5SS])
        s["found"] += sc.get("found", 0); s["items"] += sc.get("items", 0)
        s["stops_for_nothing"].append(sc.get("stops_for_nothing", 0)); s["minutes"].append(sc.get("t_s", 0.0) / 60)
        if "energy" in sc: s["pack"].append(100 - sc["energy"]["air_left_pct"])     # the Air's pack used (since 28 Sep)
        for it in sc.get("per_item", []):
            if it.get("tier") == "too_big":
                s["too_big"] += 1; s["too_big_reported"] += bool(it.get("said_too_wide"))
                s["too_big_binned"] += bool(it.get("binned"))        # gripped by a part: better than reported (28 Sep)
                continue
            s["grippable"] += 1
            t = it.get("tier", "easy"); s[t][1] += 1; s[t][0] += bool(it.get("binned"))   # easy should bin more than hard ([QyZ])
            if it.get("binned"): s["binned"] += 1
            elif it.get("said_too_wide"): s["lost"]["refused as too wide"] += 1
            elif it.get("handled"): s["lost"]["handled, not binned"] += 1
            else: s["lost"]["never handled"] += 1
    n = max(len(nights), 1); sfn = s.pop("stops_for_nothing"); mins = s.pop("minutes"); pack = s.pop("pack")
    s["lost"] = dict(s["lost"])
    s["stops_for_nothing_per_night"] = sum(sfn) / n; s["worst_stops_for_nothing"] = max(sfn, default=0)
    s["minutes_per_night"] = sum(mins) / n; s["longest_minutes"] = max(mins, default=0.0)
    s["pack_used_pct_per_night"] = sum(pack) / len(pack) if pack else None; s["worst_pack_used_pct"] = max(pack, default=None)
    s["speed"] = run_speed.combine(speeds)
    return s


FOUND_MIN, BINNED_MIN, PACK_LEFT_MIN = 7 / 9, 0.5, 25.0   # the hold-out's bar (sim-slice [bnO])


def gate(s, nights=8):
    """is a round good enough to run the hold-out after it?
    The hold-out's own bar on the tuning houses: every night scored, 7 of 9 found, half the
    grippable toys binned, every night ending with a quarter of the pack. Too-big toys aren't gated here (one or two a
    night); the hold-out itself requires each binned or reported. Returns (passed, the reasons it didn't)"""
    why = []
    if s["nights"] < nights: why.append(f"only {s['nights']} of {nights} nights scored")
    if s["items"] and s["found"] / s["items"] < FOUND_MIN: why.append(f"found {s['found']}/{s['items']}, under 7 of 9")
    if s["grippable"] and s["binned"] / s["grippable"] < BINNED_MIN: why.append(f"binned {s['binned']}/{s['grippable']} grippable, under half")
    if s["worst_pack_used_pct"] is None: why.append("no night recorded its energy")
    elif 100 - s["worst_pack_used_pct"] < PACK_LEFT_MIN: why.append(f"a night ended with {100 - s['worst_pack_used_pct']:.0f}% of the pack, under 25%")
    return not why, why


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("rounds", nargs="+", type=int); ap.add_argument("--dir", default="data/eval")
    ap.add_argument("--prefix", default="r", help="r: tuning rounds; h: hold-out runs")
    ap.add_argument("--results-only", action="store_true", help="how it did, never why: the hold-out ([bnO]: no peeking)")
    ap.add_argument("--gate", type=int, metavar="NIGHTS", help="exit 0 only if the (single) round passes the hold-out's bar")
    a = ap.parse_args()
    print(f"{'round':>5} {'nights':>6} {'found':>9} {'binned':>12} {'too big: binned+told':>21} {'nothing/night':>13} {'worst':>5} "
          f"{'min/night':>9} {'pack %':>7} {'easy':>7} {'hard':>7}" + ("" if a.results_only else "  lost (grippable)"))
    for r in a.rounds:
        s = summarize(a.dir, r, a.prefix)
        if not s["nights"]: continue
        print(f"{r:>5} {s['nights']:>6} {s['found']:>4}/{s['items']:<4} {s['binned']:>5}/{s['grippable']:<3} ({100 * s['binned'] / max(s['grippable'], 1):3.0f}%)"
              f" {s['too_big_binned']:>7}+{s['too_big_reported']}/{s['too_big']:<11} {s['stops_for_nothing_per_night']:>13.1f} {s['worst_stops_for_nothing']:>5}"
              f" {s['minutes_per_night']:>9.1f} {'' if s['worst_pack_used_pct'] is None else f"{s['worst_pack_used_pct']:.0f} max":>7}"
              f" {s['easy'][0]:>3}/{s['easy'][1]:<3} {s['hard'][0]:>3}/{s['hard'][1]:<3}"
              + ("" if a.results_only else f"  {s['lost']}") + run_speed.flag_many(s["speed"]))   # [5SS]
    if a.gate:
        ok, why = gate(summarize(a.dir, a.rounds[0], a.prefix), a.gate)
        print("gate:", "PASS" if ok else "FAIL (" + "; ".join(why) + ")"); raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
