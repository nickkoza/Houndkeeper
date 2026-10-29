#!/usr/bin/env python3
"""The confirmer's replay scored against the sim's truth ([d8d], [QDc]): confirm_stream.replay run over a recorded sweep,
as the robot would have run it, then each item of the night's truth found if a confirmed blob reaches it. A scorer: the
truth is read here, after the replay, never by the confirmer ([1B2]). --truth (the sim's true path, traj.csv) places
the frames where the robot truly was (true_path.py's ablation: what pose error costs).
Usage: score_confirm.py --run-dir runs/nights_s8_r1 --seed 8 --night 5 --db <memory.db> [confirm_stream options]
       [--truth traj.csv]   (dimos venv python)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import json, time
from pathlib import Path

import numpy as np

import confirm_stream as cs


def score(recs, t, tprev):
    """As detect_changes2.py: an item is found if a confirmed blob reaches within its size + 6 cm (a string: its length + 5 cm);
    a confirmed blob matching no item, stain or item gone since last night is a false alarm (a stop for nothing)."""
    conf = [r for r in recs if r["decision"] == "confirmed"]; used = set(); per = {}
    def near(r, x, y, rad): return min(np.hypot(cx - x, cy - y) for cx, cy in r["cells"]) <= rad
    for it in t["items"]:
        rad = it.get("length_m", 0.30) + 0.05 if it["kind"] == "string" else max(it["half"]) + 0.06
        hit = [i for i, r in enumerate(conf) if near(r, it["x"], it["y"], rad)]; used |= set(hit)
        per.setdefault(it["kind"], [0, 0])[0 if hit else 1] += 1
        it["_found"] = bool(hit); it["_range"] = max((conf[i]["range_m"] for i in hit), default=None)
    stains = sum(any(near(r, s["x"], s["y"], s["r"] + 0.05) for r in conf) for s in t["stains"])
    for i, r in enumerate(conf):
        if any(near(r, s["x"], s["y"], s["r"] + 0.05) for s in t["stains"]): used.add(i)
    now = {it["id"] for it in t["items"]}
    for it in tprev.get("items", []):
        if it["id"] in now: continue
        used |= {i for i, r in enumerate(conf) if near(r, it["x"], it["y"], max(it["half"]) + 0.06)}
    return per, len(conf) - len(used), stains


def main(argv=None):
    ap = cs.parser()
    ap.add_argument("--truth", default=None, help="the sim's true path (traj.csv): frames placed where the robot truly was (sim only: what pose error costs)")
    a = ap.parse_args(argv); n = a.night; run = Path(a.run_dir)
    import true_path
    r = cs.replay(a, pose_hook=true_path.hook(a.truth) if a.truth else None); cf, recs = r["cf"], r["recs"]
    t = json.loads((run / f"nights_s{a.seed}_n{n}.truth.json").read_text())
    tp = run / f"nights_s{a.seed}_n{n-1}.truth.json"; tprev = json.loads(tp.read_text()) if tp.exists() else {}
    per, false, stains = score(recs, t, tprev)
    found = sum(v[0] for v in per.values()); tot = found + sum(v[1] for v in per.values())
    rngs = [it["_range"] for it in t["items"] if it["_found"]]
    dec = cs.summary(r)
    print(f"night {n} (baseline {r['prev']}): confirmed {found}/{tot} items; stops for nothing {false} (stains {stains}); "
          f"decisions {dec}; confirmed at {np.median(rngs) if rngs else float('nan'):.1f} m (median); by kind "
          f"{ {k: f'{v[0]}/{v[0]+v[1]}' for k, v in per.items()} }; floor judged {(cf.seen > 0).sum() * cs.RES * cs.RES:.0f} m2, "
          f"found changed twice {(cf.hit >= 2).sum() * cs.RES * cs.RES:.1f} m2; frames {r['used']}, {np.median(r['per_frame'])*1000:.0f} ms each (median); {time.time()-r['t0']:.0f}s")
    if a.out:
        Path(a.out).write_text(json.dumps({"night": n, "found": found, "items": tot, "false": false, "stains": stains,
                                           "decisions": recs, "per_kind": per, "snaps": cf.snaps, "args": vars(a)}, indent=1))


if __name__ == "__main__":
    main()
