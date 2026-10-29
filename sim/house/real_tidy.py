#!/usr/bin/env python3
"""Tonight's candidates from a recording alone ([Lzt], [71R]): the floor comparison's decisions on tonight's recording
(confirm_stream.py --out, replayed by real_nights.sh TONIGHT=...) turned into the robot's list of places to look at,
and a night directory the morning log ([uoW]) and the phone cockpit read. Nothing here reads a truth file or a sim run:
only the decisions, the recording's path and its times.

The list (candidates.json): each place the comparison kept (confirmed or unsure, never a floor-sized change), places
within MERGE_M of a stronger one folded into it, strongest first; each with where it is in the house's frame (x, y:
where the floor map is drawn and the user's marks are, [vO9]) and in tonight's odometry (x_robot, y_robot: where the
robot would walk to), when it was decided, and the light check's verdict when one ran ([tTD]'s second stage): the
check is a hook (--check: a JSON {candidate id: {"keep": bool, "score": float}} made by whatever runs the check on the
same decisions), or the verdicts confirm_stream.py --check wrote into the decisions themselves (each candidate decision's
"check": {score, kept}); without either every candidate is kept and the list says it was not checked.

The night directory (night/): report.json in the live tidy's shape (t0, and one item per candidate: its flag, the
outcome "review: ..." so the log files it under "left for you to see to", and when it was decided), watch_only (no stop
was made: the robot walked its rounds), recording.txt (the recording's path) and night.json (only the morning log's
NIGHT_KEYS: mode "replay", started and ended from the recording's own times).
With --walk (the live tidy's status at the walk's end) the report says what the walk itself reported: no arm, and the
steps it skipped ([jFy]); each candidate carries the light check's best view (its frame's stamp and box), from which the
morning log cuts its picture.
Usage: real_tidy.py <decisions.json> <memory.db> <out dir> [--check check.json] [--walk status.json] [--mode m]
       [--route route.json]   (dimos venv python)"""
import argparse
import json
import math
import sys
from pathlib import Path

KEEP = ("confirmed", "unsure")       # the decisions the robot gets a second look at (floor_arm.candidates' default)
STRENGTH = {"confirmed": 1.0, "unsure": 0.5}
KEPT_SCORE = 1e9                     # a decision the check kept with no view of it ranks first (cascade.KEPT)
MERGE_M = 0.30                       # one thing seen in several decisions: the scorers' merge radius (score.MERGE_M)
OUTCOME = "review: flagged on the walk, not visited"


def candidates(decisions, check=None, merge_m=MERGE_M):
    """the robot's list from the comparison's decisions: kept ones only, folded within merge_m (house frame) into the
    strongest, each with its members' count. The light check's own verdicts (confirm_stream --check's, on each decision)
    drop each rejected decision before the fold, and the fold takes the best-scoring first, as the bench scores the
    launch arm (cascade.arm, score.merge): a rejected early look at a toy never takes a kept later one down with it.
    check ({id: {keep, score}}, an outside check of the folded places) drops what it rejected"""
    ds = [d for d in decisions if d.get("decision") in KEEP and d.get("kind") != "floor_change"]
    own = any(d.get("check") is not None for d in ds) and check is None
    if own:
        verdict = lambda d: d.get("check") or {}
        rejected = [d for d in ds if not verdict(d).get("kept", verdict(d).get("keep", True))]
        ds = [d for d in ds if verdict(d).get("kept", verdict(d).get("keep", True))]
        rank = lambda d: KEPT_SCORE if verdict(d).get("score") is None else float(verdict(d)["score"])   # no frame saw it: kept
        ds.sort(key=lambda d: (-rank(d), -STRENGTH[d["decision"]], d.get("ts", 0.0)))
    else:
        ds.sort(key=lambda d: (-STRENGTH[d["decision"]], d.get("ts", 0.0)))
    out = []
    for d in ds:
        for c in out:
            if math.hypot(d["x"] - c["x"], d["y"] - c["y"]) <= merge_m:
                c["members"] += 1; c["ts_last"] = max(c["ts_last"], d.get("ts", 0.0)); break
        else:
            out.append({"id": f"c{len(out) + 1}", "x": round(float(d["x"]), 3), "y": round(float(d["y"]), 3),
                        "x_robot": d.get("x_robot"), "y_robot": d.get("y_robot"), "ts": d.get("ts"), "ts_last": d.get("ts", 0.0),
                        "decision": d["decision"], "layer": d.get("layer", "1cm"), "area_cm2": d.get("area_cm2"), "members": 1,
                        "own_check": d.get("check")})
    for c in out:
        oc = c.pop("own_check"); c["view"] = best_view(oc)
        if own: c["check"] = {"keep": True, "score": oc.get("score") if oc else None}
    if own:
        return out, {"checked": True, "dropped": len(rejected)}
    if check is None:
        for c in out: c["check"] = None
        return out, {"checked": False}
    kept = []
    for c in out:
        v = check.get(c["id"])
        c["check"] = v
        if v is None or v.get("keep", True): kept.append(c)
    return kept, {"checked": True, "dropped": len(out) - len(kept)}


def best_view(verdict):
    """the light check's best-scoring view of a candidate ({ts, box_full: [x0, y0, side] in the full frame}), the crop
    the morning log shows for it; None when the check kept no views (an external check, or none ran)"""
    views = [v for v in ((verdict or {}).get("views") or []) if v.get("box_full") is not None and v.get("ts") is not None]
    if not views: return None
    v = max(views, key=lambda v: v.get("score") or 0.0)
    return {"ts": float(v["ts"]), "box_full": [int(round(float(x))) for x in v["box_full"]]}


def recording_times(db):
    """(first, last) stamp of the recording's head-camera frames: the night's start and end on the robot's clock"""
    from dimos.memory.store.sqlite import SqliteStore
    st = SqliteStore(path=str(db)); st.start()
    try:
        first = last = None
        for o in st.stream("color_image"):
            first = o.ts if first is None else first; last = o.ts
    finally:
        st.stop()
    return first, last


WALK_KEYS = ("arm", "arm_skipped", "state", "sweep_waypoints_not_reached")   # what the walk's own report adds ([jFy])


def night_dir(out, cands, db, started, ended, walk=None, mode="replay", route=None):
    """the night directory the morning log reads (report.json, recording.txt, night.json): no truth, only these. walk:
    the live tidy's own report of the walk (its status), from which only WALK_KEYS are kept (no arm: the steps it
    skipped); route: the route's file name beside it, for night.json"""
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    items = [dict({"flag": [c["x_robot"] if c["x_robot"] is not None else c["x"], c["y_robot"] if c["y_robot"] is not None else c["y"]],
                   "outcome": OUTCOME, "confirm": {"t": c["ts"], "why": c["decision"]}, "candidate": c["id"]},
                  **({"view": c["view"]} if c.get("view") else {})) for c in cands]
    rep = {"t0": started, "watch_only": True, "items": items}
    if walk:
        w = dict(walk); w["sweep_waypoints_not_reached"] = [s for s in (w.get("sweep") or []) if isinstance(s, dict) and "not_reached" in s]
        rep.update({k: w[k] for k in WALK_KEYS if w.get(k) not in (None, [])})
        if "state" in rep: rep["walk_state"] = rep.pop("state")
    (out / "report.json").write_text(json.dumps(rep, indent=1))
    (out / "recording.txt").write_text(str(db) + "\n")
    (out / "night.json").write_text(json.dumps({"seed": None, "mode": mode, "item_seed": None, "round": None,
                                                "night_tidy": None, "started": started, "ended": ended, "route": route}, indent=1))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("decisions"); ap.add_argument("db"); ap.add_argument("out"); ap.add_argument("--check", default=None)
    ap.add_argument("--walk", default=None, help="the live tidy's report of the walk (its status JSON)")
    ap.add_argument("--mode", default="replay", help="night.json's mode")
    ap.add_argument("--route", default=None, help="the route's file name in the night directory (route.json)")
    a = ap.parse_args(argv)
    D = json.loads(Path(a.decisions).read_text())
    check = json.loads(Path(a.check).read_text()) if a.check else None
    cands, how = candidates(D["decisions"], check)
    started, ended = recording_times(a.db)
    out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    (out / "candidates.json").write_text(json.dumps(dict(how, recording=str(a.db), started=started, ended=ended,
                                                         candidates=cands), indent=1))
    walk = None
    if a.walk and Path(a.walk).exists():
        w = Path(a.walk).read_text(); walk = json.loads(w[w.index("{"):]) if "{" in w else None
    night_dir(out / "night", cands, a.db, started, ended, walk, a.mode, a.route)
    print(f"candidates: {len(cands)} places ({sum(c['decision'] == 'confirmed' for c in cands)} confirmed) from "
          f"{len(D['decisions'])} decisions; " + ("checked" if how["checked"] else "not checked (no light check ran)")
          + f" -> {out / 'candidates.json'}", flush=True)


if __name__ == "__main__":
    sys.exit(main())
