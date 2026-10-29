#!/usr/bin/env python3
"""How far outside the operating area the robot truly went ([JVr] [rLQ], measured in the sim): every foot's true position
from the sim's truth trajectory (traj.csv: FL/FR/RL/RR columns; a file without them falls back to the trunk's centre and
says so) against the area, in the house's frame, which in the sim starts as the sim's own frame (night 1's odometry
begins at the truth; the house frame is that night's, within a centimetre at its start). Sim truth is for scoring only.

Also what was targeted: every goal the planner was given (dimos.log "Got new goal", the robot's believed frame) and the
safe goal it chose, against the same area; and the tidy's own notes of finds seen outside (status.json).

Usage: area_excursion.py <tidy run dir: traj.csv, dimos.log, status.json> <operating_area.json>  -> JSON on stdout"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))
import json, re, sys
from pathlib import Path

import numpy as np


def outside_m(area, P):
    """per point (N x 2, house frame): 0 inside the area, else its distance to the area"""
    from dimos.mapping.keepout_zones import _inside, _segment_distance
    z = area.zones(); ins = np.zeros(len(P), bool)
    for p in z.allowed: ins |= _inside(P, p)
    d = np.full(len(P), np.inf)
    for p in z.allowed:
        for i in range(len(p)): d = np.minimum(d, _segment_distance(P[:, 0], P[:, 1], np.array([p[i], p[(i + 1) % len(p)]])))
    return np.where(ins, 0.0, d)


def edge_m(area, P):
    """per point (N x 2): its distance to the area's nearest edge, inside or out"""
    from dimos.mapping.keepout_zones import _segment_distance
    z = area.zones(); d = np.full(len(P), np.inf)
    for p in z.allowed:
        for i in range(len(p)): d = np.minimum(d, _segment_distance(P[:, 0], P[:, 1], np.array([p[i], p[(i + 1) % len(p)]])))
    return d


def crossed_walls(area, P):
    """how many steps of a path (N x 2) cross a virtual wall"""
    n = 0
    for (ax, ay), (bx, by) in area.zones().walls:
        s = np.sign((bx - ax) * (P[:, 1] - ay) - (by - ay) * (P[:, 0] - ax))
        flip = np.flatnonzero(s[1:] * s[:-1] < 0)
        for i in flip:                                    # the step's own line crosses the wall's segment too
            p, q = P[i], P[i + 1]; t = np.sign((q[0] - p[0]) * (np.array([ay, by]) - p[1]) - (q[1] - p[1]) * (np.array([ax, bx]) - p[0]))
            n += int(t[0] * t[1] < 0)
    return n


def goals(log: Path):
    pat = re.compile(r"(Got new goal|Found safe goal)[^\n]*?(?:pos=\[([-\d.]+), ([-\d.]+)|x=([-\d.]+) y=([-\d.]+))")
    out = {"requested": [], "safe": []}
    if not log.exists(): return out
    for m in pat.finditer(log.read_text(errors="replace")):
        x, y = (m.group(2), m.group(3)) if m.group(2) else (m.group(4), m.group(5))
        if x is None: continue
        out["requested" if m.group(1).startswith("Got") else "safe"].append((float(x), float(y)))
    return out


def main():
    run, area_file = Path(sys.argv[1]), sys.argv[2]
    from houndkeeper import operating_area as oa
    area = oa.load(area_file)
    T = np.genfromtxt(run / "traj.csv", delimiter=",", names=True)
    feet = [n for n in ("FL", "FR", "RL", "RR") if f"{n}_x" in T.dtype.names]
    pts = [np.c_[T[f"{n}_x"], T[f"{n}_y"]] for n in feet] or [np.c_[T["true_x"], T["true_y"]]]
    ex = np.max([outside_m(area, P) for P in pts], axis=0)
    trunk = np.c_[T["true_x"], T["true_y"]]
    i = int(np.argmax(ex))
    g = goals(run / "dimos.log")
    G = {k: (outside_m(area, np.array(v)) if v else np.zeros(0)) for k, v in g.items()}
    st = json.loads((run / "status.json").read_text()[(run / "status.json").read_text().index("{"):]) if (run / "status.json").exists() else {}
    print(json.dumps({
        "points": "feet" if feet else "trunk centre (no feet in this traj.csv)", "rows": int(len(T)),
        "duration_s": round(float(T["sim_t"][-1] - T["sim_t"][0]), 1),
        "max_excursion_m": round(float(ex.max()), 3), "at": {"sim_t": round(float(T["sim_t"][i]), 1), "x": round(float(trunk[i, 0]), 2), "y": round(float(trunk[i, 1]), 2)},
        "rows_any_foot_outside": int((ex > 0).sum()), "share_outside": round(float((ex > 0).mean()), 4),
        "p99_excursion_m": round(float(np.percentile(ex, 99)), 3),
        "closest_foot_to_edge_m": round(float(min(edge_m(area, P).min() for P in pts)), 3),
        "trunk_max_excursion_m": round(float(outside_m(area, trunk).max()), 3),
        "virtual_wall_crossings": crossed_walls(area, trunk) if area.walls else None,
        "goals": {k: {"n": int(len(v)), "outside": int((v > 0).sum()), "max_outside_m": round(float(v.max()), 3) if len(v) else 0.0,
                      "outside_list": [list(map(lambda a: round(a, 2), p)) for p, d in zip(g[k], v) if d > 0][:20]} for k, v in G.items()},
        "noted_outside": st.get("outside_area", []),
        "seen_outside_not_visited": sorted({(round(r["x"], 1), round(r["y"], 1)) for r in st.get("sweep", [])
                                            if isinstance(r, dict) and r.get("outside_area") and r.get("decision") == "confirmed"}),
        "pose_gaps": st.get("pose_gaps", []), "returns": st.get("returns", []), "stopped": st.get("stopped"),
        "state": st.get("state")}, indent=1))


if __name__ == "__main__":
    main()
