"""One scorer for every arm ([o0E], [oR6]): placed detections (place.place's points, or the floor comparison's
candidates, both in the odometry's frame with their time) are carried into the sim's frame, merged across frames within
MERGE_M, and scored against the night's truth with the tidy's own rule for a find (score_tidy.py: within NEAR_M of the
toy, or 15 cm past a long toy's half-length). A merged candidate near no toy is a false positive (a stop for nothing).
Toys are counted per difficulty tier (TIERS, fixed before any result) and per floor kind under them; [tWE]'s unknown
objects are the toys whose kind no specific prompt names (NAMED)."""
import math

NEAR_M = 0.30            # score_tidy.NEAR_M: a find is a candidate this close to the toy
MERGE_M = 0.30           # detections of one thing across frames: within this of a candidate already formed
RMAX_M = 2.5             # detections are placed out to the floor comparison's own reach (confirm_stream CONFIRM rmax)
TIERS = ((150.0, "easy"), (50.0, "medium"), (0.0, "hard"))   # footprint (cm2) at or above which a toy is this tier:
                         # size alone (the scanned toys carry no single colour for [D4v]'s contrast); fixed 4 Oct 2026
                         # before any result


def merge(points, radius=MERGE_M):
    """points ({x, y, score, ...}) grouped across frames: strongest first, each joining the first candidate whose
    centre lies within radius, else starting one. Each candidate: centre (score-weighted mean), its members, best score"""
    out = []
    for p in sorted(points, key=lambda p: -p.get("score", 1.0)):
        for c in out:
            if math.hypot(p["x"] - c["x"], p["y"] - c["y"]) <= radius:
                c["members"].append(p); w = [max(m.get("score", 1.0), 1e-6) for m in c["members"]]
                c["x"] = sum(m["x"] * wi for m, wi in zip(c["members"], w)) / sum(w)
                c["y"] = sum(m["y"] * wi for m, wi in zip(c["members"], w)) / sum(w)
                break
        else:
            out.append({"x": p["x"], "y": p["y"], "score": p.get("score", 1.0), "members": [p]})
    for c in out:
        c["frames"] = len({m.get("frame", id(m)) for m in c["members"]})
        c["labels"] = sorted({m.get("label", "") for m in c["members"]} - {""})
    return out


def tier_of(area_cm2):
    """the toy's difficulty tier from its footprint on the floor (TIERS)"""
    return next(name for lo, name in TIERS if area_cm2 >= lo)


def reach(half_long):
    """how near a candidate must be to find a toy whose long half-length is half_long (m): score_tidy's _REACH"""
    return max(NEAR_M, half_long + 0.15)


def floor_kind(x, y, carpet_rects, rug):
    """'rug', 'carpet' or 'hard' under (x, y): the rug (its rectangle, turned by yaw_deg), then the house's carpeted
    rectangles (world x0, x1, y0, y1), else the hard floor"""
    if rug:
        th = math.radians(-rug.get("yaw_deg", 0.0)); dx, dy = x - rug["x"], y - rug["y"]
        lx, ly = dx * math.cos(th) - dy * math.sin(th), dx * math.sin(th) + dy * math.cos(th)
        if abs(lx) <= rug["w"] / 2 and abs(ly) <= rug["h"] / 2: return "rug"
    if any(x0 <= x <= x1 and y0 <= y <= y1 for x0, x1, y0, y1 in carpet_rects): return "carpet"
    return "hard"


def score_night(cands, toys, stains=()):
    """cands: merged candidates (sim frame). toys: [{id, name, x, y, reach, tier, floor, named, also?}] (x, y where it
    was dropped, as score_tidy; also: other places it truly lay, such as where the walk left it). Returns the night's
    result: each toy found or not (and by which candidate), the false positives, how many of them lie on a stain and
    how many within twice a toy's reach (a toy seen but placed too far off, most likely)"""
    per = []
    def d(c, t):                                   # to the toy where it was dropped, or where it ended (a toy the robot
        return min(math.hypot(c["x"] - x, c["y"] - y) for x, y in [(t["x"], t["y"])] + list(t.get("also", [])))   # bumped)
    for t in toys:
        hit = [i for i, c in enumerate(cands) if d(c, t) < t["reach"]]
        per.append(dict(t, found=bool(hit), by=hit))
    used = {i for p in per for i in p["by"]}
    fps = [c for i, c in enumerate(cands) if i not in used]
    on_stain = sum(any(math.hypot(c["x"] - s["x"], c["y"] - s["y"]) < s["r"] + 0.10 for s in stains) for c in fps)
    near_toy = sum(any(d(c, t) < 2 * t["reach"] for t in toys) for c in fps)
    return {"toys": per, "found": sum(p["found"] for p in per), "n": len(per), "fp": len(fps), "fp_on_stain": on_stain,
            "fp_near_toy": near_toy,
            "fp_cands": fps, "cands": len(cands)}


def tally(nights, key):
    """found/total over nights, grouped by each toy's `key` ('tier', 'floor', 'named')"""
    out = {}
    for r in nights:
        for t in r["toys"]:
            k = t[key]; out.setdefault(k, [0, 0]); out[k][0] += t["found"]; out[k][1] += 1
    return out
