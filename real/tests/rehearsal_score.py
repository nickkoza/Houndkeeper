#!/usr/bin/env python3
"""The real-data rehearsal's scoring ([Lzt], [vO9]; analysis, reads the sim's truth): a sim night run through the real
robot's path (real_nights.sh on copies of its recordings, no truth) scored two ways, so each way checks the other.

sim:    the sim's own scorer on the rehearsal's decisions (floor_diag.score: each candidate carried to the sim's frame by
        the true path, the bench's toys), beside the same scorer on the sim run's own replay of that night (--reference,
        a floor_diag replay directory): the rehearsal's loss against the sim, if any, is the real path's.
marks:  the user's protocol ([hOy]) with the sim's truth standing in for the user: each toy "marked" where it lies in
        the house's frame (the true position carried into the frame the floor map is drawn in, by the rigid fit between
        the rehearsal's own candidates in that frame and the same candidates on the true path), every flag near no toy
        "judged" a false alarm (a sim night has no real change), then real/tests/score_night.py scores it as a real
        night. Writes marks.json, review.json and result.json beside candidates.json.
Usage: rehearsal_score.py <tonight dir> <night id, e.g. s70_t1_hr> [--reference <floor_diag replay dir>]   (dimos venv)"""
import argparse
import json
import math
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HK / "sim" / "house")); sys.path.insert(0, str(HK / "src")); sys.path.insert(0, str(Path(__file__).parent))

TINY_CM2 = 2.0      # floor_diag.TINY_CM2: the 1 x 1 brick reported apart


def sim_score(decisions_json, nid, label="loose1"):
    """the sim scorer on one night's decisions (floor_diag's replay file shape, in a scratch directory)"""
    from baselines import floor_diag as fd
    D = json.loads(Path(decisions_json).read_text())
    tmp = Path(tempfile.mkdtemp(prefix="rehearsal_score_"))
    try:
        (tmp / f"{nid}_{label}.json").write_text(json.dumps({"night": nid, "decisions": D["decisions"], "used": 0,
                                                             "ms_median": 0.0, "wall_s": 0.0}))
        return fd.score(str(tmp), label)["all"]
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def rigid_fit(src, dst):
    """the 2-D rotation and shift (R, t) taking src points onto dst (least squares, Kabsch), and the residual's RMS"""
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    ms, md = src.mean(0), dst.mean(0); U, _, Vt = np.linalg.svd((src - ms).T @ (dst - md))
    R = Vt.T @ np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T))]) @ U.T     # a turn, never a mirror
    t = md - R @ ms
    return R, t, float(np.sqrt(((src @ R.T + t - dst) ** 2).sum(1).mean()))


def marks_from_truth(decisions, nid):
    """the toys as the user would mark them on the floor map (house frame), from the sim's truth: marks.json's shape,
    and the fit's RMS (m)"""
    from baselines import floor_diag as fd, nights as nn, place as pl, prompts as pr
    n = next(x for x in fd.nights_all() if x["id"] == nid)
    toys, _ = nn.truth_toys(n, pr.named); to_true = pl.odom_to_true(n["traj"])
    pairs = [((d["x"], d["y"]), to_true(d["x_robot"], d["y_robot"], d["ts"])) for d in decisions if "x_robot" in d]
    R, t, rms = rigid_fit([p[1] for p in pairs], [p[0] for p in pairs])          # house <- true
    out = []
    for k in toys:
        x, y = R @ [k["x"], k["y"]] + t
        out.append({"id": k["id"], "name": k["name"], "x": round(float(x), 3), "y": round(float(y), 3),
                    "half_long_m": max(0.0, k["reach"] - 0.15) if k["reach"] > 0.30 else 0.0,
                    "brick": k["area_cm2"] < TINY_CM2, "floor": k["floor"]})
    return {"night": nid, "toys": out, "stand_in": "the sim's truth, carried into the house frame"}, rms


def main(argv=None):
    import score_night as sn
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("tonight"); ap.add_argument("night"); ap.add_argument("--reference", default=None)
    a = ap.parse_args(argv); tn = Path(a.tonight)
    if any(a.night.startswith(f"s{h}_") for h in (78, 80, 81, *range(100, 120))): raise SystemExit(f"{a.night}: a hold-out")
    D = json.loads((tn / "decisions.json").read_text())["decisions"]
    out = {"night": a.night, "sim": {"rehearsal": sim_score(tn / "decisions.json", a.night)}}
    if a.reference:
        ref = Path(a.reference) / f"{a.night}_loose1.json"
        out["sim"]["reference"] = sim_score(ref, a.night); out["sim"]["reference_file"] = str(ref)
    marks, rms = marks_from_truth(D, a.night)
    C = json.loads((tn / "candidates.json").read_text())
    first = sn.score_night(marks, C["candidates"])
    review = {i: "false_alarm" for f in first["flags"] for i in f["ids"]}
    r = sn.score_night(marks, C["candidates"], review); r["checked"] = C.get("checked")
    for name, v in (("marks.json", marks), ("review.json", review), ("result.json", r)):
        (tn / name).write_text(json.dumps(v, indent=1))
    out["marks"] = {"fit_rms_m": round(rms, 3), "found": r["found"], "n": r["n"], "main": r["main"], "bricks": r["bricks"],
                    "floor": r["floor"], "false_alarms": r["false_alarms"], "candidates": r["candidates"],
                    "pooled": sn.pool([r])}
    (tn / "rehearsal_score.json").write_text(json.dumps(out, indent=1))
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
