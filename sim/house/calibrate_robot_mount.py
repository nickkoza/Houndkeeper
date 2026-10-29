#!/usr/bin/env python3
"""The robot's head-camera mount, calibrated over tuning houses ([gJq], decision [gsJ]): each seed's own stitch solve
(runs/nights_s<seed>/stitch_mount.json; a run from before 3 Oct kept it as head_mount.json), those not refused, and the
median pitch, yaw and roll written to data/robot/head_mount.json; refuses hold-out seeds before reading anything. Since
[yFg] (7 Oct 2026) the robot's mount comes from the floor self-calibration (camgeo.py calibrate, which also solves the
camera's height): this step refuses to write over a mount that one wrote.
Usage: calibrate_robot_mount.py <seed>... [--out data/robot/head_mount.json]"""
import argparse, json, time
from pathlib import Path

import numpy as np

from calibrate_lag import refuse_holdout

HERE = Path(__file__).resolve().parent


def house_solve(run):
    """a run dir's own stitch solve of the mount, or None (refused, or none)"""
    run = Path(run); p = run / "stitch_mount.json"
    if not p.exists() and not (run / "calibration.json").exists(): p = run / "head_mount.json"   # before [gJq]
    if not p.exists(): return None
    d = json.loads(p.read_text())
    return None if d.get("refused", True) else d


def calibrate(seeds, runs):
    seeds = refuse_holdout(seeds)
    got = {s: house_solve(Path(runs) / f"nights_s{s}") for s in seeds}
    ok = {s: d for s, d in got.items() if d}
    if not ok: raise ValueError("no tuning house's stitch solved a mount; nothing written")
    out = {k: round(float(np.median([d[k] for d in ok.values()])), 3) for k in ("pitch_deg", "yaw_deg", "roll_deg")}
    out.update(from_stitch=True, refused=False, calibration="robot", seeds=sorted(ok), skipped=sorted(set(seeds) - set(ok)),
               made=time.strftime("%Y-%m-%d %H:%M"))
    return out


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("seeds", nargs="+", type=int)
    ap.add_argument("--runs", default=str(HERE / "runs")); ap.add_argument("--out", default=str(HERE.parents[1] / "data" / "robot" / "head_mount.json"))
    a = ap.parse_args(); d = calibrate(a.seeds, a.runs)
    if Path(a.out).exists() and json.loads(Path(a.out).read_text()).get("from_selfcal"):
        raise SystemExit(f"{a.out} was written by the floor self-calibration ([yFg], camgeo.py calibrate): not written over")
    Path(a.out).write_text(json.dumps(d, indent=1)); print(f"robot mount pitch {d['pitch_deg']} yaw {d['yaw_deg']} roll {d['roll_deg']} deg from {d['seeds']} -> {a.out}")


if __name__ == "__main__":
    main()
