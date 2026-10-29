#!/usr/bin/env python3
"""Arm 1, the floor comparison ([WD9], [d8d]): the confirmer replayed over each night's walk-through as the live tidy
sets it up (tidy_module._sweep_and_pick): confirm_stream's CONFIRM settings, the baselines' own views, the head camera's
calibrated mount, the robot's stored frame lag on the recording's clock, frames placed by odometry. Its candidates (spots
confirmed or left unsure: both get looked at again) go to the shared scorer as points in the odometry's frame, where
the robot would walk to ([o0E]). Each night replays from a copy of its baselines without any picture of tonight
(a stale mosaic_n3 would be read as mapped structure). Records the time per frame and per night (CPU).
--sensitivity loosens the confirmer ([3hK]: the floor comparison as the recall engine, a local recognizer behind it
restoring precision): SENSITIVITIES, fixed 4 Oct 2026 before any loosened result was seen. "today" is CONFIRM as the
live tidy runs it; each looser one lowers the evidence a spot needs (hits, ratio, span), marks spots unsure from further
off, and the loosest also lower the change thresholds (tc, tb), switch the slop check off and keep the spots the
confirmer would drop as not a find (loose2: those the slop check drops; never floor-sized changes).
--nights replays only some nights; with --merge they replace those nights in an existing <out.json> (a night re-run
after a fix, the others standing), the re-run recorded under "reruns".
Usage: floor_arm.py <out.json> [--jobs 6] [--sensitivity today|loose1|loose2|loose3] [--nights s96_t1 ... [--merge]]
(dimos venv python)"""
import argparse
import json
import re
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent; HOUSE = HERE.parent
sys.path.insert(0, str(HOUSE))


SENSITIVITIES = {         # name -> (confirm_stream options over CONFIRM, the decisions kept as candidates)
    "today": ({}, ("confirmed", "unsure")),
    "loose1": ({"hits": 3, "ratio": 0.5, "span": 0.6, "unsure-at": 1.6, "slop-frac": 0.2}, ("confirmed", "unsure")),
    "loose2": ({"hits": 3, "ratio": 0.45, "span": 0.5, "unsure-at": 2.0, "tc": 0.038, "tb": 0.8},
               ("confirmed", "unsure", "not_a_find")),
    "loose3": ({"hits": 2, "ratio": 0.4, "span": 0.3, "unsure-at": 2.5, "tc": 0.032, "tb": 0.6, "slop": 0, "small-px": 4,
                "small-peak": 0.4}, ("confirmed", "unsure", "not_a_find")),
}
SCORE = {"confirmed": 1.0, "unsure": 0.5, "not_a_find": 0.25}   # table.py's "confirmed only" keeps score 1.0


def confirmer_cmd(n, run, out, sensitivity="today"):
    """the replay's command line for one night, its baselines copied to `run`, at a sensitivity (SENSITIVITIES)"""
    cmd = [sys.executable, str(HOUSE / "confirm_stream.py"), "--run-dir", str(run), "--seed", str(n["seed"]), "--night", "3",
           "--db", n["db"], "--traj", n["traj"], "--out", str(out)]
    if (run / "head_mount.json").exists(): cmd += ["--mount", str(run / "head_mount.json")]
    views = sorted(str(p) for p in run.glob("views_n[12].npz"))
    if views: cmd += ["--views", *views]
    for k, v in SENSITIVITIES[sensitivity][0].items(): cmd += [f"--{k}", str(v)]
    return cmd


def candidates(decisions, keep=("confirmed", "unsure")):
    """the replay's decisions that get a second look (by default confirmed and unsure; floor-sized changes never), as
    scorer points in the odometry's frame"""
    return [{"x": d["x_robot"], "y": d["y_robot"], "ts": d["ts"], "score": SCORE[d["decision"]],
             "label": d["decision"], "kind": d.get("kind", ""), "hits": d.get("hits"), "frame": f"decision{i}"}
            for i, d in enumerate(decisions) if d["decision"] in keep and d.get("kind") != "floor_change"]


def run_night(n, logdir, sensitivity="today"):
    tmp = Path(tempfile.mkdtemp(prefix=f"floorarm_{n['id']}_")); run = tmp / Path(n["run_dir"]).name
    try:
        shutil.copytree(n["run_dir"], run, ignore=shutil.ignore_patterns("mosaic_n3*", "tall_n3*", "detect_n3*", "corrections_*"))
        out = tmp / "out.json"; t0 = time.time()
        r = subprocess.run(confirmer_cmd(n, run, out, sensitivity), capture_output=True, text=True, cwd=HOUSE)
        wall = time.time() - t0
        (Path(logdir) / f"floor_{sensitivity}_{n['id']}.log").write_text(r.stdout + "\n" + r.stderr)
        if r.returncode != 0 or not out.exists():
            raise SystemExit(f"{n['id']}: confirm_stream failed (exit {r.returncode}): " + "\n".join(r.stderr.strip().splitlines()[-6:]))
        res = json.load(open(out))
        m = re.search(r"frames (\d+), (\d+) ms each", r.stdout)
        return {"points": candidates(res["decisions"], SENSITIVITIES[sensitivity][1]), "decisions": len(res["decisions"]), "wall_s": round(wall, 1),
                "frames": int(m.group(1)) if m else None, "ms_per_frame_median": int(m.group(2)) if m else None,
                "lag_line": next((l for l in r.stdout.splitlines() if "frame lag" in l), "")}
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main(argv=None):
    from baselines import nights as nn
    ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--sensitivity", default="today", choices=list(SENSITIVITIES))
    ap.add_argument("--nights", nargs="*", default=None); ap.add_argument("--merge", action="store_true")
    a = ap.parse_args(argv); ns = [n for n in nn.night_set() if not a.nights or n["id"] in a.nights]; logdir = Path(a.out).parent
    if a.nights and len(ns) != len(a.nights): raise SystemExit(f"unknown night among {a.nights}")
    old = json.load(open(a.out)) if a.merge else None
    if old is not None and old.get("sensitivity", "today") != a.sensitivity: raise SystemExit(f"{a.out} is at another sensitivity")
    with ThreadPoolExecutor(a.jobs) as ex:
        res = dict(zip([n["id"] for n in ns], ex.map(lambda n: run_night(n, logdir, a.sensitivity), ns)))
    over, keep = SENSITIVITIES[a.sensitivity]
    if old is not None:
        old["nights"].update(res)
        old.setdefault("reruns", []).append({"nights": sorted(res), "when": time.strftime("%Y-%m-%dT%H:%M:%S%z")})
        json.dump(old, open(a.out, "w"), indent=1)
    else:
        json.dump({"arm": "floor", "sensitivity": a.sensitivity, "options": over, "keep": list(keep),
                   "setup": "live tidy: CONFIRM, views, mount, robot lag, odometry", "nights": res}, open(a.out, "w"), indent=1)
    for k, v in res.items(): print(k, len(v["points"]), "candidates;", v["frames"], "frames,", v["ms_per_frame_median"], "ms each;", v["wall_s"], "s")


if __name__ == "__main__":
    main()
