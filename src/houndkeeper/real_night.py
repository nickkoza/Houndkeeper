#!/usr/bin/env python3
"""A find-and-flag night on the dog alone ([jFy]): what real/night.sh asks before the robot walks and where the night is
kept. Reads only the house's own files (real/local/, the house's run directory built from its baseline recordings, the
recordings catalog): never a sim run, a truth file or the sim's model.

  preflight  why the night must not start ([qDh]): the stops not known ([U4p]), the robot network's isolation check
             stale or failed or too little disk ([6xc], [VVJ]), the house's frame and floor map incomplete or refused
             by its gate, no operating area ([JVr], [Qd5]). The sim robot standing in for the dog skips the dog's own
             checks (the stops, the robot network) and says so.
  tonight    tonight's directory ([oWO], [q66]): under the real nights' root, named by the date its evening began.

Without the arm every step that needs it is skipped: the live tidy walks its rounds watching only (tidy_module,
HK_ARM=none) and its report says which steps it skipped (tidy_module.ARM_STEPS), which the morning log repeats.
Usage: real_night.py preflight [--house DIR] [--robot real|sim] [--area FILE] [--stops FILE]
       real_night.py tonight                                     (dimos venv python)"""
from __future__ import annotations

import argparse, datetime as dt, json, os, sys, time
from pathlib import Path

HK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(HK / "src"))
from houndkeeper import recordings   # noqa: E402

HOUSE = HK / "data" / "house" / "real"             # the house's run directory ([qDh]), unless HK_HOUSE names another
STOPS = HK / "real" / "local" / "stops.json"       # the stops as tested (first-boot.md §3), gitignored
STOP_KEYS = {"without_dimos": ("lying", "standing"), "through_dimos": ("lying", "standing")}   # [U4p]: two stops, each
                                                   # tested lying down, then standing
HOUSE_FILES = ("mosaic_n1.npz", "mosaic_n2.npz", "views_n1.npz", "views_n2.npz", "views_near_n1.npz",
               "views_near_n2.npz", "head_mount.json", "stitch.args", "shape/gate.json")   # what the walk and tonight's
               # comparison at the launch setting read from the house (real_nights.sh builds them)
SIM_TRUTH_FILES = ("*.truth.json", "*.traj.csv")  # the sim's truth and true path: a real house has none


def _date(v) -> bool:
    try: dt.date.fromisoformat(str(v)[:10]); return True
    except ValueError: return False


def stops_problems(path=STOPS) -> list[str]:
    """[U4p]: why the stops are not known: each stop tested lying down and standing, the dead-man passed, each dated"""
    p = Path(path)
    if not p.exists(): return [f"the stops are not on record ({p}: first-boot.md step 3, each stop tested lying down and standing)"]
    try: d = json.loads(p.read_text())
    except ValueError as e: return [f"the stops' record is unreadable ({p}): {e}"]
    out = []
    for stop, poses in STOP_KEYS.items():
        for pose in poses:
            if not _date((d.get(stop) or {}).get(pose, "")):
                out.append(f"the stop {stop.replace('_', ' ')} has no date tested {pose} ({p})")
    if not _date(d.get("deadman", "")): out.append(f"the cockpit's dead-man has no date passed ({p}: real/tests/deadman_sim.sh, then on the robot)")
    return out


def baselines(house) -> dict[int, Path]:
    """the house's baseline recordings, {night: memory.db}, from its recordings.txt ("<night> <db>" lines)"""
    p = Path(house) / "recordings.txt"; out = {}
    for line in (p.read_text().splitlines() if p.exists() else []):
        n, _, db = line.partition(" ")
        if n.isdigit() and db.strip(): out[int(n)] = Path(db.strip())
    return out


def house_problems(house) -> list[str]:
    """[qDh]: why the house's frame and floor map are not in order for tonight"""
    h = Path(house)
    if not h.is_dir():
        return [f"no house frame or floor map at {h} (build it from the two baseline recordings: sim/house/real_nights.sh {h} <night 1> <night 2>)"]
    out = []
    sim = sorted(q.name for g in SIM_TRUTH_FILES for q in h.glob(g))
    if sim: out.append(f"{h} holds sim truth ({', '.join(sim[:3])}): not a real house")
    b = baselines(h)
    for n in (1, 2):
        if n not in b: out.append(f"the house names no baseline night {n} ({h / 'recordings.txt'})")
        elif not b[n].exists(): out.append(f"baseline night {n}'s recording is gone: {b[n]}")
    missing = [f for f in HOUSE_FILES if not (h / f).exists()]
    if missing: out.append(f"the house's floor map is incomplete: no {', '.join(missing)} (run sim/house/real_nights.sh on it)")
    g = h / "shape" / "gate.json"
    if g.exists():
        try:
            nights = json.loads(g.read_text()).get("nights") or {}
            bad = [f"night {k}: {'; '.join(v.get('refused') or [])}" for k, v in sorted(nights.items()) if v.get("refused")]
            if bad: out.append("the house's frame was refused by its gate (" + "; ".join(bad) + ")")
            elif sorted(nights) != ["1", "2"]: out.append(f"the house frame's gate judged nights {sorted(nights)}, not 1 and 2")
        except ValueError as e: out.append(f"the house frame's gate is unreadable ({g}): {e}")
    return out


def preflight(house=None, robot="real", area=None, stops=None, session=None, now=None) -> tuple[list[str], list[str]]:
    """([qDh]) (why the night must not start, notes): every check run, so the user sees all that is missing at once.
    session(real, now) -> [why]: the recorded session's own preflight (recordings.preflight)"""
    house = Path(house or os.environ.get("HK_HOUSE") or HOUSE); session = session or recordings.preflight
    real = robot == "real"; why, notes = [], []
    if real: why += stops_problems(stops or STOPS)
    else: notes.append("the sim robot stands in for the dog: the dog's stops and the robot network are not checked")
    why += session(real, now)
    why += house_problems(house)
    if area:
        if not Path(area).exists(): why.append(f"no operating area at {area}")
    elif real: why.append("no operating area (draw it on the cockpit's Area page and give its file as AREA)")
    else: notes.append("no operating area: the sim robot walks without one")
    return why, notes


NIGHTS = HK / "data" / "real_nights"             # [oWO]: the real nights' root, unless HK_REAL_NIGHTS names another
EVENING_STARTS_H = 6                               # [oWO]: a night is named by the date its evening began; before 6 am
                                                   # is the evening before


def nights_root() -> Path:
    return Path(os.environ.get("HK_REAL_NIGHTS") or NIGHTS)


def night_name(now=None) -> str:
    """[oWO]: the night's directory name, the date its evening began (YYYYmmdd)"""
    now = now or dt.datetime.now()
    return (now - dt.timedelta(days=1) if now.hour < EVENING_STARTS_H else now).strftime("%Y%m%d")


def tonight_dir(now=None) -> Path:
    """[q66]: tonight's directory under the real nights' root: the list, and night/ with the log beside it"""
    return nights_root() / night_name(now)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0]); sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("preflight"); s.add_argument("--house"); s.add_argument("--robot", choices=["real", "sim"], default="real")
    s.add_argument("--area"); s.add_argument("--stops")
    sub.add_parser("tonight")
    a = ap.parse_args(argv)
    if a.cmd == "tonight": print(tonight_dir()); return 0
    why, notes = preflight(a.house, a.robot, a.area, a.stops)
    for n in notes: print(f"note: {n}")
    if why:
        print("not starting the night:"); print("\n".join(f"  {w}" for w in why)); return 1
    print("preflight passed"); return 0


if __name__ == "__main__":
    sys.exit(main())
