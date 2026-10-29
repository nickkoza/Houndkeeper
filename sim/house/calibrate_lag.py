#!/usr/bin/env python3
"""The robot's frame lag, calibrated over tuning houses ([gJq], decision [gsJ]): each seed's two baseline recordings
(runs/nights_s<seed>/seed_recordings.txt), each camera's lag estimated from the night's turning
(floor_mosaic2.lag_detail_from_turns), kept only if it passes calibrate.lag_refusal, and the median written to
data/robot/frame_lag.json as the entry for the clock the recordings were stamped on, the other clocks' kept ([55G]). A
recording stamped on another clock than --clock is refused (calibrate.recording_clock). Refuses hold-out seeds before
reading anything. --recording adds a tuning house's other recordings (a walk), each with its house's seed.
Usage: calibrate_lag.py <seed>... --clock wall|robot [--recording SEED DB] [--out data/robot/frame_lag.json]   (dimos venv)"""
import sys as _sys, pathlib as _pl; _sys.path.insert(0, str(_pl.Path(__file__).resolve().parents[2] / "src"))   # houndkeeper
import argparse, json, time
from pathlib import Path

from houndkeeper import calibrate as cal

HERE = Path(__file__).resolve().parent
STREAMS = ("color_image", "arm_color_image")


VALIDATION = {78}   # the validation house: never calibration data either ([gJq], [yFg])


def refuse_holdout(seeds):
    """the seeds a calibration may read: every one a tuning seed, or a ValueError naming the hold-out ones ([gJq])"""
    import make_house
    bad = sorted(int(s) for s in seeds if int(s) in make_house.HOLDOUT | VALIDATION)
    if bad: raise ValueError(f"hold-out seeds {bad} are never calibration data")
    return [int(s) for s in seeds]


def calibrate(seeds, estimate, clock, streams=STREAMS):
    """estimate(seed) -> [(night label, {stream: frame_lag_detail dict})]; returns the frame_lag.json dict"""
    if clock not in cal.CLOCKS: raise ValueError(f"clock must be one of {cal.CLOCKS}")
    seeds = refuse_holdout(seeds)
    nights = [(f"s{s} {lab}", per) for s in seeds for lab, per in estimate(s)]
    out = {"clock": clock, "gate": cal.LAG_GATE, "seeds": seeds, "made": time.strftime("%Y-%m-%d %H:%M"), "streams": {}}
    for st in streams:
        ests = [per[st] for _l, per in nights if st in per]
        lag, why = cal.robot_lag(ests)
        labs = [l for l, per in nights if st in per]
        out["streams"][st] = {"lag_s": lag, "nights": {labs[i]: {"lag_s": ests[i]["lag_s"], "corr": round(ests[i]["corr"], 3),
                              "turning": ests[i]["turning"], "refused": r} for i, r in why}}
    head = out["streams"].get("color_image", {}).get("lag_s")
    if head is None: raise ValueError("no tuning night passed the gate for the head camera; nothing written")
    out["lag_s"] = head
    out["streams"] = {k: v for k, v in out["streams"].items() if v["lag_s"] is not None or v["nights"]}
    for v in out["streams"].values():
        if v["lag_s"] is None: v.pop("lag_s")
    return out


def check_clock(label, db, clock, traj=None):
    """a recording calibrates only its own clock's lag ([55G]): its clock (calibrate.recording_clock) must be `clock`"""
    got = cal.recording_clock(db=db, traj=traj)
    if got != clock: raise ValueError(f"{label} ({db}) was stamped on the {got} clock, not the {clock} clock")


def estimate_from_runs(runs, clock, extra=()):
    """estimate(seed) over each seed's baseline nights (runs/nights_s<seed>/seed_recordings.txt) and the extra
    recordings given for it ([(seed, db)]), each checked to be on `clock` first"""
    def one(label, db, traj=None):
        from dimos.memory.store.sqlite import SqliteStore
        from floor_mosaic2 import lag_detail_from_turns
        check_clock(label, db, clock, traj)
        st = SqliteStore(path=db); st.start()
        per = {s: lag_detail_from_turns(st, s) for s in STREAMS if next(iter(st.stream(s)), None) is not None}
        st.stop()
        print(f"{label}: " + "; ".join(f"{s} {d['lag_s']} corr {d['corr']:.2f} turning {d['turning']}" for s, d in per.items()), flush=True)
        return per
    def est(seed):
        run = Path(runs) / f"nights_s{seed}"; res = []
        lst = run / "seed_recordings.txt"
        for line in (lst.read_text().split("\n") if lst.exists() else []):
            if not line.strip(): continue
            n, db = line.split(maxsplit=1)
            if n not in ("1", "2"): continue
            res.append((f"n{n}", one(f"s{seed} n{n}", db, cal._traj_of(db, [Path(runs)]))))
        for k, (s_, db) in enumerate(e for e in extra if int(e[0]) == seed):
            res.append((f"rec{k + 1}", one(f"s{seed} rec{k + 1}", db)))
        return res
    return est


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("seeds", nargs="+", type=int)
    ap.add_argument("--clock", required=True, choices=cal.CLOCKS, help="the clock the recordings were stamped on ([3Pv]: robot from 3 Oct)")
    ap.add_argument("--recording", nargs=2, action="append", default=[], metavar=("SEED", "DB"),
                    help="another recording of a tuning house (a walk), its clock found from the run that lists it")
    ap.add_argument("--runs", default=str(HERE / "runs")); ap.add_argument("--out", default=str(HERE.parents[1] / "data" / "robot" / "frame_lag.json"))
    ap.add_argument("--dry-run", action="store_true", help="print the entry; write nothing")
    a = ap.parse_args()
    refuse_holdout(list(a.seeds) + [int(s) for s, _ in a.recording])
    seeds = sorted(set(a.seeds) | {int(s) for s, _ in a.recording})
    d = calibrate(seeds, estimate_from_runs(a.runs, a.clock, [(int(s), db) for s, db in a.recording]), a.clock)
    if a.dry_run: print(json.dumps(d, indent=1)); return
    cal.store_lag(a.out, a.clock, d); print(f"robot frame lag {d['lag_s']:.3f} s ({a.clock} clock) -> {a.out}, the other clocks' kept")


if __name__ == "__main__":
    main()
