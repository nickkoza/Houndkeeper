"""How fast a sim run ran ([5SS]): its sim/wall ratio and frame interval, read from the sim's own SIM_SPEED lines (the
last one is the whole run's) or, failing those, from its path file (traj.csv: sim_t against host_t, or before the
robot's clock [3Pv] against wall_t). A run under FLATTER_BELOW is marked latency-flattered wherever it is summarised:
until [3Pv] is measured (one night at two speeds), its delays may have cost less robot motion than they will."""
import csv
import re
from pathlib import Path

FLATTER_BELOW = 0.9          # [5SS]: sim/wall under this marks a run's numbers, until [3Pv] is measured
FRAME_SIM_S = 0.07           # before the sim reported frames: one every two 35 ms loops (the first loop past 50 ms of sim time)
_LINE = re.compile(r"SIM_SPEED sim/wall = ([0-9.]+)[^;]*;(?: frames every ([0-9.]+) ms wall, ([0-9.]+) ms robot;)?")


def from_log(path) -> dict | None:
    """the last SIM_SPEED line of a DimOS log: {sim_wall, frame_wall_ms, frame_sim_ms} (frames None before 3 Oct)"""
    try:
        text = Path(path).read_text(errors="replace")
    except OSError:
        return None
    m = None
    for m in _LINE.finditer(text): pass
    if m is None: return None
    return {"sim_wall": float(m.group(1)), "frame_wall_ms": float(m.group(2)) if m.group(2) else None,
            "frame_sim_ms": float(m.group(3)) if m.group(3) else None}


def from_traj(path) -> float | None:
    """sim/wall over a path file: sim_t against host_t (the host's clock, since [3Pv]) or wall_t (the host's, before)"""
    try:
        with open(path) as fh: rows = list(csv.DictReader(fh))
    except OSError:
        return None
    if len(rows) < 2: return None
    col = "host_t" if "host_t" in rows[0] else "wall_t"
    try:
        dw = float(rows[-1][col]) - float(rows[0][col]); ds = float(rows[-1]["sim_t"]) - float(rows[0]["sim_t"])
    except (KeyError, ValueError):
        return None
    return ds / dw if dw > 0 else None


def speed(log=None, traj=None) -> dict:
    """a run's speed: {sim_wall, frame_interval_ms {wall, sim, derived}, latency_flattered, source}; None where unknown"""
    lg = from_log(log) if log else None
    r = lg["sim_wall"] if lg else (from_traj(traj) if traj else None)
    fi = None
    if lg and lg["frame_wall_ms"] is not None:
        fi = {"wall": lg["frame_wall_ms"], "sim": lg["frame_sim_ms"], "derived": False}
    elif r:
        fi = {"wall": round(1e3 * FRAME_SIM_S / r, 1), "sim": 1e3 * FRAME_SIM_S, "derived": True}
    return {"sim_wall": r, "frame_interval_ms": fi, "latency_flattered": None if r is None else r < FLATTER_BELOW,
            "source": "log" if lg else ("traj" if r is not None else None)}


def of_run(out) -> dict:
    """a DimOS tidy run's folder (tidy_run.sh: dimos.log, traj.csv)"""
    out = Path(out)
    return speed(out / "dimos.log", out / "traj.csv")


def combine(speeds) -> dict:
    """many runs' speeds: how many, how many flattered or unknown, the slowest"""
    rs = [s.get("sim_wall") for s in speeds if s]
    known = [r for r in rs if r is not None]
    return {"nights": len(speeds), "flattered": sum(r < FLATTER_BELOW for r in known), "unknown": len(speeds) - len(known),
            "sim_wall_min": min(known) if known else None}


def of_nights(run_dir, nights) -> dict:
    """a sweep run's nights (sweep_night.sh: <room>_n<k>.sweep.log, <room>_n<k>.traj.csv), combined"""
    run_dir = Path(run_dir); out = []
    for n in nights:
        log = next(iter(sorted(run_dir.glob(f"*_n{n}.sweep.log"))), None)
        traj = next(iter(sorted(run_dir.glob(f"*_n{n}.traj.csv"))), None)
        out.append(speed(log, traj))
    return combine(out)


def flag(sp) -> str:
    """the mark one run's numbers carry: '' when it ran at real time"""
    if not sp or sp.get("sim_wall") is None: return " [speed not recorded]"
    return f" [latency-flattered: sim/wall {sp['sim_wall']:.2f}]" if sp.get("latency_flattered") else ""


def flag_many(c) -> str:
    """the mark a summary of many runs carries"""
    if not c or not c["nights"]: return ""
    bits = []
    if c["flattered"]: bits.append(f"{c['flattered']} of {c['nights']} nights latency-flattered (sim/wall from {c['sim_wall_min']:.2f}, under {FLATTER_BELOW})")
    if c["unknown"]: bits.append(f"{c['unknown']} of {c['nights']} nights' speed not recorded")
    return " [" + "; ".join(bits) + "]" if bits else ""


if __name__ == "__main__":   # run_speed.py <dimos or sweep log> [traj.csv]: one line for a run's notes
    import json, sys
    sp = speed(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
    print(json.dumps(sp) + flag(sp))
