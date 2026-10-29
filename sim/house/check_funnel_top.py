#!/usr/bin/env python3
"""The top of the funnel, held: fails if the floor map or the walk-through detection has slipped below what was measured
on 27 Sep 2026, less a margin (the findings plus a margin, set as the requirement, so
the top of the funnel cannot degrade silently). The numbers are sim-slice's non-functional requirements ([FHA], [d8d]).
Runs the measurements the requirements name and compares; exit 1 on any miss.
Usage: check_funnel_top.py [--pictures a.npz b.npz] [--bench data/bench/rug2]   (dimos venv; ~10 min)"""
import argparse, re, subprocess, sys
import numpy as np
from pathlib import Path

HERE = Path(__file__).resolve().parent; HK = HERE.parents[1]; PY = sys.executable
# each: the requirement, and (in the comment) what was measured on 27 Sep
MAP_EDGE_MEDIAN_MM = 5.0      # carpet edges from the drawn edge, median (measured 3.3-3.7)
MAP_EDGE_P95_MM = 16.0        # the same, 95th percentile (12-13)
MAP_NIGHTS_AGREE_MM = 8.0     # the two nights' edges agree, median (4.7-6.1)
MAP_RUG_CORR = 0.80           # the rug's two nights correlate (0.88)
DETECT_MIN = 0.93             # toys the walk-through raises a candidate by, on the bench (0.98: 41 of 42)
DETECT_FALSE_MAX = 50.0       # candidates near no toy a night, on the bench (40.5)


def run(args):
    return subprocess.run([PY] + args, cwd=HERE, capture_output=True, text=True).stdout


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pictures", nargs=2, default=[str(HERE / "runs" / "nights_s70" / f"mosaic_n{n}.npz") for n in (1, 2)])
    ap.add_argument("--seed", type=int, default=70); ap.add_argument("--bench", default=str(HK / "data" / "bench" / "rug2"))
    ap.add_argument("--map-error", default=str(HK / "data" / "bench" / "houses6c"), help="[nv5] the bench whose baseline maps are scored against the actual floor ('' to skip)")
    a = ap.parse_args(); misses = []
    def check(name, value, ok, need):
        print(f"{'ok  ' if ok else 'MISS'} {name}: {value} (needs {need})")
        if not ok: misses.append(name)

    out = run(["bench_mapping.py", "--seed", str(a.seed), "--pictures", *a.pictures])
    rug = re.search(r"pictures rug\s+patches\s+\d+\s+corr\s+([-\d.]+)", out)
    check("rug: the two nights' correlation", rug and float(rug.group(1)), bool(rug) and float(rug.group(1)) >= MAP_RUG_CORR, f">= {MAP_RUG_CORR}")
    for edge in ("seam_x", "seam_y"):
        m = re.search(rf"pictures {edge}: \d+ points; nights agree to ([\d.]+) mm median, [\d.]+ mm p95; vs the drawn edge ([\d.]+) mm median, ([\d.]+) mm p95", out)
        if not m: check(f"{edge}: measured", None, False, "an edge seen in both pictures"); continue
        agree, med, p95 = (float(v) for v in m.groups())
        check(f"{edge}: from the drawn edge, median mm", med, med <= MAP_EDGE_MEDIAN_MM, f"<= {MAP_EDGE_MEDIAN_MM}")
        check(f"{edge}: from the drawn edge, p95 mm", p95, p95 <= MAP_EDGE_P95_MM, f"<= {MAP_EDGE_P95_MM}")
        check(f"{edge}: nights agree, median mm", agree, agree <= MAP_NIGHTS_AGREE_MM, f"<= {MAP_NIGHTS_AGREE_MM}")

    if a.map_error:                                          # [nv5] the floor map against the actual floor: above detection,
        import json as _json                                 # its ceiling (detection cannot be better
        bench = Path(a.map_error); errs = []                 # than the map); measured and logged, no requirement set yet
        for run_ in sorted(bench.glob("nights_s*")):
            seed = int(run_.name.split("_s")[-1]); pic = run_ / "mosaic_n1.npz"; traj = run_ / f"nights_s{seed}_n1.traj.csv"
            if not (pic.exists() and traj.exists()): continue
            o = run(["floor_fidelity.py", str(pic), "--seed", str(seed), "--traj", str(traj), "--error", str(run_), "--built", f"bench {bench.name}"])
            m_ = re.search(r"floor map error ([\d.]+)%", o)
            if m_: errs.append((seed, float(m_.group(1))))
        print("floor map error (above detection, its ceiling): " + (f"{np.mean([e for _s, e in errs]):.2f}% mean; " + ", ".join(f"house {s_} {e:.2f}%" for s_, e in errs) if errs else "not measured"))

    out = run(["bench_detect.py", "--bench", a.bench, "--every", "8"])
    m = re.search(r"walk-through detected (\d+)/(\d+) \(\d+%\).*?candidates near no toy ([\d.]+) a night", out)
    if not m: check("walk-through detection: measured", None, False, "a bench result")
    else:
        det, tot, false = int(m.group(1)), int(m.group(2)), float(m.group(3))
        check("walk-through detection: share of toys", f"{det}/{tot} = {det / tot:.0%}", det / tot >= DETECT_MIN, f">= {DETECT_MIN:.0%}")
        check("walk-through detection: candidates near no toy a night", false, false <= DETECT_FALSE_MAX, f"<= {DETECT_FALSE_MAX}")
    print("top of funnel:", "held" if not misses else f"SLIPPED ({len(misses)}: {', '.join(misses)})")
    sys.exit(1 if misses else 0)


if __name__ == "__main__":
    main()
