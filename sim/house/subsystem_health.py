#!/usr/bin/env python3
"""Every subsystem's health, in the order the robot uses them (sim-slice [63R]): how
successful each one is. Built from a detection bench's own records (the floor
map's frame and shape summaries, the mount calibration, tonight's placement fits, the confirmer's snaps and decisions,
the bench's finds) and, for the stages after the walk-through, a tidy round's funnel (funnel.py's JSON). Sim-only
measures against the truth (path error, floor map error) are marked so: they score, they never steer.
Usage: subsystem_health.py <bench dir> [--funnel funnel.json] [--out health.json] [--html health.html]"""
import argparse, glob, json, math, os
from pathlib import Path

import numpy as np

HK = Path(__file__).resolve().parents[2]
WORKING, WATCH, FAILING, INFO = "working", "watch", "failing", "info"


def path_error_cm(corr, traj):
    """a night's corrected path against the true one, after one rigid fit: median cm (sim only)"""
    import sys
    sys.path.insert(0, str(HK / "src")); from houndkeeper.localize import Localizer
    T = np.genfromtxt(traj, delimiter=",", names=True); z = np.load(corr) if corr else None
    loc = Localizer(times=list(z["times"]), Ts=list(z["Ts"])) if z is not None else None
    A = np.array([(loc.correction(t)[:2, :2] @ [x, y] + loc.correction(t)[:2, 2]) if loc else (x, y)
                  for t, x, y in zip(T["wall_t"], T["odom_x"], T["odom_y"])])
    B = np.c_[T["true_x"], T["true_y"]]; ma, mb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ma).T @ (B - mb)); R = (U @ Vt).T
    if np.linalg.det(R) < 0: Vt[-1] *= -1; R = (U @ Vt).T
    return float(np.median(np.hypot(*((A - ma) @ R.T + mb - B).T)) * 100)


def latest_map_errors():
    """the latest floor map error logged per house ([nv5]), from data/eval/floor_map_error.jsonl"""
    out = {}; f = HK / "data" / "eval" / "floor_map_error.jsonl"
    for line in (open(f) if f.exists() else []):
        r = json.loads(line); out[r["seed"]] = r
    return out


def stage(name, does, measure, value, target, status, per_house=None, sim_only=False):
    return dict(name=name, does=does, measure=measure, value=value, target=target, status=status,
                per_house=per_house or {}, sim_only=sim_only)


def health(bench, funnel=None):
    bench = Path(bench); nights = json.load(open(bench / "nights.json")); seeds = sorted({n["seed"] for n in nights})
    runs = {s: bench / f"nights_s{s}" for s in seeds}; S = []
    # 1. odometry
    od = {s: path_error_cm(None, runs[s] / f"nights_s{s}_n1.traj.csv") for s in seeds if (runs[s] / f"nights_s{s}_n1.traj.csv").exists()}
    S.append(stage("Odometry", "dead reckoning from the legs and IMU", "path error after a rigid fit, median (night 1)",
                   f"{np.median(list(od.values())):.0f} cm" if od else "n/a", None, INFO, {s: f"{v:.0f} cm" for s, v in od.items()}, True))
    # 2. pose graph
    pg = {s: json.load(open(runs[s] / "frame" / "summary.json")) for s in seeds if (runs[s] / "frame" / "summary.json").exists()}
    pge = {s: path_error_cm(str(runs[s] / "frame" / "corrections_1.npz"), str(runs[s] / f"nights_s{s}_n1.traj.csv"))
           for s in seeds if (runs[s] / "frame" / "corrections_1.npz").exists()}
    S.append(stage("Lidar pose graph", "DimOS's loop closures over both clean nights' lidar", "loop closures; path error after",
                   (f"{int(np.median([v['loop_closures'] for v in pg.values()]))} loops, " if pg else "") + (f"{np.median(list(pge.values())):.0f} cm" if pge else "n/a"),
                   None, INFO, {s: (f"{pg[s]['loop_closures']} loops, " if s in pg else "") + f"{v:.0f} cm" for s, v in pge.items()}, True))
    # 3. shape solve
    sh = {s: path_error_cm(str(runs[s] / "shape" / "corrections_1.npz"), str(runs[s] / f"nights_s{s}_n1.traj.csv"))
          for s in seeds if (runs[s] / "shape" / "corrections_1.npz").exists()}
    v = np.median(list(sh.values())) if sh else None
    S.append(stage("Lidar shape solve", "every keyframe pair's lidar match straightens the map", "path error after a rigid fit, median",
                   f"{v:.1f} cm" if v is not None else "n/a", "≤ 3.5 cm (measured 1 Oct)", WORKING if v is not None and v <= 3.5 else WATCH if v is not None else FAILING,
                   {s: f"{x:.1f} cm" for s, x in sh.items()}, True))
    # 4. mount calibration
    mt = {s: json.load(open(runs[s] / "head_mount.json")) for s in seeds if (runs[s] / "head_mount.json").exists()}
    used = {s: not m.get("refused", True) for s, m in mt.items()}
    S.append(stage("Camera mount calibration", "the head camera's true angle, solved from both nights' views", "houses where it locked enough pairs",
                   f"{sum(used.values())} of {len(mt)}", "every house (else the robot's own mount is kept)", WORKING if all(used.values()) else WATCH,
                   {s: f"{m.get('pairs', '?')} pairs" + ("" if used[s] else ", kept robot's") for s, m in mt.items()}))
    # 5. floor picture
    me = latest_map_errors(); fe = {s: me[s]["error"] * 100 for s in seeds if s in me}
    v = np.median(list(fe.values())) if fe else None
    S.append(stage("Floor picture", "the clean floor built from every view, laid on the solved map", "floor map error against the actual floor, median ([nv5])",
                   f"{v:.2f}%" if v is not None else "not measured", "no target yet (tuning mean 1.0%, 1 Oct)", INFO if v is not None else WATCH,
                   {s: f"{x:.2f}%" for s, x in fe.items()}, True))
    # 6. tonight's placement
    fits = {}
    for n in nights:
        c = n.get("corrections")
        if c and os.path.exists(c):
            z = np.load(c)
            if "fits" in z.files and len(z["fits"]) > 1: fits[n["run"]] = float(np.median(z["fits"][1:, 0]) * 1000)
    v = np.median(list(fits.values())) if fits else None
    S.append(stage("Tonight's placement", "each lidar scan matched to the clean nights' map as it arrives", "fit rms, median",
                   f"{v:.0f} mm" if v is not None else "not recorded", "≤ 20 mm", WORKING if v is not None and v <= 20 else WATCH,
                   {r: f"{x:.0f} mm" for r, x in fits.items()}))
    # 7. floor snap
    st = {n["run"]: json.load(open(bench / f"stages_{n['run']}.json")) for n in nights if (bench / f"stages_{n['run']}.json").exists()}
    if st:
        lk, fr = sum(x["snap_locked"] for x in st.values()), sum(x["snap_frames"] for x in st.values()); share = lk / max(fr, 1)
        S.append(stage("Floor snap", "each frame's floor slid onto the clean floor before comparing", "frames locked",
                       f"{lk} of {fr} ({share:.0%})", "most frames (≥ 50%, proposed)", WORKING if share >= 0.5 else FAILING if share < 0.1 else WATCH,
                       {r: f"{x['snap_locked']}/{x['snap_frames']}" for r, x in st.items()}))
    # 8. finding
    det = tot = fal = 0; per = {}
    for n in nights:
        f = bench / f"decisions_{n['run']}.json"
        if not f.exists(): continue
        dec = json.load(open(f)); cand = [d for d in dec if d["decision"] in ("confirmed", "unsure")]
        found = len({tuple(d.get("toy_xy") or (round(d["x"], 1), round(d["y"], 1))) for d in cand if d.get("at_toy")})
        fl = sum(1 for d in cand if not d.get("at_toy")); fal += fl; tot += len(n["items"]); per[n["run"]] = f"{fl} near no toy"
    if (bench / "last_result.json").exists():
        r = json.load(open(bench / "last_result.json")); det, tot = r["detected"], r["toys"]
        S.append(stage("Finding (walk-through)", "the floor compared frame by frame; finds confirmed on approach", "toys a candidate was raised for",
                       f"{det} of {tot} ({det / tot:.0%})", "≥ 90% ([d8d])", WORKING if det / tot >= 0.9 else WATCH if det / tot >= 0.8 else FAILING))
    if nights and per:
        S.append(stage("Stops for nothing", "candidates raised where no toy lies", "a night, mean", f"{fal / len(per):.1f}", "≤ 50 a night ([d8d])",
                       WORKING if fal / len(per) <= 50 else FAILING, per))
    if st:
        c = sum(x["decisions"]["confirmed"] for x in st.values()); u = sum(x["decisions"]["unsure"] for x in st.values())
        S.append(stage("Confirm on approach", "a candidate confirmed by several views, or left unsure for a sidestep", "confirmed of all candidates",
                       f"{c} of {c + u} ({c / max(c + u, 1):.0%})", None, INFO, {r: f"{x['decisions']['confirmed']}c/{x['decisions']['unsure']}u" for r, x in st.items()}))
    # 9+. after the walk-through: a tidy round's funnel
    if funnel and os.path.exists(funnel):
        F = json.load(open(funnel)); stg = F["stages"]; tot_k = [sum(F["funnel"][t][k] for t in F["funnel"]) for k in range(len(stg))]
        names = {"stopped for": ("Stopping for finds", "the robot walks to a find and stops"), "seen close": ("Close look", "the wrist camera sees the find"),
                 "grasp tried": ("Grasp planning", "a reachable grasp is found and tried"), "lifted": ("Grasp", "the jaws lift it"),
                 "in the bin": ("Into the bin", "carried and dropped in the tray")}
        for k in range(1, len(stg)):
            nm, does = names.get(stg[k], (stg[k], ""))
            share = tot_k[k] / max(tot_k[k - 1], 1)
            S.append(stage(nm, does, f"of those reaching it ({F.get('round', '')})", f"{tot_k[k]} of {tot_k[k - 1]} ({share:.0%})", None,
                           WORKING if share >= 0.9 else WATCH if share >= 0.7 else FAILING))
    return {"bench": str(bench), "houses": seeds, "stages": S}


CSS = """body{margin:0;background:#f5f7f6;color:#1c2630;font:15px/1.5 system-ui,sans-serif}.w{max-width:1000px;margin:0 auto;padding:24px 16px}
h1{font-size:26px;margin:0 0 4px}.m{color:#55626e;margin:0 0 20px}table{width:100%;border-collapse:collapse;background:#fff}
th,td{text-align:left;padding:9px 10px;border-top:1px solid #d9e0e3;vertical-align:top}th{font-size:13px;color:#55626e;font-weight:600}
.n{font-variant-numeric:tabular-nums;white-space:nowrap}.c{display:inline-block;padding:1px 8px;border-radius:9px;font-size:12px;font-weight:600}
.working{background:#dcefe3;color:#2d7a4b}.watch{background:#f7ead3;color:#9a6512}.failing{background:#f6dcd6;color:#b3452f}.info{background:#e7ecee;color:#55626e}
.d{color:#55626e;font-size:13px}.ph{color:#55626e;font-size:12px;margin-top:4px;overflow-wrap:anywhere}table{table-layout:fixed}.scroll{overflow-x:auto}
@media (prefers-color-scheme:dark){body{background:#12181d;color:#e5ebef}table{background:#1a2229}th,td{border-color:#2c3740}.m,.d,.ph,th{color:#9aa7b2}}"""


def html(h):
    rows = []
    for i, s in enumerate(h["stages"], 1):
        ph = " · ".join(f"{k}: {v}" for k, v in s["per_house"].items())
        rows.append(f'<tr><td class="n">{i}</td><td><span class="c {s["status"]}">{s["status"]}</span></td>'
                    f'<td><b>{s["name"]}</b><div class="d">{s["does"]}</div></td>'
                    f'<td><b class="n">{s["value"]}</b><div class="d">{s["measure"]}{" · sim only" if s["sim_only"] else ""}</div>'
                    f'{f"<div class=ph>{ph}</div>" if ph else ""}</td><td class="d">{s["target"] or "none set"}</td></tr>')
    return (f'<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
            f'<title>Subsystem health</title><style>{CSS}</style></head><body><div class="w"><h1>Subsystem health</h1>'
            f'<p class="m">Every subsystem the robot uses, in the order it uses them. Bench {os.path.basename(h["bench"])}, houses '
            f'{", ".join(map(str, h["houses"]))}. Stages 11 on come from the latest tidy run\'s funnel.</p><div class="scroll"><table>'
            f'<colgroup><col style="width:32px"><col style="width:84px"><col style="width:30%"><col><col style="width:20%"></colgroup>'
            f'<thead><tr><th>#</th><th>Status</th><th>Subsystem</th><th>Value</th><th>Target</th></tr></thead><tbody>{"".join(rows)}</tbody>'
            f'</table></div></div></body></html>')


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("bench"); ap.add_argument("--funnel"); ap.add_argument("--out"); ap.add_argument("--html")
    a = ap.parse_args(); h = health(a.bench, a.funnel)
    for i, s in enumerate(h["stages"], 1): print(f"{i:2d}. {s['name']:26s} {s['value']:28s} {s['status']}")
    if a.out: json.dump(h, open(a.out, "w"), indent=1)
    if a.html: open(a.html, "w").write(html(h))


if __name__ == "__main__":
    main()
