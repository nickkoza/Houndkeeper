#!/usr/bin/env python3
"""Arms compared by their curves, not one operating point ([HGT]): each arm with a score swept over its threshold
through the shared placement and scorer (table.score_arm), as a precision-recall curve and as recall against false
positives a night (the free-response form, FROC), with average precision (AP) as the summary and the operating point we
would run marked. An arm with no score (the frontier model's answers) is one point; [80w]'s target is a point too.

Recall is toys found over toys; precision is merged candidates near a toy over all merged candidates; false positives
a night are merged candidates near no toy, over the nights. AP is the all-points interpolated area under the
precision-recall curve (each recall step weighted by the best precision at that recall or more), recall from 0.

floor_arm_scored: a directory of floor_diag replays as an arm whose points carry a continuous score read from the
night's final evidence at each candidate's cells (the log-odds `L`, or the vote's mean change `score / seen`).
Usage: curves.py <out.png> <name>=<arm.json or replay dir:key> ... [--op name=found/n,fp] [--view subset|all]
(dimos venv python)"""
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import numpy as np

from baselines import table as tb

# the reference palette's categorical slots, in fixed order (dataviz, light mode); text stays in ink colours
SERIES = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300")
INK, MUTED, GRID = "#1a1a19", "#5f5e58", "#e4e3dc"
MAX_STEPS = 120          # a curve of more distinct scores is swept at this many, spread evenly over their ranks


def sweep(arm, ctx, view=None, thresholds=None):
    """the arm scored at every threshold that changes it (its points' distinct scores, high to low): a list of
    {thr, found, n, fp, cands, nights}"""
    if thresholds is None:
        sc_ = sorted({p["score"] for ps in tb.placed_points(arm, -1e9).values() for p in ps}, reverse=True)
        thresholds = sc_ if len(sc_) <= MAX_STEPS else [sc_[int(i)] for i in np.unique(np.linspace(0, len(sc_) - 1, MAX_STEPS).round())]
    out = []
    for t in thresholds:
        res = tb.score_arm(arm, t, ctx, view=view)
        out.append({"thr": float(t), "found": sum(r["found"] for r in res.values()), "n": sum(r["n"] for r in res.values()),
                    "fp": sum(r["fp"] for r in res.values()), "cands": sum(r["cands"] for r in res.values()), "nights": len(res)})
    return out


def pr_points(curve):
    """(recall, precision, false positives a night) per threshold"""
    return [(c["found"] / max(c["n"], 1), (c["cands"] - c["fp"]) / c["cands"] if c["cands"] else 1.0, c["fp"] / max(c["nights"], 1))
            for c in curve]


def average_precision(curve):
    """all-points interpolated AP: sum over recall steps of the best precision at that recall or beyond"""
    pts = sorted(pr_points(curve), key=lambda p: p[0])
    if not pts: return 0.0
    rec = [0.0] + [p[0] for p in pts]; prec = [p[1] for p in pts]
    best = [max(prec[i:]) for i in range(len(prec))]
    return float(sum((rec[i + 1] - rec[i]) * best[i] for i in range(len(pts))))


def floor_arm_scored(d, sens, key="L", keep=("confirmed", "unsure")):
    """floor_diag replays in `d` as an arm, each candidate scored by the night's final evidence at its cells: key "L",
    the log-odds' highest (clamped, as decided); "Lsum", its unclamped sum (a ranking without ties at the clamp); "mean",
    the vote's highest mean change (score / seen, 1 at the threshold)"""
    import glob
    RES, HX, HY = 0.01, 8.4, 4.9
    nights = {}
    for p in sorted(glob.glob(f"{d}/s*_t*_{sens}.json")):
        from baselines import floor_diag as _fd
        D = _fd.reportable(json.load(open(p)), p); Z = np.load(p[:-5] + ".npz")   # [LQs]: never from a float16 cache
        E = Z[key] if key in ("L", "Lsum") else Z["score"] / np.maximum(Z["seen"], 1)
        En, Rn = None, RES                                      # the near grid's (its decisions are marked layer "near")
        if Path(p[:-5] + ".near.npz").exists():
            Zn = np.load(p[:-5] + ".near.npz"); En, Rn = Zn["score"] / np.maximum(Zn["seen"], 1), float(Zn["res"])
        pts = []
        for i, r in enumerate(D["decisions"]):
            if r["decision"] not in keep or r.get("kind") == "floor_change": continue
            E_, R_ = (En, Rn) if r.get("layer") == "near" and En is not None else (E, RES)
            v = [float(E_[int((HY - y) / R_), int((x + HX) / R_)]) for x, y in r.get("cells", [])
                 if 0 <= int((HY - y) / R_) < E_.shape[0] and 0 <= int((x + HX) / R_) < E_.shape[1]]
            pts.append({"x": r["x_robot"], "y": r["y_robot"], "ts": r["ts"], "score": round(max(v), 4) if v else -1e9,
                        "label": r["decision"], "frame": f"decision{i}"})
        nights[D["night"]] = {"points": pts, "frames": D["used"]}
    return {"arm": "floor", "sensitivity": sens, "nights": nights}


def plot(path, curves, points=(), target=None, title=""):
    """two panels, stacked for a phone's width: precision against recall, and recall against false positives a night
    (FROC). curves: [(name, curve, op)] with op the operating point's (recall, precision, fp a night) or None;
    points: [(name, (recall, precision, fp a night))] single points (an arm with no score); target: (recall, fp a
    night) [80w]'s corner, recall at least and false positives at most"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 13, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "text.color": INK})
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(6.4, 10.5), dpi=150)
    for ax in (a1, a2):
        ax.grid(True, color=GRID, lw=0.8); ax.set_axisbelow(True)
        for s in ("top", "right"): ax.spines[s].set_visible(False)
    fpmax = 1.0
    for k, (name, cv, op) in enumerate(curves):
        col = SERIES[k % len(SERIES)]; P = sorted(pr_points(cv), key=lambda p: p[2])
        if not P: continue
        R, Pr, F = zip(*P); fpmax = max(fpmax, max(F))
        lab = f"{name} (AP {average_precision(cv):.2f})"
        a1.plot(R, Pr, "-", color=col, lw=2, label=lab)
        a2.plot(F, R, "-", color=col, lw=2, label=lab)
        if op is not None:
            a1.plot(op[0], op[1], "o", ms=10, mfc=col, mec="white", mew=2, zorder=5)
            a2.plot(op[2], op[0], "o", ms=10, mfc=col, mec="white", mew=2, zorder=5)
    for name, (r, p, f) in points:
        a1.plot(r, p, "D", ms=9, mfc=INK, mec="white", mew=2, zorder=6, label=name)
        a2.plot(f, r, "D", ms=9, mfc=INK, mec="white", mew=2, zorder=6, label=name)
    if target is not None:                                      # the corner: recall at least, false positives at most
        tr, tf = target
        a2.fill_between([0, tf], [tr, tr], [1.02, 1.02], color=SERIES[2], alpha=0.12, lw=0, label="[80w] target")
        a2.plot([tf, tf], [tr, 1.02], "--", color=MUTED, lw=1); a2.plot([0, tf], [tr, tr], "--", color=MUTED, lw=1)
    a1.set_xlabel("recall (toys found)"); a1.set_ylabel("precision"); a1.set_xlim(0, 1.02); a1.set_ylim(0, 1.02)
    a2.set_xlabel("false positives a night"); a2.set_ylabel("recall (toys found)"); a2.set_ylim(0, 1.02)
    a2.set_xlim(0, min(fpmax, 12) * 1.05)
    a1.set_title(title or "precision against recall", loc="left", fontsize=14)
    a2.set_title("recall against false positives a night (FROC)", loc="left", fontsize=14)
    a2.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=1, frameon=False, fontsize=12)
    fig.tight_layout(); fig.savefig(path, facecolor="white"); plt.close(fig)


def main(argv=None):
    a = list(sys.argv[1:] if argv is None else argv)
    out, view_kind, ops, arms = a[0], "all", {}, []
    i = 1
    while i < len(a):
        if a[i] == "--view": view_kind = a[i + 1]; i += 2; continue
        if a[i] == "--op":
            k, v = a[i + 1].split("="); f, fp = v.split(","); ops[k] = f; ops[k + "#fp"] = float(fp); i += 2; continue
        arms.append(a[i].split("=", 1)); i += 1
    from baselines import floor_diag as fd
    loaded = []
    for name, src in arms:
        if ":" in src and Path(src.split(":")[0]).is_dir():
            d, key = src.split(":"); arm = floor_arm_scored(d, "loose1", key)
        else: arm = json.load(open(src))
        loaded.append((name, arm))
    ids = set().union(*[set(arm["nights"]) if "nights" in arm else {f["frame"]["night"] for f in arm["frames"]} for _, arm in loaded])
    ctx = fd.context(ids); view = fd.subset_view() if view_kind == "subset" else None
    res = []
    for name, arm in loaded:
        cv = sweep(arm, ctx, view)
        res.append({"name": name, "ap": round(average_precision(cv), 3), "curve": cv})
        print(f"{name}: AP {average_precision(cv):.3f}, {len(cv)} thresholds")
    json.dump(res, open(Path(out).with_suffix(".json"), "w"), indent=1)
    plot(out, [(r["name"], r["curve"], None) for r in res])


if __name__ == "__main__":
    main()
