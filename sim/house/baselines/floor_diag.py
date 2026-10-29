#!/usr/bin/env python3
"""The floor comparison as the primary finder ([H9w]): why it misses toys and where its false flags come from, and
each fix measured on the baseline nights and confirmed on nights not used to find it.

replay:  the floor arm's confirmer (floor_arm.confirmer_cmd: CONFIRM at a sensitivity, the baselines' views, the stored
         mount and lag, odometry) on one night, with any further confirmer options (--view-noise, --fuse, --dark,
         --parallax, --camera both, --logodds ...; --cache-out saves the comparison, --cache-in replays from it), keeping every decision (confirmed, unsure, dropped and why) and the evidence
         arrays (frames that saw each cell, found it changed, the pooled score, tonight's closest views, the reference).
score:   a directory of replays scored as the floor arm is (table.score_arm): all frames and, on the baseline nights,
         the frontier subset; toys under TINY_CM2 (the 1 x 1 brick) reported apart; per floor kind.
causes:  analysis only (reads the truth): each missed toy with the decisions near it, and each false flag with what lies
         under and around it, sorted into causes (CAUSES).
CONFIRM_NIGHTS: the other toy set's walk-through of each bench house (houses 70, 71, 73, 76, 77), never used to find a
fix; house 72 is still out ([8VL]) and no hold-out is opened.
Usage: floor_diag.py replay <night> <sensitivity> <out dir> [confirm_stream options ...]
       floor_diag.py score <out dir> <sensitivity>
       floor_diag.py causes <out dir> <sensitivity>
(dimos venv python)"""
import collections
import glob
import json
import math
import os
import shutil
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent; HOUSE = HERE.parent
sys.path.insert(0, str(HOUSE)); sys.path.insert(0, str(HOUSE.parents[1] / "src"))

import numpy as np

from baselines import floor_arm as fa, nights as nn, place as pl, prompts as pr, score as sc, table as tb

TINY_CM2 = 2.0           # toys under this footprint (the 1 x 1 brick, 0.8 cm across) reported apart; the tiers stay [oR6]'s
CONFIRM_NIGHTS = ((70, 2, "20261002_045928"), (71, 1, "20261002_004708"), (73, 2, "20261002_025214"),
                  (76, 1, "20261002_033537"), (77, 2, "20261002_044419"))
RES, HX, HY = 0.01, 8.4, 4.9   # the confirmer's grid (detect_changes2)


# the real camera's frame size ([7cU]): tuning nights recorded at 1280 x 720 (data/bench/houses_hr), each house's baselines
# swept again at that size; ids end in _hr. Tuning and confirmation split fixed before any was replayed (5 Oct 2026)
HR_TUNING = ("s70_t1_hr", "s71_t2_hr", "s76_t2_hr", "s96_t1_hr", "s70_t11_hr", "s76_t11_hr")
HR_CONFIRM = ("s70_t2_hr", "s71_t1_hr", "s76_t1_hr", "s96_t2_hr", "s71_t11_hr", "s96_t11_hr")


def nights_hr():
    """the 1280 x 720 nights recorded so far, each {id, seed, db, run_dir, tidy, items, traj}: its tidy run found by
    its recording"""
    b = HOUSE.parents[1] / "data" / "bench" / "houses_hr"; out = []
    if not (b / "nights.json").exists(): return out
    recs = {p.read_text().strip(): p.parent for p in (HOUSE.parents[1] / "data" / "runs" / "dimos_tidy").glob("2026100[5-9]_*/recording.txt")}
    for e in json.load(open(b / "nights.json")):
        if e["seed"] in nn.HOLDOUTS: raise SystemExit(f"house {e['seed']}: a hold-out")
        tidy = recs.get(e["db"])
        if tidy is None or not (b / f"nights_s{e['seed']}").is_dir(): continue
        out.append({"id": f"s{e['seed']}_t{e['toy_set']}_hr", "seed": e["seed"], "db": e["db"], "run_dir": str(b / f"nights_s{e['seed']}"),
                    "tidy": str(tidy), "items": e["items"], "traj": str(tidy / "traj.csv")})
    return out


def nights_all():
    """the six baseline nights ([8VL]), the five confirmation nights, and the 1280 x 720 nights ([7cU])"""
    out = list(nn.night_set())
    for seed, ts, tidy in CONFIRM_NIGHTS:
        if seed in nn.HOLDOUTS: raise SystemExit(f"house {seed}: a hold-out")
        n = nn._bench(seed, ts, tidy); n["traj"] = str(Path(n["tidy"]) / "traj.csv"); out.append(n)
    return out + nights_hr()


def replay(nid, sens, out, extra=()):
    """one night replayed (floor_arm's way), its decisions and evidence kept under out/<night>_<sens>.json/.npz"""
    import confirm_stream as cs
    out = Path(out).resolve(); out.mkdir(parents=True, exist_ok=True); cwd = os.getcwd(); os.chdir(HOUSE)
    n = next(x for x in nights_all() if x["id"] == nid)
    tmp = Path(tempfile.mkdtemp(prefix=f"floordiag_{nid}_")); run = tmp / Path(n["run_dir"]).name
    src = Path(os.environ["HK_FLOORDIAG_RUNS"]) / Path(n["run_dir"]).name if os.environ.get("HK_FLOORDIAG_RUNS") else Path(n["run_dir"])
    try:                                                      # HK_FLOORDIAG_RUNS: baselines rebuilt elsewhere (a calibration
        shutil.copytree(src, run, ignore=shutil.ignore_patterns("mosaic_n3*", "tall_n3*", "detect_n3*", "corrections_*"))   # tried, [4t7])
        a = cs.parser().parse_args(fa.confirmer_cmd(n, run, tmp / "o.json", sens)[2:] + list(extra))
        t0 = time.time(); r = cs.replay(a); cf = r["cf"]
        json.dump({"night": nid, "sens": sens, "extra": list(extra), "baselines": str(src), "precision": r.get("precision", "float32"),
                   "cache_differs": r.get("cache_differs", []),
                   "decisions": r["recs"], "used": r["used"],
                   "wall_s": round(time.time() - t0, 1), "ms_median": float(np.median(r["per_frame"]) * 1000)},
                  open(out / f"{nid}_{sens}.json", "w"), indent=0)
        np.savez_compressed(out / f"{nid}_{sens}.npz", seen=cf.seen, hit=cf.hit, score=cf.score, Vb=cf.Vb, band=cf.band,
                            base=cf.base, rgb=cf.rgb, ref_rgb=cf.ref_rgb, **({"L": cf.L, "Lsum": cf.Lsum} if hasattr(cf, "L") else {}))
        if r.get("cfn") is not None:                          # the near grid's evidence ([H9w]), its decisions marked layer "near"
            cn = r["cfn"]; np.savez_compressed(out / f"{nid}_{sens}.near.npz", seen=cn.seen, hit=cn.hit, score=cn.score, res=cn.res)
    finally:
        shutil.rmtree(tmp, ignore_errors=True); os.chdir(cwd)


def precision(D):
    """[LQs]: the precision a replay's comparison was judged at: its own record, else (replays before 14 Oct 2026) float16
    when it was replayed from a saved comparison (--cache-in), float32 when painted"""
    if D.get("precision"): return D["precision"]
    return "float16" if "--cache-in" in (D.get("extra") or []) else "float32"


def reportable(D, path=""):
    """[LQs]: a replay a reported number may come from: judged as the robot judges live, at its float32 and with the
    options it states. A replay from a float16 cache, or from a cache painted with other options than it states (a
    screen: the cache holds the paint's colour threshold and floor locks), is refused unless HK_CACHE_SCREEN=1
    (HK_ALLOW_F16_CACHE=1, its old name): analysis only, never a reported number"""
    p = precision(D); cached = "--cache-in" in (D.get("extra") or [])
    differs = D.get("cache_differs", ["unknown"]) if cached else []
    if (p != "float32" or differs) and "1" not in (os.environ.get("HK_CACHE_SCREEN"), os.environ.get("HK_ALLOW_F16_CACHE")):
        why = f"a {p} cache" if p != "float32" else f"a cache painted with other options ({', '.join(differs)})"
        raise SystemExit(f"{path or D.get('night')}: replayed from {why}, not as the robot judges live: paint it again "
                         "(or HK_CACHE_SCREEN=1 for analysis only, never a reported number)")
    return D


def arm_of(d, sens):
    """a directory of replays as a floor-arm file (floor_arm.candidates: confirmed and unsure, never floor-sized)"""
    keep = fa.SENSITIVITIES[sens][1]; nights = {}
    for p in sorted(glob.glob(f"{d}/s*_t*_{sens}.json")):
        D = reportable(json.load(open(p)), p); nights[D["night"]] = {"points": fa.candidates(D["decisions"], keep), "frames": D["used"],
                                                      "ms_per_frame_median": D["ms_median"], "wall_s": D["wall_s"]}
    return {"arm": "floor", "sensitivity": sens, "nights": nights}


def context(ids):
    ctx = {}
    for n in nights_all():
        if n["id"] not in ids: continue
        toys, stains = nn.truth_toys(n, pr.named)
        ctx[n["id"]] = {"toys": toys, "stains": stains, "to_true": pl.odom_to_true(n["traj"]), "night": n}
    return ctx


def subset_view():
    F = Path(__file__).resolve().parent / "frontier_run"     # the fixed frame set and its frontier subset ([8VL], [DZr])
    M = json.load(open(F / "frames_manifest.json")); ids = {e["id"] for e in json.load(open(F / "frontier_set.json"))["frames"]}
    v = {}
    for f in M["frames"]:
        if f["id"] in ids: v.setdefault(f["night"], []).append(f)
    return v


def summarise(res):
    """found (all, without the tiny toys, the tiny ones), false positives a night, per floor kind and night"""
    toys = [t for r in res.values() for t in r["toys"]]; main = [t for t in toys if t["area_cm2"] >= TINY_CM2]
    fl = collections.defaultdict(lambda: [0, 0])
    for t in main: fl[t["floor"]][0] += t["found"]; fl[t["floor"]][1] += 1
    fpn = {k: r["fp"] for k, r in res.items()}
    return {"found": sum(t["found"] for t in toys), "n": len(toys),
            "main": [sum(t["found"] for t in main), len(main)],
            "tiny": [sum(t["found"] for t in toys if t["area_cm2"] < TINY_CM2), sum(t["area_cm2"] < TINY_CM2 for t in toys)],
            "fp_per_night": round(sum(fpn.values()) / max(len(fpn), 1), 1), "fp": fpn, "floor": {k: v for k, v in sorted(fl.items())},
            "missed": {k: [t["name"] for t in r["toys"] if not t["found"]] for k, r in res.items()}}


def score(d, sens):
    arm = arm_of(d, sens); ctx = context(set(arm["nights"])); out = {"all": summarise(tb.score_arm(arm, 0.0, ctx))}
    base = {n["id"] for n in nn.night_set()}
    if set(arm["nights"]) <= base: out["subset"] = summarise(tb.score_arm(arm, 0.0, ctx, view=subset_view()))
    return out


CAUSES = ("a toy seen, the flag off it", "dark chequered tile (house 96)", "patterned rug", "small chequered tile (house 71)",
          "furniture edge, a moved movable or its shadow", "wood (glossy hard floor): view seams, highlights", "plain carpet")


def fp_cause(f, nid):
    """a false flag's cause, by rules fixed after looking at contact sheets of every false flag at loose1 (5 Oct 2026):
    near a toy first, then the floor patterns, then structures and movables, then the plain floors"""
    if f["dtoy"] < 0.6: return CAUSES[0]
    if f["floor"] == "rug": return CAUSES[2]
    if f["floor"] == "hard" and nid.startswith("s96"): return CAUSES[1]
    if f["floor"] == "hard" and nid.startswith("s71"): return CAUSES[3]
    if f["dmov"] < 0.3 or f["band25"] > 0.05: return CAUSES[4]
    if f["floor"] == "hard": return CAUSES[5]
    return CAUSES[6]


def causes(d, sens):
    """analysis only: each missed toy's nearest decisions, each false flag's surroundings and cause"""
    arm_ids = [json.load(open(p))["night"] for p in sorted(glob.glob(f"{d}/s*_t*_{sens}.json"))]
    ctx = context(set(arm_ids)); out = {"misses": [], "fps": [], "fp_causes": collections.Counter()}
    keep = fa.SENSITIVITIES[sens][1]
    for nid in arm_ids:
        D = json.load(open(f"{d}/{nid}_{sens}.json")); Z = np.load(f"{d}/{nid}_{sens}.npz"); c = ctx[nid]; n = c["night"]
        n3 = json.load(open(f"{n['run_dir']}/nights_s{n['seed']}_n3.truth.json")); rects = nn.carpet_rects(n["seed"])
        movs = collections.defaultdict(list)
        for k in (1, 2, 3):
            p = Path(n["run_dir"]) / f"nights_s{n['seed']}_n{k}.truth.json"
            if p.exists():
                for m in json.load(open(p)).get("movables", []): movs[m["id"]].append(m)
        moved = [m for v in movs.values() if max(math.hypot(a["x"] - b["x"], a["y"] - b["y"]) for a in v for b in v) > 0.05 for m in v]
        dec = D["decisions"]
        for r in dec: r["tx"], r["ty"] = c["to_true"](r["x_robot"], r["y_robot"], r["ts"])
        cands = sc.merge([dict(x=r["tx"], y=r["ty"], score=fa.SCORE[r["decision"]], rec=r, frame=f"d{i}") for i, r in enumerate(dec)
                          if r["decision"] in keep and r.get("kind") != "floor_change"])
        res = sc.score_night(cands, c["toys"], c["stains"])
        for t in res["toys"]:
            if t["found"]: continue
            near = sorted(dec, key=lambda r: math.hypot(r["tx"] - t["x"], r["ty"] - t["y"]))[:3]
            out["misses"].append({"night": nid, "toy": t["name"], "floor": t["floor"], "area_cm2": t["area_cm2"], "reach": t["reach"],
                                  "near": [{"d_m": round(math.hypot(r["tx"] - t["x"], r["ty"] - t["y"]), 2), "decision": r["decision"],
                                            "why_not": r.get("why_not"), "kind": r.get("kind"), "hits": r["hits"], "seen": r["seen"],
                                            "area_cm2": r["area_cm2"], "slop": r.get("slop")} for r in near]})
        for cnd in res["fp_cands"]:
            r = max((m["rec"] for m in cnd["members"]), key=lambda r: fa.SCORE[r["decision"]])
            i, j = int((HY - r["y"]) / RES), int((r["x"] + HX) / RES)
            f = {"night": nid, "x": round(cnd["x"], 2), "y": round(cnd["y"], 2), "decision": r["decision"], "hits": r["hits"],
                 "seen": r["seen"], "area_cm2": r["area_cm2"], "slop": r.get("slop"),
                 "floor": sc.floor_kind(cnd["x"], cnd["y"], rects, n3.get("rug")),
                 "band25": round(float(Z["band"][max(i - 25, 0):i + 25, max(j - 25, 0):j + 25].mean()), 2),
                 "dtoy": round(min(min(math.hypot(cnd["x"] - x, cnd["y"] - y) for x, y in [(t["x"], t["y"])] + t["also"]) for t in c["toys"]), 2),
                 "dmov": round(min([math.hypot(cnd["x"] - m["x"], cnd["y"] - m["y"]) - max(m["w"], m["d"]) / 2 for m in moved] or [9.0]), 2)}
            f["cause"] = fp_cause(f, nid); out["fps"].append(f); out["fp_causes"][f["cause"]] += 1
    return out


def main(argv=None):
    a = list(sys.argv[1:] if argv is None else argv)
    if a[0] == "replay": replay(a[1], a[2], a[3], a[4:])
    elif a[0] == "score": print(json.dumps(score(a[1], a[2]), indent=1))
    elif a[0] == "causes":
        r = causes(a[1], a[2]); json.dump(r, open(Path(a[1]) / f"causes_{a[2]}.json", "w"), indent=1)
        n = len(glob.glob(f"{a[1]}/s*_t*_{a[2]}.json")) or 1
        for k, v in r["fp_causes"].most_common(): print(f"{v:4d} ({v / n:.1f} a night)  {k}")
        for m in r["misses"]: print("miss", m["night"], m["toy"], m["floor"], m["near"][0] if m["near"] else "")
    else: raise SystemExit(__doc__)


if __name__ == "__main__":
    main()
