#!/usr/bin/env python3
"""Detection bench ([d8d], [QDc]; the slice's walk-through target): replay the confirmer over every recorded tuning night
at once, with any settings, and report how many toys the walk-through raised a candidate for. A candidate is a spot
confirmed or left unsure (both get looked at again: sidestep, close look); a toy is detected if one lies within NEAR_M of
it. Nights and baselines are fixed (a snapshot: data/bench/<name>/nights.json and its runs), so settings compare fairly.
Usage: bench_detect.py [--bench data/bench/detect_cal7] [confirm_stream options, e.g. --tb 0.6 --ratio 0.5]
(dimos venv python)"""
import json, math, os, shutil, subprocess, sys, tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent; HK = HERE.parents[1]
NEAR_M = 0.30                 # a candidate this close to a toy is on it, or within 15 cm beyond a long toy's half-length
                              # (the scan's origin can sit at one end of a 29 cm dinosaur); toys lie 60 cm apart or more


PLACEHOLDERS = ("@traj", "@corrections", "@mount")      # each night's own file, from its nights.json entry


def confirmer_args(n, extra):
    """the confirmer's options with each night's own files put in: --truth @traj, --corrections @corrections, --mount
    @mount, --views @views. A night without the key stops the bench ([SdA]): passed on literally, the confirmer crashed
    on it and the night scored as no finds"""
    out = []
    for x in extra:
        if x in PLACEHOLDERS or x == "@views":
            if x[1:] not in n: raise SystemExit(f"bench night {n.get('run', '?')}: {x} asked for, but its nights.json entry has no '{x[1:]}'")
            v = n[x[1:]]; out += list(v) if x == "@views" else [v]
        else: out.append(x)
    return out


def replay(cmd, out, err, cwd=HERE):
    """one confirmer run; its result (out.json), or the bench stops with its stderr kept in err ([SdA]): a crash is not a
    quiet night"""
    r = subprocess.run(cmd, capture_output=True, text=True, cwd=cwd)
    Path(err).write_text(r.stderr)
    if r.returncode != 0 or not Path(out).exists():
        raise SystemExit(f"confirm_stream failed (exit {r.returncode}{', no result' if r.returncode == 0 else ''}); its stderr: {err}\n"
                         + "\n".join(r.stderr.strip().splitlines()[-6:]))
    return json.load(open(out))


def night(bench, k, n, extra):
    """one recorded night replayed: (toys detected, toys confirmed, candidates near no toy, toys)"""
    sys.path.insert(0, str(HK / "sim" / "arm")); import items as _items
    tmp = Path(tempfile.mkdtemp(prefix=f"bench{k}_")); run = tmp / f"nights_s{n['seed']}"
    shutil.copytree(bench / f"nights_s{n['seed']}", run)
    t = json.load(open(run / f"nights_s{n['seed']}_n3.truth.json")); rug = t.get("rug")
    t["items"] = [{"id": uid, "kind": "toy", "x": p[0], "y": p[1], "half": _items.footprint_half(name), "height_m": 0.05, "new_tonight": True}
                  for name, uid, p in n["items"]]
    json.dump(t, open(run / f"nights_s{n['seed']}_n3.truth.json", "w"))
    out = tmp / "out.json"
    try:
        res = replay([sys.executable, str(HERE / "score_confirm.py"), "--run-dir", str(run), "--seed", str(n["seed"]), "--night", "3",
                      "--db", n["db"], "--out", str(out)] + confirmer_args(n, extra), out, bench / f"err_{n['run']}.txt")
    finally:
        if not out.exists(): shutil.rmtree(tmp, ignore_errors=True)   # a failed night leaves no temp copy behind
    dec = res.get("decisions", [])
    sn = res.get("snaps") or []                                     # the subsystems' own records, for the pipeline report ([63R])
    json.dump({"snap_frames": len(sn), "snap_locked": sum(1 for x in sn if x and x[5]), "decisions": {k: sum(d["decision"] == k for d in dec)
               for k in ("confirmed", "unsure", "not_a_find", "dropped")}}, open(bench / f"stages_{n['run']}.json", "w"))
    if n.get("traj") and all("x_robot" in d for d in dec):          # where the robot would really walk to: its odometry's
        import numpy as np                                          # spot at that moment, through the sim's true pose then
        T = np.genfromtxt(n["traj"], delimiter=",", names=True); U = {c: np.unwrap(T[c]) if c.endswith("yaw") else T[c] for c in T.dtype.names}
        it = lambda c, t: float(np.interp(t, U["wall_t"], U[c]))
        for d in dec:
            t = d["ts"]; th = it("true_yaw", t) - it("odom_yaw", t); c, s_ = math.cos(th), math.sin(th)
            px, py = d["x_robot"] - it("odom_x", t), d["y_robot"] - it("odom_y", t)
            d["x"], d["y"] = c * px - s_ * py + it("true_x", t), s_ * px + c * py + it("true_y", t); d.pop("cells", None)
    if n.get("to_true"):                                             # scoring only: the maps' own frame (the pose graph's, the
        import numpy as np                                           # shape solve's) laid on the sim's by its paths' rigid fit,
        R_, t_ = np.array(n["to_true"][0]), np.array(n["to_true"][1])   # so finds are compared with where the toys truly lie
        for d in dec:
            d["x"], d["y"] = (R_ @ [d["x"], d["y"]] + t_).tolist()
            if d.get("cells"): d["cells"] = [(R_ @ c + t_).tolist() for c in np.asarray(d["cells"], float)]
    shutil.rmtree(tmp, ignore_errors=True)
    cand = [d for d in dec if d["decision"] in ("confirmed", "unsure")]
    reach = {tuple(p): max(NEAR_M, _items.footprint_half(name)[0] + 0.15) for name, _, p in n["items"]}   # a long toy's origin
    near = lambda d, p: min(math.hypot(cx - p[0], cy - p[1]) for cx, cy in (d.get("cells") or [[d["x"], d["y"]]])) < reach[tuple(p)]   # can sit at an end
    det = sum(any(near(d, p) for d in cand) for _, _, p in n["items"])
    conf = sum(any(near(d, p) for d in cand if d["decision"] == "confirmed") for _, _, p in n["items"])
    falses = [d for d in cand if not any(near(d, p) for _, _, p in n["items"])]
    on_rug = sum(bool(rug) and abs(d["x"] - rug["x"]) <= rug["w"] / 2 and abs(d["y"] - rug["y"]) <= rug["h"] / 2 for d in falses)
    json.dump(falses, open(bench / f"false_{n['run']}.json", "w"))                 # where they are, for a look
    labelled = [dict({k: v for k, v in d.items() if k != "cells"}, at_toy=any(near(d, p) for _, _, p in n["items"])) for d in dec]
    json.dump(labelled, open(bench / f"decisions_{n['run']}.json", "w"))           # every decision, at a toy or not: the
                                                                                   # stop gate's bench (which ones to stop for)
    return det, conf, len(falses), len(n["items"]), [name for name, _, p in n["items"] if not any(near(d, p) for d in cand)], on_rug


def main():
    args = sys.argv[1:]; bench = HK / "data" / "bench" / "detect_cal7"
    if args[:1] == ["--bench"]: bench = Path(args[1]); args = args[2:]
    args = [x for x in args if x != "--per-night"]                               # the bench's own, not the confirmer's
    nights = json.load(open(bench / "nights.json"))
    with ThreadPoolExecutor(max_workers=min(len(nights), max(1, (os.cpu_count() or 4) - 3))) as ex:
        res = list(ex.map(lambda kn: night(bench, kn[0], kn[1], args), enumerate(nights)))
    det, conf, false, tot = (sum(r[i] for r in res) for i in range(4))
    missed = [m for r in res for m in r[4]]
    for n, r in zip(nights, res):
        if r[4] or "--per-night" in sys.argv: print(f"  {n['run']} s{n['seed']}: {r[0]}/{r[3]}, {r[2]} near no toy ({r[5]} on the rug); missed {r[4]}")
    json.dump({"settings": args, "detected": det, "confirmed": conf, "toys": tot, "false_per_night": round(false / len(res), 1)},
              open(bench / "last_result.json", "w"))                  # the latest replay's result, for the pipeline report ([63R])
    print(f"settings {' '.join(args) or '(defaults)'}: walk-through detected {det}/{tot} ({det / tot:.0%}), confirmed {conf}/{tot}; "
          f"candidates near no toy {false / len(res):.1f} a night ({sum(r[5] for r in res) / len(res):.1f} on the rug); missed: {missed}")


if __name__ == "__main__":
    main()
