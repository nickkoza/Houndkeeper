#!/usr/bin/env bash
# The detection bench ([d8d], [QDc]): record night 3's walk-through for each tuning house and toy set (no stops, no
# picking), and snapshot each house's baselines, so bench_detect.py can replay the confirmer over them with any settings.
# Tuning sets only ([I0Q]), or the validation house once ([ISF]). Usage: make_detect_bench.sh <name> <seed>:<toy set> ...
# e.g. rug1 70:1 70:2 71:1. Replay with --corrections @corrections when the baselines carry a floor map (floor_frame.json).
set -uo pipefail
HK=$(cd "$(dirname "$0")/../.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
export HK PY=$DIMOS/.venv/bin/python; B=$HK/data/bench/$1; shift; mkdir -p "$B"
[ -f "$B/nights.json" ] || echo "[]" > "$B/nights.json"
for st in "$@"; do
  seed=${st%%:*}; set_=${st##*:}
  log=$B/walk_s${seed}_t${set_}.log
  SEED=$seed ITEM_SEED=$set_ MODE=walk bash $HK/sim/dimos/night_tidy.sh > "$log" 2>&1
  db=$(awk '/^walk recording:/{print $3}' "$log" | tail -1)
  [ -f "$db" ] || { echo "$(date +%T) s$seed t$set_: no recording (see $log)"; continue; }
  [ -d "$B/nights_s$seed" ] || cp -r "${RUN_DIR:-$HK/sim/house/runs/nights_s$seed}" "$B/nights_s$seed"   # baselines: the same all tuning nights
  "$PY" - "$B/nights.json" "$seed" "$db" "$log" "$set_" <<'EOF'   # (the project's python: the floor frame needs numpy)
import json, sys
p, seed, db, log, set_ = sys.argv[1:]
items = json.loads(next(l for l in open(log) if l.startswith("items: "))[7:])
n = [x for x in json.load(open(p)) if not (x["seed"] == int(seed) and x.get("toy_set") == int(set_))]
rec = {"seed": int(seed), "toy_set": int(set_), "db": db, "items": items, "run": f"s{seed}_t{set_}"}
run = __import__("pathlib").Path(p).resolve().parent / f"nights_s{seed}"   # absolute: the bench replays from sim/house
if (run / "floor_frame.json").exists():             # the baselines' floor map has its own frame ([KkQ]): tonight's frames
    import subprocess                               # placed on its lidar map as the robot would, and (scoring only) that
    HK = __import__("pathlib").Path(__import__("os").environ["HK"]); PY = __import__("os").environ["PY"]   # from the script above
    c = run / f"corrections_s{seed}_t{set_}.npz"   # frame laid on the sim's by night 1's paths, to score finds where toys lie
    subprocess.run([PY, str(HK / "sim/house/tonight_frame.py"), db, "--run", str(run), "--out", str(c)], check=True, cwd=HK / "sim/house")
    sys.path.insert(0, str(HK / "sim/house")); import floor_fidelity as ff
    R, t = ff.frame_from_path(str(run / "shape" / "corrections_1.npz"), str(run / f"nights_s{seed}_n1.traj.csv"))
    rec.update(corrections=str(c), to_true=[R.tolist(), t.tolist()])
n.append(rec)
json.dump(n, open(p, "w"), indent=1)
EOF
  echo "$(date +%T) s$seed t$set_: recorded $(python3 -c "import json;print(len(json.load(open('$B/nights.json'))))") nights"
done
