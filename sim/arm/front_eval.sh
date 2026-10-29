#!/usr/bin/env bash
# The front-grasp measurement ([ZMc], [Hwc]): two grasp benches one after the other (headless; in parallel they run out of GPU memory), 50 trials each on fixed seeds,
# pooled. Usage: front_eval.sh <label> [extra grasp_bench args]. Output under data/eval/front_<label>_s{41,42}.log.
set -u; L=$1; shift; cd "$(dirname "$0")"; OUT=$PWD/../../data/eval
WS=${HK_WORKSPACE:-$(cd ../../.. && pwd)}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
for s in 41 42; do HK_PICK_RECORD_DIR=$PWD/../../data/demos_auto MUJOCO_GL=egl \
  "$DIMOS/.venv/bin/python" -u grasp_bench.py --trials 50 --seed $s --headless --zone front "$@" > $OUT/front_${L}_s$s.log 2>&1; done
python3 - "$OUT" "$L" <<'PY'
import json, sys, collections
out, L = sys.argv[1], sys.argv[2]; R = []
for s in (41, 42):
    r = [json.loads(l) for l in open(f"{out}/front_{L}_s{s}.log") if l.startswith("{")]; R += r
    if len(r) < 50: print(f"WARNING seed {s} ran {len(r)}/50 trials (see its log); the numbers below are partial")
b = [r for r in R if r["expect"] == "bin"]; v = [r for r in R if r["expect"] == "review"]; x = [r for r in R if r["expect"] == "excluded"]
if x: print(f"  {len(x)} trials set aside: the physics blew up (sim unstable): " + ", ".join(sorted({r['kind'] for r in x})))
ok = sum(r["ok"] for r in b); print(f"FRONT {L}: pickable {ok}/{len(b)} binned ({100 * ok / max(len(b), 1):.0f}%), review items {sum(r['correct'] for r in v)}/{len(v)} correctly left")
print("  pickable failures:", dict(collections.Counter(r["fail"] for r in b if not r["ok"]).most_common()))
per = collections.defaultdict(lambda: [0, 0])
for r in b: per[r["kind"]][0] += r["ok"]; per[r["kind"]][1] += 1
print("  by item:", ", ".join(f"{k} {a}/{n}" for k, (a, n) in sorted(per.items(), key=lambda kv: kv[1][0] / kv[1][1])))
PY
