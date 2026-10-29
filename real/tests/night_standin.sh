#!/usr/bin/env bash
# The dog-alone night rehearsed ([jFy]): DimOS's sim robot stands in for the dog and real/night.sh runs the night as
# it would on the robot. This script is the world, not the robot: it puts the sim robot on its dock and toys on the
# floor of a sim house's third night (house_items.py, as tidy_run.sh does, reading the sim's truth for it), and hands
# real/night.sh only DimOS's sim arguments. The house's frame and floor map are the real-data rehearsal's
# (real/tests/rehearsal.sh: the house's two baseline nights copied as bare recordings and built by real_nights.sh), so
# the night sees nothing of the sim but its camera, lidar and odometry. The toys placed are kept beside the night
# (world_items.json) for scoring after the fact; nothing the night runs reads them.
# Usage: night_standin.sh [house] [item seed]   (default house 70, seed 1)
#   env: OUT (default <workspace>/build/realnight/standin)  HK_HOUSE (default the rehearsal's run for the house)
#        RADIUS, WALK_MAX_MIN, LIGHT_CHECK (passed on)
set -uo pipefail
H=${1:-70}; ITEM_SEED=${2:-1}
case " 78 80 81 $(seq -s ' ' 100 119) " in *" $H "*) echo "house $H is a hold-out: never rehearsed"; exit 2;; esac
HK=$(cd "$(dirname "$0")/../.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
PY=$DIMOS/.venv/bin/python
source "$HK/sim/dimos/gpu_lock.sh"                 # the sim's renderer: the GPU alone while DimOS runs ([jlr])
OUT=${OUT:-$WS/build/realnight/standin}/s$H; mkdir -p "$OUT"
export HK_HOUSE=${HK_HOUSE:-$WS/build/rehearsal/s$H/run} HK_REAL_NIGHTS=$OUT/nights
TRUTH=$HK/sim/house/runs/nights_s$H/nights_s${H}_n3.truth.json    # the world's: where the dock is, where toys can go
[ -f "$TRUTH" ] || { echo "house $H has no third night's scene ($TRUTH)"; exit 1; }
mapfile -t HI < <($PY "$HK/sim/dimos/house_items.py" "$TRUTH" auto "$ITEM_SEED")
START=${HI[0]}; ITEMS=${HI[1]}
[ -n "$ITEMS" ] || { echo "no toys placed"; exit 1; }
EXPORTS=$(cd "$HK/sim/dimos" && $PY -c "import env; print(env.shell_exports('''$ITEMS'''))") && [ -n "$EXPORTS" ] \
  || { echo "env.py failed: not running the sim without its settings"; exit 1; }
eval "$EXPORTS"
export DIMOS_SIM_SPEED=${DIMOS_SIM_SPEED:-1} OMP_NUM_THREADS=${OMP_NUM_THREADS:-2}
STAMP=$(date +%Y%m%d_%H%M%S)
echo "{\"house\": $H, \"item_seed\": $ITEM_SEED, \"start\": \"$START\", \"items\": $ITEMS, \"at\": \"$STAMP\"}" > "$OUT/world_$STAMP.json"
echo "world: house $H night 3, dock $START, $(echo "$ITEMS" | $PY -c 'import json,sys; print(len(json.load(sys.stdin)))') toys ($OUT/world_$STAMP.json)"
ROBOT=sim STANDIN_ARGS="--simulation mujoco --mujoco-room nights_s${H}_n3 --mujoco-start-pos=${START// /}" \
  nice -n 15 bash "$HK/real/night.sh"
RC=$?
TN=$(ls -d "$HK_REAL_NIGHTS"/*/ 2>/dev/null | sort | tail -1)
[ -n "$TN" ] && cp "$OUT/world_$STAMP.json" "$TN/world_items.json"
echo "stand-in night: exit $RC (${TN:-no night directory})"
exit $RC
