#!/usr/bin/env bash
# A tidy night end to end on DimOS, as designed ([YWo], [2w8]): sweep the floor, compare it with last night's, visit
# each new thing head-on, look closer and pick. The sweep and the comparison are the PoC's own tools, unchanged
# (sim/house: sweep_night.sh, floor_mosaic2.py, tall_layer.py, the locked detector detect_changes2.py, run and scored by score_detect2.py).
#   nights 1-2: the house with nothing on the floor (the baseline; swept once per seed and reused). Two, because the
#     house map only keeps what it saw on 2+ nights (house_map.py, smart clearing), and the detector compares with both
#   night 3: the same house with items down, starting from the map of nights 1-2: sweep, compare, flags -> the tidy
# In the sim the tidy relaunches DimOS after the sweep (the PoC's tools read a finished recording) with the items back
# where they were; the real robot does both in one session. The sweep covers RADIUS m around the dock (the full house
# takes ~1 h a night, and the detector's ~30 false alarms a night would each cost a stop).
# MODE=walk: night 3's walk-through only, the tidy never stopping, recorded (the detection bench). MODE=confirm (default, the design): night 3 is one pass, the robot confirming finds on approach and stopping for each
# ([d8d], [QDc]). MODE=visit: the earlier flow (sweep, compare the finished nights, then visit each flag), for comparison.
# MODE=baselines: the two clean nights and their floor map only (floor map error per floor, [aj8]).
# Usage: night_tidy.sh   env: STITCH (1) MODE (confirm) SEED (70) N_ITEMS (auto: one per 200 sq ft) ITEM_SEED (1) RADIUS (5) FRESH_BASELINE (0) PHOTO_FLOORS (1) REGIME (night) HK_ITEMS (train; heldout only to judge)
set -uo pipefail
source "$(dirname "$0")/gpu_lock.sh"   # the GPU alone while DimOS runs ([jlr])
HK=$(cd "$(dirname "$0")/../.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
SIM=$HK/sim/house; PY=$DIMOS/.venv/bin/python
SEED=${SEED:-70}; export HK_HOUSE_SEED=$SEED; RUN=${RUN_DIR:-$SIM/runs/nights_s$SEED}; [ -n "${RUN_DIR:-}" ] && export RUN_DIR=$RUN;   # (sweep_night.sh, tidy_run.sh and make_nights.py follow it)   # RUN_DIR: an experiment's own baselines, never the shared ones
 OUT=$HK/data/runs/night_tidy/$(date +%Y%m%d_%H%M%S); mkdir -p "$OUT" "$RUN"
hud() { printf '{"state": "%s", "note": "%s"}' "$1" "$2" > /dev/shm/houndkeeper_hud.json.tmp && mv /dev/shm/houndkeeper_hud.json.tmp /dev/shm/houndkeeper_hud.json; echo "$(date +%T) $1: $2"; }
EXPORTS=$(cd $HK/sim/dimos && $PY -c "import env; print(env.shell_exports())") && [ -n "$EXPORTS" ] \
  || { echo "env.py failed: not running the sim without its settings"; exit 1; }
eval "$EXPORTS"                                   # every variable env.py sets (an empty one would run stock DimOS)
source $SIM/lib/build.sh                      # each derived file removed first and required again ([SdA], [gI8])
export PYTHONPATH=$HK/src:${PYTHONPATH:-}
export NIGHT_SEED=$SEED NIGHT_MODE=${MODE:-confirm} NIGHT_ITEM_SEED=${ITEM_SEED:-1} NIGHT_ROUND=${ROUND:-} NIGHT_TIDY_DIR=$OUT   # the
                                              # tidy's night.json names its night by these (ROUND: the round, if one runs it)
# the house, nights 1-3, each with its own lighting (day or night, [QMt]); indoors unless INDOOR=0. Regenerated every time
# (seeded, a few seconds): a baseline is reused only if its scene file is unchanged too (see stamp below). Chairs, a stool
# and a kid's chair move every night, baselines too (MOVABLES=0 keeps them out; chairs are
# the first furniture that moves, [nAH] TJu)
build --cd $SIM --tail 3 --log "$OUT/make_nights.log" "$RUN"/nights_s${SEED}_n{1,2,3}.truth.json -- \
  $PY make_nights.py --seed $SEED --nights 3 --per-night 0 $([ "${INDOOR:-1}" = 1 ] && echo --indoor) ${RUN_DIR:+--out-dir "$RUN"} \
   $([ "${MOVABLES:-1}" = 1 ] && echo --movables) $([ "${RELIEF:-1}" = 1 ] || echo --flat) $([ "${PETS:-1}" = 1 ] && echo --pets) \
   $([ "${INDOOR:-1}" = 1 ] && [ "${PHOTO_FLOORS:-1}" = 1 ] && echo --photo-floors) \
   $([ "${INDOOR:-1}" = 1 ] && [ -n "${REGIME-lit}" ] && echo --regime ${REGIME-lit}) \
  || { echo "the house's nights could not be made"; exit 1; }   # photographed floor, lit as
                                              # the robot works: at night, with the testing light on (REGIME=lit, 27 Sep; REGIME=night draws dim nights and
                                              # lamp on or off, [72t]); calibration nights too ([QMt]); REGIME= draws each
# where the sim puts the robot on its dock (the sim's setup, not the robot's knowledge: the route below finds the dock
# where its own odometry began, [HuM])
START=$($PY -c "import json;p=json.load(open('$RUN/nights_s${SEED}_n1.truth.json'))['start_pos'];print(f'{p[0]},{p[1]}')")
export ROUTE_ARGS="--near dock --night 3 --radius ${RADIUS:-5} --clear ${ROUTE_CLEAR:-2}" OVERLAY=0   # ROUTE_CLEAR=1: a small room's
                                              # sweep (two cells of clearance leaves the demo room only scattered points, 2 Oct 2026)
if [ -n "${AREA:-}" ]; then                   # the operating area ([JVr]): a file in the house's frame, kept with its map
  cp "$AREA" "$RUN/operating_area.json" && export HK_AREA=$RUN/operating_area.json || { echo "no operating area at $AREA"; exit 1; }
  echo "operating area: $HK_AREA" | tee "$OUT/area.txt"; cp "$HK_AREA" "$OUT/"
else unset HK_AREA; fi
export SEED_RECS=$RUN/seed_recordings.txt     # later nights start from the house map built from earlier nights' recordings,
                                              # as the PoC did (sweep_night.sh: house_map.py + inject_map.py); "<night> <db>"
sweep() {   # sweep_night.sh <night>, then the floor picture and tall layer from its recording
  local n=$1; hud SWEEP "night $n: sweeping ${RADIUS:-5} m around the dock"
  bash $SIM/sweep_night.sh $SEED $n > "$OUT/sweep_n$n.log" 2>&1 || { echo "night $n: sweep failed"; tail -5 "$OUT/sweep_n$n.log"; exit 1; }
  local db; db=$(awk '/^recording:/{print $2}' "$OUT/sweep_n$n.log" | tail -1)
  [ -f "$db" ] || { echo "night $n: no recording"; tail -5 "$OUT/sweep_n$n.log"; exit 1; }
  { grep -v "^$n " "$SEED_RECS" 2>/dev/null; echo "$n $db"; } > "$SEED_RECS.tmp"; mv "$SEED_RECS.tmp" "$SEED_RECS"   # for later nights' map
  picture $n "$db"
}
source $SIM/night_picture.sh                  # MOSAIC_ARGS, MOSAIC_V and the picture's recipe, shared with real nights ([V35])
picture() {   # night $1's floor picture from its recording $2, and how it was made (a change remakes it, no re-sweep)
  hud DETECT "night $1: building the floor picture"
  night_picture "$RUN" "$1" "$2" "$OUT/picture_n$1.log" || exit 1
}
RENDER=$(PYTHONPATH=$HK/src $PY -c "import json; from houndkeeper.sim_hook import render_settings; print(json.dumps(render_settings()))")
DATA=$DIMOS/data/mujoco_sim
stamp() {   # how night $1 was rendered, and of what: the scene file and every file it names (the floor's rug and stains
  local x="$DATA/scene_nights_s${SEED}_n$1.xml"   # live in its texture, not the XML: a rug put in the baseline went unseen, 26 Sep)
  echo "$RENDER scene=$( { cat "$x"; grep -oE 'file="[^"]+"' "$x" | sort -u | sed -E 's/file="(.*)"/\1/' \
    | while read -r f; do find "$DATA" -maxdepth 3 -name "$(basename "$f")" -print -quit | xargs -r cat; done; } | sha1sum | cut -c1-12)"
}
# nights 1-2: baseline (no items), reused only if rendered exactly as tonight will be
unset HK_SIM_ITEMS
archived=0
keep_old() {   # an existing baseline is never overwritten: the whole run directory is copied aside first (5 Oct 2026: a
  # changed render stamp silently re-swept the shared baselines of houses 72 and 96 under other experiments)
  [ "$archived" = 1 ] && return 0
  local a="$RUN/archive/$(date +%Y%m%d_%H%M%S)"; mkdir -p "$a"
  ( cd "$RUN" && find . -maxdepth 1 -mindepth 1 ! -name archive -exec cp -a {} "$a/" \; ) || { echo "could not archive $RUN"; exit 1; }
  echo "$(date -Iseconds) baseline re-sweep: the previous run directory kept in $a" >> "$RUN/run_notes.txt"; archived=1
}
for b in 1 2; do
  stale=0; [ -f "$RUN/render_n$b.json" ] && [ "$(cat "$RUN/render_n$b.json")" != "$(stamp $b)" ] && stale=1
  if [ "$stale" = 1 ] && [ "${FRESH_BASELINE:-0}" != 1 ]; then   # rendered differently: never re-swept unasked
    echo "baseline night $b of $RUN was rendered differently ($(cat "$RUN/render_n$b.json")) from tonight ($(stamp $b))."
    echo "Refusing to re-sweep it: set FRESH_BASELINE=1 to re-sweep (the old one is archived), or RUN_DIR=<own dir> for an experiment."
    exit 1
  fi
  if [ "${FRESH_BASELINE:-0}" = 1 ] || [ ! -f "$RUN/mosaic_n$b.npz" ] || ! grep -q "^$b " "$SEED_RECS" 2>/dev/null || [ "$stale" = 1 ]; then
    [ -f "$RUN/mosaic_n$b.npz" ] && keep_old
    sweep $b; stamp $b > "$RUN/render_n$b.json"
  elif [ "$(cat "$RUN/mosaic_n$b.args" 2>/dev/null)" != "$MOSAIC_ARGS v$MOSAIC_V" ] || { [[ "$MOSAIC_ARGS" == *--house* ]] && [ ! -f "$RUN/house_lidar.npy" ]; }; then   # same recording, picture made differently
    picture $b "$(awk -v n=$b '$1==n{print $2}' "$SEED_RECS")"
  else echo "baseline night $b: reusing its floor picture and recording ($(stamp $b))"; fi
done
if [ "${STITCH:-1}" = 1 ]; then              # the baselines stitched into one floor, the mount calibrated ([WFI], [fXD])
  hud DETECT "stitching the baseline nights' floor"
  bash $HK/sim/dimos/stitch_baselines.sh "$RUN" "$(awk '$1==1{print $2}' "$SEED_RECS")" "$(awk '$1==2{print $2}' "$SEED_RECS")" \
    || { echo "stitching the baselines failed"; exit 1; }
  if [ -f "$RUN/calibration_check.json" ]; then   # the nightly check of the robot's calibration ([9tQ], [yFg]): reported
    cp "$RUN/calibration_check.json" "$OUT/"       # with the night, never written into the robot's calibration
    echo "calibration check: $($PY -c "import json; print(json.load(open('$RUN/calibration_check.json'))['note'])")"; fi
fi
[ "${MODE:-confirm}" = baselines ] && { echo "baselines and floor map done (MODE=baselines)"; exit 0; }   # the floor map alone
# night 2: the same house, items down
mapfile -t HI < <($PY $HK/sim/dimos/house_items.py "$RUN/nights_s${SEED}_n3.truth.json" "${N_ITEMS:-auto}" "${ITEM_SEED:-1}")   # tonight's: its chairs
export HK_SIM_ITEMS="${HI[1]}"; echo "items: $HK_SIM_ITEMS" | tee "$OUT/items.json"
HK_ARM_DIR=$HK/sim/arm $PY - "$RUN" "$SEED" <<'EOF'
import json, sys, numpy as np
import os; sys.path.insert(0, os.environ["HK_ARM_DIR"])
run, seed = sys.argv[1], sys.argv[2]
t = json.load(open(f"{run}/nights_s{seed}_n3.truth.json"))                       # night 3's own house and lighting
import items                                                                     # scored at each toy's own size (a long toy
t["items"] = [{"id": uid, "kind": "toy", "x": p[0], "y": p[1], "half": items.footprint_half(name), "height_m": 0.05,   # found at
               "new_tonight": True} for name, uid, p in json.loads(os.environ["HK_SIM_ITEMS"])]                         # its end counts)
json.dump(t, open(f"{run}/nights_s{seed}_n3.truth.json", "w"), indent=1)
EOF
[ "${MODE:-confirm}" = walk ] && export HK_TIDY_WATCH_ONLY=1   # the walk-through alone, recorded, never stopping: detection's
                                              # bench ([d8d]: passes of the walk alone)
if [ "${MODE:-confirm}" = confirm ] || [ "${MODE:-confirm}" = walk ]; then   # one pass: walk the route, confirm finds on approach, stop for each ([d8d], [QDc])
  build --cd $SIM --stdout "$OUT/route.json" -- $PY sweep_route.py $ROUTE_ARGS || { echo "no route"; exit 1; }
  SWEEP=$($PY -c "import json; r=json.load(open('$OUT/route.json')); assert r['waypoints']; print(json.dumps({'route': r['waypoints'], 'run_dir': '$RUN', 'night': 3}))") \
    && [ -n "$SWEEP" ] || { echo "the route has no waypoints: not running the tidy without one"; exit 1; }   # (no SWEEP: the
                                              # head-camera stand-in, a different experiment)
  export NIGHT_ROUTE=$OUT/route.json          # copied into the tidy's run, beside its night.json
  echo "route: $($PY -c "import json; r=json.load(open('$OUT/route.json')); print(len(r['waypoints']), 'waypoints,', r['length_m'], 'm')")"
  SWEEP="$SWEEP" ROOM=nights_s${SEED}_n3 bash $HK/sim/dimos/tidy_run.sh "$HK_SIM_ITEMS" "$START" 2>&1 | tee "$OUT/tidy.log" | tail -14
  TIDY_RC=${PIPESTATUS[0]}
  [ "${MODE:-confirm}" = walk ] && echo "walk recording: $(awk '/^recording:/{print $2}' "$OUT/tidy.log" | tail -1)"
  [ "$TIDY_RC" = 0 ] || echo "the tidy failed (tidy_run.sh exit $TIDY_RC)"
  exit "$TIDY_RC"
fi
sweep 3                                       # MODE=visit: sweep, compare the finished nights, then visit each flag
hud DETECT "comparing tonight's floor with last night's"
build --tail 3 --log "$OUT/detect.log" "$RUN/detect_n3.json" -- $PY $SIM/score_detect2.py --seed $SEED --nights 3 --run-dir "$RUN" \
  || { echo "the comparison failed"; exit 1; }
FLAGS=$($PY -c "
import json; b=json.load(open('$RUN/detect_n3.json'))['blobs']
print(json.dumps([[x['x'], x['y']] for x in b if x['kind'] == 'present']))")
echo "flags: $FLAGS" | tee "$OUT/flags.json"
hud "PLAN STOPS" "$(echo "$FLAGS" | $PY -c 'import json,sys; print(len(json.load(sys.stdin)))') finds: one head-on stop each"
FLAGS="$FLAGS" ROOM=nights_s${SEED}_n3 bash $HK/sim/dimos/tidy_run.sh "$HK_SIM_ITEMS" "$START" 2>&1 | tee "$OUT/tidy.log" | tail -12
