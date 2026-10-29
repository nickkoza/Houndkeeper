#!/usr/bin/env bash
# The tidy loop end to end on DimOS's own sim ([YWo]): DimOS runs the robot (navigation, mapping, postures, cameras,
# the arm through its sim manipulator adapter); houndkeeper.tidy finds items and picks them. Viewer on (DimOS's).
# Usage: tidy_run.sh [items json] [start "x, y"]   env: FLAGS (finds from a finished sweep) or SWEEP (route + earlier
# nights: walk it and confirm finds on approach, [d8d]); neither: the head-camera stand-in
set -uo pipefail
source "$(dirname "$0")/gpu_lock.sh"   # the GPU alone while DimOS runs ([jlr])
HK=$(cd "$(dirname "$0")/../.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
OUT=$HK/data/runs/dimos_tidy/$(date +%Y%m%d_%H%M%S); mkdir -p "$OUT"
ROOM=${ROOM:-nights_s70_n1}                     # the house from the PoC nights, with no clutter of its own (make_nights.py
TRUTH=$HK/sim/house/runs/nights_${ROOM#nights_}; TRUTH=${TRUTH%_n*}/$ROOM.truth.json; [ -n "${RUN_DIR:-}" ] && TRUTH=$RUN_DIR/$ROOM.truth.json   # an experiment's own (night_tidy.sh RUN_DIR)   # --seed 70 --per-night 0); ROOM=office1 for DimOS's
if [ "$ROOM" != office1 ] && [ -f "$TRUTH" ]; then                                    # stock scene
  mapfile -t HI < <($DIMOS/.venv/bin/python $HK/sim/dimos/house_items.py "$TRUTH" "${N_ITEMS:-auto}" "${ITEM_SEED:-1}")
  START=${2:-${HI[0]}}; ITEMS=${1:-${HI[1]}}
else
  START=${2:-"-1.0, 1.0"}
  ITEMS=${1:-'[["triceratops","triceratops",[0.3,1.1,0.1]],["toy_taxi","toy_taxi",[-1.0,2.1,0.1]],["block","block",[-1.9,0.3,0.1]]]'}
fi
echo "room $ROOM, start $START, items $ITEMS"
EXPORTS=$(cd $HK/sim/dimos && $DIMOS/.venv/bin/python -c "import env; print(env.shell_exports('''$ITEMS'''))") \
  || { echo "env.py failed: not running the sim without its settings"; exit 1; }
eval "$EXPORTS"                                   # every variable env.py sets (an empty one would run stock DimOS)
cd "$DIMOS"; export PATH="$DIMOS/.venv/bin:$PATH"; unset OPENAI_API_KEY; export PYTORCH_CUDA_ALLOC_CONF=expandable_segments:True
export DIMOS_SIM_TRUTH_FILE="$OUT/truth.json"; export DIMOS_SIM_TRUTH_TRAJ="$OUT/traj.csv"   # where every body, and the robot's path, truly were
export DIMOS_SIM_TRUTH_BODIES="trunk,tray"       # the truth file names the robot's trunk and its tray too (score_tidy.py; [UvE])
export HK_LOOKS_DIR="$OUT/looks"                 # every close look's inputs, for replaying a miss offline
export DIMOS_SIM_SPEED=${DIMOS_SIM_SPEED:-1}    # real time: arm commands are streamed in wall-clock time, as on the real robot
dimos stop >/dev/null 2>&1; sleep 2
ROOM_ARG=(); [ "$ROOM" != office1 ] && ROOM_ARG=(--mujoco-room "$ROOM")
REC=(); [ "${RECORD:-1}" = 1 ] && REC=(--record sqlite)   # the tidy's own frames, so a miss or a false stop can be replayed
MIND=(); [ "${HK_MIND:-0}" = 1 ] && MIND=(houndkeeper.phone-cockpit --local-relay --extensions-dir "$HK/cockpit/extensions" --open-browser false)
                                                 # HK_MIND=1: what he thinks, live, on the cockpit's Mind page ([RNC]; 127.0.0.1:7780)
dimos --simulation mujoco "${ROOM_ARG[@]}" --mujoco-start-pos "$START" "${REC[@]}" run unitree-go2 mcp-server houndkeeper.tidy \
  "${MIND[@]}" > "$OUT/dimos.log" 2>&1 &
READY=0
for i in $(seq 1 80); do timeout 15 dimos mcp list-tools 2>/dev/null | grep -q tidy_start && { READY=1; break; }; sleep 3; done
[ "$READY" = 1 ] || { echo "FAILED to start"; grep -m5 -E "Error|Traceback" "$OUT/dimos.log"; dimos stop; exit 1; }
if [ -n "${SEED_RECS:-}" ] && [ -f "$SEED_RECS" ]; then     # start from the house map the sweeps built (as sweep_night.sh does)
  if $DIMOS/.venv/bin/python $HK/sim/house/house_map.py "$SEED_RECS" --out "$OUT/house_map" > "$OUT/seed.log" 2>&1 \
     && M=$(ls -t "$OUT"/house_map/house_map_upto*.npy | head -1) && [ -n "$M" ] \
     && $DIMOS/.venv/bin/python $HK/sim/house/inject_map.py "$M" > "$OUT/seed_inject.log" 2>&1; then
    grep -v Zenoh "$OUT/seed_inject.log" | tail -n 1 | tee -a "$OUT/seed.log"
  else echo "not seeded: the house map or its injection failed ($OUT/seed.log); DimOS maps as it goes" | tee -a "$OUT/seed.log"; fi   # ([SdA]: said, not silent)
fi
echo "ready; starting tidy ($OUT)"; sleep 5
export NIGHT_STARTED=$(date +%s)              # night.json: when the tidy began (epoch s)
if [ -n "${SWEEP:-}" ]; then dimos mcp call tidy_start -j "$("$DIMOS/.venv/bin/python" -c 'import json,sys; print(json.dumps({"sweep": sys.argv[1]}))' "$SWEEP")" 2>&1 | tail -1   # sweep, confirming on approach
elif [ -n "${FLAGS:-}" ]; then dimos mcp call tidy_start -j "$("$DIMOS/.venv/bin/python" -c 'import json,sys; print(json.dumps({"flags": sys.argv[1]}))' "$FLAGS")" 2>&1 | tail -1   # the sweep's finds
else dimos mcp call tidy_start 2>&1 | tail -1; fi                                  # no flags: the head-camera stand-in
for i in $(seq 1 400); do
  S=$(timeout 20 dimos mcp call tidy_status 2>/dev/null | tail -1); echo "$S" > "$OUT/status.json"
  STATE=$(echo "$S" | python3 -c "import sys,json
try:
    t=sys.stdin.read(); t=t[t.index('{'):]; print(json.loads(t)['state'])
except Exception as e: print('?')")
  echo "$(date +%T) $STATE"
  case "$STATE" in done|failed|stopped) break;; esac   # stopped: [Qd5], the run ended where it stood
  grep -qE "\[mujoco\] (Traceback|libc\+\+abi)" "$OUT/dimos.log" 2>/dev/null && { echo "the sim process crashed (dimos.log)"; break; }   # fail fast, not 100 min
  # of "starting"; a renderer abort (Filament's PostconditionPanic) prints no Traceback, only libc++abi (18 Oct)
  [ "$STATE" = starting ] && [ "$i" -ge 20 ] && { echo "still starting after 5 min"; break; }
  sleep 15
done
NIGHT_ENDED=$(date +%s) "$DIMOS/.venv/bin/python" -c "import sys; sys.path.insert(0, sys.argv[2]); from houndkeeper.morning_log import write_night_json; write_night_json(sys.argv[1])" \
  "$OUT" "$HK/src" || echo "night.json not written: the morning log goes without the night's names"   # seed, mode, item seed,
                                                 # round, night_tidy folder, times, and route.json (NIGHT_*; no truth, [uoW])
cp "$OUT/status.json" "$OUT/report.json"; sleep 3
"$DIMOS/.venv/bin/python" "$HK/sim/dimos/score_tidy.py" "$OUT" | tee "$OUT/score.txt"   # (it measures the items: needs MuJoCo)
RUNID=$(dimos status 2>&1 | awk '/Run ID/{print $3}'); echo "report: $OUT/report.json"; dimos stop >/dev/null 2>&1
DB=""; [ -n "$RUNID" ] && DB=$(ls -d "$DIMOS"/recordings/*"$RUNID"*/memory.db 2>/dev/null | head -1)   # no run id (the sim
[ -n "$DB" ] && echo "$DB" > "$OUT/recording.txt" && echo "recording: $DB"   # gone): the glob would name the oldest recording
[ -n "$DB" ] && touch "$(dirname "$DB")/houndkeeper-sim"   # a sim run, whatever it streams (recordings.SIM_MARKER, [YP8])
# The butler's morning log, every night ([uoW]): the recording cataloged
# first (DimOS has stopped writing it), since crops come only from recordings the catalog knows ([hKP], [R1X])
"$DIMOS/.venv/bin/hk-data" scan --min-age 0 >/dev/null 2>&1 || echo "catalog scan failed: the morning log goes without pictures"
"$DIMOS/.venv/bin/python" "$HK/src/houndkeeper/morning_log.py" "$OUT" > "$OUT/morning_log.out" 2>&1 \
  && echo "morning log: $OUT/morning_log.html" || echo "no morning log: $OUT/morning_log.out"
