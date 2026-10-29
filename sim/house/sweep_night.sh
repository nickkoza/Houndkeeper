#!/usr/bin/env bash
# Usage: sweep_night.sh <seed> <night> [max_waypoints] [explore_seconds]
# Explores first so the map exists (move_to rejects goals in unknown space), then sweeps.
# Launches the night's scene (agent off, recording on), drives the sweep route with move_to, stops.
set -uo pipefail
source "$(dirname "$0")/../dimos/gpu_lock.sh"   # the GPU alone while DimOS runs ([jlr])
SEED=$1; export HK_HOUSE_SEED=$SEED; N=$2; MAXWP=${3:-9999}; EXPLORE=${4:-}   # the house's own plan ([uCP])
# exploration's longest wait (s): 240, or 60 once the house map is loaded (below)
HK=$(cd "$(dirname "$0")/../.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
SIM=$HK/sim/house; ROOM=nights_s${SEED}_n${N}
PY=${PY:-$DIMOS/.venv/bin/python}   # the environment with numpy and DimOS (a bare "python" had neither: the
                                                 # house map seeding failed silently, 27 Sep)
RUN=${RUN_DIR:-$SIM/runs/nights_s$SEED}; LOG=$RUN/$ROOM.sweep.log; STATE=$SIM/runs/state.json
source "$SIM/lib/build.sh"                      # each derived file removed first and required again ([SdA], [gI8])
export DIMOS_SIM_TRUTH_TRAJ=$RUN/$ROOM.traj.csv    # sim-only scoring aid: the robot's true and believed path (localization's grade)
state() { printf '{"night": %s, "phase": "%s", "wp": %s, "of": %s, "t": %s}\n' "$N" "$1" "${2:-0}" "${3:-0}" "$(date +%s)" > "$STATE.tmp" && mv "$STATE.tmp" "$STATE"; }
state starting
# Required nav settings in DimOS's costmap, ours set from sim/dimos/env.py ([UvE]): the pass-under rule off (it sees
# only a cell's min and max, so it marked half-seen furniture, and would mark walls under a ceiling, as free), and
# points above the indoor house's ceiling cut dropped first. Read from the config DimOS will run with; refuse without.
eval "$(cd "$SIM/../dimos" && $PY -c 'import env; print(env.costmap_exports())')"
COSTMAP_OK=0; $PY "$SIM/../dimos/effective_config.py" --check-costmap unitree-go2-agentic > "$RUN/costmap_check.log" 2>&1 && COSTMAP_OK=1
grep "^DimOS costmap" "$RUN/costmap_check.log"
[ "$COSTMAP_OK" = 1 ] || { echo "the costmap lacks our settings (sim/dimos/env.py COSTMAP; $RUN/costmap_check.log); refusing to run"; exit 1; }
if ! grep -q 'mujoco_sim_speed' "$DIMOS/dimos/simulation/mujoco/mujoco_process.py"; then
  echo "the DimOS fork has no sim speed option (sim/dimos/dimos_fork_changes.patch not applied); refusing to run"; exit 1
fi
# Sim speed: as fast as the loop allows (measured 0.16-0.48x real time, 3 Oct 2026) unless the caller sets DIMOS_SIM_SPEED
# (1 = real time). Logged per night, and the night's measured speed at its end ([5SS]).
export DIMOS_SIM_SPEED="${DIMOS_SIM_SPEED:-0}"
echo "$(date -Iseconds) night $N sim speed: $([ "$DIMOS_SIM_SPEED" = 0 ] && echo 'fast (max)' || echo "${DIMOS_SIM_SPEED}x")" >> "$RUN/run_notes.txt"
cd "$DIMOS"; export PATH="$DIMOS/.venv/bin:$PATH"; unset OPENAI_API_KEY
START=$(python3 -c "import json;p=json.load(open('$RUN/$ROOM.truth.json'))['start_pos'];print(f'{p[0]}, {p[1]}')")
# The route is planned on the robot's own map of the nights before this one ([HuM]; ROUTE_ARGS e.g. --near dock --radius r,
# --night last wins); a first night has none, so it is planned after the exploration, on tonight's own recording
route() { build --stdout --log "$RUN/route.log" "$RUN/route.json" -- $PY "$SIM/sweep_route.py" ${ROUTE_ARGS:-} --night "$N" "$@"; }
has_wps() { python3 -c "import json,sys; sys.exit(0 if json.load(open('$RUN/route.json'))['waypoints'] else 1)"; }
ROUTE_LATE=0
if ! route; then
  grep -q "no map of this house yet" "$RUN/route.log" && ROUTE_LATE=1 || { echo "FAILED: no route"; exit 1; }
  echo "no map yet: the route is planned after the exploration, on tonight's recording"
elif ! has_wps; then echo "FAILED: the route has no waypoints: a night with nothing swept is not a swept night"; exit 1; fi
dimos stop >/dev/null 2>&1; sleep 2
EXTRA_BP=""; [ "${OVERLAY:-1}" = 1 ] && EXTRA_BP="houndkeeper.floor-overlay"
[ -n "${HK_AREA:-}" ] && EXTRA_BP="$EXTRA_BP houndkeeper.operating-area"   # the operating area ([JVr]): exploration and
                                                 # the sweep stay inside it (its costmap layer; the route is planned inside too)
[ "${SWEEP_WRIST:-0}" = 1 ] || export DIMOS_SIM_ARM_CAMERA=""   # the floor pictures use the head camera only: rendering the wrist
                                                                # camera too was half the sweep's camera time (SWEEP_WRIST=1 keeps it)
export HK_NIGHT=$N HK_BASELINE=${OVERLAY_BASE:-$SIM/runs/nights_s7_r2} HK_MODEL_DIR=${HK_MODEL_DIR:-$HK/data/media/floor}
dimos --simulation mujoco --mujoco-room "$ROOM" --mujoco-start-pos "$START" --record sqlite run unitree-go2-agentic $EXTRA_BP \
  --disable mcp-client --disable speak-skill --disable perceive-loop-skill --robot-width "${ROBOT_WIDTH:-0.5}" --voxelgridmapper.carve-columns "${CARVE:-false}" > "$LOG" 2>&1 &
READY=0
for i in $(seq 1 60); do timeout 15 dimos mcp list-tools 2>/dev/null | grep -q move_to && { READY=1; break; }; sleep 3; done
if [ "$READY" != 1 ]; then
  echo "night $N: startup failed ($(grep -m1 -iE "error" "$LOG" | cut -c1-120)); relaunching without the overlay and carve flag" | tee -a "$RUN/run_warnings.txt"
  dimos stop >/dev/null 2>&1; sleep 2
  dimos --simulation mujoco --mujoco-room "$ROOM" --mujoco-start-pos "$START" --record sqlite run unitree-go2-agentic \
    --disable mcp-client --disable speak-skill --disable perceive-loop-skill --robot-width "${ROBOT_WIDTH:-0.5}" > "$LOG" 2>&1 &
  for i in $(seq 1 60); do timeout 15 dimos mcp list-tools 2>/dev/null | grep -q move_to && { READY=1; break; }; sleep 3; done
fi
[ "$READY" = 1 ] || { echo "FAILED to start:"; grep -m3 -E "Error|Traceback" "$LOG"; dimos stop >/dev/null 2>&1; exit 1; }
RUNID=$(dimos status 2>&1 | awk '/Run ID/{print $3}')
[ -n "$RUNID" ] || { echo "FAILED: DimOS gave no run id: its recording could not be found"; dimos stop >/dev/null 2>&1; exit 1; }
[ "$ROUTE_LATE" = 1 ] && echo "== $ROOM ready (run $RUNID); route after the exploration" \
  || echo "== $ROOM ready (run $RUNID); route $(python3 -c "import json;r=json.load(open('$RUN/route.json'));print(len(r['waypoints']),'waypoints,',r['length_m'],'m')")"
# Seed from the saved house map (every previous night of this run), then only a short settle instead of mapping. The
# house map keeps what 2+ nights saw, so night 2's is empty: seeded only with a map of SEED_MIN_PTS points or more ([fis];
# 0 injected counted as seeded on every night 2 until 3 Oct 2026, so it explored 60 s, not 240). Seeded maps were
# 86,000-101,000 points (houses 7 and 70, nights 3-8).
SEED_RECS=${SEED_RECS:-$RUN/resweep_recordings.txt}; SEEDED=0; SEED_MIN_PTS=${SEED_MIN_PTS:-1000}   # this house's own:
state "loading house map"                       # never another house's map
if [ -f "$SEED_RECS" ] && [ "$N" -gt 1 ] && $PY "$SIM/house_map.py" "$SEED_RECS" --upto $((N-1)) --out "$RUN/house_map" > "$RUN/seed_n$N.log" 2>&1 \
   && cp "$RUN/house_map/house_map_upto$((N-1)).npy" "$RUN/seed_n$N.npy"; then
  NPTS=$($PY -c "import sys, numpy as np; print(len(np.load(sys.argv[1])))" "$RUN/seed_n$N.npy" 2>/dev/null || echo 0)
  if [ "$NPTS" -lt "$SEED_MIN_PTS" ]; then
    echo "not seeded: the house map has $NPTS points (under $SEED_MIN_PTS); the full mapping pass" | tee -a "$RUN/seed_n$N.log"
  elif $PY "$SIM/inject_map.py" "$RUN/seed_n$N.npy" > "$RUN/seed_n${N}_inject.log" 2>&1; then
    grep -v Zenoh "$RUN/seed_n${N}_inject.log" | tail -n 1 >> "$RUN/seed_n$N.log"; SEEDED=1
    echo "seeded: $(tr '\n' ' ' < "$RUN/seed_n$N.log")"   # keep the full mapping pass: it refines the seeded map
  else echo "not seeded: injecting the house map failed (seed_n${N}_inject.log); the full mapping pass" | tee -a "$RUN/seed_n$N.log"; fi
fi
trap 'dimos stop >/dev/null 2>&1' EXIT            # however the sweep ends (a crash, a kill, an error), its DimOS goes with it:
                                                 # an orphaned sim held the GPU lock for 90 min on 27 Sep
sim_crashed() { grep -q "\[mujoco\] Traceback" "$LOG" 2>/dev/null; }   # the sim process died: the sweep must fail, not walk on
die_crashed() { echo "FAILED: the sim process crashed:"; grep -m1 -A12 "\[mujoco\] Traceback" "$LOG" | tail -3; dimos stop >/dev/null 2>&1; exit 1; }
EXPLORE=${EXPLORE:-$([ "$SEEDED" = 1 ] && echo 60 || echo 240)}   # seeded from earlier nights, a short refining pass is enough
                                                                  # (it never finished early: every sweep waited the full 240 s)
state mapping
DONE0=$(grep -c "Exploration complete" "$LOG"); NOF0=$(grep -c "No frontier found\|No path found" "$LOG"); GOAL0=$(grep -c "Goal reached, finding next frontier" "$LOG")
dimos mcp call begin_exploration >/dev/null 2>&1
for i in $(seq 1 "$EXPLORE"); do                # end when DimOS says the house is explored, or, once it has reached a frontier, first
  [ "$(grep -c "Exploration complete" "$LOG")" -gt "$DONE0" ] && break   # finds none left or picks one it cannot reach (max EXPLORE s):
  [ "$(grep -c "No frontier found\|No path found" "$LOG")" -gt "$NOF0" ] && [ "$(grep -c "Goal reached, finding next frontier" "$LOG")" -gt "$GOAL0" ] && break
  sleep 1                                       # it retries ten times 2 s apart, or waits out a 30 s goal timeout, and the robot
done                                            # stood through each (2 Oct 2026, the demo room's night 1: 20 s, then 24 s)
dimos mcp call end_exploration >/dev/null 2>&1
sim_crashed && die_crashed
echo "explored ${EXPLORE}s; pose $(timeout 2 dimos topic echo /odom 2>&1 | grep -m1 PoseStamped)"   # (2 s: the robot waits on it)
if [ "$ROUTE_LATE" = 1 ]; then                    # a first night: tonight's recording so far is the map (read while it records)
  TONIGHT=$(ls -d "$DIMOS"/recordings/*"$RUNID"*/memory.db 2>/dev/null | head -1)
  [ -n "$TONIGHT" ] || { echo "FAILED: no recording of tonight's exploration to plan on"; exit 1; }
  route --db "$TONIGHT" || { echo "FAILED: no route from tonight's exploration"; exit 1; }
  has_wps || { echo "FAILED: the route has no waypoints: a night with nothing swept is not a swept night"; exit 1; }
  echo "route from the exploration: $(python3 -c "import json;r=json.load(open('$RUN/route.json'));print(len(r['waypoints']),'waypoints,',r['length_m'],'m')")"
fi
NWP=$(python3 -c "import json;print(len(json.load(open('$RUN/route.json'))['waypoints'][:$MAXWP]))")
T0=$(date +%s); i=0
python3 -c "import json;[print(p[0],p[1]) for p in json.load(open('$RUN/route.json'))['waypoints'][:$MAXWP]]" | while read X Y; do
  i=$((i+1)); state sweeping "$i" "$NWP"; R=$(dimos mcp call move_to -t 180 -j "{\"x\": $X, \"y\": $Y}" 2>&1 | tail -1 | cut -c1-110)
  if [ "$i" = 1 ] && [ "$SEEDED" = 1 ] && echo "$R" | grep -q "cancelled or failed"; then
    echo "seed not usable for navigation; falling back to the full mapping pass"
    dimos mcp call begin_exploration >/dev/null 2>&1; sleep 240; dimos mcp call end_exploration >/dev/null 2>&1
    R=$(dimos mcp call move_to -t 180 -j "{\"x\": $X, \"y\": $Y}" 2>&1 | tail -1 | cut -c1-110)
  fi
  echo "wp$i ($X,$Y) +$(( $(date +%s)-T0 ))s: $R"
  sim_crashed && die_crashed
done
sim_crashed && die_crashed                        # (the loop runs in a pipe's subshell: its exit doesn't end the script)
state processing; [ -n "${OVPID:-}" ] && kill $OVPID 2>/dev/null
dimos stop >/dev/null 2>&1; sleep 2
DB=$(ls -d "$DIMOS"/recordings/*"$RUNID"*/memory.db 2>/dev/null | head -1); echo "recording: ${DB:-NOT FOUND}"
[ -n "$DB" ] || { state failed; exit 1; }
[ -n "$DB" ] && touch "$(dirname "$DB")/houndkeeper-sim"   # a sim run, whatever it streams (recordings.SIM_MARKER, [YP8])
echo "$(date -Iseconds) night $N measured speed: $($PY "$HK/sim/dimos/run_speed.py" "$LOG" "$RUN/$ROOM.traj.csv" 2>/dev/null || echo "speed not read")" >> "$RUN/run_notes.txt"
state done
