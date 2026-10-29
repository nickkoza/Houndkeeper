#!/usr/bin/env bash
# A find-and-flag night on the dog alone ([jFy]): one command on the laptop, before the arm arrives. It refuses to
# start, naming each thing, when the stops, the robot network's isolation, the disk, the house's frame and floor map or
# the operating area are not in order ([qDh]); then DimOS walks the robot along the route made from the house's own
# baseline recordings (sweep_route.py), recording it as a cataloged session (hk-data record), watching only: with no arm
# every step that needs one is skipped and the walk's report says which (tidy_module, HK_ARM=none). When the walk ends
# the recording is judged on the house's floor map as the robot judges a night (sim/house/real_nights.sh TONIGHT=...:
# the floor comparison at the launch setting, the light check on each candidate), and tonight's directory gets the
# robot's list and night/ with the morning log, the walk's report and log and the route ([oWO], [q66]), and the flags'
# review crops (real/tests/flag_crops.py); the recording's catalog entry names the night. Until the walk has ended
# ("done") everything is kept in a work directory beside the nights' root (<root>.work/<night>), so a refused, failed or
# crashed night leaves nothing in the root, where the scorer counts a night once it has its list ([oWO], [wTj]).
# Nothing here reads a sim truth file, the sim's model or a sim-only stream.
#
# ROBOT=sim rehearses it with DimOS's sim robot standing in for the dog (real/tests/night_standin.sh sets up the
# sim's world and passes its arguments in STANDIN_ARGS): the same steps, except that the dog's own checks (the stops,
# the robot network) are skipped and said, and the recording is cataloged as a sim one.
# Usage: real/night.sh
#   env: ROBOT (real; or sim)  HK_HOUSE (the house's run dir, default data/house/real)  AREA (the operating area's file;
#        needed on the robot)  HK_REAL_NIGHTS (the nights' root, default data/real_nights)  RADIUS (5: the route's reach
#        from the dock, m)  ROUTE_CLEAR (2)  LIGHT_CHECK (1: the launch setting's light check; 0 skips it)
#        WALK_MAX_MIN (120: the walk is stopped after this)  STANDIN_ARGS (ROBOT=sim: DimOS's sim arguments)
set -uo pipefail
HK=$(cd "$(dirname "$0")/.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}; PY=$DIMOS/.venv/bin/python
export PATH=$DIMOS/.venv/bin:$PATH PYTHONPATH=$HK/src:${PYTHONPATH:-}
ROBOT=${ROBOT:-real}; HOUSE=$(realpath -m "${HK_HOUSE:-$HK/data/house/real}"); export HK_HOUSE=$HOUSE
case "$ROBOT" in real|sim) ;; *) echo "ROBOT is real or sim, not '$ROBOT'"; exit 2;; esac
[ "$ROBOT" = sim ] && [ -z "${STANDIN_ARGS:-}" ] && { echo "ROBOT=sim needs the sim's world (STANDIN_ARGS): real/tests/night_standin.sh"; exit 2; }
AREA=${AREA:+$(realpath "$AREA")}

echo "== preflight ($ROBOT robot)"
$PY -m houndkeeper.real_night preflight --house "$HOUSE" --robot "$ROBOT" ${AREA:+--area "$AREA"} || exit 1
if pgrep -f "bin/dimos .*run unitree-go2" >/dev/null || pgrep -f "mujoco-room" >/dev/null; then
  echo "not starting the night: DimOS is already running (one robot session at a time)"; exit 1; fi
TN=$($PY -m houndkeeper.real_night tonight) || exit 1
[ -e "$TN/candidates.json" ] || [ -e "$TN/night" ] && { echo "not starting the night: tonight has already run ($TN)"; exit 1; }
W=$(dirname "$TN").work/$(basename "$TN")          # the night's work until its walk ends: never in the nights' root
[ -e "$W" ] && mv "$W" "$W.$(date +%H%M%S)"         # an earlier try this evening, kept aside
mkdir -p "$W/night"; exec > >(tee -a "$W/launch.log") 2>&1
WALKED=0
trap '[ "$WALKED" = 1 ] || echo "aborted before the walk ended: nothing in $(dirname "$TN"); see $W" | tee "$W/ABORTED"' EXIT
echo "tonight: $TN (until the walk ends: $W)"
B1=$(awk '$1==1{print $2}' "$HOUSE/recordings.txt"); B2=$(awk '$1==2{print $2}' "$HOUSE/recordings.txt")

echo "== the route, from the baselines' own recordings"
$PY "$HK/sim/house/sweep_route.py" --recordings "$HOUSE/recordings.txt" --night 3 --run "$HOUSE" --near dock \
  --radius "${RADIUS:-5}" --clear "${ROUTE_CLEAR:-2}" ${AREA:+--area "$AREA"} > "$W/night/route.json" 2> "$W/route.log" \
  || { echo "no route ($W/route.log)"; exit 1; }
SWEEP=$($PY -c "import json, sys; r = json.load(open(sys.argv[1])); assert r['waypoints']
print(json.dumps({'route': r['waypoints'], 'run_dir': sys.argv[2], 'night': 3}))" "$W/night/route.json" "$HOUSE") \
  || { echo "the route has no waypoints ($W/route.log)"; exit 1; }
echo "route: $($PY -c "import json, sys; r = json.load(open(sys.argv[1])); print(len(r['waypoints']), 'waypoints,', r['length_m'], 'm')" "$W/night/route.json")"

echo "== the walk: DimOS on the $ROBOT robot, recorded; no arm, watching only"
export HK_ARM=none HK_TIDY_WATCH_ONLY=1           # [jFy]: every step that needs the arm skipped, and reported
[ -n "$AREA" ] && export HK_AREA=$AREA
if [ "$ROBOT" = real ]; then
  [ -n "${ROBOT_IP:-}" ] || ROBOT_IP=$(sed -n 's/^ROBOT_IP=\([^ #]*\).*/\1/p' "$HK/real/.env" 2>/dev/null)   # never printed
  [ -n "${ROBOT_IP:-}" ] || { echo "no robot address (ROBOT_IP in real/.env)"; exit 1; }
  export ROBOT_IP; SESSION=(--real); WORLD=()
else SESSION=(); read -r -a WORLD <<< "$STANDIN_ARGS"; fi
cd "$DIMOS" || exit 1
hk-data record "${SESSION[@]}" --purpose "find-and-flag night $(basename "$TN")" --tag night --tag dog-alone -- \
  "${WORLD[@]}" run unitree-go2 mcp-server houndkeeper.tidy > "$W/night/dimos.log" 2>&1 &
REC_PID=$!
READY=0
for _ in $(seq 1 80); do
  kill -0 "$REC_PID" 2>/dev/null || break
  timeout 15 dimos mcp list-tools 2>/dev/null | grep -q tidy_start && { READY=1; break; }; sleep 3
done
if [ "$READY" != 1 ]; then
  echo "DimOS did not start ($W/night/dimos.log):"; grep -m5 -E "not starting|Error|Traceback" "$W/night/dimos.log"
  timeout 60 dimos stop >/dev/null 2>&1; wait "$REC_PID"; exit 1; fi
R=$(dimos mcp call tidy_start -j "$($PY -c 'import json, sys; print(json.dumps({"sweep": sys.argv[1]}))' "$SWEEP")" 2>&1 | tail -1)
echo "tidy_start: $R"
STATE=refused
if [[ "$R" != *refused* ]]; then
  T_END=$(( $(date +%s) + ${WALK_MAX_MIN:-120} * 60 )); T_START=$(( $(date +%s) + ${START_MAX_MIN:-5} * 60 ))
  while :; do
    S=$(timeout 20 dimos mcp call tidy_status 2>/dev/null | tail -1)
    [ -n "$S" ] && echo "$S" > "$W/night/walk.json"
    STATE=$($PY -c "import json, sys
t = sys.stdin.read()
try: print(json.loads(t[t.index('{'):])['state'])
except Exception: print('?')" <<< "$S")
    echo "$(date +%T) $STATE"
    case "$STATE" in done|DONE|failed|stopped) break;; esac
    [ "$STATE" = starting ] && [ "$(date +%s)" -ge "$T_START" ] && {
      echo "the robot never began its walk in ${START_MAX_MIN:-5} min (no pose or camera reached the tidy): stopped"; break; }
    [ "$(date +%s)" -ge "$T_END" ] && { echo "the walk ran past ${WALK_MAX_MIN:-120} min: stopped"; break; }
    kill -0 "$REC_PID" 2>/dev/null || { echo "DimOS ended during the walk"; break; }
    sleep 15
  done
fi
timeout 60 dimos stop >/dev/null 2>&1; wait "$REC_PID"   # the session catalogs the recording once DimOS stops
RID=$(sed -n 's/^cataloged: \([A-Za-z0-9._-]*\).*/\1/p' "$W/night/dimos.log" | tail -1)
[ -n "$RID" ] || { echo "no recording was cataloged ($W/night/dimos.log)"; exit 1; }
DB=$(hk-data path --numbers-only "$RID") && [ -f "$DB" ] || { echo "recording $RID not found"; exit 1; }
echo "walk: $STATE; recording $RID"
[ "$STATE" = refused ] && { echo "the tidy refused the walk: $R"; exit 1; }
case "$STATE" in done|DONE) ;; *) echo "the walk did not end ($STATE): not a night; recording $RID kept"; exit 1;; esac
mkdir -p "$TN/night" && cp -a "$W/night/." "$TN/night/" && cp "$W/launch.log" "$TN/" || { echo "could not move the night into $TN"; exit 1; }
WALKED=1; rm -rf "$W"; exec > >(tee -a "$TN/launch.log") 2>&1   # from here on the night is tonight's

echo "== tonight on the house's floor map: the floor comparison and the light check, the list, the morning log"
TONIGHT=$DB TONIGHT_DIR=$TN LIGHT_CHECK=${LIGHT_CHECK:-1} \
  TIDY_OPTS="--walk $TN/night/walk.json --mode find-and-flag --route route.json" \
  bash "$HK/sim/house/real_nights.sh" "$HOUSE" "$B1" "$B2" || { echo "tonight's judging failed: see above"; exit 1; }
hk-data note "$RID" "night: $TN" && hk-data tag "$RID" judged
echo "== the flags' review crops"                  # people-screened, only from a recording cleared for its frames ([Vd3])
nice -n 15 $PY "$HK/real/tests/flag_crops.py" "$TN" 2>&1 | grep -v Zenoh || echo "no review crops ($TN/review_crops): the night stands without them"
echo "night done: $TN (candidates.json, night/morning_log.html); recording $RID"
