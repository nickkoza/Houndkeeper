#!/usr/bin/env bash
# The baseline nights' floor map from recordings alone ([V35], [CCS]), and a tidy night on it ([Lzt]): what
# night_tidy.sh MODE=baselines builds, from one or two DimOS recordings (memory.db) with no sim truth, no house plan and
# no sweep: each night's floor picture, tall layer and views (night_picture.sh), then with two nights the house's frame
# and shape, the stitched floor and camera mount (stitch_baselines.sh), and the two nights' disagreement on the floor they
# both saw (floor_agreement.py). With TONIGHT, a third recording is then judged on that map as the robot judges a night:
# the floor comparison at the launch setting ([tTD]'s stage 1, near grid included) gives tonight's candidates, the light
# check runs on them when there is one (CHECK_CMD, a hook), and the list becomes a night the morning log and the phone
# cockpit read (real_tidy.py, morning_log.py). The same scripts as the sim's nights; nothing here reads a truth file,
# a sim run, the sim's render stamp or a sim-only topic. The floor map error ([nv5]) needs the sim's own floor and path,
# so it is skipped and says so. CPU only (no sim, no GPU lock).
# Usage: real_nights.sh <run dir> <night 1 memory.db> [<night 2 memory.db>]
#   env: TONIGHT (a tidy night's memory.db, judged on the two baselines)  TONIGHT_DIR (where tonight's list and night/
#        go; default <run dir>/tonight)  LIGHT_CHECK=1 (the launch setting's own light check on each candidate, [Zaf];
#        default off)  TIDY_OPTS (further real_tidy.py options)  CHECK_CMD (another light check: run as
#        CHECK_CMD <decisions.json> <memory.db> <run dir> <out check.json>, its JSON {candidate id: {keep, score}}; unset:
#        the candidates go unchecked, and the list says so)  TONIGHT_OPTS (further confirm_stream.py options, e.g. its
#        own --check)  ROBOT_MOUNT, ROBOT_LAG (the head camera's mount and frame lag:
#        default the real robot's own, data/robot/real/; no mount yet: the first stitch calibrates one from night 1; no
#        lag yet: the sim robot's, said, until real/calibration/timing.md is run)  FORCE=1 (build despite the preflight)
set -uo pipefail
[ $# -ge 2 ] && [ $# -le 3 ] || { sed -n 12,21p "$0" | sed 's/^# \{0,1\}//'; exit 2; }
RUN=$(realpath -m "$1"); D1=$(realpath "$2"); D2=${3:+$(realpath "$3")}; T=${TONIGHT:+$(realpath "$TONIGHT")}
HK=$(cd "$(dirname "$0")/../.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
SIM=$HK/sim/house; PY=$DIMOS/.venv/bin/python
export PYTHONPATH=$HK/src:${PYTHONPATH:-}
export RECORDING_CLOCK=robot                   # the real robot's recordings are stamped on its own clock, the wall clock
                                               # ([55G]): the robot clock's stored lag, measured on day one (timing.md)
case "$RUN/" in "$SIM/runs/"*) echo "refusing to write into the sim's own runs ($RUN): give a run dir of its own"; exit 2;; esac
for d in "$D1" ${D2:+"$D2"} ${T:+"$T"}; do [ -f "$d" ] || { echo "no recording: $d"; exit 2; }; done
[ -z "$T" ] || [ -n "$D2" ] || { echo "TONIGHT is judged on two baseline nights: give both"; exit 2; }
mkdir -p "$RUN"
if ls "$RUN"/nights_s*.truth.json "$RUN"/*.traj.csv >/dev/null 2>&1; then   # a real run has none; their presence would
  echo "$RUN holds sim truth files: not a real night's run dir"; exit 2; fi  # let the sim-only splits in unnoticed
REAL_LAG=$HK/data/robot/real/frame_lag.json
if [ -z "${ROBOT_LAG:-}" ]; then
  if [ -f "$REAL_LAG" ]; then ROBOT_LAG=$REAL_LAG
  else ROBOT_LAG=$HK/data/robot/frame_lag.json
    echo "frame lag: the real robot has none yet (real/calibration/timing.md); the sim robot's is used: $ROBOT_LAG"; fi
fi
export ROBOT_LAG; echo "lag $ROBOT_LAG" > "$RUN/calibration_used.txt"
echo "== preflight"                            # the streams and transforms each step reads, and the floor grid's reach
(cd "$SIM" && set -o pipefail && $PY real_nights.py "$D1" ${D2:+"$D2"} ${T:+"$T"} 2>&1 | grep -v Zenoh) | tee "$RUN/preflight.txt"
[ "${PIPESTATUS[0]}" = 0 ] || [ "${FORCE:-0}" = 1 ] || { echo "preflight: a step could not run (FORCE=1 tries anyway)"; exit 1; }
PREV_RECS=$(cat "$RUN/recordings.txt" 2>/dev/null || true)
{ echo "1 $D1"; [ -n "$D2" ] && echo "2 $D2"; } > "$RUN/recordings.txt"   # "<night> <db>", as the sim's seed_recordings.txt
source "$SIM/night_picture.sh"
for n in 1 2; do db=$([ $n = 1 ] && echo "$D1" || echo "$D2"); [ -n "$db" ] || continue
  if [ "$(cat "$RUN/mosaic_n$n.args" 2>/dev/null)" = "$MOSAIC_ARGS v$MOSAIC_V" ] && grep -qxF "$n $db" <<< "$PREV_RECS"; then
    echo "== night $n: its floor picture, tall layer and views already made from this recording"; continue; fi
  echo "== night $n: floor picture, tall layer, views"
  night_picture "$RUN" $n "$db" "$RUN/picture_n$n.log" || exit 1
done
if [ -z "$D2" ]; then
  echo "one night: the floor picture, tall layer and views in odometry's frame; the house's frame and shape, the stitch"
  echo "and the nights' disagreement need a second night"; exit 0; fi
echo "== stitch: house frame, house shape, mount, pictures in the house's frame"
export ROBOT_MOUNT=${ROBOT_MOUNT:-$HK/data/robot/real/head_mount.json}
bash "$HK/sim/dimos/stitch_baselines.sh" "$RUN" "$D1" "$D2" || { echo "stitching the nights failed"; exit 1; }
echo "== the nights' disagreement"
source "$SIM/lib/build.sh"                     # removed first and required again ([SdA], [gI8])
if [ "$(cat "$RUN/agreement.args" 2>/dev/null)" != "$(cat "$RUN/stitch.args")" ]; then
  build --cd "$SIM" "$RUN/agreement.json" "$RUN/agreement.png" -- $PY floor_agreement.py "$RUN" --json "$RUN/agreement.json" --png "$RUN/agreement.png" \
    || { echo "the nights' disagreement failed"; exit 1; }
  cp "$RUN/stitch.args" "$RUN/agreement.args"
else echo "already measured on this stitch"; fi
echo "floor map error ([nv5]): sim only, it needs the sim's own floor and true path; skipped for recordings without them"
echo "done: $RUN (mosaic_n1/2.npz, tall_n1/2.npz, views_n1/2.npz, frame/, shape/, stitch/, head_mount.json, house_lidar.npy, views_near_n1/2.npz, agreement.json)"
[ -n "$T" ] || exit 0

# ---- tonight ([Lzt]) ----
# The launch setting ([tTD]: confirm_stream.py's SETTINGS["launch"], the one the live tidy reads too): the floor
# comparison loosened at slack 1 with the 5 mm near layer, then the light check on each candidate when LIGHT_CHECK=1.
LAUNCH=(--setting launch); [ "${LIGHT_CHECK:-0}" = 1 ] || LAUNCH+=(--no-check)
TN=${TONIGHT_DIR:-$RUN/tonight}; mkdir -p "$TN"
echo "== the baselines' views on the near grid: stitch_baselines.sh painted them (near_views.sh, 5 mm, out to 1 m)"
for k in 1 2; do [ -s "$RUN/views_near_n$k.npz" ] || { echo "no near views for night $k: the stitch did not finish"; exit 1; }; done
echo "== tonight: the floor comparison ($T)"
build --cd "$SIM" --log "$TN/confirm.log" "$TN/decisions.json" -- $PY confirm_stream.py --run-dir "$RUN" --night 3 --db "$T" \
  --clock "$RECORDING_CLOCK" --lag-file "$ROBOT_LAG" --mount "$RUN/head_mount.json" --views "$RUN/views_n1.npz" "$RUN/views_n2.npz" \
  "${LAUNCH[@]}" ${TONIGHT_OPTS:-} --near-views "$RUN/views_near_n1.npz" "$RUN/views_near_n2.npz" --out "$TN/decisions.json" \
  || { echo "tonight's comparison failed ($TN/confirm.log)"; exit 1; }
CHECK=()
if [ -n "${CHECK_CMD:-}" ]; then               # the light check on each candidate ([tTD]'s second stage), when there is one
  echo "== tonight: the light check"
  $CHECK_CMD "$TN/decisions.json" "$T" "$RUN" "$TN/check.json" > "$TN/check.log" 2>&1 && [ -f "$TN/check.json" ] \
    || { echo "the light check failed ($TN/check.log)"; exit 1; }
  CHECK=(--check "$TN/check.json")
fi
echo "== tonight: the robot's list, the morning log and the phone's page"
$PY "$SIM/real_tidy.py" "$TN/decisions.json" "$T" "$TN" "${CHECK[@]}" ${TIDY_OPTS:-} 2>&1 | grep -v Zenoh || { echo "the list failed"; exit 1; }
$PY "$HK/src/houndkeeper/morning_log.py" "$TN/night" > "$TN/morning_log.out" 2>&1 \
  && echo "morning log: $TN/night/morning_log.html" || { echo "no morning log ($TN/morning_log.out)"; exit 1; }
echo "tonight done: $TN (decisions.json, candidates.json, night/)"
