#!/usr/bin/env bash
# The hold-out night, as sim-slice [bnO] fixes it: run once, on code recorded as it ran, headless, and reported as
# numbers only. A house never used (seed 80) on floors never drawn for tuning, toys never used for tuning, item seeds
# 1-3 over its two baseline nights. Nothing of it is looked at but the summary this prints (results only, never
# why, so no hold-out asset is used up by looking at it): the nights' logs stay on disk unread.
# Usage: sim/dimos/holdout.sh RUN_NUMBER [SEED]   (the settings are the consistency round's; see its queue). SEED: the
# hold-out house (80 is spent; 81 is the next, sim-house-nights [ISF]).
set -uo pipefail
cd "$(dirname "$0")/../.."
WS=${HK_WORKSPACE:-$(dirname "$(pwd)")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
N=${1:?"hold-out run number, e.g. 1"}; HS=${2:-81}
if [ "${RESUME:-0}" != 1 ] && ls data/eval/h${N}_s*_i*.score.json >/dev/null 2>&1; then
    echo "refusing: hold-out run $N already ran (a hold-out is run once)" >&2; exit 1
fi                                                    # RESUME=1: a run cut short (the machine stopped mid-night): the nights
                                                      # with a score stand, the others run again from their start, unseen
if [ "${RESUME:-0}" != 1 ]; then
COMMIT=$(git rev-parse --short HEAD)                  # what ran: the commit, and the uncommitted changes on top of it
git diff HEAD -- src sim tools params docs .railyard > "data/eval/h${N}.uncommitted.diff"
git ls-files --others --exclude-standard -- src sim tools params docs | tar czf "data/eval/h${N}.untracked.tgz" -T - 2>/dev/null
git -C "$DIMOS" diff > "data/eval/h${N}.dimos_fork.diff"   # the DimOS fork, which keeps its own repository
echo "hold-out run $N at commit $COMMIT plus $(grep -c '^diff --git' "data/eval/h${N}.uncommitted.diff") changed files, $(date '+%F %T')"
else COMMIT=$(git rev-parse --short HEAD); echo "hold-out run $N resumed $(date '+%F %T') (its first nights' code: h${N}.uncommitted.diff)"; fi
export DIMOS_SIM_HEADLESS=1 HK_FLOORS=heldout HK_ITEMS=heldout
export HK_STOP_GATE='{"present_area_cm2": 0, "gone_hits": 5, "unsure_stop_hits": 2}'
export HK_WHOLE_ITEM=1 HK_ARM_BLACK=1 HK_HIGH_CARRY=1 HK_BIN_EXIT=1 HK_ODOM_DELAY=1
export HK_WRIST_STEREO=1 HK_STEREO_FULL=1 HK_WRIST_LIGHT=1 HK_GRASP=ours   # the wrist rig of round 16 (waymark 0011 [A8K], 0010
                                                                         # [J7G]) and our grasp through the proposers (sim-grasp [WuR])
export HK_MUJOFIL_DIR=${HK_MUJOFIL_DIR:-$WS/build/mujofil-f177-site}
for i in 1 2 3; do
    [ -f "data/eval/h${N}_s${HS}_i$i.score.json" ] && { echo "night $i: scored already"; continue; }
    T0=$(date +%Y%m%d_%H%M%S)                         # the tidy run this night makes is named after its start, after this
    SEED=$HS ITEM_SEED=$i MODE=confirm bash sim/dimos/night_tidy.sh > data/eval/h${N}_s${HS}_i$i.log 2>&1; rc=$?
    D=$(ls -d data/runs/dimos_tidy/*/ 2>/dev/null | sort | tail -1)
    if [ -z "$D" ] || [ "$(basename "$D")" \< "$T0" ]; then   # none newer: the night failed before its tidy ran, and the
        echo "night $i: no tidy run, no score (rc=$rc)"; continue; fi   # newest run is an earlier night's, not this one's
    cp "${D}score.json" "data/eval/h${N}_s${HS}_i$i.score.json" 2>/dev/null || echo "night $i: no score (rc=$rc)"
    cp "${D}status.json" "data/eval/h${N}_s${HS}_i$i.status.json" 2>/dev/null   # the stages each toy reached, for the funnel
done
echo "$COMMIT dimos $(git -C "$DIMOS" rev-parse --short HEAD) mujofil $HK_MUJOFIL_DIR" > "data/eval/h${N}.commit"
python3 sim/dimos/round_summary.py "$N" --prefix h --results-only
RUN=sim/house/runs/nights_s$HS                          # the floor map's error against the actual floor ([nv5]): a number,
[ -f "$RUN/shape/corrections_1.npz" ] && (cd sim/house && "$DIMOS/.venv/bin/python" floor_fidelity.py "../../$RUN/mosaic_n1.npz" \
    --seed $HS --traj "../../$RUN/nights_s${HS}_n1.traj.csv" --corrections "../../$RUN/shape/corrections_1.npz" --error "../../$RUN" \
    --built "hold-out $N, house $HS" 2>&1 | tail -1)    # nothing of the house looked at
