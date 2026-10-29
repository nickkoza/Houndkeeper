#!/usr/bin/env bash
# The baseline nights' views on the near grid ([BW3], [Ot0]): 5 mm cells out to 1 m, painted with the stitch's mount, in
# its frame and on its relief, as the full-resolution round painted them; the launch setting's near layer compares the
# floor against them, live and in the replay. Remade only when the stitch is (views_near.args), so every stitched house
# has them: a live night without them ran with its near grid off (18 Oct 2026). CPU only. Run by stitch_baselines.sh.
# Usage: near_views.sh <run dir> <night 1 db> <night 2 db>   env: ROBOT_LAG (the robot's frame lag .json), RECORDING_CLOCK
set -euo pipefail
RUN=$1; D1=$2; D2=$3
HK=$(cd "$(dirname "$0")/../.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
SIM=$HK/sim/house; PY=$DIMOS/.venv/bin/python
source "$SIM/lib/build.sh"; source "$SIM/lib/clock.sh"
ROBOT_LAG=${ROBOT_LAG:-$HK/data/robot/frame_lag.json}
[ -f "$RUN/stitch.args" ] || { echo "near views: $RUN is not stitched"; exit 1; }
if [ "$(cat "$RUN/views_near.args" 2>/dev/null)" = "$(cat "$RUN/stitch.args")" ] && [ -s "$RUN/views_near_n1.npz" ] && [ -s "$RUN/views_near_n2.npz" ]; then
  echo "near views: already painted on this stitch"; exit 0; fi
rm -f "$RUN/views_near.args"                          # a failed run must not leave a stamp saying it's done
for k in 1 2; do db=$([ $k = 1 ] && echo "$D1" || echo "$D2")
  build_bg --cd "$SIM" --log "$RUN/views_near_n$k.log" "$RUN/views_near_n$k.npz" -- env OMP_NUM_THREADS=4 nice -n 10 \
    $PY baseline_views.py "$RUN" $k "$db" --stack 4 --every 8 --mount "$RUN/head_mount.json" \
    --corrections "$RUN/shape/corrections_$k.npz" --relief auto --lag-file "$ROBOT_LAG" $(clock_args "$RUN" $k) \
    --res 0.005 --rmax 1.0 --out "$RUN/views_near_n$k.npz"
done
build_wait || { echo "the near grid's views failed"; exit 1; }
cp "$RUN/stitch.args" "$RUN/views_near.args"; echo "near views: $RUN/views_near_n1.npz, views_near_n2.npz"
