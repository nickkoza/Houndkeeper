#!/usr/bin/env bash
# The baseline nights' floor map ([WFI], [KkQ], [FHA], principle [fXD]): the head camera's mount and frame lag (the
# robot's own calibration, data/robot/head_mount.json and frame_lag.json, [gJq]; this house's stitch solves a mount
# only as a check), then the map's geometry: DimOS's pose graph over both nights' lidar (house_frame.py), straightened by
# every keyframe pair's lidar match (house_shape.py); each night's picture (15 views a cell, the darker quarter), tall
# layer and views laid down in that frame; the house's lidar map and a manifest (floor_frame.json) for placing tonight
# on it (tonight_frame.py). Since v8 ([yFg]) the floor's relief is part of the house's map: solved from the views in the
# house's frame painted on the plane with the robot's calibration (floor_selfcal.py relief), then every picture and view
# set painted on it; the same views give the nightly check of the robot's calibration (floor_selfcal.py check, [9tQ]:
# calibration_check.json, a report, never a write). Every picture and view set carries the calibration it was painted
# with ([Q9k]). Then the near grid's views (near_views.sh, [BW3]), on both paths: a stitch made before them gains them. Run by night_tidy.sh once both baselines exist; skipped when the stamp says it's done for
# these recordings. Reads only the recordings, never the sim's truth or plan, so real nights stitch too ([V35]).
# Usage: stitch_baselines.sh <run dir> <night 1 db> <night 2 db>   env: ROBOT_MOUNT, ROBOT_LAG (the robot's calibration .json)
set -euo pipefail
RUN=$1; D1=$2; D2=$3
HK=$(cd "$(dirname "$0")/../.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
SIM=$HK/sim/house; PY=$DIMOS/.venv/bin/python
source "$SIM/lib/build.sh"                            # every output below removed first and required again ([SdA], [gI8])
source "$SIM/lib/clock.sh"                            # clock_args: each night's recording's clock, whose stored lag is used ([55G])
export PYTHONPATH=$HK/src:${PYTHONPATH:-}
ROBOT_MOUNT=${ROBOT_MOUNT:-$HK/data/robot/head_mount.json}   # the robot's calibration ([gJq]): read here, never written;
ROBOT_LAG=${ROBOT_LAG:-$HK/data/robot/frame_lag.json}         # calibrate_robot_mount.py and calibrate_lag.py write them
sha() { if [ -f "$1" ]; then sha256sum "$1" | cut -c1-12; else echo none; fi; }
# A house's own pose-graph settings over house_frame.py's (key=value ...), from FRAME_PGO or the run's frame_pgo.args,
# kept in the stamp: a frame the gate refused at the defaults, repaired and recorded (decision [myl]: house 72)
FRAME_PGO=${FRAME_PGO:-$(cat "$RUN/frame_pgo.args" 2>/dev/null || true)}
MAP_VERSION=v9                                        # = houndkeeper.calibrate.MAP_VERSION: older maps are refused ([dmT])
STAMP="$MAP_VERSION ${FLOOR_CAMERA:-head} $D1 $D2 mount=$(sha "$ROBOT_MOUNT") lag=$(sha "$ROBOT_LAG") relief=floor_selfcal3 loops=verified clock=$(clock_args "$RUN" 1)|$(clock_args "$RUN" 2)${FRAME_PGO:+ pgo=$FRAME_PGO}"
                                                      # v7: the robot's calibration, by hash, and each night's clock
                                                      # ([55G], 3 Oct); pgo= only for a house with its own settings;
                                                      # v8: the floor's relief solved and painted on, the products
                                                      # stamped ([yFg], [Q9k], 7 Oct); v9: DimOS's loops verified
                                                      # before they enter the frame, one thread ([dmT], [fxk], 11 Oct)
gate() {   # the frame's gate ([Tu2], decision [myl]): a night the frame turns or moves far from its own odometry (house
  # 72, 3 Oct: -90 deg) stops here, before any picture, view or lidar map is built in it and read by the live tidy;
  # a stitch made before the gate is judged too, each time it is reused
  build --show "house frame gate|REFUSED" "$RUN/shape/gate.json" -- $PY "$SIM/frame_gate.py" "$RUN/shape" --out "$RUN/shape/gate.json" || {
    echo "the house's frame was refused: a night placed far from its own odometry; nothing is built in it"; exit 1; }
}
near() {   # the near grid's views on this stitch ([BW3], [Ot0]): every stitched house has them, made once per stitch
  ROBOT_LAG=$ROBOT_LAG bash "$HK/sim/dimos/near_views.sh" "$RUN" "$D1" "$D2" || exit 1
}
if [ "$(cat "$RUN/stitch.args" 2>/dev/null)" = "$STAMP" ] && [ -f "$RUN/head_mount.json" ]; then
  gate; near; echo "baselines already stitched for these recordings"; exit 0; fi
rm -f "$RUN/stitch.args"                              # a failed run must not leave a stamp saying it's done
cd "$SIM"
views() {   # views() <mount json> [corrections dir [out name [relief]]]: both nights' views, stacked, painted with that
  # mount (in that frame; on the house's relief there, unless relief is none)
  local m=$1 cdir=${2:-} name=${3:-views} rel=${4:-auto}
  for k in 1 2; do db=$([ $k = 1 ] && echo "$D1" || echo "$D2"); c=()
    [ -n "$cdir" ] && c=(--corrections "$cdir/corrections_$k.npz")
    build_bg "$RUN/${name}_n$k.npz" -- $PY baseline_views.py "$RUN" $k "$db" --stack 4 --mount "$m" "${c[@]}" --relief "$rel" --lag-file "$ROBOT_LAG" $(clock_args "$RUN" $k) --out "$RUN/${name}_n$k.npz"
  done
  build_wait || { echo "the baselines' views failed"; exit 1; }
}
# The mount belongs to the robot, not the house ([gJq]; the real robot keeps its own, real_nights.sh, [V35]). This house's
# stitch still solves one, kept as a check (stitch_mount.json); only calibrate_robot_mount.py, over tuning houses, ever
# writes the robot's. A robot with none yet is calibrated from this night for this run only.
if [ -f "$ROBOT_MOUNT" ]; then cp "$ROBOT_MOUNT" "$RUN/head_mount_start.json"; echo "mount: the robot's ($(sha "$ROBOT_MOUNT"))"
else build "$RUN/head_mount_start.json" -- $PY calibrate_mount.py "$D1" --lag-file "$ROBOT_LAG" $(clock_args "$RUN" 1) --out "$RUN/head_mount_start.json"
  echo "mount: the robot has none; this run's own from night 1 (calibrate_robot_mount.py sets the robot's)"; fi
views "$RUN/head_mount_start.json"
rm -f "$RUN/head_mount.json" "$RUN/stitch_mount.json" "$RUN/calibration.json" "$RUN/relief.npy" "$RUN/relief.json"   # never an
                                                      # earlier run's
build --clear "$RUN/stitch" --show "locked|solve 4|mount corrected|mount correction refused" "$RUN/stitch/summary.json" -- \
  $PY stitch_floor.py "$RUN" --out "$RUN/stitch" --mount "$RUN/head_mount_start.json" --mount-out "$RUN/stitch_mount.json"
cp "$RUN/head_mount_start.json" "$RUN/head_mount.json"   # the mount every picture and the live tidy use: the robot's
# The floor map's geometry ([KkQ], [FHA]; 1 Oct 2026): the pose graph over both nights' lidar (house_frame.py), its
# shape straightened by every keyframe pair's lidar match (house_shape.py), and the pictures, tall layers and views laid
# down there: 15 views a cell, the darker quarter taken (glints only add light). On the six tuning houses this took the
# floor map error from 3.4% to 1.0% (mean) and the walk-through from 72% to 85% of toys ([nv5], sim-house-nights [ISF]).
# Each loop DimOS proposes is verified by the robot's own scans before it enters the frame ([dmT], [fxk]); DimOS on one
# thread, so the same recordings give the same frame (Open3D's parallel ICP sums differ run to run: house 71, 8 Oct)
build --clear "$RUN/frame" --show "house frame:" "$RUN/frame/corrections_1.npz" "$RUN/frame/corrections_2.npz" "$RUN/frame/verify.json" -- \
  env OMP_NUM_THREADS=1 $PY house_frame.py "$D1" "$D2" --out "$RUN/frame" --verify lidar ${FRAME_PGO:+--pgo $FRAME_PGO} || { echo "the house's frame failed"; exit 1; }
build --clear "$RUN/shape" --show "pass 4|house shape:" "$RUN/shape/corrections_1.npz" "$RUN/shape/corrections_2.npz" "$RUN/shape/settle.npz" -- \
  $PY house_shape.py "$D1" "$D2" --out "$RUN/shape" --init "$RUN/frame" || { echo "the house's shape solve failed"; exit 1; }
gate
# The floor's relief ([yFg]): both nights' views in the house's frame, on the plane, with the robot's calibration; the
# relief solved on them with that calibration held (three rounds), and the same views' free solve as the nightly check
views "$RUN/head_mount.json" "$RUN/shape" views_flat none
build_bg --log "$RUN/relief.log" --show "floor relief:" "$RUN/relief.npy" "$RUN/relief.json" -- $PY floor_selfcal.py relief \
  "$RUN/views_flat_n1.npz" "$RUN/views_flat_n2.npz" --frame "$RUN/shape/corrections_1.npz" "$RUN/shape/corrections_2.npz" --out "$RUN/relief.npy"
build_bg --log "$RUN/calibration_check.log" --show "calibration check:" "$RUN/calibration_check.json" -- $PY floor_selfcal.py check \
  "$RUN/views_flat_n1.npz" "$RUN/views_flat_n2.npz" --out "$RUN/calibration_check.json"
build_wait || { echo "the house's floor relief or the calibration check failed"; exit 1; }
printf '{"mount": "%s", "mount_sha": "%s", "lag": "%s", "lag_sha": "%s", "relief": "%s", "relief_sha": "%s"}\n' "$ROBOT_MOUNT" \
  "$(sha "$ROBOT_MOUNT")" "$ROBOT_LAG" "$(sha "$ROBOT_LAG")" "$RUN/relief.npy" "$(sha "$RUN/relief.npy")" > "$RUN/calibration.json"   # which
                                                      # calibration this run used
for k in 1 2; do db=$([ $k = 1 ] && echo "$D1" || echo "$D2"); C="$RUN/shape/corrections_$k.npz"
  build_bg --log "$RUN/mosaic_n$k.log" "$RUN/mosaic_n$k.npz" -- $PY floor_mosaic2.py "$db" "$RUN/mosaic_n$k.npz" --gain --dt robot --lag-file "$ROBOT_LAG" $(clock_args "$RUN" $k) \
    --mount "$RUN/head_mount.json" --views 15 --pct 25 --camera "${FLOOR_CAMERA:-head}" --corrections "$C"
  build_bg "$RUN/tall_n$k.npz" -- $PY tall_layer.py "$db" "$RUN/tall_n$k.npz" --corrections "$C"
done
build_wait || { echo "the floor pictures or tall layers in the house's frame failed"; exit 1; }
views "$RUN/head_mount.json" "$RUN/shape"             # the confirmer's views, in the same frame, on the relief
rm -f "$RUN/house_lidar.npy" "$RUN/floor_frame.json"
$PY - "$RUN" "$D1" "$D2" <<'PYEOF'
import json, sys, numpy as np
from pathlib import Path
run, d1, d2 = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
z = np.load(run / "shape" / "settle.npz"); C = z["passes"][-1]; st = z["pts_start"]
P = np.vstack([z["pts"][st[k]:st[k + 1]] @ C[k][:2, :2].T + C[k][:2, 2] for k in range(len(C))])
key = np.floor(P / 0.05).astype(np.int64); _, i = np.unique(key[:, 0] * 1000003 + key[:, 1], return_index=True)
np.save(run / "house_lidar.npy", P[i])                # the house's lidar map in the floor map's frame: tonight is placed on it
json.dump({"dbs": [d1, d2], "corrections": [str(run / "shape" / f"corrections_{k}.npz") for k in (1, 2)]},   # (live:
          open(run / "floor_frame.json", "w"), indent=1)                                                      # tidy_module;
print(f"house lidar map: {len(i)} points -> {run / 'house_lidar.npy'}")                                         # bench:
PYEOF
echo "$STAMP" > "$RUN/stitch.args"                    # tonight_frame.py)
near
