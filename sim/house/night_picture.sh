# Sourced: one night's floor picture, tall layer and views from its recording, in odometry's frame, before the stitch
# puts the nights in the house's frame. The one recipe for it, shared by the sim's nights (sim/dimos/night_tidy.sh) and
# real ones (sim/house/real_nights.sh, [V35]); it reads only the recording and the run dir's earlier pictures.
# Defines MOSAIC_ARGS, MOSAIC_V and night_picture <run dir> <night> <memory.db> <log>; needs PY (dimos venv python);
# ROBOT_LAG, when set, is the frame lag the pictures and views are placed with.
MOSAIC_ARGS="--gain --dt auto"               # auto-exposed frames matched in brightness (no seams); each placed where the
                                              # camera was when it took it ([uKM] 2). Not yet in the house's lidar frame ([KkQ]):
                                              # its first-night map warped (nights 19-27 cm apart; odometry 20-27), 27 Sep
MOSAIC_V=7                                    # the picture builder's version: bump when floor_mosaic2 changes, so saved
                                              # pictures are rebuilt (2: each cell the mean of its pixels; 3: poses at their own time, 27 Sep;
                                              # 4: the matched picture brought back to the camera's own level; 5: the house's frame;
                                              # 6: back to odometry's frame, the builder's fixes kept; 7: with the night's views, 27 Sep)
_NP_SIM=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$_NP_SIM/lib/build.sh"               # build: each output removed first and required again ([SdA], [gI8])
source "$_NP_SIM/lib/clock.sh"               # clock_args: the clock the recording was stamped on, for the robot's lag ([55G])
night_picture() {   # night $2's floor picture from its recording $3 into run dir $1, the builder's output in log $4
  local run=$1 n=$2 db=$3 log=$4
  rm -f "$run/mosaic_n$n.args"                # the recipe's stamp: written again only once picture, tall layer and views are made
  rm -f "$run/stitch.args"                    # a baseline picture remade: the stitch must be redone over it
  [ "$n" = 1 ] && rm -f "$run/house_lidar.npy"   # night 1 founds the house's frame; later nights are placed on it
  local outs=("$run/mosaic_n$n.npz") corr=() clk
  clk=$(clock_args "$run" "$n")
  local lag=(); [ -n "${ROBOT_LAG:-}" ] && lag=(--lag-file "$ROBOT_LAG")   # the robot's own frame lag when the caller names
                                              # one (real_nights.sh); else each tool's default, the sim robot's
  [[ "$MOSAIC_ARGS" == *--house* ]] && { outs+=("$run/mosaic_n$n.corrections.npz"); corr=(--corrections "$run/mosaic_n$n.corrections.npz"); }   # the picture's frame
  build --log "$log" "${outs[@]}" -- $PY "$_NP_SIM/floor_mosaic2.py" "$db" "$run/mosaic_n$n.npz" $MOSAIC_ARGS $clk "${lag[@]}" \
    || { echo "night $n: floor picture failed"; return 1; }
  build --log "${log%.log}_tall.log" "$run/tall_n$n.npz" -- $PY "$_NP_SIM/tall_layer.py" "$db" "$run/tall_n$n.npz" "${corr[@]}" \
    || { echo "night $n: tall layer failed"; return 1; }
  build --log "${log%.log}_views.log" "$run/views_n$n.npz" -- \
    $PY "$_NP_SIM/baseline_views.py" "$run" "$n" "$db" --out "$run/views_n$n.npz" "${corr[@]}" $clk "${lag[@]}" \
    || { echo "night $n: views failed"; return 1; }   # the night's own views of the floor, for the sweep's comparison ([WFI])
  echo "$MOSAIC_ARGS v$MOSAIC_V" > "$run/mosaic_n$n.args"   # all three made: the night's picture may be reused
}
