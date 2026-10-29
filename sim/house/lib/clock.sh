# Sourced: the clock a night's recording was stamped on, as arguments for the tools that place frames with the robot's
# stored frame lag ([gJq], decision [55G]). RECORDING_CLOCK (robot or wall) when it is given: real_nights.sh gives
# robot, the real robot's own clock. Else the sim's path file of night <n> in run dir <run>, when the sweep wrote one
# there. Else nothing: the tool finds the run that lists the recording, and stops when none does.
clock_args() {   # clock_args <run dir> <night>
  if [ -n "${RECORDING_CLOCK:-}" ]; then echo "--clock $RECORDING_CLOCK"; return 0; fi
  local t; t="$1/$(basename "$1")_n$2.traj.csv"
  if [ -f "$t" ]; then echo "--traj $t"; fi
  return 0
}
