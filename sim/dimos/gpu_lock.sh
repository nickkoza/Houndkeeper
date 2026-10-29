# Sourced by the scripts that run DimOS's sim: hold the GPU alone for the rest of the script ([jlr]; the Python side,
# for standalone sims, is src/houndkeeper/gpu_share.py). Re-entrant: a script run by one that holds it doesn't wait.
if [ -z "${HK_GPU_LOCKED:-}" ]; then
  _gl=$HOME/.cache/houndkeeper/gpu.lock; mkdir -p "$(dirname "$_gl")"
  echo $$ > "${_gl%.lock}.night-waiting"                     # announced first: new standalone sims hold off (the pid: a
                                                             # night killed while it waits leaves a stale one, gpu_share.py)
  exec 9>>"$_gl"
  flock -n 9 || { echo "GPU: waiting for standalone sims to finish"; flock 9; }
  rm -f "${_gl%.lock}.night-waiting"; export HK_GPU_LOCKED=1
fi
