#!/usr/bin/env bash
# The twin of each tuning house built and scored against its truth ([JLx], [POG]): house_twin.py from the run dir's
# outputs (night 1 and 2's floor pictures; TRIMMED=<dir> takes <dir>/s<seed>/mosaic_n<k>_tm.npz instead, the trimmed
# mean of [evl], as [POG] asks), then twin_score.py beside night 1's own picture(s). CPU only: no sim, no GPU.
# Writes data/twin/bench/<date>/s<seed>/ (the twin) and score_s<seed>.json; one house at a time.
# Usage: twin_bench.sh <seed> [<seed> ...]   env: TRIMMED (dir of trimmed pictures) OUT (data/twin/bench/<date>)
set -u
HK=$(cd "$(dirname "$0")/../.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
SIM=$HK/sim/house; PY=$DIMOS/.venv/bin/python
OUT=${OUT:-$HK/data/twin/bench/$(date +%Y%m%d)}; mkdir -p "$OUT"; export PYTHONPATH=$HK/src:${PYTHONPATH:-}
cd "$SIM"
for s in "$@"; do RUN=$SIM/runs/nights_s$s; pics=(); score=("$RUN/mosaic_n1.npz")
  if [ -n "${TRIMMED:-}" ]; then pics=(--pictures "$TRIMMED/s$s/mosaic_n1_tm.npz" "$TRIMMED/s$s/mosaic_n2_tm.npz"); score+=("$TRIMMED/s$s/mosaic_n1_tm.npz"); fi
  $PY house_twin.py "$RUN" --name "twin_s$s" --out "$OUT/s$s" "${pics[@]}" 2>&1 | grep -v Zenoh | tail -1 | cut -c1-160
  $PY twin_score.py "$OUT/s$s" --seed "$s" --run "$RUN" --picture "${score[@]}" --json "$OUT/score_s$s.json" 2>&1 | grep -v Zenoh | tail -1 | cut -c1-200
done
