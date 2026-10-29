#!/usr/bin/env bash
# Watch an unattended queue and exit, with a reason, the moment it needs attention (a crash is caught at
# once, not found after a long unattended wait): the sim crashed, nothing has logged for STALL_MIN minutes, the sim
# runs below MIN_SPEED x real time, or the queue's process is gone. Exits 0 when the queue finishes cleanly.
# Usage: watchdog.sh <queue pid> [STALL_MIN=12] [MIN_SPEED=0.2]
set -u
Q=$1; STALL_MIN=${2:-12}; MIN_SPEED=${3:-0.2}
HK=$(cd "$(dirname "$0")/../.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
DLOGS=$DIMOS/logs
newest() { find "$HK/data/eval" "$HK/data/runs" "$HK/sim/house/runs" "$DLOGS" -newermt "-${STALL_MIN} minutes" -type f 2>/dev/null | head -1; }
while true; do
  if ! kill -0 "$Q" 2>/dev/null; then echo "$(date +%T) the queue (pid $Q) has ended"; exit 0; fi
  logs=$(ls -t "$HK"/sim/house/runs/nights_s*/nights_s*_n*.sweep.log "$DLOGS"/*/main.jsonl 2>/dev/null | head -3)
  for l in $logs; do
    if [ "$(stat -c %Y "$l")" -gt $(( $(date +%s) - 300 )) ] && grep -q "\[mujoco\] Traceback" "$l"; then
      echo "$(date +%T) SIM CRASHED: $l"; grep -m1 -A12 "\[mujoco\] Traceback" "$l" | tail -3 | cut -c1-200; exit 1; fi
  done
  if [ -z "$(newest)" ]; then echo "$(date +%T) STALLED: nothing logged for ${STALL_MIN} min"; ps -eo pid,etime,args | grep -E "night_tidy|sweep_night|tidy_run|mujoco_process" | grep -v grep | cut -c1-120; exit 2; fi
  sp=""; slog=""                                 # the speed the newest log reports ($logs runs newest first: one grep over
  for l in $logs; do                             # all three and its last line was the oldest log's speed)
    sp=$(grep -h "SIM_SPEED" "$l" 2>/dev/null | tail -1 | grep -oE "sim/wall = [0-9.]+" | awk '{print $3}')
    [ -n "$sp" ] && { slog=$l; break; }
  done
  if [ -n "$sp" ] && awk -v s="$sp" -v m="$MIN_SPEED" 'BEGIN{exit !(s < m)}'; then
    last=$(grep -h "SIM_SPEED" "$slog" | tail -1 | cut -c1-20)
    echo "$(date +%T) SLOW: the sim at ${sp}x real time (under ${MIN_SPEED}; $last)"; grep -h "SIM_SPEED" "$slog" | tail -1 | cut -c60-220; exit 3; fi
  sleep 60
done
