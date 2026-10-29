#!/bin/bash
# [Mj2] silence stops the robot, in sim: the Go2 with Cockpit on a local relay, driven from a headless browser
# that then falls silent (killed, hidden, frozen, closed); odometry must show it stopped within 0.6 s.
#
#   real/tests/deadman_sim.sh [OUT_DIR]        CASES="killed hidden" to run a subset; MAX_STOP=0.6
#
# Headless, one sim only: refuses to start if a MuJoCo sim is already running. Cleans up everything it started
# (dimos stop, then its process group; the odometry logger; any browser) and exits 0 only if every case passed.
set -u
HK=$(cd "$(dirname "$0")/../.." && pwd)
WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${DIMOS:-${HK_DIMOS:-$WS/dimos}}; DIMOS=$(cd "$DIMOS" && pwd)   # decision [cy4]
PY=$DIMOS/.venv/bin/python
T=$HK/real/tests
OUT=${1:-/tmp/hk_deadman_$(date +%Y%m%d_%H%M%S)}; mkdir -p "$OUT"
# what is tested: the phone cockpit as shipped (its 200 ms deadman); BLUEPRINT=unitree-go2-cockpit for DimOS's stock one
BLUEPRINT=${BLUEPRINT:-"unitree-go2 houndkeeper.phone-cockpit --extensions-dir $(cd "$(dirname "$0")/../.." && pwd)/cockpit/extensions --open-browser false"}
CASES=${CASES:-"hidden frozen closed killed"}
MAX_STOP=${MAX_STOP:-0.6}
ROOM=${ROOM:-nights_s95_n1}
WARM_HZ=${WARM_HZ:-12}    # deadman.py counts a case only at 11 Hz or more (15 Hz is real time)
URL=http://127.0.0.1:7780/
export PATH=$HOME/.local/bin:$DIMOS/.venv/bin:$PATH

if pgrep -f 'mujoco-room' >/dev/null; then
  echo "a MuJoCo sim is already running (pgrep -f mujoco-room); one sim at a time" >&2; exit 2
fi
[ -d "$T/node_modules/playwright" ] || { echo "no Playwright: see real/tests/README.md" >&2; exit 2; }
read -r load _ </proc/loadavg
awk -v l="$load" -v n="$(nproc)" 'BEGIN{exit !(l > n)}' && \
  echo "warning: load $load on $(nproc) cores: the sim may run slower than real time (the check then says INVALID)" >&2
command -v deno >/dev/null || { echo "no deno on PATH (the relay needs it)" >&2; exit 2; }

SIM_PID=; LOG_PID=; NODE_PID=
cleanup() {
  [ -n "$NODE_PID" ] && kill "$NODE_PID" 2>/dev/null
  [ -n "$LOG_PID" ] && kill "$LOG_PID" 2>/dev/null
  if [ -n "$SIM_PID" ]; then
    (cd "$DIMOS" && timeout 60 dimos stop >>"$OUT/stop.log" 2>&1)
    for _ in $(seq 40); do kill -0 "$SIM_PID" 2>/dev/null || break; sleep 0.5; done
    kill -TERM -- "-$SIM_PID" 2>/dev/null; sleep 2; kill -KILL -- "-$SIM_PID" 2>/dev/null
  fi
  [ -n "$LOG_PID" ] && { sleep 0.5; kill -KILL "$LOG_PID" 2>/dev/null; }
  kill_browsers
  left=$(pgrep -g "${SIM_PID:-0}" 2>/dev/null; for b in $(browser_pids); do grep -qa ms-playwright "/proc/$b/cmdline" 2>/dev/null && echo "$b"; done)
  [ -n "$left" ] && echo "left running after cleanup:" $left >&2
}
# our browsers only (Playwright launches each as its own process group); other Chromiums on the machine are not ours
browser_pids() { grep -o '"browser_pid":[0-9]*' "$OUT/events.jsonl" 2>/dev/null | cut -d: -f2; }
kill_browsers() {
  for b in $(browser_pids); do grep -qa ms-playwright "/proc/$b/cmdline" 2>/dev/null && kill -KILL -- "-$b"; done
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# the sim with Cockpit, as the hand test ran it; setsid makes it its own process group for cleanup
eval "$(cd "$HK/sim/dimos" && "$PY" -c "import env; print(env.shell_exports())")"
export DIMOS_SIM_HEADLESS=1 PYTHONPATH=$HK/src:${PYTHONPATH:-}; unset OPENAI_API_KEY
START=$("$PY" -c "import json,sys;p=json.load(open(sys.argv[1]))['start_pos'];print(f'{p[0]}, {p[1]}')" \
  "$HK/sim/house/runs/${ROOM%_n*}/$ROOM.truth.json")
(cd "$DIMOS" && dimos stop >/dev/null 2>&1)
cd "$DIMOS"
setsid dimos --simulation mujoco --mujoco-room "$ROOM" --mujoco-start-pos "$START" \
  run --local-relay $BLUEPRINT >"$OUT/sim.log" 2>&1 &
SIM_PID=$!
echo "sim pid $SIM_PID, log $OUT/sim.log"

for _ in $(seq 300); do
  grep -q 'local relay ready' "$OUT/sim.log" && break
  kill -0 "$SIM_PID" 2>/dev/null || { echo "sim exited early" >&2; tail -20 "$OUT/sim.log" >&2; exit 1; }
  sleep 1
done
grep -q 'local relay ready' "$OUT/sim.log" || { echo "relay not ready in 300 s" >&2; exit 1; }

"$PY" "$T/deadman.py" log "$OUT/odom.csv" >"$OUT/odomlog.log" 2>&1 &
LOG_PID=$!
for _ in $(seq 120); do [ -s "$OUT/odom.csv" ] && break; sleep 1; done
[ -s "$OUT/odom.csv" ] || { echo "no odometry in 120 s" >&2; exit 1; }
# the sim runs slow for its first ~45 s (maps warming up): wait until odometry keeps real-time pace
odom_hz() { awk -F, -v now="$(date +%s.%N)" '$1 > now - 5 {n++} END {printf "%.1f", n / 5}' "$OUT/odom.csv"; }
for _ in $(seq 36); do
  sleep 5; hz=$(odom_hz)
  awk -v h="$hz" -v m="$WARM_HZ" 'BEGIN{exit !(h >= m)}' && break
done
echo "odometry at $hz Hz before the first case (wanted $WARM_HZ)"

: >"$OUT/events.jsonl"
for c in $CASES; do
  node "$T/cockpit_silence.mjs" "$c" "$URL" >>"$OUT/events.jsonl" 2>>"$OUT/browser.log" &
  NODE_PID=$!; wait "$NODE_PID"; NODE_PID=
  kill_browsers
  tail -1 "$OUT/events.jsonl"
  sleep 3   # odometry for the hold after a kill; the robot at rest before the next case
done

kill "$LOG_PID"; wait "$LOG_PID" 2>/dev/null; LOG_PID=
"$PY" "$T/deadman.py" check "$OUT/odom.csv" "$OUT/events.jsonl" --max-stop "$MAX_STOP" --sim-log "$OUT/sim.log" | tee "$OUT/result.txt"
rc=${PIPESTATUS[0]}
exit "$rc"
