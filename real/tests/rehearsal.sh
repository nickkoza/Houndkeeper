#!/usr/bin/env bash
# The real-data rehearsal ([Lzt], [vO9]): a sim house's baseline nights and one tidy night, their recordings copied
# without truth (memory.db only: no run directory, no path file, no sim marker) into a directory of their own, carried
# through the real robot's path (sim/house/real_nights.sh with TONIGHT: floor map, tonight's candidates, the light check
# when CHECK_CMD is given, the morning log), then scored two ways (rehearsal_score.py: the sim's scorer beside the sim
# run's own replay of that night, and the user's protocol with the truth standing in for the user). CPU only, niced.
# Usage: rehearsal.sh [house] [toy set]   (default house 70, toy set 1: s70_t1_hr)
#   env: OUT (default <workspace>/build/rehearsal)  REFERENCE (a floor_diag replay directory holding the sim run's own
#        replay of the night; default the full-resolution round's near choice)  CHECK_CMD (passed on to real_nights.sh)
set -uo pipefail
H=${1:-70}; TS=${2:-1}; NID=s${H}_t${TS}_hr
case " 78 80 81 $(seq -s ' ' 100 119) " in *" $H "*) echo "house $H is a hold-out: never rehearsed"; exit 2;; esac
HK=$(cd "$(dirname "$0")/../.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}; DIMOS=${HK_DIMOS:-$WS/dimos}   # the workspace: decision [cy4]
PY=$DIMOS/.venv/bin/python
OUT=${OUT:-$WS/build/rehearsal}/s$H; REC=$OUT/rec; RUN=$OUT/run
REFERENCE=${REFERENCE:-$WS/build/hr/r11/v/F_sl1_l3_N}
[ -f "$REFERENCE/${NID}_loose1.json" ] || REFERENCE=$WS/build/hr/r11/v/M_sl1_l3_N
export OMP_NUM_THREADS=${OMP_NUM_THREADS:-2} CUDA_VISIBLE_DEVICES=
BENCH=$HK/data/bench/houses_hr
SEED_RECS=$BENCH/nights_s$H/seed_recordings.txt
B1=$(awk '$1==1{print $2}' "$SEED_RECS" 2>/dev/null); B2=$(awk '$1==2{print $2}' "$SEED_RECS" 2>/dev/null)
TN=$($PY -c "import json,sys; print(next(e['db'] for e in json.load(open(sys.argv[1])) if e['seed']==int(sys.argv[2]) and e['toy_set']==int(sys.argv[3])))" \
  "$BENCH/nights.json" "$H" "$TS" 2>/dev/null)
[ -f "$B1" ] && [ -f "$B2" ] && [ -f "$TN" ] || { echo "house $H toy set $TS: its recordings were not found ($SEED_RECS, $BENCH/nights.json)"; exit 1; }
mkdir -p "$REC/base1" "$REC/base2" "$REC/tonight"
for p in "base1 $B1" "base2 $B2" "tonight $TN"; do set -- $p   # memory.db only: what a real recording is
  [ -f "$REC/$1/memory.db" ] && [ "$(stat -c %s "$REC/$1/memory.db")" = "$(stat -c %s "$2")" ] || nice -n 15 cp "$2" "$REC/$1/memory.db"
  echo "$1 <- $2" ; done > "$REC/SOURCES.txt"
cat "$REC/SOURCES.txt"
TONIGHT=$REC/tonight/memory.db nice -n 15 bash "$HK/sim/house/real_nights.sh" "$RUN" "$REC/base1/memory.db" "$REC/base2/memory.db" \
  || { echo "the real path failed: see above"; exit 1; }
echo "== scoring (the sim's truth, after the fact)"
nice -n 15 $PY "$HK/real/tests/rehearsal_score.py" "$RUN/tonight" "$NID" --reference "$REFERENCE" 2>&1 | grep -v Zenoh
