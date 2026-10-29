#!/usr/bin/env bash
# The arm host's checks on the real Pi, one command from the laptop ([JIY], decision [Uyb]): the Pi's half over SSH,
# run by root from the laptop's own copy of its code (never the Pi's release, which a check must not trust: decision
# [ZyY]), then the laptop's half (ports, a password login, the clock, the reports); the record goes to
# real/local/arm-host-checks/ (gitignored: it holds the house's addresses). Exit 0 when nothing failed.
# Usage: arm_host/pi_check.sh <pi host> [--kill-test] [--only UYK,27n] [--expect-unreachable]
#   two cards from one commit: python3 -m arm_host.pi_check compare <record a> <record b>
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/.." && pwd); export PYTHONPATH="$ROOT"
[ -f "$ROOT/real/.env" ] && set -a && . "$ROOT/real/.env" && set +a
PI=${1:?usage: pi_check.sh <pi host> [--kill-test] [--only IDS] [--expect-unreachable]}; shift
PIARGS=(); LAPARGS=(); AWAY=0
while [ $# -gt 0 ]; do case $1 in
  --kill-test) PIARGS+=(--kill-test); shift;;
  --only) [[ ${2:-} =~ ^[A-Za-z0-9,]+$ ]] || { echo "--only: criterion IDs, comma-separated"; exit 2; }
          PIARGS+=(--only "$2"); LAPARGS+=(--only "$2"); shift 2;;
  --expect-unreachable) LAPARGS+=(--expect-unreachable); AWAY=1; shift;;     # run from the guest network
  *) echo "unknown: $1"; exit 2;; esac; done
OUT="$ROOT/real/local/arm-host-checks"; mkdir -p "$OUT"; STAMP=$(date +%Y%m%d-%H%M%S)
if [ $AWAY = 0 ]; then
  # the files the Pi's half needs, into a new directory root makes (no one else can write it), run with isolated
  # Python (-I: no environment, no current directory, no user site), and removed after
  tar -C "$ROOT" -cf - arm_host/__init__.py arm_host/pi_check.py arm_host/packages.py arm_host/release.py \
      arm_host/pi-packages.lock.json \
    | ssh -o BatchMode=yes -o ConnectTimeout=10 "hk@$PI" \
    "D=\$(sudo mktemp -d) || exit 1; sudo tar --no-same-owner -C \"\$D\" -xf - \
      && sudo python3 -I -B -c 'import runpy, sys; sys.path.insert(0, sys.argv.pop(1)); runpy.run_module(\"arm_host.pi_check\", run_name=\"__main__\")' \"\$D\" on-pi ${PIARGS[*]:-}; rc=\$?; sudo rm -rf \"\$D\"; exit \$rc" \
    > "$OUT/$STAMP-pi.json" || echo "the Pi's half did not finish" >&2
fi
python3 -m arm_host.pi_check from-laptop --host "$PI" --on-pi "$OUT/$STAMP-pi.json" "${LAPARGS[@]}" > "$OUT/$STAMP.json"; rc=$?
echo "record: $OUT/$STAMP.json" >&2
exit $rc
