#!/usr/bin/env bash
# The phone cockpit's relay secrets ([rhP]): a robot key for each participant (bound to the robot id its bridge runs
# with: the robot, and "project", the project service of project-app [eTc]) and one viewer token for the phone,
# written to real/local/relay-auth.json (gitignored, mode 600) in the shape DimOS's relay reads (docs/web/relay.md
# "Auth file"). Refuses to overwrite: rotating is deleting the file, re-running, restarting the relay.
set -euo pipefail
HK="$(cd "$(dirname "$0")/../.." && pwd)"
OUT="${1:-$HK/real/local/relay-auth.json}"; ROBOTS="${ROBOTS:-${ROBOT_ID:-barrymore} project}"; VIEWER="${VIEWER:-phone}"
[[ -e "$OUT" ]] && { echo "$OUT exists; delete it first to rotate" >&2; exit 1; }
mkdir -p "$(dirname "$OUT")"
umask 077
robots=""; for r in $ROBOTS; do robots+="${robots:+, }\"$r\": \"$(openssl rand -hex 32)\""; done
token="$(openssl rand -hex 32)"
printf '{\n  "robots": { %s },\n  "viewers": { "%s": "%s" }\n}\n' "$robots" "$VIEWER" "$token" > "$OUT"
chmod 600 "$OUT"
echo "wrote $OUT (robot ids: $ROBOTS; viewer $VIEWER); the phone's token: jq -r '.viewers.$VIEWER' $OUT"
