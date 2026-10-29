#!/usr/bin/env bash
# A running host to another profile in seconds, no reflash ([H9j], decision [tDE]): its settings rewritten, its
# service restarted. Usage: arm_host/switch_profile.sh <pi host> houndkeeper --laptop-ip <ip> [--speaker-mac ..]
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); export PYTHONPATH="$(cd "$HERE/.." && pwd)"
PI=$1; PROFILE=$2; shift 2
# Settings made and checked here first: a failed or empty generation never reaches the Pi's file or restarts its service;
# on the card before the rename, so a power cut leaves the old file or the new one ([27n])
CONF=$(python3 -m arm_host.card host --profile "$PROFILE" "$@")
python3 -c 'import json, sys; json.loads(sys.argv[1])' "$CONF" || { echo "settings for $PROFILE are not valid JSON; $PI unchanged" >&2; exit 1; }
printf '%s\n' "$CONF" | ssh "hk@$PI" "sudo tee /etc/houndkeeper/host.json.new >/dev/null && sudo sync /etc/houndkeeper/host.json.new && sudo mv /etc/houndkeeper/host.json.new /etc/houndkeeper/host.json && sudo sync /etc/houndkeeper && sudo systemctl restart hk-hostd"
echo "$PI is now $PROFILE"
