#!/usr/bin/env bash
# A quick patch: push a numbered release of the arm host to the running Pi and make it current ([871], [WHV], [H9j]):
# unpacked beside the old ones, switched in by arm_host.release, rolled back by itself if its check fails. Applied only
# while the arm is stowed and the robot lies still: until the arm adapter reports its state, that is the operator's to
# confirm (--robot-still). A clean baseline is a new card instead: flash_card.sh.
# Usage: arm_host/deploy.sh <pi host> --robot-still
set -euo pipefail
PI=$1; [ "${2:-}" = "--robot-still" ] || { echo "confirm the robot is lying still and the arm stowed: --robot-still"; exit 2; }
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
N=$("$(dirname "$0")/build_release.sh" "$T")
# The release tree is root's and only root writes it ([CIu], decision [ZyY]): the release streams in over SSH (no file in
# a shared /tmp), is unpacked by root beside the others under a temporary name keeping no owner from the archive, and
# renamed whole: never over an existing release (the running one included), never half there. Activated by root with
# isolated Python (-I) from the current release's code, known to work, not the candidate's own. Users and units change
# only on a new card, so a host without that boundary (a card from before it) is refused before root runs anything of
# its tree: it is reflashed, never patched.
ssh "hk@$PI" "set -e; R=/opt/houndkeeper/releases
  { id hkd >/dev/null 2>&1 && grep -qx 'User=hkd' /etc/systemd/system/hk-hostd.service \
    && [ -z \"\$(sudo find /opt/houndkeeper /etc/houndkeeper ! -user root -print -quit)\" ] \
    && [ -z \"\$(sudo find /opt/houndkeeper /etc/houndkeeper ! -type l -perm /022 -print -quit)\" ]; } \
    || { echo 'this card predates the privilege boundary (decision ZyY): flash a new one (flash_card.sh), never deploy onto it'; exit 4; }
  [ ! -e \$R/$N ] || { echo 'release $N is already on the host'; exit 3; }
  S=\$(sudo mktemp -d \$R/.incoming-XXXX); sudo tar --no-same-owner -C \$S -xzf -; sudo chmod -R go-w \$S
  sudo mv \$S/$N \$R/$N; sudo rmdir \$S
  A=/opt/houndkeeper/current; [ -f \$A/arm_host/release.py ] || A=\$R/$N
  sudo python3 -I -B \$A/arm_host/release.py activate $N" < "$T/release.tgz"
