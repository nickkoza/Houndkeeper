#!/usr/bin/env bash
# The arm host's packages, fetched by the laptop so the Pi never goes online ([DzZ], [871]): kept offline, it
# is ready as soon as it is plugged in. The trust comes from the
# system image itself (pi-image.json's checksum): its apt sources, its archive keyrings and its list of installed
# packages are read out of the image's root partition (debugfs, no root), and the laptop's apt resolves firstboot.sh's
# PKGS against exactly that system, for arm64, checking every repository's signature and every package's checksum as
# the Pi's own apt would. The .debs land where flash_card.sh puts them on the card.
# Usage: arm_host/fetch_packages.sh            (once per image; again to pick up security updates)
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
J() { python3 -c "import json, sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])" "$HERE/pi-image.json" "$1"; }
IMG=$(J file); SHA=$(J sha256); URL=$(J url)
CACHE=${HK_PI_CACHE:-$HOME/.cache/houndkeeper/pi-image}; DEBS=${HK_PI_DEBS:-$HOME/.cache/houndkeeper/pi-debs/${IMG%.img.xz}}
PKGS=$(sed -n 's/^PKGS="\(.*\)"$/\1/p' "$HERE/firstboot.sh"); [ -n "$PKGS" ] || { echo "no PKGS line in firstboot.sh"; exit 1; }
mkdir -p "$CACHE"
if ! echo "$SHA  $CACHE/$IMG" | sha256sum -c --quiet 2>/dev/null; then
  echo "the system image isn't on this laptop yet: downloading it once"; curl -fL -o "$CACHE/$IMG" "$URL"
  echo "$SHA  $CACHE/$IMG" | sha256sum -c --quiet || { echo "checksum mismatch"; exit 1; }; fi
W=$(mktemp -d -p "${XDG_CACHE_HOME:-$HOME/.cache}"); trap 'rm -rf "$W"' EXIT
echo "reading the image's apt sources, keyrings and installed packages"
xz -dc "$CACHE/$IMG" > "$W/pi.img"                    # from the checked file, never a loose copy
OFF=$(( $(sfdisk -J "$W/pi.img" | python3 -c "import json, sys; print(json.load(sys.stdin)['partitiontable']['partitions'][1]['start'])") * 512 ))
FS="$W/pi.img?offset=$OFF"; R=$W/root; mkdir -p "$W/keys" "$R/etc/apt/sources.list.d" "$R/etc/apt/preferences.d" \
  "$R/var/lib/apt/lists/partial" "$R/var/cache/apt/archives/partial" "$R/var/lib/dpkg"
DBG() { /usr/sbin/debugfs -R "$1" "$FS" 2>/dev/null; }
DBG "dump /var/lib/dpkg/status $R/var/lib/dpkg/status"
for f in $(DBG "ls -p /usr/share/keyrings" | awk -F/ '$6 ~ /\.(pgp|gpg)$/ {print $6}'); do   # links followed inside the image
  t=$f; for _ in 1 2 3; do l=$(DBG "stat /usr/share/keyrings/$t" | sed -n 's/^Fast link dest: "\(.*\)"$/\1/p'); [ -n "$l" ] || break; t=$(basename "$l"); done
  DBG "dump /usr/share/keyrings/$t $W/keys/$f"; done
for s in $(DBG "ls -p /etc/apt/sources.list.d" | awk -F/ '$6 ~ /\.sources$/ {print $6}'); do
  DBG "cat /etc/apt/sources.list.d/$s" | sed "s#/usr/share/keyrings/#$W/keys/#" > "$R/etc/apt/sources.list.d/$s"; done
[ -s "$R/var/lib/dpkg/status" ] && ls "$R"/etc/apt/sources.list.d/*.sources >/dev/null 2>&1 \
  || { echo "could not read the image's package state or sources"; exit 1; }
APT=(apt-get -q -o Dir="$R" -o Dir::State::status="$R/var/lib/dpkg/status" -o Dir::Etc::trusted=/dev/null
     -o Dir::Etc::trustedparts="$W/none" -o APT::Architecture=arm64 -o APT::Architectures=arm64
     -o Acquire::Languages=none -o APT::Sandbox::User="$(id -un)" -o Debug::NoLocking=1)
echo "resolving against the image, for arm64: $PKGS"
"${APT[@]}" update
"${APT[@]}" install -y --download-only --no-install-recommends $PKGS
n=$(ls "$R"/var/cache/apt/archives/*.deb 2>/dev/null | wc -l); [ "$n" -gt 0 ] || { echo "nothing downloaded"; exit 1; }
rm -rf "$DEBS.new"; mkdir -p "$DEBS.new"; cp "$R"/var/cache/apt/archives/*.deb "$DEBS.new/"
rm -rf "$DEBS"; mv "$DEBS.new" "$DEBS"
PYTHONPATH="$(cd "$HERE/.." && pwd)" python3 -m arm_host.packages write --debs "$DEBS"     # the pinned set ([UYK], [iwZ])
echo "$n packages for the card in $DEBS ($(du -sh "$DEBS" | cut -f1)), each checked against the image's own archive keys"
