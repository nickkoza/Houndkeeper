#!/usr/bin/env bash
# One step from a blank SD card to an arm host ([OmH], [H9j], [ObX]): put the card in the laptop, run this, put the
# card in the Pi and power it. The Pi never goes online: its packages ride on the card, fetched by the laptop
# (fetch_packages.sh, checked against the image's own archive keys): plugged in, it is ready.
# Usage: arm_host/flash_card.sh /dev/sdX --profile houndkeeper --laptop-ip <laptop's address>
#        [--ssid <network>] [--speaker-mac AA:BB:..] [--country US] [--yes]   (network names: real/.env)
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); ROOT=$(cd "$HERE/.." && pwd)
PY=${PY:-python3}; export PYTHONPATH="$ROOT"
DEV=$1; shift; PROFILE=""; LAPTOP=""; SSID=""; MAC=""; COUNTRY=US; YES=0
while [ $# -gt 0 ]; do case $1 in --profile) PROFILE=$2; shift 2;; --laptop-ip) LAPTOP=$2; shift 2;; --ssid) SSID=$2; shift 2;;
  --speaker-mac) MAC=$2; shift 2;; --country) COUNTRY=$2; shift 2;; --yes) YES=1; shift;; *) echo "unknown: $1"; exit 2;; esac; done
[ -n "$PROFILE" ] && [ -n "$LAPTOP" ] || { echo "--profile and --laptop-ip are needed"; exit 2; }
IMG=$(python3 -c "import json; print(json.load(open('$HERE/pi-image.json'))['file'])"); SHA=$(python3 -c "import json; print(json.load(open('$HERE/pi-image.json'))['sha256'])")
URL=$(python3 -c "import json; print(json.load(open('$HERE/pi-image.json'))['url'])")
CACHE=${HK_PI_CACHE:-$HOME/.cache/houndkeeper/pi-image}; DEBS=${HK_PI_DEBS:-$HOME/.cache/houndkeeper/pi-debs/${IMG%.img.xz}}
mkdir -p "$CACHE"
if ! echo "$SHA  $CACHE/$IMG" | sha256sum -c --quiet 2>/dev/null; then
  echo "the system image isn't on this laptop yet: downloading it once"; curl -fL -o "$CACHE/$IMG" "$URL"
  echo "$SHA  $CACHE/$IMG" | sha256sum -c --quiet || { echo "checksum mismatch"; exit 1; }; fi
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
[ -f "$ROOT/real/.env" ] && set -a && . "$ROOT/real/.env" && set +a
[ -n "$SSID" ] || SSID=${ROBOT_SSID:?"set ROBOT_SSID in real/.env (see real/.env.example), or give --ssid"}
PSK=""; [ "$SSID" = "${ROBOT_SSID:-}" ] && PSK=${ROBOT_PSK:-}; [ "$SSID" = "${HOUSE_SSID:-}" ] && PSK=${HOUSE_PSK:-}   # from real/.env
CON=$(nmcli -t -f NAME,TYPE con show | awk -F: -v s="$SSID" '$1==s && $2 ~ /wireless/{print $1; exit}')
if [ -n "$PSK" ]; then printf '%s' "$PSK" > "$T/psk"
elif { [ -n "$CON" ] && nmcli -s -g 802-11-wireless-security.psk con show "$CON" > "$T/psk" 2>/dev/null && [ -s "$T/psk" ]; }; then :
else read -r -s -p "Wi-Fi password for $SSID: " P; echo; printf '%s' "$P" > "$T/psk"; fi
KEY=$(ls "$HOME"/.ssh/id_ed25519.pub "$HOME"/.ssh/id_rsa.pub 2>/dev/null | head -1); [ -n "$KEY" ] || { echo "no SSH public key in ~/.ssh"; exit 2; }
REL=$("$HERE/build_release.sh" "$T")
NOMIC=""; [ "$SSID" = "${ROBOT_SSID:-}" ] || NOMIC=--no-microphone     # the microphone only on the robot network ([gFa])
ls "$DEBS"/*.deb >/dev/null 2>&1 || { echo "no packages for the card yet: fetching them on the laptop (once per image)"
  HK_PI_CACHE="$CACHE" HK_PI_DEBS="$DEBS" "$HERE/fetch_packages.sh"; }
ls "$DEBS"/*.deb >/dev/null 2>&1 || { echo "still no packages in $DEBS: not making a card that would need the internet"; exit 1; }
$PY -m arm_host.packages check --debs "$DEBS" || { echo "the packages in $DEBS are not the ones pinned in" \
  "arm_host/pi-packages.lock.json: run arm_host/fetch_packages.sh and commit the lock it writes"; exit 1; }   # [UYK]
OFF=--offline
TS=""; [ "$SSID" = "${ROBOT_SSID:-}" ] && TS=${ROUTER_ON_ROBOT_NET:-}    # the one clock: the router the laptop follows ([aVJ])
[ -n "$TS" ] || echo "no time source for this network (ROUTER_ON_ROBOT_NET in real/.env; robot network only): the Pi's clock won't be set"
$PY -m arm_host.card files --out "$T/boot" --profile "$PROFILE" --ssh-key-file "$KEY" --ssid "$SSID" --psk-file "$T/psk" \
  --laptop-ip "$LAPTOP" --release "$REL" --country "$COUNTRY" --speaker-mac "$MAC" --time-server "$TS" $OFF $NOMIC
cp "$HERE/firstboot.sh" "$T/release.tgz" "$T/boot/hk/"; mkdir -p "$T/boot/hk/debs"; cp "$DEBS"/*.deb "$T/boot/hk/debs/"
# the card: only a removable one, named, not in use ([ObX]). Checked after everything that can wait on the user (the
# Wi-Fi password) is done, and its identity checked again just before the write: a card swapped for another disk
# between the question and the write is not written.
lsblk_now() { lsblk -J -b -o NAME,PATH,SIZE,RM,TRAN,TYPE,MODEL,SERIAL,MOUNTPOINTS > "$T/lsblk.json"; }
lsblk_now; INFO=$($PY -m arm_host.card device --lsblk "$T/lsblk.json" --name "$DEV") || exit 3
ID=${INFO%%$'\t'*}
echo "will ERASE: ${INFO#*$'\t'}"; if [ $YES = 0 ]; then read -r -p "type the device name ($DEV) to go on: " A; [ "$A" = "$DEV" ] || exit 1; fi
sudo -v                                                        # any password asked now, not between the check and the write
for p in $(lsblk -nro PATH "$DEV" | tail -n +2); do udisksctl unmount -b "$p" >/dev/null 2>&1 || true; done
lsblk_now; NOW=$($PY -m arm_host.card device --lsblk "$T/lsblk.json" --name "$DEV") || { echo "$DEV is no longer safe to write"; exit 3; }
[ "${NOW%%$'\t'*}" = "$ID" ] || { echo "$DEV is not the card you confirmed (was $ID, now ${NOW%%$'\t'*})"; exit 3; }
echo "writing the system image to $DEV (a few minutes)"
xz -dc "$CACHE/$IMG" | sudo dd of="$DEV" bs=4M conv=fsync status=progress
sync; sudo partprobe "$DEV" 2>/dev/null || true; sleep 3
BOOTP=$(lsblk -nro PATH,LABEL "$DEV" | awk '$2=="bootfs"{print $1}')
M=$(udisksctl mount -b "$BOOTP" | sed -E 's/.* at (.*)\.?$/\1/; s/\.$//')
case "$M" in /media/?*|/run/media/?*) [ -d "$M" ] || { echo "the card's boot partition didn't mount"; exit 1; };;
  *) echo "the card's boot partition mounted somewhere unexpected ('$M'): not copying"; exit 1;; esac
cp -r "$T/boot/." "$M/"; sync; udisksctl unmount -b "$BOOTP" >/dev/null
echo "card ready: profile $PROFILE, release $REL, packages on the card: plug it in"
echo "put it in the Pi and power on: about 5 minutes to a working host (hk@hk-arm.local)"
