#!/usr/bin/env bash
# The card's first boot, run once by cloud-init ([OmH], decision [mX4]): packages from the card (decision [XzS]; the
# Pi never goes online), the eyes' SPI pin, the host's settings, the clock's source ([aVJ]), the
# release (current and known-good, [27n]), the profile's services ([BRx]), and apt's own timers masked so it never
# updates itself ([871]). Logs to /var/log/hk-firstboot.log.
set -uo pipefail
exec > >(tee -a /var/log/hk-firstboot.log) 2>&1
B=/boot/firmware/hk; CARD=$(cat $B/card.json)
REL=$(python3 -I -c "import json,sys; print(json.loads(sys.argv[1])['release'])" "$CARD")
OFFLINE=$(python3 -I -c "import json,sys; print(int(json.loads(sys.argv[1])['offline_packages']))" "$CARD")
PKGS="python3-spidev alsa-utils bluez pipewire pipewire-pulse pipewire-alsa wireplumber libspa-0.2-bluetooth"
export DEBIAN_FRONTEND=noninteractive
if [ "$OFFLINE" = 1 ] && ls $B/debs/*.deb >/dev/null 2>&1; then
  echo "packages from the card"; apt-get install -y --no-download $B/debs/*.deb || dpkg -i $B/debs/*.deb
else                                              # never the internet: a card without its packages was made wrong
  echo "FAILED: no packages on the card ($PKGS); re-flash it (flash_card.sh fetches them on the laptop)"; exit 1
fi
systemctl mask apt-daily.timer apt-daily-upgrade.timer apt-daily.service apt-daily-upgrade.service   # [871]
raspi-config nonint do_spi 0
# the eyes' SPI clock is divided from the core clock, which the Pi varies with load and throttling: pinned, or the LEDs'
# bit timing drifts and they show garbled colours, a flash ([GTG]). hostd keeps the eyes off until it is pinned.
CFG=/boot/firmware/config.txt
if grep -q "Raspberry Pi 4" /proc/device-tree/model 2>/dev/null; then PIN="core_freq=500 core_freq_min=500"; else PIN="core_freq=250 core_freq_min=250"; fi
for L in $PIN; do grep -qx "$L" $CFG || { echo "$L" >> $CFG; REBOOT=1; }; done
# the privilege boundary ([CIu], decision [ZyY]): the services run as hkd (a system account: no login, no sudo, no
# SSH key; only the devices the daemon uses: SPI, GPIO, sound, Bluetooth, vcgencmd, the arm's serial link); hk, the
# key-only SSH login, is the admin and runs no service. Its home holds its audio session's state (PipeWire,
# kept running without a login); everything root runs or reads stays root's.
id -u hkd >/dev/null 2>&1 || useradd --system --user-group --home-dir /var/lib/houndkeeper --shell /usr/sbin/nologin hkd
usermod -aG spi,gpio,audio,bluetooth,video,dialout hkd; passwd -l hkd >/dev/null
install -d -o hkd -g hkd -m 750 /var/lib/houndkeeper
loginctl enable-linger hkd
printf 'PasswordAuthentication no\nKbdInteractiveAuthentication no\nPermitRootLogin no\nAllowUsers hk\n' > /etc/ssh/sshd_config.d/10-houndkeeper.conf
install -d -o root -g root -m 755 /opt/houndkeeper /opt/houndkeeper/releases /etc/houndkeeper
install -o root -g root -m 644 $B/host.json /etc/houndkeeper/host.json
if [ -f $B/timesyncd.conf ]; then                 # the one clock: the router only, no fallback on the internet ([LSY])
  install -D -m 644 $B/timesyncd.conf /etc/systemd/timesyncd.conf.d/10-houndkeeper.conf
  timedatectl set-ntp true; systemctl restart systemd-timesyncd; fi
# the release: root's and writable by no one else, before anything is installed or run from it ([CIu])
tar --no-same-owner -C /opt/houndkeeper/releases -xzf $B/release.tgz
chown -R root:root /opt/houndkeeper; chmod -R go-w /opt/houndkeeper
ln -sfn "releases/$REL" /opt/houndkeeper/current; ln -sfn "releases/$REL" /opt/houndkeeper/known-good   # [izZ]
sync
install -o root -g root -m 644 /opt/houndkeeper/current/arm_host/systemd/*.service /etc/systemd/system/
SVCS=$(python3 -I -c "import json; print(' '.join(json.load(open('/etc/houndkeeper/host.json')).get('services') or ['hk-hostd']))")
systemctl daemon-reload; systemctl enable hk-release-recover; systemctl enable --now $SVCS   # each from power-up
[ -n "${REBOOT:-}" ] && shutdown -r +1 "core clock pinned for the eyes: restarting once"
echo "first boot done: release $REL, profile $(python3 -I -c "import json; print(json.load(open('/etc/houndkeeper/host.json'))['profile'])")"
