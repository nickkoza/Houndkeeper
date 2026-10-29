# Red team: the arm host (Raspberry Pi on Barrymore), 3 Oct 2026

> **Why this is here:** the arm host is the robot's one always-on computer with a camera, a microphone and lights; this is the security review its hardening ([CIu], [0Py] and the tests in tests/arm_host/) answers to.

Scope: the arm host spec [JIY], arm_host/ (hostd, eyes, health, release, card, the scripts, the profiles, the
systemd unit), tests/arm_host/, principle [0Py], real/.env.example. Nothing was run against a block device or a Pi;
everything below was checked by reading, by running the code on the laptop, or by a test. Each item says which.

Test command: `../dimos/.venv/bin/python -m pytest -q -p no:cacheprovider tests/arm_host`
(99 passing after the fixes, up from 47). `railyard.mjs check`: no findings.

## Ranked findings

| # | Severity | Finding | State |
|---|---|---|---|
| 1 | Critical | A NaN breath period strobes the eyes ([0Py], [GTG]) | Fixed, tested |
| 2 | High | Any device on the network could use the HTTP API; no authentication at all | Partly fixed (address allowlist); a shared secret still needed before the arm |
| 3 | High | The arm has no link-loss stop: no heartbeat, no arm code, the transport can't detect a drop in 300 ms | Open (design) |
| 4 | High | SPI timing follows the Pi 3's varying core clock: garbled LED frames, a flash | Fixed in software (pinned at first boot; eyes refused until pinned); unconfirmed on hardware |
| 5 | High | Eyes jump on a crash, a restart and a stop; the "fade out on stop" never ran | Fixed, tested |
| 6 | High | Card writer: a USB hard drive counted as a card; nested (encrypted/LVM) mounts unseen; mounts outside /home, /usr... ignored; card swappable between the check and dd | Fixed, tested |
| 7 | Medium | LED supply sag (motor load) resets the LEDs mid-frame: a flicker at gait rate; 3.3 V data into 5 V LEDs | Partly fixed (eyes off on the Pi's under-voltage flag); hardware open |
| 8 | Medium | Resource exhaustion: negative Content-Length hangs a thread for good; any body size read into 1 GB; unbounded speech queue; no socket timeout | Fixed, tested |
| 9 | Medium | Malformed /eyes bodies killed the eye thread (LEDs latched on), or threw out of the handler | Fixed, tested |
| 10 | Medium | Re-deploying a dirty build overwrote the running release in place; then "rollback" restored the same broken directory | Fixed, tested |
| 11 | Medium | The candidate release's own code performs its activation and rollback; check.sh passes on the old daemon; a power cut after the switch boots an unchecked release with no rollback | Partly fixed |
| 12 | Medium | hostd runs as hk, who has passwordless sudo: any bug in the daemon is root | Partly fixed (NoNewPrivileges etc.) |
| 13 | Medium | No speaker volume limit (hearing safety) | Open |
| 14 | Medium | The first (online) card streams the microphone over the house network to an address that may be someone else's there | Fixed, tested |
| 15 | Medium | Harvested .debs are installed by path with no signature or checksum check | Open |
| 16 | Medium | The Wi-Fi password stays on the card's FAT boot partition, readable by any local user and by anyone who pulls the card | Open (unconfirmed what is safe to delete) |
| 17 | Low | Emoji SSID / non-ASCII password silently mangled by cloud-init's YAML; bad country or password length bricks a headless card | Fixed, tested |
| 18 | Low | WS2812 latch 33 us, shorter than WS2812B-V5's 280 us reset | Fixed, tested |
| 19 | Low | switch_profile.sh: restarts hostd with no arm-stowed check, and drops the speaker unless re-given | Open |
| 20 | Low | provision.sh is stale: writes a host.json with no profile, unquoted values into JSON | Open |
| 21 | Low | Voice may not reach PipeWire at all from a system service (no XDG_RUNTIME_DIR) | Open, unconfirmed |
| 22 | Info | /health tells anyone allowed the release and services; README's `arm_host/set -a` typo | Typo fixed |

## Details

### 1. A NaN breath period strobes the eyes (Critical; fixed)

- **Attack.** `POST /eyes {"breathe": {"colour": [1,1,1], "period_s": NaN}}` (Python's json.loads accepts `NaN`; the
  string `"nan"` works too, through `float()`). Also reachable from a profile's `eyes_idle`.
- **Evidence.** eyes.py (before): `max(float(period_s), MIN_BREATH_S)` returns NaN (`max(nan, 4.0)` is nan);
  `step()` then made `target` NaN, and `min(lim, nan)` returns `lim`, so the output rose by the slew limit every frame
  without end. WS2812.show sent `255 * c**2.2` with no clamp, and `encode` keeps only the low 8 bits, so on the wire
  the brightness ramped up and wrapped to dark. Measured on the laptop: output 10.0 after 20 s, and 157 full-to-dark
  jumps in those 20 s, the rate rising past 10 Hz: squarely in the photosensitive range.
- **Fix.** `_clamp` maps any non-finite value to dark; `breathe` takes a non-finite period as the minimum; `_slew`
  clamps its result to 0..1; `encode` clamps each byte; `/eyes` parses JSON with NaN/Infinity refused (400).
- **Tests.** test_eyes.py `test_a_nonsense_breath_period_never_leaves_0_to_1` (failed before), test_hostd.py
  `test_nonsense_eye_commands_are_refused_and_change_nothing`.

### 2. The HTTP API has no authentication (High; partly fixed)

- **Attack.** hostd bound 0.0.0.0:47100 and served anyone: play any audio on the porch (`/say`), drive the eyes,
  read `/health`. The first card goes on the house network (README), so for a while that is everyone on the house
  Wi-Fi, guests included. Once arm commands join this API it would be anyone moving the arm. There is no "pairing"
  for the API (pairing in the code is only the Bluetooth speaker).
- **Fix.** `make_handler` answers only the laptop's address and 127.0.0.1 (check.sh); everything else gets 403.
- **Remains.** A source address is not authentication: anyone with the Wi-Fi password can ARP-spoof the laptop's
  address on the same segment. Before any arm endpoint: a per-card shared secret (written to the card next to
  host.json and to real/local/ on the laptop), an HMAC over each command with a sequence number and a short validity
  window (no replay), or mutual TLS. A firewall rule on the Pi (nftables: 47100 from the laptop only) as a second wall.
- **Tests.** `test_only_the_laptop_may_call`.

### 3. No stop on link loss for the arm (High; open, design)

- The spec's failure behaviour ("the arm stops within 300 ms and disarms", [41H], [3dt]) has no mechanism yet:
  hostd has no arm code (the houndkeeper profile sets `arm: true` and nothing reads it), no inbound heartbeat
  (reports only go out), and HTTP over TCP is the wrong carrier: TCP retransmission hides a dead link for seconds and
  each request is independent.
- **Recommendation.** (a) The arm's own controller stops on a CAN command timeout (a few tens of ms), so a dead or
  hung hostd also stops it; [BRx] "disarmed while it is down" is only true if this is in the arm, not in hostd.
  (b) hostd forwards motion only inside a lease: every arm command carries a deadline no more than 200 ms out, and a
  watchdog thread stops the arm when the newest lease expires (DimOS streams at 10 Hz or more, over UDP). (c) Arming
  refused while under-voltage or throttling is flagged (spec failure behaviour; nothing does this yet). (d) switch
  profile, deploy and restart all disarm first. Write the lease test against a fake CAN bus ([wcJ]) before the arm
  code.

### 4. SPI timing follows the core clock (High; fixed in software, unconfirmed on hardware)

- **Hazard.** On a Pi 3 the SPI clock is divided from the VPU core clock, which the firmware moves between 250 and
  400 MHz with load and lowers when throttling. The divisor is set once for 2.4 MHz; at a different core clock every
  bit's high time changes, a 0 reads as a 1, and the LEDs show a wrong, typically brighter colour for that frame. At
  60 frames a second that is a flicker exactly when the Pi gets busy or hot. (Well known for SPI-driven WS2812 on a
  Pi; the rpi_ws281x project's documentation asks for a fixed core clock. Not observed here: no hardware yet.)
- **Fix.** firstboot.sh pins the core clock (`core_freq`/`core_freq_min` equal: 250 on a Pi 3, 500 on a Pi 4) and
  reboots once; hostd reads `vcgencmd get_config` and, when the clock is not pinned or can't be read, never writes to
  the LEDs and refuses `/eyes` (409, said in the report as `eyes_blocked`). health.py `core_clock_fixed`, tested.
- **To confirm on the Pi.** That the two values read back equal after the reboot; and watch the LEDs with a phone's
  slow-motion video under full CPU load and through a throttle event.

### 5. Eyes jump on crash, restart and stop (High; fixed)

- **Evidence.** WS2812 LEDs hold their last frame. A new hostd started at `output = (0,0,0)` and sent it: a step from
  wherever the lights were to dark. The unit's comment said the eyes fade out on stop through hostd's `finally`, but
  Python installs no SIGTERM handler, so systemd's stop killed the process without running it; and even the
  `finally` only set a target and exited, with nothing left to fade. A crash loop (Restart=always, 2 s) became a
  repeating step. The eye thread also died on any exception (finding 9), leaving the lights latched.
- **Fix.** The slew limit is now held at the lowest layer too: `WS2812.show` clamps and slew-limits by its own clock,
  whatever it is handed, and keeps what the lights show in /run/houndkeeper/eyes.json (RuntimeDirectory, preserved
  across restarts), so a successor fades from where the lights are. SIGTERM raises SystemExit and `fade_out` fades to
  dark (at most 2.5 s; TimeoutStopSec is 5). `run_eyes` never dies: an error fades the eyes out.
- **Remains.** A reboot with the LEDs still powered clears /run: hostd then starts from dark while the LEDs may still
  hold a colour (one step). Only if the LEDs are powered through a reboot; power them from a rail the Pi's reboot
  cuts, or keep the state on the persistent partition once [UcX] exists.
- **Tests.** `test_the_led_driver_slew_limits_itself`, `test_a_restarted_driver_carries_on_from_where_the_lights_are`,
  `test_the_eye_loop_survives_a_fault_and_fades_out`, `test_stopping_fades_the_eyes_to_dark_before_exit`. The thread
  race (`_breath` cleared between the check and the unpack) is now under a lock; its test passed before the fix too
  (the race is narrow), so it is a characterisation test.

### 6. The card writer could erase a disk the laptop uses (High; fixed)

- **Evidence.** card.py `choose_device` took `tran == "usb"` as removable: an external USB SSD or backup drive
  (rm 0) of 256 GB or less passed. Mounts were looked for only on the disk and its partitions, not deeper, so a
  LUKS or LVM volume (mounted one level down) looked unused; an open encrypted or LVM volume with nothing mounted was
  not seen at all. Any mount outside a short list (e.g. /mnt/..., /srv, /data, /media/<user>/Backup) was allowed. The
  check ran before the Wi-Fi password prompt and the release build, so the user could swap disks while the script
  waited, and dd then wrote whatever was at that name; sudo's password prompt also sat between the check and dd.
  The only remaining guard was typing the device name (and `--yes` skips it).
- **Checked on this laptop, read-only:** its sda (SATA) and nvme0n1 are refused, before and after.
- **Fix.** Removable now means `rm` set, or the laptop's own card slot (`mmcblkN`, not eMMC boot areas); every
  descendant is walked; any crypt/LVM/RAID/dm holder refuses; mounts allowed only under /media/ and /run/media/ (a
  freshly inserted card's auto-mounts), shown in the confirmation. flash_card.sh now asks everything first, checks
  the card last, asks for sudo before the re-check, and re-reads lsblk just before dd, refusing unless path, serial,
  model and size are what the user confirmed. The copy onto the boot partition refuses a mount point outside
  /media.
- **Tests.** Six new cases in test_card.py (all failed before). The shell flow itself is not under test.

### 7. LED power and data electrical hazards (Medium; partly fixed)

- **Hazard (unconfirmed).** If the LEDs share a rail that sags with the motors, a dip below their reset threshold
  blacks them out and the next frame, 16 ms later, restores them: a flicker at the gait rate (2 to 3 Hz). Decision
  [jAM] takes 3.3 V data into 5 V LEDs and adds a level shifter only "if they flicker": the failure mode it waits for
  is the hazard. A floating data line during boot can latch random colours.
- **Fix.** hostd turns the eyes off (fading) and refuses `/eyes` once the Pi reports under-voltage, latched until
  restart.
- **Recommendation for [jAM].** A level shifter (74AHCT125) always; a 330 ohm series resistor and a 10 k pull-down on
  the data line; the LEDs on a regulated 5 V with bulk capacitance, not the motor rail. Suggested as a change to the
  decision; not edited here (a design choice, not a bug fix).

### 8. Resource exhaustion on the API (Medium; fixed)

- `int(Content-Length)` of -1 made `rfile.read(-1)` read to end of stream: a caller holding the connection kept a
  thread forever, with no socket timeout. A large Content-Length was read whole into memory, and the speech queue was
  unbounded, on a 1 GB Pi. A bad header threw out of the handler.
- **Fix.** 10 s socket timeout; 411 for a missing or negative length, 413 over 4 MB for a clip or 4 KB for an eye
  command (refused before reading); at most 8 clips waiting (503). Tests: three in test_hostd.py.

### 9. Malformed /eyes bodies (Medium; fixed)

- A two-channel colour shrank the output tuple to two channels; WS2812.encode's unpack then raised in the eye thread,
  which died with the LEDs latched. Unknown keys, a list or a string as the body raised out of the handler.
- **Fix.** `eye_command` validates the shape; `_rgb` insists on three channels; 400 for anything else. Tests:
  parametrised in test_hostd.py and test_eyes.py.

### 10. A re-deployed dirty build overwrote the running release (Medium; fixed)

- build_release.sh named every dirty build of one commit `N-dirty`; deploy.sh untarred into releases/ directly, over
  the running directory if the name matched, and `activate` then recorded that same directory as "previous": a failed
  check rolled back to the broken code. Contrary to [WHV] ("nothing is upgraded in place").
- **Fix.** Dirty builds carry a content hash; deploy.sh refuses a name already on the host and unpacks into a
  temporary directory renamed whole; `activate` refuses the current release and takes a lock (two activations can't
  restore each other's "previous"). Tests: three in test_release.py (all failed before).

### 11. Rollback that depends on the candidate (Medium; partly fixed)

- deploy.sh ran `arm_host.release` from the new release's directory, so a broken candidate could break its own
  rollback. check.sh passed when *any* hostd answered: a restart that failed left the old daemon answering, as the
  old release. A daemon that answered once and died later passed.
- **Fixed.** deploy.sh activates with the current release's code (the new one's only on a host with none); check.sh
  requires `/health` to name its own release, and again 10 s later.
- **Open.** A power cut between the link switch and the end of the check boots into an unchecked release, and a
  release that crashes later loops on Restart=always forever, with no rollback ([27n], [871]). Suggested: write a
  `pending` marker before the switch, clear it on a passed check; a boot-time unit (or `OnFailure=` with
  `StartLimitBurst=`) switches back when it finds the marker or a start-limit failure.

### 12. The daemon can reach root (Medium; partly fixed)

- hostd runs as hk; firstboot gives hk `NOPASSWD:ALL`; the release tree is writable by hk. A code-execution bug in
  hostd or in what it feeds (aplay parses whatever WAV it is sent) is root and can rewrite the releases.
- **Fixed.** The unit sets `NoNewPrivileges=yes` (sudo no longer works from the daemon), `ProtectSystem=full`,
  `PrivateTmp=yes`. Unconfirmed on the Pi that audio and Bluetooth still work under these (they should: nothing
  needs privilege).
- **Open.** A separate service user without sudo, and releases owned by root.

### 13. No speaker volume limit (Medium; open)

- Nothing caps loudness: a clip at full scale plays at whatever the sink and the speaker's own volume are. A child's
  ear at the robot's head on Halloween is the case to plan for. Suggested: firstboot sets the PipeWire sink volume
  with a ceiling (`wpctl set-volume -l`); hostd scales each clip so its peak is no higher than a fixed level and
  refuses clips longer than a set time; measure the level at 10 cm with a sound meter and record it as a decision.
  Needs the speaker in hand.

### 14. The microphone on the house network (Medium; fixed)

- The README's first card (online, for harvesting) goes on the house network with any profile; Halloween streams
  the microphone, unencrypted UDP, to `laptop_ip`, which is the laptop's robot-network address and on the house
  network may be nobody or someone else's device.
- **Fix.** flash_card.sh passes `--no-microphone` for any SSID but the robot network; card.py then sets
  `ears: false`. Test: `test_a_card_for_another_network_sends_no_microphone`.
- **Note.** On the robot network the stream is still plaintext UDP readable by anyone holding the Wi-Fi password;
  nothing on the laptop receives it yet (no listener in the repository): whoever writes it must keep nothing ([gFa]).

### 15. Harvested packages are trusted blindly (Medium; open)

- harvest_packages.sh copies every .deb from the Pi's apt cache (over rsync or straight off the card); firstboot
  installs them by path (`apt-get install ./x.deb`, then `dpkg -i *.deb`), which checks no signature. Whatever can
  write ~/.cache/houndkeeper/pi-debs, or a host compromised on the house network before the harvest, gets root on
  every later card. `dpkg -i *.deb` also installs every stale version the cache held.
- **Suggested.** At harvest, copy the Pi's signed apt lists (InRelease and Packages) as well, verify InRelease with
  the Raspberry Pi and Debian keyrings on the laptop, and keep only debs whose SHA-256 matches; write a manifest; have
  firstboot check the manifest and install only the named packages.

### 16. The Wi-Fi password stays on the card (Medium; open)

- network-config (with the PSK) stays on the FAT boot partition after first boot: readable by every local user (FAT
  has no permissions) and by anyone who takes the card out of a robot on a porch. With it, the robot network and
  everything on it are reachable. The laptop side is fine: the PSK goes through a 0700 temporary directory removed
  on exit, is never on a command line, and real/.env is gitignored and mode 600.
- **Suggested, after checking on the Pi** that cloud-init (NoCloud, dsmode local) does not re-read network-config on
  later boots: have firstboot overwrite the password in /boot/firmware/network-config once the network is up. Not
  done here: wrong, it leaves a headless Pi with no network.

### 17. Values cloud-init would misread (Low; fixed)

- Confirmed with PyYAML 6.0.3: JSON's surrogate-pair escape of an emoji SSID comes back as two broken characters, so
  the card would never join. Also unchecked: a country that is not two letters (passed to raspi-config), and a
  password WPA rejects (under 8 or over 63 characters). Each left a headless, password-locked Pi with no network.
- **Fix.** card.py refuses them on the laptop. Tests: three in test_card.py.

### 18. Short latch (Low; fixed)

- The trailing low was 10 bytes (33 us at 2.4 MHz): enough for the original WS2812 (50 us is the spec, but the idle
  line covers it) but short of the WS2812B-V5's 280 us. Now 80 us before and 320 us after, the data still inside the
  SPI FIFO for two LEDs. Test: `test_the_latch_gap_suits_every_ws2812`.

### 19 to 22 (Low and Info; open unless noted)

- switch_profile.sh restarts hostd with no arm-stowed check (deploy.sh has one) and writes host.json without the
  speaker unless `--speaker-mac` is given again.
- provision.sh predates the card path: it writes host.json with no profile and puts `$LEDS` into JSON unquoted.
  Retire it or make it call `arm_host.card host`.
- hostd's aplay runs from a system service with no XDG_RUNTIME_DIR, so with pipewire-alsa it may not reach the user's
  PipeWire (and so the Bluetooth speaker). Unconfirmed: check on the Pi; if so, set `XDG_RUNTIME_DIR=/run/user/<uid>`
  in the unit.
- `/health` gives release and service names to the allowed callers only now. README's `arm_host/set -a` fixed.

## What I changed

- arm_host/eyes.py: NaN/inf safe, three channels enforced, a lock, the slew limit and clamp also in WS2812.show with
  its state kept across restarts, longer latch.
- arm_host/hostd.py: handler moved to `make_handler` (testable); address allowlist; body limits and timeouts;
  bounded speech queue; validated eye commands; eye loop that never dies; SIGTERM fade-out; eyes off on
  under-voltage or an unpinned core clock, reported as `eyes_blocked`.
- arm_host/health.py: `read_core_clock`, `core_clock_fixed`.
- arm_host/card.py: card selection hardened, `identity`, Wi-Fi value checks, `--no-microphone`.
- arm_host/flash_card.sh: questions first, card last, sudo before the re-check, identity re-checked before dd,
  boot mount checked, microphone off off the robot network.
- arm_host/release.py, build_release.sh, deploy.sh, check.sh: see 10 and 11.
- arm_host/firstboot.sh: core clock pinned, one reboot.
- arm_host/systemd/hk-hostd.service: RuntimeDirectory kept across restarts, NoNewPrivileges, ProtectSystem,
  PrivateTmp.
- tests/arm_host/: 52 new test cases; links answered under symbols/links/.

The spec was not edited. Two places where the spec and the code should be looked at together: [GTG]'s Tests row now
has more behind it (the driver's own limit, restart, stop, a fault in the loop); and [jAM]'s "level shifter if they
flicker" (finding 7) is worth turning round.
