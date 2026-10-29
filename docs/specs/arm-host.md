# Arm host

**Type:** Capability spec · **Status:** Draft; in work from 2 Oct 2026 (the Go2 Air arrives in October, the arm later) · **Source:** the Raspberry Pi software, deployment and update mechanism, needed ahead of real hardware testing; CALM node `arm-host`; principles [zGo], [eNx], [5oU] · **ID:** [JIY]

## Purpose

The small computer riding on the robot (CALM node `arm-host`) runs the Maker Arm adapter and the wrist camera, and,
from before the arm arrives, the robot's voice, ears and eyes: the Go2 Air has no speaker or microphone of its own. It
has to be dependable before the real arm is bolted on: rebuilt the same way every time, running the right software
from power-up, updated only on purpose, and never the reason the arm moves when it shouldn't. This spec is a
prerequisite of the arm's bench test ([0h9]) and of the arm on board ([Bhf]).

## Acceptance criteria

1. [UYK] **Rebuilt from the repository.** A blank SD card becomes a working arm host by one script run from the
   development laptop, from pinned versions kept in the repository; no step is done by hand. Two builds from the same
   commit run the same software.
2. [BRx] **Ready from power-up, arm disarmed.** On power-up the arm adapter and the wrist camera stream start by
   themselves; the arm stays disarmed until DimOS arms it ([3dt]). A service that dies is restarted, and the arm is
   disarmed while it is down.
3. [871] **Updated only on purpose.** Software changes arrive only as a numbered release pushed from the laptop. A
   release is applied only while the arm is stowed and the robot is docked or lying still; if the new release fails
   its own start-up check, the previous one comes back by itself. The host never updates itself or takes packages
   from anywhere but the laptop.
4. [DzZ] **No internet.** The host works, builds and updates with no route to the internet; everything it installs
   comes from the laptop ([8XQ], [eNx]).
5. [Ot6] **Wrist video stays in the house.** The wrist camera is streamed to DimOS on the laptop over the house
   network and nowhere else, and the host stores no footage ([zGo], [snp]).
6. [aVJ] **One clock.** The host's clock follows the laptop's, so wrist frames and arm states line up with DimOS's
   recordings ([Bj8], [Yms]).
7. [4WJ] **It says how it is.** The host reports to DimOS, at least once a second: its release, each service's
   state, CPU temperature and throttling, and the Pi's own under-voltage flag (a sagging supply, [EkS]).
8. [27n] **Power can be cut at any time.** Losing power mid-write never leaves the host unable to boot or running a
   half-applied release.
9. [CIu] **Locked down.** Reachable only from the house network, by SSH key; no password logins and no default
   accounts left enabled; a service the network reaches cannot take over the host.
10. [MA0] **Whole without the arm.** With no arm connected the host runs, reports the arm absent, and its other
    services work; connecting the arm later needs no rebuild.
11. [u9G] **The robot's voice.** Speech DimOS sends is played on the robot's speaker in the order sent, and the host
    reports whether a speaker is connected; a speaker that drops is reconnected without a restart.
12. [gFa] **The robot's ears, kept in the house.** Sound from the robot's microphone reaches DimOS on the laptop over
    the house network and nowhere else; the host keeps no recordings ([zGo]).
13. [GTG] **The robot's eyes never flash.** The host shows the eye lights DimOS asks for, but every change of
    brightness or colour is slow and smooth whatever it is asked, at start-up and on any fault included: no step
    change, and a full swing from dark to full takes at least 2 seconds ([0Py]).
14. [OmH] **A card becomes a host in one step.** A blank SD card put in the laptop becomes, by one command, a card
    that boots straight into a working host for the chosen profile: no other step on the laptop or on the Pi, and
    nothing downloaded once the laptop holds its copies of the system and its packages.
15. [H9j] **Profiles, switched quickly.** The host runs one of a set of profiles (today only Houndkeeper), each the
    services and settings for one use; a running host changes profile in under a minute without reflashing, and a
    reflash with another profile gives a clean baseline of that profile. A quick patch of the running software stays
    possible ([871]).
16. [ObX] **The card is safe to write.** The command writes only to a removable card the user names, never to a disk
    the laptop is using, and shows what it will erase before it does.

## Non-functional requirements

- Wrist video: at least 15 frames a second at 640 x 480, reaching DimOS within 150 ms of capture at p95, over the
  house Wi-Fi with the robot within 5 m of the access point (proposed; confirm, then measure).
- Arm commands: from DimOS to the arm host within 50 ms at p95 under the same conditions, well inside the 300 ms stop
  ([41H]) (proposed).
- Power-up to armable: under 60 s (proposed).
- An update applied, or rolled back, in under 2 minutes (proposed).

## Failure behavior

- If the link to DimOS drops, the arm stops within 300 ms and disarms ([41H], [3dt]); the host keeps running and
  waits for DimOS to arm it again.
- If the under-voltage flag or thermal throttling appears, the host reports it and refuses to arm until it clears.
- If a release fails its start-up check, the previous release comes back and the failure is reported; the arm stays
  disarmed throughout.

## Decisions

1. [Ezx] **First hardware: a Raspberry Pi 3 Model B already on hand** (also recorded in the maker-arm-adapter spec):
   1 GB RAM, a hardware H.264 encoder, 2.4 GHz-only Wi-Fi. A 2 GB Raspberry Pi 4 if it falls short. The criteria do not
   depend on the model.
2. [yAa] **Stock Raspberry Pi OS Lite (64-bit), set up by the card's first boot ([mX4]), not a custom OS image.**
   The SSH provisioning script this decision first chose is removed (28 Oct 2026): it took the Pi online, against
   [DzZ], and the card command ([OmH]) replaced it. The 3 Model B runs the 64-bit OS (observed 2026-09-25 in
   Raspberry Pi's OS documentation; recheck on first install).
3. [WHV] **Services under systemd; releases as side-by-side directories with a "current" link** (proposed): an update
   unpacks the new release beside the old, switches the link, and restarts; rollback switches the link back. Nothing
   is upgraded in place.
4. [uuL] **Packages come from a mirror on the laptop** (proposed): the laptop holds the OS packages and Python wheels
   each release needs, so building and updating need no internet ([DzZ]).
5. [UcX] **Power-cut safety by a read-only root filesystem** with writable state kept small and on its own partition
   (proposed; Raspberry Pi OS ships an overlay-filesystem option). SD cards corrupt under power loss; the arm's pack
   can be cut at any moment.
6. [jAM] **The eye lights are WS2812 LEDs driven from the SPI pin (GPIO 10), not the PWM pin** (proposed, 2 Oct
   2026). Hard-won elsewhere: the usual WS2812 driver on a Pi uses the PWM hardware, which the Pi's analogue audio
   also uses, so the two fight; SPI leaves audio alone. The LEDs' data line takes the Pi's 3.3 V directly at short
   lengths (est.; a level shifter if they flicker).
7. [c5z] **The speaker is Bluetooth (A2DP), named on the card (`--speaker-mac`) and reconnected by the host; the
   microphone is USB**. The pairing by the removed provisioning script is gone with it (28 Oct
   2026). The Pi 3 Model B has Bluetooth built in; a battery Bluetooth speaker adds no wiring and no load on the
   robot's pack. Chosen over an I2S amplifier and microphone on the Pi's header: more wiring, and at 3 W no louder
   than the Go2 Pro's own speaker.
8. [cPu] **Commands and reports over the house network as small JSON messages** (proposed): DimOS sends speech, eye
   and arm commands to the host over HTTP; the host sends its report once a second over UDP to the laptop, and the
   microphone as 16 kHz 16-bit mono over UDP. One transport for everything until DimOS's own network layer runs on
   the host.

9. [mX4] **First boot is cloud-init** (forced: Raspberry Pi OS Lite 2026-09-15 (Trixie, arm64) sets itself up on first
   boot from cloud-init's `user-data`, `network-config` and `meta-data` on the boot partition; observed 2026-10-02 in
   the image itself). The card command writes those, plus a first-boot script, the release and the packages, onto the
   boot partition; `meta-data`'s instance id is new on every card, so each card runs its first boot.
10. [XzS] **Packages kept on the laptop, fetched by the laptop itself** (`arm_host/fetch_packages.sh`, against the
    image's own apt sources and archive keys, pinned by [iwZ]). Harvesting them back from a Pi that had been online,
    first chosen here on 2 Oct 2026, is removed (28 Oct 2026): it needed the Pi on the internet, against [DzZ].
    Chosen over an apt mirror or proxy on the laptop ([uuL]): nothing to run on the laptop. The system image is
    downloaded once and kept by its checksum.
11. [tDE] **Profiles are settings, not separate images** (proposed): one release carries every profile's code; a
    profile is a file naming the services that run and their settings (host.json), so switching is a rewrite and a
    restart.
12. [iwZ] **The card's packages are pinned by a lock in the repository** (proposed, 18 Oct 2026):
    `arm_host/pi-packages.lock.json` names the system image, first boot's package list and every package file's checksum. Fetching writes it; a card
    is never made from package files it does not name; the on-Pi check compares what is installed with it. The
    archive drops superseded versions, so the laptop's copies are the pinned set and the lock is how a card is known
    to carry them ([UYK]). A release is built byte for byte the same from one commit, and carries a manifest of its
    files.
13. [izZ] **Releases survive a power cut without a read-only root** (proposed, 18 Oct 2026): a release is synced to the
    card before the link points at it, and the switch is synced after. A second link, `known-good`, names the last
    release that passed its check; at boot, before the daemon starts, a current release that is not the known-good one
    (an activation the cut interrupted) is switched back, by the known-good release's own code ([27n]). The read-only
    root of [UcX] needs Raspberry Pi OS's `overlayroot` package, which the image does not carry (observed 2026-10-18
    in the 2026-09-15 image's package list): a new package on the card, so it waits for a decision on new dependencies ([eK3]). Until then the file
    system rests on ext4's journal and the boot's repair (`fsck.repair=yes`).
14. [LSY] **The clock is kept by systemd-timesyncd, from the router** (proposed, 18 Oct 2026): it is in the image
    (observed 2026-10-18, version 257.13 in the 2026-09-15 image); chrony ([ay4]) is not, and would be a new package on
    the card, so it waits for a decision on new dependencies ([eK3]). The card command sets the router's address on the robot network (from
    `real/.env`) as the only source, with no fallback servers on the internet ([DzZ]). The on-Pi check passes an offset
    to the laptop within 10 ms plus half the quickest round trip of 20 exchanges: Wi-Fi's jitter, well inside the
    30-150 ms offsets being calibrated ([ay4]).
15. [Uyb] **The on-Pi checks are one command from the laptop** (proposed, 18 Oct 2026): `arm_host/pi_check.sh <host>`
    runs the Pi's half over SSH, from the laptop's own copy of its code ([ZyY]), and the laptop's half (a port scan, a password login
    refused, the clock offset, the reports received), and keeps the record under `real/local/` (it holds the house's
    addresses). Each check is a judge over what a command printed, so the judges are tested on the laptop; what needs
    a person (the power cuts, a bench supply, a scan from the guest network) the record lists as manual.
16. [ZyY] **The services run as their own account; root's code is root's alone** (proposed, 18 Oct 2026, from a
    security review before first boot): the host's services run as `hkd`, a system account with no login shell, no
    sudo, no SSH key and only the device groups the daemon uses (SPI, GPIO, sound, Bluetooth, video for `vcgencmd`,
    dialout for the arm's serial link), confined by systemd (no new privileges, no capabilities, the system read-only).
    `hk` is the admin only: a key-only SSH login (password locked, the only account SSH lets in), keeping
    sudo, and no service runs as it. Everything root runs or reads (`/opt/houndkeeper` and every release, the
    `current` and `known-good` links, `/etc/houndkeeper`, the unit files) is owned by root and written only by root:
    first boot unpacks the release as root and installs the units from it only once it is root's; a push streams the
    release over SSH and root unpacks, renames and activates it. Root runs Python only isolated (`python3 -I`, never
    with a `PYTHONPATH` or from a directory another account can write); the boot-time recovery stays root's, since it
    switches links in root's tree, and runs from the known-good release. The on-Pi check does not trust the Pi's
    installed code: it ships its own modules into a fresh directory root makes, runs them there and removes it.
    Chosen over the earlier layout (the daemon as `hk`, which has sudo, and the tree owned by `hk`), where taking over
    the daemon was taking over root.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [UYK] (rebuilt from the repository) | `tests/arm_host/test_rebuild_and_power_cut.py` (the lock names every package file by checksum; a card is refused packages the lock does not name; the lock in the repository is the pinned image's; two builds of one commit are the same file, with a manifest) and `tests/arm_host/test_pi_check.py` (installed packages judged against the lock; one fingerprint for two hosts from one commit); on the Pi: two cards from one commit, `arm_host/pi_check.sh` on each, then `pi_check compare` | Partial: passing on the laptop (18 Oct 2026); the lock written from the packages fetched 3 Oct; on-Pi check written, not yet run |
| [BRx] (ready from power-up, disarmed) | `tests/arm_host/test_rebuild_and_power_cut.py` (every profile service is a unit that restarts and starts at power-up; host.json names them, first boot enables each) and `tests/arm_host/test_pi_check.py` (services enabled and running, a killed daemon back as a new process, the arm armed fails); on the Pi: `arm_host/pi_check.sh <host> --kill-test` after a power-cycle, with the fake arm link ([wcJ]) | Partial: passing on the laptop (18 Oct 2026); on-Pi check written, not yet run; the arm's armed state and its disarm while its service is down are not built (the adapter reports neither), so the check says "not built" |
| [871] (updated only on purpose) | push a release; push a deliberately failing one and check the rollback; check no update applies with the arm unstowed | Not yet written |
| [DzZ] (no internet) | `tests/arm_host/test_offline_card.py`: first boot installs only from the card and stops rather than go online; the card is never made without its packages, which the laptop fetches with `arm_host/fetch_packages.sh` against the image's own sources, installed-package list and archive keys; then build and update with the laptop's route to the internet removed | Partial: 3 passing (3 Oct 2026); fetch_packages.sh run once (46 packages, 14 MB, every repository signature checked); not yet on a Pi |
| [Ot6] (video stays in the house) | `tests/arm_host/test_pi_check.py` (no picture, video or sound file kept; every connection open or being opened is with the laptop, the time source or the host itself, an attempt elsewhere fails); on the Pi: `arm_host/pi_check.sh <host>` during a session; the router's log as for [gFa] | Partial: passing on the laptop (18 Oct 2026); on-Pi check written, not yet run; the wrist camera stream is not built, so its check says "not built" |
| [aVJ] (one clock) | `tests/arm_host/test_rebuild_and_power_cut.py` (the card sets the router as the only time source, no fallback on the internet, one address or refused; first boot installs it; the report says whether the clock is synchronised) and `tests/arm_host/test_pi_check.py` (synchronised to that source; the offset to the laptop from the quickest of 20 exchanges, decision [LSY]); on the Pi: `arm_host/pi_check.sh <host> --only aVJ` an hour after boot | Partial: passing on the laptop (18 Oct 2026); on-Pi check written, not yet run |
| [4WJ] (reports its state) | `tests/arm_host/test_pi_check.py` (reports from the Pi's address at least once a second through a 5 s window, every field there); on the Pi: `arm_host/pi_check.sh <host> --only 4WJ` (the laptop listens on the report port); under-voltage raised with a bench supply (manual, listed in the record); the report seen in DimOS | Partial: passing on the laptop (18 Oct 2026); on-Pi check written, not yet run; DimOS does not read the report yet |
| [27n] (power cut) | `tests/arm_host/test_rebuild_and_power_cut.py` (a release synced before the link points at it and the switch synced after; the release that passed its check linked as known-good; an activation cut short undone at boot by the known-good release's own code; settings synced before their rename) and `tests/arm_host/test_pi_check.py` (current is the known-good release and matches its manifest; the root repaired at boot); on the Pi: cut power 20 times at random points, some mid-update, `arm_host/pi_check.sh <host> --only 27n` after each boot | Partial: passing on the laptop (18 Oct 2026); on-Pi check written, not yet run; the read-only root (decision [UcX]) not built: it needs a package not in the image (decision [izZ]) |
| [CIu] (locked down) | `tests/arm_host/test_pi_check.py` (sshd's effective settings keys only and no root; no account with a usable password; no default account; only SSH and the API listen on the network; only those answer the laptop's scan; a password login refused; the services' account with no login, no sudo and no key; the profile's services run as it; root's code and settings owned by root and writable by no other account) and `tests/arm_host/test_privilege_boundary.py` (decision [ZyY]: no unit runs as the admin, the daemon confined, the recovery root's with isolated Python; first boot makes the services' account and give the tree only to root, installing units only after; root's scripts and anything under sudo run isolated Python; a push writes the tree only through sudo; the card gives keys and sudo only to the admin; pi_check.sh run end to end through a stand-in ssh and sudo runs the laptop's own code from a fresh directory and removes it); on the Pi: `arm_host/pi_check.sh <host>` from the house network, and `--expect-unreachable` from a machine on the guest network | Partial: passing on the laptop (18 Oct 2026); on-Pi check written, not yet run |
| [MA0] (whole without the arm) | `tests/arm_host/test_hostd.py` (the host's report with no arm); on the Pi: boot with no arm | Partial: passing on the laptop (2 Oct 2026); the on-Pi check not yet run |
| [u9G] (the robot's voice) | `tests/arm_host/test_hostd.py` (speech played in order); on the Pi: pair, play, switch the speaker off and on | Partial: passing on the laptop (2 Oct 2026); the on-Pi check not yet run |
| [gFa] (ears kept in the house) | `tests/arm_host/test_hostd.py` (microphone sent only to the laptop's address, nothing written to disk); network capture on the Pi | Partial: passing on the laptop (2 Oct 2026); the on-Pi check not yet run |
| [GTG] (eyes never flash) | `tests/arm_host/test_eyes.py` (no step, and the 2 s minimum swing, under any sequence of commands, start-up and a fault) | Partial: passing on the laptop (2 Oct 2026); the on-Pi check not yet run |
| [871] (rollback, on the laptop) | `tests/arm_host/test_release.py` (a release that fails its check is rolled back) | Partial; 3 Oct 2026: `tests/arm_host/test_report_and_updates.py` checks deploy.sh refuses a push without the operator's --robot-still; not built, so marked xfail there: the host itself refusing a release while the arm is out (activate takes no arm or robot state) and anything turning off apt's own daily timers (never updating itself) |
| [OmH] (one step) | `tests/arm_host/test_card.py` (the first-boot files for a profile: a user with the laptop's key, no password, Wi-Fi, a new instance id, the offline package path); on the Pi: a blank card, one command, boots working | Partial: passing on the laptop (2 Oct 2026); a card not yet booted on the Pi |
| [H9j] (profiles) | `tests/arm_host/test_card.py` (each profile's host.json and services); on the Pi: switch profile, timed | Partial: passing on the laptop (2 Oct 2026); a card not yet booted on the Pi |
| [ObX] (safe to write) | `tests/arm_host/test_card.py` (a fixed disk, a mounted system disk and a too-large disk refused; a removable card accepted) | Partial: passing on the laptop (2 Oct 2026); a card not yet booted on the Pi |
| [4WJ] (report contents) | `tests/arm_host/test_health.py` (the Pi's throttle and under-voltage flags read right); `tests/arm_host/test_rebuild_and_power_cut.py` (each service the profile runs, and whether the clock is synchronised) | Partial; 3 Oct 2026: `tests/arm_host/test_report_and_updates.py` checks the values (release, each service's state, CPU temperature, throttle and under-voltage flags, arm, speaker; None off the Pi) and the daemon's loop sending JSON to the laptop's address at least once a second; 18 Oct 2026: the services are the profile's, not only the daemon; not covered: the report arriving in DimOS |
