# Phone cockpit

**Type:** Capability spec · **Status:** Draft · **Source:** dog-first-boot [wZV] · **ID:** [Swt]

## Purpose

Drive and stop Barrymore from a phone, and run the calibration procedures (`real/calibration/`) watching their numbers
live, without the Unitree app and without the phone joining the robot network (dog-first-boot [wZV]). Built on DimOS's
Cockpit rather than beside it, to avoid redoing work, through extension interfaces general enough to offer upstream,
with our own opinionated pages loaded through them: the opinionated pages stay ours, the extension interfaces can go
upstream.

## Acceptance criteria

1. [Eir] **Panels from outside Cockpit** (upstream). A blueprint's panel can name a JavaScript module; the relay serves
   modules from an extensions folder it is given; the Cockpit, meeting a panel kind it doesn't know, loads that module,
   which registers its panel through the interface the built-in panels use (the panel's spec, its channels, the send
   surface). The manifest stays v1 (additive params only). A module that fails to load shows as an error panel; the
   others keep working.
2. [Xzt] **Touch driving** (upstream). On a touch screen the drive panel offers a thumb joystick and an always-visible
   stop, sending the same twist channel with the same limits; lifting the thumb sends zero.
3. [Mj2] **Silence stops the robot.** When the page stops sending (the phone locked, Wi-Fi lost, the browser closed or
   killed), the robot stops within 0.6 s; measured, not assumed.
4. [rhP] **The phone reaches it privately.** Over Tailscale, to the laptop's relay, with a publicly trusted certificate
   and a viewer login; never the hosted relay or hosted teleop (the camera never leaves the house, [zGo]).
5. [vRJ] **Procedures page** (ours, an extension). Lists the procedures; walks one step by step; shows its numbers live
   as the recording runs; at the end shows the result beside the value in `params/robot.yaml` and writes it (source
   measured) only when the user confirms.
6. [F2q] **Status page** (ours, an extension). The battery's charge and power draw, the camera link (frames dropped,
   stalls), and whether the robot network's isolation check last passed.
7. [CSu] **Upstream pieces stay upstream-shaped.** [Eir] and [Xzt] live in our DimOS fork as their own commits, with
   their own tests, in DimOS's style and with nothing of houndkeeper in them; everything of ours lives in houndkeeper.

## Decisions

1. [Kq0] **Extensions over a separate app**. The Cockpit already has the connection, login, video,
   map, layout and a deadman stop (300 ms); its manifest passes unknown panel kinds through. A loader for them is the
   only missing piece, small and general; our pages are then plain extensions. Considered: our own page on DimOS's Web
   SDK (redoes the layout, login and video), forking Cockpit's own panels (unshareable).
2. [gwa] **Silence halts in place; the stop button halts** (proposed, 3 Oct 2026). Zero velocity, the robot standing:
   lying down mid-stride on a rug is riskier than standing still. Lying down is its own command, after a halt.
3. [4uq] **Tailscale for the phone**. The laptop and the phone are on the tailnet: it crosses
   the house's Guest/Private split, and `tailscale cert` gives a certificate phone browsers trust (WebTransport needs a
   secure page; a private CA works in desktop Firefox only). Considered: a domain with a DNS-challenge certificate
   (a house address in public DNS), the laptop alone.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [Eir] (panels from outside) | the fork's branch feat/cockpit-extensions: cockpit `extensions.test.tsx` (7: load order, the subscription gate, no listing, paths outside /ext/ refused, a failing module named on unknown panels, built-ins not replaceable, the panel toolkit), relay `server_test.ts` (listing and serving, none without the flag, bad dir), the bridge's Python tests | Passing: 161 cockpit, 16 relay, 191 bridge tests (3 Oct 2026) |
| [Xzt] (touch driving) | the fork's branch feat/cockpit-touch-teleop: `TeleopPanel.test.tsx` (9) and the SDK's `teleopMachine.test.ts` (5): drags give the keys' twists, release and STOP send zero, a hidden page disarms | Passing: 163 cockpit, 343 SDK tests (3 Oct 2026); not yet on a phone |
| [Mj2] (silence stops) | sim: `real/tests/deadman_sim.sh` drives from a headless phone-sized browser, then the page falls silent four ways (killed, hidden, frozen, closed); odometry must show the turn stopped within 0.6 s (`real/tests/README.md`); `tests/test_deadman_check.py` checks the analysis | Partial: with the phone cockpit's 200 ms deadman, 3 runs on a quiet machine, 12 of 12 cases stopped in 0.11-0.53 s (hidden, frozen, closed, killed; 3 Oct 2026). At DimOS's default 300 ms, 4 of 14 overran (0.61-0.91 s), hence the 200; not covered (test audit, 3 Oct 2026): the 12 of 12 is a manual run with no committed results file; only the analysis is under test |
| [rhP] (privately) | the relay bound to the Tailscale address with certificate and auth; the phone connects; nothing listens on the house addresses | Planned |
| [vRJ] (procedures page) | `tests/test_cockpit_data.py`: a run from the page's commands on synthetic streams (frame drops, the taped square with the entered tape, walking power), nothing written before confirm, every run logged | Partial: 4 passing (3 Oct 2026); the page itself not yet driven against the sim |
| [F2q] (status page) | `tests/test_cockpit_data.py` (link numbers from the battery, odometry and frames); sim: the page against the running sim | Partial: the numbers pass; the page not yet against the sim |
| [CSu] (upstream-shaped) | the fork's two commits build and pass DimOS's own tests alone; `git grep -i houndkeeper` in them finds nothing | Planned |
