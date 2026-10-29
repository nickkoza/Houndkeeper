# Dog first boot

**Type:** Capability spec · **Status:** Draft · **Source:** the move to dog first boot and sim to real to sim, on a
network with no internet, which is easier said than done; principles [zGo], [eNx], [Feg] · **ID:** [MTv]

## Purpose

The real Go2 Air's first power-up, from the box to its first recording through DimOS, done safely and leaving the
robot on a network it cannot leave. It opens sim-to-real calibration ([V3o]): nothing can be measured until the robot
stands on its own segment and DimOS can drive it.

## Acceptance criteria

1. [B2V] **Online once, on purpose.** The robot is activated and its LAN key obtained in one deliberate online
   session, whose date and the firmware version it found are recorded. After it, the robot never again reaches the
   internet.
2. [hNo] **A robot segment with no way out.** The robot, the arm host and the laptop share a network segment with no
   route to the internet. The laptop keeps its own internet over a separate link and never forwards between the two.
3. [VVJ] **Isolation shown, not assumed.** A test from the segment shows no name resolves and no connection leaves it,
   and every attempt the robot makes to call out is logged and listed.
4. [3il] **Firmware pinned.** The firmware found at activation is recorded; nothing updates it except a deliberate
   change, whose versions are recorded and whose dependent sims are re-run ([7Xn]).
5. [x20] **One clock on the segment.** Everything on it takes its time from the one source the laptop also follows;
   data are stamped at their source ([Feg]).
6. [U4p] **A stop at hand before the first stand.** Two ways to stop the robot (one that works without DimOS, one
   through it) are each tested before it first walks, in a space cleared of people, pets and stairs.
7. [4R9] **DimOS drives the real dog.** Over the segment, DimOS stands, walks, turns and lies the robot down, and its
   lidar, camera and odometry streams arrive at the rates the sim assumes.
8. [buA] **The first recording stays home.** A first recording is saved on the laptop, and nothing of it leaves the
   house ([zGo]).
9. [Ad4] **Credentials kept off the record.** The robot's LAN key and any account token live only on the laptop, out of
   the repository; the account password is stored nowhere by this project.
10. [Fhn] **The house's network stays out of the repository**: router information lives in an
   uncommitted `.env`, never in the repo. No file git would carry (tracked, or
   untracked and not ignored) holds a value from `real/.env`: an address or the house's subnets, a network name, a
   MAC, a key or password, the router's user. Tests and docs use placeholder addresses (10.9.x.x, `*.example`). A test
   reads `real/.env` where it exists and fails on any such value; found once (3 Oct 2026: the house subnet in two new
   tests, never committed).

## Decisions

1. [Ozg] **DimOS reaches the Go2 over local WebRTC, which on current firmware needs a per-robot key from Unitree's
   cloud.** Forced. DimOS's Go2 connection uses the WebRTC "LocalSTA" method (robot and laptop on the same network,
   robot as a Wi-Fi client); on Go2 firmware 1.1.15 and later the LAN handshake (data2=3) needs a per-device AES-128 key,
   which is fetched once from Unitree's cloud with the user's account and the robot's serial number:
   `unitree-fetch-aes-key --email <account> --sn <serial>`, then set as DimOS's `unitree_aes_128_key` (a secret config
   field). Observed 2026-10-02 in the DimOS fork (`dimos/robot/unitree/connection.py`, `dimos/core/global_config.py`)
   and its `unitree_webrtc_connect` package (`_cli.py`); the package's own documentation adds that the key is needed in
   both local modes (robot as Wi-Fi client, "STA-L"; robot as its own access point, "AP") and that the cloud is used
   only to obtain it: the connection itself runs locally, with no internet (legion1581/unitree_webrtc_connect README,
   read 2026-10-02). This is why [B2V] has an online window at all, and why it can be the only one.
2. [ZaY] **The segment is a separate network on the house router, closed by its firewall** (2 Oct 2026; the house
   router can host a second network with its own subnet and firewall rules). A second wireless network, 2.4 GHz (the
   arm host's Pi 3 Model B has 2.4 GHz only), on its own bridge and subnet; at the top of the firewall, anything from
   that subnet that is not to the laptop is logged and dropped, and only the laptop may open connections into it; the
   robot's DNS points at a logging-only resolver on the laptop that answers nothing (so the names it tries are listed,
   [VVJ]); time from the router, which the laptop also follows ([x20]). The laptop stays on the house network: no
   cable, no second adapter. The router's make, the network names, the addresses and the router's configuration are
   kept out of the repository (`real/.env` and `real/local/`, gitignored; `real/.env.example` says what goes there):
   the configuration is kept as a file there, re-applied from it and diffed against the router's export at every
   change, not clicked. Chosen over a separate travel router: the house router already does everything needed, and
   its risk, one wrong rule opening the way out, is covered by that file and by the isolation test run at every
   change. Also considered: the robot's own access point (AP mode; no router, but the arm host and laptop would hang
   off the robot's Wi-Fi and nothing would log what the robot tries), the fallback for a first test.
3. [zAo] **If the robot must go online, it does so from the robot segment, through a contained window** (proposed,
   3 Oct 2026; prepared, not applied). The robot joins the segment first, closed, and stays there long enough for the
   logging resolver to list the names it tries; only then is a window opened, and only for the robot's fixed address:
   - **an allow-list of Unitree's names, learned, not assumed**: only the Unitree names from that list that binding
     needs are allowed; the router resolves them into an address list (it follows their CNAMEs) and the laptop's
     resolver answers those names, and only those, from the router's resolver, so the robot and the firewall see the
     same addresses; every other name still gets "no such domain";
   - **HTTPS only** (TCP 443) to those addresses, and their replies; nothing new may come in. The names found in
     research (`global-robot-api.unitree.com`, and `robot-api.unitree.com`, from the WebRTC package's
     `unitree_cloud.py`, read 2026-10-03) resolve to a shared CDN, so an address alone is not trusted; Unitree's
     telemetry seen on the G1 goes to raw addresses on port 17883 over MQTT (arXiv 2509.14139, Sep 2025), which the
     port alone keeps out;
   - **a bandwidth cap** on the robot's address (256 kbit/s each way: a firmware image cannot come down in the window);
   - **a time limit**: the router closes it by itself after 30 minutes, the allow-list's entries time out at the same
     time, and a clean undo removes everything the window added;
   - **everything else still logged and dropped**: the window's accept sits below the rule that logs every new
     connection out, so what the robot reached is in the log too, and the isolation check reports the window open
     while any of it is there.
   Then read and record the firmware version, refuse updates, close the window, and confirm the isolation check passes
   again. The robot's first stand happens on the segment. Chosen over activating on the house's guest network, which
   gives the robot the whole internet, unlogged, with no cap or clock; the commands are local
   (`real/local/online-window.rsc`), the how-to generic (`real/bringup/first-boot.md` step 1).

4. [wZV] **No Unitree app: DimOS's Cockpit drives and stops the robot, from the laptop; the phone stays off the robot
   network**, from first boot: the phone is preferably not on the robot network, and the Unitree app is avoided. The
   Go2 Air ships without a handheld remote (an optional accessory). DimOS's
   Cockpit (its web app and relay, run on the laptop with a local relay; never the hosted relay or the hosted teleop,
   which send the camera out of the house) gives video, the map, driving and an e-stop, with a deadman: 300 ms without
   the browser's commands and the robot stops. The phone reaches it on the house network and talks only to the laptop;
   a page of our own on DimOS's Web SDK adds a touch joystick, a large stop, and the calibration procedures run live
   (real/calibration/). A gamepad on the laptop is the hardware stop; the robot's power button the one that needs no
   software. The one step the app may still be needed for is binding the robot to the Unitree account (the LAN key is
   looked up among the account's bound robots, `device/bind/list`): asked of Dimensional first (they may bind it, or
   know a way without the app); else one Bluetooth session, the robot's online window contained as below.

## Open questions (to answer on the day; each becomes a decision)

- Does the robot arrive bound to an account (Dimensional's, or for transfer to ours), and can it be bound without the
  Unitree app?

- Does the robot stand and walk with no cloud at all, or only after reaching it once? (The WebRTC package says the
  local connection needs no internet once the key is held; to confirm on this robot.)
- Does the robot keep retrying outbound connections, and to where (the segment's DNS log answers this)?
- Does the LAN key change with a firmware update or a re-activation?
- Which Wi-Fi bands does the Air join, and does it hold its clock with no internet time?
- Is the Air's firmware at or past 1.1.15 out of the box (the key) and can it stay where it is?

## Tests

| Criterion | Test | Status |
|---|---|---|
| [B2V] (online once) | `real/bringup/first-boot.md` step 1, its record filled in | Planned |
| [hNo] (a segment with no way out) | `real/bringup/robot-network.md` (the router set up from the local config; its export diffed against it) | Planned |
| [VVJ] (isolation shown) | `tests/test_isolation_check.py`, `tests/test_robot_logs.py` (the router's rules checked as designed, read-only, every 5 minutes: `real/bringup/isolation_check.py`) | Partial; not covered (test audit, 3 Oct 2026): the tests parse saved router output and resolver arguments; no name is shown failing to resolve, no connection blocked |
| [VVJ] (isolation shown, on the segment) | `real/bringup/robot-network.md` "Prove it": no DNS, no outbound from the arm host; the router's DNS and firewall logs after a day with the robot on | Planned |
| [3il] (firmware pinned) | first-boot record; the version checked against it at every session start | Planned |
| [x20] (one clock) | the arm host's and the robot's clocks against the laptop's after an hour | Planned |
| [U4p] (a stop at hand) | `real/bringup/first-boot.md` step 3: each stop tested lying down, then standing | Planned |
| [4R9] (DimOS drives the dog) | `real/bringup/first-boot.md` step 4: stand, walk, turn, lie down; stream rates logged | Planned |
| [buA] (the first recording stays home) | the recording's location; the router's logs show nothing left during it | Planned |
| [Fhn] (the house's network out of the repo) | `tests/test_no_network_specifics.py`: every value in `real/.env` (and its addresses' /24 subnets) searched for in the files git would carry | Passing: 1 (3 Oct 2026), skipped where there is no `real/.env` |
| [Ad4] (credentials off the record) | `git grep` for the key and token finds nothing; DimOS reads the key from its local config | Planned |
