# First boot: box to first recording

The dog-first-boot spec ([MTv]), as steps. Fill in the record at the bottom as you go; it is the evidence for [B2V],
[3il] and the spec's open questions.

## 0. Before the box is opened

- The robot segment is up and proven (`robot-network.md`), with the robot not yet on it.
- A cleared space: no stairs, no kids or pets in the room, nothing fragile at robot height ([U4p]).
- The Air's battery charging supervised the first time (a new pack; LiPo).
- DimOS on the laptop at the version the sims ran; the fork's tests pass.

## 1. The online window ([B2V], [3il], decision [zAo]): contained, and as short as it can be

Only if the robot must reach Unitree's cloud itself (binding it to the account with the Unitree app, [wZV]). The
LAN key is fetched by the laptop, not the robot, and needs no window. The commands are the local file
`real/local/online-window.rsc` (open, check, undo); the steps:

1. **Closed first.** The robot joins the robot network (`$ROBOT_SSID`; set over Bluetooth by the app from a phone that
   stays on its own internet) and gets its fixed address (`$ROBOT_IP`). Leave it there, closed, while it tries its
   names: `hk-robot-dns` lists every name it asks for (`real/local/logs/robot-dns.log`), `hk-robot-syslog` every
   connection it tries (`router.log`, including any to a raw address, which no DNS line shows).
2. **Choose the names.** From that list, only Unitree's names that binding needs; nothing else, however harmless it
   looks. Write them into the local file's allow-list, and into `real/.env` as `ROBOT_DNS_ALLOW`, with
   `ROBOT_DNS_UPSTREAM` the router's house-side address.
3. **Open.** Back up the router; on the laptop, stop `hk-robot-dns` and run `python3 real/bringup/robot_logs.py
   dns-window` (the allowed names answered by the router's resolver, all else "no such domain", all logged); paste the
   open block. The router then lets the robot's address reach those names' addresses on HTTPS only, capped at
   256 kbit/s, and closes it all by itself after 30 minutes. Note the time it went online.
4. **Check.** The check block: the window's rules sit below the rule that logs every attempt out, the allow-list holds
   only the chosen names and what they resolved to, the timer shows its next run about 30 minutes away. The isolation
   check (`real/bringup/isolation_check.py`) now reports the window open: that is expected.
5. **Bind** in the app. Read the firmware version. **Turn automatic updates off.** Do not accept an update (the cap
   makes one impractical anyway); if one is forced, close the window and decide, recording both versions.
6. **Close** at once (the undo block; the timer would anyway), stop `dns-window`, clear `ROBOT_DNS_ALLOW`, start
   `hk-robot-dns` again. The isolation check passes again. Note the time it went offline, and what the robot reached
   during the window (`router.log`), in the record.
7. On the laptop, fetch the LAN key: `unitree-fetch-aes-key --email <account> --sn <serial>`. Put it in DimOS's local
   secret config (`unitree_aes_128_key`), never in the repository ([Ad4]).

## 2. On the segment

1. The robot is on the robot network at its fixed address (`$ROBOT_IP`). `ping $ROBOT_IP` from the laptop (the laptop reaches the segment through the router: robot-network.md).
2. Open question: does the app still control it from a phone on the robot network? Record the answer.

## 3. The stops, before any walking ([U4p])

1. Robot lying down. Test the stop that works without DimOS (the app's lie-down/damp; the power button's long press):
   note which work and how fast.
2. Start DimOS against the robot (robot IP `$ROBOT_IP`, the key from step 1). Test DimOS's stop (its kill skill / a
   zero-velocity stop): note latency.
3. Standing, repeat both stops once each.

## 4. DimOS drives the dog ([4R9])

Stand, walk forward 1 m, turn, walk back, lie down, all through DimOS teleop. Log the lidar, camera and odometry
rates and compare them with the sim's assumptions (`params/robot.yaml`: frame and odometry delays, video stream).

## 5. The first recording ([buA])

A short recording (a lap of the room): saved on the laptop; the router logs read afterward to confirm nothing left.

## Record

Kept locally (`real/local/first-boot-record.md`), not in the repository: it holds the house's specifics.

| Item | Value |
|---|---|
| Date | |
| Online window (start to end; names allowed) | |
| What the robot reached in the window (router log) | |
| Firmware version at activation | |
| Update offered / applied (versions) | |
| LAN key fetched (yes/no; stored where, not the key) | |
| Robot address on the robot network | |
| App works on the segment with no internet? | |
| Stands/walks with no cloud? | |
| Names the robot tried (router DNS log, day 1) | |
| Stop without DimOS: which, how fast | |
| DimOS stop latency | |
| Stream rates: lidar / camera / odometry | |
