# Calibration: the real robot's numbers into params/robot.yaml

Sim-to-real calibration ([V3o]): every `go2_air` parameter still marked `estimate`, `unknown` or `sim` has a
procedure here ([4hm]). Running it puts a measurement in `params/robot.yaml` (`source: measured`, the date, and how;
`measure.py --write` does it for the ones a recording answers directly), and every sim that depends on the parameter
is then re-run with the result kept beside the estimate's ([7Xn]).

Two kinds of measurement:

- **Direct.** The parameter itself, off the real robot: a tape measure, a scale, a stopwatch, or a recording read by
  `measure.py` (frames that never arrive, a wall's range spread, odometry against a taped walk).
- **Matched.** What the parameter does after the camera's processing and DimOS have had it (a frame's effective noise,
  the lidar points DimOS passes on). These are measured the same way on a real night and on a sim night of the same
  scene (`measure.py <command> real.db --sim sim.db`), and the sim's parameter is tuned until the two agree:
  sim-to-real-to-sim.

`measure.py`'s analyses are checked on known inputs (`tests/test_measure.py`). On a sim night (house 95, night 1,
2 Oct 2026) they gave back the sim's own settings where those are comparable: heading drift 0.54 deg per root metre
(the sim's 0.5), the frames' lag behind odometry from the robot's turning 0.10 s (the sim's 0.15 s frame delay less
its 0.03 s odometry delay: 0.12 s), a scale error of -1.1% (one night's draw of the sim's 2% spread).

## The gap report ([KbT])

`measure.py gap <real memory.db> [--box x0 x1 y0 y1 z0 z1] [--truth walk.csv] [--out report.md]` runs every analysis
that applies on a real recording and sets each beside the sim's value and `params/robot.yaml`'s, in a markdown table:
quantity, kind, sim, real, file, gap, and what to do (a direct quantity: the parameter to write and the command that
writes it; a matched one: the sim parameter to raise or lower and by what factor). The sim's values come from
`sim_reference.json` (or `--sim <sim memory.db>` for a sim night of the same scene, measured on the spot), made by
`measure.py simref sim/house/runs/nights_s95 sim/house/runs/nights_s70 sim/house/runs/nights_s72 sim/house/runs/nights_s77`:
the analyses on house 95's nights 1-2 and three tuning houses' (never a hold-out house), with the recordings, dates and
the parameters in force. The report says when the parameters have changed since the reference was made: re-make it.

The sim's frames and scans are stamped on the wall clock while the sim runs at ~0.8 of real time, so its stream
(drops, stalls), turning lag and per-second lidar rates are not comparable with the real robot's: the report marks
them, and the real stream is compared with the file. The sim's own values (3 Oct 2026, eight nights): effective read
noise 0.0085 (0.0064-0.014; the file's raw 0.004 after gain, noise reduction and JPEG), shot noise not resolved
(~0), 5,340 lidar points a message at 1.5 messages a second, a flat wall's spread 1.8 cm (of the 2 cm drawn), heading
drift 0.53 deg per root metre (the file's 0.5).

## The parameters and their procedures

| Parameter (`go2_air.`) | Now | Procedure | Kind | Re-run after |
|---|---|---|---|---|
| `head_camera_pitch_deg` | estimate 0 | [head_camera.md](head_camera.md) 1-3 | direct | floor pictures, baselines |
| `head_camera_pos_m` | sim | [head_camera.md](head_camera.md) 4 | direct | floor pictures, baselines |
| `head_camera_mount_error` | estimate | [head_camera.md](head_camera.md) 5 (`calibrate_mount.py` on a real night) | direct | the confirmer, floor pictures |
| `frame_delay_s` | estimate | [timing.md](timing.md) 1-2 | direct | floor pictures, detection bench |
| `odom_delay_s` | estimate | [timing.md](timing.md) 3 | direct | floor pictures |
| `video_stream` | estimate | [timing.md](timing.md) 4 (`measure.py stream`) | direct | detection bench |
| `odom_drift` | estimate | [odometry.md](odometry.md) | direct | floor map, pose graph, nights |
| `camera_imperfections` | estimate | [camera.md](camera.md) | matched | detection bench, floor map error |
| `lidar` (range spread, rate) | vendor | [lidar.md](lidar.md) 1-2 | direct / matched | floor map, shape solve |
| `lidar_dropouts` | estimate | [lidar.md](lidar.md) 3 | matched | mapping, costmap |
| `walk_power_W` | estimate | [power_and_pose.md](power_and_pose.md) 1 | direct | battery coverage ([7Xn]) |
| `crouch_height_m` | unknown | [power_and_pose.md](power_and_pose.md) 2 | direct | arm reach, tip limits |
| `mass_kg` (check) | vendor | [power_and_pose.md](power_and_pose.md) 3 | direct | payload, tip limits |

The arm's are in [jaw_pads.md](jaw_pads.md) and [arm_gravity.md](arm_gravity.md) (`maker_arm.gravity_ff_fraction`) and come with the Maker Arm.

## Day one, in order (after first boot, `real/bringup/first-boot.md`)

All on the robot network, the stops tested first ([U4p]). About two and a half hours, most of it the robot standing or
walking a taped course while it records; then the first two nights.

### First: the five the launch numbers lean on ([GWP])

A sensitivity study (docs/research/sensitivity-2026-10-13.md) made each sim estimate worse on six tuning nights and
watched the launch false alarms ([tTD]; the lead keeps 39/48 at 11.2 a night when replayed live). These five moved them
most, in this order, so they come first and the mount, lag and drift are checked again on the second night. Each has a
**pass band**: inside it, the false alarms stayed within the replays' own spread (about 1 a night). The **fails** step
is the smallest the study ran that raised them; between the two is untested and counts as outside, never as a pass
(the sim is never kinder, [x3f]). `measure.py` prints `day one ([GWP]): pass | outside | fails` against these bands
(`DAY_ONE_BANDS`, the one place they are kept); the compression is the one judged by eye (4).

1. **The head camera's mount, and whether it stays put** ([head_camera.md](head_camera.md) 1-3, then 5).
   - Run: the marks and one frame (20 minutes); after the first two nights, the nightly check on each night's views
     alone (`floor_selfcal.py check`, up to half an hour of CPU each), then the two side by side with
     `measure.py mount` (the commands: head_camera.md, "Whether it stays put between nights").
   - Pass: each night within 0.1 deg of pitch (0.3 deg yaw, 5 mm height) of the robot's calibration, and night 2
     within the same of night 1.
   - Why: half a degree of pitch off the map's calibration took the false alarms from 11.2 to 19.0 a night (1 deg:
     20.0). Nothing under 0.5 deg was run, so the band is the nightly check's own tolerance ([9tQ]); the tuning
     houses' checks, one mount and nothing moved, spread 0.06 deg.
   - Outside: look for the cause before recalibrating (a loose bracket, a head that flexes, a pose that is not the
     walking one); re-seat it, repeat the marks, then recalibrate through the one writer ([gJq], `camgeo.py
     calibrate`) and record both baseline nights again (their views were painted with the old mount). The launch
     numbers are not trusted on the robot until two nights pass.
2. **The frames' lag behind odometry** ([timing.md](timing.md) 1-3).
   - Run: the clock on the screen (10 minutes); turning on the spot at the end of the taped walks (5 minutes);
     `measure.py lag <db> --expect <stored lag s>` (the stored lag: `jq .clocks.robot.lag_s` on the lag file the
     real nights use, `calibration_used.txt` in the run names it). Again on night 2 (its turns, or the nightly
     check's floor lag).
   - Pass: within 20 ms of the stored lag, and the two nights within 20 ms of each other.
   - Why: 50 ms off cost 17.5 false alarms a night (150 ms: 19.7). Nothing under 50 ms was run; 20 ms is what
     `measure.py` already calls agreement for the lag, and the nightly check warns past 10 ms.
   - Outside: a turning correlation under 0.7 means turn again. Otherwise store the measured lag (timing.md 2,
     `calibrate.store_lag`) and repaint the baselines; two nights more than 20 ms apart point at the link (record
     where the robot works, at night, timing.md).
3. **Odometry's drift** ([odometry.md](odometry.md) 1-2).
   - Run: the straight 5 m and the five laps of the taped square (30 minutes);
     `measure.py odom <db> --truth real/local/taped_square.csv` (it prints the drift as a multiple of the file's
     estimate). The square again before night 2 (10 minutes).
   - Pass: heading and position drift per root metre at or under the file's estimate (0.5 deg and 5 mm per root
     metre), the five straight walks' scale errors within the 2% spread.
   - Why: twice the estimate cost 15.8 false alarms a night (four times: 16.5). The study's first day-one table
     said "within 2x"; the x2 step, finished after it was written, says not.
   - Outside: write the measured drift (odometry.md 3) and re-run the tuning nights' replays at it ([7Xn]; the
     study's `driftK` variant) before trusting the launch numbers; until then the floor comparison leans on the
     lidar's floor lock to carry more.
4. **The stream's compression on the darkest floor** ([camera.md](camera.md) 3).
   - Run: two minutes standing, then two walking slowly, over the darkest floor (15 minutes with setting up);
     `measure.py compression <db> --sim <a sim night over a like dark floor> --sheet real/local/compression.png`.
   - Pass: on the side-by-side picture, the real dark floor no flatter or blockier than the sim's own frames
     re-encoded at JPEG q50; fails at their q30 re-encode or worse. Judged by eye with the numbers beside it: on the
     sim's own frames q50 and q30 move the numbers less than two floors differ, so there is no automatic verdict
     (camera.md 3, step 5).
   - Why: q30 every frame cost 15.3 false alarms a night, most on the dark tile (13 to 28 on that night); q50 and
     q75 re-encodes cost nothing (11.0).
   - Outside: lower the sim's `jpeg_quality` (or raise the dips) until a sim night of the dark floor looks the same,
     and re-run the bench ([7Xn]); on the robot, the stream's resolution and bitrate and the Wi-Fi are what can move it.
5. **The noise** ([camera.md](camera.md) 1).
   - Run: the still minute (5 minutes); `measure.py noise <db> --sim <sim db>`.
   - Pass: the real camera's effective noise up to 4 times the sim's (as a spread).
   - Why: 4 times cost 12.3 (+1.1, within the spread); nothing larger was run.
   - Outside: scale the sim's noise by the ratio it prints and re-run (camera.md 1); untested past 4x, so treat the
     launch numbers as unknown until the replays are re-run at it.

Compression is judged after the noise agrees (grain fills what compression flattens, so a noisier camera reads kinder):
record 4 and 5 together, read 5 first.

### Then the rest

6. **Weigh and measure the robot standing and crouched** (power_and_pose.md 2-3): 10 minutes.
7. **Head camera's place** (head_camera.md 4): a tape measure, with the marks of step 1: 10 minutes.
8. **The wall and the black sofa** (lidar.md): the robot standing 1.5 m from a plain wall, then from the sofa and
   the TV: 15 minutes.
9. **The battery walk** (power_and_pose.md 1): 5 minutes standing, then 20 of walking the room on its own route,
   recorded with `houndkeeper.battery`; `measure.py power`.
10. **Two recorded nights in the demo room's real counterpart**: the real version of the sim's baseline nights, for
   the matched measurements and the gap report ([esI], [KbT]), and the second look at 1-3 above.

Then: `measure.py` on each recording, the file updated, the dependent sims re-run.
