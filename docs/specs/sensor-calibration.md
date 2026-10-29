# Sensor calibration with Kalibr

**Type:** Capability spec · **Status:** Draft · **Source:** consumer hardware, which makes Kalibr a must across the robot and the system; principles [Feg], [sJQ], [x3f] · **ID:** [fxE]

## Purpose

Every calculation that combines two sensors needs to know when and where each sample was taken relative to the
others: the robot's own clock self-consistent ([Feg]: what matters for calculations is
knowing when all data is timestamped relative to other data). The Go2 is consumer hardware: its camera reaches the
laptop through H.264 and WebRTC and is stamped on arrival, its state on another channel, with no shared hardware
clock. The offsets and mounting between sensors therefore have to be measured, and Kalibr (ETH Zurich ASL; the
continuous-time joint estimation of Furgale, Rehder and Siegwart) is the accepted way to measure them: one trajectory,
every sample at its own time, the time offsets and extrinsics solved together. This spec makes Kalibr the calibration
of record for every sensor pairing on the robot and the arm, checked first in the sim where the truth is known.

## Acceptance criteria

1. [m8Z] **Every sensor pairing calibrated, with one record.** For each pairing below, Kalibr (or, where Kalibr has no
   model, the named method) gives the time offset and the extrinsic transform, with their uncertainties, written to
   one calibration record per robot (`data/robot/calibration.json`, per clock as the frame lag is, [55G]) that every
   consumer reads; `params/robot.yaml` and `data/robot/frame_lag.json`, `head_mount.json` take their values from it
   (source measured, the date and the method):
   - the head camera's intrinsics (its fisheye model) and the head camera to the Go2's IMU / body: time offset and
     mount (today's frame lag and head mount, [gJq]);
   - the wrist camera's intrinsics and the wrist camera to the arm's end link (hand-eye) and to the arm's encoder
     clock: time offset and mount;
   - the arm host's clock to the laptop's (the Pi's samples on the robot's time);
   - the lidar to the body: extrinsic and time offset (Kalibr has no lidar model: a lidar-IMU method, chosen in the
     work and named in the record).
2. [Hw0] **Proved in the sim before trusted on the robot.** The same procedure runs in the sim on an AprilGrid rendered
   in the scene, with the robot's own excitation (below), and recovers the sim's known delays and mounts within stated
   tolerances (time offset within 5 ms, mount within 0.2 deg: estimates, to be set from the sim's own result). It also
   settles the robot-clock lag question (sim-slice [55G]: the robot's own estimate -0.030 s against about +0.02 s by the
   floor map error): whichever of the stamping or the estimator is wrong is found and fixed, not picked between.
3. [3h9] **Excitation the robot can make.** A recorded routine the Go2 performs on its own in front of the board:
   standing body pitch, roll and yaw sweeps, a few steps forward and back, turns in place; the arm sweeping the wrist
   camera across the board. Whether it excites every axis enough is measured by Kalibr's own observability report;
   what it cannot excite (likely translational acceleration) is stated, with its effect on the uncertainty.
4. [VNY] **Offline, never a runtime dependency** (a new dependency is taken on only for
   measured lift, [eM8]). Kalibr runs from its Docker image on recordings exported from DimOS's (a converter from `memory.db` to the
   bag format Kalibr reads); nothing on the robot or in the night's pipeline imports it. The converter and the board's
   definition are in the repository; the image is pinned by digest.
5. [dTP] **Re-calibrated when it can have changed.** After a firmware update ([3il]), a mount or camera bump, an arm
   rebuild, or when a night's own lag or mount check disagrees with the record past its tolerance; each calibration
   dated and kept, the record naming which is in force.
6. [1sW] **The robot's sensor rates are known.** Kalibr wants the IMU at a high rate (typically 200 Hz or more), raw or
   lightly filtered. What DimOS receives from the Go2 (rate, filtering, and whether its samples carry the robot's own
   timestamps) is measured on day one and written down; if it falls short, the record says so and the estimate's
   uncertainty reflects it.

## Decisions

1. [9tQ] **Kalibr over our own estimator, our estimator as the nightly check**. The turn-correlation
   lag estimator and the mount solve from the floor stay, as each night's cheap check against the record ([gJq]'s
   check, warned past 40 ms); the record itself comes from Kalibr. Considered: improving our estimator into a joint
   continuous-time solve (Kalibr's method in our own least squares): kept as the fallback if the Go2's motion or IMU
   rate defeats Kalibr ([1sW], [3h9]), with no new dependency. Since 7 Oct 2026 the mount's check is the floor
   self-calibration (sim-slice [yFg]: `floor_selfcal.py check`, warned past 0.1 deg, 5 mm or 10 ms of the floor rows'
   lag), run where the house's floor map is made; it reports and never writes the record.
2. [Ljd] **The sim first, with the board in the scene** (4 Oct 2026). The sim's delays and mounts are known exactly, so
   the procedure is validated where truth exists, and the sim's own stamping is checked by it ([Feg]: a lag that
   disagrees with the pipeline is a fault to find).

## Open questions

1. **Hardware timestamps where we have them.** The video's RTP timestamps are set by the sender near capture, in
   the robot's clock; if the Go2's state messages carry the same clock, frames and poses could be aligned without the
   network delay at all. Checked at first boot. For hardware we choose (the wrist camera; onboard compute), cameras
   that report device or start-of-frame timestamps and an IMU on the same clock remove most of what this spec measures.
2. **Real statistics into the sim**, some time in the future: the delays, jitter, stalls and rates measured on the real robot fed back into the sim's models
   (today's are estimates: `frame_delay_s`, `odom_delay_s`, `cmd_delay_s`, `stall_burst_share`), so the sim's timing is
   the real robot's. To discuss after day one.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [m8Z] (every pairing, one record) | the record written per pairing with uncertainties; every consumer reads it (static test) | Not yet written |
| [Hw0] (proved in the sim) | sim recording with the board: recovered offsets and mounts against the sim's truth | Planned |
| [3h9] (excitation the robot can make) | the routine recorded; Kalibr's observability per axis | Planned |
| [VNY] (offline only) | no robot-side or pipeline code imports Kalibr; the converter's round trip on a recording | Not yet written |
| [dTP] (re-calibrated when it can have changed) | the record's dates and triggers; a night's check past tolerance flags it | Not yet written |
| [1sW] (rates known) | day one: the IMU's rate and stamping measured and recorded | Planned |
