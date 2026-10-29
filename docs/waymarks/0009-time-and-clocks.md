# [uKM] — Time and clocks: what we borrow from netcode and sensor fusion

**Status:** Proposed · **Date:** 2026-09-27 · **Source:** video games' netcode, where with
everything timestamped and the clocks compensated a lot of error can be corrected, and whose prior research can be
borrowed; and DimOS's Go2 connection as observed 27 Sep 2026 · **Serves:** [Feg] (principle), [KkQ] (placing
frames), [nAH] UMH (late frames), [FHA] (the floor map), the arm's close look · **ID:** [uKM]

## Context

The robot's software sees the world through samples that each arrive late, by different amounts, over Wi-Fi and USB:
head-camera frames (WebRTC), odometry and lidar (Unitree's state stream), the wrist camera, the arm's joint readings.
A frame is only useful with the pose it was taken from, so every error in "when" becomes an error in "where": at a
turn of 0.3 rad/s, 80 ms is 1.4 deg, which moves a floor point 2 m out by ~5 cm. DimOS, as observed at a7492431f,
stamps both frames and odometry with the host's clock on arrival (`time_is_now`) and drops the frame's own
presentation time (`pts`). Multiplayer games solved the same problem, clients and server on different clocks over a
lossy network, and robotics solved it for cameras and IMUs; both are borrowed rather than reinvented.

## Decisions

1. [RmR] **Stamp at the source, keep the stamp end to end.** Each sample keeps the time it describes: for WebRTC
   video the encoder's `pts` (in the stream's 90 kHz media clock), for Unitree's state messages any robot-side time
   they carry (unknown yet), for our own sensors the capture time. Arrival time is kept beside it, never instead.
   (Netcode: every snapshot carries its server tick.)
2. [oI1] **Estimate each clock's offset and keep it up.** Two ways, both standard: (a) round trip, as NTP / Cristian's
   algorithm: time a request out and back and take the midpoint, filtered over many exchanges (the minimum-delay
   samples are the trustworthy ones); (b) from the data, as camera-IMU temporal calibration does (Kalibr: Furgale et
   al. 2013; VINS-Mono's online time offset: Qin & Shen 2018): the lag that best lines up the image's rotation with
   the odometry's (or IMU's) turn rate. Offsets drift slowly: re-estimated through the night, not once.
3. [929] **Look state up at a sample's own time: interpolate, and rewind.** A frame's pose is the pose history
   interpolated at the frame's corrected time (netcode's entity interpolation), and a decision about something seen
   earlier is made against the world as it was then (lag compensation's rewind: Bernier, "Latency Compensating
   Methods", Valve 2001). So a pose history is kept, not just the latest pose.
4. [toa] **A jitter buffer before anything that needs order.** Samples are held a little (a few tens of ms, sized
   from measured jitter) and released in time order, so a late frame is used at its time, not dropped or misplaced
   (as games and VoIP buffer packets; Fiedler, "Snapshot Interpolation").
5. [oea] **Extrapolate only short and know it.** Where a pose is needed ahead of the latest sample (steering the arm
   toward a moving view), dead reckoning from the last known state, bounded in time and flagged; never used to place
   floor pixels.

## Consequences

- Needs a DimOS fork change to keep `pts` on published images (to offer upstream), and a clock-offset estimator
  running beside the tidy. `calibrate_lag.py` is today's offline form of 2(b). Since 27 Sep the frames' lag
  is found from each recording's turns (`floor_mosaic2.lag_from_turns`: 75-85 ms, the sim's truth 80) and taken off
  everywhere frames are placed: the floor pictures (saved in each as `lag_s`), the baseline views, the mount
  calibration, the confirmer's replay, and the live sweep (the earlier nights' median).
- The sim models the imperfection this answers ([nAH] UMH: frames stamped late and jittered); its odometry has no
  arrival delay yet, which is kinder than the real robot and is to be added.
- The same rules apply to the arm's joint readings and the wrist camera, which arrive by a different path (USB, the
  arm's own controller) with their own clocks.
