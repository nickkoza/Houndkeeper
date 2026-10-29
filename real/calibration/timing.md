# Timing: how late the frames and the odometry are, and what never arrives

Turns `go2_air.frame_delay_s`, `odom_delay_s`, `cmd_delay_s`, `video_stream` and `stall_burst_share` into measurements ([4hm]). DimOS stamps each camera
frame and odometry message when it arrives over the robot's link, not when it was taken; the floor pictures place
each frame by the pose at its stamp, so a lag smears the floor ([nAH] UMH). The night pipeline already finds the
frames' lag behind odometry from the robot's turning (`floor_mosaic2.py --dt auto`); these steps measure each part.

Kit: the laptop on the robot network, a second screen or a phone showing a millisecond clock, a clear floor to turn on.

## 1. Glass to stamp: the camera's own delay (`frame_delay_s`)

1. On the laptop, open a full-screen clock that shows milliseconds and is drawn from the laptop's own clock (the
   one DimOS stamps with): `python3 -c` loop printing `time.time()` to 3 decimals in a big font, or any web stopwatch
   synced to the system clock.
2. Stand the robot 0.5-1 m from the screen, the head camera looking at it, and record a minute.
3. For 20 frames spread over the minute, read the clock in the picture and subtract it from the frame's stamp:
   each is that frame's delay (the screen's own lag, ~10-20 ms at 60 Hz, is in it; note the refresh rate).

Their median is `frame_delay_s.mean`, their spread (sd) its `jitter`. Expect 100-300 ms over WebRTC.

## 2. The frames against odometry, from turning

Turning on the spot shows a frame's lag against odometry directly: the picture turns later than the pose says.

1. At the end of the taped walks (odometry.md), turn the robot on the spot through a full turn each way, at
   walking turn speed, recording.
2. `measure.py lag <memory.db> --expect <the stored lag, s>`: the frames' lag behind odometry and how well the two
   turnings correlate (keep it if the correlation is over 0.7); with `--expect`, whether it is within day one's
   band of the lag the floor pictures use (20 ms; 50 ms off cost the launch numbers 6 false alarms a night: README.md,
   day one 2, [GWP]). It searches out to a second; a turn it cannot measure says why and gives no number (turn
   again), and a lag past 0.3 s is said: the nightly check reaches only that far, so widen it before trusting it.
   Every sim night before 17 Oct 2026 ran with no frame delay at all (a fallback in the sim chose the zero delay),
   so the sim's stored lag (-0.03 s) is the odometry's alone: expect the real lag to differ from it by far more.

This lag is the frame delay less the odometry's. With step 1's frame delay it gives step 3. It is also the robot
clock's frame lag (the real robot's own clock is the wall clock): it replaces the sim's `robot` entry in the robot's
lag file (`calibrate.store_lag`, sim-slice decision [55G]), which the floor pictures of real nights read.

## 3. The odometry's own delay (`odom_delay_s`)

`odom_delay_s.mean` = step 1's frame delay - step 2's lag. Its jitter: the spread of the odometry messages' arrival
intervals against their median (`measure.py stream` prints the camera's; the odometry's is the same analysis on
the `odom` stream). A negative or larger-than-50 ms result means one of the two is off: repeat both.

## 4. What never arrives (`video_stream`)

`measure.py stream <memory.db> --write` on any real recording of 10 minutes or more (the battery walk is a good
one): the frames' nominal period, the share lost singly, and the stalls (0.3 s or more with nothing) a minute and
their mean length, and the bunches: frames stamped within a few ms of each other just after a gap, which a jitter
buffer let through late (their share of a stall's frames is `stall_burst_share`). A sim night recorded since the
robot's clock ([3Pv], 3 Oct) can be compared, its frames stamped on sim time; an older one can't (stamped on the wall
clock while it ran below real time).

## 5. Command to motion (`cmd_delay_s`)

1. Robot standing still on a clear floor, recording. From the laptop, send one forward velocity step (0.3 m/s for
   2 s) through DimOS, and log the send time on the laptop's clock.
2. In the recording, find the first odometry message whose speed passes 0.05 m/s; subtract `odom_delay_s.mean` from
   its stamp (odometry is stamped on arrival) and the send time from that.
3. Repeat 10 times, spread over a few minutes. Their median, less the walking controller's own rise to 0.05 m/s
   (about a step, ~0.1 s, est.; read it from the same odometry), is `cmd_delay_s.mean`; their spread its `jitter`.

Expect 20-100 ms over the robot's link. The sim holds each command that long on the robot's clock (MockController).

The Wi-Fi matters: record where the robot works, at night, with the house's usual traffic.
