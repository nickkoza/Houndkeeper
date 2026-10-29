# Odometry: how far the robot's own sense of where it is drifts

Turns `go2_air.odom_drift` into a measurement ([4hm]). The sim draws, per night, a scale error (sd 2%), position
noise per root metre, heading noise per root metre and per root second, and where the night believes it started
([nAH] 2Ak). Whether the Air's estimate uses its lidar is unknown; this measures what DimOS receives as odometry.

Kit: masking tape, a 5 m tape measure, a set square, a clear floor with 3 x 3 m free (the demo room's open floor).

## 1. A straight 5 m: the scale

1. Tape a start line and a finish line 5.00 m apart, square to the direction of walking.
2. Stand the robot with its front feet on the start line; record; walk it straight (DimOS teleop, slow) until its
   front feet are on the finish line; stop. Note how far past or short of the line the front feet stopped (cm).
3. Repeat five times, walking both ways.

Each walk's scale error is the odometry's distance between the two stops (`measure.py odom` with the taped walk as
the truth, below) over the taped distance, less 1. The sim's `scale_sd` is their spread across walks (a per-night
error: check the five agree among themselves within one walk's reading error, ~1 cm in 5 m).

## 2. Five laps of a square: the heading and position drift

1. Tape a 2.5 m square (a cross at each corner); mark the start corner and the robot's heading there with a line
   along its body.
2. Recording, walk five laps (teleop, at the robot's normal speed), stopping on the start cross each lap. At the end,
   measure where the robot actually stands against the start cross (cm forward, cm left) and its heading against the
   taped line (degrees, from the set square).
3. `measure.py odom <memory.db> --truth real/local/taped_square.csv`: the truth file holds the taped corners in
   order with the times the robot stood on each (wall_t, true_x, true_y, true_yaw; one row per stop, the start
   repeated at each lap's end; the measured end pose as the last row). It reports the scale error, the end-point
   error after aligning the start, the heading drift and both per root metre walked, and each drift as a multiple
   of the file's estimate against day one's band (at or under 1; twice the estimate cost the launch numbers 4.6 false
   alarms a night: README.md, day one 3, [GWP]). Walk the square again before the second night.

## 3. Writing it

From the five straight walks and the square: `scale_sd` (the straight walks' spread), `xy_sd` and `yaw_sd_deg` (per
root metre, the square's), `yaw_rate_sd_deg` (per root second: the robot standing still for five minutes, recording,
its odometry's heading drift; 0 if it holds), `start_xy_sd` and `start_yaw_sd_deg` (the spread of where five power-ups
on the same taped start say they are, against each other). Write the dict by hand with source measured and the
dates; then re-run the floor map and pose-graph sims ([7Xn]).

On a sim night with its true path, `measure.py odom` gave back the sim's heading drift (0.54 deg per root metre
against its 0.5) and a scale error of -1.1% (one draw of its 2% spread).
