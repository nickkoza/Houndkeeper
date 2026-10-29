# Arm mount placement

**Type:** Capability spec · **Status:** Draft · **Source:** D2 arm on board [Bhf]; principle [5oU] · **ID:** [Gy7]

## Purpose

Decides where on the robot's back the Maker Arm is mounted, from measurements rather than a guess, so the printed mount
is designed around a position that is known to reach the floor well, keep the robot from tipping, walk well, leave the
head camera clear and leave room for the bin.

## Acceptance criteria

1. [mNm] It evaluates a range of mount positions along the robot's back, in the orientation where the folded arm points
   its gripper forward.
2. [ilX] For each position it reports: floor area the arm can pick from while lying; the measured tipping point lying
   and crouched; the worst centre-of-mass margin carrying the arm's rated 1.5 kg; loaded walking (worst tilt, speed
   tracking); the share of the head camera's view blocked by the folded arm; and the length of back left for the bin.
3. [MWx] Every position is tested with the same scenarios and settings, so the positions compare fairly.
4. [aYh] It recommends a position and says what the recommendation trades away against the runner-up.
5. [dKo] It exports the recommended position as a CAD reference (Go2 plus arm, millimetres) for designing the mount.
6. [Izu] It states which robot body the numbers come from; results from the sim's Go1 stand-in are re-run on the Go2
   model before the mount is fabricated.
7. [28K] **Upright use is not locked out.** For each position it also reports what the arm can do with the robot
   standing: the highest table or counter height it can reach in front and to the side; how far the robot tilts when
   the arm moves out with 0.5 kg and with the rated 1.5 kg while standing; and how well it walks carrying 0.5 kg with
   the arm out. The recommendation names the best position for floor picking and for upright use, and says what the
   chosen position gives up in each.
8. [EKu] The tipping point is measured leaning left, right, forward and back, since the mount's position along the
   back matters most for forward and backward tipping.

## Failure behavior

- A position where the folded arm collides with the robot, or cannot reach the floor at all, is reported as unusable,
  never scored.

## Decisions

1. [nOa] **Sweep results (sim).** `sim/suite/run mount`, 2026-09-24, Go1 stand-in ([WD6]), 3.7 kg arm, positions
    -5 to +20 cm from trunk centre. Floor reach lying grows forward (0.15 m2 at -5 cm to 0.28 m2 at +20 cm), as does
    table reach in front (0.6 m to 1.0 m; 0.9 m to the side everywhere) and bin length (3 to 28 cm), with bin headroom
    under the folded arm 4.7-6.9 cm. Lidar view blocked rises from 0 to 1.4%, and tilt walking with 0.5 kg and the arm
    out from 3.3 to 8.4 deg. Nothing fell at any position, standing tilt with 1.5 kg stayed under 7 deg, and the worst
    tipping point is sideways and the same everywhere (crouched 28.8 deg, lying 38.8 deg). Per position:

    | Mount | Floor reach lying | Bin length / headroom | Lidar blocked | Loaded walk tilt | Table reach front / side | Standing swing tilt 0.5 / 1.5 kg | Carry-walk 0.5 kg tilt |
    |---|---|---|---|---|---|---|---|
    | -5 cm | 0.15 m² | 3 / 7 cm | 0% | 4.9° | 0.6 / 0.9 m | 3.6 / 5.4° | 3.3° |
    | 0 | 0.20 | 8 / 7 | 0% | 4.6° | 0.75 / 0.9 | 2.8 / 5.8° | 4.6° |
    | +5 | 0.20 | 13 / 5 | 0% | 7.7° | 0.75 / 0.9 | 2.9 / 6.6° | 5.6° |
    | +10 | 0.25 | 18 / 5 | 0.4% | 5.5° | 0.9 / 0.9 | 3.1 / 4.7° | 6.6° |
    | +15 | 0.23 | 23 / 5 | 0.7% | 6.4° | 0.9 / 0.9 | 3.1 / 5.4° | 7.0° |
    | +20 | 0.28 | 28 / 5 | 1.4% | 7.8° | 1.0 / 0.9 | 3.5 / 6.1° | 8.4° |
2. [Pvh] **The arm is mounted 10 cm forward of trunk centre** (as the position that
    serves both floor picking and table use). Near the best floor reach (0.25 m2), counter height (0.9 m) in front,
    an 18 cm bin and 0.4% of the lidar view blocked. Against +20 cm it gives up 0.03 m2 of floor reach and the 1.0 m
    front reach for less lidar blocking and less walking tilt; against 0 cm it gains 0.05 m2 of floor reach, counter
    reach and 10 cm of bin for about 2 deg more walking tilt. The value lives in the robot parameters, and the sims
    and the CAD export read it from there.
3. [9mk] **The sweep's automatic pick is not trusted.** Equal-weight rank sums named 0 cm best for floor picking and
    -5 cm best for upright use, because small tilt differences outweighed reach. It should drop any position that
    fails a hard limit, then rank by reach.
4. [5Sm] **Unexplained result.** Lying, the backward tipping point is 51-55 deg up to +10 cm, then reaches the 80 deg
    test limit at +15 cm. Possibly the arm propping the robot; not yet investigated.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [mNm] (range of mount positions) | `sim/suite/run mount` | Passing: -5 to +20 cm |
| [ilX] (per-position report) | `run mount` results | Passing: headroom excludes the arm's own base; results under Decisions |
| [MWx] (same scenarios and settings) | one code path for every position | Passing |
| [aYh] (recommendation with trade-off) | `run mount` summary | Partial: the automatic pick uses equal-weight rank sums and names misleading winners (see Decisions); the recommendation was made by hand (see Decisions) |
| [dKo] (CAD export of the recommended position) | `tools/export_models.py`, mount position from the robot parameters | Passing |
| [Izu] (body stated; re-run on Go2) | report header; Go2 re-run | Partial: report states the Go1 stand-in |
| [28K] (upright use measured and weighed) | `run mount` upright part | Passing: measured at every position; heights tested only up to 1.0 m |
| [EKu] (tipping in all four directions) | `run mount` tipping part | Passing: one result unexplained (see Decisions) |
