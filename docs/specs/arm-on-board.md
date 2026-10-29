# Arm on board

**Type:** Capability spec · **Status:** Draft · **Source:** deliverable D2; principle [5oU]; [E2T] adapted from MOSS · **ID:** [Bhf]

## Purpose

The Maker Arm rides on the Go2 Air, powered from on board, stowed safely while walking, and used only when the robot
is stopped and down in its picking posture, without making the robot fall or map worse. Those operating rules are for
floor picking; the hardware must not rule out using the arm standing later ([t8K]).

## Acceptance criteria

1. [G48] The arm is carried stowed; while the base moves, the arm cannot deploy.
2. [FWp] The arm moves only when the base is stopped and in its crouched or lying posture.
3. [GYa] Walking, mapping and relocalisation with the arm on are no worse than without it, on the same nightly tests.
4. [FQ3] The robot stands up from lying and lies down from standing with the arm and kit on, without tipping.
5. [Rrm] Arm, mount, bin and arm power together weigh no more than the kit budget (1.8 kg beyond the 3.7 kg arm).
6. [pkg] Nothing trails from the robot: no tether, no cable in the camera's view.
7. [qpO] The arm obeys the tip-safety envelope for the weight it carries.
8. [odU] Criteria [G48] and [FWp] are enforced in software, not by the hardware: the mount, stow, bin and arm power
   leave the arm free to reach a table in front of the standing robot, so a later spec can relax those rules once
   standing use is measured ([28K], [BPL]).
9. [E2T] **Every hardware build is logged honestly.** Each finding while building the mount, bin and arm power gets a
   row: what was seen, which part and revision, what was done on this build, what changes next revision, and what
   validation actually happened. Ordered or designed is never written as validated. (The table layout is MOSS's
   assembly feedback log, docs/assembly_feedback.md in github.com/metrox-eth/moss at b2c887a, CC BY 4.0.)
10. [WkC] **The robot never walks or changes posture with the arm out of the stow.** A move the arm's safety checks
    refuse partway leaves the arm held where it stopped; once the base reads still the stow is tried again, a bounded
    number of times, and if the arm still cannot be stowed the night stops where the robot is and reports the arm out
    for review. A stow nothing refused that comes to rest short of the stow is blocked: the arm is held where it rests,
    not pressed on what is in its way and not tried again the same way, and the night stops saying which joints are out.

## Failure behavior

- If the arm loses power or its link, it is treated as stowed and unusable; the night continues without picking.
- If the base starts moving with the arm out, the base stops.
- If the arm cannot be returned to the stow, the night stops in place with the arm reported out ([WkC]).

## Decisions

1. [EkS] **The arm host runs from the Air's pack through a 5 V buck converter**. The Raspberry
   Pi needs 5 V; the pack runs 28-33.6 V (8S Li-ion, est.), so the converter takes an input rated to at least 40 V and
   gives 5.1 V at 2.5 A or more (Raspberry Pi's recommended supply for the 3 Model B), behind a PTC polyfuse on its
   input. Its draw is in the battery model: about 5 W at the Pi (Raspberry Pi's ~400 mA typical for a bare 3B, plus the
   USB camera, the CAN adapter and Wi-Fi streaming, est.) through a 90 % converter (est.), all night. The arm itself
   stays on the pack directly: a 48 V boost gives no extra torque at the arm's speeds (RobStride
   RS02 data: 17 N·m peak, 24-60 V operating range).

2. [G0m] **The folded arm lies over the bin's front half, and what stands there blocks the stow.** Hard-won (18 Oct
   2026, sim arm night data/runs/dimos_tidy/20261018_202533, house 70, item seed 2). At the stow the upper arm and elbow
   run over the bin from 2 to 15 cm forward of its middle, 8 cm wide, their underside 7.6 cm above the bin's floor (its
   rim: 6 cm); the only gripper-down release points the arm reaches lying are in that front half (the corner release:
   5 cm forward). A 25 x 9 x 9 cm boat shoe let go there stood 15 cm tall on the ballet flat binned before it; three
   stows each ended at the same pose, the shoulder 0.13 rad short, nothing refused. Checked on the night's end state:
   at the stow the arm's links 3 and 4 sit up to 25 mm inside the shoe, at the pose it held, clear. Retrying the same
   move, opening the jaws (closed, 15 cm away) or backing out along the approach cannot help: the stow pose itself is
   occupied. Of 12 items binned on 9 sim nights (the hold-out houses left out), this shoe and the flat under it were
   the only ones reaching into the fold. A typical shoe upright (9 cm) under the fold's strip blocks it alone.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [G48] (cannot deploy while moving) | `tests/test_arm_guard.py` (an arm command refused while odometry says the base moves, or is stale; posture changes only stowed; the arm-up sweep refused; the base stopped if it moves with the arm out); real interlock test | Partial: every arm command in the tidy goes through one guard (`src/houndkeeper/arm_guard.py`); unit tests and a CPU physics check (lying, three reaches and back, 18 Oct 2026: no false refusal) pass; the real interlock waits on the arm |
| [FWp] (moves only stopped and down) | `tests/test_arm_guard.py` (lying or crouched only; unknown or changing posture refused); real check | Partial: guard enforced on every arm command. The standing look (sim-slice [QDc]) moves the arm standing, which this criterion as written forbids: the guard allows it only stopped, empty-jawed and within the poses whose balance was measured (sim-tip-safety [BuG], every joint within 0.23 rad of the stow); still to settle. Posture is the one the robot last took, not yet read back from the robot |
| [WkC] (never walks with the arm out) | `tests/test_arm_stow_stop.py` (a stow refused partway held, retried once the base settles and finished; still refused, the night stopped with the arm's pose filed; the standing look and the stop's end stow both through it; a stow that comes to rest short, nothing refused, held where it rests and stopped after one try, naming the joints out; an arm still on its way not called blocked) | Passing (unit tests); seen in a sim night, blocked by a binned shoe ([G0m]); the blocked stop not yet seen in one |
| [GYa] (nightly tests no worse) | D1 tests, arm on vs off | Not yet written |
| [FQ3] (stand up and lie down loaded) | `tests/test_loaded_postures.py` (sim: arm stowed, a 1.8 kg dummy kit at the mount or at the bin end); real test with dummy weight, then arm | Partial: sim passes (Go1 stand-in on leg PD, worst tilt under half the lowest measured tipping point); real not yet |
| [Rrm] (kit within budget) | weigh the kit | Not yet written |
| [pkg] (no tether) | inspection | Not yet written |
| [qpO] (envelope obeyed) | `tests/test_arm_guard.py` (reach past the envelope for the weight held refused; a step faster than the safe speed refused; no envelope, the stow only) | Partial: reach and speed from `sim/arm/tip_envelope.json` checked at every arm command, the payload the robot weighed judged as the next heavier one measured. An item up to 0.5 kg, lying or crouched, now has a measured safe speed (sim-tip-safety [2bW]: 3 rad/s lying, 1 rad/s crouched), so it moves at the envelope's limit (capped at the arm's global 1 rad/s); between 0.5 and 1.5 kg lying, the envelope still has no safe speed and the guard falls to the global 1 rad/s and says so; whether the envelope matches the current robot parameters is not checked |
| [odU] (standing use not blocked by hardware) | `sim/suite/run mount` table reach at the chosen mount; fit check of the printed mount | Not yet written |
| [E2T] (hardware builds logged) | the build log exists and every row states its validation | Not yet written: first build with the printed mount |
