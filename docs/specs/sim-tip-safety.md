# Tip-safety scenario

**Type:** Capability spec · **Status:** Draft · **Source:** D0 [jX2]; principle [5oU] · **ID:** [BPL]

## Purpose

Finds where the robot actually falls over, and which arm motions could take it there, and turns that into the only
limits every other capability obeys: how far the arm may reach with how much weight, in which posture, and how fast
it may move. Nothing is limited unless the sim shows it is needed.

## Acceptance criteria

1. [oxZ] For each posture the arm is used in (crouched, lying, and standing, so upright use [t8K] has an envelope) and
   each payload (none, 0.5 kg, the arm's rated 1.5 kg),
   it reports how far the robot's centre of mass sits inside its foot support at every reachable arm position.
2. [Yns] It reports, per posture and payload, the fastest arm speed at which the robot never passes its measured
   tipping point ([4KC]) for that speed and every slower one.
3. [YGD] It writes a safety envelope: safe reach and safe speed per posture and payload.
4. [uMP] A step where no motion ran is reported as "no data", never as safe.
5. [jfW] A step where the robot tipped is reported as tipped, and the scenario recovers the robot before the next step.
6. [uMO] Every other scenario and every arm command refuses a target or speed outside the envelope for the payload it
   carries.
7. [4KC] **It measures the tipping point.** For each posture, each lean direction (left, right, forward, back) and each
   arm load (stowed; reaching out to either side with the rated 1.5 kg), it finds the largest lean from which the
   robot recovers level on its own, and reports the worst case per posture.
8. [HKS] **Limits come from the sim, not from round numbers.** Every limit in the envelope is derived from the measured
   tipping point and the centre-of-mass margin, and states the margin it keeps; no limit is imposed where the sim
   shows no risk.
9. [Tru] Any scenario that sees the robot pass its measured tipping point reports it as tipped, with the arm's pose,
   torques and contacts at that moment.

## Failure behavior

- If the envelope file is missing or out of date for the current robot parameters, the arm refuses to move beyond
  the stowed and ready poses.

## Decisions

1. [R5p] **Measured tipping points (sim).** Measured 2026-09-24 by the recovery part of `sim/suite/run tip`, on the
    Go1 stand-in ([WD6]) with the arm mounted 12 cm forward of trunk centre, arm stowed: crouched 28.8 deg, lying
    38.8 deg, both leaning sideways, the worst direction. With the arm reaching out to the side, the arm props the
    robot on the floor and it recovers from further; that support is not relied on. The mount has since moved to
    10 cm forward ([Gy7] Decisions); these are re-measured there before they are used for limits.
2. [BuG] **Standing, measured for the standing look** (27 Sep 2026, `sim/arm/standing_look.py`, arm at 10 cm forward; the
   legs holding the stand by position, the real robot balances actively): stowed, 22.5 deg sideways, 70.6 forward,
   38.1 back. With the wrist camera aimed down at the floor 0.5-1.0 m ahead and up to 35 cm aside from 0.58-0.65 m up
   (each joint within 0.23 rad of the stow), the same to 0.1 deg in every direction, and the trunk tilts at most
   0.31 deg while the arm moves there at 1 rad/s. No payload in the jaws. The lifted sweep pose (wrist camera 73 deg
   down from 68 cm) the same: 21.9 sideways, 70.6 forward, 38.1 back. All measured with the rigid, perfectly
   gravity-compensated arm and a 5 ms physics step; they stand to be re-measured with the arm's sag, flex and play
   ([nAH] z7P) at the 1.25 ms step every sim now uses (a flexing arm at 5 ms had shaken the walking robot over).
3. [2bW] **Safe arm speed with a light item held** (18 Oct 2026, the dynamic part of `sim/arm/tip_bench.py` run
   headless on the CPU with `--dynamic-only --dyn-payloads 0.0,0.25,0.5` into a copy, its 0.25 and 0.5 kg rows then
   added to `sim/arm/tip_envelope.json` beside the September rows; worst trunk tilt over the three side and forward
   swings from the raised pose, against the 8 deg limit):
   - Lying, 0.25 kg: 5.7-6.4 deg at 0.4-3.0 rad/s; 0.5 kg: 5.8-6.6 deg. Safe to 3.0 rad/s at both, so a held toy is
     limited by the arm's global 1 rad/s as a measured limit, not as a fallback.
   - Crouched, 0.25 kg: 6.7-6.8 deg at 0.4-1.0 rad/s, no reachable target at 1.4; 0.5 kg: 7.4-7.8 deg at 0.4-1.0,
     8.4 at 1.4 (7.5-7.7 at 2.0-3.0). Safe to 1.0 rad/s at both, 0.2 deg inside the limit at 0.5 kg.
   - The empty lying arm re-run in the same session came within 1.3 deg of the September rows (5.0-6.4 against
     5.0-7.7), so the sim has not moved under them. The empty crouched arm again reached no target, as in
     September, and stays "no data".
   - Still the rigid arm at the old mount and step, as [BuG]; 1.5 kg lying remains unsafe even at 0.4 rad/s.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [oxZ] (CoM margin across reach, posture, payload) | `sim/suite/run tip` static part; `sim/arm/standing_look.py` | Partial: crouched and lying pass; standing measured for the standing look's poses only, no payload |
| [Yns] (fastest safe speed, monotone) | `run tip` dynamic part; `tests/test_arm_guard.py` (a 0.3 kg item gets the 0.5 kg row) | Partial: measured at 0, 0.25, 0.5 and 1.5 kg ([2bW]); 1.5 kg swings only tested beyond the safe reach; the empty crouched arm reaches no target, so it has no data |
| [YGD] (envelope written) | `sim/arm/tip_envelope.json` | Passing |
| [uMP] (no data is not safe) | tip bench | Passing |
| [jfW] (tipped reported and recovered) | tip bench | Written |
| [uMO] (envelope enforced everywhere) | arm command check; `tests/test_arm_guard.py` | Partial: every arm command through DimOS (the tidy) passes the arm guard, an item up to 0.5 kg judged by a measured speed row ([2bW]); the standalone sims (`sim/arm/vslice.py` and the benches) still apply only the global speed limit |
| [4KC] (tipping point per posture, direction, load) | `sim/suite/run tip` recovery part; `sim/arm/standing_look.py` | Partial: standing measured stowed and for the standing look's poses, no payload; results under Decisions |
| [HKS] (limits derived from the sim) | envelope states its source and margin | Not yet written |
| [Tru] (passing the tipping point reported as tipped) | tip alarm uses the measured tipping point | Not yet written: the tip alarm uses a fixed 25 deg |
