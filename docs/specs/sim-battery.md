# Battery and coverage

**Type:** Capability spec · **Status:** Draft · **Source:** D0 [jX2]; principle [sJQ] · **ID:** [qNo]

## Purpose

Estimates how much house one charge covers, and what each part of a night costs in energy, for each way of powering
the arm, so hardware choices can be made before buying anything.

## Acceptance criteria

1. [G7m] It reports energy per pick against item mass, from simulated joint torques and the arm motors' published data.
2. [jlD] It reports the area one charge covers for each arm power option (on the Air's pack directly, through a boost
   converter, on its own pack), with the arm held rigid or cradled while walking.
3. [ysG] Every input is labelled published, measured, simulated or estimated, and the report shows low and high results
   where an input is a range.
4. [GWJ] During a slice run it shows each pack's charge and the live power draw.
5. [8nj] Its inputs come from the robot parameters (CALM node `robot-parameters`); when a value there changes to
   measured, the next report uses it.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [G7m] (energy per pick vs mass) | `sim/suite/run power` | Passing |
| [jlD] (area per charge per option) | battery model report | Passing: not yet a suite scenario |
| [ysG] (inputs labelled, ranges shown) | `sim/arm/battery_model.py` | Passing |
| [GWJ] (live charge and power in the slice) | slice HUD | Passing |
| [8nj] (inputs from the robot parameters) | params wiring | Not yet written |
