# Walking with the arm

**Type:** Capability spec · **Status:** Draft · **Source:** D0 [jX2], D2; principle [5oU] · **ID:** [pZM]

## Purpose

Shows whether the Go2 still walks well carrying the Maker Arm and its kit: the same walking script with and without
the load, comparing falls, tilt, height and how well it follows speed commands.

## Acceptance criteria

1. [BlJ] The robot completes a fixed script (stand, walk slow and fast, turn, walk backward, sideways, walk while
   turning) carrying the arm stowed plus its kit (mount, bin, battery), without falling.
2. [2V3] It reports, for each segment, trunk height, worst tilt and the achieved speed against the commanded speed, both
   loaded and unloaded.
3. [XrO] It covers every stow the design allows: the arm powered and held rigid, and unpowered in a cradle.
4. [cRC] It includes standing up from lying down and crouching down from standing, loaded.

## Non-functional requirements

- Loaded worst tilt within 5 degrees of unloaded on the same script (proposed; confirm).

## Failure behavior

- A fall ends the run and is reported with the segment it happened in.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [BlJ] (script completed loaded, no fall) | `sim/suite/run walk` | Partial: arm only; kit mass not yet added |
| [2V3] (per-segment report, loaded vs unloaded) | `run walk` output | Passing |
| [XrO] (both stow modes) | cradle vs powered hold | Not yet written |
| [cRC] (stand up and crouch loaded) | posture transitions | Partial: tested in the lie-down checks, not in `run walk` |
