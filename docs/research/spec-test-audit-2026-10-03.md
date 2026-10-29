# Spec test audit, 3 Oct 2026 (read-only; the audit agent's findings, kept here)

> **Why this is here:** it found Tests rows marked Passing with no test behind them; the rows were downgraded after it, and it is the record of why their statuses read as they do.

Every test file a Tests table names exists; the gap is how much of each criterion they cover.

## Over-claimed (the claim is wider than the tests)
- arm-host: [4WJ] keys only, no values, cadence or delivery (duplicate "Not yet written" row); [871] stowed/docked and
  "never updates itself" untested (duplicate row); [MA0], [u9G], [gFa], [H9j] assert what the class or an injected
  lambda fills in itself.
- sim-house-nights: [yyJ] furniture movement only; [O7l] synthetic empty floor, crowded-house fallback untested; [bP1]
  synthetic rectangle, look and real-floor application untested; [uCP] only LAYOUT_SEED and START.
- sim-slice: [nv5] only the map_error helper; [evl] only trimmed_mean, its "changes nothing without it" a manual check.
- sim-grasp: [N4z] three of four tests skip without the gitignored floor zips; [WuR] "chosen by configuration" untested.
- sim-harness [8sK] renders nothing; manners [N2T] trigger and height unchecked; dog-first-boot [VVJ] argv strings and
  parsed output only; phone-cockpit [Mj2] result from a manual run, no results file; sim-items [nuQ] untested claim;
  sim-to-real [KbT] "second night" is the same S95 recording.

## Stale the other way
- recordings.md rows say "Not yet written" though tests/test_recordings.py exists (partly updated 3 Oct: [R1X]).
- subsystem-demos [qEP] names tests/test_demo_room.py, which does not exist (row is Not yet written).

## Passing with no test file (script, data or manual check)
arm-mount [mNm] [ilX] [MWx] [dKo] [28K] [EKu]; sim-battery [G7m] [jlD] [ysG] [GWJ]; sim-grasp [udO] [bxP] [bfo] [gF0];
sim-house-nights [D4v] [EL6] [5se] [h3t]; sim-slice [5AP] [bXX]; sim-tip-safety [YGD] [uMP]; training-pipeline [cLC]
[soS] [7wf]; sim-reach [OG0]; sim-walk-payload [2V3]; legged-manipulator-sim [lps] [EaS] and phone-cockpit [Eir] [Xzt]
(tests on DimOS fork branches). `sim/suite/run` has no pass/fail logic and hard-codes the venv path.

## Machine-coupled
- test_measure.py:185 absolute path to a recording; [N4z] gitignored zips; [Mj2] [nv5] [evl] [WuR] [FHA] numbers in
  gitignored data/ or the untracked notes.
- Not in git yet: the code and tests behind [VVJ] [Mj2] [vRJ] [F2q] [evl] [4hm] [KbT], and phone-cockpit.md itself.

## Assertions that cannot fail
test_eyes::test_minimum_swing_is_two_seconds (a constant); test_hostd::test_microphone_goes_only_to_the_laptop and
::test_report_with_no_arm; test_health::test_report_has_what_the_spec_asks; test_card::test_profiles;
test_floor_mosaic_trimmed::test_cells_with_few_or_no_views (spans the input range); test_showcase pair test (<= 2 vs
the docstring's one); test_movable_chairs shell-string search; test_robot_logs resolver argv; test_toy_layout's old-
layout test; test_measure's second-night smoke (same night, substring checks).

Next: downgrade the over-claimed rows to Partial with what is and isn't covered, then write the missing tests,
starting where a wrong claim would mislead a decision ([nv5], [KbT], [Mj2], [VVJ]).
