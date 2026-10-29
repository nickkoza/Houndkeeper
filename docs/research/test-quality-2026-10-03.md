# Test quality review, 3 Oct 2026

> **Why this is here:** the mutation review behind the tests/*_mutants.py files: of 86 planted one-line bugs the suite first caught 22; each mutants file pins bugs the first tests missed.

I tested the suite against small deliberate bugs ("mutants") to see whether it catches them. I also checked whether the
criteria the specs mark Passing are really tested, and looked for weak assertions and tests that are coupled to this
machine or depend on timing.

## Baseline

Command: `../dimos/.venv/bin/python -m pytest -p no:cacheprovider tests`.

- Result before this review: 391 passed, 2 skipped, 1 xfailed, in 6 min 29 s.
- Slowest tests:
  - `test_measure.py::test_gap_on_a_perturbed_sim_night...`: 72 s.
  - `test_floor_photos.py::test_generated_floors_come_from_the_split`: 44 s.
  - `test_one_object_per_scan.py`: 29 s.
  - `test_measure.py::test_gap_command_smoke...`: 27 s.
- The filament tests start Vulkan on the RTX 2070 (see "Flaky and coupled tests").

## How the mutants were run

- Each mutant is a copy of one module with one plausible one-line bug.
- The copy is loaded in place of the real module before any test imports it (a pytest plugin preloads `sys.modules`, or
  puts a mirrored package with symlinks first on the path).
- The module's own tests are then run against it.
- The real source was never edited. The harness is in the job's scratch dir (`run_mutants.py`, `mutants.py`,
  `mutplugin.py`).
- Controls:
  - An unchanged copy of each module passes.
  - A deliberately broken copy fails.
- Size of the run: 86 mutants over 9 modules.
  - Before the new tests: 22 caught, 64 survived.
  - With the new tests: 81 caught, 5 survive. The 5 are equivalent or defence-in-depth (listed at the end).
- `recordings.py` and `cockpit_data.py` were being edited by other sessions during this review. Their mutants were made
  from the version on disk at the time, so the numbers for those two modules describe a moving target.

### Mutants that survived the existing suite, each a missing test

Every row below survived the suite as it was. Every row is now caught by a test added in this review (file in brackets),
except the five listed after the tables.

**floor_fidelity.py** ([nv5], [FHA]). Before this review, only `map_error` and `truth` had any test.
`score`, `lay_on`, `frame_from_path`, `to_true_frame`, `truth_structures` and `error_of` had none.

| id | bug | killed by |
|---|---|---|
| ff1 | the map's ragged 5 cm edge judged | `test_the_maps_ragged_edge_is_not_judged` |
| ff2 | rug, chairs and wall band judged anyway | `test_what_is_excluded_is_not_judged` |
| ff3 | the truth judged over all its floor instead of the map's | `test_the_truth_is_judged_over_the_maps_floor_even_beside_a_bright_rug` |
| ff4 | error threshold doubled | `test_the_error_threshold_is_the_confirmers` |
| ff5 | slack 6 cm instead of 2 cm | `test_a_map_off_by_more_than_the_slack_is_error_and_within_it_is_not` |
| ff6 | `to_true_frame` turns the wrong way | `test_the_maps_frame_comes_from_its_path_and_lands_points_where_they_truly_are` |
| ff7 | path fit returns a mirror on a straight walk | `test_a_straight_walk_still_gives_a_turn_not_a_mirror` |
| ff8 | per-patch warp measured from the window corner | `test_the_score_finds_the_frame_and_each_patchs_own_warp` |
| ff9 | wall mask a cell off | `test_the_structures_mask_covers_each_wall_cell_where_the_sim_built_it` |
| ff10 | wall band inverted in `error_of` | `test_a_nights_error_leaves_out_the_band_by_the_walls` |
| ff11 | `lay_on` shift sign flipped | `test_the_score_finds_the_frame...` |

All of these tests are in `tests/test_floor_mutants.py`.

About ff3: the existing test `test_the_edge_of_what_the_map_saw_is_not_error` claims to catch exactly this bug, but it
passes it by a hair. On its synthetic floor the largest colour difference is 0.0439 against a threshold of 0.045.

**floor_agreement.py** ([FHA], [V35]). Before this review, `regions`, `footprint` and the split with truth had no test.
The existing tests used pictures with no structures, both views covering everything, and no truth files.

| id | bug | killed by |
|---|---|---|
| fa1 | one night's truth file counts as having the truth | `test_a_night_with_one_truth_file_has_no_truth` |
| fa2 | structures taken from either picture, not both | `test_agreement_judges_only_floor_both_saw...` |
| fa3 | floor seen by either picture is judged | same |
| fa4 | the 20 cm band round structures not left out | same |
| fa9 | change outside the judged floor counted | same |
| fa5 | moved chairs left in the open floor | `test_with_the_truth_the_rug_and_moved_chairs_are_split_from_the_open_floor` |
| fa6 | chairs that stayed put counted as moved | same |
| fa7 | rug and chairs not grown | same |
| fa8 | footprints flipped in y | same |

**detect_changes2.py `feats` / `tolerant_diff`** (the detector's own test, used by [nv5] and [FHA]). No test imported it
directly, and only one of seven mutants was caught (dc7).

| id | bug | killed by |
|---|---|---|
| dc1 | slack one-sided in y | `test_tolerant_diff_forgives_every_misalignment_up_to_the_slack_and_no_more` |
| dc2 | no slack in x | same |
| dc3 | colour compared without the shift | same |
| dc4 | darkening reads as brightening (shadows flagged) | `test_tolerant_diff_brightening_is_signed_so_a_shadow_is_not_a_brightening` |
| dc5 | 6-cell surroundings instead of 25 | `test_feats_judge_a_toy_against_the_floor_round_it_not_against_itself` |
| dc6 | unseen floor included in the surroundings' brightness | `test_feats_are_blind_to_the_floor_outside_what_was_seen` |

**calibrate.py** ([oI1], [KkQ]). `mount_correction` and `agreement` had no test at all.

| id | bug | killed by |
|---|---|---|
| cl2 | odometry turn rate per sample instead of per second | `test_the_lag_is_found_when_frames_and_odometry_arrive_unevenly` |
| cl6 | image rate per frame instead of per second | same |
| cl3 | weak phase correlations kept | `test_frames_that_do_not_match_their_neighbour_are_not_used` |
| cl9 | heading lanes split on the wrong axis | `test_the_mounts_pitch_and_yaw_are_found_from_walking_lanes_both_ways` (a synthetic floor rendered through a mis-mounted camera) |

cl2 and cl6 survived because the old test's clocks were perfectly even, which hid both bugs.

These tests are in `tests/test_calibrate_mutants.py`.

**measure.py** ([4hm]).

| id | bug | killed by |
|---|---|---|
| me1 | start alignment turns the wrong way | `test_odometry_in_its_own_frame_is_laid_on_the_truth_at_the_start` |
| me2 | end drift is really the worst drift | `test_the_end_drift_is_at_the_end_not_the_worst_on_the_way` |
| me3 | drift per metre instead of per root metre | `test_position_drift_is_per_root_metre` |
| me8 | standing-still jitter counted as walking | `test_standing_still_jitter_is_not_distance_walked` |
| me10 | truth heading not unwrapped | `test_a_truth_heading_that_wraps_past_180_is_not_a_turn` |
| me7 | drop share over received frames instead of expected | `test_frame_gaps_share_is_of_the_frames_that_should_have_come` |
| me5 | writing `go2_air.lidar.range_sd` replaces the whole lidar dict | `test_writing_one_field_of_a_dict_keeps_the_others` |
| me4 | a key only another group has gets written there | `test_a_key_only_another_group_has_is_not_written_there` |

These tests are in `tests/test_measure_mutants.py`.

- The old odometry test started both paths at heading 0 and the origin, which hid me1. It also never asserted `end_m`
  or `xy_per_root_m`.
- me5 matters: `measure.py --write` does make exactly this write.

**recordings.py** ([HqQ]).

| id | bug | killed by |
|---|---|---|
| re13 | a recording flagged as showing people can be used | `test_a_recording_flagged_for_people_is_refused_like_an_unscreened_one` ([R1X]) |
| re10 | a battery-carrying recording can be added as sim | `test_a_robot_recording_cannot_be_added_as_sim` ([YP8]) |
| re11 | a run folder holding other files gets moved | `test_a_run_folder_holding_anything_else_is_not_moved` ([u6u]) |
| re1 | prune deletes a cataloged sim recording outside `dimos/recordings` | `test_prune_never_deletes...` ([B2O]) |
| re2 | drop-local removes a copy outside the store | `test_drop_local_refuses_a_laptop_copy_outside_the_store` ([Zw9]) |
| re3 | drop-local accepts the laptop's own file as the server copy | `test_drop_local_refuses_a_server_copy_that_is_the_laptops_own_file` ([Zw9]) |
| re8 | a mismatched server copy marked verified | `test_a_copy_that_did_not_match_is_not_marked_verified` ([Zw9]) |
| re5 | one private address is enough to count as in the house | `test_a_host_with_any_public_address_is_outside_the_house` ([fya]) |

These tests are in `tests/test_recordings_mutants.py`.

- re13 is the one that matters most. The old test checked only unscreened and clear recordings, never one flagged as
  showing people.
- re11: the old "neighbours" test used a folder whose name is not a run id, so a different check refused it first.

**cockpit_data.py** ([vRJ]).

| id | bug | killed by |
|---|---|---|
| cd6 | taped-square end not turned into the robot's start frame | `test_the_taped_square_measures_the_end_in_the_robots_own_start_frame` |
| cd10 | heading error not wrapped | `test_a_heading_that_ends_past_180_degrees_is_a_small_error` |
| cd11 | every pass near the cross counts as a lap | `test_wandering_near_the_start_is_not_a_lap` |
| cd3 | a second start throws away a recording in progress | `test_a_second_start_while_recording_keeps_the_run` |
| cd9 | `stand_power_W` written without its anchor line, so the write fails | `test_confirming_a_power_run_writes_the_stand_power_beside_the_walk_power` |
| cd7 | True and False accepted as numbers to write | `test_true_and_false_are_not_numbers_to_write` |

These tests are in `tests/test_cockpit_data_mutants.py`. The old tests always started the walk at heading 0, which hid
cd6, and never confirmed a power write.

**arm_host/release.py** ([871]).

| id | bug | killed by |
|---|---|---|
| rl1 | the first release failing leaves `current` pointing at it | `test_the_first_release_failing_leaves_no_current` |
| rl2 | a check that times out counts as passing | `test_a_check_that_hangs_is_a_failed_check` |
| rl6 | a release that is not there is not refused | `test_a_release_that_is_not_there_changes_nothing` |
| rl7 | an absolute link breaks when the tree is moved (card image) | `test_the_current_link_holds_when_the_tree_is_moved` |

These tests are in `tests/arm_host/test_release_mutants.py`.

**arm_host/eyes.py** ([GTG], [0Py]).

| id | bug | killed by |
|---|---|---|
| ey1 | `Eyes.step` does not cap a long frame time (a 5 s stall jumps to full in one frame) | `test_a_stalled_loop_catches_up_over_several_frames_not_one` |
| ey5 | a backwards clock step in the driver drives full brightness in one frame | `test_a_clock_stepping_back_changes_nothing_on_the_wire` |
| ey7 | an infinite colour treated as full | `test_an_infinite_colour_is_dark` |

These tests are in `tests/arm_host/test_eyes_mutants.py`.

- ey1: `WS2812.show` caps the step again, so this was defence-in-depth, but it was untested.
- ey5: it is a flash, but it cannot happen with `time.monotonic`.

### Still surviving: equivalent or defence-in-depth

- **cl4**: frame pairs across a gap are kept. Harmless while the phase correlation does not wrap.
- **cl7**: the parabola refinement's sign is flipped. Its effect is bounded by the 0.05 deg fine step, below any useful
  tolerance.
- **re6**: an option-like destination gets past the first check. The `REMOTE_DEST` pattern refuses it anyway, and `sync`
  checks for a leading `-` itself.
- **re12**: drop-local accepts an unverified server copy. It hashes the server copy again before removing anything.
- **cd5**: a write while recording. Nothing is pending while recording, so there is nothing to write.

## What the new tests found besides gaps (production code not changed; each is a design decision)

1. **`frame_lag`'s phase-correlation floor (0.05) lets about 14% of garbage frame pairs through.**
   - On noise, the response's 90th percentile is 0.056.
   - With one frame in seven garbled, the lag was still found (75 ms against a true 80 ms), but the correlation fell from
     0.99 to 0.55.
   - A floor of 0.1 to 0.15 would reject those pairs.
   - `test_frames_that_do_not_match_their_neighbour_are_not_used` accepts a correlation above 0.4 for now.
2. **`mount_correction` cannot fully tell yaw from roll.** On the synthetic floor:
   - Pitch and yaw are recovered to within 0.03 deg when the true roll is 0.
   - With a true roll of -0.3 deg, it returned yaw 0.54 deg (true 0.4) and roll -0.18 deg.
   - Both axes are judged from views split by heading, so roll is only weakly observable from walking lanes. A turn on
     the spot, or a view of a wall, would separate them.
3. **The detector's relative colour carries a change about 50 cm into the floor around it.** Its surroundings use a
   gaussian of 25 cells.
   - A wrong patch excluded with a 5 cm margin still scored 5% error on the open floor around it.
   - `floor_agreement.regions` grows the rug by only 5 cm, so a rug that moved or changed between nights leaks into the
     "open" share.

## Criteria claimed beyond what their tests show

From a read of every Tests table (I checked the ones marked * myself).

- sim-slice **[nv5]** "Passing (the measure)": until today only `map_error` on synthetic arrays was tested. The path fit,
  the exclusions, the per-night `error_of` and the dated log were not. The first three are now covered by
  `tests/test_floor_mutants.py`. The log append is still untested.*
- sim-slice **[evl]** Passing: only the `trimmed_mean` helper is tested. "Choosing it changes no picture built without
  it" rests on a manual comparison.
- sim-to-real **[KbT]**: the spec says the report "runs on two different sim nights", but
  `test_gap_command_smoke_on_a_second_sim_night` uses the same house 95 recording as the other test.*
- recordings **[R1X]**: the spec covers unscreened and flagged recordings. Only unscreened ones were tested until
  today.* Every other recordings row still reads "Not yet written" although `tests/test_recordings.py` exists.
- arm-host:
  - **[4WJ]**: the test checks only that keys exist, nothing about the values or the once-a-second cadence.
  - **[871]**: the rollback is tested, but "only while stowed or docked" and "never updates itself" are not.
  - **[gFa]**: the microphone test asserts on `MicStreamer.destinations`, a set the class fills in itself.*
  - **[u9G]**: the test shows only that a FIFO is first-in first-out.
  - **[MA0]**: the test echoes an injected lambda.
  - **[H9j]**: the test checks only that profile names exist.
  - [4WJ] and [871] each have a second row saying "Not yet written" that contradicts the first.
- sim-house-nights:
  - **[yyJ]**: only the chairs moving is tested; items, stains, rug, lighting and "no person" are not.
  - **[O7l]**: the layout is tested on an empty synthetic floor, never a real house or the crowded-house fallback.
  - **[bP1]**: tested on a synthetic rectangle; the look of the wear is never checked.
  - **[uCP]**: only `LAYOUT_SEED` and `START` are checked.
- sim-grasp:
  - **[N4z]**: three of its four tests skip without the gitignored floor zips, and none checks the sub-millimetre
    patch, the jitter or the rug edges.
  - **[WuR]**: "chosen by configuration" is untested.
- sim-harness **[8sK]**: nothing is rendered.
- manners **[N2T]**: `_nope` is called directly; the trigger and the height are not checked.
- dog-first-boot **[VVJ]**: only argv strings and parsed fixtures are checked; nothing resolves or blocks anything.
- phone-cockpit **[Mj2]**: the evidence is a manual sim run with no committed results.
- Passing rows that name no test file at all: arm-mount [mNm] [ilX] [MWx] [dKo] [28K] [EKu]; sim-battery [G7m] [jlD]
  [ysG] [GWJ]; sim-grasp [udO] [bxP] [bfo] [gF0]; sim-house-nights [D4v] [EL6] [5se] [h3t]; sim-slice [5AP] [bXX];
  sim-tip-safety [YGD] [uMP]; training-pipeline [cLC] [soS] [7wf]; sim-reach [OG0]; sim-walk-payload [2V3]. The
  "sim/suite/run" behind several of them has no pass/fail logic: it appends to `history.jsonl`.

## Weak assertions

- `tests/arm_host/test_eyes.py::test_minimum_swing_is_two_seconds`: asserts a constant.
- `tests/arm_host/test_hostd.py::test_microphone_goes_only_to_the_laptop`: `m.destinations == {...}` is the class's own
  bookkeeping.
- `tests/arm_host/test_hostd.py::test_report_with_no_arm`: echoes the injected `arm_present`.
- `tests/arm_host/test_health.py::test_report_has_what_the_spec_asks`: `assert k in r`, keys only.
- `tests/test_floor_mosaic_trimmed.py:61`: `30 <= out <= 250` spans the whole input range.
- `tests/test_floor_map_error.py::test_the_edge_of_what_the_map_saw_is_not_error`: passes the bug it names (ff3 above).
- `tests/test_floor_map_error.py::test_a_map_that_is_the_floor...`: `shifted > 0.10` with a 10 px shift on 20 px tiles
  cannot see the threshold or the slack (ff4 and ff5 survived it).
- `tests/test_pet_bowls.py:51`: `m.body("pet_bowl_0").id >= 0`, so the bowl exists but its place is never checked.
- Tests that check a script's text instead of behaviour:
  - `test_movable_chairs.py:24`, `test_pet_bowls.py:55` and `test_floor_relief.py:34,54` search `night_tidy.sh` for a
    string.
  - `test_head_camera.py:122` greps `tidy_module.py`.
- `tests/test_measure.py::test_gap_command_smoke_on_a_second_sim_night`: substrings of the markdown on the same night as
  the other test.
- `tests/test_toy_layout.py::test_the_old_layout_heaped_every_toy_by_the_start`: describes the replaced layout.
- `tests/test_showcase.py`: allows two differences where the spec says a pair differs in one thing.

## Flaky and coupled tests

**Machine-coupled**

- `tests/test_recordings.py::test_a_session_refuses_a_failed_or_stale_isolation_check_and_low_disk`:
  `preflight(real=True) == []` reads the real free space of the filesystem holding pytest's tmp dir. On a disk with less
  than 50 GB free it fails. `shutil.disk_usage` should be monkeypatched.
- `tests/test_measure.py` (two tests): absolute `<home>/houndkeeper/dimos/recordings/20261002-...` path. They skip
  elsewhere, so [KbT]'s real-data evidence exists on this machine only.
- DimOS checkout through `Path.home()/houndkeeper/dimos` or `HK.parent/dimos`:
  - `test_frame_stamp`, `test_wrist_idle`, `test_odom_delay`, `test_head_camera` and `test_see_through_walls` insert it
    on `sys.path`, so they fail rather than skip without it.
  - `test_filament_flicker` and `test_headless_viewer` use it too.
  - `test_mujofil_default` reads `HK.parent/build/mujofil-f177-site`.
- `tests/test_cockpit_data.py` and the cockpit part of `tests/test_redteam_data.py` use
  `importorskip("houndkeeper.cockpit_data")`. An ImportError in that module (a typo, a renamed DimOS class) skips 15
  tests silently instead of failing them. The same holds for the new `test_cockpit_data_mutants.py`, which copies the
  pattern for consistency. Skipping only when `dimos` itself is missing would fix this class.
- GPU: the filament tests (`test_filament_*`, `test_mujofil_default`, `test_floor_photos`) render through Vulkan on the
  NVIDIA card. On a GPU-less machine (the home server) they need mujofil's software path or they skip.
- `tests/test_floor_photos.py` and `test_house_photo_floor.py` skip without the gitignored floor zips
  (`tools/fetch_floors.sh`).
- Network: `test_recordings.py::test_nothing_goes_outside_the_house` calls `getaddrinfo` (localhost and literal IPs only,
  so no lookup leaves the machine).
- Network specifics in the repo: `test_recordings.py` and `test_redteam_data.py` carry a literal LAN address and user
  name. Any private address shows the same thing, so a documentation-range or generic `10.x` address would keep the
  house's network out of the repo.

**Wall-clock and timing**

- `tests/test_gpu_share.py`:
  - Real subprocesses and `stdout.readline()` with no timeout. If the lock logic regresses, a test blocks the whole suite
    forever instead of failing.
  - Asserts `time.time() - t0 > 1.0` and `< 5.0`.
  - A `pytest-timeout` mark, or reading with a deadline, would bound it.
- `tests/test_settled_weighing.py`: `LaggingArm` settles on the wall clock (0.5 s). `fresh.joint_efforts()[0] > 1.0`
  fails if the machine stalls 0.4 s between building the arm and reading it (a night pipeline running alongside).
- `tests/test_live_localize.py::test_the_live_localizer_matches_scans_as_they_arrive`: background thread, `sleep(0.3)` x6
  and a poll of up to 10 s. Slow (2 s minimum) and load-sensitive.
- Random seeds: every `default_rng` and `random.Random` in the suite is seeded. None found unseeded.

## Ranked: tests still to add

1. **Stop the cockpit tests from skipping silently.** Import `houndkeeper.cockpit_data` unguarded when `dimos` imports.
   Today a broken cockpit module turns 15 tests into skips.
2. **Make preflight's disk check independent of this machine.** Monkeypatch `shutil.disk_usage` in
   `test_a_session_refuses...`, both above and below the floor.
3. **Bound `test_gpu_share`'s reads.** Use a deadline per `readline`, so a lock regression fails in seconds instead of
   hanging the suite.
4. **[R1X] for the frame consumers.** The morning log's crops and any export must call `require_screened`. Test each
   consumer with a recording flagged as showing people (only `require_screened` and `hk-data path` are tested).
5. **`frame_lag` on garbled frames should keep its correlation above 0.9.** This fails today. It needs the response
   floor raised, which is a design decision (finding 1).
6. **Mount yaw against roll ([KkQ]).** Decide whether roll matters for the floor map. If it does, add a procedure or view
   that observes it, and a test with true roll that is not zero (finding 2).
7. **[KbT] on a genuinely second night.** Use a different recording from the first test, or say "one night" in the spec.
8. **The floor map error's dated log line ([nv5]).** Run `main --error` on a tmp log and check the appended record.
9. **`detect_changes2.main`'s blob logic** (small-and-faint drop, link-px joining, "something left" against "something
   here"). It is frozen for the hold-out, but it has no unit test of its own.
10. **arm-host [4WJ] and [871]'s other half.** Check the report's values and cadence, and that an update is refused
    unless stowed or docked.

## Tests added in this review

All are new files, linked with railyard, and `railyard check` shows no findings from them. The three findings it reports
come from `docs/findings/railyard-gap-2026-10-03.md`.

| File | Tests | Covers |
|---|---|---|
| `tests/test_floor_mutants.py` | 17 | [nv5], [FHA], [V35]; about 20 s |
| `tests/test_calibrate_mutants.py` | 3 | [oI1], [KkQ]; the mount test about 15 s |
| `tests/test_measure_mutants.py` | 8 | [4hm] |
| `tests/test_recordings_mutants.py` | 8 | [R1X], [YP8], [u6u], [B2O], [Zw9], [fya] |
| `tests/test_cockpit_data_mutants.py` | 6 | [vRJ] |
| `tests/arm_host/test_release_mutants.py` | 4 | [871] |
| `tests/arm_host/test_eyes_mutants.py` | 3 | [GTG] |
