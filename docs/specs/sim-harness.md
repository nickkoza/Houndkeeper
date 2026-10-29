# Sim harness

**Type:** Capability spec · **Status:** Draft · **Source:** deliverable D0; principles [33f], [5oU], [1kb] · **ID:** [jX2]

## Purpose

The umbrella for every sim scenario: grasp [8JV], tip safety [BPL], walking [pZM], reach [RHS], battery [qNo], house nights [dws] and the full slice [YWo].
A simulated Go2 carrying a Maker Arm that does the whole tidy loop in a house (sweep, find what is new, look closer,
pick it up, put it in the bin on its back) running on DimOS, plus fast single-skill scenarios that tell us, with a
recorded score, whether a change made the robot better or worse. It is where every capability is proven before it
is tried on the real robot.

## Acceptance criteria

1. [ut2] One command runs a named scenario and appends its result, with the code version of both this repository and
   DimOS, to a results history. Scenarios: grasp, tip, walk, reach, power, slice.
2. [p2t] Every scenario opens the live viewer unless asked not to.
3. [gNg] The grasp scenario ([8JV]) succeeds on at least 90% of trials over at least 40 trials.
4. [1zs] The robot never tips during any scenario: it never passes its measured tipping point ([BPL]), and any run
   where it does is reported as a failure with the arm's pose, torques and contacts at that moment.
5. [zwe] The arm never moves outside the tip-safety envelope ([BPL]) in any scenario.
6. [2UQ] The full slice ([YWo]) runs on DimOS.
7. [uxL] In the slice scenario, every item the floor comparison flags is either picked into the bin, named "nothing" by
   the look-closer step, or flagged for human review. None is silently dropped.
8. [rwj] The slice scenario reports battery used per pack and the area covered, from the battery model.
9. [OfO] No scenario requires network access once models are cached.
10. [bIu] Physical parameters come from the robot parameters (CALM node `robot-parameters`); a scenario never
    hard-codes one.
13. [3Pv] **The sim behaves the same at any speed.** Every delay the sim injects (camera frames,
    odometry, lidar, commands) is held in sim time: a message becomes readable only once the sim's clock reaches its
    capture time plus its delay, and is stamped on a robot clock that advances with sim time. Every time constant on
    the robot's side that judges sim data (pose gaps, lag estimation, timeouts, control rates) reads that clock, not
    the wall clock. A night run fast and the same night run slow give the same results within run-to-run noise; the
    sim's real-time factor is a cost, never a variable in the results.
14. [5SS] **Every run records its speed, and a run too slow to trust says so.** Each run's summary and score carry its
    sim/wall ratio and frame interval; until [3Pv] is measured, numbers from a run below 0.9 sim/wall are marked
    "latency-flattered" wherever they are summarised.
15. [01j] **A shared baseline is never overwritten unasked, and never without a copy.** A night whose baseline was
    rendered differently (its stamp changed) refuses to re-sweep it unless asked (`FRESH_BASELINE=1`); a re-sweep of an
    existing baseline first copies the whole run directory into its `archive/`; an experiment runs on baselines of its
    own (`RUN_DIR`). Hard-won, 5 Oct 2026: one agent's uncommitted render change silently re-swept the shared baselines
    of houses 72 and 96 under other agents' runs, overwriting the only copies of house 72's night-1 path (recovered from
    scratch copies).

## Non-functional requirements

- A full slice run completes in under 15 minutes of wall time on the development laptop (RTX 2070), viewer on.
- Per-trial time for the grasp scenario is set in [8JV].

## Failure behavior

- A scenario whose physics diverges (non-finite state, bodies interpenetrating at start) stops and reports it; it
  never reports a score.
- If the local VLM or a model file is unavailable, the look-closer step reports "unavailable" and the item goes to
  human review; it never guesses.

## Decisions

1. [Rpj] **The robot's clock is DimOS's `dimos.utils.clock`, published by the sim.** Hard-won: the sim ran at 0.16-0.48x
   real time while its delays, stamps and the robot's loops were in wall time, so every delay cost a fraction of the
   robot motion it will (docs/research/sim-kindness-audit-2026-10-03.md, findings 1-2), and on house 93 frames and poses came
   0.35-0.43 s of wall time apart, which aliased the lag estimate and starved the pose-gap gate
   (house 93 trial, 3 Oct 2026). Shaped like ROS's `use_sim_time` so it can go upstream.
   - The sim (`mujoco_process.py`) ticks a `SimClock` once a loop: robot time is the wall time of the first tick plus
     the sim time since, never turned back by a reset. It is published at `DIMOS_SIM_CLOCK` (a file under /dev/shm,
     seqlocked); `sim/dimos/env.py` names one per launch. Unset, the sim stamps on the wall clock as DimOS did.
   - Every message is held in a `DelayLine` until robot time reaches its capture plus its delay (and a stall's end, for
     a frame held through one), then stamped on arrival, as DimOS's Go2 connection stamps the real robot's: frames
     (`frame_delay_s`), odometry and lidar (`odom_delay_s`), and commands in `MockController` (`cmd_delay_s`). A stall
     lets `stall_burst_share` of its frames through at its end, one a loop (the shared memory holds one).
   - The robot's side reads `clock.now()`, `clock.sleep()` and `clock.wait()`: the tidy module, the arm interface, the
     cockpit, and DimOS's replanning navigator (control rate, stuck windows). `tests/test_robot_clock_static.py` lists
     every wall-clock read left in houndkeeper's modules.
   - The connection polls each shared-memory slot at 200 Hz of wall time, reading only a new sequence number, so no
     message is missed at any speed up to a loop of 5 ms of wall time.
2. [Bpe] **A run's speed is read from the sim's own report, and under 0.9 sim/wall its numbers are marked.** The sim's
   SIM_SPEED line (every 30 s of wall time, the last one the whole run's) carries sim/wall and the head frames'
   interval in wall and robot time; a run without one falls back to traj.csv (`sim_t` against `host_t`, or `wall_t`
   before the robot's clock) and an interval derived from a frame every two 35 ms loops. `sim/dimos/run_speed.py`
   reads both for score_tidy, the round, vslice and detect-run summaries. The 0.9 is [5SS]'s; the mark
   stays until [3Pv]'s two-speed night is measured.
3. [LPl] **Frames reach the robot at most every two sim loops (70 ms of sim time), not 20 a second.** Observed in the
   code 3 Oct 2026 (fork at a7492431f): the loop covers 35 ms of sim time and renders once 50 ms have passed since the
   last frame, so frames are 70 ms apart at any speed (house 93's 374-431 ms of wall time at 0.16-0.18x is that). It
   is the same at any speed, and harsher than the Go2's camera; recorded so a frame-rate figure is read in sim time.
4. [x5h] **DimOS's tf buffer extended; houndkeeper's copies retired.** A pose at a sample's own time ([Feg], waymark
   [uKM] decision [929]) is looked up in DimOS's time-indexed transform buffer, live and over recordings, not in
   lookups of our own: an application of [eM8] (its register, [eK3], entries 5 and 20). The fork branch
   `nick/feat/tf-interpolation` adds, in DimOS's terms and for any DimOS user: `get(..., interpolate=True,
   max_gap=...)` (linear translation and slerp rotation between the samples either side, every edge of a chain, never
   extrapolated, refused across a wider bracket); a buffer clock for "now", which the running fork sets to
   `dimos.utils.clock.now` ([Rpj]); and an odometry stream as a recording lookup's moving edge (`StreamTF`). The default
   stays nearest-sample: DimOS's callers look up mount edges published at 5 Hz or once, which interpolation would
   refuse. Our side is `houndkeeper.rec_poses` (offline: floor_mosaic2, confirm_stream and three
   since-retired analysis tools, each of which had its own slerp) and `tidy_module._head_camera` (live: interpolated, the nearest
   pose only for a frame newer than the last pose). Measured 4 Oct 2026 on house 70 night 1 (recording
   20261001-232147, 4911 head frames): every frame's pose agrees with the retired lookup to 4e-15 and the same frames
   are placed; the night's floor picture (`floor_mosaic2.py --gain --dt auto --traj`, 1226 frames) is byte for byte
   the same file before and after, its floor map error ([nv5]) 9.07% of 90.3 m2 both times. Differences by
   construction: a frame stamped exactly on the first or last pose is now placed (it was dropped); every offline
   reader refuses frames across a pose gap over 0.2 s, as the others did (none on that night); the live
   head-camera pose, nearest before, is now interpolated (not yet measured in a sim night).
   A recording lookup goes by the transforms' own stamps, not the times the recorder stored them at ([3Pv]): a sim
   slower than real time stamps on the robot's clock, and recorders stored a message without a stamp of its own (a
   tf message) at the wall clock, up to 413 s after its stamp on house 96's walk. `StreamTF` scans each stream once
   for its stamps and reads the stored times they map to; DimOS's recorders now store such a message at
   `dimos.utils.clock.now()` (the wall clock off the sim), so stored time and stamp agree, and older recordings are
   read through the stamps. Measured 4 Oct 2026: house 96's walk (20261003-200756, 5076 head frames), poses found
   for 1975 frames before and 5076 after, every frame the retired lookup placed; house 70 night 1's floor picture
   still byte for byte the same.

5. [UvE] **Our sim and costmap settings are DimOS options set from houndkeeper; the fork carries none of our values.**
   Conventional: fork rule 6 (`docs/dimos-fork.md`): a DimOS change is an extension point or a general feature,
   so the fork's defaults are the stock sim's and every houndkeeper value is set from our side. Audit:
   the DimOS overlap audit of 4 Oct 2026 (private notes), section 19 and finding 1.
   - The fork's sim knobs are typed `GlobalConfig` fields (`mujoco_*`, listed by `dimos show-config`), each also read
     from its `DIMOS_SIM_*` name, so `sim/dimos/env.py` sets them as before. Stock defaults: the person figure on,
     MuJoCo's default sensor geom groups, the arm camera rendered every frame, no named truth bodies, the walking policy
     on a GPU when there is one. `env.py`'s `SIM_OPTIONS` sets ours: no person, sensors see group 4 (the outside walls'
     boxes, [yyJ]), the wrist camera a frame a second while the Maker Arm rests within 0.15 rad of its all-zero fold
     ([hJ5]), the policy on CPU; `tidy_run.sh`, which asks for the truth file, adds `trunk` and `tray` to it.
   - DimOS's costmap keeps its 0.6 m pass-under and keeps every point unless `ignore_above` is set. CostMapper takes a
     sparse occupancy config from the environment (`COSTMAPPER__CONFIG__<FIELD>`) or a config file, built as the
     algorithm's own config class. `env.py`'s `COSTMAP` sets pass-under 3.0 m (the rule off) and `ignore_above` 2.0 m
     ([Xl1]); `sweep_night.sh` exports them too and refuses to run unless the config DimOS will run with has them
     (`sim/dimos/effective_config.py`).
   - A frame of the sim loop covers the same sim time at a finer physics step for each robot's own step (the fork had
     taken the Go1's 5 ms for every robot, so the G1 ran twice the steps a frame).
   - `tests/test_dimos_fork_rule6.py` fails if the fork's diff names houndkeeper, reads an `HK_` variable or cites one
     of our spec IDs. Proved 4 Oct 2026 without a sim: the MJCF models our tidy (house 70 night 3, office1) and sweep
     (house 70 night 1) runs build hash the same before and after, and the effective config equals the old hard-wired
     values (`tests/test_dimos_options.py`).

## Tests

| Criterion | Test | Status |
|---|---|---|
| [ut2] (one command runs a scenario; result recorded with both code versions) | `sim/suite/run <scenario>`, `sim/suite/history.jsonl` | Partial: DimOS commit not yet recorded |
| [p2t] (viewer on unless asked not to) | `--headless` flag per scenario; `DIMOS_SIM_HEADLESS=1` for DimOS nights (`tests/test_headless_viewer.py`) | Partial: grasp, slice, tip, DimOS nights yes; walk, reach, power open no viewer |
| [gNg] (grasp scenario >= 90% over >= 40 trials) | `sim/suite/run grasp --trials 40` | Failing: below target; results in [8JV] Decisions |
| [1zs] (never tips; a tip is a reported failure with pose, torques, contacts) | tip alarm in `sim/arm/vslice.py`; grasp bench records TIPPED OVER | Partial: alarm reports, not yet a pass/fail gate in every scenario |
| [zwe] (tip envelope produced and enforced everywhere) | `sim/suite/run tip`; envelope check in the arm planner | Partial: envelope produced, not enforced |
| [2UQ] (full slice runs on DimOS) | `sim/dimos/tidy_run.sh` | Partial: the loop runs on DimOS; the standalone slice's floor comparison is not in it yet |
| [uxL] (every flagged item picked, named "nothing", or sent to review) | slice item accounting | Partial: standalone slice only |
| [rwj] (battery used per pack and area covered) | slice battery report | Partial: HUD only, not in results |
| [OfO] (no network needed once models are cached) | run the suite with networking off | Not yet written |
| [bIu] (parameters only from the robot parameters) | check for hard-coded parameters in sim/ | Failing: values still duplicated in sim/arm; the mount position is read from the parameters |
| [3Pv] (the same at any speed) | unit, on a fake sim clock: `tests/test_sim_clock.py` (held until capture plus delay; stamps follow sim time at 0.16-2x; a stall's frames late in a burst; commands late; the same message sequence at 0.16, 0.5, 1 and 2x gives identical stamps, decisions and stopping point, and on the wall clock it did not); static: `tests/test_robot_clock_static.py` | Partial (3 Oct 2026): unit and static passing, 10 tests; `mind.py` (`Mind.json`) still reads the wall clock, listed |
| [3Pv] (the same at any speed) | unit: a recording stored on the wall clock while stamped on the robot's, lag growing to 400 s, every pose found at its own time: `tests/test_rec_poses.py` (and DimOS's `dimos/protocol/tf/test_tf.py`, `dimos/memory/test_tap.py`); recorded: house 96's walk, a pose for each of its 5076 frames (skipped where the recording is missing) | Passing (4 Oct 2026) |
| [3Pv] (the same at any speed) | sim: one tidy night at two speeds, results within run-to-run noise: `ROOM=nights_s70_n1 ITEM_SEED=1 DIMOS_SIM_SPEED=1 sim/dimos/tidy_run.sh`, then the same with `DIMOS_SIM_SPEED=0.3`; compare the two `score.json` (found, binned, stops for nothing, `t_s`) and `vslice_summary.py` over both | Planned |
| Our DimOS settings set from houndkeeper, none in the fork (decision [UvE]) | `tests/test_dimos_options.py` (the effective DimOS config our scripts produce equals the old hard-wired values; the costmap check refuses stock); `tests/test_dimos_fork_rule6.py` (the fork's diff names no houndkeeper, reads no `HK_`, cites no spec ID of ours) | Passing (4 Oct 2026); behaviour on a sim night not yet compared |
| [5SS] (speed recorded, slow runs flagged) | `tests/test_run_speed.py`: the sim's SIM_SPEED line (sim/wall, frame interval) or traj.csv read into score.json; round, vslice and detect-run summaries mark nights under 0.9 latency-flattered, and a night without a speed "speed not recorded" | Passing (3 Oct 2026): 8 tests; no sim run yet carries the new frame-interval line |
| [01j] (a shared baseline never overwritten) | `tests/test_baseline_guard.py` (a stale baseline refused without `FRESH_BASELINE=1`; the run directory copied aside once before a re-sweep; `RUN_DIR` honoured) | Passing (5 Oct 2026) |
