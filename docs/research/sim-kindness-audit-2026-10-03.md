# Where the sim is kinder than the real world: audit, 3 Oct 2026

> **Why this is here:** principle [x3f] says the sim is never kinder than the real world; this is the code-reading audit of where it was. Read it beside research/sensitivity-2026-10-13.md: this audit listed camera frame delay as modelled, and the sensitivity study later found it never had been.

Principle [x3f] says the sim is never kinder than the real world. Related principles: [33f], [CaM], [Feg], [fXD], and
"the robot knows only its sensors". This audit reads the code only. It ran no sims and opened no hold-out (80/81) logs or
images. Every finding below was checked against the code (file:line given). Anything not confirmed is marked
**unconfirmed**. Findings are ranked by how much they could flatter results, highest first. Severity: H (could move a
headline number a lot), M, L.

What is already modelled, so it is not repeated below:
- odometry drift and delay; frame stamp delay, frame loss and stalls; the lidar's point budget, range noise and
  dark/shiny dropouts;
- the head camera's fisheye, mount error, motion blur, rolling shutter, noise, auto-exposure, JPEG and bitrate dips;
- the arm's flex, backlash and partial gravity feed-forward (these are applied in the DimOS night sim too, through
  `sim_hook.add`, then `go1_arm.add_arm`, then `add_imperfections`).

Fixes made in this audit: **none**. Every fix below changes a sim-wide behaviour or a rendered stamp. Several would
force baseline re-sweeps (GPU), touch the DimOS fork patch, or need a design decision. So each is a proposal with a
concrete change.

---

## 1. [H] The sim runs 2 to 6 times slower than real time, but every delay, stamp and control loop is in wall-clock time, so latency costs a fraction of what it will

- **What the sim does**
  - The sim steps physics in sim time.
  - It stamps odometry, frames and lidar with `time.time()`. See `dimos/simulation/mujoco/mujoco_process.py`:
    `state_t = time.time()`, `shm.write_video(pixels, state_t + frame_delay())`, `ts=time.time()` for the lidar.
  - The injected delays (`frame_delay_s` 150 ms, `odom_delay_s` 30 ms) are in wall seconds.
  - DimOS's navigation, the tidy module's loops and perception (OWLv2 looks, the Confirmer) all run on the wall clock.
- **Measured speed**
  - Recent baseline sweeps (`sim/house/runs/nights_s9*/…sweep.log`, `SIM_SPEED` lines) ran at **0.16 to 0.20** sim/wall.
    `sweep_night.sh` runs "fast (max)", which today is slower than real.
  - The tidy run (`DIMOS_SIM_SPEED=1`) reaches **0.48** (`data/runs/dimos_tidy/*/dimos.log`). Cameras take 69% of the loop.
- **What reality does:** the robot moves at full speed in the same clock the delays and compute live in.
- **How it flatters results**
  - **Frame placement.** A frame stamped 150 ms late misplaces the floor by v·150 ms in reality. In the sim it is
    v·150 ms·(0.18 to 0.48), which is 27 to 72 ms of robot motion. At 0.5 m/s that is 1.4 to 3.6 cm instead of 7.5 cm.
    Turning at 1 rad/s, it is 1.5 to 4 degrees instead of 8.6.
  - **The lag correction.** The jitter left after correcting the lag shrinks by the same factor.
    `calibrate.frame_lag` and `floor_mosaic2.lag_from_turns` measure the lag in wall time on wall-time turning rates.
  - **Control and perception.** Every controller and detector gets 2 to 6 times more updates per metre walked.
    - Obstacle avoidance and stopping on a confirmed find look much better than they will.
    - A detector that takes 1 s of compute costs 0.2 to 0.5 s of robot motion.
    - The project has seen this once already. On 23 Sep the overnight run at real time did worse than fast mode
      (private video notes: by then the tuning was chasing sim results rather than real ones).
  - **Arm timing distorts the other way.** Arm commands stream in wall time, so in sim time the arm moves about twice
    as fast as commanded. That is harsher dynamics, but it shortens the energy the arm uses in sim time.
- **Fix**
  1. Gate first. Write each run's sim/wall ratio into its score or summary. Mark any number from a run under 0.9 as
     "latency-flattered" in `round_summary.py` and `vslice_summary.py`.
  2. Then change `mujoco_process.py`:
     - Hold the delays in sim time. Queue each frame and odometry message until `data.time >= capture + delay`, rather
       than stamping a future time. Stamp them on a robot clock that advances with sim time (for example
       `wall0 + (data.time - sim0)`).
     - Until that clock is in place, a stopgap: scale each injected delay by `1/ratio` using a rolling ratio, so that
       delay times robot speed matches reality.
  3. Longer term, give DimOS a sim clock (lock-step), so wall-time compute is charged in sim time.

## 2. [H] Frames reach the robot at once; only their stamp is late. Stalls drop frames instead of delivering them in a late burst, and nothing delays commands or lidar

- **What the sim does**
  - `shm.write_video(pixels, state_t + frame_delay())` makes the frame readable at once, with a stamp 150 ms in the
    future. The fork patch's `MujocoConnection._stamp` passes that stamp through.
  - Live code (`tidy_module._watch_frame` → `_head_camera(ts)`, tf tolerance 0.2 s) therefore reacts to each frame
    about 150 ms earlier than the real robot can.
  - `StreamModel.delivered` returns False during a stall, so the frames are lost. A real WebRTC jitter buffer often
    delivers them late in a burst, stamped together on arrival, which makes their stamps badly wrong.
  - `MockController.get_command` reads `cmd_vel` straight from shared memory: no command delay, and no smoothing like
    Unitree's sport mode.
  - The lidar is stamped `time.time()` at render, with no delay and no scan-time skew.
  - Odometry never stalls when the video does, though both share one link.
- **How it flatters results:** reaction to a find, stopping distance and the approach all get about 150 ms (plus
  command latency) for free. Also, no frame is ever placed with a stamp off by hundreds of ms.
- **Fix** (in `mujoco_process.py` and `stream_model.py`)
  - Deliver frames late. Keep the stamp-on-arrival model, but make the frame readable only at its arrival time.
  - Add `cmd_delay_s` (estimate 50 ms mean, 20 ms jitter; WebRTC/DDS on the same link, measure in
    real/calibration/timing.md) to `params/robot.yaml`, applied in `MockController`.
  - During a stall, deliver some of the held frames in a burst at its end (`burst_share` est. 0.5) rather than drop
    them all.
  - Give the lidar the odometry delay.

## 3. [H] The sweep route is planned on the generator's true floor plan

- **What the sim does**
  - `sim/house/sweep_route.py:9,20-26,50-62`: `import make_house as mh`, `g = mh.base_grid()`.
  - The route uses a BFS over the true walls and furniture from the true dock, lawnmower rows over the true free floor,
    and waypoints pushed 0.7 m clear of the true furniture.
  - `make_house.py:153-154` loads the plan from `HK_HOUSE_SEED`, which `night_tidy.sh:21` exports.
  - `night_tidy.sh:89` (tidy) and `sweep_night.sh` (baselines) both use the route.
- **What reality does:** the real robot has no plan. It has only its own lidar map and the earlier nights' maps.
- **How it flatters results:** coverage, unreachable waypoints, stalls near furniture and time per night are all
  measured with a perfect map. This breaks "the robot knows only its sensors".
- **Fix**
  - Build the route from `house_map.py`'s output (or DimOS's costmap) after an exploration night.
  - Add a test that fails if any script on the robot's path imports `make_house`. Only the generator and the scorers
    may.

## 4. [H] Toys are placed only where they are easy: open floor, upright, well apart, and inside the swept area, from an item pool without the hard kinds

- **What the sim does** (`sim/dimos/house_items.py`)
  - `clear(p, margin=0.5)` (:71) keeps every toy 0.5 m from every wall, every piece of furniture, tonight's chairs and
    the bowls.
  - `start_clear=1.0` (:45) keeps toys 1 m from the dock, and `WALK_RADIUS = RADIUS - 0.5` (:54) keeps them inside the
    swept area.
  - `SPREAD far=1.5` puts toys 1.5 m apart, falling back to 0.8 m; a pair or a trio at most.
  - The item pool (:69) keeps only `gso/box/cylinder` items with fate `bin/auto`. That excludes socks, towels, rope,
    balls and every "review" item: card or paper, a game case, a shoe, a teddy, a toy dog.
  - Each item spawns in its scan's canonical pose with no yaw (`items.py:186`, `add_frame(pos=…)`).
  - The Confirmer's blind band (`band_m 0.20` around mapped structures) never holds a toy, by construction.
- **What reality does:** toys collect along baseboards, under tables and the sofa, in corners and in heaps, on their
  sides or upside down. Socks, paper and cords are among the commonest things on a floor.
- **How it flatters results:** found and binned both leave out every occluded, edge-adjacent, piled or off-route toy.
  Grasping only ever meets toys at their most stable, scanned pose.
- **Fix**
  - Draw 30 to 40% of toys from "hard spots": 0 to 0.3 m from structures, under tables, between chair legs, in
    corners.
  - Allow touching clusters, and put some toys outside the swept radius, scored as missed.
  - Give each toy a uniform yaw and a random drop and tumble, settled before the run. `grasp_tier(name, quat)` already
    handles any landed pose.
  - Add cloth, rope, ball and flat-paper items, plus "leave alone" items, scored separately.

## 5. [H] The jaws are the "rubber" stand-in in every sim, gripping 14 to 200 times harder in twist than the real fingers

- **What the sim does**
  - `sim/arm/jaws.py:7`: `JAWS = os.environ.get("HK_JAWS", "rubber")`. Nothing in the repo sets `HK_JAWS`.
  - `go1_arm.add_jaws` returns early for "rubber" (`go1_arm.py:55`), so the fingers keep
    `friction=[1.2, 0.02, 0.001]` (:154).
- **What reality does**
  - `params/robot.yaml`: `jaw_friction_bare [0.4, 1e-4, 1e-4]`, `pad_tpu_mu 0.6`.
  - Twist resisted at 7.5 N: 0.02 m gives 150 N·mm per jaw, against about 11 N·mm for the v3 pad (waymark 0006).
  - Waymark 0006 itself calls rubber "not a real jaw".
- **How it flatters results:** held items can't slip or twist out. The widest item accepted is 95 mm, against 89 mm
  with pads, and this feeds both the robot's "too wide" call and `score_tidy`'s tiers.
- **Fix**
  - Set `HK_JAWS=pads_v3` (or `bare` until pads are fitted) in `sim/dimos/env.py`, and make that jaws.py's default.
  - Record `HK_JAWS` in `sim_hook._imperfections`, so runs with different jaws aren't compared.

## 6. [H] The floor comparison meets the same light, the same floor and a perfectly clean reference every night

- **What the sim does**
  - **The same light.** `night_tidy.sh:29` defaults `REGIME=lit`, while its usage line says "(night)".
    `make_nights.py:283-286` returns one fixed lighting for every night, baselines included: ambient 0.16,
    ceiling 0.9, lamp on, ceiling position fixed per house.
  - **The same floor.** The only per-night floor change is a 1 px roll and ×0.99 to 1.01 brightness
    (`make_nights.py:202`). Carpet wear lanes are seeded per house.
  - **A clean reference.** The baselines use `--per-night 0`: nothing on the floor but furniture, chairs and bowls.
    Nothing carries over between nights.
- **What reality does:** other lamps on or off, TV glow, headlights, bulbs warming up, vacuum stripes, footprints,
  crumbs and pet hair. Baselines always hold shoes, a basket and yesterday's missed toy.
- **How it flatters results:** lighting and floor change can never cause a false find, and a contaminated reference
  never hides a toy. Whole sources of false alarms and misses are missing.
- **The lit regime is a deliberate choice.** It stays the base; only per-night variation is added on top.
- **Fix**
  - Draw a per-night perturbation inside `lit`: ambient ±20%, one extra lamp on or off, colour temperature ±300 K, and
    an occasional TV glow.
  - Add per-night floor marks: pile-lay patches and specks.
  - Seed the baselines with persistent clutter, some of it tidy targets, and score toys present in the baseline
    separately.

## 7. [H] Friction is 1.0 for every foot, toy and floor; wheeled toys never roll and feet never slip

- **What the sim does**
  - Floor plane and relief slabs: `friction="1.0"`, floor priority 1 (`make_nights.py:972,1016`).
  - Items: `[1.0, 0.02, 0.001]` (`items.py:178,189`).
  - Go1 feet: MuJoCo's default 1.0.
  - Wheeled toys (fire truck, bus, baby car) are single rigid bodies.
- **What reality does (estimates):** rubber feet on polished wood or tile are about 0.5 to 0.7. Plastic or ceramic on
  hard floor is about 0.2 to 0.5, so items skid or roll when a jaw touches them.
- **How it flatters results:** grasps never push the toy away; walking never slips on tile or near the water bowl.
- **Fix**
  - New params, `world.friction` per material (estimates; measure with a tilt test):
    - floor: wood/tile 0.5, carpet 0.9;
    - items: ABS 0.35, ceramic 0.3, rubber 0.8, fabric 0.6;
    - randomise ±20% per night.
  - Low rolling friction (or wheel joints) on wheeled toys.

## 8. [H] The hold-out is not independent of the tuning

- **What the sim does**
  - `make_house.py:40`: `FIXED_PLAN = {70, 71, 80}`. Hold-out house 80 had the same plan (walls, furniture, dock,
    carpet, route) as tuning houses 70 and 71.
  - Every other house, 81 included, is a variant of one plan: groups moved at most 3 cells, sometimes mirrored, with
    the same outer walls and the same dock cell.
  - Baselines are swept once per house and reused across tuning rounds (`night_tidy.sh:58-65`), so every round
    compares against the same two recordings and the same odometry-drift draw.
  - The three item seeds share one house, one route and one pair of baselines, so they aren't independent nights.
  - **Unconfirmed:** `holdout.sh` → `stitch_baselines.sh:34-35` may write s81's mount refinement back into
    `data/robot/head_mount.json`, which later tuning starts from.
- **Fix**
  - Label the s80 hold-out results "same plan as tuning" wherever they are cited. Never put a hold-out seed in
    `FIXED_PLAN`.
  - Draw hold-outs from a different plan generator (room count, size, dock).
  - Re-sweep the baselines with a fresh drift seed for each validation round.
  - Treat item seeds within a house as one sample in confidence intervals.
  - Have `holdout.sh` point `ROBOT_MOUNT` at a temporary copy.

## 9. [M-H] The arm sags 5 times less than the real one will, and the standalone benches see its true pose

- **Gravity feed-forward**
  - **What the sim does:** `go1_arm.py:190-194` sets `gravcomp = 0.8` (`maker_arm.gravity_ff_fraction`).
  - **What reality does:** the params note says the maker-arm SDK sends none (tau 0), and `arm_host/` has no
    feed-forward either (grep).
  - **How it flatters results:** at floor reach with a 0.3 kg item, a bare PD sags about 0.047 rad at the shoulder,
    about 2 cm at the jaws. With 80% compensation it is about 0.5 cm.
- **Benches read the true pose**
  - **What the sim does:**
    - `wrist_refine.py:36` takes the camera pose from live physics (`d.cam_xpos`, `d.cam_xmat`), which includes the
      true flex, lash and trunk pose.
    - `vslice.py:142,246-345` copies true `qpos`, flex joints included, into the IK and collision copy.
    - `base_xy` and `grasp_point` (`vslice.py:454-455`) are truth.
    - The DimOS path (`DimosRobot`) is correct: encoders plus odometry.
  - **How it flatters results:** grasp_bench, eval_policy, vslice_multi and record_demos numbers carry no arm-pose
    error.
- **Fix**
  - Set `gravity_ff_fraction: 0.0` (source: measured, "maker-arm-sdk b30d05a arm.py:316 sends tau 0") until the arm
    host sends a feed-forward.
  - Give the standalone Sim a belief copy: motor joints only, flex and lash zero, base from drifted odometry. Add a
    test that the capture pose differs from the truth when `HK_ARM_IMPERFECT` is on.

## 10. [M-H] Scoring counts generously

- **Finds**
  - `score_tidy.py:23,42`: any stop within 0.30 m of a toy counts as found, whatever the stop concluded. That includes
    "not seen standing" and a stop meant for another spot.
  - For long toys the radius grows to footprint + 0.15 m.
  - This "found" feeds the hold-out bar `FOUND_MIN 7/9` (`round_summary.py:39`).
- **Falls**
  - `tidy_module.py:114` records `fell`, but `score_tidy.py` never reads it. Items binned after a fall still count.
- **Errors left out of the counts**
  - Binning a non-item (a pet bowl, a chair part) is never penalised. `stops_for_nothing` skips any stop whose outcome
    starts with "binned" or "same as" (:29).
  - Unscored nights drop out of found/items silently (`round_summary.summarize`).
  - Items are scored at their spawn point, not where they settled (`night_tidy.sh` heredoc, :74-80).
- **Benches** (`bench_detect.py`, `confirm_stream.py:521`)
  - "Unsure" counts as detected within 0.30 m.
  - The replay uses tonight's finished map as mapped structure (`tonight=True`), which the live robot lacks.
  - Stain stops aren't counted as false.
- **Fix**
  - Count a find only when the stop's close look saw an object within about 10 cm, or binned it. Report the lenient
    number beside it.
  - Fail a night (or stop crediting items) at its first fall.
  - Check every truth body in the bin.
  - Count unscored nights as 0 of their toys, and print "N of M nights scored".
  - Score items at their `rest` positions.
  - Replay benches with `tonight=False`.

## 11. [M] Odometry has no turn-dependent yaw error, no gyro bias, and starts where the house's own frame does

- **What the sim does** (`dimos/simulation/mujoco/odom_drift.py`)
  - Heading error grows with distance (`yaw_sd·√ds`) and with time (`yaw_rate_sd·√dt`) only. Turning on the spot adds
    almost none.
  - There is no constant gyro bias: the error grows as √t, not linearly in t.
  - The believed start is the true start ±2 cm / 1°.
  - The true start is identical every night: `data.qpos[0:3] = start_pos`, same yaw.
  - Odometry is expressed in the sim's world frame, which is the house frame.
- **What reality does**
  - Legged odometry's heading error is largest when turning in place, from foot scuff: a yaw scale error of a percent
    or more of the angle turned.
  - MEMS gyro bias drifts degrees per hour.
  - The Go2's odometry starts at (0, 0, 0) wherever it booted. Until a dock exists, it is placed by hand
    (±5 cm, ±3 to 5°).
  - Also, `floor_mosaic2.py:16` sizes the canvas to this house exactly.
- **How it flatters results:** the sweep's many turns cost little heading. Night-to-night alignment is nearly given.
- **Fix**
  - New params in `params/robot.yaml`: `odom_drift.yaw_scale_sd 0.01` and `yaw_turn_sd_deg 0.5` per √rad turned
    (estimates; measure with real/calibration/odometry.md's loop, adding spins on the spot), and
    `yaw_bias_deg_per_h` sd 3 (estimate).
  - Raise `start_yaw_sd_deg` to 3 and `start_xy_sd` to 0.05 until a dock exists.
  - Vary the true start by the same amount each night.
  - Start odometry at the origin rather than in house coordinates.

## 12. [M] The world holds no pet, no person, no cables, no doors, no stairs, no thresholds; chairs never move when bumped; the rug never moves; the carpet is rigid

- **What the sim does**
  - `DIMOS_SIM_PERSON` is off. The pet appears only as bowls nudged by 3 cm sd (`make_nights.py:906`).
  - No cords anywhere.
  - One open-plan floor with stub walls, furniture on a 35 cm grid with a free cell around each piece
    (`make_house.py:131`).
  - Chairs are static bodies (`movables.py:4`).
  - `RUG_SHIFT = (0, 0)` (a deliberate choice, recorded here as an accepted kindness).
  - Carpet and rug are rigid 12 mm / 8 mm slabs (`make_nights.py:900`), so a Lego plate never sinks into the pile.
- **Fix**
  - A pet: a mocap body on a random walk (reuse `person_on_track.py`) that sometimes moves a toy. Bowl nudge sd 0.10 m.
  - Lamp and TV cords from the `rope` composite, marked do-not-pick.
  - A door whose state changes per night, a 1 to 2 cm threshold, and one step edge.
  - Chairs on free joints with high damping.
  - Soft solref/solimp on the carpet geoms (a few mm of sink).

## 13. [M] Dense items weigh 2 to 4 times too little

- **What the sim does:** `items.py:469`. Items with no mass range get the bounding-box volume × 60 to 200 kg/m³.
- **Computed** (by the robot/arm audit pass; real values estimated from typical product listings):

| Item | Sim mass | Real (est.) |
|---|---|---|
| 16 oz mug | 93–309 g | 350–450 g |
| Ramekin | 40–132 g | 150–250 g |
| Scissors | 14–47 g | 60–90 g |
| Crayola 24 crayons | 15–51 g | ~130 g |
| Card game | 37–123 g | ~350 g |
| Tape roll | 20–65 g | ~100 g |

- **Fix:** sourced per-item masses in `CLUTTER_KG`, or material classes (ceramic, paper or card, solid PVC) with their
  own densities.

## 14. [M] Camera calibration is exact where it shouldn't be

- **Head camera intrinsics.** The sim warps with the same `front_camera_720.yaml` it publishes (`lens.py`,
  `sim_hook.set_head_camera`), so fx, cx, cy and D are exactly right. That file is DimOS's calibration of *a* Go2;
  this unit's will differ.
- **Mount error.** `head_camera_mount_error` is one fixed value in every run of every house, so tuning can quietly
  absorb it.
- **Wrist camera.** No mount error at all (`dimos_robot.py:30`, `floor_mosaic2.wrist_extrinsic`).
- **Fix**
  - `head_camera_intrinsics_error` (estimates: fx ±1%, cx/cy ±4 px at 720p, D[0] ±5%), rendered but not published.
  - Draw the mount error per "robot build" seed around the params value.
  - Add `wrist_camera_mount_error` (estimates: ±1°, ±3 mm), applied in `sim_hook.add`'s `post`.

## 15. [M] The motors never heat, the joints have no stiction, and the readings are noise-free

- **Torque limits:** only peak torque (`maker_arm.xml` ±10 N·m; `TAU` 14/17). The RS02 is rated 6 N·m
  (`maker-arm-sdk protocol.py:34`), yet 72% of floor-reach poses with 0.3 kg need more than 6 N·m at J2, held
  through looks. There is no thermal model for the Go2's legs in long crouches either.
- **Stiction:** `dof_frictionloss` is 0 on every arm joint.
- **Readings:** encoder and torque readings go to the adapter exact. This flatters weighing, the 3 N·m contact stop
  (`pick.py:280`) and settle detection.
- **Power and effort:** power, and the published effort, use `actuator_force`, which leaves out the gravcomp share
  (`hud.py:28`, `power_budget.py:16`).
- **Fix**
  - Params: `rated_torque_Nm` (vendor), `thermal_tau_s` (estimate), `frictionloss_Nm` 0.1 to 0.3 (estimate),
    torque noise and bias, and encoder quantisation.
  - A first-order I²R heat state with a derate.
  - Use `qfrc_actuator` for power and effort.

## 16. [M] Gaps in the robot's body and footing

- **The walker.** The robot walks with DimOS's Go1 Playground policy, the simulator it was trained in, on μ = 1.0, so
  its feet never slip. The real Air runs Unitree's sport controller (see finding 7).
- **The head.** The head's front 12 cm has no world collision (`body_shell` is `contype 0`, `go1_arm.py:213`). Where
  the L1 sits, the head can pass into furniture.
- **The battery.** `battery_model.py` is energy bookkeeping (80% usable): no voltage sag, fade or BMS cutoff.
- **Fix**
  - A contype-1 head box.
  - Per-surface foot friction (finding 7).
  - Battery `internal_resistance_ohm` and `fade_per_cycle` params, fitted from `battery_module.py`'s real stream.

## 17. [L-M] What the camera and lidar models still leave out

- **Camera**
  - No depth of field: the wrist camera's fixed-focus lens at 10 to 20 cm is rendered perfectly sharp.
  - No auto white balance shifting between nights.
  - No fixed-pattern noise or hot pixels.
  - The noise and the bitrate dips are seeded 0, so they are the same every run (`camera_imperfect.Degrade`,
    `default_rng(0)`).
  - A per-frame JPEG stands in for H.264, so there is no P-frame smear or keyframe pulsing.
- **Lidar**
  - No mixed-pixel ghost points at edges, no multipath, no extrinsic error.
  - The lidar is placed by the same instant's belief pose (`odom_drift.to_belief`), with no scan-time skew.
  - The WebRTC lag (finding 2) never moves.
- **Fix**
  - A defocus blur by depth for the ELP profile (`focus_m`, `f_number` from the Arducam lens).
  - Per-night white-balance gains ±5%.
  - Seed `Degrade` per run.
  - Mixed-pixel points at depth edges.
  - A slow random walk on `frame_delay` (for example ±50 ms over minutes).

---

## Where the sim is harsher than reality (kept, noted so they aren't "fixed" by mistake)

- **Lidar range** is capped at 3 m (`constants.py` `MAX_RANGE`); the L1 reaches 30 m. The three depth cameras miss
  the rear.
- **Resolution:** head frames are 640×360; the real camera is 1280×720.
- **The Go1 stand-in** is lighter (12.8 kg body) and has less knee torque (35.5 against about 45 N·m) than the Air.
- **The arm's torque cap** of 10 N·m is below the RobStride peaks.
- **Wall-clock timeouts** in `tidy_module` (blocked after 3 s, navigation timeouts) give the robot less sim time
  than they say.

## Recommended order

1. **Timing (findings 1 and 2).** Record sim/wall in every summary and flag under 0.9. Then hold frames and odometry
   until their arrival in sim time, and add a command delay. This alone may move detection-on-approach and stop
   distances the most, and it touches everything.
2. **The route from the robot's own map (finding 3),** with the import gate.
3. **Hard-spot toy placement, random landed poses and the missing item kinds (finding 4).** Re-tune against these
   before the next hold-out.
4. **Real jaws and no phantom feed-forward (findings 5 and 9).** Two one-line changes, but they change grasp results;
   re-run grasp_bench once.
5. **Per-night lighting and floor variation, and contaminated baselines (finding 6); per-material friction
   (finding 7).**
6. **Scoring tightened (finding 10) and the s80 hold-out relabelled (finding 8).**
