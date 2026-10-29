# Build or depend

**Type:** Register · **Status:** Draft; entries decided under [eM8], sign-off needed only for new dependencies or a change of direction · **Source:** principle [eM8] · **ID:** [eK3]

## Purpose

One place where each tool the project might take on, or might write itself, is weighed before it happens: the need each system
is meant to address, writing it ourselves against the dependency, and what DimOS already offers. Standard robotics
tools are where the search starts (private related-work notes, 4 Oct 2026), not a shopping list.

## Acceptance criteria

1. [Srm] **Every entry answers four questions before anything is adopted or built:**
   - **The need:** the job it does here, in the project's terms, and what it would change (which measure, which
     criterion).
   - **What DimOS offers** for that job today (module, command, or nothing), checked in the fork's code.
   - **Writing it ourselves:** what it would take, what we have already, and what it keeps costing (bugs, upkeep).
   - **The dependency's cost:** size, platforms (the laptop, the Pi, the robot's own ARM computer later), maintenance
     and activity, licence; and whether it is already installed.

   Then a **verdict** (adopt / build / borrow the method only / not now) with the measure that would decide it. Sign-off
   is needed only for what [eM8] leaves open:
   taking on a **new dependency** (anything not already installed or in DimOS), or a **change of direction**. Using
   what is already installed, borrowing a method, building something small, and "not now" are settled by the rule
   and by measurement, and need no sign-off. Nothing is adopted on the verdict alone: an A/B on our own measures
   comes first wherever one is possible.

## Decisions

1. [ay4] **Clock sync between the laptop and the arm host: chrony (NTP), from the router** (chrony as the current
   standard). The two talk over Wi-Fi, which has no hardware timestamping, so PTP
   would run in software and do no better than NTP, which chrony handles well on jittery links; a millisecond is far
   inside the 30-150 ms offsets being calibrated, and Kalibr ([fxE]) measures what is left. Matches dog-first-boot
   [x20] (one time source on the segment). PTP comes back for onboard compute if its sensors are wired and its network
   card timestamps in hardware (an onboard-compute requirement).

## Register (for review)

Each entry: need · DimOS offers · build ourselves · dependency cost · verdict for review. Filled 4 Oct 2026.

Ordered by likely lift for this project, highest first. Checked on 4 Oct 2026: the fork at a7492431f (22 Sep 2026) by
`git grep`; the DimOS venv by `importlib.metadata`; repository activity (last push) and licences from the GitHub API.
Where something could not be checked it says so. "Installed" means in the DimOS venv today, on the x86_64 laptop.

### 1. Factor-graph solvers (GTSAM, Ceres, g2o, scipy least_squares)

- **Need (today):** the robust solves that place keyframes and views: `house_shape.py` (lidar shape),
  `stitch_floor.py` (the floor-picture constraints between views), `floor_refine.py` ([G9q], [ObC]). They move the
  floor map's error ([nv5]) and its precision ([FHA]).
- **DimOS offers:** GTSAM inside its pose graph (`dimos/mapping/loop_closure/pgo.py`, `pgo_auto.py`); no general solver API.
- **Ours:** `sim/house/robust_ls.py` (79 lines): rows whitened by their information, a true Huber or soft_l1 loss on
  scipy's `least_squares` with the exact sparse Jacobian, beside the old iteratively reweighted `lsqr`. Linear in small
  SE(2) corrections. It does not give marginal covariances (which views are poorly pinned).
- **Cost:** GTSAM installed (gtsam-extended 4.3a1, about 97 MB; simplified BSD; active 2 Oct 2026; DimOS's pyproject
  marks it for Python 3.11+, aarch64 wheels not checked). scipy 1.17 installed. Ceres (BSD-3, active today) and g2o
  (active 28 Sep 2026; licence file not readable by the API) are C++ with no Python package installed: new builds.
- **Verdict for review:** left to the A/B now running (`robust_ls.py` on scipy against the old `lsqr` path). GTSAM is
  the next arm of it if the A/B leaves a gap; Ceres and g2o not now. Deciding measure: [nv5]'s floor map error and the
  two nights' agreement on the tuning houses, plus solve time.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 2. Kalibr (camera-IMU time offset and extrinsics)

- **Need (real robot, first boot onward):** the time offset and mount of every sensor pairing, with uncertainties, in
  one record ([fxE]: [m8Z]). It settles the open lag disagreement ([55G]: -0.030 s by turning against about +0.02 s by
  the floor map error), which bounds every floor picture.
- **DimOS offers:** `dimos cameracalibrate` (chessboard intrinsics, ROS CameraInfo YAML), the AprilTag/ArUco PDF
  generator (`dimos/cli/apriltag.py`, `apriltag3d.py`); no camera-IMU or time-offset tool.
- **Ours:** the turn-correlation lag (`src/houndkeeper/calibrate.py`, `sim/house/calibrate_lag.py`) and the targetless
  mount (`calibrate_mount.py`, `calibrate_robot_mount.py`), kept as the nightly check ([9tQ]); a joint continuous-time
  solve of our own is the stated fallback.
- **Cost:** offline only, from its Docker image ([VNY]); Docker is installed, the image is not pulled (size not
  checked). BSD-3; last push March 2024 (quiet, ROS1-era build). Needs a `memory.db` to bag converter and IMU at 200 Hz
  or more ([1sW], unknown until day one).
- **Verdict for review:** adopt as an offline tool, as [fxE] specifies. Deciding measure: [Hw0], the sim recovering its
  known delays within 5 ms and mounts within 0.2 deg.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 3. Image registration (OpenCV ECC, features with RANSAC) against `floor_register`

- **Need (today):** subpixel registrations (locks) for the floor-picture constraints and the joint fit; targets are a
  few mm on a 1 cm grid ([FHA], [WFI]), and a peak at 1 cm is cells broad ([ObC]).
- **DimOS offers:** nothing for image-to-image registration found by grep.
- **Ours:** `src/houndkeeper/floor_register.py` (172 lines): masked edge NCC by FFT, coarse to fine, a few turns;
  `surface_belief` reads confidence from the whole correlation surface.
- **Cost:** none new: `cv2.findTransformECC` is in opencv-contrib 4.13 (installed; Apache-2.0; active). Features with
  RANSAC are also in OpenCV but suit the repetitive tile floor poorly (a tile repeat is a false match).
- **Verdict for review:** borrow ECC as a refinement after our NCC peak; features not now. A test is queued.
  Deciding measure: registration error against the sim's truth, then [nv5] on the tuning houses.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 4. Open-vocabulary detection (OWLv2 through DimOS): the missing baseline

- **Need (today, as an experiment):** the review found the published tidying robots detect objects in the current
  view rather than compare the floor to earlier nights; our approach has never been measured against that. It would
  move the detection share ([WFI]'s 93%) or show the floor comparison is the better bet.
- **DimOS offers:** `dimos/perception/detection/detectors/owlv2.py` (also OmDet, YOLOE, EdgeTAM wrapped).
- **Ours:** `sim/house/detect_changes2.py` and `confirm_stream.py` ([d8d], [QDc]); the detection bench `bench_detect.py`.
- **Cost:** none new: transformers 4.53 and torch 2.7 installed; model weights download on first use (cache not
  checked); GPU time on the shared GPU. Not an onboard question yet.
- **Verdict for review:** adopt as an experiment through DimOS's wrapper: the same bench nights, detector alone, change
  detector alone, then both (change says where, detector says what). Deciding measure: toys confirmed and candidates
  near no toy per night on the bench.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 5. A time-indexed transform buffer (a pose at a sample's own time)

- **Need (today, more on the real robot):** every frame placed at its own stamp less its lag ([gJq], [Feg]). Five of
  our files interpolated poses each in their own way (`Slerp` in `floor_mosaic2.py`, `confirm_stream.py` and three
  analysis tools since retired), with their own gap limits (`POSE_GAP_S`).
- **DimOS offers:** `dimos/protocol/tf/tf.py` (`TBuffer`, `MultiTBuffer`, `TF`): a 10 s buffer per frame pair, chained
  lookups, but `get` returns the nearest sample within a tolerance, with no interpolation.
- **Ours:** the copies above; `dimos_robot._samples` reads the arm's samples on its own clock rather than the wall's.
- **Cost:** ROS tf2 (BSD-3, active) interpolates properly but brings ROS 2 (rclpy is not installed); not worth it.
- **Verdict for review:** build one small shared pose-at-time helper of our own (tf2's semantics: interpolate, refuse
  past a gap, say which), used by all five; or propose interpolation upstream to DimOS's `TBuffer`. Deciding measure:
  identical floor pictures byte for byte after the swap, and a test that refuses an extrapolated pose.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 6. Hand-eye for the wrist camera (OpenCV calibrateHandEye, easy_handeye, MoveIt calibration)

- **Need (real arm):** the wrist camera's mount on the arm's end link ([m8Z]); today the sim's wrist pose comes from
  the model ("its pose from the arm's joint angles", `sim/arm/pick.py`), so a real mount error goes straight into the
  grasp point.
- **DimOS offers:** `dimos cameracalibrate` for the intrinsics and the AprilTag tools for the target; no hand-eye.
- **Ours:** nothing beyond the model's nominal mount.
- **Cost:** `cv2.calibrateHandEye` installed (five methods to cross-check). easy_handeye (LGPL-3.0, active Nov 2025) and
  moveit_calibration (BSD-3, active Dec 2025) both need ROS and MoveIt for what OpenCV does in one call.
- **Verdict for review:** borrow OpenCV's call; not the ROS packages. Kalibr's camera-to-pose mode ([fxE]) is the
  cross-check. Deciding measure: the recovered mount against the sim's truth, then the grasp point's error on the
  bench.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 7. Trajectory evaluation (evo: ATE and RPE)

- **Need (today):** how well the robot knows where it is ([KkQ]), graded in the sim against its true path; on the real
  robot, against an AprilTag or Kalibr session.
- **DimOS offers:** `dimos/mapping/loop_closure/eval.py` scores PGO by marker spread and time, not trajectory error.
- **Ours:** `sim/house/grade_localization.py`: one rigid 2D fit removed, then the error that changes over the night,
  which is ATE in all but name.
- **Cost:** not installed; GPL-3.0; active Sept 2026; pure Python, so aarch64 is no issue. Its value over ours is RPE
  per distance, plots and a standard name.
- **Verdict for review:** borrow the method: add RPE (per metre walked) to `grade_localization.py` and call the numbers
  ATE and RPE so they compare with published work; evo optionally as an offline cross-check, never imported. Deciding
  measure: our ATE equal to evo's on one recording.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 8. IMU noise by Allan variance

- **Need (real robot, before Kalibr):** Kalibr wants the IMU's noise density and random walk; without them its
  uncertainties are guesses ([1sW]).
- **DimOS offers:** the Go2's IMU arrives over its DDS codec (`dimos/robot/unitree/go2/dds/codec.py`); no noise tool.
- **Ours:** nothing; we do not use the IMU in the pipeline today.
- **Cost:** allantools (LGPL-3.0, active May 2026, pure Python) or allan_variance_ros (BSD-3, quiet since Jan 2025,
  ROS). The overlapping Allan deviation is about 20 lines of numpy.
- **Verdict for review:** build (the method is short and standard), checked once against allantools offline.
  Deciding measure: the same noise figures as allantools on a few hours of a still robot's IMU, and Kalibr's
  reprojection error with them against with defaults.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 9. Lidar-camera and lidar-IMU calibration (Koide's direct_visual_lidar_calibration, LI-Init)

- **Need (real robot):** the lidar to the body and to the head camera ([m8Z]'s last pairing; Kalibr has no lidar
  model). Today the camera's position offset is tuned from where the camera and the lidar each see walls meet the
  floor.
- **DimOS offers:** static transforms for a Go2 with a Mid-360 (`go2_mid360_static_transforms.py`); no calibration tool.
- **Ours:** the wall-meets-floor offset in the mount solve.
- **Cost:** Koide's toolbox: C++/ROS, active July 2026, targetless, single shot; its licence was not readable by the API
  (not checked). LI-Init: GPL-2.0, active April 2026, ROS. Both would run once in a container, not as dependencies.
- **Verdict for review:** not now; at first boot run Koide's toolbox once as an independent check of the mount. Deciding
  measure: its lidar-camera transform against ours, within [Hw0]'s tolerances.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 10. Point-cloud registration (Open3D ICP and NDT) against `house_shape`'s ICP

- **Need (today):** pairwise lidar matches for the shape solve ([KkQ]), each with its information so a wall pins across
  and not along.
- **DimOS offers:** Open3D's tensor ICP (point to plane) inside `pgo.py` and its relocaliser
  (`mapping/relocalization/lidar/relocalize.py`).
- **Ours:** point-to-line ICP in 2D with J'J information and Huber in `sim/house/house_shape.py` (219 lines, with its pose graph optimization).
- **Cost:** Open3D 0.19 installed (about 1.1 GB; MIT from memory, not checked today; aarch64 through
  `open3d-unofficial-arm`). It has no NDT; its information matrix is per pair, in 3D.
- **Verdict for review:** not now: ours is 2D on purpose and the information weighting is the part that matters. Worth an
  A/B only if the shape solve's matches are shown to be the floor map's limit. Deciding measure: [nv5] with Open3D's
  matcher swapped in.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 11. Lidar odometry and SLAM baselines (KISS-ICP, FAST-LIO2, slam_toolbox) against DimOS's mapping

- **Need (today, as a baseline):** odometry's drift bends the map (7.7 deg by the end of a night, house 72); a
  stronger front end might leave the shape solve less to do.
- **DimOS offers:** the Go2's own odometry, PGO with loop closure over Open3D ICP and GTSAM, and dimSLAM (Rust:
  cuVSLAM visual odometry with an IMU in an error-state Kalman filter, `mapping/dim_slam`).
- **Ours:** `house_frame.py` (DimOS's PGO tuned indoors) and `house_shape.py`, graded by `grade_localization.py`.
- **Cost:** KISS-ICP: MIT, active June 2026, pip-installable (aarch64 wheels not checked), not installed. FAST-LIO2:
  GPL-2.0, quiet since Jan 2025, needs a high-rate IMU and ROS. slam_toolbox: LGPL-2.1, active, ROS 2 only.
- **Verdict for review:** not now; if anything, KISS-ICP as a one-off offline baseline on recorded nights. Deciding
  measure: ATE and RPE (entry 7) against the true path, before and after `house_shape.py`.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 12. Recording and visualisation (MCAP, Foxglove, Rerun)

- **Need (today and at first boot):** recordings that every tool reads, a bag-like export for Kalibr ([VNY]), and a
  viewer for debugging nights.
- **DimOS offers:** its own `memory.db` (SQLite) store, a read-only MCAP store (`dimos/memory/store/mcap.py`, with the
  Go2's DDS recordings), Rerun as a core dependency and a bridge, Foxglove message types (`msgs/foxglove_msgs`), and a
  ROS pubsub bridge (`protocol/pubsub/impl/rospubsub.py`, if rclpy is present).
- **Ours:** readers of `memory.db`, contact sheets.
- **Cost:** rerun-sdk 0.32 installed (about 390 MB; Apache-2.0; active). The `mcap` package is a DimOS extra, not
  installed in our venv (MIT, active). Foxglove's SDK is not installed (MIT); the Foxglove app is not needed.
- **Verdict for review:** use DimOS's own; no Foxglove. Install DimOS's mcap extra only if the Kalibr converter writes
  MCAP rather than a ROS1 bag. Deciding measure: the converter's round trip ([VNY]'s test).
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 13. Coverage path planning (Fields2Cover, Nav2 coverage) against `sweep_route.py`

- **Need (today):** rows that see the whole reachable floor at close range, on the robot's own map ([HuM]).
- **DimOS offers:** frontier exploration and point-to-point planners (`navigation/global_planner/mls_planner`,
  replanning A*); no coverage planner found.
- **Ours:** `sim/house/sweep_route.py` (231 lines, boustrophedon on its lidar map), measured by `sweep_compare.py`
  (coverage within 0.6 m and 1.0 m, length, reach).
- **Cost:** Fields2Cover: BSD-3, active Sept 2026, C++ with Python bindings, built for fields (headlands, swaths), not
  rooms with furniture; not installed. Nav2's coverage server builds on it and needs ROS 2.
- **Verdict for review:** not now; ours is small and on our own map. Borrow the method if rows meet trouble around
  furniture (cell decomposition before rows). Deciding measure: `sweep_compare.py`'s coverage and length.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 14. Sensor fusion EKF and UKF (robot_localization)

- **Need (later):** fusing odometry, IMU and lidar poses in real time. The night pipeline places frames offline by the
  lidar ([KkQ]); nothing live needs a fused pose yet.
- **DimOS offers:** dimSLAM's error-state Kalman filter (above); the Go2 reports its own fused odometry.
- **Ours:** none; offline solves instead.
- **Cost:** robot_localization: BSD-3, active July 2026, ROS 2 only (rclpy not installed).
- **Verdict for review:** not now. Revisit if a live pose drives the arm or the look-closer on the real robot. Deciding
  measure: live pose error against the night's solved poses.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 15. Manipulation planning (MoveIt, OMPL) against DimOS's manipulation stack

- **Need (later):** collision-free arm motion; `pick.py` today moves through known-safe waypoints and its own IK, and
  the arm model has no collisions beyond the head shell (`sim/arm/go1_arm.py`).
- **DimOS offers:** a full planning stack (`dimos/manipulation/planning`): RoboPlan and RRT planners, Pinocchio, Pink and
  Drake IK, a world model; roboplan 0.6, pin 4.1 and drake 1.49 are installed.
- **Ours:** IK and waypoints in `sim/arm` and `dimos_robot.py`.
- **Cost:** MoveIt 2 (BSD-3, active) is ROS 2 and heavy; OMPL (active; BSD from memory) is not installed.
- **Verdict for review:** not now; when collisions matter, try DimOS's planners first. Deciding measure: picks without
  a collision on the grasp bench.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 16. Splat rendering (Open3D 0.20) for the splat twin

- **Need (later):** viewing a splat of the real house beside the twin's meshes, after first boot (backlog; [lQ1], [skp]).
- **DimOS offers:** Open3D 0.19 (meshes, point clouds); no splats.
- **Ours:** nothing.
- **Cost:** a version bump of a dependency DimOS already has; needs Vulkan on Linux; the aarch64 build DimOS uses
  (`open3d-unofficial-arm`) not checked for 0.20. Rendering only; training would be gsplat (a CUDA stack).
- **Verdict for review:** not now; when the twin is trained. Deciding measure: whether the splat view finds a
  sim-to-real difference the mesh view misses ([V3o]).
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 17. Volumetric maps (nvblox, voxblox, OctoMap)

- **Need (not today):** the pipeline needs a top-down floor picture and DimOS's 2 cm height map, plus `tall_layer.py`;
  nothing asks for full 3D occupancy.
- **DimOS offers:** a voxel map with carving on Open3D (`mapping/voxels/impl/o3d.py`).
- **Ours:** `tall_layer.py` and the height map.
- **Cost:** nvblox: Apache-2.0, active, CUDA-only. voxblox: BSD-3, quiet since 2024, ROS. OctoMap: active (licence not
  read today). None installed.
- **Verdict for review:** not now; DimOS's voxels first if 3D is ever needed. Deciding measure: none until a criterion
  asks for 3D.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 18. PTP for onboard compute (a future hardware requirement)

- **Need (later):** a shared hardware clock if onboard compute wires its sensors. Decided against for the Wi-Fi link
  ([ay4]: chrony).
- **DimOS offers:** nothing found by grep.
- **Ours:** chrony per [ay4] and [x20].
- **Cost:** linuxptp (system package; not installed here) and a network card that timestamps in hardware.
- **Verdict for review:** not now; carry as a requirement on onboard compute's hardware choice (hardware timestamps,
  cameras with start-of-frame stamps, [fxE]'s open question 1). Deciding measure: Kalibr's measured offset jitter
  with and without it.
- **Status:** decided under [eM8] (4 Oct 2026); acted on by measurement.

### 19. An IMU of our own (hardware), for calibration and an odometry check

- **Need:** Kalibr's camera–IMU calibration ([fxE]) wants raw gyro and accelerometer at about 200 Hz or more. Over
  WebRTC the Go2's low state carries its IMU as roll, pitch and yaw only in DimOS's type (`dimos/robot/unitree/type/
  lowstate.py`), rate unknown; the full low-level SDK with raw IMU is, as far as known, an EDU-model feature. An IMU of
  our own would also check the Go2's odometry independently, and later be the onboard computer's.
- **DimOS offers:** an `Imu` message type and IMU streams for other robots (G1); nothing that adds a sensor to a Go2.
- **Build ourselves:** a small IMU board (ICM-42688 or BMI088 class) on the arm host Pi's SPI, beside the eyes'
  LEDs, a driver and a recorder of a few hundred lines, stamped on the Pi's chrony-kept clock ([ay4]); its own mount
  calibrated by Kalibr.
- **Cost:** a few tens of dollars, mounting and wiring on the robot, one more thing on the arm host to keep working.
- **Verdict for review:** not now. First, on day one, record what the Go2's low state really carries and at what rate
  ([1sW]); if raw gyro at a usable rate is there, nothing is added. If not, calibrate rotation-only (the lag and the
  mount's rotation; its position by tape) before adding hardware. Add our own only if calibration or odometry checks
  need it, or choose it with the onboard computer. Decided by: the day-one low-state recording.
- Status: decided under [eM8] (4 Oct 2026); acted on by measurement.

### 20. The time-indexed transform buffer: decided (entry 5)

- **Need:** as entry 5: a pose at a sample's own time ([Feg]), live and over recordings, refused past a gap.
- **DimOS offers:** the buffer entry 5 names (`TBuffer`, `MultiTBuffer`, `TF`, and `StreamTF` over a recording).
- **Ours:** retired: the five offline copies now read `houndkeeper.rec_poses` over DimOS's `StreamTF`, and the live
  tidy's head-camera pose asks DimOS's buffer to interpolate ([x5h]). `dimos_robot._samples` is not a pose lookup
  (it waits for the arm's next joint readings) and stays.
- **Cost:** a fork branch, `nick/feat/tf-interpolation`, general to any DimOS user (fork rule 6): interpolated
  `get` with a gap limit, a clock for "now", odometry as a recording lookup's moving edge; nothing upstream until
  proven on the robot.
- **Verdict:** DimOS has it; extended in the fork; ours retired (4 Oct 2026). Measured as entry 5 asked: [x5h].

## Tests

| Criterion | Test | Status |
|---|---|---|
| [Srm] (four questions before adoption) | review: every register entry answers all four and names its deciding measure; no dependency added to pyproject.toml, the fork or the Pi's image without an entry decided adopt | Not yet written |
