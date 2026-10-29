# Glossary: the standard term, then the picture

Principle [MJo]: docs, findings and video name a thing by the field's term, then explain it with the image. Where a
project nickname appears in older notes or file names, this is its translation.

## Pose graphs and SLAM

A SLAM system has two halves, and most of the hard problems are in the first:

- **Front end: building the graph.** From raw sensor data, decide which measurements become constraints and how much to
  trust each: keyframe selection, scan matching and image registration, place recognition and loop-closure detection,
  rejecting matches that only look alike (data association), and estimating each match's uncertainty. Picture: deciding
  where to hook each spring and how stiff to make it.
- **Back end: optimizing the graph.** Given the constraints, find the poses that best satisfy them all, robust to the
  front end's mistakes. Picture: letting the net settle.

A back end cannot fix a front end's wrong spring: it can only outvote it, if the rest of the net is stiff enough.

| Ours | Front end (graph construction) | Back end (graph optimization) |
|---|---|---|
| DimOS's pose graph (`house_frame.py`) | keyframes every 25 cm or 5 deg; lidar submaps; loop-closure search within 2 m, scored; each proposed loop verified against the robot's own scans before it is added ([fxk]) | GTSAM, in DimOS's PGO |
| the house's shape (`house_shape.py`) | every keyframe pair seeing the same walls matched (point-to-line ICP); each match's information; implausible matches dropped | weighted robust least squares (`robust_ls.py`) |
| the floor stitch (`stitch_floor.py`) | overlapping floor patches registered (`floor_register.snap`, coarse to fine); the lead test; the 8 cm bound; frame gate | weighted robust least squares |
| the night's frames (`confirm_stream.py`) | each frame registered against the floor picture (tracking); changed spots masked | none yet: odometry plus tracking |


| Term | What it is | The picture | Older project name |
|---|---|---|---|
| SLAM | simultaneous localization and mapping: working out where the robot is while building the map it locates against | | |
| pose | where the robot (or a camera) was and which way it faced, at one moment | a pin in the map | keyframe, view |
| pose graph | the poses as nodes, joined by what was measured between them | a net of pins and springs | |
| node | one pose in the graph (factor graphs: a variable) | a pin | |
| constraint, edge (factor) | one measurement relating poses: odometry between two in a row, a lidar scan match, a floor-picture match, a loop closure | a spring between two pins, its rest length what was measured | spring, image spring, lock |
| residual, error | how far the current solution disagrees with a constraint's measurement | how far a spring is stretched | |
| information (matrix) | how certain a measurement is, per direction: the inverse of its covariance | the spring's stiffness, which can differ by direction (a wall match stiff across the wall, slack along it) | |
| prior, anchor | a constraint on one pose alone, holding the frame in place | a spring to a fixed nail | |
| loop closure | a constraint between poses far apart in time: "I have been here before" | a long spring across the net that pulls drift back in | |
| false loop closure | a loop closure between places that only look alike | a spring hooked to the wrong pin, bending the net | |
| robust kernel (Huber, Cauchy) | a loss that stops growing as fast once a residual is implausibly large, so an outlier is outvoted | a spring that goes slack when overstretched | reweighting |
| front end | graph construction: turning sensor data into constraints, and choosing which to trust | hooking up the springs | the matching, the locks |
| back end | graph optimization: solving the constraints together | the net settling | the solve |
| data association | deciding which measurement belongs to which place or object; its failures are false matches | a spring hooked to the right pin, or not | |
| place recognition | recognising a previously visited place, to propose a loop closure | | |
| keyframe, keyframe selection | a pose kept as a node; chosen by a minimum motion (DimOS: 0.5 m or 10 deg by default, ours 0.25 m or 5 deg), by overlap with the last keyframe (the usual rule for cameras), or by a fixed frame rate (our floor views today: every 8th frame) | a pin placed each time the robot has moved enough | views |
| preintegration | the many odometry or IMU readings between two keyframes summed into one constraint, its uncertainty growing with the gap (Forster et al. 2015) | a longer gap makes a softer spring | |
| continuous-time trajectory | the path as a smooth curve (usually a B-spline) instead of discrete nodes, so every instant has a pose; Kalibr uses it to solve sensors' time offsets | a flexible rod instead of pins | |
| pose graph optimization | finding the poses that best satisfy every constraint at once (nonlinear least squares) | letting the net settle where the springs' total stretch energy is lowest | the solve, the stitch |
| odometry, drift | the robot's own motion estimate from legs and IMU; its error grows with distance | soft springs between pins in a row | |
| scan matching, ICP | aligning two lidar scans to measure how the poses moved between them | | lidar springs, the shape solve |
| image registration | measuring the shift and turn that lays one image (here, a floor patch) on another | | snap, lock |
| bundle adjustment | solving camera poses and the scene together from many images | | joint fit (close kin) |
| global alignment | the panorama form: every overlapping image placed at once (Szeliski 2006) | | the stitch |
| map layer | a map made from the solved poses (here, the floor picture) | | the floor picture, mosaic |

## Detection

| Term | What it is | Older project name |
|---|---|---|
| change detection, background subtraction | finding what differs between tonight and the known scene | the floor comparison, floor detection |
| open-vocabulary detection | a detector that finds objects named in words (OWLv2, YOLO-E) | the local detector |
| detection cascade | detection in stages, each rejecting some of the candidates the one before passed: a cheap, sensitive stage first, then pickier, costlier ones on fewer candidates (Viola and Jones 2001) | the detection stages: floor comparison proposes, close look or recognizer confirms, OWLv2 behind |
| funnel, per-stage success breakdown | the counts a night keeps at each step of the task (present, found, reachable, grasped, binned); "funnel" is the common word, "success breakdown" the more formal | the funnel |
| precision, recall | of what was flagged, the share real; of what was there, the share found | false positives, found |
| track-before-detect, evidence accumulation | summing faint evidence across frames before deciding | pooling, fuse before deciding |
| parallax | the shift of a thing with height against the floor between two viewpoints | |
| hold-out set | data never looked at until the method is frozen | hold-out houses |
| positive control | a planted, known effect a test must detect before its null result is trusted | planted error |
