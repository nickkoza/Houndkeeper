# Nightly mapping

**Type:** Capability spec · **Status:** Draft · **Source:** deliverable D1; the grant proposal's test plan (private) · **ID:** [slD]

## Purpose

The Go2 Air alone, with no arm, maps the ground floor every night and keeps one house map that gets better rather
than being rebuilt, while logging the measurements that show whether DimOS's mapping and navigation hold up in a real,
changing home. This is the first thing on the real robot and the evidence for the next piece of hardware.

## Acceptance criteria

1. [fis] A night starts by relocalising in the saved house map and updating only what changed; losing a night never
   erases what earlier nights learned.
2. [Epi] Every night logs all eight tests, each with its measurement: stale obstacle clearing (seconds until a removed
   object leaves the map), new obstacle latency (time to detect and replan), minimum reliably detected obstacle
   height, changed-room relocalisation (success, time, position error), kidnapped robot (recovers or not, how long),
   moving-object trails (whether a trail stays in the map), night-to-night delta (changed area, whether memory
   updated), and new small items (found, missed, false alarms, against that night's light level).
3. [Bj8] Every night is recorded in DimOS's recording format so it can be replayed against later DimOS versions.
4. [8XQ] The robot is on a network that cannot reach the internet, with firmware pinned; no update is taken unplanned.
5. [2L8] Pass thresholds for each of the eight are agreed with Dimensional before they are used to judge a night.
6. [QdH] **The map gets better, never worse.** Over consecutive nights of the same house, with furniture moved and
   things left on the floor changing between nights, the house map's completeness and correctness and the floor
   model's coverage and fidelity, each measured against the truth, never fall by more than 2 percentage points from
   one night to the next, and after the last night are at least what they were after the first.

## Decisions

1. [skp] **The floor stays a picture on a grid; Gaussian splatting is borrowed from, not adopted**. Each sensor does the job it is best at: the lidar the costmap and the house's
   geometry (it measures, and it sees free space); a point cloud for showing the house; the floor its own top-down
   picture, compared night to night on one fixed grid. Full splatting was weighed for the floor and set aside: GPU
   training every night where the picture takes about a minute of CPU, no common grid to compare two nights on,
   floaters where views are thin (false alarms), and lighting baked in; and no finer detail, both being limited by the
   camera's pixels on the floor and by pose accuracy. Its pieces that answer measured problems are taken one at a time,
   each kept only if the floor map error and the nights' agreement improve: (1) the joint fit, each view against the
   floor ([G9q] in sim-slice, tested first); (2) view-dependent appearance per floor cell, for tile shine read as
   change; (3) a thin height layer, for carpet relief and rug edges. Per-view exposure (the gain) and a robust combine
   (the trimmed mean) were in already. Revisit full splatting if these plateau with the floor picture still the limit
   on detection, if a real floor defeats a flat model, or once splats are made for the house twin anyway.
2. [JzW] **The floor is a sensor in the pose graph, not a stitch beside it**: part of pose-based SLAM, not bolted onto its
   side. Today three solves run in
   a row, each holding the one before fixed: DimOS's pose graph (odometry and lidar loop closures, [KkQ]), the house's
   shape (every lidar scan pair, [FHA]), then the floor stitch ([WFI]), whose views are tied to each other by odometry
   alone, so the lidar's rigid evidence plays no part where the floor is aligned and the floor's none where the lidar is
   weak (open floor; along a corridor). One graph instead: odometry, lidar scan matches, loop closures and floor-view
   matches as factors, each weighed by what it pins, solved together; the floor picture is then the map made from the
   graph's poses (a map layer, as a landmark map is). The same graph places tonight's frames during a tidy, where today
   odometry and a check against the floor picture do. A change on the floor (a toy, a moved chair) is an outlier to
   localization: the robust weighting and the change masks keep it from pulling poses. The existing matches are reused;
   only the solve is new; GTSAM is in already. A floor-view factor in DimOS's pose graph is a general camera-and-lidar
   feature (fork rule 6). Measured against the three-solve chain on the tuning houses' true paths, a planted-error control
   first ([G9q]'s 1% recovery was a weak control, not a null).
   - Measured 5 Oct 2026 (17 tuning houses, against the true paths): the joint graph 16.0 mm and 0.40 deg, the shape
     solve 15.8 mm, the chain 31.0 mm, DimOS's pose graph 48.9 mm. The back end is not the limit, the front end is:
     floor locks between views facing apart ~100 mm off, and false loop closures (sim-slice [fxk]). The controls
     passed. Not adopted as built.
3. [4t7] **The floor's appearance depends on the direction it is seen from, and the front end models it**: the key
   to the integrated pose graph is a correct front end, and directional floor awareness is pushed for it. Measured 5 Oct: floor-picture matches between views facing within 15 deg are ~12 mm
   off at the median, between views facing apart ~100 mm ([JzW]'s findings); the same view-dependent shine, shadow and
   pile direction drives most of the floor comparison's false flags ([H9w]: 105 of 153). One model serves both: each
   floor cell's appearance kept per viewing direction (and its spread per direction, the view noise), so a match or a
   comparison is made against the floor as seen from about the same direction, and a cross-direction match carries
   the uncertainty it has earned. Judged twice: the detection cascade's first stage on fresh nights (found and false
   flags, by curve, [HGT]) and the joint graph's pose error against the chain ([JzW]); a planted-error control first.
   - Measured 6 Oct 2026: direction explains about a quarter to two fifths of a cell's spread (most on glossy wood).
     In the graph 15.8 mm against 16.0; in detection, fresh nights 33/40 at 8.8 false flags a night against the view
     noise x6's 34/40 at 8.2. Ties, so no default changed; kept as a measurement (`dirfloor.py`) and an option
     (`joint_graph.FLOOR_FRONT = "dir"`).
4. [x9X] **Tonight's frames placed by the same graph, live** (the integrated graph's next test, chosen
   over lidar-realism and lidar-weak tests). During a tidy, each
   frame is placed as it arrives by the house's graph: the baseline nights' poses held as priors, tonight's keyframes
   added incrementally (iSAM2) with odometry, lidar scan matches against the house's keyframes and floor matches
   against the floor picture, changed spots masked out of the floor matches. Today odometry and a check against the
   floor picture place them. Judged on the sim's true path (tonight's pose error, drift over the walk) and on the
   detection cascade's first stage on fresh nights (curves, [HGT]); within the robot's compute and live, not after
   the night.
   - Measured 11 Oct 2026: graph placement 22-24 mm off against today's 80-100 mm, and it never lost the robot. In
     detection 45/48 on tuning nights and 32/40 on fresh against 46/48 and 34/40, at about half the false flags; under
     recall first ([Btx]) not the default. The option is kept; 3 toys were lost to the slop check, the next thing to
     measure.

## Failure behavior

- If the robot cannot relocalise, it maps as new but does not overwrite the saved map until it has been confirmed.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [fis] (relocalise and update, never erase) | real nights; sim house map with smart clearing | Partial: sim only |
| [Epi] (eight tests logged every night) | nightly log checked for all eight | Partial: 3 of 8 have sim versions |
| [Bj8] (recorded for replay) | recording present and replays | Partial: sim nights recorded |
| [8XQ] (isolated network, pinned firmware) | bring-up checklist, network audit | Not yet written |
| [2L8] (thresholds agreed) | agreement recorded in docs | Not yet written |
| [QdH] (the map gets better, never worse) | sim nights scored against the sim house's truth: `sim/house/map_model.py` (completeness, correctness) and `sim/house/floor_model.py` (coverage, fidelity) night by night | Not yet written: the scoring exists; the night-over-night check does not. Real nights have no truth, so this is sim only |
