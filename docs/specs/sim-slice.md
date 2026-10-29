# Full slice

**Type:** Capability spec · **Status:** Draft · **Source:** D0 [jX2]; system spec · **ID:** [YWo]

## Purpose

The whole night, in sim, on DimOS: sweep, find what is new, plan the fewest stops, and at each stop look closer, name,
pick and drop into the bin, then walk on. It is the rehearsal for the demo video and the first test of the real loop.

## Acceptance criteria

1. [hJ5] DimOS maps, localises and walks the base; the arm is driven through DimOS's arm interface; the wrist camera is
   a DimOS camera stream.
2. [S7o] Each find gets its own stop, approached head-on so the find ends up in the front reach zone; the robot never
   walks over or parks on a find or an obstacle. (Grouping several finds into one stop is backlogged.)
3. [JKS] At each stop, every find in reach is looked at, found and named in one step by the open-vocabulary detector
   (OWLv2, with EdgeTAM cutting its outline), and picked, dismissed as nothing, or sent to review.
4. [bOu] The run reports every find's outcome, the stops made, the distance walked, the energy used per pack and the
   time taken.
5. [5AP] A viewer shows the state machine, power and battery, and the head and wrist cameras live, as the frames the robot's perception received, so what it is seeing can be confirmed.
6. [bXX] It can be recorded to video with labels saying it is a simulation and what is staged.
7. [d8d] **Finds are checked during the sweep, spending more only when less wasn't enough.** Each candidate is first
   judged from the views the head camera gets while walking toward it, against last night's views of the same spot
   from the same direction. If that leaves it unsure, the robot sidesteps or sways to see it from the side (seconds,
   standing, only where the costmap is clear). Only a confirmed find gets a stop: the robot walks on to park it
   head-on, looks closer with the arm and picks it, then the sweep resumes where it stopped ([Xsf]). There is no
   separate planning of stops after the sweep.
8. [XfI] **The same object is never handled twice in a night.** When look-closer finds an object within 15 cm of one
   already handled that night, the stop is recorded as the same item and ends there, whatever flag led to it (the
   detector can raise two flags for one object, e.g. a shoe and its edge seen from another direction, 25 Sep 2026).
9. [tmq] **The robot's feet never cross a spot it knows about** (on 25 Sep 2026 it kicked the lion on its way to
   it). Every spot worth a stop ([6Pn]'s gate), and every find left on the floor for review, is kept out of
   the robot's walking paths until it is binned, or the close look finds nothing there. Weak spots are not: keeping
   out every spot with any evidence walled the robot in and spent 39 of a night's 43 stops settling them (27 Sep
   2026; keeping them out was a hypothesis, dropped when it didn't help). When the
   kept-out spots leave no way on (a waypoint gives up), the robot settles the nearest unsettled ones with the close
   look and tries again: the kept-out spots block it in artificially, and the robot can unblock itself; a waypoint it
still can't reach is reported as not reached, never counted as walked.
10. [FHA] **The robot keeps a precise map of its floor, the top of the funnel**, held to high precision before the
   steps after it, above all at high-contrast edges. The floor picture every night is compared against is built so
   that two maps of the same unchanged
   floor agree: fixed transitions (carpet to tile or wood) land in the same place to within a few millimetres, and
   stay there night after night, despite the slop in the cameras' and the lidar's positions as the robot jostles (the
   floor doesn't move, so many views of it can pin it down). The rug stays put between nights in the sim ([yyJ], since
   1 Oct 2026); its edges are mapped as precisely, and a moved rug is the floor-change test's, not a find.
11. [82p] **A find is placed by its middle, not its nearest part** (a standing look aims the pickup
   better, and so picks a better park). The standing look sees a find whole; the position it
   hands on, where the robot parks and the close look aims, is the middle of the item as it lies on the floor, with
   its length and long axis, not the end nearest the robot.

12. [nv5] **The floor map's error against the actual floor is measured and kept, above detection in the funnel**
   (detection can't be better than the floor map, so its difference from the actual floor image, as a percentage,
   is tracked over time). For each tuning house, the share of its open floor
   (seen, away from mapped structures, the rug and moved chairs apart) where the robot's floor map differs from the
   sim's true floor by the detector's own test (its colour threshold and slack): the floor map error. The map is laid
   on the true floor by the one rigid move that best lays its path on the true path, so a bent map counts against
   it. The true floor is the floor's own colour, unlit, so even a perfect map scores above zero: the same measure of
   a map built from the sim's true path is kept beside it as that night's floor. Every measurement is appended to
   `data/eval/floor_map_error.jsonl` with its date and how the map was built. A scorer only: the truth reaches nothing
   the robot runs.

13. [63R] **Every subsystem's health, in the order the robot uses them** (2 Oct 2026: the floor snap was in
   the loop but locked on 0-3 of ~350 frames a night, and nothing showed it; each subsystem's performance is tracked,
   in a report generated automatically). A generated pipeline: for each subsystem, from odometry
   to the bin, what it does, the measure of its success, its value on the latest bench and runs, its target where a
   requirement sets one, and whether it is working, idle or failing; per house and overall. Rebuilt from the runs' own
   records (`sim/house/subsystem_health.py`), so a subsystem that stops doing its job shows, rather than hiding inside
   the result at the end.

14. [evl] **A floor picture can average each spot's views instead of taking one** (3 Oct 2026, tile floor error
   investigation: on a dark patterned tile, taking one whole view per spot by its brightness rank left colour speckle
   from frame to frame that the detector's colour test reads as difference). As a choice beside the default, each spot
   of the floor picture can be the mean of its closest views once the brightest and the darkest share of them, set by
   the caller, are left out: noise in any single view averages away, and a bright reflection seen in only a few views
   (a glint) stays out as the darker-view default keeps it out. Choosing it changes no picture built without it.

15. [SdA] **A step that fails stops the night; nothing from an earlier run stands in for it.** Every file the night
   pipeline derives (a floor picture, tall layer, views, the house's frame and shape, the camera mount, a route, a
   house map to seed from, a bench replay's result) is this run's own, made from tonight's inputs, or the run stops
   and names the step that failed; a step whose failure has a designed way on (a house map too small or not loaded to
   seed from: the full mapping pass) takes it and says so in the run's log. A missing, empty or crashed build is never read as an empty floor, a quiet night or
   last run's file, and a replay bench that cannot replay a night stops instead of scoring it as no finds.

16. [YuI] **A floor picture seen from too little of the night is not a baseline.** A floor picture placed from too few
   frames, or over too little of the floor, is refused, and the refusal says how many frames and how much floor it
   had.

17. [gJq] **The camera's frame lag and the head camera's mount are the robot's calibration.** Each is stored once for
   the robot, written only by an explicit calibration step over tuning houses (never from a hold-out house), and read
   by every night; a night's own estimate is a check that warns when it disagrees, and replaces the robot's value only
   when asked to. Each night records which calibration it used.
18. [G9q] **Each view is registered against the floor, not only against other views** (the floor-picture constraints,
    pictured as springs between views, tie views only to each other; a feature of the floor can exist at one place
    only, and this finds where). An experiment beside the pairwise floor constraints, not yet
    a replacement: after the stitch, each view is registered against the floor picture built without it (leave one
    out), the picture rebuilt from the corrected poses, and the two repeated a few passes. The lidar's constraints
    stay in each solve, so the house's shape is pinned and the image fit cannot bend it; each view moves only as far
    as odometry and lidar allow (no jump of a whole tile on a repeating floor); a pass is kept only if the two nights'
    pictures agree better, without truth, and stops when they stop improving. Measured against the pairwise floor
    constraints alone by the floor map error ([nv5]) and the nights' disagreement on the tuning houses; never a
    hold-out.

19. [Tu2] **A house frame that places a night far from its own odometry is refused** (for [SdA]). Each baseline night
    starts on the dock, where its odometry and the house's frame agree, and odometry drifts only so far over a night.
    A stitch whose frame turns or moves a night further from its odometry than any working house has drifted, at the
    dock or anywhere over the night, stops the night, naming the night, where and by how much; no picture, tall
    layer, view set or lidar map is built in it, and the live tidy never compares tonight against it.
20. [Q9k] **Tonight is never compared with a floor painted under another calibration** (for [gJq]). Every floor
    picture and view set carries the calibration it was painted with; one painted with another mount, lag or floor
    relief than tonight's frames is refused, naming what differs, and its house's floor map is made again before it is
    used.
21. [dmT] **A loop closure enters the house's frame only when the robot's other sensors agree with it** (for [Tu2]). A
    revisit the house's pose graph proposes is checked against what the robot itself sensed around both places before
    it can bend the frame, and one they do not confirm is left out. A house's map made before this check is refused,
    naming why, and made again before tonight is compared with it.
## Non-functional requirements

Measured on tuning nights (sim-house-nights [I0Q]), on DimOS's sim with Filament cameras, by `score_tidy.py` and
`vslice_summary.py`; the top of the funnel is kept wide, since several later steps identify false positives:
- The walk-through raises a candidate (confirmed or unsure, so it is looked at again) within 30 cm of at least 90% of
  the toys. More stops for nothing are accepted to get there; look again, the sidestep and the close look remove them.

The top of the funnel, held so it can't slip unnoticed: each requirement is what was measured on 27 Sep, in
brackets, less a margin. `sim/house/check_funnel_top.py` measures them all and fails on any miss.
- The floor map ([FHA]), on the night pipeline's own floor pictures of house 70's two empty baseline nights (the lamp
  on for one, off for the other; head camera), scored by `bench_mapping.py --pictures`: the carpet-to-hard-floor edges
  within 5 mm of the drawn edge at the median (3.3-3.7) and 16 mm at the 95th percentile (11.8-13.2); the two nights'
  edges agreeing within 8 mm at the median (4.7-6.1); the rug's two nights correlating at least 0.80 (0.88). The aim
  stays a few mm: 3 mm at the 95th percentile; the margin is a floor, not the goal. These hold in a sim
  no kinder than the world ([x3f]): every imperfection of waymark [nAH] on (the arm's sag and flex, the cameras'
  blur, noise, fisheye and late stamps, the lidar's budget, noise and dropouts, odometry drift, the camera mount's
  error, the floor's relief, moving chairs). Measured 27 Sep 13:40 with them on, before any compensation: edges
  20-22 mm from the drawn edge (median), 46-48 mm (p95), the nights agreeing to 22-29 mm, the rug correlating 0.04 (it
  also moves now); per frame the edge lands 37 mm off, flat with range, turning and timing. Ablations the same day
  (each imperfection off in turn, the rest on): without odometry drift, and without the floor's relief, the map is
  no better (17-21 mm from the drawn edge, the nights agreeing to 22-26 mm); with the robot's true pose each frame
  still lands 33 mm off (29 mm within 1 m). Lights on (the testing default, [72t]), measured on two lit nights'
  pictures rebuilt offline (`floor_mosaic2.py --truth/--dt/--mount`, 27 Sep evening): as the robot sees it 17.7-18.3 mm
  from the drawn edge; with the true pose 14.5; with the frames' 80 ms stamp lag also removed 9.5-11.9 (the nights
  agreeing to 7.7-18.5); with the mount's known pitch also corrected 9.8-9.9. So the pose costs ~5-8 mm, timing
  ~3-4, and the image itself (fisheye resampling, blur, rolling shutter, noise, compression) most of the ~6 left over
  the perfect sim's 3.3-3.7; lights on alone changed nothing (17.7-18.3 against the dim nights' 17-19).
  The walk-through on the realistic bench (`data/bench/real1`, recorded 27 Sep evening, lit, every imperfection on:
  house 70 toy sets 1-6, house 71 sets 1-2, 24 toys), with the settings tuned on the perfect sim: a candidate by 83%
  of the toys (20 of 24), 93.5 candidates near no toy a night (17.6 on the rug); the requirement is 93% and 50.
- The two empty nights' floor pictures agree as the detector sees them ([FHA]), so clean floor
  consistently stops reading as changed: compared by the detector's own test (its colour threshold
  and slack), at most 2% of the open floor it judges reads as changed between them, on every tuning house
  (`sim/house/floor_agreement.py`; the rug and moved chairs counted apart). Measured 1 Oct on the six houses of
  `data/bench/houses6c` as built then (odometry's placement, 3 views a cell): 1.6-17.1% of the open floor (houses 76,
  71, 72, 73, 70, 77: 1.6, 2.5, 5.0, 5.9, 11.4, 17.1). On house 77, rebuilt offline: with the sim's true path 7.6%,
  so the robot's placement is half of it; with that path, 15 views a cell and the darker quarter of them taken
  (reflections and glints only add light) 4.1%, the most this picture builder gets; placed by DimOS's pose graph over
  both nights' lidar (`house_frame.py`), then stitched from there (`stitch_floor.py --near`), with those views, 4.8%.
  Odometry drifts 10-68 cm (median per night) and up to 1.1 m and 10 deg over a 3-7 minute sweep, beyond the odometry
  stitch's +-32 cm, 2 deg search: most of its pairs never locked (9-103 of 150 needed, on five of the six houses).
- The walk-through, on the 14-night bench (`data/bench/rug2`, recorded 27 Sep: house 70 toy sets 1-12, house 71 sets
  1-2, 42 toys; `bench_detect.py`): a candidate by at least 93% of the toys (98%: 41 of 42), with at most 50
  candidates near no toy a night (40.5).
- At least 70% of easy toys and 50% of hard toys are binned ([QyZ] defines the tiers). A toy too big for the jaws
  counts as handled when it is left and reported for review as too wide; calling a toy that fits too wide does not.

## Decisions

1. [2w8] **Finds come from the floor comparison only**: tonight's floor picture against the saved
   floor model, so a find is something new since last night, as the system spec means it. The DimOS loop meanwhile
   finds items by running the OWLv2 open-vocabulary detector on the head camera (`TidyArm._head_finds`,
   `_find_items`, `wrist_refine.owl_boxes`). That is a stand-in: it is removed in the change that wires the floor
   comparison into the DimOS loop, and until then end-to-end runs use it. It was also the weakest link end to end
   (the triceratops flagged at the same wrong spot every run, 24 Sep).

2. [hIq] **Look-closer finds and names in one step; the local VLM is held in reserve**. OWLv2
   gives a box and a label in one pass and EdgeTAM cuts the outline, which also gives the grip width and direction, so
   a separate naming step by the local VLM (Moondream) added time and nothing the pick used. Moondream stays wired
   (`sim/arm/look_vlm.py`) for questions a label can't answer, such as "is this trash or a toy?", and comes back as
   its own criterion if that is needed.
3. [QDc] **Certainty and energy rise together**: the deeper a check goes, the surer it is and the more energy it
   uses. Each check is dearer than the one before (approach views: free; a
   sidestep: seconds standing; the arm: a full stop, lie down, look, stand, about 30-60 s est.), and each is used
   only when the cheaper one left the candidate unsure. Measured 27 Sep: the sidestep for an unsure spot didn't pay
   (21 of 199 unsure spots were at a toy, every one of those toys also confirmed), so an unsure spot is not sidestepped;
   evidence gathered on approach decides, and the gate in [6Pn] says which confirmed spots are worth the arm. A
   standing look comes before the arm's full stop (a wider field of view, and it can shift slightly without a whole
   park and re-park): facing the
   find from ~0.75 m, the wrist camera aimed down at it from ~0.6 m, one frame through the open-vocabulary detector. A
   toy seen within 45 cm of the flag becomes the find's position (the flag sat 13-30 cm off it); nothing there, with
   the flag in clear view, clears it without parking or lying down. Balance measured first: the aiming poses leave the
   standing tipping angles unchanged ([BPL] Decisions). The PoC averaged about 43 false alarms a night with a whole
   floor picture; each one settled before the arm saves a stop. It replaces sweep-then-visit, which came from the PoC's
   batch tools, and makes [S7o]'s head-on approach the sweep's own direction. How many views, over what distance, and
   the sidestep's size are to be measured, starting from the night-3 recordings of 25 Sep.

4. [Xl1] **DimOS's sim renders the house nights the same way every night, and its cameras see close.** Hard-won,
   25 Sep 2026 (MuJoCo 3.10.0): (a) MuJoCo clips each camera at `vis.map.znear` times the model's extent, and the house
   is 26 m across, so nothing within ~26 cm of a camera was drawn: the wrist camera, 18 cm above an item on a close
   look, saw through it and its own jaws. Every close look in the DimOS house nights before
   this was blind, and their "not seen at the flag" results don't count; the standalone sims (35 cm extent) were
   unaffected. Cameras now clip at 5 mm (`go1_arm.near_clip`). (b) The floor comparison needs tonight and the
   baseline nights rendered alike: a sharper shadow map tonight than in the baselines turned every shadow edge into
   a find. Each baseline night records its render settings (`sim_hook.render_settings`), and `night_tidy.sh`
   re-sweeps a baseline rendered differently from tonight. The model's extent is also taken from the house's fixed
   parts only, since MuJoCo fits the sun's shadow map to it and a larger extent moved shadow edges (655 px in one
   view for +3%).

5. [6Pn] **Confirm-on-approach settings, measured on Filament's photographed house** (26 Sep 2026, [QDc]'s "to be
   measured"). Tuned on seed 70's night-lit house (a rustic dark wood and a grey carpet), night 3 with three toys and
   night 2 with none, replayed offline (`confirm_stream.py`; the live tidy uses the same `CONFIRM`): at least 4 changed
   views, 60% of those that saw it, over at least 0.9 m of approach; tiny blobs must be clearly changed (peak 0.6); and
   a brightening counts only when strong (1.0; the detector's own 0.30 fired on glossy wood's moving lamp highlights).
   Toys confirmed 3/3 on night 3 at both settings; stops for nothing 25 -> 7 (night 3) and 21 -> 6 (empty night 2).
   Floor pictures are brightness-matched frame to frame (`floor_mosaic2.py --gain`: the cameras auto-expose). Tuning
   nights only; the numbers that count come from nights with other toy placements and houses, after it was frozen.
   Held out with those settings frozen (seed 70 toys 2-3, new house 71 toys 1-2): found 3/12, binned 0, 4.2 stops for
   nothing a night. From their recordings, two changes followed: an object-sized blob the detector's split calls
   "gone" still gets a look (the calibration nights are empty floors, and the split called dark toys on dark wood gone:
   the missed train, 26 Sep), and look-closer's OWLv2 cut-off is 0.05, not 0.08 (a visible lion on dark marble in the
   dim night house scored 0.07; the scripted grasp in the standalone sim lifted 19/23 at either).
   Frozen 27 Sep on the 14-night bench (house 70 toy sets 1-12, house 71 sets 1-2; 42 toys; recorded after the floor
   map's fixes, [lTq]): a 20 cm band by mapped structures (was 40) and a colour shift of 0.045 (was 0.06). The
   walk-through raised a candidate by 41 of 42 toys (98%), 40.5 candidates near no toy a night; the defaults found 88%
   with 22.3; the band alone 95% with 25.8. The widest was chosen for margin, as the funnel asks, since later looks
   remove false candidates (on the bench recorded before the map's fixes, the same settings gave 83% with 107 a night).
   What is stopped for is gated separately (`TidyArm.STOP_GATE`, 27 Sep): the wide top of the funnel made 75 stops a
   night live, binning 0-1 toys in two tuning nights, because every confirmed or unsure spot cost a stop, a close look
   or a sidestep. From the bench's labelled decisions (`decisions_*.json`): a confirmed "present" spot is stopped for
   with at least 4 hits over 15 cm2, a confirmed "gone" one (dark toys on dark wood; never stopped for before) with
   at least 40 hits, and an unsure spot is not sidestepped: 95% of toys still stopped for, 12 stops for nothing a night
   (40 before). An unsure spot is not sidestepped ([QDc]; `STOP_GATE["unsure"]` switches it
   back), and only spots worth a stop are kept out ([tmq]): with every weak spot kept out the first
   gated night still made 43 stops, 39 of them unblocking, and binned nothing. The risk kept: a toy that raised only weak
   evidence (5% on the bench) can be walked into.
   A rug new tonight broke into toy-sized blobs (17 of tuning night 70/2's 21 stops for nothing, 26 Sep). Major floor
   changes are now split from toy detection ([yyJ]: the rug is in the baseline); `group_m` (blobs that close judged
   floor-sized together) is kept, off, for that later test: on the rug-appears bench it cut false candidates only
   31.3 -> 29.9 a night and lost a toy. Accepted then: a toy on a rug the night the rug appears is missed.

   Wider on 28 Sep, since more false positives are acceptable if more toys are binned within a single charge of
   battery: on the realistic bench, toys getting a stop against stops a night: the gate above 16 of 24 at
   15; every confirmed spot 18 at 29; present spots of any size with 4 hits, "gone" spots with 5, and unsure spots with
   2 hits sent to the standing look 20 at 49. The last is round 10's (`HK_STOP_GATE`); the 4 toys left raise no
   candidate at all (the sweep's own sensitivity: open). A standing look is ~30 s; the battery is ~237 Wh (est.) and
   nights run 10-30 min of robot time.
6. [QyZ] **Grasp tiers, from each toy's own geometry, set before any binning result was scored against them** (26 Sep
   2026, as [D4v] asks of detection's tiers). As the toy lies when the night begins (the sim's truth records each body's
   resting orientation), its grip width is the narrowest band of the jaws' 4 cm it can be taken across along its length
   (a tapered case fits at its zipped end): *too big* if that is wider than the open jaws take (`jaws.max_m()`, 9.5 cm
   bare); *hard* if it fits but is under 2 cm (a pencil: little to close on) or over 7.5 cm (at the jaws' limit), or the
   toy stands under 2 cm tall (a disc case lying flat); *easy* otherwise (`items.grip_width_at`, `items.grasp_tier`).
   Also *too big* if its footprint doesn't fit inside the bin, 30 x 22 cm (a 35 x 28 cm wooden cart
   turtle, *easy* by its 6.9 cm head, is too big to pick up; shoes, 27 x 10 cm, rest against a side of the
   bin but bin reliably, the upper end of what is binnable). Every round's score was redone under this rule
   (`score_tidy.py`), so rounds compare. A toy counts as binned when its centre rests over the bin (within 5 cm of its
   walls, above the tray), not merely near the robot: the test had been 35 cm from the trunk's centre.
   Whatever was binned counts as handled; a toy left as too wide counts only if it truly doesn't fit.
   Hard also takes the kinds of toy that are hard to pick up (the winged pony, tiny toys, and long ones that tend to
   rotate): *awkward*, no band across it fits the
   jaws but one part does with 2 cm to spare (7.5 cm or less; a leg, a tape roll's wall), the toy not flat, so it is
   hard, not too big, and likewise a toy whose narrowest band crosses a thin part, under two thirds as wide, of a body over 7.5 cm (its
   bands' median width: a mug's handle, not a shoe's taper; a check of the rule on the held-out toys, blind to which they were, found two such called easy,
   29 Sep); *tiny*, under 6 cm every way, height too (a small toy standing up is taken readily; the same check);
   *long*, over 20 cm and at least 3.5 times as long as it is wide (a long
   dinosaur, a car carrier; not a shoe, which rests steady at under 3 to 1). Easy toys are expected to bin at a higher
   rate than hard ones, and each tier's rate is reported (`round_summary.py`).

7. [lTq] **How the floor map is measured and built** (26 Sep 2026, for [FHA]). Measured on a bench, not on nights
   (each step of the funnel optimised on its own): the house scene alone, cameras carried along two lawnmower routes
   25 cm apart in opposite directions, Filament colour and MuJoCo depth (floor hidden behind furniture is skipped),
   compared cell by cell and at fixed transitions. Found so far, head camera, true poses: (1) DimOS published the
   head camera's lens as 45 deg while it rendered at 51.8, so every floor picture from 26 Sep afternoon was projected
   with the wrong lens: DimOS now publishes the lens from params/robot.yaml, and the sim refuses to start if they
   differ. (2) A cell took one pixel's colour: two maps sampled different points of the carpet's pile and either
   side of an edge, correlating 0.60-0.94. Every pixel landing in a cell is now averaged, from the cell's sharpest
   frame: 0.96-0.99, 8-13% of cells differing. (3) The reference edge is measured, not assumed: straight down, in flat
   light, a point every centimetre (the texture's 3 mm pixels put an edge 2-3 mm off its nominal line, and a stretch of
   one 13 mm off). (4) An edge is placed where brightness changes fastest, not where it crosses halfway between the two
   floors: a shading ramp across it had moved the halfway point up to 20 mm. With true poses the stowed wrist camera
   (41 deg down from 59 cm, ~2 mm views) places both edges to 0.1-0.2 mm between maps (1.5-2.6 mm p95) and 0.3-2.5 mm
   from the drawn edge (3-4 mm p95); rug cells differing 1.7%; the head camera, looking along the floor from 29 cm,
   ~5 mm views, is several times worse. The open problem is pose error: 1 cm and 0.5 deg of error per frame spread the
   head camera's edges to ~20 mm p95. Both renderers draw an ideal pinhole lens and the bench maps with one; real
   lenses need their distortion calibrated out, still to be modelled here, as is a realistic lidar (the Go2's L1:
   ~21,600 points a second over a hemisphere, where the sim's depth cameras give a dense exact image).
   In the night pipeline itself (floor_mosaic2.py and the confirmer), scored on two empty baseline nights with the same
   measures (`bench_mapping.py --pictures`): the rug's two nights correlated 0.00 and edges disagreed by ~20 mm median,
   60 mm p95. Four more causes, each fixed where it starts: (5) DimOS published the head camera at its real-robot
   placeholder, 30 cm ahead of the trunk at trunk height, where the sim's is 29 cm ahead and 1 cm higher: the mount is
   now params/robot.yaml's, and a second, hard-coded copy in DimOS's tf publisher is gone (a test checks what is
   published). (6) Frames were stamped when DimOS polled them (20 Hz), not with the state they show, so each was paired
   with a pose up to 50 ms later: the sim now writes each frame's state time beside it. (7) Floor cells read the nearest
   pixel, half a pixel off DimOS's principal-point convention: they read the mean of the pixels they cover
   (`floor_sample.py`, shared by both). (8) Saved floor pictures were reused after the builder changed: they carry a
   version. (9) mj_step leaves body and camera positions from before its last step, so frames showed the camera 5 ms
   before the pose published with them: the sim refreshes kinematics before publishing and rendering. (10) Every tool
   reading a recording timed poses by when the recorder stored them, 5-19 ms after the pose's own time: they read the
   pose's own time now (per-frame edge error 19 -> 8 mm). (11) Edges
   are located in red minus blue, not brightness: the wood's dark planks beside a seam had pulled the reference edge
   2-3 cm and made a straight edge look jogged. On house 70's two empty baseline nights (27 Sep, 01:40): the head
   camera's pictures place the carpet edges 3.3-3.7 mm from the drawn edge (median; 12-13 mm p95), the nights agree to
   4.7-6.1 mm (median), the rug's correlation 0.88 (0.00 before these fixes); the wrist camera, placed by the stow
   pose's kinematics, 4.6-4.9 mm. Frame-to-map alignment (for the real robot's pose error; the sim's odometry is exact) corrects one known
   error exactly on well-textured views but not combined errors: parked, the plain dark carpet gives it no peak.
8. [KkQ] **Frames are placed in the house's frame by the lidar, not by odometry** (the system compensates for
   real-world error now, not later; the odometry drifts, [nAH] 2Ak). Walls and furniture
   don't move: each lidar scan's structure (0.1-1.5 m up) is matched in 2D to the house's map (trimmed ICP: the
   worst-matched fifth ignored, chairs that moved), from the last scan's correction; the house's first night builds
   the map as it goes. A frame's pose is its odometry pose corrected by the correction at its time, interpolated
   between scans (`houndkeeper/localize.py`). Measured 27 Sep against the sim's true path (`grade_localization.py`):
   the hand-built matcher, scan to map, followed a first night's own drift and lost track on night 2; DimOS's own pose
   graph (`dimos/mapping/loop_closure/pgo.py`, fed the recording's lidar with odometry poses baked in by
   `dimos map pose-fill`), run over both nights as one stream (5 cm submaps, a keyframe each 25 cm, odometry trusted
   less, loops more), brings night 1 to 15-20 mm of its true path (median; odometry 65) and the two nights to within
   33 mm of each other where both passed (odometry 308; p95 67 against 513). It is the route taken: the robot runs
   DimOS's mapping anyway. On a second, lit pair of nights with more drift (night 1's odometry 209 mm off, the nights
   overlapping at only 42 places): 91 and 73 mm per night, 34 mm between the nights; a staged handover (night 1
   corrected first, night 2 started on its dock) was worse. Registering each ~0.8 s batch of frames onto night 1's
   floor picture (`floor_register.py`) matched a third of the batches and did not improve the
   pose; the stamp lag found from the turns (`calibrate.frame_lag`: 0.080-0.085 s, the sim's truth 0.080) helps only
   once the pose is right. Localization is the open problem: the floor map stays at 17-20 mm. Then, still to build: each frame's floor patch registered to the floor
   picture within the lidar's centimetres, for millimetres; the floor's own heights (carpet, rug) from the lidar's
   floor returns, projected onto instead of a flat plane; the head camera's mount and the frames' delay calibrated
   from the recordings (on the robot: frames stamped from the encoder's `pts`, not on arrival, and that clock's offset
   to the odometry found from turns, waymark [nAH] UMH).

   Evening of 27 Sep, the pipeline as built: the house's frame is its first night's lidar map (`localize.py`, built as
   it goes, now with each scan's heights taken from its own fitted floor plane, `localize.level`: the walking body's
   ~0.5 deg of pitch and roll put the floor's far returns 3 cm up); every baseline picture, tall layer and walk is
   placed against it (`floor_mosaic2 --house`, `tall_layer --corrections`, `night_tidy.sh` picture v5), and the live
   sweep localizes each scan as it arrives (`live_localize.LiveLocalizer`), judging the floor in the house's frame
   while it walks in the odometry's (`HouseFramed`). Each frame is placed where the camera was when it took it (the lag
   found from the turns, saved in each picture as `lag_s`, used live). Three builder faults found on the way, each
   making the baseline pictures differ from any night's view: the house-mode height map took table tops and chair backs
   (0.7 m, above every ray to the floor) as sight-line blockers, and raw floor noise as tall, carving streaks through
   the pictures; the brightness matching walked the pictures' level to half the camera's (23-37 of 255 against 62),
   where 8-bit steps and noise swamp the floor's colour. Measured on house 70's lit baseline pair, picture edges
   against the drawn seams (`bench_mapping.py`, median): odometry 17.7-18.3 mm; house frame and lag 15.8-16.9; the true
   path 14.5, with the lag 9.6-12.2, with the calibrated mount as well 9.4-10.0. The pose now costs ~5 mm; the rest is
   the camera. The walk-through bench (real1, 24 toys) with the house frame: a candidate by 96% of toys (odometry: 83%)
   but 108-110 candidates near no toy a night (93.5). Nearly all are the colour test (brightening alone: 8.6 a night,
   38% of toys), at the same places every walk, and not noise: a cell's change averaged over every frame stays over
   the threshold. On the slate tile the baseline picture is a smear of views centimetres apart (each cell the median
   of its 3 closest views, from different distances), which no sharp frame matches. Open: compare each frame with the
   baseline's own views from nearby (the mount's and lens's errors then fall the same way in both), rather than with a
   blended picture.

9. [WFI] **The floor is judged after it is registered, against the nearest earlier views** (principle [fXD]).
   Four layers, each a fallback for the one before:
   1. The lidar places the robot in the house's frame to a few cm ([KkQ]).
   2. Each earlier night keeps its own views of the floor (`confirm_stream.Views`, `baseline_views.py`: every 8th head
      frame painted onto the floor grid, with where the camera stood and faced; 22-33 MB a night). A frame tonight is
      compared with the views taken from within 0.4 m and 25 deg of where it stands, nearest first; the blended
      picture fills the rest. From about the same place, the mount's and lens's errors, the floor's relief and its
      gloss fall the same way in both.
   3. The reference is snapped onto the frame's own floor (`floor_register.snap`): the turn (+-2 deg) and shift (+-8 cm)
      that best lay its edges on the frame's, by masked correlation over every placement at once (Padfield's masked
      NCC). It is taken only as a lock: a good match (0.30) clearly better than any placement 2+ cm away (by 0.08),
      and not on the search's edge. Cells already found changed are left out of the matching, so a find doesn't drag
      it; the whole 2.5 m patch moves as one piece, never cell by cell. Plain carpet, whose pile looks different each
      night, gives no clear best and is compared unsnapped.
   4. The colour threshold rises where the floor is dark enough for the frame's own noise (measured in the frame) to
      shift its colour that much (`--noise-k`): at night dark carpet away from the lamp is mostly sensor noise.
   Then, still to build: a candidate's own slop check (its neighbourhood slid a few cm as one piece: a misplaced edge
   matches somewhere, a toy nowhere), and the floor pictures themselves built the same way: every view of a swatch,
   from every night, fitted into the one arrangement where they all agree (a panorama's global alignment), for crisp
   edges again.

   Known methods this rests on: co-registration before differencing is the first step of change detection in remote
   sensing, where misregistration is the largest source of false change (Townshend et al., 1992, IEEE TGRS; Dai and
   Khorram, 1998, IEEE TGRS); comparing against the reference taken from the nearest viewpoint is how street-view
   change detection works (Sakurada and Okatani, 2015, BMVC; Alcantarilla et al., 2016, RSS); localizing against the
   nearest stored keyframe, not a global map, is visual teach-and-repeat (Furgale and Barfoot, 2010, JFR); floor
   texture alone places a camera to millimetres (Zhang et al., Micro-GPS, 2019, ICRA); and fitting many overlapping
   images into the one consistent arrangement is a panorama's global alignment, a bundle adjustment (Szeliski, "Image
   Alignment and Stitching", 2006). Risks: over-matching (the snap explaining a toy away:
   held off by the rigid whole-patch snap, the leave-out of changed cells and the lock test), repeating tile (a snap
   onto the neighbouring tile: held off by the lidar's cm and a search well inside the 48 cm tile), coverage (no
   earlier view nearby: the blended picture), and lighting (views are only like tonight's under the same lights: the
   lit regime for now; views per lighting later).

   Measured 27 Sep on walk 70/3 (bench real1, house frame, lag off, `--smooth 1`; stops for nothing): against the
   blended picture 97; snapped onto it, no frame locked (the smear); nearest views 83; nearest views and snap 72-73
   (58 of 152 frames locked, most 1-4 cm and 0-1 deg off); and the noise-aware colour threshold, 48 with all 3 toys
   (k=1), 20 with 1 of 3 (k=2). Over the whole bench (24 toys, candidates confirmed or unsure near no toy, a night):
   blended 108, nearest views and snap 100 (96% of toys); of those, carpet 53 confirmed, tile 25 confirmed and 21
   unsure, the carpet's edge 8 (the snap's own target, now the fewest). With the noise-aware colour (k=1) and the slop
   check (6 cm, a spot dropped when under 30% of its cells still differ at the best placement): 66 a night, 92% of toys.
   Carrying each lock forward as the pose's running correction (`--track`: the floor as the localizer, [fXD]) converged
   on walk 70/3 with odometry alone (locked residuals 0 mm, 0.00 deg; 37 stops for nothing against 46 unsnapped), but
   over the bench lost a toy (62% against 67% in odometry's frame): kept off by default until it clearly helps.

   The bench was scoring each candidate where the confirmer placed it; it now scores where the robot would walk to (its
   odometry's spot at that moment, through the sim's truth then), which is what the real robot does. So scored, in each
   night's own odometry frame (the pipeline since the evening of 27 Sep, `night_tidy.sh` picture v7), a night: the old
   settings 101 near no toy and 17 of 24 toys; the registration layers 62 and 16 (now the confirmer's defaults); in the
   lidar's frame 66 and 21 of 24, but that frame is not available live (below).

   Localization, measured the same evening against the sim's truth after one rigid offset for all nights (the house
   frame's own placement; what remains is what the comparison feels): odometry left two baseline nights 20-27 cm
   inconsistent; the lidar matcher building the house map as it goes, 19-27 cm (it warps its own first night); DimOS's
   pose graph over both nights, 6.5-9 cm; walks tracked by the matcher against the pose graph's map, 12-17 cm. Live, on a
   72 m tidy night against the matcher-built map, the correction slid to 0.4-0.9 m with good-looking fits (odometry
   0.12); against the pose graph's map 116 mm median (odometry 118). The live match is therefore off by default
   (`HK_LIVE_LOC=1`). Two fixes found on the way: the step prior now grows with the distance walked, not per scan
   (most of a tidy night's scans are taken standing at finds); and the sim harness's injected house map (one message of
   ~24,000 points on the lidar stream) is no longer taken for a scan (`localize.real_scan`). Lidar here places the
   robot to ~10 cm; the centimetres come from the floor ([fXD]).

   The floor pictures stitched as a panorama (global alignment, Szeliski 2006; "the stitch" below) (`stitch_floor.py`,
   first try, 27 Sep, house 70's lit baseline pair in odometry's frame): every pair of views from different times
   whose patches overlap is matched coarse to fine by image registration (4 cm cells over +-32 cm, inside the diagonal
   tiles' 68 cm repeat, then 1 cm over +-6 cm; a lock, an accepted registration, is clearly better than anything 4 cm
   away), each lock becomes a floor-picture constraint between the two views (pictured as a spring between them), and
   one robust least-squares solve (Huber; pose graph optimization) gives each view a shift and turn, a night's
   consecutive views tied as odometry allows. With single frames as views, 239 of 3,969 pairs locked; with 4 frames
   stacked into each view (`baseline_views.py --stack 4`: less noise, more floor), 404, and after the solve the pairs
   agree to 9.7 mm (median). But the repainted pictures are no better on the seams (nights agree 22-25 mm, edges 17-19,
   against odometry's 20-27 and 17-20): a rigid shift per view can't undo what the camera's own model error does,
   stretching the floor along each view by distance (0.6 deg of mount pitch moves a point 3.5 cm at 1 m, 14 cm at 2 m),
   so views from different distances disagree differently. Next: the mount calibrated into every view before stitching
   (`calibrate_mount.py`, now taking the frames' lag off), then stitch.

   With the mount calibrated into every view (`baseline_views.py --mount`) the stitch locks 473-475 pairs on house 70
   and 629-650 on house 71; the seams metric still reads 17-25 mm, but matching windows of the two nights' pictures
   against each other (60 cm windows, masked correlation) the strong matches' median offset falls from 13-15 cm
   (odometry) to 3-10 cm. On the detection bench (both houses' baselines stitched, pictures, tall
   layers and views repainted; tonight's walk by odometry with the same mount, the confirmer snapping and tracking
   onto the views), a night: with the mount calibrated on the sim's true path, 22 of 24 toys (92%) and 50.1
   candidates near no toy; calibrated from odometry alone (pitch -0.69, yaw +0.87, roll -0.14 deg against -0.89,
   +0.66, -0.31 from the true path), 18 of 24 (75%) and 49.8. The requirement is 93% and 50: the stitch and the
   tracking bring the false side there; the toys now hang on how well the robot calibrates its own camera. (Tracking
   off, same stitched bench: 19 of 24.) `stitch_baselines.sh` is the pipeline's step for it (calibrate, stack, stitch,
   repaint), not yet switched on in `night_tidy.sh`.

   The mount solved inside the stitch (28 Sep; `stitch_floor.py --mount-out`, a self-calibrating bundle adjustment:
   three shared unknowns for the camera's small turn, each pair's constraint carrying how a turn moves each view's floor
   point at the overlap, `floor_jac`, which grows with the square of the range). The sim's planted error, in the
   calibration's own terms (the sim turns its camera by (-pitch, +yaw, +roll) about MuJoCo's camera axes, x right, y up,
   z back), is pitch -0.6, yaw +0.4, roll -0.3 deg. On house 70, starting from the odometry calibration: calibrated
   from odometry alone 0.09 / 0.47 / 0.16 deg off; from the sim's true path 0.29 / 0.26 / 0.01; solved in the stitch
   0.06 / 0.19 / 0.09 (0.22 deg overall against 0.51 and 0.39). `stitch_baselines.sh` v2 solves it, repaints the views
   with it and stitches again. On the detection bench with each house's solved mount: about 21 of 24 toys (88%) at
   about 49 candidates near no toy a night, against 75% with the odometry
   calibration and 92% with the true path's. House 71's own solve was yaw +4.1 deg, refused: its stitch had locked
   mostly pairs from where the robot stood still (the same view again, matching at exactly zero: now left out), and
   without them only 25-40 of ~5,000 pairs lock on its patterned tile: two thirds match well (median score 0.57) but
   almost none clearly best, the pattern giving several equal placements. Placed first by the lidar's pose graph and
   searched +-12 cm (`--near`), still 40, the best placement mostly on the search's edge (open: the patterned floor,
   is where the stitch fails). Even so, on those 40 pairs house 71 solved the mount as yaw
   -0.61, pitch +0.05, roll -0.09 deg of correction, house 70 -0.67, +0.15, -0.07: the mount is the robot's, not the
   house's, and two houses found it within 0.1 deg. So the robot keeps it (`data/robot/head_mount.json`): each house's
   stitch starts from it and refines it only on 150 or more locked pairs and under 1.5 deg (`stitch_baselines.sh` v3).
   But yaw does not settle: a second pass from house 70's own solve moved it again (+0.19 to -0.11 deg; the truth +0.4)
   while pitch and roll held (+0.04, -0.02). A camera's small yaw looks almost like each view turned, which the views'
   own turns already take, so the floor can barely tell them apart. The stitch now solves pitch and roll and holds yaw
   (`YAW_PRIOR_RAD`); yaw comes from `calibrate_mount.py`'s opposite headings (+0.87 from odometry, ~0.5 deg off: open).
   And the stitch fails safe (v4): a final stitch resting on fewer than 150 locked pairs paints the house by odometry
   and the mount, not its corrections (house 71 found 2 of 3 toys before the stitch, 1 and 0 after, on 25-40 locks).
   Why house 71 won't lock: its "patterned tile" is a fine checkerboard, a few cm a square, strictly periodic, beside
   plain carpet; the only unique things are the checker's edge against the carpet (a line: it pins one direction), a
   stain, the rug. Its two nights, placed by the pose graph, lie 8 cm apart at the median but 36 cm at p95, beyond a
   +-12 cm search; at +-32 cm the checker repeats; blurred past its period (6-8 cm cells) nothing is left to match
   (80 sampled pairs, 0 locks at every scale; a direction-aware lock would add 1 of 80). A floor like it cannot calibrate
   or stitch itself; it keeps the robot's mount, the lidar's placement, and the per-frame snap where a frame has edges.
   A real complex, irregular tile design is far less periodic than this stand-in.

   The map measured everywhere, not on two seams (`floor_truth2.py`, 28 Sep): each 60 cm patch of a picture laid on the
   floor the sim truly drew (its 3 mm texture averaged to 1 cm), by edge correlation within +-8 cm, kept only where
   clearly best (14-57 patches a picture on house 70; phase correlation on brightness had matched at random). After one
   rigid offset for the picture (the house frame's placement, 60-70 mm, which costs the comparison nothing), median:
   odometry 54 / 61 mm (nights 1 / 2); stitched with the odometry mount 31 / 30; with the true path's mount 19 / 14;
   with the mount solved in the stitch 26 / 14; the true path itself 14 / 14 (the camera model's own floor). The
   stitch takes the map's own warping most of the way to what the true path gives; the seams metric did not see it.

   Downstream, the same evening: the standing look found the find in the wrist image but dropped it, the flag's pixel
   on the jaws' tip (the look now aims the flag 15 deg above the image's centre, `LOOK_UP_DEG`); the stowed wrist
   camera's exposure had run up to 250x gain (capped at 16x, and the look now waits for the exposure to settle,
   `DimosRobot._settled_image`); and a 4 cm wind-up toy at the flag scored 0.118 against the detector's 0.15 (the look
   now looks again in a crop around the flag, enlarged 3x, at 0.10: sliced inference, `wrist_refine.owl_boxes_zoomed`;
   the floor's own boxes there scored 0.03-0.07). A shoe was reported binned while it lay on the floor 85 cm away: the
   jaws held it through the carry, so it most likely fell out of the bin afterwards; nothing checks the bin (open).

   Live standing-look nights (house 70 toy sets 1, 3, 6; house 71 set 1; a round is the four), toys found / binned /
   stops for nothing a night: round 1 (before the evening's fixes) 1-3 / 0-1 / 6-38; round 2 (aim above the jaws,
   settled exposure; the live lidar match on, which slid) 2-3 / 0-1 / 22-30; round 3 (odometry's frame, the
   registration layers) 1-2 / 0-1 / 4-33. Round 4 (the baselines stitched, the mount calibrated, floor tracking, the
   zoomed look), night 70/1: all 3 found and each seen standing within 4-9 cm, 1 binned, 9 stops for nothing, in 810 s
   (against 1,500-2,180). Of the two not binned: the pony, judged too wide for the jaws though its tier is grippable
   (open); the 4 cm wind-up toy, lost at the close look, whose wide first look took a slate tile ("a piece of cloth",
   21 cm off) over it. A find the standing look placed is now re-acquired within 10 cm only (`PLACED_TOL`).
   Night 70/3: all 3 found, 0 binned, 16 stops for nothing, 891 s. A computer mouse and a roll of tape were in plain view
   at their flags and unseen: the detector was asked only for toys and cloth. It now also asks for the other things
   left on a home's floor (a small object, a mouse, tape, a cup, a mug, a bowl, a towel, a glove, scissors, crayons, a
   pencil case); at 25 empty flags that added a detection near the flag at 1. The Transformers figure was judged too
   wide (11.6 cm) though its tier is grippable, as the pony was: the width is measured on the find's mask laid flat on
   the floor, and a standing toy's upper parts land further along the line of sight (open: the three close looks are
   only 6 cm apart, too close to intersect their footprints). Round 4 over its four nights: 10 of 12 toys found, 3
   binned, stops for nothing 9, 16, 29 and 4 (14.5 a night; round 3 23.5, round 1 26), 620-1,634 s a night. With the
   broader queries the close look's wide pass finds the wind-up toy 1.9 cm from its flag. Round 5 (the broader queries,
   the close look held near a standing-look placement), night 70/1: all 3 found, 1 binned, 8 stops for nothing, 698 s.
   There the wind-up toy was missed at the standing look again: the detector's score for it swings with the crop's
   scale (0.09-0.12 at 3x, 0.21-0.23 at 4x, 0.03-0.10 at 5x, 0.04-0.18 at 6x, two looks), so the look now tries 3x and
   4x (at 25 empty flags still one extra detection). And the pony, gripped and lifted, weighed 2.6 kg (open: the
   weighing tares empty at the lift pose and reads loaded there, and the imperfect arm's lash and flex ([nAH] z7P) may
   not hold the same torques at the same pose; the grasp bench measures it). Round 5 over its four nights (3x and 5x
   crops from its second night; 3x and 4x from here on): 10 of 12 found, 5 binned (round 4: 3; round 1: 2), stops for
   nothing 8, 16, 21 and 7 (13 a night), 600-1,445 s a night. Round 6 (the width measured at the find's height too,
   the weighing on the arm's own readings; the sim slowed to ~0.6x by offline benches running beside it): 7 of 12
   found, 3 binned, 18 stops for nothing a night. Round 7 (the stitch step v3; nothing beside it): 10 of 12 found (all 9
   on house 70; 1 of 3 on house 71, whose patterned floor won't stitch), 4 binned, stops for nothing 11, 11, 16 and 3
   (10 a night), 313-1,185 s. Round 8 (grip by one part): 9 of 12 found (all 9 on house 70, none on house 71), 5 binned
   (2, 1, 2, 0), stops for nothing 17, 5, 7 and 6, 357-1,227 s. Of the finds called too wide on those nights,
   the pony on house 71 truly is (it lies there 9.8 cm across its narrowest, over the jaws' 9.5); the Transformers
   figure is not (6.3 cm) but its looks measured it 13.6-15.0 cm, both laid flat and at its height (open: its mask).
   The weighing, on the standalone grasp bench with the
   imperfect arm (16 trials, classic renderer: Filament, OWLv2 and EdgeTAM together outgrew the 8 GB GPU): 13 lifts
   weighed within ~15% of their true mass (17 g as 18, 532 g as 462). The pony's 2.6 kg came through DimOS, whose
   arm adapter supplies the joint torques (open: check the efforts DimOS reports against the sim's).

10. [bnO] **The hold-out night, fixed before it is run** (working toward it is mostly a
    matter of consistency, since what it will show is unknown). Run once, on code and settings
    recorded as they ran (the commit and a copy of any uncommitted changes, kept beside its results; it
    needn't be committed first), with nothing changed between its nights and nothing tuned on its
    results afterwards (its failures become the next round's work on the tuning houses, not changes judged on it):
    - a house never used: seed 80 (on the tuning houses' floor plan, it turned out: until [uCP], 30 Sep, every seed
      drew the same plan; its floors, toys, lighting and stains were its own), its floors drawn only from the held-out photos (`HK_FLOORS=heldout`;
      3 carpets, 3 woods, 2 tiles, 3 patterned tiles; no held-out rugs, so a rug is a training one);
    - toys never used for tuning: `HK_ITEMS=heldout` ([DGv]), 3 a night, item seeds 1-3 (three tidy nights over the
      house's two baseline nights);
    - every realism mod on (register [nAH]), lit (REGIME=lit), the pipeline as it stands (stitch step v4, the confirmer's
      defaults);
    - run headless, with no viewer window (`DIMOS_SIM_HEADLESS=1`: only its results
      are wanted);
    - reported per night, and nothing more: toys found, binned, too-big toys binned or reported, stops for nothing,
      length and the pack left. Results only: no failure is examined, no look, image or log of the held-out nights is
      opened, and no held-out house, floor or toy is used to debug anything (no hold-out asset is spent:
      results, not why, no peeking); seeing why it failed would make it a tuning set.
      Compared with the tuning houses' consistency round (all eight toy sets, the same code), which it runs after only
      if that round clears the same bar on the tuning houses: all 8 nights scored, 7 of 9 found, half the grippable toys
      binned, every night ending with a quarter of the pack (`round_summary.py --gate 8`, first
      applied to round 12); too-big toys aren't gated there, at one or two a night.
    Toys are scored in two groups by their tier (from each toy's own geometry, [QyZ]), never mixed (ungrippable toys
    never count in the grippable toys' metrics; they are broken out): grippable toys (easy, hard)
    by binned of those present; too-big toys by binned (gripped by a part: better than reported) or reported too big,
    against knocked over, lost or wrongly refused. Pass thresholds, set before it runs (28 Sep 2026): over
    the three nights, at least 7 of 9 toys found, at least half the grippable toys binned, every too-big toy binned or
    reported, and every night finished on one charge with at least 25% of the pack left (the battery model's energy per
    pack, in each night's score). Stops for nothing are reported, not limited (more false positives
    are acceptable if more toys are binned within a single charge of battery; the proposed "no night over
    20 stops for nothing" was dropped: the wide stop rule made 55 a night and binned twice the toys). For scale,
    the tuning houses over 22 nights of 28 Sep: 21 of 52 grippable toys binned (40%); the losses 13 never flagged (the
    3.5 cm wind-up toy 6 of them), 9 wrongly too wide (most before that day's width fixes), 5 not seen standing, 3 not
    seen at the close look, 1 lost after a failed grasp. Round 16 (29-30 Sep; the wrist stereo pair with its light,
    the grasp through the proposers, 8 toys a night by [Afy], the tuning houses' 8 nights): 56 of 64 toys found (88%),
    35 of 56 grippable binned (62%; easy 29 of 42, hard 6 of 14), too-big toys 2 binned and 2 reported of 8, at most
    35% of the pack used in a night (65% left); `--gate 8` passes. Against the hold-out's bar it clears found, binned and pack, and not every too-big
    toy (4 of 8). Hold-out run 2 (30 Sep, the first to finish; run 1 never produced a score), round 16's code
    and rig, results only: 18 of 24 toys found (75%), 11 of 21 grippable binned (52%; easy 9 of 15, hard 2 of 6), too-big
    toys 0 of 3, at most 27% of the pack used a night. Passes binned and pack; misses found (7 of 9 would be 78%) and
    too big.
11. [Afy] **At most one too-big toy a night** (binning is more important to demonstrate than
    the too-big check: detecting that an object can't be picked up shows less than binning the items that can). The
    too-big check is still tuned and judged: one a night, and on its own bench. A night's toys come in a
    fixed mix by [QyZ]'s tiers (one too big, two hard, the rest easy or medium), in
    proportion to the floor so a bigger or smaller house keeps it: one toy per 200 sq ft of the house's floor inside
    its outer walls
    (18.6 m2), rounded, at least four; of them 12.5% too big (at least one), 25% hard, the rest easy, each rounded; no
    toy twice in a night (`house_toys.night_mix`, `house_toys.pick_night`). The test house, 16.1 x 9.1 m inside its walls
    (1,580 sq ft; 16.8 x 9.8 m, 1,770 sq ft, to the walls' outer faces), has 8: one too big, two hard, five easy. The
    hold-out's toy sets, not yet run, are drawn under the same rule, replacing [bnO]'s 3 a night.
12. [NvG] **A baseline picture needs 20 placed frames and 10 m2 of floor seen** (3 Oct 2026, for [YuI]). Set from the
    night_tidy pictures on disk (`data/runs/night_tidy/*/picture_n*.log`, 2 Oct and earlier; the hold-out houses' runs
    left out): every picture the stitch went on to use had 44 frames or more and 12% or more of the 16.8 x 9.8 m grid
    (20 m2, the small demo room, house 95); the two broken ones had 1 frame (2%, 3.3 m2; house 91) and 7 frames (9%,
    house 92). Both limits sit in the gaps, about half the smallest good night. `floor_mosaic2.py --min-frames
    --min-floor-m2`; 0 turns either off (a partial picture, `--part`, for a measurement).
13. [gI8] **Every derived file is made through one build helper** (3 Oct 2026, for [SdA]; hard-won). A builder run in
    the background and collected with a bare `wait` (which always returns 0), or piped through `| grep | tail` (which
    reports the filter's status, not the builder's), passed a crash silently, and an output not removed before its
    rebuild then stood in for it: the pipeline critique of 3 Oct (the project's private notes, finding 2)
    and the static sweep of the same day (P1-P10) found it in every script that builds the floor.
    `sim/house/lib/build.sh`: `build <out>... -- <command>` removes each output, runs the command with its own log and
    `pipefail`, and fails unless each output is there again and not empty; `build_bg` and `build_wait` run builds in
    parallel and wait on each one's own PID. `tests/test_build_helper.py` also fails on a bare `wait` or a `.py` builder
    piped into `tail` in the night pipeline's scripts.

14. [gsJ] **The robot's lag is the median of the tuning nights' estimates that pass a gate, on a named clock.** Hard-won:
   a per-night lag with no gate placed house 93's frames 0.33-0.40 s off (house 93 trial, 3 Oct 2026;
   the pipeline critique of 3 Oct, findings 3 and 7). `sim/house/calibrate_lag.py` reads each tuning seed's
   baseline recordings and keeps a night's estimate only if its correlation is at least 0.7, it is not on the edge of
   the +-0.3 s search, it rests on at least 150 turning frame pairs (an estimate, to be checked against the first calibration), and it lies in -0.1 to 0.3 s; it refuses
   `make_house.HOLDOUT` seeds. It writes `data/robot/frame_lag.json` with the clock its recordings were stamped on
   (wall before [3Pv]'s robot clock, robot after) and the nights it used.
   - The range reaches below zero for the robot's clock. On the wall clock (a sim below real time) the frames' lag
     was 0.09-0.17 s on every passing night, and the old 0-0.3 s range only refused sign flips such as house 93's
     -0.25 s. On the robot's clock the lag is near zero, and a night's estimate scatters either side of it: house 96's
     nights gave -0.030 and -0.015 s (correlation 0.87 and 0.91, over 1,000 turning pairs each), and its walk gave
     -0.055 s. The old range refused all three, though they were close to the truth: the floor map error was least
     at +0.015 to +0.03 s (decision [55G]). -0.1 s keeps them and still refuses house 93's flip.
   - `floor_mosaic2.py`, the confirmer's replay (`confirm_stream.py`, `score_confirm.py`), `calibrate_mount.py` and the
     live tidy read that file by default, each camera its own lag where the file has one (the wrist camera's is not the
     head's); `--dt auto` is now the check (a warning past 40 ms, or when the night's estimate fails the gate) and
     `--dt night` lets the night's estimate stand.
   - Each picture saves its head camera's lag (`lag_s`), never the wrist camera's, and the lag file's and mount's
     hashes (`lag_sha`, `mount_sha`).
   - The mount: `stitch_baselines.sh` reads `data/robot/head_mount.json` and never writes it; each house's own solve
     stays in its run dir as a check. `sim/house/calibrate_robot_mount.py` writes it from tuning houses' solves,
     refusing hold-out seeds. The stitch's stamp carries the mount's and the lag's hashes, so a new calibration
     re-stitches.

15. [55G] **The robot's lag is stored per clock, and a recording's frames are placed with its own clock's lag.**
   Hard-won: the stored lag (0.135 s) was calibrated on wall-clock recordings. Since [3Pv]'s robot clock (sim-harness
   [Rpj]), sim recordings are stamped on sim time, and their lag is near zero. With the wall clock's lag, house 96's
   floor map error was 13.5% on night 1, against 8.6% with +0.015 s. A lag read for the wrong clock gives no warning;
   it only blurs the floor.
   - `data/robot/frame_lag.json` holds `{"clocks": {"wall": {...}, "robot": {...}}}`, each entry what
     `calibrate_lag.py --clock <clock>` writes. `calibrate.store_lag` replaces one clock's entry and keeps the others.
     A file in the earlier single-clock form is still read, for its own clock only.
   - `calibrate.load_lag(path, clock)` and `camera_lags(..., clock=)` refuse a clock the file has no lag for. They
     never fall back to another clock's lag.
   - A recording's clock: `calibrate.recording_clock`, from the sim's path file (`traj.csv`). Since [3Pv], the file
     carries `host_t` (the host's clock) beside `wall_t` (the clock the messages were stamped on). `wall_t` then
     follows `sim_t` on the robot's clock, and `host_t` when a sim runs without `DIMOS_SIM_CLOCK`. A file with no
     `host_t` is from the wall clock. Checked on 3 Oct 2026: house 96's files (`wall_t - sim_t` constant, `host_t`
     514 s over 370 s of sim) against houses 90 and 93 (no `host_t`). Only the clock columns are read, never the
     true path ([1B2]). The path file is the one that a run lists the recording in: a sweep run's
     `seed_recordings.txt`, or a tidy run's `recording.txt`. A recording that no run lists, a real robot's among
     them, has its clock given (`--clock`). It is never guessed.
   - `floor_mosaic2.py`, `calibrate_mount.py`, the confirmer's replay, and `baseline_views.py` take
     `--clock` (default `auto`, from the path file) and `--traj`. `baseline_views.py` uses the
     stored lag too, and no longer estimate one of their own. `stitch_baselines.sh` and `night_picture.sh` pass each
     night's clock (`sim/house/lib/clock.sh`). `real_nights.sh` sets `RECORDING_CLOCK=robot`.
   - `calibrate_lag.py` refuses a recording stamped on a clock other than the one it calibrates.
   - Live: the tidy uses the robot clock's lag when `DIMOS_SIM_CLOCK` is set (`tidy_module.live_clock`). It uses
     the wall clock's lag in a sim that runs without the robot's clock. On the real robot, the robot clock is the
     wall clock, and so is its `robot` entry. That lag is measured on day one (`real/calibration/timing.md`, step 2),
     not taken from the sim's.
   - Stored on 3 Oct 2026, robot clock: -0.030 s, the median of three recordings of house 96 that all passed the gate.
     Night 1 gave -0.030 s, night 2 -0.015 s and the walk -0.055 s. The head camera only: the baselines have no wrist
     frames, so the wrist camera takes the head camera's lag. The entry also keeps the floor map error by lag on
     house 96 as evidence, but not as the lag. That error is scored against the sim's truth. It was least at
     +0.015 s (night 1, 8.56%) and +0.03 s (night 2, 8.31%), 45-60 ms above the calibration (9.92% at -0.03 s on
     night 1). The calibration reads only the robot's own turning, so the gap is open: it is either a bias in the
     turning estimate on this clock, or the map's error measuring something else.

16. [ObC] **How a view is registered against the floor, for [G9q]** (4 Oct 2026; hard-won where noted). Offline,
   beside the stitch (`sim/house/floor_refine.py`), from what the stitch made: both nights' views (`baseline_views.py`)
   painted with the shape solve's corrections, and those corrections. It is congealing (Learned-Miller 2006) with the
   lidar's pose graph as a prior; Zhou and Koltun's colour map optimisation (SIGGRAPH 2014) is the published form
   (the project's related-work notes). Each pass:
   - The picture a view is registered against is every view's mean per cell, less the view and less its own night's
     views within 5 s of it. Those neighbours share its pose error and would hold it where it is: the same lesson as
     the stitch's pairs, which skip a night's views closer than 5 s (`stitch_floor.SAME_NIGHT_GAP_S`). Zhou and
     Koltun keep the view in its own map; leaving it out is congealing's and atlas building's guard against that bias.
   - Registration is the snap's edge correlation (`floor_register.ncc_surfaces`) over +-6 cm and +-0.4 deg, to a
     fraction of a cell. A lock (an accepted registration, which becomes the view's floor constraint) is a match of
     0.30 or better, not on the search's edge. How sure it is comes from its whole correlation surface
     (`floor_register.surface_belief`), not from a lead over the next best: a ridge holds only across, and two
     near-equal placements count for little. Hard-won: at 1 cm a peak is cells broad, so the snap's lead test (0.08
     over anything 2 cells away) passed 1 view in 80 on house 70 (median score 0.68, lead 0.02).
   - No view ever moves more than 8 cm or 1 deg from where the lidar put it: the search shrinks to what is left of
     that, and the solve bounds and clips at it. 8 cm is the lidar's tail (`floor_register.SNAP_MAX_CELLS`) and well
     inside half a 30 cm tile. On house 93 the pairwise floor constraints' locks sat exactly one tile off.
   - One solve places every keyframe and view: `scipy.optimize.least_squares`, each row whitened by its own
     information, the bound as box bounds on the views. The lidar's matches between keyframes are house_shape's own
     matcher and pairs, measured once at the shape as solved and asking for no change in their relative place. Each
     is never surer than 3 mm: its J'J information says 0.3 mm, while the shape solve's matches disagree by about
     4 mm. Odometry ties each view to its two keyframes at 4 mm plus odometry's drift over the way between. Only the
     locks' rows take a robust loss (soft_l1 past 2 of their own sd); the lidar's, odometry's and the priors' stay
     quadratic, so no robust loss lets the images bend the shape.
   - The pass test uses the two nights' pictures rebuilt from the views as moved (each cell their mean), judged by
     the detector's own test over the floor both saw, away from the structures both mapped. That is
     `floor_agreement`'s share without truth. A pass is kept only if it lowers that share by 2% of itself.
     `floor_mosaic2.py` takes 4 minutes a night, too slow for every pass; it builds both arms' pictures once the
     passes are done.
   - The refined corrections are the keyframes' and the views' own, merged in time, for `floor_mosaic2.py
     --corrections`.
   - Not borrowed yet: Zhou and Koltun's per-image non-rigid correction. On house 70 the locks pulled views 11 mm
     along and 11 mm across the way each camera faced, the same on both nights: an error that turns with the camera
     (its mount or lens), which a per-view shift and turn only partly absorbs.

17. [qLu] **The floor refine's gain on the tuning houses is a millimetre or two, which the floor map error cannot
   see; it cannot undo an error a whole stretch of views shares** (4 Oct 2026, for [G9q]; audited and re-measured the
   same day). The first reading, "no measurable gain" by the floor map error, is void as evidence. Its positive
   control showed that measure could not have seen a gain of this size.
   - The wiring holds. `floor_refine_bench.py` builds the refined arm's pictures from the refine's own corrections,
     into a directory of their own, and scores each arm by its own corrections. Houses that kept no pass (87, 89, 91)
     rebuilt byte for byte the same pictures. Where passes were kept, the refine moved the night's path by 0.6-5.8 mm
     (median by house), 8 mm at the 90th percentile, and 32 mm at most. The two arms' pictures then differ by more
     than 10 grey levels on 0.7-13% of the floor both saw (29% on house 72).
   - House 72's first row was measured against its stitch from before [Tu2] re-stitched it (decision [myl]).
   - The positive control (`floor_refine_bench.py --plant 1:0.40-0.55:0.05,0,1.0`, house 70): 15% of night 1's
     path was moved 5 cm and turned 1 deg (52 mm median at the path). With the same builder, night 1's floor map
     error went 2.09 -> 2.10%, and the nights' disagreement 1.58 -> 1.61% of the open floor. The measure barely
     moves for a planted error. It is the detector's own test: colour only, with 2 cm of slack. Shifting the whole
     of night 1's picture by 2, 3, 5 and 8 cm reads 2.18, 2.31, 2.69 and 3.50% against 2.09%.
   - The bench now also measures the map's placement against the true floor (`floor_fidelity.placement`). Per 40 cm
     patch, overlapping by half, it finds the sub-cell shift that lays the map's structure on the truth's. It reports
     each patch's distance from the truth, and, paired against the floor-constraint arm (the stitch's pairwise floor
     constraints alone, no refine), the mean change over the patches both matched. On the same control it read the
     planted night 2.1 mm further from the truth on average (42.8 -> 44.9 mm over 770 patches). The truth's own
     placement is coarse (35-90 mm a patch on these houses, where the sim's render meets the flat texture), so only
     paired changes are read.
   - The refine moved the planted path 1.8 mm (median) and took back 1% of the plant. The planted views' locks pulled
     -0.5 mm along the plant (median, 52 of 89 locked), against 25 mm in scattered directions for the others. A view's
     reference leaves out only its own night's views within 5 s. The rest of the planted stretch stays in, so the
     reference carries the same error and the lock sees none. The solve also holds the shape: the lidar's matches ask
     for no change from the shape as solved, at 3 mm each, and odometry ties each view to its keyframes at 4 mm plus
     drift. The locks' first pass pulled views 8-32 mm (median by house), while the views moved 1-8 mm.
   - Re-measured on the eighteen tuning houses (70-73, 76, 77, 82-93; no hold-out, not 78). The floor-constraint arm
     is today's builder over the shape solve's corrections. The refined arm is the first run's refine corrections
     rebuilt with the same builder. House 72 was refined afresh on its re-stitched run. The robot's mount and stored
     lag per clock were used throughout. Placement is the paired change in mm, negative nearer the truth, nights
     1 / 2.

   | House | Views | Passes kept | Error n1 | Error n2 | Disagree, open | Placement n1 / n2 (mm) |
   |---|---|---|---|---|---|---|
   | 70 | 974 | 1 | 2.09 -> 2.04% | 2.39 -> 2.33% | 1.58 -> 1.57% | -1.5 / -2.5 |
   | 71 | 990 | 3 | 0.47 -> 0.43% | 0.96 -> 0.84% | 0.30 -> 0.30% | +1.3 / +1.1 |
   | 72 | 1069 | 3 | 0.23 -> 0.21% | 0.59 -> 0.64% | 0.34 -> 0.36% | -2.8 / -3.2 |
   | 73 | 769 | 1 | 0.32 -> 0.33% | 0.24 -> 0.23% | 0.14 -> 0.31% | -6.8 / -0.4 |
   | 76 | 933 | 2 | 0.04 -> 0.04% | 0.00 -> 0.00% | 0.12 -> 0.12% | -0.2 / -1.8 |
   | 77 | 788 | 1 | 1.28 -> 1.26% | 1.88 -> 1.89% | 3.28 -> 3.26% | -1.6 / -1.3 |
   | 82 | 355 | 1 | 0.06 -> 0.06% | 0.10 -> 0.09% | 0.01 -> 0.01% | +1.2 / +0.7 |
   | 83 | 550 | 1 | 19.17 -> 20.77% | 17.16 -> 16.78% | 10.11 -> 10.46% | -0.2 / -1.6 |
   | 84 | 307 | 1 | 5.41 -> 5.48% | 3.99 -> 4.01% | 2.45 -> 2.49% | -0.3 / +0.5 |
   | 85 | 71 | 1 | 0.64 -> 0.56% | 0.43 -> 0.44% | 0.43 -> 0.38% | 0.0 / -3.2 |
   | 86 | 236 | 3 | 11.41 -> 11.43% | 8.57 -> 8.77% | 1.92 -> 1.85% | +0.2 / -0.5 |
   | 87 | 380 | 0 | 0.05 -> 0.05% | 0.17 -> 0.17% | 0.12 -> 0.12% | 0 / 0 |
   | 88 | 50 | 1 | 0.13 -> 0.14% | 0.27 -> 0.25% | 0.18 -> 0.18% | -0.3 / -0.4 |
   | 89 | 94 | 0 | 0.40 -> 0.40% | 0.46 -> 0.46% | 0.62 -> 0.62% | 0 / 0 |
   | 90 | 96 | 1 | 0.26 -> 0.28% | 0.62 -> 0.63% | 0.19 -> 0.19% | -0.3 / +0.2 |
   | 91 | 79 | 0 | 1.22 -> 1.22% | 1.94 -> 1.94% | 1.92 -> 1.92% | 0 / 0 |
   | 92 | 63 | 2 | 8.08 -> 8.14% | 10.41 -> 10.18% | 10.73 -> 10.81% | -0.3 / -0.8 |
   | 93 | 92 | 1 | 14.90 -> 14.96% | 14.78 -> 14.70% | 6.12 -> 6.07% | +0.3 / -0.3 |
   | Mean | | | 3.68 -> 3.77% | 3.61 -> 3.58% | 2.25 -> 2.28% | -0.6 / -0.8 |

   - By placement, the refine moved the maps nearer the true floor on 11 of the 15 nights it changed, in each night
     column. The mean was 0.6 mm (night 1) and 0.8 mm (night 2), at most 6.8 mm (house 73, night 1). That is a gain,
     but it is a millimetre or so, the size of the views' moves.
   - The floor map error and the nights' disagreement are unchanged within their noise: 3.68 -> 3.77% on night 1,
     3.61 -> 3.58% on night 2, and 2.25 -> 2.28% for the open floor's disagreement. House 83's night 1 got worse
     (+1.6 points) where its placement got 0.2 mm better.
   - Agreement without truth can still pull the better night off. The pass test's pictures (the views' mean per cell)
     do not predict the built pictures: on house 73 they agreed better while the built ones agreed worse
     (0.14 -> 0.31% of the open floor).
   - What it would take to do more: a reference that leaves out every view sharing the error (a whole stretch, or
     the other night only), and lidar rows that keep the lidar's own measurement rather than the shape as solved.
     The pull the locks share on house 70, along each camera's view, fits a camera-side error in pitch: the head
     camera's half-pixel read (decision [AKY]), and a further offset in pitch it does not explain.

18. [myl] **The house frame's limits, for [Tu2]** (4 Oct 2026; hard-won). `sim/house/frame_gate.py`, run by
   `stitch_baselines.sh` through the build helper right after the shape solve, and again whenever a stitch is reused,
   measures each night's correction (house <- odometry) at every keyframe as the turn it makes and how far it moves
   the robot from its odometry position (the move at the robot, not the correction's raw translation, which grows
   with distance from the origin). Refused past 5 deg or 0.3 m at the dock (the night's first keyframe), or past
   15 deg or 1.0 m anywhere over the night.
   - Source: the shape corrections on disk of the tuning houses (70, 71, 73, 76, 77, 82-93) with 78, 95 and 96, read
     4 Oct 2026, every one a working map: at the dock up to 3.29 deg (88, night 2) and 0.061 m (83, night 2); over the
     night up to 10.46 deg and 0.627 m (both 83, night 1). The limits sit 1.4-5x above those. House 72's 3 Oct
     stitch, which broke: 89.85 deg and 3.78 m at night 2's dock, 94.7 deg and 15.0 m over night 1. The first guess,
     about 5 deg and 0.3 m everywhere, holds at the dock but would refuse eight working houses over the night.
   - Hard-won, what it caught: house 72's frame turned about -90 deg, and the live tidy, which places tonight by
     odometry alone, compared tonight's floor with a picture turned 90 deg; nothing failed until the tidy's finds did.
   - The cause, false loop closures in DimOS's pose graph, which accepts a loop on ICP fitness alone
     (`loop_score_thresh` 0.3, DimOS's default). On house 72's night 1, as the robot came back to its dock
     (t+237-247 s), three loops to the night's first keyframes were accepted at fitness 0.07-0.16, asking the current
     keyframe to turn 46-140 deg and move 3-13 m; the pose graph settled about -50 deg off and the shape solve took it
     to -90. The night's other loops fitted at 0.04 or better and asked at most 6.4 deg; house 96's 109 loops fitted
     at 0.024 or better. Not the lidar's stamps: during the dock spin, house 72's night-1 scans sat +1.7 ms (median;
     13 ms median gap) from the odometry pose-fill paired them with, and odometry ran 28 ms behind capture; house 96's
     -2.2 ms, 9 ms and 28 ms. A stamp offset cannot turn the map anyway: the pose graph lifts each world-frame scan to
     the body with the same pose it later places it by.
   - House 72 was stitched with `loop_score_thresh=0.05` (its run's `frame_pgo.args`, kept in the stitch's stamp)
     from 4 Oct until verified loops ([fxk]) made it unnecessary on 11 Oct: unverified, at the defaults its re-stitch
     reproduces the broken frame. At 0.05 (4 Oct): 4.3 deg and 0.53 m over night 1,
     0.15 deg and 3 mm at night 2's dock, 7.0 deg and 0.62 m over it; floor map error ([nv5]) 0.23% of 84 m2 on
     night 1 and 0.59% of 75 m2 on night 2, against 3.30% of 35 m2 and 0.24% of 36 m2 broken. The broken stitch is
     kept beside it (`broken_20261003`). Not for every house: houses 76 and 77 accept true loops at 0.051 and 0.070
     (asking 1-2 deg), and house 71's graph accepts loops asking 18-171 deg at fitness as low as 0.001-0.027, which
     it survives today and a fitness bound would not catch.
   - The fix for the class is decision [fxk]: every proposed loop verified by the robot's own scans before it enters
     the graph, through a project-agnostic DimOS extension point (dimos-fork.md, rule 6). (A bound on the correction a
     loop asks, proposed here on 4 Oct, was not built: house 71's false loops asked 18-171 deg and a bound loose
     enough for true loops would not have caught the rest.)

19. [2Ke] **The floor constraints' and the lidar shape's solves (pose graph optimizations) keep their reweighted
   lsqr by default; a true robust loss on `scipy.optimize.least_squares` or a GTSAM factor graph is selectable, and
   neither is adopted** (4 Oct 2026; hard-won). Both solves are linear in their small corrections, each row whitened
   by its measurement's information, and are built once into one sparse system (`sim/house/robust_ls.py`).
   - `--solver irls` (default) is the old path, unchanged (house 90's shape reproduced to 1e-10): three or four lsqr
     passes, each pair's rows scaled by min(1, 2 cm / its disagreement), so its squared residual by that weight
     squared, a redescending loss rather than Huber.
   - `--solver ls`: one `least_squares` solve (trf, lsmr, the exact sparse Jacobian) with Huber (or soft_l1) on the
     pairs' rows only. `--solver gtsam`: the same system as a GTSAM factor graph (each pair one factor, Huber or
     Cauchy on its whole residual, Levenberg-Marquardt), with each keyframe's or view's marginal covariance written
     to `marginals.npz`. GTSAM comes with DimOS's venv, so it adds no dependency. Both take their scale as 1.345 x the
     pairs' noise (1.4826 x their median absolute residual), measured after three Huber-weighted direct solves.
     Measured from the plain answer alone, the outliers' smear inflates it, 4.6x on a test graph.
   - Hard-won: `least_squares` does not converge on these systems. With lsmr's defaults it stops 18 cm from its own
     optimum after 200 evaluations (house 90's shape, first pass). With tight lsmr tolerances it is still 3.5-4 mm
     short after 1,000 evaluations. `tr_solver="exact"` converges but took 979 s. GTSAM reaches the optimum (1e-5 m)
     but is 4-10x slower than the old path through Python factors (house 90's shape 5.9 s against 0.4 s; house 70's
     floor constraints 18 s against 1.7 s, same machine, back to back).
   - A/B (4 Oct 2026; 18 tuning houses, 70-77 and 82-93, not 78; the solve and the pictures re-run on the recorded
     nights, pictures as `stitch_baselines.sh` paints them; gtsam on all 18, ls on 70, 77, 90, 92 and 93). Per house,
     gtsam minus irls on the shape solve, median: floor map error +0.00 / +0.01 points (nights 1 / 2), the nights'
     disagreement +0.01 points (worse on 13 of 18), the path's median error against the true path 0.00 cm, |scale|
     -0.02%. Houses 92 and 93 keep their 0.8-1.0% scale under every solver, so that scale does not come from the loss.
     The floor-constraint solve (composed on the shape) does gain: its path is 0.06-0.11 cm closer to the truth on
     9-10 of 11 houses, but its map error does not move (median 0.00 / -0.01 points), and no picture uses it by
     default. No lift, so no default changes. The marginals show the floor-constraint solve's views without a locked
     pair held 1.5-2.5x more loosely than locked ones (house 70: 26 mm against 10 mm sd).

20. [AKY] **Image coordinates follow OpenCV's convention: integers at pixel centres** (4 Oct 2026, for [FHA];
   conventional, and hard-won). Every K that projects a floor point onto a frame takes it: cv2's straightening of
   the fisheye, ROS's camera_info, the wrist camera's and the pinhole head camera's centred renders (their centre at
   w / 2 - 0.5). `floor_sample.sample` reads (u, v) in it at every pyramid level.
   - Hard-won: the sampler read (u, v) as corner-based (u - 0.5) while the head camera's K is OpenCV's, so every
     head-camera cell sat half a pixel up and left (the pipeline critique of 3 Oct, finding 14). At 640 x 360,
     fy 399, that is 1.25 mrad (0.07 deg); along the view it grows with the square of the range, to centimetres at
     2-2.5 m. A unit test projecting rays through the real lens read -0.50 px on both axes at every level before,
     and within 0.02 px after.
   - Positive control first (house 70, the bench's builder, the robot's mount and stored lag): reading one whole
     pixel down and right moved night 1's floor map error 2.09 -> 1.79% and night 2's 2.39 -> 2.03%. Placement
     against the true floor came 9.4 and 9.9 mm nearer (paired). One pixel down alone gave 8.2 / 8.8 mm, one pixel
     right alone 0.8 / 0.9 mm. Nearly all the effect is along the view (pitch).
   - The fix, on the eighteen tuning houses (night 1 and 2 pictures rebuilt with each house's stitch arguments,
     scored with `floor_fidelity.py --error` and `floor_fidelity.placement`):

   | House | Error n1 | Error n2 | Disagree, open | Disagree, all | Placement n1 / n2 (mm) |
   |---|---|---|---|---|---|
   | 70 | 2.09 -> 1.92% | 2.39 -> 2.16% | 1.58 -> 1.53% | 2.04 -> 1.98% | -5.0 / -5.6 |
   | 71 | 0.47 -> 0.50% | 0.96 -> 1.03% | 0.30 -> 0.29% | 0.28 -> 0.26% | -0.2 / -2.8 |
   | 72 | 0.23 -> 0.24% | 0.59 -> 0.60% | 0.34 -> 0.35% | 0.60 -> 0.60% | -3.7 / -2.7 |
   | 73 | 0.32 -> 0.62% | 0.24 -> 0.13% | 0.14 -> 0.12% | 0.37 -> 0.36% | -2.5 / +1.4 |
   | 76 | 0.04 -> 0.04% | 0.00 -> 0.00% | 0.12 -> 0.10% | 0.14 -> 0.12% | -3.6 / -4.0 |
   | 77 | 1.28 -> 1.27% | 1.88 -> 1.85% | 3.28 -> 3.26% | 3.51 -> 3.53% | -4.6 / -5.9 |
   | 82 | 0.06 -> 0.06% | 0.10 -> 0.10% | 0.01 -> 0.01% | 0.02 -> 0.02% | -0.2 / -0.7 |
   | 83 | 19.17 -> 19.58% | 17.16 -> 16.45% | 10.11 -> 10.07% | 9.13 -> 9.07% | 0.0 / -1.3 |
   | 84 | 5.41 -> 5.38% | 3.99 -> 3.86% | 2.45 -> 2.47% | 2.69 -> 2.70% | -0.2 / +0.2 |
   | 85 | 0.64 -> 0.66% | 0.43 -> 0.45% | 0.43 -> 0.37% | 0.37 -> 0.32% | +0.7 / +0.2 |
   | 86 | 11.41 -> 11.63% | 8.57 -> 8.67% | 1.92 -> 1.80% | 1.80 -> 1.70% | -1.4 / -0.3 |
   | 87 | 0.05 -> 0.05% | 0.17 -> 0.16% | 0.12 -> 0.10% | 0.12 -> 0.09% | -0.3 / -0.4 |
   | 88 | 0.13 -> 0.13% | 0.27 -> 0.24% | 0.18 -> 0.18% | 0.16 -> 0.17% | -1.5 / -3.4 |
   | 89 | 0.40 -> 0.37% | 0.46 -> 0.45% | 0.62 -> 0.59% | 0.56 -> 0.54% | -2.9 / -1.6 |
   | 90 | 0.26 -> 0.28% | 0.62 -> 0.62% | 0.19 -> 0.20% | 0.17 -> 0.18% | -2.9 / +2.0 |
   | 91 | 1.22 -> 1.22% | 1.94 -> 1.88% | 1.92 -> 1.91% | 2.27 -> 2.22% | 0.0 / -0.5 |
   | 92 | 8.08 -> 8.15% | 10.41 -> 10.25% | 10.73 -> 10.85% | 9.69 -> 9.80% | -0.3 / -0.4 |
   | 93 | 14.90 -> 14.62% | 14.78 -> 14.42% | 6.12 -> 5.77% | 5.57 -> 5.25% | -1.6 / -2.9 |
   | Mean | 3.68 -> 3.71% | 3.61 -> 3.52% | 2.25 -> 2.22% | 2.19 -> 2.16% | -1.7 / -1.6 |

   - Kept. The maps sit nearer the true floor on 16 of 18 houses on night 1 and 14 of 18 on night 2. The nights agree
     better on 11 houses and worse on 3-4. Night 2's error falls on 9 houses. Night 1's mean rises by 0.03 points,
     mostly house 73 (0.32 -> 0.62% on 120 matched patches, while its placement came 2.5 mm nearer) and 83.
   - House 70 still prefers about one more half pixel down. That is a further offset in pitch, likely the robot's
     mount (`data/robot/head_mount.json`), whose stitch solves were painted with the old read.
   - Not yet rebuilt: every run's baseline pictures (`mosaic_n<k>.npz`) and views (`views_n<k>.npz`, painted by the
     confirmer) were read the old way, and so was the robot's mount, calibrated from those views. The stitch's stamp
     (`stitch_baselines.sh`, v7) does not yet name the sampler, so it will not rebuild them itself. Tonight's
     confirmer now reads frames the new way against baselines read the old way: within the detector's 2 cm slack,
     but not alike.
   - A pinhole recording from DimOS's own camera_info (`HK_FISHEYE=0`, cx at w / 2) is read half a pixel the other
     way now. Every tuning recording is fisheye.

21. [yFg] **The head camera's floor geometry is calibrated, not assumed** (the proposal of
    the 6 Oct camera floor geometry study). Under [gJq]:
    - The mount carries the camera's height as well as its turn (`up_m`), solved with pitch and yaw by the floor
      self-calibration over many houses' views (`camgeo.py selfcal`, `propose`: the median over houses with enough
      floor), and checked against the record past 0.1 deg of pitch, 0.3 deg of yaw (houses differ by 0.12-0.25 deg, 7 Oct) or 5 mm.
    - The floor's rows have their own frame lag: the stored lag less the self-calibration's offset (a rolling-shutter
      camera reads the floor last), per clock, written beside the stored lag.
    - The floor's relief is part of each house's map, solved with the camera at the robot's calibration, and every
      painter lays the floor on it rather than on z = 0.
    Adopted 7 Oct 2026 as follows:
    - **One writer.** `camgeo.py calibrate` writes the mount and the floor rows' offset into `data/robot/`, from
      tuning houses' self-calibrations only (hold-outs and the validation house 78 refused before anything is read).
      It copies both files to dated files beside them first, and records in each the previous values, when, and from
      what. The floor rows' offset is its own key in `frame_lag.json` (`floor_rows_offset_s`), so the stored lag
      stays the frame's (the turning estimator's, and the per-night lag check's); `calibrate_lag.py` keeps it.
    - **The relief is solved in the stitch** (`stitch_baselines.sh`, v8): the views painted in the house's frame with
      the robot's calibration, the relief solved on them with that calibration held (`floor_selfcal.py relief`,
      three rounds), then every picture and view set painted on it. It is stored in the run directory
      (`relief.npy`, with `relief.json` naming its frame), and a painter takes it by default when it paints in that
      frame; a painter that finds none, or paints in another frame, lays the floor on z = 0 and says so.
    - **The nightly check** (`floor_selfcal.py check`, in the same stitch): the geometry solved free from the views
      is the night's disagreement with the record, warned past 0.1 deg, 5 mm or 10 ms (the lag only where the views
      are close enough in time to see it), with too little floor (under 2,000 tile registrations) said and not
      judged. It writes `calibration_check.json` in the run and the night's folder, never the robot's calibration.
    Measured 6 Oct (sim, 17 tuning houses): floor locks between views facing apart 97 -> 32 mm (median), the chain's
    path 31.0 -> 24.0 mm, the joint graph 16.0 -> 15.6 mm; at one false flag a night on fresh nights 18-19 toys found
    against 11. Kalibr ([fxE]) stays the record for what the floor cannot see (the lens, the forward and sideways place,
    roll, the camera-to-IMU offset); the floor self-calibration is the nightly check ([9tQ]). On adoption (7 Oct) the
    robot's mount moved from pitch -0.802 deg, yaw 0.832 deg to -0.623 deg, 0.085 deg and 2.4 mm higher; the tuning
    houses' floor maps were rebuilt on it and their numbers held.

22. [fxk] **Loop closures are verified before they enter the house's frame, and house 72's own loop setting is retired**
   (11 Oct 2026, for [dmT] and [Tu2]; hard-won). Adopted on the 8 Oct finding and its proposal
   (recommendations 1 and 4) under [eM8] (build-or-depend [eK3]), whose sign-off is needed only for a new
   dependency or a change of direction; this is neither: it uses
   DimOS's own pose graph and code already in the repository.
   - **How.** DimOS's pose graph accepts a loop on ICP fitness alone. The fork's project-agnostic extension point
     (`PGO(loop_verifier=...)`, fork rule 6) hands each proposed loop to `sim/house/loop_verify.py` before it is added:
     house_shape's keyframes around the loop's two keyframes, placed by odometry, laid on each other at the proposed
     alignment and matched again; accepted when at least 60% of the matched pairs need under 5 cm and 2 deg more. No
     evidence refuses (the later solves' scan pairs tie revisits anyway). `house_frame.py --verify lidar` is the
     default, so the stitch (`stitch_baselines.sh` v9) and everything built in its frame use it; the floor views'
     fallback (`--verify either`) is the bench's, since the stitch has no views in a frame before the frame exists.
     Every verdict is kept in the frame's `verify.json`, the count refused in its `summary.json`.
   - **Stale maps.** The stitch's stamp names its map version (`v9 ... loops=verified`); `houndkeeper.calibrate`'s
     `MAP_VERSION` is the same, and the live tidy and the confirmer refuse a run directory whose stitch is older
     ([dmT]); a stitch reused under an older stamp is made again.
   - **One thread for DimOS's pose graph, in the stitch and in benches** (`OMP_NUM_THREADS=1`). Hard-won (house 71, 8
     Oct): Open3D's multithreaded ICP sums in a different order each run, so the same recordings gave loop fitness
     values differing in the 8th digit and graphs 46-61 mm (once 626 mm) from the true path; with one thread the run
     repeats bit for bit. With the verifier the multithreaded runs agree too (40.2-40.9 mm), but a bench comparison is
     never left to chance.
   - **House 72's `loop_score_thresh=0.05` ([myl]) retired.** At DimOS's default acceptance the unverified graph took
     34 gross false loops and the joint graph sat 313 mm from the true path; verified, 23.1 mm, the same as the hand
     setting's 23.4 (8 Oct). Re-stitched at the defaults with the verifier (11 Oct): 101 loops kept, 48 refused; the
     frame gate passed, night 1 at most 4.24 deg and 0.52 m from its odometry, night 2 7.00 deg and 0.64 m (the hand
     setting's stitch: 4.24 deg and 0.53 m, 7.01 deg and 0.62 m). Its `frame_pgo.args` is kept in the run's archive
     (`archive/20261011_105011`); the mechanism for a house's own settings stays.
   - Evidence (8 Oct, 18 tuning houses, one thread): of 2,679 loops DimOS proposed, 667 were wrong by the sim's truth
     (5 cm or 2 deg; 23 gross, 30 cm or 10 deg); the verifier refused 400 wrong ones, 21 of the 23 gross, and 33 of
     2,012 true ones (1.6%, 14 on house 76); 4 of 4 planted false loops refused. Every arm a little better over 17
     houses (DimOS's graph 48.9 -> 46.3 mm, shape solve 15.8 -> 15.5, joint graph 15.6 -> 15.3). Cost: 2-5 min a
     house in the stitch.
   - Adopted and measured (11 Oct): the 19 tuning runs and 6 detection-bench
     copies rebuilt from their recordings (archived first), every frame gate passed; 3,521 loops proposed, 632
     refused; a bench copy and its run gave bit-identical frames. Over 17 tuning houses against 7 Oct: shape solve
     15.8 -> 15.5 mm, chain 23.9 -> 20.0, joint graph 15.6 -> 15.2; floor map error ([nv5]) 3.67 -> 3.63% (mean of 38
     nights); detection's first stage on fresh nights 34/40 at 8.0 false positives a night, AP 0.685 (was 35/40, 0.654).

23. [UpK] **Over the night, the house frame's move limit grows with distance from the dock** (13 Oct 2026, for [Tu2];
   hard-won). A keyframe's correction may move the robot up to max(1.0 m, 0.3 m + 0.08 x its distance from the dock
   by odometry) from where odometry put it. The turn limits (15 deg over the night, 5 deg at the dock) and the 0.3 m
   at the dock are unchanged from [myl]. The gate's line names, for each night, the keyframe nearest its move limit
   and the limit there.
   - Why: odometry's heading drift moves the robot in proportion to how far it has gone, so a flat 1.0 m refuses a
     right frame on a long night. House 76's 1280 re-sweep (12 Oct) was refused: night 2's frame moved 1.02 m from
     odometry. The frame was right: 3 cm median and 11 cm worst against the sim's truth; odometry itself was 1.10 m
     off after a 20 m run, with an ordinary 4-5.7 deg of heading drift; no false loop closures.
   - Trial (13 Oct, `../build/diag76/gate_trial/trial.py` and `sweep.py`, outside the repository, on the shape solves
     on disk): all 106 archived tuning-house nights pass (the closest, house
     71's night 2, 0.25 m under its limit); house 76's 1280 night 2 passes with 0.31 m to spare. Night 2's frame,
     turned about the dock from keyframe 300 on by -12 to +12 deg, is refused exactly where the flat 1.0 m refused
     it (-12, -10, -8, -6, -4, +10 and +12 deg). A steeper slope (0.10-0.15 m a metre) lets 10-12 deg through, so
     0.08 it is. House 72's broken stitch is still refused on every count.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [hJ5] (on DimOS) | `sim/dimos/tidy_run.sh` with `houndkeeper.tidy` | Partial: the loop runs on DimOS; items are found and picked some of the time (overnight log) |
| [S7o] (one head-on stop per find) | `sim/dimos/tidy_run.sh`: each find approached head-on and parked with it at `cluster_stops.front_offset` | Partial: head-on approach and re-park on DimOS; parking 0.7-7.6 cm off (24 Sep); not yet run with the 25 Sep target. (The earlier test, fewest stops by grouping, went with grouping to the backlog.) |
| [JKS] (each find looked at, named, handled) | slice run | Partial: on DimOS each flag is looked at, named by the open-vocabulary detector, and picked or sent to review |
| [bOu] (full run report) | slice results; `tests/test_try_log.py` (each grasp try's own outcome, width, lean and weight) | Partial |
| [5AP] (live HUD) | viewer HUD: standalone slice; on DimOS's viewer through `sim_hook.viewer` (`DIMOS_SIM_VIEWER_HOOK`) | Passing: standalone slice; on DimOS since 25 Sep (state, power, battery, latest step; the wrist camera inset is standalone only) |
| [bXX] (labelled video) | slice video | Passing: standalone slice |
| [XfI] (the same object never handled twice) | `tests/test_same_as.py` (the rule, including never the same as itself on a re-park); DimOS night in the house with two flags on one object: one pick attempt, the second stop recorded as the same item | Partial: rule test passing; night check not yet written; seen on 25 Sep (the shoe, flags 69 cm apart) |
| [d8d] (finds checked during the sweep, rising cost) | DimOS night in the house: items binned, stops for nothing, night length and energy, against sweep-then-visit on the same house and items | Not yet written |
| [tmq] (feet never cross a known spot) | DimOS night: the sim's truth log shows no item moved except by the arm, with the keep-out published to DimOS's costmap (`TidyArm._publish_keepout`, the fork's `CostMapper._apply_keepout`) | Not yet written: the lion was kicked 38 cm on 25 Sep, before the keep-out |
| [82p] (placed by its middle) | `tests/test_whole_item.py` (a lying toy's middle, not its near end); a DimOS night with `HK_WHOLE_ITEM=1` (round 12) | Partial: rule test only; not yet run on a night |
| [FHA] (precise floor map) | `sim/house/check_funnel_top.py` (the floor map and walk-through requirements; `bench_mapping.py` for the standalone bench; `bench_detect.py` on the realistic bench `data/bench/real1`, its stitched variant built by `sim/dimos/stitch_baselines.sh`) | Failing: with the world's imperfections on, the floor map misses (edges 16-25 mm median on the drawn seams, 27-28 Sep, whatever the placement tried; but measured over the whole map against the true floor (`floor_truth2.py`), the stitched pictures' own warping is 14-31 mm against odometry's 54-61 and the true path's 14); the walk-through, re-recorded realistic (real1, 24 toys) and scored where the robot would walk to, reaches 50 candidates near no toy a night with the baselines stitched and the floor tracked, at 75% of toys (the mount calibrated from odometry; 92% from the sim's true path) against 93% required ([WFI]). Held on the perfect sim on 27 Sep (3.3-3.7 mm, 12-13 p95; 98% and 40.5) |
| [nv5] (floor map error, above detection) | `tests/test_floor_map_error.py`; measured by `sim/house/floor_fidelity.py --error`, logged to `data/eval/floor_map_error.jsonl`, run by `check_funnel_top.py` | Partial (the measure); 1 Oct, night 1 of the six houses of `data/bench/houses6c`: as built then (odometry) 0.27-7.46% (mean 3.4%: houses 76 0.27, 73 2.05, 72 2.41, 71 2.65, 70 5.29, 77 7.46); placed by the pose graph over both nights' lidar with 15 views a cell 0.06-3.51% (mean 1.5%: 76 0.06, 73 0.45, 72 0.61, 71 0.95, 77 3.49, 70 3.51). The true path's map is not yet a clean floor for the measure (its line-of-sight heights sit in odometry's frame: 3.1% on house 73). Tried against the baked-in light on houses 70 and 77 (1 Oct), none a win: each frame's own light divided out (no change); only steeper views (1.4-1.8 m, not 2.5) agree better night to night (70: 1.8 -> 1.1%; 77: 3.2 -> 2.4%) but no closer to the actual floor (2.3-3.8% over less floor); the darkest tenth of 15-30 views (2.0-2.7%); tests (3 Oct 2026): `tests/test_floor_mutants.py` (exclusions, wall band, path fit) and `tests/test_floor_map_error_frame_and_log.py` (the night's corrections applied before the rigid fit; a turned map laid back by its path; a bent map counted; a map already in the true frame judged where it lies; `--error` appends a dated line with how the map was built; check_funnel_top labels each bench house); the map's placement against the true floor (`floor_fidelity.placement`, 4 Oct 2026): a shift the error forgives read to a millimetre or two, and a floor that is not the truth gives (almost) no placement (`tests/test_floor_mutants.py`); still not covered: nothing makes the true-path reference each night (it was measured once by hand, house 73, 1 Oct), and the numbers above come from runs, not tests |
| [d8d] (the walk-through detector's blobs, unit) | `tests/test_detect_changes2_blobs.py` (detect_changes2.main on synthetic nights: a toy put down is "present" and one taken away "gone"; a speck both tiny and faint is dropped, one only tiny or only faint kept; beads 3 cm apart joined by --link-px; the same blobs with the truth files gone, matched only by score_detect2) | Partial (3 Oct 2026): the detector's blob logic only; nothing of [d8d]'s checks during the sweep. A darker-only find (the brightening clipped at 0 in "here") is not pinned |
| [63R] (every subsystem's health, in order) | `sim/house/subsystem_health.py` builds it from a bench's records (`data/eval/subsystems_<bench>.json`, `subsystems.html`) | Written: first built 2 Oct on `houses6r`; the floor snap failing (22 of 4,770 frames locked) and the close look (59%, hold-out). The weighted snap (`--snap-mode weighted`) found the same: single night frames against the composite floor give no steady peak (41% on the edge of a +-20 cm search), while tonight's frames land 2-9 cm (median) from the floor map's frame against the truth; looser lidar priors don't move that (4.7 cm either way) |
| [evl] (a floor picture can average each spot's views) | `tests/test_floor_mosaic_trimmed.py` (a trimmed mean over synthetic views removes one view's speckle and a glint seen in one view); the default picture rebuilt byte for byte the same as before; A/B on the eighteen tuning houses' recordings | Partial: the tests; on the eighteen tuning houses (3 Oct) the default picture rebuilt byte for byte the same. An option only, not the default; not covered (test audit, 3 Oct 2026): only the trimmed_mean helper: the --combine wiring and "no change without it" (a manual comparison) are untested |
| [SdA] (a failed step stops the night; nothing earlier stands in) | `tests/test_build_helper.py` (the build helper: each output removed first and required again, each background build waited on by its PID; and the gate: no bare `wait` or python step piped into `tail` in the night pipeline's scripts, which all set `pipefail`); `tests/test_bench_fails_loudly.py` (the replay bench stops on a confirmer crash, a missing result or a placeholder its night lacks) | Written: passing on synthetic builds (3 Oct); not yet run through a night (no sims run with it) |
| [Tu2] (a house frame far from its own odometry is refused) | `tests/test_frame_gate.py` (synthetic corrections: identity, drift within the limits, a turned dock, a dock offset, a night that folds mid-way, a 90-deg frame; a house's own pose-graph settings used and stamped; the measure is the move at the robot, not the raw translation; the stitch runs the gate through the build helper and stops; the tuning houses' shape corrections on disk all pass and house 72's broken 3 Oct stitch is refused; decision [UpK]: a long night legitimately drifted past 1 m far from the dock passes, the same night turned about the dock is still refused) | Written: passing (4 Oct 2026: the 18 tuning houses with 78, 95 and 96 pass; house 72's broken 3 Oct stitch refused, 89.9 deg and 3.78 m at night 2's dock; its 4 Oct re-stitch passes, 7.0 deg and 0.62 m at most) |
| [dmT] (a loop closure enters the house's frame only when the robot's own sensors agree; an older map refused; decision [fxk]) | `tests/test_loops_verified.py` (the stitch runs DimOS's pose graph with the verifier and one thread, its verdicts a required output; house_frame.py verifies by default and hands DimOS the verifier; the pose-graph bench verifies and runs one thread by default; the stitch's stamp carries the map version the readers expect; a v8 map, or a stitched house with no stamp, is stale, a house with no map yet is not; the confirmer refuses an older map unless `--stale-map`, the live tidy stops on one); `tests/test_loop_verify.py` (a true loop accepted; loops 15 cm, 6 deg, or 0.6 m and 15 deg wrong refused; no evidence refused) | Written: passing (11 Oct 2026) |
| [YuI] (too little of the night is not a baseline) | `tests/test_baseline_coverage.py` (decision [NvG]'s limits against the pictures on disk, 2 Oct) | Written: passing (3 Oct) |
| [gJq] (the frame lag and mount are the robot's calibration) | `tests/test_robot_calibration.py` (a night with no turning, a sign-flipped one, one on the search edge or from too few pairs leaves the robot's lag unchanged; the median over passing nights; hold-out seeds refused by both calibration steps; a picture saves the head camera's lag, not the wrist's; the per-night check warns past 40 ms; static: `stitch_baselines.sh` never writes the robot's mount, its stamp carries the hashes, `floor_mosaic2.py` defaults to the stored lag; the confirmer's replay and `calibrate_mount.py` default to it too, the replay placing each camera's frames with that camera's own lag); `tests/test_lag_per_clock.py` ([55G]: each clock's lag loaded, a clock the file lacks refused (load and the cameras' lags), one clock stored without losing the others; a recording's clock from its path file in both forms and from the run that lists it, its true path scrambled with no change, an unlisted recording refused; house 96's and 93's own files told apart; the widened gate keeps -0.03 s and still refuses house 93's -0.25 s and thin turning; the calibration refuses a recording on another clock; each tool that places frames, the stitch, the night picture, real nights and the live tidy take the robot clock's lag for a robot-clock recording) | Written: passing (3 Oct 2026) |
| [gJq] (the frame lag and mount are the robot's calibration) | calibration over the tuning houses' baselines (`calibrate_lag.py --clock wall` over pre-[3Pv] recordings; `--clock robot` over robot-clock ones), then a night's pictures rebuilt from the stored lag | Partial (3 Oct 2026): both clocks calibrated (wall over 18 houses; robot over house 96's two nights and its walk, decision [55G]); no night's pictures rebuilt from the robot clock's lag yet, and the robot clock rests on one house. 7 Oct 2026: every tuning house's pictures and views rebuilt from the stored lags and the adopted mount ([yFg]; house 96's on the robot clock's lag), the robot clock still resting on one house |
| [gJq] (the frame lag and mount are the robot's calibration; decision [yFg]) | `tests/test_floor_calibration.py` (the one writer refuses hold-outs and the validation house, writing nothing; backs both files up to dated files, records the previous values, when and from what, keeps the frame's lag and writes the floor rows' offset beside it, and never applies the same solves twice; views painted with another calibration refused; the floor laid with the floor rows' lag, the per-night lag check against the frame's own, the offset kept by the lag's recalibration; every painter's default the house's relief in its frame, the plane said without one, in another frame or in odometry's; the nightly check ([9tQ]) warns past 0.1 deg, 5 mm or 10 ms, does not judge too little floor or an unseen lag, and writes nothing of the robot's) | Written: passing (7 Oct 2026) |
| [Q9k] (never compared with a floor painted under another calibration) | `tests/test_floor_calibration.py` (pictures and views carry their mount, lag and relief; one made before the stamps or under another is named; the confirmer refuses it unless told to compare across, the live tidy stops) | Written: passing (7 Oct 2026) |
| [G9q] (each view against the floor) | `tests/test_floor_refine.py` (each view's reference leaves it and its own night's neighbours out; registration recovers a known shift and turn; the lidar keeps a straight wall straight under a 4 cm pull that bends it without the lidar; no evidence moves nothing; on 30 cm tiles the bound stops a one-tile jump that an unbounded search makes; a view at its bound cannot pass it; a worse pass is rejected and refining stops; the pass test is `floor_agreement`'s share without truth; the bench's plant moves only its block, by the shift and turn given); per house: floor map error, placement against the true floor (paired) and the nights' disagreement, the pairwise floor constraints alone against them plus the refine passes, `sim/house/floor_refine_bench.py` (its `--plant` the positive control) | Measured (4 Oct 2026, re-measured after the audit): unit tests passing; the floor map error cannot see the refine's moves (a 5 cm, 1 deg plant on 15% of night 1 read 2.09 -> 2.10%); by placement the maps came 0.6 / 0.8 mm nearer the true floor (mean, nights 1 / 2), nearer on 11 of 15 changed nights each; the error 3.68 -> 3.77% and 3.61 -> 3.58%; the refine took back 1% of a planted error a stretch of views shares (decision [qLu]) |
| [FHA] (the floor constraints' and the lidar shape's selectable robust solves, decision [2Ke]: a true robust loss on the pairs only, its scale measured, outliers held off, GTSAM's answer the converged Huber's, the old pass unchanged) | `tests/test_robust_ls.py` | Passing |
| [FHA] (head and wrist camera pixels read in OpenCV's convention, decision [AKY]) | `tests/test_floor_sample.py` (rays through the real lens, warped and straightened, read back at every pyramid level to 0.05 px; the wrist camera's centred render read at its centre; a one-pixel cell reads its own pixel) | Passing (4 Oct 2026); the 18 tuning houses' pictures rebuilt with it: nearer the true floor on 16 / 14 of 18 nights 1 / 2 (decision [AKY]) |
