# Grasp scenario

**Type:** Capability spec · **Status:** Draft · **Source:** D0 [jX2]; principle [5oU] · **ID:** [8JV]

## Purpose

A fast, repeatable sim test of the arm's pick: the robot lies down once, items appear one at a time within reach,
and each trial runs look closer, grip, lift, carry and drop into the bin. It scores whether a change to perception or
control made picking better or worse.

## Acceptance criteria

1. [JwG] A run takes a trial count and a seed; the same seed and code give the same items, poses and outcome.
2. [udO] Items vary in shape (at least box, flat cloth-like, brick, cylinder), mass (30-300 g), rotation and position
   anywhere in the arm's lying-down reach.
3. [bxP] Each trial starts from a position estimate off by about 5 cm, as Phase 1's detection would give.
4. [OQ9] Each trial reports success or exactly one failure reason: not seen, not lifted, dropped lifting, dropped
   carrying, missed bin, too wide to grip, tipped over.
5. [bfo] Success means the item is resting inside the bin at the end of the trial.
6. [gF0] The run ends with success rate overall and per item kind, and failures by reason, and appends it to the suite
   history.
7. [4ZY] A trial never counts as a success if the robot passed its measured tipping point ([BPL]) during it.
8. [guX] Items include soft ones that deform when gripped: cloth (sock-like) and rope. The simulator support they need
   is recorded under Decisions.
9. [N4z] **The floor under the item is modelled at wrist-camera resolution and changes every trial.** A patch
   covering the arm's reach is made from the trial's seed at about the camera's resolution (under 1 mm per pixel)
   from photographed floors: carpets, wood and tile, each with its surface relief and shine
   (normal and roughness maps) at its real size, turned, flipped and rescaled a little per trial, with stains and
   marks laid on top. Some photographed floors are held out: used only for judging, never for training or tuning
   (as held-out items are). Generated carpets, wood and tile remain available by name. About
   two floors in five have a rug on them, and most of those rugs have an edge within 15 cm of the item at a random
   angle and side, so the wrist camera meets sharp, contrasting edges near what it is looking for. In the classic
   renderer's images carpet pile looks different from each side; under Filament a carpet shows its relief under the
   light instead ([UQK]). Beyond the patch the same floor continues without joins, as far as the cameras see ([CaM]).
10. [ZMc] **Front pickups.** A front run places items only in the front reach zone, where a head-on approach leaves them.
11. [Hwc] **The grasp may lean.** The gripper may lean up to a stated maximum from vertical (30 deg unless changed),
    approaching along its own axis with the jaws closing across the item's narrowest side; each trial records the lean
    it used.
12. [XCl] **Too heavy is put back.** The pick weighs what it has lifted from the arm's joint torques (the same pose
    empty and loaded, the item clear of the floor, each read once the arm has stopped moving) and puts down anything heavier than the arm's rated 1.5 kg where it
    was, instead of carrying it; each trial records the weight it measured.
13. [7Xd] **The arm never presses on what it meets.** On the final approach the fingertips aim above the floor, and a
    rise in any joint's torque stops the descent where it is: an item or the floor met early is gripped from there,
    never pushed into. No grasp tips the robot (principle [5oU]).
14. [2ho] **An item that turns in the jaws still goes in the bin** (things will shift in the
    arm). The carry and the crossing into the bin allow for the item hanging below the jaws by as much as any item
    could (always more vertical clearance when binning: a figure gripped by one leg was
    carried by its measured length and swung into the robot's side), as high as the arm reaches short of that, and the
    item is then lowered until it rests in the bin rather than dropped. Let go, the arm leaves the bin the way it came
    down, straight up, before any other move (28 Sep 2026: a train set down pivoted against the rim and would have
    stayed in had the arm not bumped it again as it stowed).
15. [GU5] **A held item stays put in the jaws.** Between lifting it and letting go over the bin, a held item turns
    less than 30 deg in the jaws (proposed), on the front benchmark's items.
16. [WuR] **The grasp comes from a swappable proposer**: our solution is one implementation of
    gripping in DimOS, so a different solution can be swapped in later, without committing to a point cloud yet. The pick takes its grasps through one interface: given the look-closer's result,
    a proposer returns grasps best first as DimOS's own grasp message (`GraspCandidateArray`), each with the grip width
    and how far the pick may adapt it (lean, height). Houndkeeper's proposer works from the look as it is, with no
    point cloud; DimOS's point-cloud proposers (its heuristic, GraspGenX) plug in through an adapter that builds the
    cloud from the look. The proposer is chosen by configuration, and with Houndkeeper's the pick bins as before the
    interface (the 48-trial bench of waymark [dgF], within run-to-run noise).

17. [oZb] **An item is let go over the bin's middle** (29 Sep 2026: the Reebok shoe, the fire truck and the long dino
    missed the bin or were lost at it in 31 of 240 bench trials; the release pose was 11 cm from the bin's middle and
    a 28 cm shoe let go there lands across the rim). The release point is the reachable one nearest the bin's middle,
    the gripper within 40 deg of straight down and the jaws across the robot, so a long item gripped across its width
    hangs along the bin's long side. Tried 30 Sep 2026 and not adopted: it isn't what makes the shoe miss (the shoe
    turns in the jaws, [GU5]); `HK_BIN_RELEASE` stays `corner`.
## Non-functional requirements

- A trial completes in under 15 s of wall time on the development laptop (RTX 2070), viewer on.

## Failure behavior

- If the wrist camera sees nothing, the trial fails as "not seen"; it never grasps blind.
- If no collision-free path to the bin exists, the trial fails with that reason; the arm does not force its way.

## Decisions

1. [Udt] **Friction uses MuJoCo's elliptic cone in the arm sims.** Hard-won (25 Sep 2026, MuJoCo 3.10.0): with the
   default pyramidal cone, and DimOS's Go1 scene solving contacts in 1 iteration, a held 7 cm ball crept out of the
   jaws at a steady ~3 mm/s from the moment they closed, and fell during the carry (a single-grasp probe).
   30 solver iterations changed nothing (6.1 mm in the first 2 s); the elliptic cone cut it to 2.1 mm and the ball was
   binned. The no-slip pass also stopped it but changed how the lying robot settled, so it is not used. DimOS's
   walking sim keeps its own setting until walking is checked with the elliptic cone.

2. [v8J] **TPU pads on the jaws, and the sim grips with what the pad would**: TPU pads designed for real TPU values and run
   through the sim. Hard-won: a ballet flat gripped within
   2 cm of its centre of mass turned 13-42 deg in the jaws on the way to the bin, and once 172 deg and fell; the jaws
   met it at single points, which resist almost no twist. The pads, their materials, the contact model the sim uses
   and every source are in waymark [wjx] (`docs/waymarks/0006-tpu-jaw-pads.md`): printable TPU stiffness from the
   datasheets [8ME], friction estimated until measured [Iin], a collapsible zigzag-rib face rather than solid TPU
   [2L1], and sim contact values computed from each design, not tuned [WFg]. The sim's jaws are chosen by HK_JAWS
   ("rubber", every result up to 25 Sep, grippier than any real jaw; "bare"; "pads_v1"; "pads_v2"), and every result
   says which.

3. [Nb4] **The floor beyond the patch** (25 Sep 2026: flicker, seams and a rug that read as a seam). A ring of
   patch-sized cells around the patch repeats the floor without its stains, edge to edge, never overlapping (overlapping
   floors flicker). Every floor is generated to repeat seamlessly (wrapped noise; tile, plank and pattern spacings that
   fit the patch a whole number of times). A rug is drawn on a floor, bordered and shadowed, never as the whole patch.
   Rugs larger than about 1 m (the patch is 1.3 m) are not modelled yet. Until 25 Sep the surround was, in most trials,
   a different floor from the patch (asking for a floor by name skipped a random draw and shifted every later one), so
   the patch read as a square of another floor with a straight edge in the wrist camera's view; fixed, and checked on
   30 seeds.

4. [1Na] **Soft bodies are available in the simulator we use.** Observed 2026-09-24 with MuJoCo 3.10.0 (the version
   in the DimOS fork's environment): a model with a `flexcomp` cloth grid and a cable built with the
   `mujoco.elasticity.cable` plugin loads and steps. So soft items need no new simulator, only item models and
   gripper contact tuning.
5. [SVM] **Grasp success so far (sim).** Three 20-trial runs, seed 0, on 2026-09-24, Go1 stand-in ([WD6]), mount
   12 cm forward: 6/20, 9/20 and 10/20 (30-50%). Most failures were "not lifted". The target is [gNg].
6. [xCw] **Grasp scenario, full pick into the bin (sim), 25 Sep 2026.** 40 trials, seed 5, library items dropped in any
   orientation on generated floors, flags ~5 cm off, own perception: pickable items 12/29 binned (41%); items that
   should be left for review 11/11 correctly left (card, wide box, shoe and similar, each with its reason). Losses: not
   seen 6, grip didn't hold 5, lost on the way to the bin 4 (may be in it; the bin isn't checked), no reachable grip
   direction 2, dropped lifting 1. Rope 0/3. The target ([gNg]) is not met.
7. [eeT] **Scripted grasp on realistic items (sim), 25 Sep 2026.** From the look-closer handover, with its own perception
   (OWLv2 boxes + EdgeTAM masks, three fused looks), items from the library on generated floors, held-out seeds 900+:
   19/23 lifted (83%). The rolling lion and ballet flat failed. This is the lift only; binning is not included.

8. [KrW] **The weight is read on the arm's own readings** (28 Sep 2026; principle [Feg]). On DimOS the arm publishes
   one instant per sim frame (35 ms), and the printed frame's flex rings at ~10 Hz ([nAH] z7P); waiting and averaging
   by the wall clock (the sim at 0.28x real time) caught a frame or two mid-swing, and a pony weighed 2.6 kg.
   `DimosRobot.settled_efforts` now waits for 3 new readings with every joint slower than 0.01 rad/s, then averages 12
   new readings (~0.4 s of the arm's time). The standalone grasp bench, which settles by sim steps, weighs within ~15%
   with the imperfect arm (16 trials, 28 Sep: 17 g as 18, 532 g as 462).
9. [0Xr] **The arm reaches 10 cm above the bin's rim, no more** (measured 28 Sep 2026, standalone arm sim, lying,
   the Go1 stand-in's mount; `carry_plan` and `over_bin` for a 30 cm hang, the planner's path between them, 13 toy
   spots over the reach zone). The pose over the bin puts the grasp point 0.10 m above the rim at every spot; the carry
   poses are 0.14-0.20 m above it and the path between them never dips below the pose over the bin. So an item hanging
   more than about 10 cm below the jaws meets the rim however high the carry asks for (the train, 28 Sep, pivoted in the
   jaws and hung longer). Tilting the lying robot doesn't raise it: the bin and the arm tilt together. Raised by a lower
   bin wall or a higher arm mount (hardware), or kept from mattering by gripping a long item near its middle.
10. [Q8C] **A re-park aims the grip, and a grip has a real width** (29 Sep 2026, rounds 12-13: "no reachable grip
    direction" lost 7 toys, most of them again after a re-park). A re-park placed the item's middle where the arm reaches
    best; for a long toy the grip that failed can sit 10 cm or more from its middle (the 32 cm cart turtle: every grip
    4.4-4.7 cm out of reach on the grasp bench). So it places the grip point there: the item plus its best grip's offset.
    And a grip narrower than 5 mm is a sliver of the mask, not a part (the bench offered 1 mm grips): the thinnest item
    gripped is a pencil, 7 mm.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [JwG] (seeded, repeatable) | two runs with the same seed compared | Partial: headless runs repeat exactly; not automated |
| [udO] (varied items) | `sim/arm/grasp_bench.py` item library | Passing: items from the library ([QxY]), dropped in any orientation |
| [bxP] (starts ~5 cm off) | bench guess noise | Passing |
| [OQ9] (one failure reason per trial) | bench trial records | Partial: "too wide" not yet in the bench |
| [bfo] (success = resting in bin) | bin check at trial end | Passing |
| [gF0] (summary appended to history) | `sim/suite/run grasp` | Passing |
| [4ZY] (passing the tipping point is never success) | bench tilt check | Partial: the bench checks a fixed 25 deg, below every measured tipping point, rather than the measured one |
| [guX] (soft items) | cloth and rope items | Not yet written |
| [N4z] (local floor at camera resolution, new each trial) | floor generator; grasp scenario sets a new floor per trial; `tests/test_floor_photos.py` (held-out floors never drawn for training, seamless repeat, relief lit from the side it faces) | Partial: `sim/arm/floors.py` (photographed floors, `floor_photos.csv`; generated floors by name; view-dependent pile shading in classic images); used by the bench, recorder and evaluator; not covered (test audit, 3 Oct 2026): three of four tests skip without the gitignored floor zips; resolution, a new floor each trial, the jitter and rug edges are untested |
| [ZMc] (front pickups) | `sim/arm/front_eval.sh` (`grasp_bench.py --zone front`, 100 trials, seeds 41 and 42) | Written: 43/62 pickable binned (69%) on the core items, 25 Sep. Two fixes got there: the target is solved so the lowest finger point sits 3 mm above the floor (the tips reach 3.4 cm past the grasp point, and aimed 6-12 mm up they dragged on the floor), and "held" is sized for a 7 mm pencil (the jaw joint reads half the gap) |
| [Hwc] (the grasp may lean) | leaning grasp in `sim/arm/pick.py`; lean recorded per trial | Written: every front trial records its lean (up to 30 deg) |
| [7Xd] (never presses on what it meets) | `grasp_bench.py` front runs on the core items, ropes included: no trial ends tipped over | Written: guarded descent in `pick.py` (never triggered in 9 rope and pencil trials). The rope "tip-overs" of 25 Sep were not the arm: squeezing the rope blew the solver up and MuJoCo reset the state; such trials are now set aside as "sim unstable" |
| [XCl] (too heavy is put back) | `grasp_bench.py`: the 2.2 kg block is left where it was; measured weight against true mass per trial | Not yet written |
| [2ho] (a turned item still goes in the bin) | `grasp_bench.py` front runs log the carry height and whether the item touched the bin's rim; DimOS night with the ballet flat | Not yet written: the ballet flat turned in the jaws on 25 Sep and caught the rim (it fell in on the second attempt) |
| [GU5] (a held item stays put) | `grasp_bench.py` with `HK_PICK_DEBUG=1` logs each item's turn in the jaws' own frame from closing to the bin; compared across HK_JAWS | Failing: ballet flat, 8 trials each (seed 5, 25 Sep), turn by the bin: rubber 13-42 deg (one 172, dropped), bare 10-54 (one 113), pads v1 29-126, pads v2 24-134; binned 7, 7, 6, 6 of 8. The jaws' material made no measurable difference: every setup gripped with the fingertips' last ~2 cm, one point per jaw, and the shoe swung about that. (An earlier log measured the turn in the world frame, counting the arm's own swing.); 30 Sep, the Reebok shoe (`data/eval/shoe_diag.log`, 6 trials): the jaws meet it only in their last 1.5 cm, on the sole's flared edge, and it turns 53-132 deg by the bin in 4 of 5 grasps (lost on the way twice, over the side wall twice); the one binned (170 g, gripped nearly upright) turned 28 deg. Gripped higher (`HK_GRIP_LIFT=1`: a toy 6 cm or taller has its fingertips at 35% of its height, at most 4 cm; off by default): still 1 of 6 binned, turning 32-130 deg (`shoe_lift.log`), so not the grip's height: it spins about the line between two contact points, its weight 1-4.5 cm off that line |
| [WuR] (a swappable grasp proposer) | `tests/test_grasp_proposers.py` (each proposer answers through the interface; ours gives the pick's own grip options unchanged); the 48-trial bench of [dgF] through the interface (`data/eval/grasp_ours_iface.log`) | Passing: 5 unit tests; the bench bins 27 of 38 pickable against 28 before the interface (29 Sep; the 6 trials that differ are mostly perception, before the grasp: run-to-run noise) |
| [oZb] (let go over the bin's middle) | `grasp_bench.py` front runs, the nights' carry (`HK_HIGH_CARRY=1 HK_BIN_EXIT=1`), `HK_BIN_RELEASE` corner against centre and near (`data/eval/grasp_carry.log`, `grasp_release_centre.log`, `grasp_release_near.log`) | Withdrawn: binned 26 of 38 (corner), 24 (centre, 7 missed bins), 25 (near); the shoe missed 4 of 4 every way (30 Sep) |
