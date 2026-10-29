# Subsystem demos

**Type:** Capability spec · **Status:** Draft · **Source:** first-tidy [jWt] · **ID:** [Hza]

## Purpose

Short, very visual films of one subsystem at a time, doing its job on real data: to check that each subsystem is
understood, and for the project's YouTube videos.

## Acceptance criteria

1. [noB] **Every frame is real data.** A demo is drawn from the subsystem's own records on a real run of the sim (its
   recordings, its solver's saved passes, its weights), not a mock-up; where a step is slowed or exaggerated to be seen,
   the film says so.
2. [qEP] **The whole room in one view, in the house style.** A demo room small enough to see whole (about 20 x 20 ft)
   with furniture, cabinets, carpet and tile, filmed from outside it at an angle, the two walls nearest the camera
   left out so nothing is hidden. Neo-noir, as the showcase: calm, late night, warm. Its living
   corner is the channel's fireside (a fireplace, a leather wing chair, a side table), so the demos and
   the cold open share one set; the fire is never lit in a demo clip, so the edit has no moving fire to work around.
3. [HQ6] **Recreated, and committed.** Each demo rebuilds from its generator and its saved run, all of it kept in its
   own local repository, `houndkeeper-demos`, films included. The main repository keeps only what the sim needs to
   record a demo: the demo room's floor plan (`make_house.DEMO_PLANS`).
4. [nkM] **The first demo: whole-house floor alignment.** The floor map's geometry (sim-slice [FHA], [KkQ], [nv5]) from
   the problem to the payoff: the room; the robot's night, each camera frame painting a patch of floor; odometry's drift
   smearing the floor map; the lidar scan-match constraints (shown as springs) straightening the house; the floor images
   reduced to their edges; the floor-picture constraints between overlapping patches (shown as springs between them),
   each patch as opaque as it is trusted; pose graph optimization, shown as everything settling into its low-energy
   state; the floor map's error before and after. The order may change in review.
5. [tDW] **Cameras are blocked before anything renders**. For a shot (a recording, a time range, a camera fixed or
   following the robot) one quick command shows, in seconds and without a full render: a top-down plan of the room with
   the camera's view cone and the robot's route over the shot; whether the robot stays in frame and unblocked by
   furniture or walls the whole time, and when it doesn't; and a few low-resolution stills from the shot. A camera can
   also be placed by flying it in the viewer and saving its pose. The check is computed from the recording and the
   room's plan, so a hidden robot (night 1 behind the wing chair) is caught before a render, not after.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [noB] (real data) | the generator reads only a run's records; reviewed | Planned |
| [qEP] (the whole room, in style) | `tests/test_demo_room.py`: the demo room's floor in the demo camera's frame, its two near walls left out | Not yet written |
| [HQ6] (recreated) | the generator rebuilds a demo from its saved run | Planned |
| [nkM] (floor alignment demo) | the film, reviewed | Planned |
| [tDW] (cameras blocked first) | the framing check on a recorded demo night: the robot's visibility per frame from the route and the room plan, against a render's own visibility at a few frames; the plan view and stills made in seconds | Partial (3 Oct 2026): `src/block.py` in the demos repository (`houndkeeper-demos`) plans and checks a shot in about 2 s (route, camera footprint, share hidden by ray casts, spans, JSON). Not yet done: the comparison with rendered frames, the stills, the pytest, and finding the night-1 wing-chair occlusion: on layout 2's night 1 with the wide camera the check finds at most 2% hidden |
