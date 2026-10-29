# First tidy

**Type:** Capability spec · **Status:** Draft · **Source:** deliverable D4 · **ID:** [nEZ]

## Purpose

The whole loop on the real robot for at least one kind of item: find it at night, look closer, pick it up, put it in
the bin, and carry on, with failed picks left for a person to review.

This is the goal once the hardware is here: a filmed demonstration of the Go2 Air with the Maker Arm tidying on DimOS,
good enough for Dimensional to post on social media, which opens the way to more hardware (a better dog or onboard
compute).

## Acceptance criteria

1. [Swf] For at least one item class agreed in advance (for example socks), the robot picks and bins it end to end on a
   real night.
2. [m62] Within one stop a pick is tried up to 3 times, looking again before each try and gripping a different way;
   a pick that still fails is marked with where it is and a picture, and is not tried again on a later night until a
   person has reviewed it. Only failures that another try could change are retried (it closed on nothing, or the item
   slipped), never one judged too wide or too heavy.
3. [Xsf] After a pick the arm stows and the sweep resumes where it stopped.
4. [B4g] Picking is done by a policy trained on demonstrations recorded with the leader arm, or by the scripted grasp,
   and the report says which.
5. [RrD] The night is filmed for the demo only in rooms and at times chosen in advance, with no children in frame.
6. [4OX] **The demonstration video** shows the robot, running on DimOS, finding toys on a real floor, looking closer,
   picking them up and binning them, and carrying on, in one continuous run or honestly marked cuts; it says what is
   autonomous and what isn't (a person placing toys, restarting after a fault).
7. [OSi] **The demo video leaves the house as footage deliberately chosen for publishing ([zGo]):** the finished video,
   once watched end to end and approved, for Dimensional to post. Raw camera footage, recordings, datasets and the
   robot's own frames stay in the house as before; a robot's-eye clip goes into the video only if deliberately picked.
   It is a deliberate exception, filmed with no children around ([RrD]).
8. [jWt] **A retrospective YouTube video**, purely a retrospective, cut by a friend who has lent cameras. Once the robot
   works, a script walks through the project: the builder on camera explaining, over sim footage, charts, the hardware
   build and the robot working. Its house shots are deliberately set up (the robot working, the builder in the office),
   so they may be shared with the editor and published ([zGo]): a staged shot cannot leak something by accident as
   training data could; no children in frame ([JHu]); the robot's raw recordings and training data stay in the house,
   and a robot's-eye clip goes in only if deliberately picked. The final cut is approved before upload. Sim renders and
   charts carry no house imagery. The story is logged as it happens (in private video notes), since nothing is filmed
   while it's built.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [Swf] (one class end to end) | real night | Not yet written: sim slice for socks |
| [m62] (failed picks go to review, not retried) | real night log | Partial: sim flags review |
| [Xsf] (stow and resume) | real night | Partial: sim |
| [B4g] (policy or scripted, reported) | night report | Not yet written |
| [RrD] (filming rules) | filming checklist | Not yet written |
| [4OX] (the demonstration) | the video itself, reviewed end to end | Planned: after hardware |
| [OSi] (the demo video, approved) | filming checklist: sign-off before sending | Planned: after hardware |
| [jWt] (the retrospective video) | the story log kept current; the finished cut, reviewed and signed off before upload | Partial: story log started 30 Sep; video after the robot works |
