# [IQW] — DimOS's grasping against our own (sim trial)

**Status:** Decided for now: keep our grasp (sim trial, 29 Sep 2026) · **Date:** 2026-09-29 · **Source:** whether to use
DimOS's manipulation component, settled by a grasp trial; upstreaming a Maker Arm driver is in the grant proposal, so
the work is worthwhile either way · **Serves:** maker-arm-adapter [Odx],
[KXa]; sim-grasp · **ID:** [IQW]

## Context

DimOS's manipulation stack plans and executes picks (its `ManipulationModule`, `PickAndPlaceModule`) from grasp
proposals (a `GraspGenSpec`: the `HeuristicGraspModule`, or the learned GraspGenX), given an object's segmented point
cloud. Our pick chooses its own grasp: the jaw direction, width and point from the item's measured footprint, the
narrowest grip at its middle or off it, a part where the whole is too wide, the fingertips just above the floor, the
gripper allowed to lean up to 30 degrees. Moving onto DimOS's stack is worth it if its grasps do as well.

## Decisions

1. [dgF] **The trial swaps only the grasp choice** (29 Sep 2026). On the grasp bench (`grasp_bench.py`, the same
   toys, seeds and parking as our runs), `HK_GRASP=dimos` hands DimOS's own `HeuristicGraspModule.propose_grasps`
   (DimOS a7492431f) the item's point cloud (the wrist stereo pair's points inside the look's item mask, standing
   more than 4 mm off the floor), and the pick executes its grasp: the cloud's median point, closing across the
   cloud's narrow axis, straight down (2 degrees allowed for the IK), the grasp centre at the cloud's mid-height
   (raised only if the fingertips would go through the floor). The looks, the too-wide and too-big refusals, the
   approach, the carry and the bin are ours in both arms of the trial, so a difference is the grasp. Not tried:
   GraspGenX (a learned model in its own container), and DimOS's planner driving the approach.

   Results (29 Sep; 48 trials each, seed 7, the front zone, 16 toys, the 40 mm pair with its light; `grasp_ours`,
   `grasp_dimos`, `grasp_dimos_lean` under `data/eval/`):

   | Grasp | Pickable binned | Easy | Hard | No reachable grasp | Not lifted | Missed bin or lost |
   |---|---|---|---|---|---|---|
   | ours | 28 / 38 | 24 / 29 | 5 / 19 | 6 | 1 | 5 |
   | DimOS's, straight down (as it gives it) | 20 / 38 | 18 / 29 | 4 / 19 | 14 | 2 | 2 |
   | DimOS's point and jaw, our 30 degree lean (`HK_DIMOS_LEAN=30`) | 23 / 38 | 22 / 29 | 2 / 19 | 7 | 2 | 6 |

   Both kept 8 of 10 too-big toys out and refused the same 4 as too wide. A straight-down grasp is out of the lying
   robot's reach twice as often as ours; with the lean, reach is nearly equal and ours still bins 5 more, most on the
   hard toys, where the cloud's middle is a poor place to grip. So we keep our grasp. The trial was one run each
   (a few toys differ run to run in the sim), so gaps of 2 or 3 are within noise; 28 against 20 is not.

2. [uVy] **A learned grasp generator in the same slot: GraspGenX, tried and not adopted** (29 Sep 2026). DimOS's
   GraspGenX runtime (NVIDIA GraspGenX b942909, model GraspGenXModel 7c83404), in DimOS's own isolated environment,
   served to the bench over a pipe (`sim/arm/graspgenx_server.py`, `HK_GRASP=graspgenx`). The Maker Arm's jaws are
   described to it from the SDK's URDF (b30d05a) as a sweep volume:
   10.48 cm open, 2.4 cm-wide pads running 6.1 cm back from the fingertips, 20.4 cm from the wrist. It gets the same
   point cloud as [dgF], thinned to 2,048 points, returns 100 scored grasps, and the pick runs the best of the first
   30 that approach from above and that the arm reaches clear of the robot and floor (the tips raised up to 1.5 cm
   off the floor; `HK_GGX_TILT=20` also lets each be turned up to 20 degrees toward upright). Forced: on the 8 GB
   laptop GPU the sim's renderer and GraspGenX together ran out of memory (the renderer aborted), so GraspGenX runs
   on the CPU (`HK_GGX_DEVICE=cpu`: its `.cuda()` calls made no-ops in our server only), about 2 s a request on 4
   threads against 0.3-0.7 s and 0.84 GB on the GPU.

   | Grasp (48 trials each, as in [dgF]) | Pickable binned | No reachable grasp | Not lifted or dropped lifting | Lost on the way or missed bin |
   |---|---|---|---|---|
   | GraspGenX, exact poses | 14 / 38 | 11 | 3 | 7 |
   | GraspGenX, turned up to 20 degrees toward upright | 15 / 38 | 11 | 4 | 9 |

   Worst of the five (ours 28, DimOS's heuristic 20-23). Turning toward upright bought no reach: what fails is
   mostly the approach path, not the grasp pose. Its grasps lean further than ours (up to 76 degrees; ours stop at
   30), and a toy gripped on a steep slant turns and falls when the carry brings the gripper upright, so more are
   lost on the way. It did no better on the awkward toys (the pony 0 of 5, the elephant 0 of 2). Not adopted. What
   could change it: its grasps scored against the arm's reach and the carry (not only the grip), a cloud from more
   than one look, and a GPU with room for it beside the sim (the home server, later).
