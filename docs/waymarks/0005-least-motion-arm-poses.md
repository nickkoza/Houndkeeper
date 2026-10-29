# [uEC] — Of several arm poses that reach a target, take the nearest

**Status:** Proposed · **Date:** 2026-09-25 · **Source:** seen in sim overnight 24–25 Sep 2026 · **Builds on:** [5oU] · **ID:** [uEC]

## Context

A six-joint arm usually has several ways to put the gripper at the same place and angle: the base turned one way or
the other, the elbow up or down, the wrist flipped. The inverse kinematics finds them from different starting guesses,
and without a rule for choosing it takes whichever it found first. Overnight on 24–25 Sep, in the sim, that sent the arm
half way round the robot to reach a spot a few centimetres from where it already was. In a house that swing goes
through whatever is beside the robot: furniture, a wall, a person. Nothing in the specs said which pose to choose, so
the rule lived only in code, where a later change could undo it without anyone noticing.

The same hazard showed up differently on 25 Sep: the arm's "ready" pose was defined as a spot straight above the
mount that the arm cannot reach, and the solver settled on a pose 34 cm to the robot's right, so every pick swung right
and back (`Sim.ready_pose` now defines ready over the front target instead).

## Decisions

1. [0uR] **Of the poses that reach a target, the arm takes the one needing the least joint motion from where it is
   now**, weighted per joint so the base and shoulder (which move the most mass through the widest arc) count more
   than the wrist (`Sim._nearest`, weights `Sim.MOVE_W`; the robot on DimOS uses the same code through its kinematic
   copy). Hard-won: the cost is a large sweep through the robot's surroundings, arriving at the same grasp.
2. [UH4] **Every pose the arm waits at or passes through is one the arm can actually reach**, checked by solving it,
   not assumed from a description: an unreachable target gets whatever the solver settles on, which is a pose nobody
   chose.

## Consequences

- A new way of solving for arm poses (a different IK, a learned policy's targets) keeps the least-motion choice, or
  states why not.
- The real arm's first bench days should confirm the choice holds with its measured joint limits.
