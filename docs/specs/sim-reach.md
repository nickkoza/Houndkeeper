# Reach map

**Type:** Capability spec · **Status:** Draft · **Source:** D0 [jX2], D3 · **ID:** [RHS]

## Purpose

Says which floor spots the arm can pick from, for each posture, so the planner can choose where the robot parks.

## Acceptance criteria

1. [ssZ] For each posture and a stated maximum lean of the gripper from vertical, it maps which floor positions around
   the robot the gripper can reach without any part of the arm touching the robot, itself or the floor.
2. [OG0] The map is written for the stop planner, with a margin for how far parking can be off.
3. [Fmc] It is regenerated whenever the arm, its mount or the robot model changes, and a stale map is refused.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [ssZ] (reachable floor per posture, collision-free) | `sim/suite/run reach` | Partial: lying only |
| [OG0] (written with a parking margin) | `sim/arm/reach_lying.json`, planner erosion | Passing |
| [Fmc] (stale map refused) | model fingerprint check | Not yet written |
