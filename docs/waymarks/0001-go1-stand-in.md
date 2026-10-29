# [WD6] — DimOS's simulated Go2 is a Go1

**Status:** Proposed · **Date:** 2026-09-24 · **Source:** observed in DimOS's sim code · **Builds on:** [1kb], [33f] · **ID:** [WD6]

## Context

Every sim result in this repository comes from DimOS's simulator or from the standalone arm sims built on its robot
model. Neither has a Go2. The numbers the specs rest on (tipping points, walking with the arm, reach, mount position)
are numbers for a different, lighter robot.

Observed 2026-09-24 in the DimOS fork at a7492431f, and unchanged on upstream main at 6043c7371:
`dimos/simulation/mujoco/mujoco_process.py` replaces a requested `unitree_go2` with `unitree_go1`, and walks it with a
Go1 ONNX walking policy trained without a payload. The Go1 body is about 12.7 kg; the Go2 Air is 15 kg (vendor).
The real Go2 Air walks with Unitree's own controller, not this policy.

## Decisions

1. [iFe] **Every sim result is labelled as coming from the Go1 stand-in** until it is re-run on a Go2 model, and no
   hardware is fabricated from a stand-in result alone ([Izu], [7Xn]).
2. [cAC] **The Go2 model for re-runs is MuJoCo Menagerie's `unitree_go2`**, kept in `models/go2`. It has no trained
   walking policy here, so walking results stay on the Go1 until a Go2 policy exists or the real robot is measured.
3. [BJE] **Re-check this fact on every DimOS sync.** If upstream adds a real Go2 sim, the sims move to it and every
   dependent scenario is re-run.
