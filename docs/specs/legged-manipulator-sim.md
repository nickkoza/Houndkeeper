# Legged robot with an arm in DimOS's sim

**Type:** Capability spec · **Status:** Draft · **Source:** principle [1kb]; upstream contribution · **ID:** [RJi]

## Purpose

DimOS's simulator can run a walking robot carrying an arm: the dog walks with its own controller, DimOS's navigation
and mapping work as usual, and the arm is driven through DimOS's arm interface with its camera as a normal stream.
Offered upstream so DimOS supports mobile manipulation in sim.

## Acceptance criteria

1. [nTt] A Go2 with a Maker Arm loads in DimOS's sim and walks with DimOS's usual navigation and mapping.
2. [lps] The arm's joints and its wrist camera work through the connections the model gives this sim (CALM node
   `legged-manipulator-sim`).
3. [KAe] The dog's walking controller sees only its own legs; the arm's joints never disturb it.
4. [rUS] The dog can be commanded to stand, crouch and lie down.
5. [EaS] Other robots in DimOS's sim behave exactly as before.
6. [pse] It is submitted upstream as a pull request, meeting DimOS's contribution rules ([2SB]).

## Decisions

1. [oHW] **DimOS's gripper command runs opposite to the MJCF joint value.** Observed 2026-09-24 in the fork at a7492431f:
   `ManipShmReader.read_gripper_position` reflects the joint about its range and `MujocoSimModule` maps commands the
   same way. The arm bridge in the sim process follows that convention; Houndkeeper's DimOS-backed robot converts at its
   boundary so the pick routine works in joint values.
2. [PBh] **`MujocoConnection.get_odom_message()` returns None when no new message has arrived** since the last call (same
   fork). Anything polling it keeps the last pose.
3. [WL1] **DimOS's default scene has an office chair at the origin.** A robot lying there was spun ~50 deg by its own arm
   hitting the chair; tests start at (-1, 1).
4. [tw1] **ONNX Runtime on CUDA for the walking policy took ~3.3 GB of GPU memory** (fork at a7492431f, 2026-09-25): the sim
   now runs that small policy on the CPU unless DIMOS_SIM_POLICY_GPU=1, leaving the GPU to perception models.
5. [oBB] **The Go1 scene disables implicit damping and runs one solver iteration** (disableflags 32768, iterations 1): joint
   damping on gram-scale bodies (rope segments) is unstable at the 4-5 ms step.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [nTt] (loads and walks with DimOS nav) | DimOS sim run (`sim/dimos/tidy_run.sh`) | Partial: the Go1 stand-in with the arm walks with DimOS navigation; not a Go2 ([WD6]) |
| [lps] (arm via DimOS interface, camera stream) | `sim/dimos/arm_bringup.py`, `pick_bringup.py` | Passing: arm through DimOS's sim-arm adapter; arm camera as a stream |
| [KAe] (walking controller sees only legs) | policy input test | Partial: fork's policy reads and drives only the robot's joints; no dedicated test |
| [rUS] (stand, crouch, lie commands) | posture test | Partial: lie down and stand up work through the connection in DimOS's sim; crouch not exercised |
| [EaS] (other robots unchanged) | DimOS's existing sim, connection and manipulator tests | Passing: 120 tests pass on the fork with the changes |
| [pse] (upstream PR) | PR opened | Not yet written |
