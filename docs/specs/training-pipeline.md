# Training pipeline

**Type:** Capability spec · **Status:** Draft · **Source:** D4; principle [eNx] · **ID:** [Pz2]

## Purpose

Turns demonstrations into a picking policy and tells us whether it is better than the scripted grasp: sim
demonstrations to get started, leader-arm demonstrations on the real arm to make it real, all trained and run locally.

## Acceptance criteria

1. [cLC] Scripted grasps in the sim can be recorded as demonstrations: wrist camera frames, joint states and actions.
2. [5fn] Leader-arm teleoperation on the real Maker Arm is recorded through DimOS's demonstration recorder.
3. [soS] Recordings become a training dataset with DimOS's dataset tools, and a policy is trained on the development
   laptop's GPU, with no cloud compute.
4. [7wf] Every trained policy is scored on the grasp scenario against the scripted grasp on the same seed, and the result
   is recorded with the policy.
5. [MXb] A policy runs on the robot through DimOS's policy runner, and the scripted grasp takes over when the policy is
   not confident or fails.
6. [zKq] Demonstrations and datasets stay in the house ([zGo]).
7. [qMA] **The pipeline is proven in sim first.** A policy trained only on recorded sim demonstrations, with DimOS's
   dataset tools and training on the development laptop, is scored on the grasp scenario on seeds it never trained
   on, and the result is recorded next to the scripted grasp's result on the same seeds.
8. [95i] **Training data keeps accumulating while the learned pick is parked.** Every pick attempt the pick routine
   makes, in any sim or on the real robot, is recorded automatically in the demonstration format (wrist images, joint
   states, commanded targets, 10 Hz), with the robot's own judgement of the outcome and, in sim, the simulator's truth,
   into storage in the house ([zGo]).
9. [cuv] A real recording in which a person appears is discarded, not kept ([JHu]).
10. [Yms] Sim and real recordings use the same camera names, image size and joint order, so they pool into one
    dataset without rework.
11. [c9x] Every policy evaluation is recorded and reported whether the result is good or bad; none is dropped for
    being poor.

([Yms] and [c9x] adapted from MOSS's data rules, docs/data.md in github.com/metrox-eth/moss at b2c887a,
CC BY 4.0: "the same camera names on every robot, so datasets can be pooled without rework" and results "published
whether they are good or not". Here results stay in the house ([zGo]); "reported" means in the project's own logs.)

## Non-functional requirements

- The sim-trained policy succeeds on at least half as many held-out grasp trials as the scripted grasp does on the
  same seeds (proposed; confirm). This is a pipeline check, not a performance target.

## Decisions

1. [7sv] **Learned pick parked (25 Sep 2026).** The real robot starts with the scripted grasp; effort goes into making it
   reliable. The pipeline stays as built. Every pick attempt on the real robot is recorded in the demonstration format
   (wrist images, joint states and targets, outcome), so real training data accumulates without teleoperation; the
   leader arm is for the cases the scripted grasp keeps failing. A learned pick is revisited as sim pretraining plus
   fine-tuning on that real data.
2. [Q3Q] **First policy on Filament frames and photographed floors (v6), 26 Sep 2026.** ACT with v4's settings (60k steps) on
   1,916 demonstrations recorded with the fixed Filament renderer ([UQK]: upright frames, floors at their real size and
   changing every trial, normal and roughness maps) on 31 photographed training floors ([N4z]). Head to head with the
   scripted grasp, both on Filament: training floors, seeds 900+: policy 6/23 (26%), scripted 20/23 (87%); held-out
   floors, seeds 900+: 6/23 vs 20/23; held-out floors, seeds 950+: 5/25 (20%) vs 18/25 (72%). Ratio about 0.3, below
   v4's 0.48 on classic frames and the proposed bar of half. The policy does no worse on floors it never saw, so the
   floors aren't what it is missing. Two differences between its training and its test stand out and are not yet
   separated: the demonstrations were recorded before per-frame auto-exposure ([Hsz]) and are a little fewer than v4's.
   The scripted grasp's own perception works on the photographed floors, held-out ones included. Discarded: 2,595
   earlier Filament demonstrations whose floor was frozen and magnified by the renderer ([akm]).
3. [bjo] **Best sim-trained policy so far (v4), 25 Sep 2026.** ACT on 2,084 demonstrations (1,251 recorded with a grey
   placeholder floor by mistake, 833 with generated carpets, rugs, wood and tile), joint targets, 60k steps (~80 min on
   the RTX 2070). Head to head with the scripted grasp on the same held-out episodes (seeds 900+ and 950+): policy 21/55
   lifted (38%), scripted 44/55 (80%): ratio 0.48, just under the proposed bar of half. Run-to-run spread is large (the
   same checkpoint lifted 15/27 on seeds 900+ in a policy-only run). Checkpoints learn late: 4% at 20-30k, 33% at 40k.
   Inside DimOS's sim the earlier v3 policy ran end to end through DimOS's arm adapter but lifted 0/2 (DimOS's floor
   and lighting differ from the training floors). Gripper-frame actions were tried and were worse (7%).
4. [NuZ] **First sim-trained policy, 25 Sep 2026 (sim, Go1 stand-in, mount +10 cm).** ACT (LeRobot 0.6.0) trained on 241
   privileged-teacher demonstrations (wrist 128x96, 10 Hz, chunk 20, 10 steps executed per query), 30k steps on the
   RTX 2070. Held-out seeds 900+, same item, pose, floor and start for both: the policy lifted 5/24 (21%); the scripted
   grasp with its own perception lifted 19/23 (83%). Checkpoints at 10k and 20k steps did no better (15%, 12%). The
   policy has learned the motion (descend, close, lift to ~17 cm); when it fails it closes 2-6 cm from the item, when it
   succeeds within 0.6 cm. The limit is localising the item from the image; more varied demonstrations come next.
   Below the proposed bar (half the scripted rate).

## Tests

| Criterion | Test | Status |
|---|---|---|
| [cLC] (sim demos recorded) | `sim/arm/record_demos.py` | Passing: privileged-teacher demos, wrist images, joint states and targets at 10 Hz |
| [5fn] (leader-arm demos recorded) | real collection session | Not yet written: leader arm ordered |
| [soS] (dataset and local training) | `training/sim/make_dataset.py` (DimOS's LeRobot writer), `lerobot-train` ACT on the laptop | Passing |
| [7wf] (scored against scripted, same seed) | `sim/arm/eval_policy.py` | Passing: both scored on the same seeds; results under Decisions |
| [MXb] (runs via DimOS, scripted fallback) | policy on robot | Not yet written |
| [zKq] (data stays in the house) | storage audit | Not yet written |
| [qMA] (sim-trained policy scored against scripted) | `sim/arm/eval_policy.py`, held-out seeds | Failing: 0.48 of the scripted rate against a proposed 0.5; results under Decisions |
| [95i] (every pick attempt recorded, sim and real) | recording in `sim/arm/pick.py`; a run leaves one episode per attempt under `data/demos_auto/` | Partial: every attempt recorded in the standalone sim and on DimOS (tested); real robot not yet |
| [cuv] (recordings with a person discarded) | person check on real recordings | Not yet written |
| [Yms] (sim and real pool) | one dataset built from a sim and a real recording | Partial: sim-standalone and sim-dimos recordings share the format (`pick.PickRecorder`); real not yet |
| [c9x] (every evaluation reported) | evaluation log review | Partial: every `eval_policy.py` run so far is logged in the findings |
