# The cascade's check on the robot, and the lead on new nights (sim, 13-14 Oct 2026)

> **Why this is here:** the headline fell from 94% found to 90% when the numbers were re-measured the way the robot
> judges live. Cached replays had applied a kinder colour threshold than the one stated, and nothing flagged it. How
> that was found and fixed (detection-baselines [LQs]) is worth knowing before trusting any replayed benchmark.
> Paths under `../build/` were scratch space outside the repository and were not kept.

Criterion [Zaf] (the robot checks each floor-comparison candidate itself, opt-in) and decision [tTD] (launch at the
cascade's numbers once confirmed on unseen nights) in docs/specs/detection-baselines.md; the lead of
the cascade round (12 Oct 2026, summarised in the spec's [80w] Tests row). Sim only; CPU only (the 1280 re-sweep owned the GPU); nothing sent to
any hosted service; no hold-out (78, 80, 81, 100-119) opened; nothing written into `sim/house/runs/nights_s*`,
`data/runs/nights_s*`, `data/bench/*` or `data/robot/`. No default changed: the check is off unless asked for.

## Answer

**Correction, 14 Oct 10:00 (below, "Re-measured from float32 replays"): every stage-1 number in this answer came from cached replays, which judged at the paint run's colour threshold (tc 0.045), not the near choice's stated 0.032 (below, "Why the cached replays were kinder"). Painted live at 0.032, as stated, the six fresh nights give 47/48 at 35.5 false alarms a night at the threshold re-chosen on tuning (0.0504), and 44/48 at 12.2 at the old 0.2436, which finds only 39/48 on tuning. The launch bar of about 94% at about 10 does not hold live.**

**The cascade's check now runs on the robot's side, and replayed on the cascade round's six fresh nights it picks
exactly the bench's crops; on the CPU it finds 44 of 48 at 10.3 false alarms a night against the bench's 45 at 10.5,
the one toy lost a brick, and the whole difference is YOLO-E itself scoring a few crops differently on the CPU than on
the GPU.** On the new nights scored so far (seven by 06:50 on 14 Oct, four in houses the round never used) the check lost no toy the floor comparison found; the unseen houses find 31/32 (97%) at 13.5 false alarms a night, found above the bar set beforehand and false alarms just over it (12). Not confirmed yet; the sweep's other houses keep landing and are scored as they do.

- **Same crops, every one.** For all 531 candidates of the six fresh nights the robot-side stage chose the same three
  frames (same frame stamps) and cut the same boxes as the bench's lead (`cascade.py crops --by-time`), online, frame by
  frame, without seeing the rest of the night.
- **Same scores on 1538 of 1563 crops.** The 25 that differ were all re-scored by the bench's own code on the CPU: 24
  give the robot's score exactly (the remaining one differs below the threshold, likely the thread count). So the
  differences are the CPU against the GPU (YOLO-E's prompt-free head picks a different label or drops a box), not the
  robot's code. The cascade round's control, "CPU matches the GPU to the fourth decimal", held on its 20 crops but not
  on all 1563: about 1.6% of crops change, some by a whole detection.
- **What that changes:** 4 candidates of 531 cross the threshold, all from kept to dropped: one is the near-layer
  candidate the scorer credits house 96's 1 x 1 brick to (11 fresh), so found falls 45 to 44 (ordinary toys 42/44 on
  both); one is a false alarm on house 71, so false alarms fall 10.5 to 10.3 a night. In real use the check runs on the
  laptop's GPU, where it should give the bench's numbers; that replay is queued for when the GPU is free
  (`../build/confirm/gpu_q.sh`), not yet run.
- **Live, as the tidy would run it** (frames kept 60 s, a candidate scored 60 s after its decision with the views it
  has): 9 candidates of 531 get fewer views; 44/48 at 10.7 a night on the CPU. Half the candidates are scored within
  2.7 s of their decision, 90% within 31 s; 18 of the six nights' candidates are never seen three times.
- **Confirmation on new nights (frozen setup, CPU), so far:** unseen houses 73, 77, 82 and 83: **31/32 found at 13.5 false alarms a night** with the check (stage 1 alone 31/32 at 46.5). All seven new walks: 52/56 at 12.4 (stage 1 alone 52/56 at 47.1). The check dropped no toy that stage 1 found (52 of 52); the 4 misses are stage 1's own (house 73's brick, and three toys on house 96's carpet in a re-walk of a tuning layout). The rule set beforehand calls it confirmed at 93% or more found and 12 or fewer a night, pooled over the unseen nights: found passes, false alarms are 1.5 a night over, on four nights; not called yet.

## What was built ([Zaf])

| | |
|---|---|
| The stage | `src/houndkeeper/candidate_check.py` (robot side, [1B2]): `CandidateCheck` takes the head camera's frames in time order (stamp, the camera's pose then, the raw fisheye frame) and the confirmer's decisions as they are made. A candidate (confirmed or unsure, never floor-sized; decisions in one 5 cm cell share its first decision) chooses its views greedily by nearness in time to that decision, 0.6-2.0 m off, 1 s apart, inside the image; a view after the decision is always the farthest so far, so once three are chosen and the newest frame is past the farthest, the choice is final and the candidate is scored. Crops are cut when their frame is chosen (recheck.py's crop: 0.4 m of floor, raised, 384 px). YOLO-E 11l prompt-free (DimOS's wrapper, house labels dropped, the box grown by a tenth of the crop must hold the point); kept at 0.2436 or above, or when no frame saw it. Each verdict keeps its views' frame stamps, indices, ranges, boxes and scores |
| Threshold | **0.2436, the lead's**, chosen on the six tuning nights by recall first (cascade round, `results.json` "yoloe_pf_l (lead: views by time)"; 45/48 at 7.8 there). Not the judged crops' 0.2073 |
| Device | the GPU when there is one, else the CPU (`HK_CHECK_DEVICE` to choose); a GPU that fails at load or mid-night (no memory beside the sim, a driver fault) falls back to the CPU and says so. On the CPU it holds torch to 2 threads (`HK_CHECK_CPU_THREADS`): ultralytics takes 8 at its first prediction otherwise, which starved the shared machine tonight |
| Replay | `sim/house/confirm_stream.py --check` (off by default): after the night's decisions (live or `--cache-in`), the frames (every 2nd, placed as the confirmer places them: odometry at the frame's time less the camera's lag, the calibrated mount) and decisions fed in time order, each crop read back from the robot's own recording when chosen; `--check-live` as the tidy runs it. Works under `floor_diag.py replay` too (the verdicts land in each decision's `check`) |
| Live tidy | `HK_CANDIDATE_CHECK=1` (off by default): each candidate is held until scored, then passed to the stop gate if kept or logged "dropped by the check"; candidates still held at the sweep's end are scored with the views they have and visited then. Frames are kept 60 s in memory (about 0.6-1.2 GB at the tidy's frame rate, 1280 x 720) |
| The launch setting | one named setting for [tTD]: `confirm_stream.py --setting launch` is the r11 near choice (floor_arm's loose1 with the near-choice options, the 5 mm near grid; it needs the baselines' near views, `--near-views`) and `--check`; explicit options still override it; defaults unchanged. `sim/house/real_nights.sh` can select it through TONIGHT_OPTS. The live tidy selects it with `HK_DETECT_SETTING=launch`: its 1 cm options and the check run; the near grid does not run live yet (the live confirmer has no near layer), and the tidy says so |
| Tests | `tests/test_candidate_check.py` (19, all passing): views chosen online equal `cascade.pick_views_by_time` over the whole night; settling as soon as the views are fixed; the crop, projection and house labels equal the bench's; the threshold equals the round's; kept at the threshold; unseen kept and floor-sized changes skipped; frame stamps kept; old frames not used live; the live wait; GPU-to-CPU fallback at load and mid-night; the device choice; off by default; placed on the robot's side; the live tidy holding a candidate until scored; the launch setting equal to the r11 job's options; with the check off the tidy passes every decision on unchanged. `tests/test_truth_boundary.py` places the module with the robot |

Not tested live: no tidy night was run with the check on (the GPU was the re-sweep's). On the CPU a candidate costs
about 2-5 s on this shared machine (1.8 s a crop at 2 threads under load 25), which the tidy loop would wait for; on
the GPU about 0.1 s.

## The replay on the cascade round's fresh nights (Part 1)

Stage 1: the round's saved near-choice replays (`../build/hr/r11/v/F_sl1_l3_N`), their decisions fed to
`confirm_stream.check_stream`; head mount from the v8 copies the round used; CPU, 2 threads. Scored by the shared scorer
(`cascade.arm`, `table.score_arm`) at 0.2436.

| Arm (six fresh nights, 48 toys, 4 bricks) | Found | Ordinary | Bricks | False a night | Candidates kept a night | Per night false (71 t11, 71 t1, 76 t1, 70 t2, 96 t2, 96 t11) |
|---|---|---|---|---|---|---|
| stage 1 alone | 47 | 43/44 | 4/4 | 47.3 | 90.8 | 57, 61, 32, 44, 46, 44 |
| bench's lead (GPU, 12 Oct) | 45 | 42/44 | 3/4 | 10.5 | 24.5 | 9, 8, 3, 11, 13, 19 |
| **robot-side stage, CPU** | **44** | 42/44 | 2/4 | **10.3** | 23.8 | 8, 8, 3, 11, 13, 19 |
| robot-side stage, CPU, as live (60 s memory and wait) | 44 | 42/44 | 2/4 | 10.7 | 24.0 | 8, 8, 3, 12, 13, 20 |

| Agreement with the bench | |
|---|---|
| candidates | 531 on both |
| views (frame stamp and box) | 531 of 531 identical (also checked on house 76's tuning night: 114 of 114) |
| crop scores | 1538 of 1563 identical; 25 differ, 24 of them reproduced exactly by the bench's own code on the CPU |
| candidates kept | 142 against 146; the 4 (bench score, robot score): house 71 t11 (0.341, 0.214), 71 t1 (0.380, 0.217), 96 t11 (0.380, 0.228), 96 t11 (0.577, 0.066), the last the candidate credited with the brick |
| missed toys | the bench's three (house 71's 1 x 1 brick, the mug, house 76's aeroplane) and house 96 t11's 1 x 1 brick |
| time | 7-35 min a night on 2 CPU threads at load 15-30 (about 260 crops a night) |

## Confirming the lead on new nights (Part 2)

Rules written before any new night was touched (`../build/confirm/RULES.txt`, 20:16 13 Oct): the frozen
setup of the cascade round (code 8991753; stage 1 the r11 near choice, paint then choice as the round ran it; the lead's
crops; YOLO-E 11l at 0.2436; no retuning), each house's new 1280 baselines copied read-only with their near views made
as the round made them; CPU only. The one unavoidable difference: the houses' maps are the re-sweep's v9 (loops
verified), where the round used v8 copies.

**Which nights are new.** Every walk of the re-sweep is a new recording on new baselines. Its toy layout was compared
with the round's nights (`data/bench/houses_hr/nights.json`): houses 70 (set 1), 71 (set 2) and 96 (set 1) repeat the
round's *tuning* layouts exactly (same toys at the same places; the threshold was set on those toys, the binding one
house 70's figure), so they are shown but kept out of the confirmation; houses 73, 77, 82 and the rest of 72-93 were
never used by the round: those are the confirmation. None of the new walks repeats a fresh-night layout.

| Night | Class | Stage 1: found | ordinary | bricks | false a night | + check: found | ordinary | bricks | false a night |
|---|---|---|---|---|---|---|---|---|---|
| s70_t1_r1280 | layout seen: 70 t1 (tuning) | 8/8 | 8/8 | 0/0 | 41.0 | 8/8 | 8/8 | 0/0 | 8.0 |
| s71_t2_r1280 | layout seen: 71 t2 (tuning) | 8/8 | 7/7 | 1/1 | 66.0 | 8/8 | 7/7 | 1/1 | 11.0 |
| s73_t1_r1280 | unseen | 7/8 | 7/8 | 0/0 | 41.0 | 7/8 | 7/8 | 0/0 | 19.0 |
| s77_t1_r1280 | unseen | 8/8 | 8/8 | 0/0 | 35.0 | 8/8 | 8/8 | 0/0 | 6.0 |
| s82_t1_r1280 | unseen | 8/8 | 8/8 | 0/0 | 35.0 | 8/8 | 8/8 | 0/0 | 11.0 |
| s83_t1_r1280 | unseen | 8/8 | 8/8 | 0/0 | 75.0 | 8/8 | 8/8 | 0/0 | 18.0 |
| s96_t1_r1280 | layout seen: 96 t1 (tuning) | 5/8 | 5/8 | 0/0 | 37.0 | 5/8 | 5/8 | 0/0 | 14.0 |
| **unseen nights (4)** | | 31/32 | 31/32 | 0/0 | 46.5 | 31/32 | 31/32 | 0/0 | 13.5 |
| **all nights (7)** | | 52/56 | 51/55 | 1/1 | 47.1 | 52/56 | 51/55 | 1/1 | 12.4 |

- **The check lost nothing.** On all seven nights it kept every toy stage 1 found (52 of 52) and cut false alarms from
  47.1 to 12.4 a night (74%; the round's fresh nights: 78%). Candidates kept a night: 20-46 of 59-137.
- **House 83** (unseen) has the most stage-1 false alarms so far (75 a night), 18 after the check.
- **The misses are stage 1's.** House 73's brick (the large one; not a 1 x 1), and on house 96's re-walk the lacing
  sheep, the card game and the gold shoe, all on carpet (the round's tuning walk of that layout missed only the card
  game, on v8 maps). Not looked into yet: by the rules the numbers come first.
- **House 73** carries the most false alarms with the check (19 a night, 46 candidates kept); houses 77 and 82 sit at 6
  and 11.
- **The robot-side stage on the same nights** (cross-check, CPU both): the same scores on every crop and the same
  candidates kept on the first six (79, 82, 128, 59, 63, 58 candidates), so its numbers equal the table's; house 83's cross-check follows in the same run.
- **Still to come** (continues 14 Oct as the walks land, `../build/confirm/watch.sh`): houses 84-93 (about
  100 min a walk, then about 45 min to score); house 72's re-sweep walk did not land tonight. The confirmation is
  called on the unseen nights pooled, by the rule above, once they are in.

## Re-measured from float32 replays (14 Oct, after the sensitivity study)

docs/research/sensitivity-2026-10-13.md found that the cascade round's stage 1 was replayed from confirm_stream's saved
comparison (`--cache-in`), which stores each frame's paint in float16; the robot paints in float32 live. (The diagnosis below finds the real cause was the cached colour threshold, not float16.) **Every stage-1
number above went through that cache**: Part 1's replay fed the round's cached decisions (so its agreement with the bench
stands, its absolute numbers do not), and this round's new walks were painted with `--cache-out` and decided with
`--cache-in`, as the round did. (`score1280.py`'s view-noise replays do not use the cache.) Re-measured, CPU, rules
written first (`RULES.txt`, 07:3x 14 Oct):

- **Tuning, float32** (the sensitivity study's nominal replays, crops and verdicts reused): no threshold above keep-all
  keeps 45/48, because one toy scores 0 on every crop. By the rule's fallback, the most found above keep-all is 44/48,
  strictest at 0.063 (27.8 a night); with the fifth's margin the judged threshold is **0.0504**: 44/48 at 30.8 there.
  The curve's knees: 42/48 at 14.8 a night (0.185), 39/48 at about 11 (0.2436, the old threshold).
- **Fresh, float32, scored once:**

| Six fresh nights, painted in float32 | Found | Ordinary | Bricks | False a night | Kept a night |
|---|---|---|---|---|---|
| stage 1 alone | 47/48 | 43/44 | 4/4 | 61.7 | |
| **+ check at 0.0504 (judged)** | **47/48** | 43/44 | 4/4 | **35.5** | 64.0 |
| + check at 0.063 (context) | 47/48 | 43/44 | 4/4 | 32.3 | 58.0 |
| + check at 0.2436 (the old threshold, context) | 44/48 | 41/44 | 3/4 | 12.2 | 26.7 |
| (float16, as reported 12 Oct: lead at 0.2436) | 45/48 | 42/44 | 3/4 | 10.5 | 24.5 |

- **Plainly: the launch bar does not hold when the robot paints as it would live.** Stage 1 proposes about 30% more
  false alarms (61.7 against 47.3 a night). The check still halves them while keeping every toy, but at the recall the
  bar asks for (94%) about 33-36 false alarms a night remain, not 10. At about 12 a night the old threshold finds 44/48
  on these fresh nights, but on the tuning nights it finds 39/48, so nothing chosen on tuning gets there.
- **The new walks** are being replayed in float32 the same way (`f32_queue.sh`, one at a time behind the fresh nights);
  the float16 table above stays as recorded and is superseded by them.

**Proposal for the cache (shared code; nothing changed):** a cached replay is a screen, not a measurement. Either store
the paint in float32 (about twice the cache's size; then a cached replay equals a live one, which a test can hold: one
night, cached against live, the same decisions), or keep float16 for screening and have every replay record whether
it came from a cache (`cached: true` in its JSON) and the scorers mark or refuse cached replays for any reported number.
The first is simpler and makes the whole class of error impossible; I would take it.

## Why the cached replays were kinder: the colour threshold, not float16 (14 Oct)

Experiments on two tuning nights, CPU, the same code (`../build/cachediag/`):

| House 76 set 11 (the night with the biggest gap) | Stage 1 found | False a night | Candidates |
|---|---|---|---|
| near choice painted live (float32), tc 0.032 as stated | 8/8 | 52 | 114 |
| the round's cached replay (painted at its paint options, float16 cache) | 8/8 | 40 | 76 |
| the same paint options, cache saved in **float32**, near choice replayed from it | 8/8 | **40** | **76** |
| near choice painted live (float32) with **tc 0.045** | 8/8 | **40** | **76** (89 of 94 decisions identical to the row above) |

| House 70 set 11 | Candidates | Matching the live run |
|---|---|---|
| near choice painted live (float32) | 72 | (equal to the sensitivity study's nominal, decision for decision) |
| the same run's comparison saved in float16, replayed | 72 | all 72 within 1 cm (only coordinates rounded); 39 a night both |
| the same, saved in float32 (the new cache), replayed | 94 decisions | 94 of 94 identical |

- **Float16 rounding was not the cause.** Rounded to float16 the comparison decides the same spots. A float16 step at the
  colour threshold (about 0.00002) is some 200 times finer than one grey level of an 8-bit camera (about 0.004), so it
  is neither like a real camera's quantization nor a smoothing; it is invisible next to them.
- **The cause: a cached replay does not use its own `--tc`.** The saved comparison holds each frame's colour threshold
  as the paint run computed it: its `--tc` raised where the frame's own noise is higher (`_colour_threshold`). The
  round painted at the default tc 0.045 and replayed the near choice (`--tc 0.032`) from that cache; the replay judged at
  0.045 (every cached frame's threshold is 0.045 or more, checked), and so did every number built on it: the full-
  resolution round's near choice, the cascade round and [tTD]'s 45/48 at 10.5. The near choice's lower colour threshold
  was never measured; what was measured is the near choice at tc 0.045, and painted live at tc 0.045 it gives the same
  candidates (house 76: 40 a night, 76 candidates, both ways).
- **So it is a knife edge, but a real one in a setting, not in arithmetic.** At tc 0.032 the colour test passes many
  more cells: floor texture, seams and view-to-view colour shifts of a few grey levels that a real camera's noise also
  produces. tc 0.045 is the stricter, quieter choice, and it is a real lever: the honest launch candidate is "the near
  choice at tc 0.045, painted live". That arm is now running on the CPU (pre-registered in `RULES.txt`): the six tuning
  nights, the threshold by the same rule, then the six fresh nights once (`tc45_queue.sh`, several hours).
- **What still leaks through any cache with other options:** the floor locks and tracking (the snap leaves out cells its
  own decisions found changed). On house 76 they moved 5 of 94 decisions; the colour threshold moved the rest.

**The cache, fixed as a class ([LQs], `sim/house/confirm_stream.py`, `sim/house/baselines/floor_diag.py`):** the
comparison is saved in float32 and its header records its precision and the options it was painted with. A replay
records its precision and any options that differ from the paint's (`cache_differs`). The scorers (`floor_diag.arm_of`,
`cascade.decisions`, `curves.floor_arm_scored`) refuse a replay from a float16 cache, or from a cache painted with other
options, unless `HK_CACHE_SCREEN=1` (analysis only). Old caches read as float16. A cache replayed with its own options
decides exactly as the live run: 94 of 94 on house 70 set 11, and the knife-edge test fails with float16. Size: one
night's cache 411 MB in float32 against 233 MB in float16 (1.76 times); replaying it still takes about 6 minutes
against about 38 to paint.

## The honest launch arm: the near choice at tc 0.045, painted live (14 Oct)

Pre-registered in `build/confirm/RULES.txt` before any fresh night was scored: the six tuning nights painted live at
tc 0.045, the light check's threshold chosen on them by the round's rule (16:44: 0.1107), then the six fresh nights
scored once (21:48).

| | found | bricks | false alarms a night |
|---|---|---|---|
| tuning, stage 1 | 46/48 | | 45.5 |
| tuning, check at 0.1107 | 46/48 | 3/4 | 17.0 |
| **fresh, stage 1** | 46/48 | | 49.5 |
| **fresh, check at 0.1107 (chosen on tuning)** | **46/48** | **4/4** | **22.5** |
| fresh, check at 0.2436 (the robot's) | 43/48 | 3/4 | 11.7 |

Per fresh night at 0.1107: 7 to 33 (house 96 the noisiest). The cached figure, about 94% at about 10, sat between
the last two rows: at the same false alarms the honest share found is about 90%. The launch setting
(`confirm_stream.py --setting launch`) now paints at tc 0.045; the robot's threshold stays 0.2436 until the first real
nights set it ([tTD], sim-to-real-calibration [wTj]).

## Provenance

- Code: the working tree (8991753 plus this change) for Part 1; Part 2 from a frozen copy of 8991753
  (`../build/confirm/code`), the bench's lead unchanged (`cascade.py`, `recheck.py`, `recognizers.py`).
- Rules: `../build/confirm/RULES.txt`, written 20:16 13 Oct before any new night was replayed, every change
  of plan time-stamped. Scratch: `../build/confirm/` (`onboard/` Part 1, `runs/` the house copies with their
  near views, `v/` stage 1, `crops/` the check, `table.md`, `progress.txt`, `logs/`).
- Models: YOLO-E `yoloe-11l-seg-pf.pt` from DimOS's data, `HF_HUB_OFFLINE=1`; nothing downloaded.
- Machine: CPU only, nice 15, 2 threads per job, each step started only under a load of 12; the machine sat at 15-30
  all evening from other jobs, which set the pace. One slip: the queued GPU replay's gate matched another note's words
  "driver done" at 22:15 and started early; it was stopped within about 10 s (its model may have loaded on the GPU
  briefly), its output removed, and it is queued again behind the driver's own last line.

## Caveats

- Sim only; 48 toys a set, one toy is 2 points; the one toy of difference is a brick credited to a candidate 2-6 cm off.
- The robot-side numbers are CPU numbers; the GPU replay that should match the bench to the crop is queued, not run.
- The live tidy path is unit-tested, not run on a night.
