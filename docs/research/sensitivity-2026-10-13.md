# Which sim estimates the cascade's 94% leans on: a sensitivity study (sim, 13-14 Oct 2026)

> **Why this is here:** it ranks which of the sim's guesses about the robot the launch numbers depend on, which set
> day one's calibration bands ([GWP]), and it is where the sim was found never to have delayed its camera frames at
> all ("the sim never delayed its frames", below). The real calibration procedures cite it.
> Paths under `../build/` were scratch space outside the repository and were not kept.

Launch bar [tTD] (docs/specs/detection-baselines.md): the near-floor grid plus the YOLO-E check on the crops nearest in
time, about 94% found at about 10 false alarms a night. Principle [x3f] (the sim is never kinder than real); the
calibration procedures are [4hm]'s (real/calibration/README.md). Sim only; CPU only (the GPU was the 1280 re-sweep's);
nothing sent to any hosted service; no hold-out (78, 80, 81, 100-119) opened; the fresh nights not used; nothing written
into `sim/house/runs`, `data/runs/nights_s*`, `data/bench` or `data/robot`; `params/robot.yaml` not edited. Artefacts:
`../build/sens/` (driver `sens.py`, queue `runq.py`, rules `RULES.txt` written before any variant ran,
`v/<variant>/` replays, `c/` crops, verdicts and scores, `table.json`), `day/` (daytime changes), `gpu/` (the queued
render-time variants).

## Answer

**Correction (14 Oct 2026, afternoon):** the gap below is not float16 rounding. A cached replay applied the colour threshold its paint run had (tc 0.045, raised per noisy frame) and ignored its own `--tc` (0.032), so the round's near choice was really measured at tc 0.045; painted live at 0.045 it gives the cached candidates, and a float16 cache of a live paint changes nothing (docs/research/cascade-onboard-2026-10-13.md, "Why the cached replays were kinder"; fixed by [LQs]). The numbers here stand; read "float16 cache" as "cached replay at the paint run's colour threshold".

**On the CPU tonight, the cascade's numbers lean hardest on where each frame is placed: the head camera's mount
against its calibration, the frames' lag behind odometry, and odometry's drift. Each, made worse, raises stage 1's
false alarms by about two thirds and loses three toys there; the check behind it holds most of the found but its false alarms rise by half to three quarters. A harder
compressed stream (JPEG q30 all night) costs false alarms, mostly on the dark tile, not toys; four times the sensor
noise is within the replay's own spread. Before any of that: the launch figures were measured on replays from a
float16 cache, and painting the same nights live (float32, as the robot would) already gives 39/48 at 11.2 a night at
the lead's threshold, not 45/48 at 7.8.**

- **Ranked** (lead false alarms a night against the live nominal's 11.2; found 39/48): the head camera pitched 1 deg
  off its calibration 20.0 (+8.8; stage 1 found 42, false 88 against 54); the frames placed 150 ms off
  their stored lag 19.7 (+8.5; stage 1 found 42, false 88 against 54); odometry drift x4 16.5 (+5.3; stage 1 found 42,
  false 91); JPEG q30 15.3 (+4.1; stage 1 false 62); noise x4 12.3 (+1.1, found 42, within the spread). Pending in the
  queue when this was written: the 2x steps of lag and drift (lag 50 ms running), the mount at 0.5 deg, exposure, rolling shutter, jitter,
  drops (`../build/sens/table.json` fills as they finish).
- **The found count is fragile at a fixed threshold**: it moves by up to 3 toys between variants that should not help
  (noise x4 found 42 against nominal 39), because the threshold sits on one toy's score. Read found differences of 3 or
  fewer as noise; false alarms and stage 1's own found are the steadier measures.
- **Day one, first:** the head camera's mount and whether it stays put ([head_camera.md](../../real/calibration/head_camera.md)
  5), the frames' lag behind odometry ([timing.md](../../real/calibration/timing.md) 1-3) and odometry's drift
  ([odometry.md](../../real/calibration/odometry.md)); then the stream's compression on the darkest floor (no procedure
  measures it yet) and the noise ([camera.md](../../real/calibration/camera.md) 1).
- **Daytime changes** (not in the sim): YOLO-E prompt-free passes nearly every shoe, mug, bowl, glove or folded towel
  pasted into a rejected crop (0 of 30 before), rarely a light pool or shadow. A typical day's leftovers would add about
  4-5 false alarms a night, unless shoes and mugs count as things to tidy.
- **GPU (queued, not started):** eight render-time variants of house 70 end to end, one at a time after the re-sweep:
  drift first, then frame delay, the mount, compression, exposure, noise, lidar dropouts, the stream.

## The control: the launch numbers came from cached replays (see the correction above)

The cascade round's stage 1 (`M_sl1_l3_N`) was replayed from a saved comparison (`--cache-in`): the frames' paint
stored in float16 (`confirm_stream.FrameCache`). This study paints every frame again (it has to: a worse camera or
pose changes the paint), in float32, as the robot would live. That alone moves the numbers on the same six tuning nights
with the same code, crops and threshold:

| Six tuning nights, same code | Stage 1 found | Stage 1 false a night | Lead found (0.2436) | Lead false a night | Lead, false a night at 42 found |
|---|---|---|---|---|---|
| cached replay (float16; the cascade round's, `tune_t`) | 45/48 | 46.5 | 45/48 | 7.8 | |
| painted again (float32; this study's nominal) | 45/48 | 53.8 | 39/48 | 11.2 | 14.8 |

- **Checked:** the crops step run here on the cascade round's own replays reproduces its 1,469 crops exactly (every
  crop's box), and a second live replay of house 70 set 1 gives identical decisions (105 of 105). So the difference is
  the paint's precision, nothing else in this driver.
- **What it means.** Stage 1's candidates move by millimetres to centimetres and it proposes 16% more; the lead's
  threshold (0.2436) was the strictest keeping 45 on exactly the cached candidates' crops, so it sat on the edge, and
  six toys fall under it (house 71's Mario figure, 76's dinosaur, 76's school bus at 0.226, 70's cart turtle, two
  bricks). On the curve the live replay keeps 42/48 at 14.8 a night and 44/48 at 27.8. The fresh nights' 45/48 at 10.5
  ([tTD]'s figures) were scored from cached replays too.
- **Every variant below is compared with the live nominal** (the second row), never with the cached figures.

**This is the first thing to fix, before the robot**: [tTD]'s figures should be re-measured from live replays (or a
cache that keeps float32), and the lead's threshold chosen again on them with some margin under the binding toy, so the
real robot is compared with a number its own code path can produce. Not done here (it is a decision for the cascade's spec).

## Where each estimate acts, and how it was layered

| Parameter (`go2_air.`) | Applied where | When | Here |
|---|---|---|---|
| `camera_imperfections` (blur, rolling shutter, noise, ISP noise kept, JPEG q75, bitrate dips) | `src/houndkeeper/camera_imperfect.py` `Degrade`, called by `filament_cameras.py` on every head frame after rendering | after the render, on the published frame | CPU: applied again on the recorded frames |
| `head_camera_mount_error` | `sim_hook.set_head_camera`: the rendered camera sits off where DimOS says | render time | CPU: tonight's mount off the calibrated one; GPU: a larger fixed error, mapping nights too |
| `odom_drift` | `sim/dimos/env.py` -> DimOS `simulation/mujoco/odom_drift.py` (odometry, and the lidar placed in it) | run time, recorded in the odometry | CPU: more drift on the recorded odometry; GPU: from the start (`HK_ODOM_DRIFT_GAIN`) |
| `frame_delay_s`, `odom_delay_s` | DimOS `mujoco_process.py` (`DelayLine`, `OdomDelay`): stamped on arrival | recorded in the stamps | CPU: placement time shifted or jittered |
| `video_stream`, `stall_burst_share` | DimOS `stream_model.py` | recorded (frames missing) | CPU: more frames dropped and stalls |
| `lidar`, `lidar_dropouts` | DimOS `lidar_model.py` | run time; the maps of the mapping nights | GPU only |

How the worse version sits on top of a recording that already carries the nominal one (`sens.py`'s docstring):

| Variant | Layered as | Approximates |
|---|---|---|
| `jpegQ` | every head frame re-encoded at JPEG quality Q (75, 30) | a stream compressed harder all night (double compression: a little harsher than one pass at Q) |
| `noiseK` | independent noise added so the total is K x the nominal sd (extra sd x sqrt(K^2-1)), the nominal sd from the file at gain 2 (a night frame), 1 px blotches, 30% kept; then q75 | a noisier sensor or less noise reduction |
| `expS` | the frame smeared by the camera's own motion (odometry) over S seconds more (`Degrade`'s blur), q75 | exposure_max_s longer than 33 ms (night frames sit at the maximum) |
| `rsS` | S seconds more top-to-bottom skew (`Degrade`'s row warp), q75 | a slower read-out |
| `driftK` | DimOS's own `OdomDrift` run again over the recorded odometry, every spread x sqrt(K^2-1); the scorer's odometry columns moved alike | total drift K x the estimate |
| `lagS` | each frame placed S seconds later than the stored lag says | the real lag off the stored one by S |
| `jitS` | each frame's placement time jittered by S more | stamp jitter |
| `drop4` | 9.3% more frames lost singly, 6 more 0.6 s stalls a minute | the stream x4 |
| `mountP` | tonight's head camera P deg more pitched than the calibrated mount the baseline views were painted with | the head knocked, or its calibration off, between mapping and tonight |

Two limits of layering on the CPU: (1) the worse camera reaches only tonight's walk, not the mapping nights' views (on
the robot both would be worse; a mismatch is the harsher case for the comparison, a both-worse map the harsher case
for nothing in particular, so the GPU queue re-renders both); (2) the noise variants assume gain 2, while dark corners
run at higher gain in the sim.

## The table (six tuning nights; nominal = live replay)

| Variant (worse, tonight only) | Stage 1 found | Stage 1 false a night | Lead found | Lead false a night | Lead ordinary toys | Lead false a night at 42 found | Lead found at 11 a night |
|---|---|---|---|---|---|---|---|
| nominal (live, float32) | 45/48 | 53.8 | 39/48 | 11.2 | 37/44 | 14.8 | 39 |
| head camera 1 deg more pitched than calibrated (`mount1`) | 42/48 | 88.0 | 39/48 | **20.0** | 37/44 | 45.2 | 36 |
| frames placed 150 ms late (`lag150`) | 42/48 | 88.3 | 38/48 | **19.7** | 37/44 | 31.8 | 35 |
| odometry drift x4 (`drift4`) | 42/48 | **90.5** | 39/48 | 16.5 | 38/44 | **54.8** | 39 |
| JPEG q30 every frame (`jpeg30`) | 46/48 | 61.8 | 40/48 | 15.3 | 38/44 | 27.5 | 40 |
| sensor noise x4 (`noise4`) | 45/48 | 59.8 | 42/48 | 12.3 | 40/44 | 12.2 | 41 |

Per night, the lead's false alarms (s71_t2, s76_t2, s76_t11, s70_t1, s70_t11, s96_t1): nominal 7, 17, 11, 10, 9, 13;
mount1 11, 13, 17, 16, 30, 33; lag150 16, 18, 14, 23, 19, 28; drift4 10, 16, 11, 18, 14, 30; jpeg30 7, 20, 10, 15, 12, 28. House 96's dark tile takes
the largest share of every rise. Hard floor lost most under the lag (lead 14/18 hard against 16/18).

## Ranked: what the 94% depends on

| Rank | Estimate | Its value now | Made worse to | What moved | Breaks the bar? |
|---|---|---|---|---|---|
| 1 | the head camera's mount against its calibration (`head_camera_mount_error`, as calibrated) | calibrated per robot (selfcal); the rendered error 0.6 deg est. | tonight 1 deg more pitched than the map's views | stage 1 -3 toys, +64% false; lead false x1.8, x3 at 42 found; found at 11 a night 39 -> 36 | yes for false alarms; the 0.5 deg step is queued |
| 1= | frame lag behind odometry (`frame_delay_s` less `odom_delay_s`, as calibrated) | stored per robot, 0.15 s delay est. | the stored lag 150 ms off | stage 1 -3 toys, +64% false; lead false x1.8, found at 11 a night 39 -> 35 | yes for false alarms (close to 2x); the 50 ms step is queued |
| 2 | odometry drift (`odom_drift`) | 2% scale, 0.5 deg per root metre | x4 (8% scale, 2 deg per root metre, 4 deg start) | stage 1 -3 toys, +68% false; lead false +47%, x3.7 at 42 found | stage 1 yes; the lead holds found; the 2x step is queued |
| 3 | stream compression (`jpeg_quality`, bitrate dips) | q75, q30 for 3 s twice a minute | q30 every frame | stage 1 +15% false; lead false +37% (the dark tile 13 -> 28) | no for found; false alarms up a third |
| 4 | sensor noise (`shot_noise`, `read_noise`, `isp_noise_keep`) | 30% of raw kept, gain 2 at night | x4 sd | stage 1 +11% false; lead within the spread | no |
| - | lag 50 ms, mount 0.5 deg, drift x2, exposure, JPEG q50 and q75, rolling shutter, jitter, drops | | | queued (CPU), in that order | |
| - | lidar dropouts, a larger rendered mount error, compression on the mapping nights too | | | render time: GPU queue | |

Why the first three lead: the floor comparison places every frame on the floor through odometry at the frame's
moment, so a mount, lag or drift error smears tonight against the map exactly where a toy would show (more changed-looking
cells, more candidates); the crops are cut through the same odometry, so a toy can fall at a crop's edge. A noisier or
blockier frame changes the floor's colour a little but not where it lands.

## Daytime changes the sim does not have

The sim moves its chairs between nights (`movables.py`), but plants nothing else between the mapping nights and the
tidy night. On the CPU, the cascade round's control method (`../build/cascade/control.py`) with household
things instead of toys: 30 crops of candidates near no toy that the lead rejected (picked with the truth, analysis
only), each with a thing pasted on the candidate's point (rendered from the item library's scans on the CPU,
`day/render_sprites.py`, Mesa's software renderer; scale matched to the crop's ~960 px a metre; lit to the crop; soft
and JPEG'd), then the lead's check at its threshold (`day/paste_day.py`).

| Planted at the point (two views each) | What it stands for | Passed by the check (of 60) |
|---|---|---|
| nothing (the control) | | 0 of 30 |
| three shoes (sneaker, trainer, loafer) | shoes kicked off | 178 of 180 |
| a mug | a mug left down | 60 |
| a pet bowl | a bowl moved | 60 |
| work gloves | gloves | 58 |
| a folded dishtowel / a hand towel | laundry dropped | 60 / 28 |
| a hat | a hat dropped | 40 |
| two lunch boxes | a bag put down | 49 / 5 (one design passes, one mostly not) |
| a tall boot | a slipper | 7 |
| a lamp's light pool, +50% / +100% | a lamp left on | 0 / 2 of 30 |
| a shadow, -40% | a moved chair's shadow | 0 of 30 |

- **YOLO-E prompt-free calls most small household things objects**: shoes, mugs, bowls, gloves and a folded towel pass
  almost always. Things larger than the crop (a 30 cm boot standing, a bag filling it) pass less, because the crop
  (0.4 m of floor) shows only part of them. Light pools and shadows almost never pass: the check removes lighting
  changes, so a lamp left on costs stage 1 candidates, not false alarms.
- **How many false alarms a day's changes add.** Around one real object, stage 1 makes about one candidate spot and the
  check passes about one (toys of 100-300 cm2: 1.05 spots, 0.95 passed; over 300 cm2: 1.5 and 1.25; `day/per_object.py`
  on the cascade round's tuning nights). A typical day's leftovers in the robot's walk (two shoes, a bag, a towel or a
  sock, a mug; a lamp) is then about 2 x 1 + 0.5 x 1.5 + 0.7 + 1 + 0, **about 4-5 more false alarms a night**, on top
  of the 10 or so the sim measures, unless they count as finds: the item library already holds shoes and mugs as
  things to tidy, so whether a shoe by the door is a find or a false alarm is a decision about targets, not the detector's.
- **Caveat.** Pasted renders, no contact shadow, one lighting; a crude proxy. The check's behaviour on shoes is the
  firm part; the per-day count is an estimate.

**Proposed (not built): planting daytime changes in the sim.** Between night 2 (the second mapping night) and night 3
(the tidy night), `make_nights.py` would place a "day" layer: 2-6 household things that are not targets (shoes in pairs,
a bag, a towel or a jumper, a box, a mug), from a non-target list of the scanned library plus a few new scans (a
backpack, a laundry basket), on free floor where people leave things (by doors, the sofa, the table); a lamp switched
on or off (the lighting regime already toggles per night); a chair moved further than `movables.py`'s "a lot". The
truth would mark them `day` (not targets), and the scorer would report false alarms on them apart from the rest, so
"false alarms a night" keeps its meaning and the new number shows on its own. It needs a criterion ID and a decided
list of what counts as a target first.

## Queued for the GPU (not started)

`../build/sens/gpu/gpu_queue.sh` waits for "driver done" in `../build/r1280/progress.txt`, then
runs one render at a time: house 70, toy set 1 (the re-sweep's own `s70_t1_r1280` is the nominal), mapping nights and
the walk-through re-rendered end to end (`night_tidy.sh FRESH_BASELINE=1 MODE=walk`) in its own `RUN_DIR`
(`gpu/runs/<variant>/nights_s70`), `params/robot.yaml` overridden in every Python process through
`gpu/site/sitecustomize.py` (`HK_SENS_PARAMS`, tested on the CPU: the override shows in `params.get`) or the existing
`HK_ODOM_DRIFT_GAIN`. After each night, `gpu/post.py` runs stage 1 and the lead on it on the CPU (its near views made in
its own run directory; a shared one is copied first). Order in `gpu/variants.tsv` (re-read before each job; `touch
gpu/STOP` stops it between jobs); about 95 minutes of GPU each:

| Order | Variant | Override | Why |
|---|---|---|---|
| 1 | `drift4` | `HK_ODOM_DRIFT_GAIN=4` | the CPU's second-ranked; on the GPU it also reaches the mapping nights' stitch |
| 2 | `delay4` | `frame_delay_s` 0.3 s, jitter 0.12 | the CPU's first-ranked, with the robot's stored lag unchanged |
| 3 | `mount4` | `head_camera_mount_error` x4 (2.4 / -1.6 / 1.2 deg, 16 mm) | whether the self-calibration still takes it out |
| 4 | `jpeg30` | `jpeg_quality` 30 | compression on the mapping nights as well as tonight |
| 5 | `exp4` | `exposure_max_s` 0.133 | blur in the map too |
| 6 | `noise4` | `shot_noise` x16, `read_noise` x4 | noise at the sim's own gain, not an assumed one |
| 7 | `lidar_dark` | `lidar_dropouts` 15% albedo, 90% / 95% dropped | the costmap's band and the map |
| 8 | `stream4` | `video_stream` x4 | frames lost, with the burst stamps |

The nominal for all of them is the re-sweep's own house-70 night (`s70_t1_r1280`), put through the same `post.py`
(queued on the CPU behind the variants). One night a variant: a toy is about 6 points of found there, so the GPU
results say where it breaks, not by how much.

None has started; start it with `setsid nohup ../build/sens/gpu/gpu_queue.sh &` once it is approved.
(Ran 15 Oct: "On the GPU" below.)

## Day one: what to measure first

| Order | Measure | Procedure ([4hm]) | Why first | What "good enough" looks like (from this study) |
|---|---|---|---|---|
| 1 | the head camera's mount, and whether it moves between nights (a knock, a loose bracket) | [head_camera.md](../../real/calibration/head_camera.md) 5 (`calibrate_mount.py` on a real night), repeated on a second night | 1 deg off the map's calibration costs as much as anything tested | the same within a few tenths of a degree night to night |
| 2 | the head frames' lag behind odometry, and how much it jitters night to night | [timing.md](../../real/calibration/timing.md) 1-3: the clock on the screen, turning on the spot; `measure.py lag` | the largest effect here; the stored lag is all the pipeline has | known to well under 150 ms (the 50 ms step will say how far under) |
| 3 | odometry drift: scale, heading per metre | [odometry.md](../../real/calibration/odometry.md): the straight 5 m, five taped loops; `measure.py odom` | second largest; unmeasured on the Air | within 2x of the estimate (2% scale, 0.5 deg per root metre), or the floor lock must carry more |
| 4 | the stream's compression on the darkest floor | none yet: add a still minute and a walking minute over the darkest floor to [camera.md](../../real/calibration/camera.md), and log the WebRTC stream's bitrate | q30 everywhere costs a third more false alarms, most on dark tile | the real frames' blockiness on dark floor no worse than the sim's q75 with dips |
| 5 | effective noise | [camera.md](../../real/calibration/camera.md) 1 (the still minute) | x4 noise stayed within the spread | |

Day one's order in real/calibration/README.md already has the head camera marks (2), the clock (3) and the taped walks
(6); this study says do them before anything optional, and check the mount, lag and drift twice (two nights) before trusting the launch numbers on
the robot.

## The smaller steps (14 Oct 2026, evening), and day one's pass bands

The queue's next steps have scored since the table above (`v/<variant>/c/score_<variant>.json`; the same six tuning
nights, live; read here without re-running `table.py`). Lead false alarms a night against the live nominal's 11.2:

| Variant | Stage 1 found | Stage 1 false a night | Lead found | Lead false a night | Lead found at 11 a night |
|---|---|---|---|---|---|
| head camera 0.5 deg more pitched (`mount05`) | 44/48 | 90.0 | 40/48 | **19.0** | 37 |
| frames placed 50 ms late (`lag050`) | 45/48 | 81.7 | 36/48 | **17.5** | 35 |
| odometry drift x2 (`drift2`) | 44/48 | 72.0 | 41/48 | **15.8** | 40 |
| JPEG q50 every frame (`jpeg50`) | 46/48 | 57.8 | 42/48 | 11.0 | 42 |
| JPEG q75 every frame again (`jpeg75`) | 46/48 | 52.8 | 41/48 | 11.0 | 42 |

- Half the mount step and a third of the lag step cost nearly as much as the full ones: the launch numbers break
  somewhere under 0.5 deg and under 50 ms. Twice the drift already costs 4.6 a night, so "within 2x" in the day-one
  table above is too loose: at or under the estimate. q50 costs nothing (a q50 pass over frames already at q75 is
  only a little harsher than q75), q30 does.
- Day one's pass bands ([GWP], real/calibration/README.md "Day one, in order", `measure.py`'s `DAY_ONE_BANDS`): mount
  within 0.1 deg (the nightly check's tolerance; fails at 0.5), lag within 20 ms (fails at 50), drift at or under the
  estimate (fails at x2), compression no flatter or blockier than the sim's frames at q50 (fails at q30; judged by eye
  on a side-by-side, since on house 96's dark floor q50 and q30 re-encodes move the measures by a few per cent, less
  than two houses' floors differ: real/calibration/camera.md 3), noise up to x4 (nothing larger run). Between a band and its failing step is untested and counts as outside. Not yet scored: exposure,
  rolling shutter, jitter (`exp100` scored 32/48 found at 6.2, a found loss, not a false-alarm one: unread here).

## On the GPU: each estimate made worse for the whole run (15 Oct 2026)

The queue above ran end to end: house 70, toy set 1, mapping nights and the walk re-rendered with each variant live,
then `gpu/post.py` on the CPU (live paint at tc 0.032, the lead at 0.2436). The nominal is the re-sweep's own night
(`s70_t1_r1280`). One night each, 8 toys: a toy is 12 points of found, so read direction, not rates.

**Story first: a knocked camera fills the night with false alarms; a hard-squeezed stream hides the toys on carpet;
drift bad enough stops the map being built at all. The rest barely moved.**

| Variant (worse from the mapping nights on) | Stage 1 found | Stage 1 false | Lead found | Lead false a night | Lost |
|---|---|---|---|---|---|
| nominal (`s70_t1_r1280`) | 7/8 | 88 | 8/8 | 14 | |
| mount x4 (2.4 / -1.6 / 1.2 deg, 16 mm; `mount4`) | 7/8 | 87 | 7/8 | **25** | 1 carpet |
| frame delay 0.3 s, jitter 0.12 (`delay4`): void, the delay never applied (below) | 8/8 | 73 | 8/8 | 13 | |
| stream x4 (12% dropped, 8 stalls a minute; `stream4`) | 8/8 | 73 | 7/8 | 12 | 1 carpet |
| noise x4 sd (`noise4`) | 8/8 | 54 | 7/8 | 13 | the rug's |
| lidar dark-floor dropouts (`lidar_dark`) | 8/8 | 37 | 7/8 | 9 | 1 hard |
| exposure 133 ms (`exp4`) | 8/8 | 62 | 7/8 | 6 | the rug's |
| JPEG q30 every frame (`jpeg30`) | **3/8** | 61 | **3/8** | 3 | all 4 carpet, the rug's |
| odometry drift x4 (`drift4`) | | | | | no map: refused |

- **Mount: the biggest false-alarm risk.** Stage 1 is unchanged; the rise (14 -> 25) is at the check, so the
  self-calibration takes out enough to place the floor, not enough to keep the crops clean. Agrees with the CPU's
  rank 1.
- **Compression: the biggest recall risk.** q30 from the mapping nights on loses the toys at stage 1, before the check:
  every carpet toy and the rug's, hard floor untouched. The CPU's q30 (tonight only, on 0.045's replays) cost false
  alarms instead; with the map squeezed too, carpet's texture and a toy's look alike. Day one: judge the dog's stream
  on carpet as well as the darkest floor (`DAY_ONE_BANDS["compression"]`, fails at q30, now on found too).
- **Drift x4 stops the map, by design.** The house frame gate refused both nights: each moved about 1.3 m off its own
  odometry over the night (limit 1.0 m), and night 2 sat 9 deg turned at the dock (limit 5). Nothing is built, so
  nothing is detected wrongly; the cost is a lost night, not false alarms. The day-one band (at or under the estimate)
  stands.
- **Minor:** noise, stalls, lidar dropouts and exposure each moved the lead by one toy or fewer and its
  false alarms down or flat. One toy is inside a single night's spread. Frame delay was not tested here: the next section says why.
- The nominal's lead counts a toy its stage 1 misses (8/8 against 7/8); unread here. Logs:
  `../build/sens/gpu/post_<variant>.log`, `night_<variant>.log`; drift4's refusal in
  `gpu/runs/drift4/nights_s70/shape/gate.json.log`.

## The sim never delayed its frames (found 17 Oct 2026)

**Story first: every sim night before 17 Oct had a perfect camera clock. With the real camera's lag in, a stale lag
costs the found; the right lag gets it back but not the false alarms: about 8 more a night on this one.**

The fork's MuJoCo loop built its frame delay as `DelayLine.from_spec(...) or DelayLine(0, 0)`; a delay line with
nothing in flight counted as empty, so the zero line was always chosen and frames were stamped when rendered. The
lidar, odometry and command delays were unaffected. So the launch numbers ([tTD], [LQs]), the sim's stored robot-clock
lag (-0.03 s: the odometry's delay alone) and `delay4` above were all made without the 0.15 s (jitter 0.03) the robot
file intends. Fixed in the fork (`__bool__`, `is None`; `test_delay_line.py`). The nightly lag estimate itself was
sound: a planted +0.15 s shift moved it by 0.150 s exactly.

Re-run on the GPU (house 70, toy set 1), each night judged with the stale stored lag and with the night's own:

| Frame delay | Lag used | Stage 1 found / false | Lead found / false a night |
|---|---|---|---|
| none (the bug; `s70_t1_r1280`) | -0.03 stored | 7/8 / 88 | 8/8 / 14 |
| 0.15 s, jitter 0.03 (`fd_nominal`) | -0.03 stale | 8/8 / 91 | 7/8 / 20 |
| 0.15 s, jitter 0.03 | +0.115 tonight's | 8/8 / 92 | 8/8 / 22 |
| 0.3 s, jitter 0.12 (`fd_delay4`) | -0.03 stale | 6/8 / 98 | **4/8** / 26 |
| 0.3 s, jitter 0.12 | +0.293 tonight's (gate loosened to 0.6) | 7/8 / 83 | 6/8 / 21 |

- **The real camera's lag likely costs false alarms the launch numbers do not carry**: 14 to about 21 on this night
  with the right lag. One night, so direction; the honest numbers' false alarms are likely low by a similar share.
- **A stale lag costs the found.** At the nominal delay the nightly check warned (165 ms off, over its 40). At 0.3 s
  with heavy jitter it could not measure (turning correlation 0.44 to 0.67, under the gate's 0.7) and said "no check",
  so the case that hurts most passes with no warning. Day one measures the lag ([GWP]; `measure.py lag` now searches
  out to 1 s and gives no number rather than 0 when the turn is too poor).
- At 0.3 s the delay line delivers 44% of frames on the same stamp as the one before (held behind a late one); a real
  link spreads them by a few ms. That bunching is likely part of why the estimate failed: partly a model artifact.
- Logs: `../build/sens/gpu/post_fd_*.log` (stale), `postlag_fd_*.log` (tonight's), `lag/`, `lagruns/`.

## Reproduce

```
cd ../build/sens
python3 runq.py                         # the queue (jobs.txt: 'replay V N', 'post V'); MAXJ file: at most this many at once
$PY sens.py replay <variant> <night> v/<variant>      # one night, stage 1, painted again with the variant
$PY sens.py crops <variant> v/<variant>/c v/<variant> # time-nearest crops, cut through the same variant
$PY sens.py verify v/<variant>/c                      # YOLO-E 11l prompt-free on the CPU
$PY sens.py score <variant> v/<variant>/c v/<variant> # stage 1 and the lead at 0.2436
$PY table.py                                          # the table
$PY day/render_sprites.py; $PY day/paste_day.py 30; $PY day/per_object.py
```
