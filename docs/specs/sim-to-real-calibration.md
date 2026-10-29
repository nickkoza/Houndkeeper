# Sim-to-real calibration

**Type:** Capability spec · **Status:** Draft · **Source:** principle [33f], [sJQ] · **ID:** [V3o]

## Purpose

Keeps the sim honest: every physical number the sims use comes from one place, says where it came from, and gets
replaced by a measurement as soon as the real robot can give one; and every scenario that gates a deliverable is run
on the real robot too, so the gap between sim and real is known.

## Acceptance criteria

1. [J6A] Every physical parameter the sims use lives in one place, the robot parameters (CALM node
   `robot-parameters`), labelled vendor, measured, sim or estimate, with its source.
2. [4hm] For each parameter still marked estimate or unknown, there is a written procedure to measure it on the real
   robot, and running it updates the file.
3. [esI] Each gating sim scenario has a real counterpart that reports the same measurements.
4. [KbT] A gap report compares sim and real for each such scenario, and it is refreshed whenever either changes.
5. [CCS] Real recordings and photos can be replayed through the perception scenarios, so perception is tuned on real
   floors as well as simulated ones.
6. [V35] **A floor map from real nights alone.** One or two real recordings build what the sim's baseline nights build
   (each night's floor picture, tall layer and views, the house's frame and shape, the stitched floor and the nights'
   disagreement) from what the recordings hold, with no sim truth or house plan; a measure that needs the truth is
   skipped and says so.
7. [Lzt] **A whole tidy night from its recordings alone.** The real robot's recordings (frames stamped on arrival,
   in the formats its recorder writes) carry a night end to end: the baselines to the house's frame and floor map,
   tonight's recording to the floor comparison's candidates and, when the robot has it, the light check on each, then
   to the morning log and the cockpit's list, with no sim truth, no sim-only file or topic and no sim render stamp
   read on the way; a step that cannot run without one stops and names it.
8. [vO9] **A real night scored without the sim's truth.** Found and false alarms on a real night are counted against a
   toy set laid out and marked on the house's floor map before the night, with the same rule for a find as
   the sim's scoring, by floor kind (hard floor, carpet, rug) and with the smallest toys (the 1 x 1 bricks) reported
   apart; no photo or mark holds a person, and no raw camera frame leaves the house.
9. [71R] **Every unmatched flag is judged by a person.** A flag near no marked toy is shown in the cockpit, at phone
   size, with its crop and place, and the user marks it a false alarm or a real change (something not laid out);
   only false alarms count against the robot, and the night's score is final only when none is left unjudged.
10. [iDT] **A real number the gap report can set beside the sim's.** A real night's score is reported in the sim's
   own metrics (share of toys found, false alarms a night, both by floor kind) from enough nights to say how far it
   can be trusted, so the gap report ([KbT]) compares sim and real detection for the same scenario.
11. [7Xn] **Real measurements re-run the sims.** When a parameter changes from estimate to measured (for example the
   robot's balance, mass or crouch height once the hardware arrives), every scenario that depends on it is re-run and
   its result is recorded next to the estimate-based result, so any decision made in sim (mount position, tip limits,
   battery coverage) is checked against the real robot's numbers.
12. [jFy] **A find-and-flag night on the dog alone.** Before the arm arrives, one command on the laptop runs a real
   night: the robot walks the route made from its own baseline recordings, records the night, and by morning the
   floor comparison and the light check have run on that recording and the morning log and the cockpit's list are
   written beside it, all from what the robot sensed, with no sim truth, sim model or sim-only file; every step that
   needs the arm is skipped and the log says so; the command refuses to start when the stops, the robot network's
   isolation or the house's frame are not in order, and says which.
13. [GWP] **Day one says whether the sim's launch numbers still hold.** The measurements the launch false alarms lean on
   most are taken first, in the order a sensitivity study ranked them (the head camera's mount and whether it stays put
   between nights, the frames' lag behind odometry, odometry's drift, the stream's compression on the darkest floor,
   the noise), each with what to run, how long it takes, a pass band (how far it may sit from the sim's value before
   the false alarms rise past the replays' own spread) and what to do when it is outside; the mount, lag and drift are
   checked again on the second night, and a mount that moves between nights by more than its band is flagged.

## Decisions

1. [hOy] **A real night is scored by laid-out toys, matched by place** (proposed, 13 Oct 2026). Before the night the
   user lays out a known set and marks each toy on the house's floor map in the phone cockpit (a tap per toy, its name
   from a short list), optionally with one photo of the floor taken with no person in it; the photo stays on the laptop.
   Each morning the robot's candidates (in the house's frame, where the floor map is drawn) are matched to the marks: a
   toy is found when a candidate lies within the sim's reach of it (30 cm, or 15 cm past half a long toy's length: the
   sim scorer's `NEAR_M` and `reach`), candidates within 30 cm of each other are one (`MERGE_M`), each candidate finds
   at most the toys within its reach. The tap is itself off by a few centimetres and the map by the stitch's error (1-3
   cm in the sim), well inside 30 cm. Considered: reviewing every flag with no layout (no misses can be counted: a toy
   never flagged is never seen); fiducial tags on each toy (changes what the robot sees); a ceiling camera (raw frames
   of the house, people in them).
2. [wTj] **Five laid-out nights make a first number, ten a launch number** (proposed, 13 Oct 2026). The sim's launch
   point ([tTD]) is 45 of 48 toys over six nights, 8 toys a night. A night of 8 laid out toys gives each night's found
   share in steps of 12.5 points; over five nights (40 toys) 94% found has a 95% interval of about 82-98%, over ten
   (80 toys) about 87-98% (Wilson; the sim's own six nights, 45/48, about 83-98%). Each night lays out 8 toys like the
   sim's set: 2 bricks among them (1 x 1 or 2 x 2), the rest everyday toys of mixed size; across the nights, each
   floor kind (hard floor, carpet, the rug) gets at least a quarter of the toys, and the layout is moved between nights.
   False alarms a night are counted on every night, laid out or not, since they need no marks. Reported per night and
   pooled, as the sim reports them. The first two real nights set the light check's threshold and are not counted in the launch
   number; the nights after them are scored with it unchanged (proposed, 14 Oct 2026), so the number is not tuned on
   the nights it reports.
3. [Vd3] **Photos and crops stay in the house** (13 Oct 2026; principle [zGo], [JHu]). The layout photo, if taken,
   and the crops shown for review come from the house's own machines only: the photo is taken when no one is in the
   room, never sent anywhere, and every crop goes through the people screen as the morning log's do ([hKP]); the score
   itself (counts, places, kinds) carries no image and may be shared.
4. [oWO] **A real night's score is kept as files in the night's own directory, marked and judged from the project
   service** (14 Oct 2026). Conventional: the night launcher, the scorer and the cockpit must all pick the same place.
   - The real nights live under one root (`HK_REAL_NIGHTS`, default `data/real_nights/`), one directory per night named
     by the date its evening began (`YYYYmmdd`; before 6 am counts as the evening before). The night writes its
     candidates there (`candidates.json` and `night/`, as `sim/house/real_tidy.py` lays them out); the marking page
     writes `marks.json` before it; the review writes `review.json` (a judgement per candidate) and `result.json` (the
     scorer's night); the flags' crops are `review_crops/` (`real/tests/flag_crops.py`, people-screened, [Vd3]); every
     save and judgement is a line in `score_actions.log.jsonl` with its time ([Dlx]). No database: files beside the
     night, as the sim's runs keep theirs.
   - Marks are refused once the night has run (its `candidates.json` exists): they are made before the night, never
     after its flags are seen.
   - A night's place in the order of nights that ran decides its role ([wTj]): the first two are tuning nights (each
     result says `tuning`), and pooling reports them apart from the launch number.
   - The two pages (Marks and Review) are project pages ([C3p]): on the laptop's project service, so they work with the
     robot docked or off. The marks are taps on the house's floor map (the Area page's, `operating_area_module.
     house_map_image`), in the house's frame; the name comes from a short list that carries each toy's length (for the
     sim's reach) and whether it is a 1 x 1 brick (reported apart), and the floor kind is picked per toy.
   - Considered: marks stored with the robot (they would need the robot up before the night), a database (a new
     dependency for a handful of small files), judging on the laptop (not phone-first, [Yj2]).
5. [qDh] **What a real night checks before it starts** (14 Oct 2026, for [jFy]). Conventional: each check reads one
   file the house already keeps, so the night and the bring-up agree on what "in order" means.
   - The stops ([U4p]): `real/local/stops.json` (gitignored, written by hand from first-boot.md §3's record), each of
     the two stops (`without_dimos`, `through_dimos`) with the date it was tested lying down and standing, and the
     cockpit's dead-man (`deadman`) with the date it passed; a stop missing any of them refuses the night.
   - The robot network and the disk: the recorded session's own preflight ([6xc]: an isolation check passed within
     the hour, `real/local/isolation.json`, and 50 GB free where DimOS records), asked before the walk, not after.
   - The house's frame and floor map: one run directory built by `sim/house/real_nights.sh` from the two baseline
     recordings, `data/house/real/` unless `HK_HOUSE` names another: both nights' recordings present, their pictures,
     views and near views, the camera's mount, and the frame's gate (`shape/gate.json`) passed for both nights. It
     holds no sim truth file.
   - The operating area ([JVr]): the tidy refuses a real run without one ([Qd5]), so the night refuses first.
   The sim robot standing in for the dog skips the stops and the robot network (they are the dog's) and says so.
6. [q66] **A real night's log is kept with its night, and the night names its recording** (14 Oct 2026; resolves
   morning-log [t0a] against the Morning page, with [oWO]'s layout). A night run by the dog alone ([jFy]) is the
   night directory [oWO] names (`data/real_nights/<YYYYmmdd>/`, or `HK_REAL_NIGHTS`): tonight's decisions, the list,
   and `night/` with the morning log, the walk's own report and log, the route, and `recording.txt` naming the
   cataloged recording (`data/recordings/real/<id>/`), whose catalog note names the night in turn; so the catalog finds
   the night from its recording and the night its recording, as [t0a] asks, without writing into the data store's
   recording folders. The Morning page reads the newest night across the sim's tidy runs and the real nights, ordered
   by when each began, unless `HK_MORNING_RUNS` names one place. The sim robot standing in for the dog keeps its nights
   in the rehearsal's own directory (`HK_REAL_NIGHTS` set there), since its recordings are sim ones.
   A night enters the root only once its walk has ended: until then its work is kept beside the root
   (`<root>.work/<night>/`), so a refused, failed or crashed night leaves nothing the scorer would count or take for
   one of the two tuning nights ([wTj]); it is kept there, marked aborted.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [J6A] (one labelled parameter file) | `params/robot.yaml`; no hard-coded parameters in sim | Partial: file exists, sims not wired |
| [4hm] (measurement procedures update the file) | `real/calibration/` (README.md lists each parameter's procedure; `measure.py` reads a recording and `--write`s; `tests/test_measure.py`) | Partial: a procedure for every `go2_air` estimate, unknown and sim value (head camera, timing, odometry, camera, lidar, power and pose) and the gripper pads; the stream's compression on the darkest floor since 14 Oct 2026 (camera.md 3, `measure.py compression`: JPEG q95, 75, 50 and 30 told apart on a made-up dark floor, `test_compression_measure_tells_jpeg_qualities_apart_on_a_dark_floor`; run on sim houses 96 and 70, where q50 and q30 re-encodes of the sim's frames move the measures less than the two floors differ, so it is judged by eye on a side-by-side picture, not by a number); the analyses pass on known inputs and give back the sim's own heading drift and frame lag on a sim night; not yet run on the real robot |
| [esI] (real counterparts) | `real/tests/` | Partial: detection's counterpart drafted (a real night scored by its marks, [vO9]-[iDT]: `real/tests/score_night.py`); the others not yet written |
| [KbT] (gap report) | `real/calibration/measure.py gap` (beside `sim_reference.json`, made by `measure.py simref` from existing sim nights); `tests/test_measure.py` | Partial: the report recovers known gaps between a sim and a perturbed sim (synthetic frames, noise, scans and odometry; a house 95 night with late, dropped and noisier frames and thinned scans), and runs on two different sim nights (3 Oct 2026: `test_gap_report_on_two_different_sim_nights_reports_each_nights_own_values`, house 70 night 1 against house 95 night 1, each side's values that night's own; the recordings are found beside the DimOS checkout and the tests skip without them, so this evidence is on this machine only); the sim's reference made from eight nights (houses 95, 70, 72, 77); no real recording yet, and refreshing it when either side changes is by hand (the report flags a reference made under other parameters); the compression on the dark floor and the mount night to night are set beside the sim by their own commands (`measure.py compression --sim`, `measure.py mount`, [GWP]), not yet rows of the report |
| [CCS] (real data replayed through perception) | real photo tests | Partial: Cheerio photo tests |
| [V35] (a floor map from real nights alone) | `sim/house/real_nights.sh`; `tests/test_real_nights.py` (no-truth helpers, the nights' disagreement without truth); sim nights replayed with their truth hidden | Partial: house 95's two baseline recordings, copied without their truth, rebuild their sim run's floor map (3 Oct 2026: floor pictures 0.17-0.19% apart by the detector's own test, nights' disagreement 3.02% against the sim run's 3.10%); not yet run on a real recording |
| [Lzt] (a whole tidy night from its recordings alone) | `real/tests/rehearsal.sh` (a sim house's recordings copied without truth through `sim/house/real_nights.sh` TONIGHT=..., then `real/tests/rehearsal_score.py`); `tests/test_real_nights.py` (the list from the decisions and the check's verdicts; the morning log and the phone's message from a night with no traj.csv, truth or catalog entry; the live tidy's camera pose without MuJoCo); `tests/test_truth_boundary.py` (real_tidy.py reached as robot code) | Partial: 13 Oct 2026, house 70 from bare recordings (no truth): 8/8 toys at 41 false alarms, against the sim replay's 8/8 at 43; the marking protocol with the truth standing in for the marks gave the sim scorer's numbers exactly; a one-night mount was flagged by the nightly check; the light check now runs on the recording at the launch setting (14 Oct 2026: 49 places to 13); the real robot's launcher is [jFy]'s; the live tidy at the launch setting runs the near grid as the replay does (detection-baselines [BW3]) |
| [vO9] (a real night scored without the truth) | `tests/test_real_nights.py` (marks matched by the sim's reach, floor kinds, bricks apart); `real/tests/score_night.py`; the rehearsal with the truth standing in for the marks; `tests/test_real_night_scoring.py` (the Marks page's side, `houndkeeper.night_scoring`: marks from the short list in the scorer's shape, refused off the list, when stale or once the night has run, logged; the browser smoke `tests/browser/score_pages_390.mjs`: a toy picked, its floor picked, its place tapped and saved through the confirm dialog at 390 px); `tests/js/hk_marks.test.mjs` (taps in the house's frame by the Area page's transform, the count, the save command, a run night closed, nothing moving) | Partial: the scorer and the Marks page pass (14 Oct 2026: 19 Python, 11 page tests, the smoke against a loopback relay and the project service on a drawn map); not yet used before a real night, and the optional layout photo not built |
| [71R] (every unmatched flag judged) | `tests/test_real_nights.py` (judgements counted, an unjudged night refused when pooled); `tests/test_real_night_scoring.py` (the Review page's side: each flag with its crop and place, a judgement written for every candidate in the flag and logged, final only when none is left, taken back, the message bounded; the flags' crops from the frame nearest the decision, never kept when the people screen does not clear them or the recording is not cleared, `real/tests/flag_crops.py`; the smoke judging a flag at 390 px); `tests/js/hk_review.test.mjs` (from `tests/fixtures/review_message.json`, drawn map and crops: flags left first, the judgement command, the headline, the tuning note, a withheld picture said) | Partial: passes on synthetic nights (14 Oct 2026); `flag_crops.py` not yet run on a recording (its view choice and crop are the bench's, tested on a synthetic camera) and not yet called by the night launcher |
| [iDT] (a real number beside the sim's) | `tests/test_real_nights.py` (pooled in the sim's figures with the found share's interval); `tests/test_real_night_scoring.py` (the first two real nights kept apart from the launch number, [wTj]) | Partial: pooled figures pass; not yet in `measure.py gap` |
| [7Xn] (measured parameters re-run dependent scenarios) | suite re-run triggered by params change; history shows both | Not yet written |
| [jFy] (a find-and-flag night on the dog alone) | `real/night.sh` (preflight `houndkeeper.real_night`, the route, `hk-data record` of DimOS with the tidy at `HK_ARM=none`, then `sim/house/real_nights.sh` TONIGHT=... with the light check, `real/tests/flag_crops.py`); `real/tests/night_standin.sh` (the sim robot standing in); `tests/test_real_nights.py` (the refusals named, each stop and the dead-man, the house's gate and no truth, the stand-in's skipped checks, the launcher stopping before any session, a night failing before its walk ends leaving nothing in the nights' root, no sim file named, the walk without the arm, the night's layout and its log saying the arm was skipped); `tests/test_truth_boundary.py` (`real/night.sh` and `real_night.py` as robot code) | Partial: tests pass (14 Oct 2026); the half after the walk rehearsed on house 70's walk night (14 Oct 2026: the light check took 49 places to 13, 7 of 8 toys at 6 false alarms after the fact); the walk itself (DimOS, the stand-in sim robot) not yet run: the machine was loaded and its GPU taken |
| [GWP] (day one says whether the launch numbers hold) | `real/calibration/README.md` "Day one, in order" (the five first: what to run, how long, the band, why, what to do outside), `real/bringup/day-one.md`; `measure.py`'s `DAY_ONE_BANDS` and its verdicts (`mount`, `lag --expect`, `odom`, `noise --sim`; `compression --sim --sheet` set beside the sim's q50 and q30); `tests/test_measure.py` (the bands' pass, outside and fails; a mount moved 0.5 deg between nights flagged and a few hundredths passed; the tuning houses' own nightly checks pass and fail with 0.5 deg planted; the compression set beside the sim's frames at q50 and q30 with its side-by-side picture; only the dark floor read) | Partial: written and passing (14 Oct 2026); bands from the sensitivity study's steps (mount 0.5 deg, lag 50 ms, drift x2, q30 fail; q50, noise x4 pass; under each failing step untested, so outside); the compression has no automatic verdict (judged by eye: q50 and q30 move the sim's dark-floor measures less than two floors differ); the nightly check on one night's views alone not yet run on a sim house; not yet run on the real robot |
