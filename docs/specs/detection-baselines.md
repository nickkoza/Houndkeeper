# Finding toys: the floor comparison against plain detection

**Type:** Experiment spec · **Status:** Draft · **Source:** related-work notes (private; the missing baseline); build-or-depend [eK3] · **ID:** [tsz]

## Purpose

Published tidying robots find clutter by detecting objects in the current view; Barrymore finds it by comparing
tonight's floor with the empty nights'. The two have never been measured against each other. This experiment does, and
adds a frontier model as a ceiling, so the choice is made on numbers. Part of the project is keeping recognition
lightweight and easy, so a frontier model that works better does not by itself end this path; the hardest cases might
instead be handed to plain detection. The point is the tradeoff table, not a winner: what each costs, where each fails,
and whether a combination beats both.

## Acceptance criteria

1. [WD9] **The arms, on the same nights.** Each runs on the same recorded tuning nights (never a hold-out), the same
   frames, scored the same way against the sim's truth:
   - **Floor comparison** (today's: the floor pictures, the detector, the confirmer, [d8d]);
   - **Local detection**: OWLv2 through DimOS's wrapper, on the head camera's frames, with a stated prompt set ("toy",
     "sock", "ball" ...) and threshold, its detections placed on the floor by the same camera model and poses;
   - **Frontier model**: a hosted vision model (named before it runs: Anthropic's or OpenAI's) asked the same
     question on the same frames, its answers placed the same way;
   - **Combinations**: the floor comparison first, with the hard cases handed to local detection (and, as a further
     arm, to the frontier model), "hard" defined before results are seen (the confirmer unsure, a dark or patterned
     floor, a small item, the house 93-style floors).
2. [JQk] **The tradeoff table.** Per arm: found and missed by difficulty tier ([D4v]) and floor kind; false stops a
   night; time per frame and per night; GPU memory and whether it fits the robot's own computer later; money per
   night (hosted); what it needs (baselines, prompts, a network); and where it fails, with examples (crops, sim only).
3. [6hv] **A frontier model sees only sim images** in this experiment, and is used only once the model and a spending
   limit are named (principle [eNx]: external AI used on purpose and signed off; this spec is the named use;
   no house data). Each call is logged (when, model, frames, cost).
4. [tWE] **The floor comparison is not judged by recall alone.** Its reasons to exist (light: CPU, no prompts, no
   network; it sees anything new, not only what it is told to look for; it improves as the floor map improves) are
   measured too: an unknown-object set (items no prompt names) and the onboard compute each arm would need.

5. [80w] **Near-frontier recognition at a fraction of the cost: the target, set before the run**, since recognition at
   far less cost scales better. **Its scope is a floor the robot knows** (the specialization on which it can come close;
   the frontier model will always generalise better): its own house, after the empty baseline nights have taught it the
   floor. The frontier model needs no such learning and generalises to floors it has never seen; that is not the
   contest. The claim is always stated with its scope ("on the floor of a house it has learned"), and made only when the
   combination (the floor comparison proposes, local detection confirms, the frontier model sees only what both are
   unsure of) meets all three, measured against the frontier arm alone on the same frames:
   - **found:** within 5 points of the frontier model's share (on the 4 Oct subset: at least 88% against its 93%);
   - **false positives:** no more than 1 a night above the frontier model's (on that subset: at most 1.3 a night);
   - **cost:** the frontier model asked about no more than 5% of the frames the frontier arm needs, as single crops,
     everything else on the robot's own (or the laptop's) compute.

   Met first on the sim's tuning nights; the claim is said of the real robot only once met again on real nights.
   Until both, it is not said, and the findings say which part is missing.
6. [Zaf] **The robot checks each floor-comparison candidate itself, opt-in** (toward [tTD]'s launch bar). On a tidy
   night, each candidate the floor comparison proposes is looked at again by a local recognizer on crops of the frames
   nearest in time to its decision that saw its floor point, and the robot stops only for a candidate the check keeps
   at the threshold chosen on tuning nights, or for one no frame saw. The check reads nothing of the sim's truth, keeps
   each crop's frame time with its verdict, runs on the GPU when it can and on the CPU when it cannot, and keeps the
   same candidates as the bench's cascade when a recorded night is replayed through it. It is off by default.
7. [LQs] **A reported number is measured as the robot judges live.** A night replayed from a saved comparison decides
   exactly as the same night painted live with the same options; a replay that could differ (judged at lower precision
   than the robot runs at, or with options its saved comparison was not painted with) can still be looked at as a
   screen, but no reported number comes from it.
8. [BW3] **The robot judges with what the bench measured.** At a named setting the live tidy runs every layer and
   option the bench's replay runs at that setting, including the near-floor grid against the baseline nights' own
   near views, and decides as the replay decides: the same frames through the live tidy's confirmer give the
   replay's decisions. A house without the views a setting needs is said so, never silently judged another way.

## Decisions

1. [YKW] **Local detection first, the frontier model second** (4 Oct 2026). OWLv2 costs nothing new (it is in DimOS;
   build-or-depend entry 4) and runs at home; the frontier arm waits for a named model and budget (named the same day:
   [L9V]).
2. [o0E] **One placement and one scorer for every arm** (4 Oct 2026). A detection in an image becomes a floor point
   through the same camera model, lag and poses the floor comparison uses ([gJq], the robot's own calibration), so the
   arms differ only in what they see, not in how they are placed or scored.
3. [L9V] **The frontier arm runs on a Claude subscription, through Claude Code** (no API keys; the three baselines run
   first, all in sim). Claude Code agents on the subscription look at the sim frames themselves and answer in a fixed
   JSON form (each toy: a label and a pixel box), which the shared placement and scorer then treat like any detector's
   ([o0E]). The model is the session's own (Claude Opus 5.5). The quota is not free like the local arms, so it is spent
   with care and runs staged: a pilot of 20 frames, then a go or stop on its results; if worth it, 60-80 frames in all,
   chosen toward the hard cases, downscaled to about 768 px on the long side; no retries or re-runs without sign-off;
   the other arms scored on the same subset too. Its time per frame is recorded but is not a deployable figure: an agent
   reading files is not a production endpoint.
4. [8VL] **The night set and the frame set** (4 Oct 2026, `sim/house/baselines/nights.py`, `frames.py`). Six recorded
   tuning walk-throughs with toys down and a truth file: the detection bench's houses6r walks of houses 70 (toy set 1),
   71 (set 2), 73 (set 1), 76 (set 2) and 77 (set 1), and house 96's walk of 3 Oct (set 1, stamped on the robot's
   clock); 48 toys, both toy sets. House 72 is left out while its live tidy's misses are looked into; no hold-out
   (78, 80, 81, 100-119). From each night's head camera, 45 frames spread evenly over the frames with a pose (270 in
   all, inside [L9V]'s budget), saved as the camera delivered them (the fisheye frame), each with the camera's pose
   then (the robot's stored lag on the recording's clock, the calibrated mount, odometry's frame), the lens and the
   image size. The detectors see these frames; the floor comparison runs on its own stride (every 4th frame), which its
   evidence over frames needs, and local detection is also run at that stride for comparison.
5. [oR6] **The scorer's rules** (4 Oct 2026, `sim/house/baselines/score.py`, `place.py`). A box stands on its
   bottom-centre pixel, straightened through the fisheye's calibration and laid on the floor (z = 0) from the camera's
   pose; detections are placed out to 2.5 m (the floor comparison's own reach). Points become the sim's frame through
   the sim's path (scoring only), merge across frames within 0.30 m, and a merged candidate finds a toy within the
   tidy's own reach (0.30 m, or 15 cm past a long toy's half-length) of where it was dropped or where the walk left it
   (the robot bumps some toys as it walks: house 73's cart 0.9 m, its brick 0.36 m); one near no toy is a false
   positive. Tiers by footprint alone, fixed before any result: easy at 150 cm2 or more, hard under 50, medium between (the scanned toys
   have no single colour for [D4v]'s contrast). Floor kind under each toy: rug, carpet or hard floor.
6. [XEi] **Local detection's prompts and threshold** (4 Oct 2026, `sim/house/baselines/prompts.py`, `owl_arm.py`,
   `table.py`). OWLv2 (google/owlv2-base-patch16-ensemble, float32) through DimOS's wrapper with twelve prompts fixed
   before any result: toy, stuffed animal, ball, block, shoe, cup, card game, figure, toy car, doll, sock, book. The
   threshold is the best F1 (toys found against candidates near a toy) on house 70's night alone, ties to the higher,
   then applied to every night. A toy kind no specific prompt names is an unknown object ([tWE]): the aeroplane, the
   pull-along cart, the lacing sheep, the two dinosaurs and the bowl. Why the misses were mostly the prompt and not
   the detector: docs/research/detection-misses-2026-10-05.md.
7. [DZr] **The frontier model's question** (4 Oct 2026, `sim/house/baselines/prompts.py`, `frontier.py`). One fixed
   instruction with each frame: every toy or other loose object on the floor, not furniture, rugs, patterns, stains or
   shadows, as JSON only, `[{"label", "box": [x0, y0, x1, y1]}]` in pixels of the frame as sent. The frontier set is
   70 of the 270 frames, chosen from the sim's truth (which frames to ask about, never what the model is told): 60%
   hard (a hard-tier toy in view, a toy on house 70's slate, 71's patterned or 96's dark chequered tile, or a toy at
   the image's edge, partly in view), 25% easy, 15% with no toy in view, round-robin over the nights. The pilot is 20
   of them covering every night and stratum; the rest go in batches of about 20, one agent each. Frames are sent as
   JPEG (quality 85) no longer than 768 px (the head camera's 640 x 360 frames go at full size, scale 1.0); the scale
   is recorded and the answers' boxes are mapped back to the full frame, scored 1.0, into the same placement and
   scorer. Every arm is also scored on exactly that subset: the toys its frames see within 2.5 m, the candidates
   lying where they look, and the detectors' detections from those frames only.

8. [3hK] **The floor comparison loosened, a local recognizer to restore precision**. The floor comparison's thresholds
   were tuned to hold false flags down, which caps its recall; with a recognizer behind it, they can be lowered, the
   floor comparison becoming the recall engine and the recognizer the precision engine. Measured as a curve: the floor
   comparison at several sensitivities, each candidate checked by a local recognizer on crops of the frames that saw it
   ("is there an object here, and what"), against the frontier arm on the same frames. The all-local form of [80w] is
   met when found and false positives meet its numbers with the frontier model asked about nothing.
9. [En1] **The local recognizers tried, in order** (4 Oct 2026; build-or-depend [eK3], DimOS first): OWLv2 large (the
   same wrapper, bigger weights), YOLO-E prompt-free (finds anything, the unnamed kinds), Florence-2, and Moondream 2
   (a small vision-language model that can be asked what does not belong on the floor), all through DimOS's own
   wrappers, local, on the same frames and scorer as the first arms. Each is scored alone, as a detector, and as the
   recognizer of [3hK]. Models outside DimOS (Qwen2.5-VL, Grounding DINO) only if these fall short.
10. [H9w] **The floor comparison is to be the primary finder again; the detector its backup**, since the detector
   generalises less than the floor comparison. A change of direction from the post hoc lead (OWLv2 on every frame as the
   main finder): a named-object detector finds what its prompts name, while the floor comparison finds anything that was
   not there before, on floors the robot has mapped ([80w]'s scope). The work raises the floor comparison's own ceiling
   (37/43 on the subset at loose1) and cuts its noise at the source, before any recognizer; [3hK]'s recognizer and OWLv2
   stay behind it. Measured on the same frames and scorer; a gain counts only once confirmed on nights not used to find
   it.
11. [HGT] **Methods are compared by their curves, not by one operating point**. A raw count ("37/43 at 16.8") is one
   threshold's point; a method that looks worse there can be better at the threshold we would choose. Each arm with a
   score (the floor comparison's log-odds or its peak, OWLv2's and the recognizers' confidences) is swept over its
   threshold and reported as a precision-recall curve and, since a false alarm costs a stop, as recall against false
   positives a night (the free-response form, FROC); with average precision (AP) as the one-number summary and the
   operating point we would run marked on each curve. An arm with no score (the frontier model's answers) is a single
   point on the same axes. [80w]'s target is a point on these axes, so whether a curve passes through its corner is read
   directly. The threshold that sets the operating point is still chosen on tuning nights only.

12. [ms4] **The night set at the real camera's resolution** (5 Oct 2026, `sim/house/baselines/floor_diag.py`
   HR_TUNING, HR_CONFIRM; waymark 0008 [7cU]). Twelve walk-throughs recorded at 1280 x 720 (data/bench/houses_hr),
   each house's empty baseline nights swept again at that size in the experiment's own run directory, on tuning houses
   70, 71, 76 and 96 (no hold-out), toy sets 1, 2 and 11 (sets 2 and 11 carry the 1 x 1 brick). Split before any
   replay: tuning s70_t1, s71_t2, s76_t2, s96_t1, s70_t11, s76_t11; confirmation s70_t2, s71_t1, s76_t1, s96_t2,
   s71_t11, s96_t11, so house 96's dark tile has confirmation nights of its own. Scored as [8VL]'s nights are, on all
   frames (no frontier subset exists for them).
13. [Ot0] **The near floor compared again on a finer grid, opt-in** (8 Oct 2026, toward [H9w] and the 1 x 1 brick at
   the real camera's resolution, [ms4]). The floor comparison's grid is a setting of the comparison, not a constant:
   a finer grid divides the 1 cm one (each 1 cm cell split k x k), and every size the comparison uses in cm (the
   surround its colour is read against, the eroded edge of the seen floor, the smallest and the floor-sized blob, the
   slop check's window) keeps its size on the floor; the sizes given in cells (smoothing, slack, the slop check's
   reach) are that grid's cells. A second layer at 5 mm compares the floor nearer than 1 m beside the 1 cm layer,
   against the baseline nights' views painted on that grid; it places each frame as the 1 cm layer does (its floor
   lock), and a near find within the scorer's merge distance of a 1 cm find is that find. Off by default; on the 1 cm
   grid alone the comparison's output is unchanged to the byte.

14. [Btx] **Recall first: reach the frontier on found, then trim false positives**: more toys binned is worth more false
    positives, which are trimmed later. [80w]'s two numbers are not traded evenly: the operating point of each method is
    chosen first to reach the frontier arm's found share (40/43 on the subset, 93%), on tuning nights, and false
    positives a night are reported at that point; a change that raises found at more false positives is kept over one
    that cuts false positives at a loss of found. False positives are the next target once found is at the frontier,
    still on the same curves ([HGT]); the stops they cost are measured, not ignored.

15. [bFx] **The sim's head camera renders at the real camera's resolution by default**, the only realistic sim
    (principle [x3f]). The real Go2 head camera is 1280 x 720; 640 x 360 was the kinder sim, hiding the 1 x 1 brick (0
    found at 640, genuine finds at 1280 with the near-floor grid). `HK_CAMERA_PX` defaults to the real size; `640x360`
    stays opt-in for quick loops and is never what [80w] is reported on. Cost: the sim runs about 2.3x slower (a house's
    baselines and three walks 75-80 min against 30). Each shared baseline is re-swept at 1280 when its house is next
    rebuilt, under [01j]'s guard. With it: the near-floor grid ([Ot0]) proposes at [Btx]'s recall, and a light local
    check on each candidate ([3hK]) brings the false positives back down. Applied 12 Oct 2026 (`sim/dimos/env.py`):
    unset, `HK_CAMERA_PX` renders params `go2_air.head_camera_px`; `HK_CAMERA_PX=WxH` overrides it. The size enters the
    render stamp, so a shared baseline swept at 640 x 360 is stale against a default night and [01j] refuses it until it
    is re-swept (`FRESH_BASELINE=1`, archived first); an opt-in 640 x 360 night still matches it
    (`tests/test_head_camera.py`).

16. [tTD] **Launch at the cascade's numbers: about 94% found at about 10 false alarms a night**. The bar for launch is
    the near-floor grid plus the YOLO-E check with crops from the frames nearest in time: 45/48 at 10.5 false alarms a
    night on the fresh 1280 nights, once confirmed on nights it has not seen. [80w]'s 1.3 a night stays the frontier
    comparison, not the launch bar. Work turns from trimming false alarms to reproducing this result on the real robot:
    the cascade in the robot's own pipeline, its sensitivity to the sim's estimated parameters, and a way to score real
    nights. The bar is one named setting, not a default (13 Oct 2026): `confirm_stream.py --setting launch` (the replay,
    and `sim/house/real_nights.sh` through TONIGHT_OPTS) is the near choice above and the light check of [Zaf]; the live
    tidy selects it with HK_DETECT_SETTING=launch, near grid included ([BW3]). Measured honestly (14 Oct 2026, after
    [LQs]): the numbers above came from cached replays that judged at colour threshold 0.045, so the launch setting is
    the near choice at 0.045, painted live. On the six fresh nights, scored once with the threshold chosen on tuning by
    the rule set in advance (0.1107): 46/48 found (bricks 4/4) at 22.5 false alarms a night; at the robot's 0.2436,
    43/48 (bricks 3/4) at 11.7; stage 1 alone 46/48 at 49.5. At about ten a night the honest figure is 90% found, not
    94%. What launches is the algorithm; its threshold is set as real testing begins: the robot keeps 0.2436 until the
    first real nights set it (sim-to-real-calibration [wTj]).

## Tests

| Criterion | Test | Status |
|---|---|---|
| [WD9] (the arms on the same nights) | `sim/house/baselines/` (`frames.py`, `floor_arm.py`, `owl_arm.py`, `frontier.py`, `table.py`); `tests/test_baselines.py` (the placement and scorer on synthetic cameras and floors) | Partial: floor comparison and local detection on the six nights of [8VL], one scorer; the frontier arm prepared (pilot of 20, then 50 more), not run; combinations not yet. 4 Oct 2026: floor comparison (confirmed or unsure) 39/48 toys, 16.7 candidates near no toy a night (confirmed only: 29/48, 10.0); OWLv2 on the 45-frame set 41/48 at 0.5 a night (threshold 0.2 from house 70; 34/48, 0.0 a night when seen in 2+ frames), at the floor comparison's own stride 39/48, 1.2 a night (threshold 0.3). House 96's floor comparison and stride-4 OWLv2 lost 61% of its frames to the recording pose lookup on its robot-clock stamps (both 4/8 there; OWLv2 on the frame set, posed before that change, 8/8) |
| [JQk] (the tradeoff table) | `sim/house/baselines/table.py` (table.md, table.json, failure crops) | Partial: arms 1 and 2, full set and the frontier subset. 4 Oct 2026: floor comparison CPU only, about 2.5 s a frame and 20 min a night replayed (a machine at load 60), no prompts or network; OWLv2 0.74 s a frame on an RTX 2070 (float32, 12 prompts), 1.9 GB GPU allocated (2.5 GB reserved), 34 s a night on 45 frames or about 8 min at stride 4. Floor comparison weakest on carpet (16/22) and rugs (4/6), strongest on hard floor (19/20), most false positives on house 96's dark chequered tile (55); OWLv2 weakest on hard floor (14/20), missing the lego brick, the dinosaurs and the bowl Frontier arm added 4 Oct 2026, on the 70-frame subset (43 toys, weighted to hard cases): frontier 40/43 (hard tier 9/11, hard floor 17/19, carpet 17/18, unknown kinds 17/17), 0.3 false positives a night; floor comparison (confirmed or unsure) 34/43 (8/11, 18/19, 13/18, 12/17), 12.0; OWLv2 31/43 (8/11, 12/19, 13/18, 12/17), 0.3. House 96's floor numbers are damaged by the robot-clock pose lookup regression (being fixed). The combinations not yet scored |
| [6hv] (frontier: sim only, signed off) | the call log `frontier.py load` writes; the frame set is sim recordings only | Passing (4 Oct 2026): 70 sim frames in 4 batches (a 20-frame pilot, then 50 on a go), Claude Opus 5.5 through Claude Code subagents on a Claude subscription; about 119k tokens in all (~49k pilot, ~70k the rest); no real frame |
| [tWE] (more than recall) | `sim/house/baselines/prompts.py` (kinds no prompt names), `table.py` (found by named and unknown kinds; each arm's compute) | Partial: 4 Oct 2026, unknown kinds (18 toys: aeroplane, cart, lacing sheep, two dinosaurs, bowl) found 14/18 by the floor comparison and 14/18 by OWLv2 (its generic "toy" prompt), named kinds 25/30 and 27/30 |
| [80w] (near frontier at a fraction of the cost) | the combination arm against the frontier arm on the same frames: found, false positives a night, share of frames sent to the frontier model; and its all-local form ([3hK]): the floor comparison at several sensitivities, each candidate checked by each local recognizer ([En1]), the curve against the frontier arm; sim first, then real nights; `sim/house/baselines/` (`floor_arm.py --sensitivity`, `recognizers.py`, `recog_arm.py`, `recheck.py`, `cascade.py`), `tests/test_recognizers.py`, `tests/test_cascade.py` | Partial: all-local form measured, not met (sim, the combination with the frontier model not yet run). 4-5 Oct 2026, on the 70-frame subset against the frontier's 40/43 at 0.3 a night: the floor comparison alone at today/loose1/loose2/loose3 35, 37, 35, 35 of 43 at 12.0, 16.7, 25.7, 37.2 false positives a night (all frames, house 96 now placed: 42, 43, 41, 40 of 48 at 17.0, 26.0, 38.3, 58.0); checked by each of 9 local recognizers on crops of the frames that saw each candidate, the closest is loose1 checked by YOLO-E prompt-free large, 34/43 (79%) at 1.3 a night (all frames 38/48 at 2.3), noise met and found 4 toys short; Moondream 2 keeps every true candidate but leaves 8.0 a night (loose1: 37/43); no sensitivity's floor comparison reaches 88% on the subset, so found is the missing part. Alone as detectors on the subset: OWLv2 large 33/43 at 0.3, Moondream 34/43 at 0.7, Florence-2 34/43 at 1.7, YOLO-E 22-31/43. Post hoc lead, not a result: today checked by YOLO-E prompt-free large, united with OWLv2's own detections, 39/43 (91%) at 1.2 a night (all frames 46/48 at 1.8); to be set as an arm before fresh nights Close-up checker, 5 Oct 2026 (the study is concluded and its checker superseded by `cascade.py`): on 1984 standing looks from 37 confirm-mode tidy nights (houses 70 and 71, threshold on house 71, scored on house 70's 140 toy stops and 1646 empty stops), the live tidy's zoomed OWLv2 keeps 79% of toy stops at 4.7% of empty stops let through, Florence-2 base 74% at 4.7%, YOLO-E prompt-free large 66% at 5.0%, OWLv2 base / large 47% / 51% at 2.1% / 2.6%; Moondream (a yes/no question, 591 stops) says yes to 588, taking the gripper for a shoe; measured directly on house 70's 29 nights (live proposer and live check) 93/117 (79%) at 2.7 a night and 57 stops; projected onto the curve 64-68% at 0.6-0.8 a night, so the walk-over design is not met, found short by 9-10 toys of 43; multi-view voting on the head crops does not reach 88% (best in the noise target still 34/43 at 1.3). Post hoc lead, not a result: OWLv2 large alone on all 45 frames a night 39/43 (91%) at 0.5 on the subset's floor (33/43 on the subset frames alone), and with the floor comparison at today confirmed by the standing look, projected 40.5/43 (94%) at 1.1 a night. Floor comparison as the primary finder ([H9w]), 5 Oct 2026 (`sim/house/baselines/floor_diag.py`, `confirm_stream.py --view-noise`, `tests/test_floor_primary.py`): each cell's change threshold raised to 6 times how much the empty nights' own views of it vary; confirmed on five nights not used to choose it (the other toy set of houses 70, 71, 73, 76, 77) 34/40 at 8.2 false positives a night against 33/40 at 23.0 at loose1 (the 1 × 1 brick, 3 of the 40, never found by any variant); on the six baseline nights 44/48 at 8.0 (subset 38/43, 88%, at 4.8); the floor comparison alone still not met, the noise the missing part Log-odds filter per floor cell, 5 Oct 2026 (`confirm_stream.py --logodds`, `sim/house/baselines/curves.py` [HGT], `tests/test_floor_primary.py`, `tests/test_curves.py`): on the six baseline nights 46/48 (every toy but the two 1 x 1 bricks) at 5.3 false positives a night against view noise x6's 44/48 at 8.0 (subset 40/43 at 3.2, AP 0.87 against 0.83), but on the five confirmation nights 28/40 at 6.8 against 34/40 at 8.2 (AP 0.46 against 0.63): not adopted, view noise x6 stays; from the curves, at 1.3 false positives a night the floor comparison finds 70-84% of the subset, OWLv2 large 91% at 0.5 a night; the floor comparison alone still not met At the real camera's 1280 x 720 ([ms4], [7cU]), the comparison as tuned at 640 x 360: 39/48 (tuning) and 41/48 (confirmation) at 10.3 and 11.0 false positives a night, 2 of 8 1 x 1 bricks found (none at 640 x 360); on the six nights recorded at both sizes 38/48 at 10.3 against 43/48 at 5.8: to be retuned at 1280 x 720 before it is judged On the head camera's adopted calibration ([yFg], 7 Oct 2026: the bench's baselines rebuilt with the robot's mount, the floor rows' lag and each house's relief), view noise x6 on the five confirmation nights 35/40 at 8.0 false positives a night (AP 0.654) against 34/40 at 8.2 (0.625), on the six baseline nights 46/48 at 6.8 (AP 0.743) against 44/48 at 8.0 (0.737); view noise x7 33/40 at 6.8 (0.613), not taken; the floor comparison alone still not met On verified loop closures ([fxk], 11 Oct 2026: the bench's baselines rebuilt with each loop checked by the robot's scans), view noise x6 on the five confirmation nights 34/40 at 8.0 false positives a night (AP 0.685) against 35/40 at 8.0 (0.654), on the six baseline nights 46/48 at 6.5 (AP 0.743) against 46/48 at 6.8 (0.743); at the recall-first point ([Btx]: the strictest threshold finding 45/48 on the baseline nights) 4.5 a night there and 31/40 at 4.2 on the confirmation nights (7 Oct: 4.7; 33/40 at 5.2); the floor comparison alone still not met At the real camera's 1280 x 720 retuned ([ms4], [Ot0], [Btx]; 11-12 Oct 2026, v8 maps): chosen on the six tuning nights for the most found, the loosened comparison with the 5 mm near layer 45/48 at 46.5 false positives a night there and 47/48 (43/44 and all 4 bricks, 2 of them within 3 cm) at 47.3 on the six fresh nights, against view noise x6's 42/48 at 11.8 (AP 0.422 against 0.546); the loosening without the near layer 42/48 at 36.7 fresh (no gain); the frontier's 93% found at about 41 a night on its curve; defaults unchanged, the sim's camera default proposed at 1280 x 720; the floor comparison alone still not met The cascade at 1280 x 720 ([bFx], [3hK]; 12 Oct 2026, v8 maps): the near choice checked per candidate on three crops by YOLO-E prompt-free large (chosen on the six tuning nights for the fewest false positives at 45/48: 7.3 a night against the comparison alone's 46.5; YOLO-E small 18.8, OWLv2 base and large with "small object" unable to trim without losing a toy, two multi-view scores 37.5-38.8), judged once on the six fresh nights 41/48 at 10.3 false positives a night against 47/48 at 47.3 (AP 0.753 against 0.422): found short of [Btx]'s 93%, most lost toys out of their crops (views far in time from the candidate's decision, odometry drift); post hoc lead, not a result: views nearest in time to the decision 45/48 at 10.5 fresh (45/48 at 7.8 tuning); YOLO-E large 35 ms a crop on the RTX 2070, about 0.1 s a candidate; all-local [80w] not met |
| [Zaf] (the robot checks each candidate itself) | `src/houndkeeper/candidate_check.py`, `sim/house/confirm_stream.py --check`, `src/houndkeeper/tidy_module.py` (`HK_CANDIDATE_CHECK=1`); `tests/test_candidate_check.py` (views nearest in time chosen online as the bench chooses them, the bench's crop, the threshold, a candidate no frame saw kept, frame stamps kept, GPU to CPU fallback, off by default, the live tidy holding a candidate until scored), `tests/test_truth_boundary.py` (placed on the robot's side) | Passing (unit tests); replayed, not yet run live. 13 Oct 2026 (docs/research/cascade-onboard-2026-10-13.md): on the cascade round's six fresh nights the robot-side stage chose the bench's views for all 531 candidates; on the CPU 44/48 at 10.3 false alarms a night against the bench's 45/48 at 10.5 (GPU), every difference YOLO-E's own CPU-against-GPU scores (25 of 1563 crops; 4 candidates; the toy lost a credited 1 x 1 brick); as the live tidy runs it 44/48 at 10.7; the GPU replay queued. Those stage-1 replays were cached, and a cached replay judged at its paint's colour threshold (tc 0.045), not the stated 0.032 ([LQs]): painted live at 0.032, the six fresh nights give 47/48 at 35.5 a night at the threshold re-chosen on tuning (0.0504) and 44/48 at 12.2 at 0.2436 (39/48 on tuning) |
| [LQs] (reported numbers as the robot judges live) | `sim/house/confirm_stream.py` (`FrameCache`: float32, with its precision and its paint's options in a header, old caches read as float16; `cache_differs`), `sim/house/baselines/floor_diag.py` (`precision`, `reportable`: each replay records its precision; `arm_of`, `cascade.decisions` and `curves.floor_arm_scored` refuse a replay from a float16 cache or from a cache painted with other options unless `HK_CACHE_SCREEN=1`, analysis only); `tests/test_floor_primary.py` (a cached replay equals the live one at a knife edge, which a float16 cache fails; another option is reported as a screen; the refusal) | Passing. 14 Oct 2026: a real night (house 70, set 11) painted live and replayed from its float32 cache, 94 of 94 decisions identical; the cache 411 MB against 233 MB in float16 (docs/research/cascade-onboard-2026-10-13.md) |
| [BW3] (the robot judges with what the bench measured) | `sim/house/confirm_stream.py` (`options`: the replay's options at a setting, for the live tidy; `Layers`, `near_frame`, `near_decide`: the 1 cm and near layers paired as the replay pairs them), `src/houndkeeper/tidy_module.py` (`near_views`; the launch setting's near grid live); `tests/test_near_grid.py` (the live options equal the replay's at every setting; the pairing, decisions, keep-out and tall layer; a tidy at the launch setting builds the near layer on the baseline nights' own near views, and without a setting the 1 cm layer alone); `tests/test_near_views_built.py` (every stitch paints the baseline nights' near views, `sim/dimos/near_views.sh`, so a live night has them) | Passing (unit tests). 17 Oct 2026: tuning night s70_t1_hr (v8 baselines, analysis only), its first 400 judged frames through the replay's launch pairing and through the live tidy's confirmer: 92 decisions each, identical at every tick, both layers' evidence identical; before it the live 1 cm layer at the launch setting also ran without view noise |
