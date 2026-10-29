# Look closer

**Type:** Capability spec · **Status:** Draft · **Source:** deliverable D3; principles [eNx], [zGo] · **ID:** [JUW]

## Purpose

When the floor comparison flags something, the robot stops, lies down, and uses the arm's wrist camera to see it up
close: where exactly it is, which way it lies, whether it can be gripped, and what it is, so real finds are picked and
false ones are dropped without anyone looking.

## Acceptance criteria

1. [hwo] From a flag up to 10 cm off (proposed; confirm), it re-locates the item to within the grip's tolerance.
2. [pYZ] It reports whether the item fits the gripper and the bin (its footprint inside the bin's 30 x 22 cm, with
   room for the measure's error), and if so the direction to grip across its narrowest side.
3. [ojR] It names the item with a model running on the robot's own compute, or says "nothing" when there is only floor.
4. [LJm] Shadows, marks and floor texture are not items.
5. [snp] It never sends an image anywhere; the close-up is kept only as a review picture on the house's own storage.
6. [yau] It removes a known share of false finds while keeping nearly all real ones, both measured on real carpet.
7. [TBu] An item is judged and gripped from a view of the whole of it. A look that sees only part of the item, cut
   off by the image's edge, doesn't decide its width, grip or name: the robot looks again, aimed toward the cut, and
   an item it can't see whole goes to review.
8. [GMs] With the wrist stereo pair ([A8K]): the item is the region standing above the floor that holds what the
   detector found, when that region is much bigger (the detector boxed a part: a handle, a head, a puzzle piece); its
   size for the bin is the largest any look saw (a look cut off by the frame only under-reads); and an item too wide
   for the jaws is gripped by one part only where both fingers have floor to land beside that part (a leg, a handle,
   a ring's wall), never beside a bowl's rim or a box's corner. An item no look sees whole (which [TBu] sends to review)
   is measured by the parts the looks did see, a lower bound, and reported too big if even that won't fit the bin (the
   pair's narrower frame cut most too-big toys off in every look, 29 Sep). The close looks are framed for the camera
   fitted: standing further back and higher by tan(65/2) / tan(its vertical view / 2), so it covers the floor the stock
   camera did (at most 45 cm up); unscaled, the pair's 50 deg view cut even crayons off the frame and a paired grasp
   bench lost 18 of 48 toys as unseen (6 with the stock camera). The middle path was taken, with not much
   more time on too-big and awkward items. On fresh landings (the full-resolution too-big and near-limit benches,
   replayed offline): 27 of 41 too-big trials called too big (the robot's own rule, rescuing by any
   part: 17), 2 of 23 grippable ones refused (0-1).

## Failure behavior

- If the item cannot be seen, or the model is unavailable, the find goes to review; the arm never grasps blind.

## Decisions

1. [pTW] **Re-location error so far (sim).** In the grasp scenario on 2026-09-24, the wrist-camera refine put items
    0.2-3.3 cm from their true position, from starting guesses about 5 cm off. Sim only; real photo pairs not yet run.

2. [MFH] **Too wide only if two measures agree** (28 Sep 2026). The width is measured on each look's mask laid on the
   floor; a standing toy's upper parts land further along the line of sight and, nearer the camera, look bigger, so
   it reads wide (a pony and a Transformers figure, grippable by their tiers, refused on live sim nights). A find
   measured too wide is measured again with the masks laid at the height the looks triangulated it at, and is too
   wide only if that also says so. On 47 saved looks, replayed offline: wrongly too wide 0 (the flat measure alone:
   3), wrongly attempted 7 (6). A wrong attempt costs a failed grasp; a wrong refusal leaves the item on the floor.
   (`pick._width_at_height`.) A third opinion follows when both still say too wide: gripped by one part
   (`wrist_refine.part_grip_options`): grasp points spread over the footprint, and across each only the run of the item
   under the point, split where a finger-wide gap (2.5 cm) opens, kept only where the part keeps its width within
   1.2 cm 1.5 cm either side (a leg, not a box's corner or a tapering hand), preferring parts nearer the middle. A
   Transformers figure lying with its legs apart (6.3 cm grippable by its tier) measured 13.6 flat and 15.0 at its
   height: every strip through its middle spans both legs; by one part it fits at 3.6-3.9 cm, 11 cm from its middle.
   On the 47 saved looks: wrongly too wide still 0, wrongly attempted 10 (7 with two opinions; on the looks' older
   truth, the narrowest extent of the whole item, so some of those may be grippable by a part). Live (28 Sep, night
   70/3), the figure was still refused at its first stop: the looks there didn't fuse, and that path returned the first
   look's flat width with no second or third opinion; two later stops found it fitting by one leg (3.7, 3.8 cm) but
   were skipped as the same object. Now the unfused path has the same opinions, and an object refused as too wide is
   tried again by a look that finds it fits (`TidyArm._handle_find`, `too_wide`). Rerun of night 70/3 with both: the
   figure's looks fused (3/3) and fitted, yet it was reported too wide, most likely from a re-look inside the grasp
   retries (`pick_with_tries`' relook), whose notes aren't kept (open: record the relooks' notes; give the relook the
   same width opinions and the standing-look hold). Done the same day: each re-look's notes are kept (`relooks` in the
   report), the re-look is held within 10 cm, and a re-look calling the item too wide after an earlier look found it
   fits keeps the earlier fit: the item didn't change size between tries (`pick.pick_with_tries`).
3. [L1v] **Why a look must see the whole item** (28 Sep 2026, for [TBu] and sim-slice [82p]). The close looks put the
   find in the upper image, clear of the jaws (`pick.IN_VIEW`), which leaves little frame beyond it. On night 70/6 an
   elephant lying pointing away was handed on by the standing look at its box's foot, its tail; every close look saw
   the tail and a hind foot, the body off the frame's top ("a shoe"), and it was gripped 2.3 cm across by the tail
   (whole: 4.7) and swung out of the jaws on the way to the bin. On night 71/1 a pony standing upright ran off the top
   of all four close looks; gripped by one leg it didn't lift, and backing out dragged it into the robot's side. Both
   were seen whole from standing.
4. [c1X] **The whole toy's box, not its part's** (29 Sep 2026, for [pYZ]; behind `HK_WHOLE_BOX` until measured). On
   the too-big bench (`width_bench.py`, 22 toys too big by sim-slice [QyZ]) the detector's box nearest the flag was
   often a part: a chunky puzzle's piece (the 31 x 24 cm board measured 7 x 5 cm), a school bus's wheel (3 x 2 of
   24 x 16), the cart turtle's head (6 x 4 of 35 x 28), so the robot tried what it should have reported. The box the
   robot measures is the largest detection holding 80% of the chosen one and at least 1.5 times its area
   (`wrist_refine.whole_box`); judged offline on the saved looks for too-big toys reported and grippable ones
   (shoes above all) not refused.
5. [xzk] **Future improvement: a close-range depth camera at the wrist**, marked for future improvement while the
   project goes as far as it can with the hardware it has. Deferred, not adopted: the
   look stays camera-only on the arm as it is. Why it is expected to help: the too-big bench showed the size questions
   are geometric ones asked of a picture model: the detector's box was often a part ([c1X]), tall toys read narrow
   laid flat at a guessed height (a 12.4 cm treat dispenser measured 6 cm), and a patterned floor can't be told from
   something on it. With depth, the item is whatever stands above the floor as one blob, whatever its texture: its
   whole outline, height and grip width measured, not inferred; the detector would still name it. What makes depth
   work without texture is light the sensor sends out, not the second lens: time of flight (each pixel times its own
   returning light), structured light (a known pattern projected and read back), or stereo with a dot projector
   painting texture on plain surfaces. Plain stereo, and the arm's own looks from several places, match texture and
   fail on a plain toy on a plain floor. Candidates (not chosen, specs to check when buying): a close-range time-of-
   flight module; a close-range structured-light or projector stereo camera (the RealSense D405, first named here,
   has no projector: close-range stereo on texture alone, so it is not the answer to plain toys); or a line laser on
   the wrist read by the wrist camera (laser triangulation: a few dollars, texture-free, one profile a look; but
   better no bare laser near people's eyes; Class 1 modules, as in phones' dot projectors, are eye-safe).
   Nothing may flash in visible light ([0Py], safe for photosensitive people): any
   pulsed light is infrared at 940 nm (850 nm glows faint red), and a steady visible light is allowed. First to trial: a ring of 940 nm LEDs round a
   wrist camera that sees infrared, flashed one at a time (multi-flash imaging, Raskar et al. 2004): each casts a thin
   shadow beside whatever stands above the floor and none from a floor's printed pattern, so the shadows across the
   flashes give an item's true outline, whatever the floor. A few LEDs, invisible and eye-safe; outlines, not heights
   (a small Class 1 ToF module adds heights, if they prove needed). Also to try, cheapest of all: a steady light beside the wrist camera's lens, for less noise in a dark
   room and shadows falling behind the item where the camera can't see them. Multizone
   ToF (8 x 8) is too coarse for shape, and most
   depth cameras see nothing nearer than 25-30 cm. Before buying, trial it in the sim: the wrist camera's depth
   rendered with a noise model as unkind as the part ([x3f]: error growing with range, holes at edges, dropouts on
   dark or shiny surfaces), and a depth size measure benched against the camera-only one on the same looks. Adopting it
   changes the arm's sensors: an architecture decision and the model, then. The infrared trial's parts and model:
   waymark [6hI]; the depth options surveyed, parallax from the arm's own looks (failed) and a stereo pair of our own:
   waymark [On0].

## Tests

| Criterion | Test | Status |
|---|---|---|
| [hwo] (re-locate within grip tolerance) | grasp scenario refine error; real photo pairs | Partial: sim only; results under Decisions |
| [pYZ] (fits the gripper, grip direction) | narrowest-width rule tests | Partial: sim only |
| [ojR] (named locally or "nothing") | local VLM on sim and real crops | Partial: the local open-vocabulary detector (OWLv2) names each find in sim; real crops not tried |
| [LJm] (shadows and marks are not items) | sim and real photo tests; `pick.look_closer` asks a detection only one of its three looks saw to be seen again by a fourth look aimed at it | Partial: on 25 Sep (DimOS house night) one look of three named the edge of a wall's shadow "a plastic object" and the arm tried to grasp it; single-look detections now need that second sighting (4 of 172 front-benchmark trials were single-look, 1 of them a real pick) |
| [snp] (no image leaves) | network audit | Not yet written |
| [yau] (false finds removed, real kept, on real carpet) | real trials | Not yet written |
| [TBu] (judged from the whole item) | `tests/test_whole_item.py` (a mask cut off by the frame; looks re-aimed at the cut; a toy never seen whole goes to review); a DimOS night with `HK_WHOLE_ITEM=1` (round 12) | Partial: rule tests only; not yet run on a night |
| [GMs] (stereo: the whole item, the bin by the largest look, a part only with floor for the fingers) | `tests/test_wrist_stereo.py` (a boxed part grows to the standing item; fingers need floor beside a part); offline replay of the full-resolution benches, concluded 29 Sep (27 of 41 too big called, 2 of 23 grippable refused) | Partial: rule tests, offline replay and live benches (29 Sep; results in waymark 0011 [A8K]); the live rate is well below the offline one |
