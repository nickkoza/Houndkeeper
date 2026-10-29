# House nights

**Type:** Capability spec · **Status:** Draft · **Source:** Phase 0 work; criteria.json (written before results) · **ID:** [dws]

## Purpose

Simulates many nights in the same house on DimOS, with litter, stains, a rug and furniture that change between
nights, and scores how well the floor comparison finds what is new. It is the honest scoreboard for detection.

## Acceptance criteria

1. [yyJ] Nights are generated from a seed, with items varied in kind, colour and size, carried over between nights,
   stains that stay, a rug, furniture that moves a little or a lot, and lighting that varies. The rug is down from the
   first night, in the baseline: a major floor change (a new rug) is a separate test, not part of toy detection's
   (reduced accuracy for a few nights after one is accepted). The rug stays where it lies from night to night: moving
   it asked too much of the detector; until 1 Oct 2026 it shifted a few cm and turned a few degrees each night. The
   house has no people in it: v1 isn't a cobot. DimOS's standing figure is left out of the scene (the fork adds it
   only with
   DIMOS_SIM_PERSON=1, for DimOS's own tests); it never moved in our runs.
2. [D4v] Each item is scored found, missed or never seen, by difficulty tier (easy, medium, hard) defined before any
   result was seen.
3. [EL6] Tuning uses only designated tuning nights; held-out nights are scored once, after the detector is frozen, and
   the record says so.
4. [5se] False alarms are counted per night, and those caused by moved furniture are counted separately.
5. [h3t] The night runs on DimOS's own mapping and navigation, at a stated sim speed.
6. [uCP] **Each house has its own layout**, so tuning does not fit one floor plan (make_nights.py had used the same
   layout every night). From seed 72 up, the seed also draws the floor plan: the
   house mirrored left to right or not, the furniture moved in its groups (the TV, sofa and coffee table together; the
   kitchen counter and island; the dining table; the play shelf; pieces against a wall stay against it), the stub walls
   and the entry nook moved, the rug with the living area. Every open cell is reachable from the dock, or the plan is
   drawn again. Seeds 70, 71 and 80 keep the plan they were run on. Everything that reads the plan (walls, furniture,
   the sweep route, chairs, clutter regions, the rug, the dock) takes it from the seed, never a copy.
7. [WTj] **Cabinets read as cabinets** (the kitchen island in the background of a render was one flat colour). The
   kitchen counter, the island and the TV console have door fronts standing proud of the
   carcass with thin gaps between them, a handle on each door and drawer (a drawer row on the counter), applied panels
   without handles on the island's seating side, and panelled ends. Their faces show the house's cabinet finish at its
   real scale on every side, the grain running up the doors, not one colour stretched across a face. The finish (paint
   over a faint grain, or stained wood) is drawn per house from the seed, and drawing it changes none of the house's
   other draws.
8. [jmf] **Furniture shows its material on every face** (it had looked drab and untextured). Tables, their legs, the
   sofa and the dining chairs and stool carry the house's wood and upholstery at real
   scale on their upright faces as well as their tops, not a colour stretched across the face. The house's carpet is the
   one it is said to be: house 70's is a low pile (it had kept the carpet first drawn, a dark cut pile, since
   27 Sep), in a warm neutral hue (`HOUSE70_CARPET_SRGB`).
9. [O7l] **Toys lie where a household leaves them** (on 1 Oct 2026 the robot tripped over a shoe at the start of a
   night: a couple of toys close together is realistic, all of them is not). A night's toys are spread over the floor
   the night's walk covers, most of them well apart, with now and
   then a few close together (a pair, a small cluster); never all of them in one patch, and the robot's first metre
   clear. Until 1 Oct every toy lay 1-3 m from the start, 0.6 m apart: all eight within a 3-4 m patch, every night.
10. [aj8] **Tuning spans the floors a hold-out can draw**, so a hold-out is of similar difficulty to tuning, not wildly
   harder (the second hold-out, 2 Oct 2026, was harder), and tuning runs on more floors. The six tuning
   houses had five hard floors and three carpets of the training set's 28 and 8, and only the four "home" carpets,
   while a held-out house draws any held-out carpet, woven and commercial ones included. Twelve more tuning houses
   (seeds 82-93, each its own layout) are each laid with a named pair of training floors, together every kind of
   carpet and most hard floors the training set has; the next hold-out house is seed 94 or above.
11. [bP1] **Carpet is worn along the paths people walk** (desire paths: that is almost always how carpet wears, and
   unless freshly vacuumed it shows them). Every carpeted area carries a few smooth, gently winding bands, across it
   from edge to edge and toward where
   people sit, where the pile is matted: a little greyer and darker, less textured, flatter in relief. They belong to the house,
   the same every night (a fixture of the floor, in the baseline like the rest of it).
12. [HuM] **The sweep route comes from the robot's own map**. The route's rows, its reachable cells and its clearance
   from furniture are planned on the lidar map the robot
   built (house_map.py's, or DimOS's costmap) on its earlier nights, from where its dock was seen; a first night with
   no map explores before it sweeps. The generator's plan is never read.
13. [1B2] **Nothing on the robot's side reads the generator's truth.** No code that runs as the robot (the night's
   modules, the route, the floor pipeline, the confirmer) imports the house generator or opens its truth files; only
   the generator, the scorers and the analysis tools do, and a test enforces the boundary.
14. [ejh] **Hold-out houses are wholly unlike the tuning houses**. A hold-out house shares no floor plan with any
   tuning house: not the base plan, not a mirrored or
   rearranged variant of it. Its rooms, walls, doors and furniture placement are drawn by a generator whose plans the
   tuning houses never use, and a test measures that no hold-out plan is close to any tuning plan. Both hold-outs so
   far shared the tuning plan (house 80 the fixed plan of tuning houses 70 and 71 itself; house 81 a rearranged variant
   of it, like every tuning house from 72 up), and their results are labelled so wherever they are shown.
15. [pI7] **A one-night mark does not wall the route off** ([HuM]). A cell of the low "stands up off the floor" layer
   (tall_layer.py's, 4-60 cm) blocks the sweep route only when it is marked on as many of the mapped nights as the
   route's --min-nights asks of the lidar's standing layer, and on at least two when two or more nights are mapped; with a
   one-night map the one night counts. When the route plans nothing it says why on stderr (the floor the dock reaches,
   its best clearance against the clearance asked, how far from the dock the walkable floor lies) and exits non-zero, so
   the night stops on the reason, not on an empty route. House 72's 1280 re-sweep (13 Oct 2026): a thin ridge of returns
   4-6 cm up, on night 2's layer only and on nothing in the house, ran across the dock's bay and left the dock 2.4 m2 of
   floor with 0.65 m clearance, under the route's 0.7 m: no waypoints, exit 0, an empty log.

## Decisions

1. [drP] **Moved furniture, first result (sim).** Real-time house nights, timeline seed 8, held-out nights 5-12, run
    22-23 Sep 2026 with detector v2 frozen: 0 of 175 false alarms were on a chair or on the spot a chair had left.
    Only 2 big chair moves happened in 12 nights (a generator bug re-rolled the move type instead of the position), so
    big moves are barely tested. Details in `docs/research/realtime-run.md`.

2. [QMt] **The house is indoors, lit by day or by night**: windows and lights, not an open top; both drawn at random,
   nights with soft shadows and warm lights, days with sharp shadows and bright lights, though sunlight through a
   window is rarely sharp. With `make_nights.py --indoor` the house
   has a ceiling (a downward-facing plane: drawn from inside, see-through from the viewer above) and ~1 m windows in
   the outside walls, 0.9-2.1 m up (whole wall below, where the lidar looks). Each night draws day or night from its
   own seeded stream (items and every other draw are unchanged): night is a warm ceiling light and sometimes the
   lamp, soft-shadowed; day is soft light from the two windows nearest the swept area, and on about a third of days
   the sun shines in through one of them (sharp) with a dim warm fill rising from its patch. The tidy's nights (`night_tidy.sh`) are
   all lit as night (`--regime night`), calibration nights included, as the robot works: compared with a lamp-lit
   floor, a window-lit one differs everywhere (26 Sep: 35 stops for nothing on an empty night). A house's ceiling light
   hangs in one place every night (it had moved up to 1.2 m between nights). Forced, MuJoCo 3.10's
   classic renderer: it ignores `bulbradius`, draws at most 8 lights and has no bounce light, so each light was drawn
   as 3-4 copies across its size for soft shadows. Since 26 Sep each is one light: the robot's cameras are Filament,
   which draws soft shadows itself ([Hsz]), and the copies filled the viewer's 8 lights so lights dropped in and out.
   The DimOS costmap drops points above 2 m first
   (`HeightCostConfig.ignore_above` in the fork), so the ceiling blocks nothing and walls stay walls. Furniture with legs or
   overhangs is built as such (mostly the pieces with legs or an overhang): coffee and dining tables on legs, a
   sofa on 10 cm legs with back and arms, a console on legs, an island whose worktop overhangs the dining side, a
   counter with a toe-kick (`make_nights.furniture_xml`); the sweep route and item placement still keep off the whole
   footprint, so no items lie under furniture yet. Walls, ceiling, furniture, sofa and worktops carry photographed textures (ambientCG, CC0,
   `tools/fetch_textures.sh`), one set per house from its seed, so the head camera tells floor from wall and furniture as
   in a real house; MuJoCo 3.10 stretches 2D textures on boxes' vertical faces (3.12 adds proper texture coordinates).
   With `--photo-floors` (the default for the Filament nights, `night_tidy.sh`) the floor is photographed too: a
   carpet in the carpeted areas and wood or tile elsewhere, drawn per house from the arm sim's training
   floors (`sim/arm/floor_photos.csv`; the held-out floors are kept for a held-out house), at 3 mm a pixel with its
   normal and roughness layers; the rug is a photographed rug, and the stains and nightly drift are as before. House
   70, the main tuning house, has carpet and a complex, high-contrast tile from 27 Sep (complex high-contrast designs
   are kept, not backed off): large slate-look porcelain tiles
   (~48 cm) laid on the diagonal, each cut from a different CC0 stone photo with its own crop and tone (browns, rust,
   grey-brown), thin dark grout, and a small four-square inset at one crossing in four, the tile corners clipped
   around it (`slate_tile.py`; sizes and palette are the project's chosen values), with a low-pile
   carpet at the lower end of what homes have (Carpet016, its top 8 mm above the
   tile, est.). House 71 has a
   patterned high-contrast tile from the catalogue (`tile_pattern`: marble checkers, painted and mosaic patterns);
   the other houses keep their floors. The rug stays in the sim unless it proves the blocker, then
   it waits for later. The
   proof-of-concept analysis tools that read the floor image at 1 cm a pixel (`floor_truth2.py`,
   `floor_model.py`) don't handle it; the night's own detection never reads the image.
   Closer to a real room, as close as is reasonable in the general things that won't change between cameras: walls are
   painted (the plaster photo in grey for
   its surface, a common interior paint per house for its colour: white to tan; PaintedPlaster017's own blue-grey read
   as a slate room), with white baseboards; windows have half-open slatted blinds and a view out that is bright by day
   and dark by night (Filament drew its flat clear colour there); the sofa is fabric or leather with split seat and
   back cushions and throw pillows, and a TV stands on the console; the wall-to-wall carpet is one of the home kinds
   (cut pile, berber, plush), the woven and commercial photos kept for rugs and the arm sim (a woven grey carpet had
   a third of a cut pile's contrast at 40 cm). Near-carpet contrast in a daytime render ~10 (was 3.9). Left to the
   camera model: saturation and sharpening (the Go2's camera will set them).

3. [I0Q] **The Filament vslice's tuning and held-out nights, and the bar between them**: tuning gets well above target
   first, so there is margin to lose on the hold-out (the first one, on 26 Sep 2026, came too soon). Tuning, looked at
   freely: house 70 toy sets 1-12 and house 71 toy sets 1-2 (every night run on 26 Sep, including the two first scored
   as held out: their misses shaped the fixes that followed; house 70's sets 6-12 added the same day, never held out,
   so the detection bench counts in steps finer than 4 points a toy). Held out, not run or looked at until the pipeline is
   frozen: house 71 toy sets 3 and up, and new houses (seed 72 and up). Held out is run only once the tuning nights
   clear the bar: 3 tuning nights in a row with at least 80% of toys binned and at most 3 stops for nothing a night.

4. [ISF] **Tuning spans more houses** (the first hold-out, 30 Sep 2026, found 75% against 88% on the tuning houses).
   Toys found is the measure to tune, on more tuning houses with new layouts and maps, not one house's layout over
   and over. New houses from seed 72 up join the tuning set,
   each with its own layout ([uCP]), floors drawn from the training photos and toys from the training pool, looked at freely;
   [I0Q]'s reservation of them as held out is released (no night on them had run). The hold-out house of sim-slice
   [bnO] (seed 80) is spent and is never tuned on; the next hold-out is a fresh layout (seed 81) on the same held-out
   floors and toys, which stay unseen. Finding is tuned first, on the detection bench over every tuning house at once
   (`make_detect_bench.sh`, `bench_detect.py`), then checked in full tidy rounds across them. The first four new houses: 72 and
   73 (mirrored) and 76 and 77 (as drawn), so the set has both ways round. A validation house checks that tuning generalises before
   a hold-out is spent: once the new houses are tuned to 90-95% too, another new house must come in at 90-95% as well,
   not the 75% of the first hold-out. It is seed 78 (its own
   layout, as drawn; training floors and toys), not recorded or looked at while tuning, run once finding reaches 90-95%
   of toys over the tuning houses' detection bench; it passes at 90-95% found. Below that, tuning is still fitting the
   houses it sees, and the hold-out waits.
   First measurement on the re-recorded six (1 Oct 2026; cabinets, house 70's beige carpet, highlight-protected exposure,
   every lamp drawn; 12 nights, `data/bench/houses6c`): 69/96 toys detected (72%) at the defaults, 25.9 false candidates a
   night; best settings 76/96 (79%: `--tc 0.035 --ratio 0.5`, 30 false a night); the wrist camera added (`--camera both`)
   gives nothing. Of the misses, 13 of 26 were in view, often for 100+ frames, and never registered as a change (small
   toys near the floor's colour: the Schleich lion, the 1x1 Lego, the Mario figure), 6 changed but formed no candidate,
   5 were never in view, 2 were dismissed. Floor comparison alone, as [2w8] has it, tops out near 80% here: the
   validation house waits.
   With the floor map rebuilt (1 Oct 2026, the same six houses' recordings, `data/bench/houses6s`: DimOS's pose graph
   over both baseline nights' lidar, straightened by every keyframe pair's lidar match, pictures of 15 views a cell,
   tonight's frames placed on the baselines' lidar map; sim-slice [FHA], [nv5]): 82/96 (85%) at the defaults, 85/96
   (89%) at `--tc 0.035 --ratio 0.5`; candidates near no toy 23.1 and 28.9 a night, 11.6 and 18.1 of them off the rug
   (the rug still moved on these recordings). By the grasp tiers, easy 85%, hard 83% at the defaults. Left: the
   Schleich lion and the 1x1 Lego, small and near the floor's colour. The validation house was run at 89%, one point
   under the goal and judged close enough: house 78, recorded once, replayed once
   at the tuned settings above, fixed before it was recorded. Its baseline night 1 had two chairs stacked (a chair moved a
   lot onto another's home, which then stayed home on it; the chair mover's fault, fixed the same day, `movables.place`);
   the run was kept, as unlikely to hurt it much. Result (1 Oct, once, `data/bench/val78`): 10/16
   toys (62%; nights 6/8 and 4/8), candidates near no toy 23.0 a night (0.5 on the rug), against 89% on the tuning
   houses at the same settings: the gap the first hold-out showed (75% against 88%). Its floor map's shape was the best
   of any house (1.6 cm from the true path at the median, 3.7 p95), but its picture among the worst: floor map error
   2.98% (sim-slice [nv5]; tuning mean 1.0%) and its two clean nights disagreeing on 4.4% of the open floor, as on
   tuning houses 70 and 77, where light baked into the picture (sheen at grazing angles, shadow edges) is what is left.
   Toys still heaped by the start ([O7l]: all eight within 2.9 m); two more nights with them spread followed.
   The second hold-out (1 Oct 2026; house 81, held-out floors and toys, toys spread, the new
   floor map, the tuned settings; `data/eval/h3*`): 17/24 found (71%), 7/21 grippable binned (33%), 1/3 too-big binned
   or reported, 48.7 stops for nothing a night; the first hold-out (house 80, old floor map) had 75% and 52%. Its floor
   map's error against the actual floor: 8.54% of the open floor, against 1.0% on the tuning houses and 2.98% on
   house 78: on held-out floors the floor's picture, not its placement, is now the limit. (Night 2's first try ran
   0 minutes when the machine restarted; it was set aside unscored and run again.)
   The six tuning houses re-recorded (2 Oct, `data/bench/houses6r`: toys spread ([O7l]), house lights with their sizes
   and bounce, chairs that never stack, the rug still; the floor map wired in): 79/96 (82%) at the defaults, 86/96 (90%)
   at `--tc 0.035 --ratio 0.5`, 27.2 candidates near no toy a night. A sweep around it found nothing better: tc 0.030
   83%, 0.040 85% (a narrow peak, so fragile), ratio 0.4 85% with fewer candidates (23.7), hits 3 no change; span 0.6
   the same 90% with more confirmed outright (73 against 65, so fewer sidesteps). Left: the 1x1 Lego (4 of 10 misses),
   and on unseen floors the floor's picture.
5. [A7G] **Hold-outs draw plans of their own, from a registry.** `make_house.HOLDOUT_OWN` (seeds 100-119) are the
   hold-outs from [ejh] on: each draws `unique_plan(seed)`, rooms split from the same outer footprint (3-5 rooms, their
   walls, doorways of 1.4-2.1 m or an open end), furniture set by rule (TV, counter and play shelf backed to a wall and
   facing the room, the sofa facing the TV across a coffee table, an island parallel to the counter, the dining table
   free-standing with room for chairs), carpet, rug, lamp, bowls and dock by rule, every open cell reachable and every
   room walkable. No tuning seed calls it. `HOLDOUT_SHARED` (80, 81) keep their shared plans byte for byte; `PREVIEW`
   (900-999) are drawn only for review and are never hold-outs, so the hold-outs' own plans stay unseen. Pieces facing
   another way carry quarter turns (`FACING`), built facing -y and turned as a body (`make_nights.furniture_xml`).
   - The measure: `plan_overlap`, the Jaccard overlap of two plans' structure (inside walls and furniture, kind for
     kind) at their best alignment over shifts of up to 6 cells and mirroring either way. Threshold **0.30**: the 5th
     percentile of the 325 tuning-vs-tuning pairs (plan 70 and the 25 variant seeds 72-99; min 0.248, p5 0.297, median
     0.429; 3 Oct 2026). Measured: the 20 own-plan hold-outs at most 0.291 (median 0.179) against every tuning plan,
     house 95 included; house 80 1.000, house 81 0.724, so both fail it and are labelled shared.
   - The outer footprint stays 48 x 28 cells: a dozen floor tools hard-code its 8.4 x 4.9 m half extents, and a corner
     filled solid would draw as an outside wall (`walls_xml`: windows, and hidden from Filament). For own plans an
     inside wall meeting the outer one stays an inside wall; the older plans keep their drawing (house 95's y = 17 wall
     still draws as an outside one, with windows).
   - Not yet wired: `movables.place` still uses the base plan's chair homes; `CHAIR_HOMES` holds the own plan's.

6. [bpL] **The truth meets the robot's code only from beside it** ([1B2]). Where a result of robot code is scored, or
   run on the sim's true path, a scorer or analysis tool imports the robot's code and hands it the truth through an
   injected option the robot never sets: `pose_hook` (the true path, `true_path.py`, into `floor_mosaic2.main`,
   `calibrate_mount.main`, `confirm_stream.replay`), `scorer` (`score_detect2.py` into `detect_changes2.main`) and
   `scored` (`morning_score.py` into `morning_log.write`). The scoring itself lives in the scorer (`score_confirm.py`,
   `score_detect2.py`, `score_floor_model.py`, `morning_score.py`). Hard-won: a `--truth` flag inside robot code is a
   read of the truth the static guard cannot tell from a live one, and the 12 breaches it found on 3 Oct 2026 were all
   of that kind or an import chain into old scoring code. `tests/test_truth_boundary.py` holds both halves: robot code
   names no truth file, and no robot code sets an injected option.
   - Allowed, with its reason in the guard's ALLOWED: `morning_log.py` names `traj.csv`, the sim's path file, because
     it is where a sim night's odometry is kept; it loads only the odometry columns. A night's own odometry file (or the
     recording's) would remove it.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [yyJ] (seeded varied nights) | `sim/house/make_nights.py`, `tests/test_movable_chairs.py` | Passing: big chair moves at their drawn 15% since 27 Sep (the move was re-drawn on every retry: 3%); chairs on in every night since 27 Sep |
| [O7l] (toys where a household leaves them) | `tests/test_toy_layout.py` | Passing: spread over the walk's floor (spans 4.8-5.9 m, min to median, against 3.6-4.5 before), a pair or a trio at most within 1 m; house 78 with them spread: 13/16 against 10/16 heaped (1 Oct) |
| [aj8] (tuning spans the floors a hold-out can draw) | `tests/test_floor_variety.py` (written after the table: a characterisation test) | Written: the twelve floor-variety houses recording from 2 Oct |
| [bP1] (carpet worn along walked paths) | `tests/test_carpet_wear.py` | Passing (watched failing first): lanes on the carpet only, smooth inside it, the house's own; the tuning houses take them at their next re-record |
| [D4v] (found/missed/unseen by tier) | `sim/house/metrics.py` | Passing |
| [EL6] (tuning vs held-out, scored once) | run notes and frozen detector copy | Passing |
| [5se] (false alarms, furniture separately) | `sim/house/score_movables.py` | Passing |
| [h3t] (on DimOS, stated sim speed) | `sim/house/sweep_night.sh` | Passing |
| [WTj] (cabinets read as cabinets) | `tests/test_cabinets.py` (fronts, gaps, handles, texture coordinates at scale, finish per seed) | Written: not yet wired into the house (after the six-house bench, so its houses match) |
| [jmf] (furniture textured on every face; house 70's carpet) | `tests/test_cabinets.py` (furniture and chairs as textured meshes), `tests/test_house70_carpet.py` (house 70 lays Carpet016 in its hue) | Written |
| [uCP] (a layout per house) | `tests/test_house_layout.py` (70, 71, 80 unchanged; 72 and up differ, every open cell reachable, the same seed the same plan); house 70 regenerated byte-identical, 30 Sep | Passing: 4 tests |
| [HuM] (route from the robot's map) | `tests/test_sweep_route.py` (a mapped room: reachable floor only, clear of what the map holds, rows from the dock's end to the far wall, a first night without a map plans nothing; house 72's recorded baselines scored on the plan), `sim/house/sweep_compare.py` | Written (3 Oct 2026): passing; the route plans on the lidar scans in the house's frame, never the plan. The callers plan on the robot's own dock and nights (`--near dock`, `--night N`), and a first night explores, then plans on its own recording (sweep_night.sh); both run statically on house 72's recordings (3 Oct: night 3, 12 waypoints, 18.7 m; night 1 from its own recording, 15, 13.9 m). No sim night has run on it yet |
| [pI7] (a one-night mark does not wall the route off) | `tests/test_sweep_route.py` (a one-night thin line across the dock's bay no longer cuts it off, a mark on both nights still blocks, a one-night map keeps its marks; nothing planned: the reason on stderr and a non-zero exit) | Written (14 Oct 2026) |
| [1B2] (no truth on the robot's side) | `tests/test_truth_boundary.py` (imports and truth-file names of everything robot code reaches; every module placed on a side; no robot code sets an injected truth option; each allowed name still found); `tests/test_scored_beside.py` (the pose ablation through pose_hook, the replay's scorer, no --truth on the robot's tools); `tests/test_detect_changes2_blobs.py` (the detector alone reads no truth file, its scorer matches its blobs) | Passing (3 Oct 2026): the 12 breaches the guard first found are fixed ([bpL]): scoring moved to score_confirm, score_detect2, score_floor_model and morning_score; the true-path ablation to true_path; wrist_refine's unused import of the old detector dropped and overlay on the locked detector's identical test; the HUD file named in hud.py; morning_log's traj.csv odometry allowed with its reason |
| [ejh] (hold-outs wholly unlike tuning) | `tests/test_holdout_plans.py` (registry; existing plans unchanged; own plans deterministic, valid, reachable; every own plan below 0.30 against every tuning plan, 80 and 81 above it; [A7G]) | Passing (3 Oct 2026): own-plan hold-outs at most 0.291; the movable chairs follow an own plan's table, stool and play shelf, and stay as they were in every other plan (`test_chairs_follow_a_holdout_plans_own_table_and_rooms`, on preview seeds) |
