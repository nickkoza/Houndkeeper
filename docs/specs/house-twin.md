# House twin

**Type:** Capability spec · **Status:** Draft · **Source:** a twin of the real house helps test the sim; real to sim (the house generator) could be offered upstream in itself; the floor ground-truthing tools can make realistic floor textures for it · **ID:** [lQ1]

## Purpose

Barrymore builds a simulated copy of the house he lives in from what his own sensors recorded: the walls and
furniture from his lidar map, the floor's look from his floor picture, the lighting from what his camera saw 
the robot knows only its sensors). The copy is a MuJoCo house the night pipeline runs in like any procedural one, so
any change can be tried on "his" house overnight in the sim before it runs on the real floor.

It also tests the sim, both ways round. A procedural house has its truth: build its twin from the sim robot's own
recordings, run the same night in both, and the difference is what the twin-building loses. The same build on a real
house gives a twin whose nights can be set beside the real nights: the gap is what the sim gets wrong about the world.

## Acceptance criteria

1. [UdA] **A twin from a recording alone.** From one or more nights' recordings (lidar map, floor picture, the
   house's frame and shape; the replay path's outputs, sim-to-real-calibration [V35]), a command writes a MuJoCo house
   the night pipeline loads: walls and fixed furniture as boxes from the map's occupied cells and the tall layer, the
   floor textured with the floor picture, unmapped floor marked as such (not invented). Nothing of a sim house's truth
   is read (a recording of a sim night is treated as a real one).
2. [JLx] **The twin matches its source where it was seen.** Built from a procedural house's own nights: walls and
   furniture footprints within a stated distance of the truth (a map-IoU and a boundary distance), the floor texture's
   difference from the truth floor no worse than the floor picture's own error ([evl] [FHA] figures), each figure
   reported per house over the tuning houses.
3. [HYk] **The same night in both.** The night pipeline run in a twin and in its source house with the same seeded
   items gives found / missed / false-alarm figures within a stated margin; a larger gap is reported per cause (the
   map, the floor, the lighting).
4. [iuP] **Real nights beside twin nights.** For a real house, a night's real numbers (the gap report's, [KbT]: camera
   drops, odometry drift, floor-picture disagreement, finds) are set beside the same night in the twin, and the
   difference names the sim parameter it points at. This is the sim-to-real check run in reverse ([V3o]).
5. [TgM] **Never kinder than the source.** The twin keeps the sim's flaws ([x3f]): noise, flex, lag and lighting
   variety are the sim's, not switched off for the twin; furniture the map could not see is absent, not guessed.
6. [au3] **Private.** A real house's twin (its floor texture is a picture of the floor) lives under `data/` beside the
   recording it came from, is cataloged with it ([HqQ]), and never leaves the house ([fya]).
7. [POG] **Floor textures made with the ground-truthing tools**. The tools that score a floor
   picture against the sim's true floor (`floor_truth2.py`: the texture on the picture's 1 cm grid,
   each 60 cm patch registered by masked edge correlation, the warp left after one rigid offset) also make the twin's
   floor: the floor picture's lighting taken out (each spot's trimmed mean of views, [evl]), its patches
   registered to each other so seams line up, the result one texture at the sim's texture scale. On a
   procedural house the same tools measure what this loses against the true texture (the [JLx] floor figure). A real
   house's textures can also join the procedural generator's floor library for the house's own sims (realistic floors,
   [CaM]); being pictures of the house, they never go into a released benchmark ([zGo]).
8. [4TQ] **The builder is upstream-shaped**. The general part (a DimOS recording's map, tall layer
   and floor picture in, a MuJoCo scene DimOS's sim loads out, for any robot DimOS drives) lives in our DimOS fork as
   its own module and commits, in DimOS's style, with its own tests on a recording that isn't ours, and nothing of
   houndkeeper in it ([CSu]'s rule). The house-specific part (our floor picture pipeline, the night pipeline's
   loading, the validation against our procedural houses) stays in houndkeeper.

## Decisions

1. [1Gs] **Boxes and a texture, not a mesh** (proposed, 3 Oct 2026). The night pipeline needs what the robot walks
   round and what the floor looks like; the lidar's 2-D map and tall layer give footprints and heights, the floor
   picture gives the texture. A photogrammetry mesh would carry more but needs views the head camera doesn't take and
   is slower to check. Revisit if furniture undersides (where toys hide) matter.
2. [lVm] **Validated on procedural houses first** (proposed, 3 Oct 2026). Their truth is known, so every figure in
   [JLx] and [HYk] can be measured before a real house is twinned. The real twin's first use is [iuP].
3. [e1y] **A column is a slab only where the floor under it was seen** (3 Oct 2026). The lidar's lowest return over a
   sofa is its seat, and over a kitchen island its top's edge, so a high lowest return alone would hollow out
   everything solid. Where the floor picture saw the floor under a column (half a 5 cm cell or more), the column is a
   slab from its lowest return up (a table top the robot sees and reaches under); elsewhere it stands on the floor.
   Hard-won: without the floor's evidence, house 70's twin had 2,333 slabs of 4,328 boxes, the sofa and island among
   them; with it, 1,020, the dining table's top the largest.
4. [0Dt] **An enclosed unmapped pocket is the inside of something solid** (3 Oct 2026). The lidar returns from an
   object's faces only, so its inside is never mapped. A pocket wholly enclosed by mapped columns (gaps of 10 cm or
   less in the faces), with no seen floor in it and no larger than 6 m2, is filled to the nearest column's height. A
   pocket open on a side the robot never scanned stays open: the twin keeps what the map shows ([TgM]).
5. [P2B] **Walls end at a stated ceiling** (3 Oct 2026). The robot's lidar map stops at its reach (1.5 m in the sim),
   lower on a far wall (house 70's walls ended between 1.30 and 1.50 m, 3 Oct). A column within 25 cm of the map's own
   top (its returns' 99.9th percentile) is taken as cut and goes up to the ceiling, a stated height (2.4 m, the
   procedural houses'; for a real house, measured). Tall furniture that reaches the band goes up with it.
6. [9Sa] **Lit as the procedural houses are** (3 Oct 2026). The twin takes the generator's lit regime (make_nights:
   fixture strength, tint, bounce and headlight), one fixture over each 5.6 m square of mapped floor holding 4 m2 or
   more, about the 33 m2 a fixture the lit regime gives the open plan. The lamp is left out: where it stood is not
   in the map. Lighting from what the camera saw is the next step ([lQ1]'s purpose).
7. [03u] **The floor texture is the pictures' 1 cm grid** (3 Oct 2026). It covers the house's metres as a procedural
   floor does (one texture over the floor plane), at the pictures' own resolution: drawn at the procedural floors' 3 mm
   (`--px 0.003`), the extra pixels would be interpolated, adding GPU memory and no detail.
8. [j7J] **The general part is DimOS's `simulation/mujoco/scene_from_map.py`** (3 Oct 2026; branch
   `nick/feat/real-to-sim-scene` of the fork). It goes one step past DimOS's own `mapping/occupancy/extrude_occupancy.py`
   (an occupancy grid to fixed-height boxes, `--mujoco-room-from-occupancy`). In: points in a world frame
   (`HeightGrid.from_points`, the floor's height found or given) and optionally a `FloorPicture` (rgb, seen mask,
   resolution, origin, in `OccupancyGrid`'s grid convention). Out: `boxes_from_heights` ([e1y], [0Dt], [P2B] as
   options) and `build_scene` (a `MapScene`: MJCF text, its texture as assets, `write()`), any robot (the scene holds
   none; DimOS's sim adds it). Not in it: the pose correction, the floor picture's making and registration, the
   lighting regime, the scoring against procedural truth, the night pipeline's loading (all houndkeeper's). Upstream
   next: a `dimos map to-mujoco` command and a `mujoco_room_from_map` setting that loads a written scene's directory
   with its textures (DimOS's loader takes textures from one fixed folder today).

## Tests

| Criterion | Test | Status |
|---|---|---|
| [UdA] (from a recording alone) | builder on a procedural house's recordings with its truth files absent; the house loads in MuJoCo: `tests/test_house_twin.py` (house 95's run outputs copied without truth, the twin parsed alone and with DimOS's robot; truth files refused); `sim/house/house_twin.py` | Passing (3 Oct 2026; written after the builder, not watched failing). Also built from [V35]'s truth-hidden replay of house 95 |
| [JLx] (matches where seen) | footprint IoU, boundary distance, floor-texture error against the procedural truth, per tuning house: `sim/house/twin_score.py`, `sim/house/twin_bench.sh`; the scorer's measures in `tests/test_house_twin.py` | Partial: measured 3 Oct 2026 on the eighteen tuning houses: 99-100% of the plan's seen faces have the twin within 5 cm, the twin's own faces 60-81% within 5 cm (p90 7-15 cm), footprint IoU 0.28-0.38; floor map error no worse than night 1's trimmed picture on 14 of 18 (median 0.43% against 0.45%), worse on 73, 88, 92, 93 |
| [HYk] (same night in both) | night pipeline in the twin and the source, same seed (a sim run: later, not before an explicit go) | Planned |
| [iuP] (real beside twin) | the first real night against its twin | Planned |
| [TgM] (never kinder) | the twin's scene uses the same noise and lighting settings as procedural houses (a static check of the generated scene): `tests/test_house_twin.py` (the lit regime's fixtures, bounce and headlight; no scene options); unseen structure absent and only enclosed insides filled: the fork's `test_scene_from_map.py` | Passing (3 Oct 2026) |
| [POG] (textures by the ground-truthing tools) | a twin's floor texture from a procedural house's floor pictures, scored by `floor_truth2.py` against the true texture; a real texture refused by the benchmark export: `tests/test_house_twin.py` (night 2 laid on night 1 by `floor_register.snap`, blended without a seam; watched failing with the registration taken out) | Partial: the texture and its score (3 Oct 2026, eighteen tuning houses: patches 20 mm off the true floor at the median, p90 50, the trimmed pictures 20 and 44); the benchmark export's refusal not yet written (no export exists) |
| [4TQ] (upstream-shaped) | the fork's module builds and passes DimOS's tests alone; `git grep -i houndkeeper` in it finds nothing: `dimos/simulation/mujoco/test_scene_from_map.py` on branch `nick/feat/real-to-sim-scene` (DimOS's go2_short recording among them); `tests/test_house_twin.py` | Partial: its 9 tests and DimOS's extrusion test pass in the branch's worktree alone, ruff and mypy clean, no houndkeeper in it (3 Oct 2026); not yet committed, nor in the fork's working branch (house_twin.py loads it from the worktree until then) |
| [au3] (private) | a real twin is written under `data/` and cataloged; nothing under `sim/` or the repo: `tests/test_house_twin.py` | Partial: the builder refuses the repo outside `data/` (3 Oct 2026); cataloging with the recording not yet written |
