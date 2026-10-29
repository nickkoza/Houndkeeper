# The operating area: where he may go

**Type:** Capability spec · **Status:** Draft · **Source:** first runs in only part of the house · **ID:** [JVr]

## Purpose

At first the robot runs in only part of the house, so there needs to be some way of putting up a soft barrier in
the lidar data to have him stay in that area. A virtual boundary, drawn on the house's
map, that Barrymore treats as a wall: he plans, explores, sweeps and tidies only inside it. Standard terms: a keep-out
zone or virtual wall, applied as a costmap layer (Nav2's keepout filter is the well-known form).

## Acceptance criteria

1. [rLQ] **He stays inside.** An allowed area (one or more polygons in the map frame), and optional virtual walls
   (line segments, e.g. across a doorway), are marked lethal in the navigation costmap with a soft margin of rising cost
   inside the edge, so no plan crosses them and he does not hug them. Measured in the sim: over a night with an area
   smaller than the house, no foot ever outside it.
2. [mvn] **The boundary is a map layer, not sensor data.** Nothing is injected into the lidar scans or the point cloud:
   mapping, localization and the pose graph see only what the sensors saw, so the boundary can never be matched as a
   wall or bend the map. The lidar still maps what it sees beyond the boundary.
3. [OrO] **Every task respects it.** Exploration ignores frontiers outside the area; the sweep route is planned only
   inside it; finds outside it are not pursued (the morning log may note them as seen, not visited); the dock, if
   outside, is refused when the area is set.
4. [daF] **Drawn from the phone.** The area is drawn and edited on the house map in the phone cockpit ([Yj2]), shown on
   the live view, and stored with the map in the map frame; each change is confirmed and recorded with its time, like
   a calibration write.
5. [Qd5] **Safe when unsure.** If he finds himself outside the area (pushed, or after a relocalization), he plans the
   shortest way back in and does nothing else until inside; if localization is lost, he stops and reports. On the real
   robot a run does not start without an area set.
6. [HZO] **Not a safety barrier.** The boundary depends on localization; stairs, drops and anything dangerous are kept
   behind a physical barrier (a gate), and the docs and the cockpit say so where the area is drawn.

## Decisions

1. [u4W] **A general DimOS costmap feature, extending the fork's keep-out discs** (5 Oct 2026; build-or-depend [eM8]).
   The fork's `CostMapper` already marks floor discs lethal from a `keepout` input (the tidy's spots too low for the
   lidar). Polygons and line segments in the map frame are the same idea, made general: any DimOS robot could use them
   (fork rule 6). The houndkeeper side only supplies the area. Nav2's keepout filter is the model, not a dependency.

2. [8vf] **The area is a small file kept with the map, in the house's frame.** `<map dir>/operating_area.json`: one or
   more allowed polygons and the virtual walls, with when and by what it was last changed; every save is appended whole
   to `operating_area.log.jsonl` beside it. The map directory is the sim house's run directory, or on the real robot
   `real/local/house/` (gitignored; `HK_MAP_DIR` overrides). Conventional: one place every reader looks.
   (`src/houndkeeper/operating_area.py`)
3. [GBS] **The tidy publishes the area, placed in the odometry's frame by its own localization.** DimOS's costmap is
   built in the odometry's frame ("world"), which drifts from the house's; the tidy already holds the correction between
   them (the floor tracking, or the live lidar match), so it moves the area by it and republishes when it moves the area
   by 2 cm or 0.5 deg, and every 5 s for a late subscriber. A run without the tidy (an exploration night) publishes it
   through `houndkeeper.operating-area` with the frames taken to agree; never both in one run. DimOS's layer also takes
   zones in another frame through its transform tree, and forbids the whole costmap if it cannot place them: a
   boundary that cannot be located is not driven past.
4. [Kr7] **The soft margin stays below the Go2 planner's wall threshold.** Forced: DimOS's Go2 planner
   (`replanning_a_star`) rebuilds its own gradient from the costmap, treating every cell of cost 50 or more as a wall and
   overwriting lower costs (observed 5 Oct 2026, the fork's `dimos/mapping/occupancy/gradient.py`, branch maker-arm). So
   the margin rises to 40 of 100 over 0.3 m inside the edge: a margin of 50 or more would move the wall inward. On that
   planner the keeping off comes from its own 1.5 m gradient off the lethal edge and its half-width inflation; the
   margin is what any other cost-aware consumer reads.
5. [ma7] **Outside, a passable band back in, and nothing else.** As Nav2's keepout filter lifts a zone's cost for a
   robot already inside it: while the robot's own cell is forbidden, the layer makes passable (cost 49) only the
   forbidden cells no further from the area, nor from the robot, than the robot is from the area plus 0.5 m; mapped
   obstacles stay lethal. The tidy, before any goal, sends him to the nearest point 0.35 m inside the edge, and stops
   and reports after three tries.
6. [XSa] **Localization lost is no pose for 12 s; a gap of 2 s stops him.** The pose the navigator plans with is
   odometry's. Silent for 2 s of robot time, he cancels his goal, stands still and says so; silent 10 s more, the run
   ends as "stopped" with the reason (the status, the morning log); if it returns, he goes on and the gap is kept in
   the status. Hard-won: a sim night on 5 Oct 2026 ended at 2 s, on a 2.6 s gap the tidy saw in its own odometry while
   the night's recording of the same stream had none longer than 0.1 s (house 72, after a stand-up). The live match to
   the house's lidar map is off by default (`HK_LIVE_LOC`), so it adds no lost signal yet.
7. [nkV] **Only the real robot refuses to start without an area.** The tidy refuses on the robot (DimOS's global config
   is neither a simulation nor a replay); the sim runs without one, so every existing bench and night runs as before.

## Open questions

1. Whether the area also limits what the floor picture and change detection spend time on outside it.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [rLQ] (stays inside) | unit: allowed areas, keep-out polygons and virtual walls rasterized lethal, the margin, no plan across the edge, a diagonal wall without a gap, the frame (`dimos/mapping/test_keepout_zones.py` in the fork); published in the odometry's frame by the correction (`tests/test_operating_area.py`) | Passing |
| [rLQ] (stays inside) | a sim night with an area smaller than the house: no foot outside, no goal outside (`sim/house/area_excursion.py` on a tidy night) | Measured: no foot outside in four runs, 0 of 35,180 foot positions; no goal outside; the control night without an area went 1.12 m out (5 Oct 2026) |
| [mvn] (map layer only) | the point cloud the costmap is built from unchanged with zones set, and only the costmap changed (`test_the_point_cloud_is_untouched_and_only_the_costmap_changes`, fork); the message carries the area alone (`tests/test_operating_area.py`) | Passing |
| [OrO] (every task) | exploration finds no frontier outside (fork, `test_frontier_exploration_finds_no_frontier_outside_the_area`); the sweep route planned inside only, finds and blockers outside noted not pursued, a dock outside refused, the morning log notes them (`tests/test_operating_area.py`) | Passing |
| [daF] (from the phone) | the page's drawing, save command and confirm (`tests/js/hk_area.test.mjs`); the robot side stores, logs, refuses an unconfirmed or stale save (`tests/test_operating_area.py`); drawn and saved at 390 px (`tests/browser/area_390.mjs`) | Passing |
| [Qd5] (safe when unsure) | outside: the shortest way back in first, a stop after three tries; no pose for 2 s: stops and reports; the robot refuses to start without an area, the sim runs without (`tests/test_operating_area.py`); the passable band back in (fork) | Passing |
| [HZO] (not a safety barrier) | the warning on the page in every state and in the save's confirm, and in the docs (`tests/js/hk_area.test.mjs`, `tests/test_operating_area.py`) | Passing |
