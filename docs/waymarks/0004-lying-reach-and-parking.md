# [SLn] — Lying reach is a thin band beside the body, so parking decides the pick

**Status:** Proposed · **Date:** 2026-09-25 · **Source:** sim overnight 24–25 Sep 2026, Go1 stand-in ([WD6]), mount +10 cm · **ID:** [SLn]

## Context

The lying reach map at the +10 cm mount is two side bands 20–35 cm from the body's centre line and a 5 cm deep strip
in front. An item must end up inside that band, so the robot has to park within a few centimetres of a spot right
beside the item. In DimOS's sim, walking up to park beside items kicked them: a block moved 43 cm and a toy 30 cm in
one run. The Go1 walking policy barely responds to slow velocity commands (a proportional side-step stalled 12–28 cm
short).

## Decisions

1. [bed] **Approach head-on, then shuffle in** (replaces a side approach). Stop 0.9 m short of the
   item facing it, re-check it with the head camera, then move at a fixed speed the walking policy tracks (0.25 m/s),
   forward then sideways, stopping on odometry within 3 cm, so the item ends at the front target of the reach map for
   the grasp's allowed lean. No foot passes near the item. The side approach measured 1.7–7.8 cm parking error in four
   of five stops; head-on is not yet measured.
2. [A2c] **The parking spot comes from the reach map, never a constant** (`cluster_stops.front_offset`). A hard-coded
   offset from an older map put items outside reach. It aims for the middle of the region where every jaw direction
   is reachable (`sim/arm/reach_dirs.py`), not merely some direction: an item's narrow side is wherever it landed,
   and beyond ~50 cm about 30% of directions were out of reach at the old spot (25 Sep 2026).
3. [Ftr] **Re-check both on the real Go2**: its reach band depends on the real lying height, and its walking controller
   responds to small commands differently from the sim's Go1 policy.
