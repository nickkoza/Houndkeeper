# Item library

**Type:** Capability spec · **Status:** Draft · **Source:** D0 [jX2]: soft and rigid items, and some not so easy to pick up, since toys come in all shapes and sizes · **ID:** [QxY]

## Purpose

One shared set of simulated things left on the floor, used by the grasp scenario, the full slice and demonstration
recording, so every sim is scored against the same variety of shapes, sizes, masses and softness that a real house
has, including things the robot should find hard.

## Acceptance criteria

1. [X94] Every item is defined once and used by every scenario that places items; no scenario keeps its own list.
2. [867] It includes rigid items of several shapes (box, brick, cylinder, sphere, and at least one irregular toy shape
   built from several parts), sized from about 3 cm to about 25 cm and weighing from about 10 g to 500 g.
3. [qdj] It includes soft items that deform when gripped: at least a cloth (sock-sized) and a rope or cable.
4. [996] It includes items that are hard to pick for known reasons, and each says why: one that rolls away, one too
   flat to pinch from the floor, one wider than the open gripper, one too heavy for the arm's rated load at reach, and
   one thin enough to be hard to see.
5. [EWu] Each placed item has a stated correct outcome (picked into the bin, or left for review with a reason), so a
   scenario can score "left for review because it is too wide" as right rather than as a failed pick. For a scanned
   item it is judged from how that item landed: the same toy can be grippable on its back and too wide on its side.
6. [JwL] Items are placed from a seed, with position, rotation and mass drawn within each item's stated ranges.
7. [nuQ] **Some items are only for judging,** never for training. A fixed, named set of held-out items is never used to train a model, record demonstrations,
   or tune the scripted grasp or the detector; it is used only to score them. Every scenario draws from the training
   items unless told to judge, and every reported result says which set it was scored on.

## Decisions

1. [09E] **Scanned items assume solid density.** The Google Scanned Objects models (mujoco_scanned_objects at the commit in
   `models/items/gso/SOURCE_COMMIT`, fetched 2026-09-24) give a teddy bear 11 kg; masses here are per-item estimates of
   the real object, drawn per trial.
2. [Kl2] **Attaching items leaves their joints zeroed in the robot's keyframes** (MuJoCo 3.10.0 MjSpec attach); a zero
   quaternion is invalid and the model goes unstable at once. Every joint not on the robot starts at the model default.

3. [Bnq] **Scanned items are judged as they land** (2026-09-25). Fixed per-item outcomes scored a fire truck lying 11 cm
   across as a failed pick. `items.judge` uses the item's true width where the jaws would close (the narrowest extent
   of its lowest 4 cm within +-2 cm of its middle), its height and its mass against the jaws' 9.5 cm and the arm's
   rated 1.5 kg. Primitives, cloth and rope keep their fixed outcome.
4. [WKu] **The library grew to 72 scanned objects** (2026-09-25), chosen from the 1,030
   in the source for things left on floors. Masses without a stated range come from size (bounding-box volume x 60-200
   kg/m3, an estimate for hollow plastic toys). Scans that fuse several separate pieces into one rigid body (animal
   sets, puzzle boards with loose pieces) are left out after a check of the contact sheet (`sim/arm/item_sheet.py`);
   `toy_taxi` is two cars and stays only so the 2026-09-25 baseline remains comparable.
5. [Vpm] **The held-out items are new scans, not a slice of the 72** (2026-09-25). Every one of the 72 has already been
   seen while tuning the scripted grasp (fix1-fix8 on the front benchmark), so none of them can judge that tuning
   fairly. The held-out set is drawn fresh from the source's unused floor-clutter scans, across the same kinds (figures,
   vehicles, soft toys, shoes, blocks), and the 72 become the training set. Results so far (the 66% front benchmark) are
   on items the tuning saw.
6. [4Oe] **The models are not in git.** `tools/fetch_items.sh` fetches them at the pinned source commit from the list in
   `sim/arm/items.py` (instructions in `models/README.md`).

7. [DGv] **The held-out items, drawn 26 Sep 2026**, with gripper training and household training
   expanded alongside. The library now has 47 small household things from Google Scanned
   Objects besides the toys (shoes, mugs and bowls, towels, stationery, games and discs, computer mice, lunch boxes, a
   toothbrush holder, a spatula; four scans left out on the contact sheet as fused pieces). About 20% of each kind of 3
   or more, drawn with `random.Random(20260926)` over the whole library and leaving out the benches' fixtures, is held
   out: 24 of 126 (`items.HELD_OUT`), among them three shoes, two mugs, a dish towel and scissors. Everything before
   26 Sep used every item (the v6 policy's demonstrations and the 26 Sep tuning nights included), so these count as
   unseen from then on only. Demonstrations and grasp benches draw from `items.grippable`: every training item that is
   one rigid body and fits the jaws (36, up from 15 fixed-weight items), each run loading a seeded share of them.
8. [w8q] **One object per scan, checked by geometry**: every toy is checked for physically separate
   pieces, which are unfair to the binning algorithm and must not skew its stats. A scan whose surface falls into more than one group of pieces more than 1 cm apart (each group at
   least 2% of the surface) is several objects fused into one rigid body, and no night or score uses it
   (`items.separate_objects`, `items.pool`). Checked on 29 Sep over the whole pool: 2 of 84 training scans (the toy
   taxi, a taxi and a police car 1.6 cm apart; the car-carrier train), none of the 21 held-out. The contact-sheet
   list (`items.FUSED`) had caught the rest by eye; the taxi had been kept for the 25 Sep baseline. Toys whose pieces
   touch (a pony's mane, a turtle's wheels) are one object. Split into separate toys later if wanted.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [X94] (one definition, used everywhere) | grasp scenario, demo recorder, policy evaluation and the DimOS sim hook import the library | Partial: the standalone slice still has its own items |
| [867] (rigid shapes, sizes, masses) | library listing check | Partial: primitives plus 72 scanned objects; `sim/arm/item_survey.py` drops each 6 times and lists size, mass and how often it lands grippable; not yet a pass/fail check |
| [qdj] (soft items) | cloth and rope load and deform in the grasp scenario | Partial: both load and stay stable in the robot model; rope placed in DimOS runs; cloth not yet placed in trials |
| [996] (hard items, with reasons) | library listing check; grasp scenario outcomes | Partial: all five in the library with reasons; outcomes scored on a few trials only |
| [EWu] (expected outcome per item) | grasp scenario scores review-with-reason as correct | Written |
| [nuQ] (held-out items only judge) | `tests/test_held_out_items.py` (the pools are disjoint; what trainers draw from has none; house nights place them only when judging) | Passing: `items.HELD_OUT`, `items.pool`; `PICKABLE` is training items only; `eval_policy.py --items heldout`, `HK_ITEMS=heldout` for house nights |
| [JwL] (seeded placement) | two runs with the same seed place the same items | Partial: placement is seeded; repeatability not checked |
