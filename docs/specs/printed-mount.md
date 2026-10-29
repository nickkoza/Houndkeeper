# The printed mount, trays and bin

**Type:** Capability spec · **Status:** Draft · **Source:** placement [Gy7] ([Pvh]); arm on board [Bhf] ([Rrm], [pkg]); principles [5oU], [x3f], [sJQ] · **ID:** [3JA]

## Purpose

The parts that put the arm, its electronics and the bin on Barrymore's back, designed against real models of the
robot and the arm, positioned accurately, and printed on an X1, in polycarbonate where needed: the Go2's mounting
bolts sit further back, so an adapter brings them forward; the Raspberry Pi and other electronics need trays; the toys
need a bin; and every part is both weight- and flex-conscious. [Gy7] decided where the arm goes; this spec makes the
parts that hold it there.

## Acceptance criteria

1. [I9y] **Real models, accurately placed.** The Go2's body with its actual mounting points (the bolt pattern on its
   back: positions, thread, depth, measured on the robot with calipers or taken from Unitree's published drawings, each
   number marked measured or vendor-published, [sJQ]) and the Maker Arm's base with its bolt pattern (from the
   maker-arm SDK's model, checked against the real base), assembled with the arm at [Pvh]'s position (10 cm forward of
   body centre) in one reference model, in millimetres. [Izu]'s re-run of the placement on the Go2 body comes first; if
   it moves the position, the parts follow it.
2. [vqz] **An adapter from the rear bolts forward to the arm.** One printed part (or a few, bolted) that takes the Go2's
   rear mounting bolts and carries the arm's base at the decided position, under the arm's weight and its rated 1.5 kg
   load at full reach, with stated safety factors for walking, lying down and a fall onto the back.
3. [vQO] **Trays for the electronics.** The arm host (the Raspberry Pi), its buck converter from the Air's pack ([EkS]),
   the speaker and microphone, the eye LEDs' wiring, and anything else the build adds, each held, ventilated, reachable
   for its connectors and cards, with cables routed and strain-relieved so nothing trails ([pkg]).
4. [Frm] **A bin, magnetically attached**, since it will be crashed into often and swapped now and then. Behind the
   folded arm, in the length [Pvh] leaves (about 18 cm), sized for the toys the arm picks, with an opening the arm can
   drop into from its put-away pose, and not catching the legs, the lidar or the head camera's view. It is held by
   magnets with locating features, so it breaks away in a crash (the bin comes off before the mount or the robot takes
   the blow, at a stated release force) and snaps back to the same position (repeatable within a stated tolerance, so
   the arm's drop still lands in it); it is swapped or emptied by pulling it off. The magnets keep a stated distance
   from the arm's motor encoders and the robot's IMU, checked in the design.
5. [mgb] **Weight-conscious.** Each part's mass predicted before printing and weighed after; mount, trays, bin and arm
   power together within [Rrm]'s budget (1.8 kg beyond the arm), and the robot's centre of mass with everything on
   recomputed for the tip-safety envelope ([qpO]).
6. [lUO] **Flex-conscious.** The mount's stiffness at the arm's base (how far it bends and twists under the arm's
   load) predicted before printing and measured after, against a target set from what picking can tolerate (the jaws'
   error at reach). The measured value replaces the sim's estimate (`maker_arm.frame_flex_Nm_per_rad`, today 1500 N·m/rad
   at the base, an estimate) so the sim flexes as the real mount does ([x3f]).
7. [ngX] **Printable on the X1.** Every part fits the X1's build volume, is oriented so its load runs along the layers
   rather than across them, needs little or no support, and has a stated material: polycarbonate where strength or heat
   needs it (the X1 prints PC), or a stated alternative where PC is not needed. Holes for bolts and heat-set inserts
   sized with tested clearances; each part exported as a print-ready file with its settings noted. The X1's build volume
   (256 mm on each side; the mount may not print as a single piece) sets each piece's size: what does not fit is split
   into pieces joined by locating features (pins or dovetails) and bolts or heat-set inserts, each joint where the
   stress is low, and the joints are in the analysis ([tY3]): a joint is where stiffness leaks.
8. [Uaq] **Nothing collides, nothing is blocked.** With the real models, in every posture (standing, walking, lying,
   standing up, the arm stowed and moving): no part hits a leg, the body or the folded arm; the head camera's and the
   lidar's views stay as clear as [Pvh] assumed; the battery and the charging port stay reachable.
9. [uip] **Every build logged** ([E2T]). Each printed version is recorded: what changed, why, its mass and measured
   stiffness, and what failed.

10. [JW7] **The arm and its host are one removable module**. The arm, the arm host and its electronics sit on one module
   that comes off the robot's adapter as a unit (a few fasteners, one connection for power and one for data, no
   rewiring) and fixes the same way to a bench plate for testing off the robot; the robot side (the adapter on the rear
   bolts, the bin) stays. The module's interface to the adapter and to the bench plate is the same, so the arm is
   calibrated once and keeps its mount geometry (its stiffness and position, [lUO], [I9y]) on either.
11. [tY3] **Finite-element analysis of the mount, and a film of it** (the structural analysis is FEA, CFD being for air:
   the electronics tray's cooling, later). Each printed part carrying load (the adapter, the module's plate, the bench
   plate) is analysed under its load cases ([vqz]: the arm and 1.5 kg at full reach, walking, lying down, a fall onto
   the back) with the chosen filament's printed-specimen properties from its maker's datasheet, along and across the
   layers (printed-plastic data, not generic PC, with PC from several makers as options;
   `cad/materials/materials.json`), unpublished figures assumed conservatively and said so; the candidates compared on
   stiffness, mass and margin before one is chosen, giving deflection and twist at the arm's base, peak stress against
   strength with its factor, and where it would fail first. Checked against the printed part's measured stiffness
   ([lUO]). A film of each analysis is rendered: the load applied slowly, the deformation exaggerated by a stated
   factor, stress in colour, smooth and steady (nothing flashes, [0Py]), one clip per load case in the demos' style
   ([HQ6]), for the story and for review on a phone.
12. [Xtj] **Design values from independent tests, with a safety margin**, since makers' figures are not trusted alone.
   Each candidate filament's design values come from the makers' datasheets set beside independent tests (published
   papers, and testers' reported results, each source named with its method): the lower of the two is the starting
   point, then knocked down by stated factors for print quality (not annealed, not perfectly dry, the chosen infill and
   walls), temperature in use, creep (the arm's weight sits on the mount all the time) and fatigue (every step flexes
   it). The design then holds a stated safety factor on stress, and meets the stiffness target at the low end of the
   modulus. The factors, their sources and the resulting allowables are recorded in `cad/materials/` beside the raw
   data.
13. [4JP] **A seat for his magnetic hat**. The little bowler (the logo's, and the tip-of-the-hat introduction: the arm
    lifts it, he bows, the arm sets it back) sits on a magnetic seat on the printed parts, where the arm's gripper can
    take it and put it back from a stored pose, and it stays on through walking and lying down (holding force against
    the hat's mass and the walking accelerations, stated). Seated, it blocks neither the head camera's nor the lidar's
    view, nor the arm's fold; its magnets keep the same stated distance from the arm's encoders and the robot's IMU as
    the bin's ([Frm]).
## Decisions

1. [Lsn] **Designed as parametric files in the repository, Fusion for any touch-ups** (proposed, 4 Oct 2026). A part
   generated from code (dimensions in one place, read from the robot parameters and the reference models) can be
   regenerated when the position or a measurement changes, and checked by tests; the exported files still open in
   Fusion. This needs a code-CAD library (build123d or CadQuery), a new dependency, taken under [eM8]; considered:
   designing only in Fusion by hand (no tests, no regeneration), OpenSCAD (weaker for fillets and assemblies).
2. [uqL] **Stiffness predicted by a simple model first, finite elements only if it is close** (proposed, 4 Oct 2026).
   A beam and plate estimate gives the mount's bending and twist with a stated margin; only if the margin is thin does a
   finite-element tool (a further new dependency, under [eM8]) earn its place. The printed part is measured either
   way, and the measurement is what the sim uses.

3. [Qfe] **The first concept: an adapter on the four rear holes, a module that carries the arm and its host, and the
   arm's base kept where the sims put it** (4 Oct 2026; `cad/mount/`, its numbers in `report.json`).
   - The datum: Unitree's STEP of the Go2 with the EDU dock (`GO2简化-带拓展坞-0911.stp`, 2023-09-11, mm), read by
     `tools/go2_step_cache.py`; the parts' ids in `cad/mount/go2_ref.py`. Its axes are X across (+ left), Y along (head
     at low Y), Z up; the thigh joints lie on Z = 0 at Y = 108 and 494, so the body centre is (0, 301, 0) and the robot
     frame is X = 301 - Y_step, Y = X_step, Z = Z_step. With the dock removed the top cover is flat at Z 57.0 from
     X -120 to +112 within |Y| 38; the rear holes are at X -62.7 and -89.3, Y ±20.3; the front holes (7.0 mm) at
     X 112.3, Y ±19.1. These are `go2_air.back_mount` (source: vendor). The threads, their depth and what lies under
     the holes are not modelled.
   - The adapter sits on the flat of the back's top cover, bolted through the four rear holes Unitree's CAD shows
     (7.2 mm, 40.6 x 26.6 mm). It reaches from the cover's rear edge to where the head rises, and its underside clears
     the cover's side ridge and shoulders.
   - The module joins the adapter by four M5 into heat-set inserts (two behind the arm base, two on ears beside it) and
     two dowels. The bench plate carries the same interface.
   - The arm's base seat stays at the sims' height (118.4 mm above body centre: the body top they use plus
     `mount_plate_m`), so the sims' reach and tipping results still hold.
   - Every part fits the X1 whole, so no part has an internal joint.
   - Considered: the front holes as a second fixing (their thread and purpose are unknown); a raised bin seat (see
     [MEp]).
4. [MEp] **A flat deck with thick skins, printed deck-down** (4 Oct 2026). Walking fatigue governs ([Xtj]: about 1 MPa
   of stress range).
   - Beam estimates in the adapter: 1.42 MPa with 8 skin layers; 0.85 MPa with a 16 mm raised bin seat, whose step
     would hang over the bed; 0.98 MPa with a flat deck, 18 top and bottom layers and 8 walls (under 1.03 for CarbonX
     PC+CF and 1.16 for PC).
   - The flat deck and thick skins were chosen, at about 100 g more.
   - If the analysis ([tY3]) finds the margin too thin, the next step is to raise the arm 16 mm. That is the sims' call,
     through `mount_plate_m`.
5. [pIr] **The bin and the hat locate mechanically and are held by magnets, kept away from the encoders and the IMU**
   (4 Oct 2026).
   - **Bin:** sits on two non-magnetic stainless balls in the deck, through a 45 deg cone socket and a V-slot.
   - **Hat:** its crown opening drops over a boss.
   - **Magnets:** Ø10 x 3 mm N42 pairs (supermagnete S-10-03-N, 17.7 N on steel, observed 4 Oct 2026), taken at 60%
     for the 0.6 mm gap.
   - **Keep-out:** every magnet at least 120 mm from the arm's motors and 80 mm from the Go2's IMU.
   - Considered: printed cones on the deck. They would stand off the bed when the adapter prints upside down.

## Open questions

1. The Go2 Air's mounting points: Unitree's own STEP of the Go2 (simplified, with the expansion dock; `models/go2/_src/`,
   found 4 Oct 2026) gives the body's contours and the dock's attachment; whether the Air's back is the same shell, and
   what load Unitree rates the points for, are checked on the real robot (calipers) on day one:
   - the cover: the flat 57 mm above body centre, the side ridge, the rear lip;
   - the four rear holes: positions (40.6 x 26.6 mm), diameter (7.2 mm), the thread, its depth and what it is in; then
     set `REAR_BOLT` and `go2_air.back_mount.thread`;
   - the front holes and slots: whether the Air has them, and whether they are threaded (a second fixing);
   - the arm base's M5 square (79 mm) and nut traps, on the real base;
   - the bowler: brim, crown opening and mass (estimated 50 mm, 31 mm, 40 g);
   - after printing: each part weighed ([mgb]) and the twist and droop at the arm's base under known masses ([lUO]).

## Tests

| Criterion | Test | Status |
|---|---|---|
| [I9y] (real models, placed) | the reference assembly from the measured bolt patterns, every number's source recorded; `tests/test_mount_cad.py` (arm base at [Pvh]; the adapter on Unitree's cover) | Partial: vendor-CAD numbers recorded 4 Oct 2026 (`go2_air.back_mount`, `maker_arm.base_bolts`); reference assembly `models/go2/_src/go2_with_mount_reference_mm.stl`; the adapter sits no more than 0.23 mm into the cover (within the 0.4 mm meshing tolerance). Nothing measured on the Air yet |
| [vqz] (the adapter) | load cases computed with safety factors; the printed part loaded to the rated case; `tests/test_mount_cad.py` (walking stress within the fatigue allowable) | Partial: load cases in `cad/mount/load_cases.json` (4 Oct 2026). Beam stresses 0.49 sustained, 5.67 working, 0.98 walking range, 3.93 fall (MPa), all within [Xtj]'s allowables (walking: 1.03 CarbonX, 1.16 PC). Not yet load-tested |
| [vQO] (trays) | every board held, connectors reachable, nothing trailing (photo check) | Partial: bays designed 4 Oct 2026 for the Pi 3B (stand-offs, a window to its USB/Ethernet), the buck, the CAN adapter, the speaker (grille) and the mic (port), with power and data connectors in the side walls; not printed |
| [Frm] (bin) | volume and opening against the toy set; the arm drops into it in the sim; the magnets' release force and repeat position measured; their distance to the encoders and the IMU; `tests/test_mount_cad.py` (magnet distances) | Partial: 117 x 147 x 100 mm inside (1.7 l), 21 mm under the folded arm. Predicted 4 Oct 2026: pulls off at 50 N, slides off at 93 N, 5.8 x the walking lift; 170 mm from the nearest motor, 83 mm from the IMU. Not measured; the sim drop not run |
| [mgb] (weight) | predicted against weighed per part; total within [Rrm]; centre of mass recomputed; `tests/test_mount_cad.py` (predicted mass within budget) | Partial: predicted 4 Oct 2026 at 1.42 kg (PolyMax PC), 1.43 (Bambu PC) and 1.58 (CarbonX PC+CF), against 1.8 kg; not weighed; centre of mass not recomputed |
| [lUO] (flex) | predicted stiffness; measured on the printed mount; the sim's parameter replaced | Partial: beam estimate 4 Oct 2026 at the long-term modulus, with the adapter alone and with the adapter and module bonded as the two bounds. PC: bending 970 to 1963, twist 734 to 991 N·m/rad. CarbonX: 3280 to 6635 and 2480 to 3348. The sim's 1500 is unchanged, pending a measurement |
| [ngX] (printable) | every piece within the X1's 256 mm build volume, oriented, exported with its settings; joints located and in the analysis; `tests/test_mount_cad.py` (fits with 5 mm to spare; overhangs and bridges) | Partial: 4 Oct 2026, every part fits whole (largest 245 x 228 mm) and prints without support; 3MF and STEP in `cad/mount/print/` with settings in its README; joints in `load_cases.json`. Hole clearances wait on the coupon print |
| [Uaq] (no collisions) | the real models through every posture in the sim, clearances reported; `tests/test_mount_cad.py` (the adapter on the cover, the module clear of the body) | Partial: static check 4 Oct 2026 (`report.json`). Legs at least 28 mm away standing, lying and walking. Module 4.0 mm above the head's rise. Battery 29.5 mm, its port 36 mm, its way out clear. No part blocks the head camera or the lidar. Motions and the moving arm not yet run |
| [uip] (builds logged) | each version's record | Not yet written |
| [JW7] (one removable module) | the module comes off the robot and onto the bench plate by its stated fasteners and two connectors; the arm's position against the plate's reference repeats within a stated tolerance; `tests/test_mount_cad.py` (the same interface on the adapter and the bench plate) | Partial: geometry 4 Oct 2026 (4 M5 and 2 dowels, identical on both); repeatability not measured |
| [tY3] (FEA and its film) | each load-bearing part analysed per load case: deflection, twist, peak stress and factor; the prediction checked against the printed part's measured stiffness; one clip per load case, contact-sheet checked | Not yet written |
| [Xtj] (design values, with margin) | each candidate's allowables traced to datasheet and independent sources, every knock-down factor stated; the FEA uses them and reports its margin | Partial: allowables recorded 4 Oct 2026 (`cad/materials/allowables.json`, sources in `independent.json`). No independent test of any candidate exists; other PC grades show 0.35 to 0.61 of XY strength across the layers (makers: 0.78 to 0.90). Knock-downs: print 0.90/0.80, moisture 0.90, 40 C 0.85, creep 0.50 (modulus 0.70), fatigue 0.033 at 1e7 cycles; safety factor 3 (fall 2, fatigue 1.5). Bambu PC: working peak 13.9/5.9 MPa, long-term modulus 976/722 MPa; CarbonX ezPC+CF: 12.2/4.3 MPa and 3298/722 MPa (modulus unverified, assumed). Coupons on the X1 and the FEA's margin not yet done |
| [4JP] (the hat's seat) | the arm takes and returns the hat in the sim from its stored pose; holding force against walking; views and fold clear; magnet distances; `tests/test_mount_cad.py` (magnet distances) | Partial: seat designed 4 Oct 2026, holding 10.6 N against a 40 g hat (est.), 13.5 x its weight at 2 g. 3.1 mm from the folded arm, 125 mm from the nearest motor and the IMU. The arm's reach to it is not checked in the sim |
