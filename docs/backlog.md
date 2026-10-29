# Backlog

Why this exists: work parked or queued, with the reason and the date, so it is picked up from where it
was left rather than rediscovered. What is decided lives in the specs; this is what is not yet. Numbers marked (est.)
are estimates; everything else was measured, in the sim unless it says otherwise.

## Tables as tidy surfaces (later phase; parked 24 Sep 2026)
A table becomes a second surface to scan and keep tidy. It knows what belongs on the table and flags only what doesn't.
Not in the first phase. Kept possible by [t8K] (hardware and map don't assume floor-only) and [28K] (mount sweep scores table reach).

## Dense wrist-view geometry for grasping (idea, 24 Sep 2026)
Feed-forward multi-view geometry (DUSt3R/MASt3R, VGGT, pi3, MapAnything) gives a dense 3D point map from 2-3 images in
one pass; single-image metric depth models (Depth Anything V2, Depth Pro, MoGe, UniDepth) are a lighter option. With
camera poses known from the arm's joint angles, these would give item height, outline and narrowest side in 3D, for
tall, soft and odd-shaped toys and low-contrast real carpet. Local only; whether VGGT fits the RTX 2070's 8 GB is
unverified. Gaussian splats and feature fields (F3RM, LERF-TOGO, GraspSplats) need too many views per find. Don't use
the sim's rendered depth for grasping: it hides the problem these models solve.

## Group several finds into one lying-down stop (backlogged 25 Sep 2026)
Each find now gets its own head-on stop with the item in the front reach zone ([S7o]). Grouping finds into one stop
saves lie-downs but needs side-band picks and precise parking; revisit once front picking is reliable. The code
(`cluster_stops.plan`) was deleted on 25 Sep before the repo's first commit, so it is not in git. Its last evaluation
(PoC nights s8 v2, 8 nights): 257 flags (113 real) became 163 stops, 1.58 flags a stop, every flag given a parking
spot, route 39 m a night. Its design, for a
rewrite: each stop is a lying pose (x, y, heading); everything inside the collision-checked lying reach map (then ~0.2
m2: two side bands and the front corners, eroded by 5 cm for parking error) is handled from it. Candidate poses were
sampled around each find (16 headings), kept only where the lying footprint (0.40 x 0.25 m half-extents incl. splayed
legs) clears walls, furniture, chairs and other finds; a greedy set cover picked poses covering the most uncovered
finds; stops were ordered nearest-neighbour from the dock, then improved by 2-opt; distances straight-line.

- **Ideas borrowed from MOSS are now in specs**, credited there: arm stops, arming, first-run caps
  and replay tests ([41H], [3dt], [JYS], [wcJ] in maker-arm-adapter); robot-free decision logic and the "against a
  person" yardstick (houndkeeper spec); the hardware build log ([E2T]); pooling and reporting rules ([Yms], [c9x]).
- **Look at MOSS**: github.com/metrox-eth/moss, an open, mostly 3D-printed tracked litter-picking
  rover with an SO-101-derived arm, teleoperated on dimOS and recording LeRobot-format demos into a pooled open dataset
  (Apache-2.0 code, CC BY 4.0 hardware; early prototype, dimOS side "cold-tested only" per its README, checked
  2026-09-25). Worth reading for: its pick pipeline and dataset layout (compare with [95i]), its Pi 5 build (arm-host),
  and its dimOS modules. Their pooled data would not transfer directly (different arm), and contributing ours is
  limited by [zGo]: only sim data or footage deliberately chosen for publication.

## Interactive mode: "Hound, pick up!" (parked 25 Sep 2026)
DimOS is built for agentic robots; this is the tidy made conversational. Someone says "Hound, pick up!" and points
at something on the floor with a hand or foot. That skips the first stage of detection (the floor comparison): the
person has already seen something there, so the robot goes straight to finding what is pointed at and picking it.
The demo: the robot is idle in another room, hears the call, walks over and finds the person, follows the point to the
item, and puts it in the bin.
Needs, roughly in order: a wake phrase heard on the robot, local speech only ([zGo], no hosted models); finding the
person who spoke (DimOS person tracking) and walking to them; reading where they point (arm or foot direction from a
local pose model, meeting the floor) into a search area; then the existing look-closer and pick on that area, with the
same confirm, retry and give-up rules as the night tidy. People are in view by design here, so this must first be
squared with keeping people, and especially kids, out of visual data (the house rules); daytime use also means more
light and more clutter than the night tidy. Not specced yet.

## A fully held-out house run (once the tidy works well)
The final exam for the whole tidy, run once everything is frozen: a house the robot and every tuning step have never
seen. Generate a new house (new floor plan and rooms, new floors and rugs, new wall/furniture/fabric textures, new
lighting draws) and put down held-out toys only ([nuQ]); run a few calibration sweep nights (the baseline nights, and
the house map built from them); then run the tidy nights and score them once: items binned, stops for nothing, items
left for review and why, night length and energy. Nothing is tuned on it; a second look means a second new house.
Needs first: a house layout generator (today `make_house.py` has one fixed floor plan, so every sim so far is the same
house), the held-out item set ([nuQ]), a held-out texture set (textures kept out of the tuning houses), and a frozen
pipeline (detector, confirm-on-approach, pick) with every parameter written down.

## Re-gripping through the centre of mass: dropped as a lot of effort for very little gain
The grasp already goes through the middle of the item's footprint; what's left is the 1-2 cm between that and the real
centre of mass (a shoe's heavier heel), which only the wrist's motor torques could sense (~0.03 N m for a shoe, likely
near the RS00s' torque resolution). Items turning in the jaws are left to the pads ([wjx]). Not to be re-proposed
without a measured reason.

## Rugs larger than the floor patch (noted 25 Sep 2026)
Rugs are drawn inside the 1.3 m wrist-resolution patch ([N4z]), so none is larger than about 1 m. Real area rugs are
often 1.5-2.5 m, with the robot working entirely on one. Needs the rug drawn into the surround cells too (a rug-sized
texture over them, not the repeating floor), without reintroducing overlapping floors.

## Major floor changes: a new rug, blended in (parked 26 Sep 2026)

Split from toy detection: the house nights put the rug down from the first night, in the baseline ([yyJ]). Later test:
a rug that appears partway (`make_nights.py --rug-night 3`), with the new floor blended into the floor geometry, and the
robot's accuracy allowed to dip for a few nights while it learns the new floor. Known: on 26 Sep a new patterned rug
broke into toy-sized blobs (17 stops for nothing in one night); grouping blobs (`--group-m`) helped little on the bench.

## Revisit if grip trouble shows up: work done on the one-lamp renderer
Until 30 Sep the renderer anything fell back to drew one lamp per frame (waymark 0007 [3my]); the scored rounds, the
hold-outs and the detection benches chose the fixed build by name, so finding's numbers stand. Done on the one-lamp
build, and to revisit if grasping or the close look gives trouble once more toys reach that far down the funnel:
- the arm sim's grasp and stereo benches (`sim/arm/grasp_bench.py`, `stereo_eval.py`, the width benches): re-run on the
  fixed build before leaning on their numbers;
- the parked learned pick's demonstrations and its ACT training ([NuZ]): re-record and retrain if it is unparked;
- settings calibrated before the fixed build (28 Sep): `make_nights.LIT_CEILING` (each lit fixture's strength, set by
  the cameras' gain over house 70) and any early detector threshold; if measured on the one-lamp build, the lit house
  now renders brighter than intended (lower gain, less noise: kinder than meant, [x3f]).

## The learned pick: ACT now, a VLA maybe later
The learned pick stays ACT (Action Chunking with Transformers, LeRobot): a small model (~80M parameters, per the ACT
paper) trained from scratch on this one task's demonstrations, wrist image and joint state in, a chunk of joint
targets out. It fits the task (one fixed pick), the RTX 2070 (60k steps in ~80 min) and robot-class compute. The real
robot still starts on the scripted grasp; the learned pick is sim pretraining plus real fine-tuning later.
A VLA (vision-language-action model: a pretrained vision-language model trained further on many robots' data, e.g.
SmolVLA ~0.45B, pi0 ~3B, OpenVLA ~7B) is the candidate to revisit. What it could bring: pretraining on a wide range
of real images, which may make it steadier under lighting and textures it never saw in our demos (the sim-to-real
worry), and a language input that suits the interactive mode ("pick up the green dinosaur"). What it costs: far more
compute (a 7B model won't fine-tune on an 8 GB card; whether SmolVLA does is untested), slower inference on the robot,
and the same need for demos of this arm. Open weights only, run locally ([zGo]: no hosted models).
Cheapest first try: SmolVLA through LeRobot on the same demonstrations as the current ACT, scored head to head on the
same held-out seeds. Revisit when ACT's gap to the scripted grasp stalls, when real footage shows it breaking on
lighting or floors the sim didn't cover, or when the interactive mode is specced.

## How narrow can the sim-to-real gap get before hardware?
An exercise worth running for its own sake: once real night video is recorded, refine the sim toward it and
measure the gap. The footage stays in the house ([zGo]) and is kept free of people. Ways to score the gap, cheapest
first:
- **How different the pictures look:** real and sim frames of like scenes (same floor type, light, camera height),
  compared by image statistics (e.g. KID on floor crops, colour and noise histograms). Says how far apart, not
  whether it matters.
- **Whether perception behaves the same (the gap that matters):** run the floor comparison, confirm-on-approach and
  the close look on real footage and on the same scenes in sim; compare false alarms per night,
  misses and the close look's naming. A sim tuned until these match is a sim the tuning can trust.
- **A camera model after the renderer:** fit sensor noise at the real exposure, motion blur, the lens's distortion
  and the camera's compression to the real footage, as its own stage after Filament (waymark [UQK] names the missing
  effects).
Sim-side levers to pull as the numbers point: photographed floors ([N4z]) and wall and furniture textures ([QMt]),
light units and exposure ([Hsz]), scanned toys, the camera model. Record each change with its before-and-after score.
Only stretches of footage `tools/screen_people.py` clears are used.

## Soft items and cloth: deformable bodies (parked 26 Sep 2026 after an hour-boxed try)
Every item is rigid except a cloth sock (a MuJoCo flex) and a rope (a cable), both only in the arm-sim benches. Real
floors have soft things: towels and clothes (cloth), lunch boxes, plush and fabric cases (squash about a centimetre
under the jaws' ~7 N, est.), shoes (a few millimetres). Contact softness was tried first, 26 Sep, and doesn't do it:
MuJoCo's softness applies to every contact the item has, so an item soft enough to give a few mm at the jaws sinks
12 mm into the floor under its own weight (and softer settings go unstable); milder settings give under 1 mm.
Pad-item contact pairs could separate the two but run to hundreds of explicit pairs per item. What it needs:
- **Filament must draw flex bodies first**: mujofil draws geoms only, so a flex cloth is invisible to the cameras. A
  patch like the floor fixes ([akm]): a mesh rebuilt from the flex's vertices each frame, textured from the scan.
- **Towels and clothes as flex cloth**, sized to the scan and wearing its photo.
- **Soft items as 3D flex or a soft shell around a rigid core**, calibrated from a squeeze test (force vs give).
- Which items, and how much give (est.): lunch, teddy, toy dog, the foam dinosaur, cold-therapy elephant, fabric pencil
  cases, towels, the hat: about 12 mm; every shoe: about 4 mm.
- **Done 26 Sep:** Filament draws flex bodies (waymark [akm]); the four towels are flex cloth wearing their scans'
  photos (`items.TOWELS`), stable at DimOS's 5 ms Euler step once the cloth's contact time constant was 0.02 s (0.01,
  the old cloth sock's, is exactly MuJoCo's two-step floor and blew up). Not yet in the house nights (the scorer looks
  items up by body, and a cloth has none of its own) nor the demonstrations (`grippable` leaves cloth out).
- **Still open, 3D soft bodies** (lunch boxes, plush, shoes): `flexcomp` boxes with solid elasticity went unstable at
  DimOS's 5 ms Euler step at every stiffness tried, with or without the 8-point `dof="trilinear"` form; with the
  `implicitfast` integrator they ran stably, but switching DimOS's integrator changes the whole robot's simulation
  (its gait included), so it wasn't done.
- The jaw pads' "give" (waymark [wjx]) is modelled the same way (`solimp` width) and so probably doesn't compress
  either; check it when this is done.

## Improvements parked on 22 Sep 2026, before the grant
Written while the project waited on the grant and brought in from the grant package on 25 Sep 2026; paths updated to
where things are now, content otherwise as written on 22 Sep and not re-checked against later work (some has since
moved on: the sim now renders carpet pile by viewing angle, [N4z], and detection's launch bar is [tTD]'s, not the 72%
below).

### Where things stand (sim, detector v2, held-out nights 5–8 of re-sweep 2)
- Found 72% (easy 86%, medium 74%, hard 54%). About 43 false alarms a night.
- Look-closer keeps 95% of real finds and removes 29% of false alarms, leaving about 30 a night.
- Navigation: 49/49 waypoints every night; stalled 0–2% of the time.
- The overnight chair timeline (`sim/house/runs/nights_s8/`, real-time sim speed) was still running when this was written. Its results go in `final.txt` there.

### False alarms cost time and battery, not correctness
Each false alarm means stopping and deploying the arm for a closer look. Too many, and the robot runs out of battery before the night is done.
- Go2 battery: about 1–2 h (Air/Pro), about 2–4 h (EDU), per Unitree.
- Sweep of the sim layout (~1,770 sq ft) at real time: about 14 min, about 25 min including the mapping pass.
- One closer look with the arm: 20–40 s (est.). At about 60 looks a night, that's 20–40 min (est.), more than the sweep itself.
- Ranking candidates by confidence barely helps: 30 looks a night reaches 52 of 91 real finds, against about 41 for random picking.

Ideas, cheapest first:
1. **Use the other views the sweep already took (multi-view consistency).** The lawnmower route sees most spots from several positions. Align each view on the floor around a candidate (never on the candidate itself), then check what's left over:
   - **Real things** appear consistently across views. Pile sheen changes with viewing direction and doesn't.
   - **Things with height** leave a consistent parallax shift. A 1 cm lego seen from 1 m projects about 3.5 cm off its footprint from the head camera (est.).
   - **Flat marks** line up with zero height.

   So: inconsistent means noise, drop it with no stop. Consistent and old means a known mark. Consistent and new means a candidate, and tall-and-new means the most confident candidate. It costs no stop. A simple two-view check exists in the detector (`TWO_VIEW`), switched off.
   Limits: small items need enough pixels in two or more views, and floor alignment must be within a few mm.
2. **Local VLM at look-closer.** Once geometry has picked an area of interest, a small local vision-language model names what's in the wrist-camera crop (sock, crumb, cable, cat toy). That decides pick, ignore or human review, and which grasp to use. It must run locally (no hosted APIs); a few-billion-parameter model on the RTX 2070 (est.). It only ever sees one crop, never the whole house.
3. **Cheap look before the arm.** Stop and pitch the body down, or step closer, so the head camera gets a sharper view. Deploy the arm only for what survives.
4. **Look after the sweep, not during it.** Gather candidates, then visit them in one efficient loop, inspecting all candidates within arm reach from one stop.
5. **Carry unchecked candidates to the next night.** Texture and alignment false alarms rarely repeat at the same spot; real clutter does unless picked up.
6. **Report false alarms as minutes of looking per night** against the battery, not as a raw count.

### Finding small things (hard items, 54%)
- **Build floor pictures at about 5 mm near the robot, not 1 cm.**
  - The sim's head camera resolves about 1.3–2.4 mm across within 1 m. We throw that detail away.
  - Finer isn't automatically better. At full camera resolution the individual carpet tufts read as change, so about 2–5 mm looks right for carpet.
- **Two-stage alignment.** The robot's position estimate gives coarse alignment (the sim's floor pictures land about 2 cm off; the real Go2's accuracy is unknown). A local image match against the stored floor then snaps it to within millimetres.
- **Fix the floor model's waviness.** It went from 2.8 to 4.3 cm over re-sweep 2 nights 5–8, and blur is about 3 cm.

### Carpet realities
- **Walking on carpet changes its look** (the pile flips).
  - Fix: keep several looks per spot (vacuumed, walked on, lamp on), and flag only what matches none of them.
- **Carpet looks different from the opposite direction.**
  - Fix: compare each spot against past views taken from a similar direction. The lawnmower route passes each strip in alternating directions on a regular pattern, so matching views build up.
- **Sunlit patches** add false spots at their edges, and sunlight moves between runs. Night runs avoid this.
- **The sim models none of these.** Its carpet drifts by about ±1% and renders the same from every angle.
  - Add pile-patch drift to the floor generator, calibrated on the real robot's floors.
  - Viewing-angle effects can only be tested on real carpet.

### Items that stay out, and furniture that moves
- **Tall toys left out for two or more nights get absorbed as structure** (the furniture mask uses the obstacle map's "2+ nights" rule).
  - Fix: use the smart-clearing tenure rule from the house map, so walls and sofas earn permanence over many nights and a toy doesn't.
  - Also score tall items with the lidar layer instead of calling them unobserved.
- **Moving chairs** (added in timeline s8): see how often chairs, or the spot they just left, get flagged as clutter.
  - Likely fix: anything bigger than the arm can pick up gets labelled "moved furniture", not a candidate.

### The arm on the dog: risks (not pressure-tested yet)
Reach is about 1 m (est.). The plan is to crouch or lie the Go2 down before the arm moves.
- **Weight: low risk.** Arm ≈3 kg (est.), maybe 4–5 kg with mount, camera and power (est.). The Go2 carries about 7–8 kg normally (Unitree). The cost is battery life.
- **Reach: low risk.** Crouched, it covers about 0.9 m of floor around the robot (est.). One stop can inspect or pick several candidates, which pairs with batching the looks after the sweep.
- **Tipping: forward is comfortable; sideways at full reach is the case to check.** Crouching lowers the centre of mass but doesn't widen the footprint. At full sideways reach the centre of mass moves about 6–7 cm, against a half-width of about 10–15 cm (est.).
  - Keep arm motions slow and the heavy joints close to the body.
  - Compute the real margin from the URDF masses.
- **Walking with the arm stowed: medium / unknown.** A 1 m arm folds onto a 0.7 m body, so the folded shape sets clearance, and the high centre of mass is a concern over carpet-to-tile edges and on turns.
- **Precision at full reach:** printed-part flex and motor give add up over 1 m. Close the loop with the wrist camera (look, adjust, grasp); grasp near the body where possible.
- **Sensors:** the stowed arm may block part of the head lidar's view. The deployed arm appears in the head camera's view.
- **Power:** the arm needs 48 V; the Go2 battery is about 29 V nominal. That means a boost converter or a second battery. Power output may depend on the tier.
- **Tier:** SDK access, mounting rails and battery size depend on the tier. Crouching does not: the DimOS contact says the Air and Pro can crouch (1 Oct 2026).
- **Safety:** swinging 1 m of arm near furniture, pets or people means the "room is clear" rule must be enforced, not assumed.

**Picking up:** gripper and object choice matter more than the learning method.
1. Scripted straight-down grasp for socks, tissues, paper and toys of about 5 cm or more. That's enough for Phase 4.
2. Cheerio- and lego-sized items sunk into carpet are hard for a parallel gripper, and suction fails on porous carpet. Consider a scoop or sweep attachment. Until then they go to human review.
3. Then demonstration learning (LeRobot ecosystem; ACT or SmolVLA-class; a few dozen to a couple hundred demos, est.), trained locally. Failed picks become new examples.
4. Sim reinforcement learning only for approach and arm positioning. Don't trust sim-trained grasps on small soft objects.

**Cheapest pressure tests:**
1. On paper, from the URDF: reach envelope, centre-of-mass margin crouched at full reach in each direction, and a power budget.
2. When the arm arrives (October), on a bench at crouched mount height:
   - weigh it and measure its reach
   - try straight-down pinches on real carpet (Cheerio, lego, sock, tissue)
3. With a robot: walk with a dummy weight at the mount point before fitting the real arm.

### The arm in the sim
MakerMods ships a URDF in `maker-arm-sdk` (`maker-arm-sdk/urdf/maker_arm/`); how complete it is is unknown.
1. **Arm on the dog, stowed** (a few hours, est.): mount with realistic mass (≈3 kg, est.) and check the walking policy still walks. That's Phase 2's "gait doesn't regress" test.
2. **Real look-closer** (about half a day, est.): wrist camera on the modelled arm, with the arm's reach limiting what it can inspect.
3. **Maker Arm driver in DimOS** (about half a day, est.): same interface as the real driver.
4. **Picking up** (1–3 days, est., least certain): litter needs collision and mass, a gripper model, and a finer physics step.

### Findings from the overnight real-time run (timeline s8, chairs, 12 nights)
- **Detector v2 on held-out nights 5–12: 58% found (91/156), about 18 false alarms a night.**
  - That's against 72% and 43 on the old timeline at ~3.3×. Real time, a new floor and chairs all changed at once, so the drop has no single known cause.
- **More misses are items seen as tiny specks.** Held-out misses that were detected and then dropped went from 8 to 34; most are under 8 px. Unobserved items went from 8 to 24.
  - Likely cause (not verified): per-cell hit-count thresholds tuned at ~3.3× become looser at real time, because there are ~3× more scans. The lidar "stands up" layer grew from ~23 to ~80 m², and more floor near tall things gets masked.
  - Fix: normalise count thresholds by how many scans saw each cell.
- **Look-closer:** keeps 84% of real finds and removes 53% of false alarms, about 8 left a night.
- **Chairs:** v2 flagged no chair and no spot a chair had left (0 of 175 false alarms).
  - But only 2 big moves happened, because the generator re-rolls the move type when a big move can't find space, so "moved a lot" was barely tested.
  - Fix: retry the position, not the move type.
- **Floor model is much better at real time:** fidelity 0.30 and 49% confident patches after night 12, with waviness 2.0 cm. The old timeline ended at 4.3 cm.

### Sim fidelity
- **Speed:** the sim used to run at about 3.3× real time. `DIMOS_SIM_SPEED=1` now runs it at about 0.91–0.93×. Earlier results were at about 3.3×, so re-run key results at real time.
- **Pass-under and sim speed are DimOS options now** (sim-harness [UvE]): `sim/dimos/env.py` sets the costmap's
  pass-under rule off (it marked half-seen furniture as open floor) and the ceiling cut; `sweep_night.sh` reads the
  config DimOS will run with and refuses to run without them, or without the fork's sim speed option.
- **Held-out integrity:** the nights 5–8 of each sweep scored so far are used up. Score any further change on fresh nights only.

## Arm host software, deployment and updates (shelved 25 Sep 2026)
Drafted as `docs/specs/arm-host.md` [JIY] and shelved until the sim works: rebuilt from the repo, ready
from power-up with the arm disarmed, updates only on purpose with rollback, no internet, video stays in the house,
clock sync, health reporting, power-cut safety, SSH-key access. Needed before the arm's bench test in October.

## Confirming an item landed in the bin (parked 28 Sep 2026)
"Binned" today means the jaws still held something over the bin and then opened; nothing confirms it left the jaws or
landed inside. One idea: check the robot's weight goes up. Options considered, and parked as none worth it yet:
- The whole robot's weight from its feet: a 100 g toy is under 1% of a ~15 kg Go2 (est.), the foot sensors aren't
  scales, and whether the Go2 Air reports them is unchecked. Unlikely to resolve.
- The arm's load before and after opening over the bin (it already weighs the item on lifting, [KrW]): says the item
  left the jaws over the bin, for anything heavy enough to weigh; not whether it bounced off the rim.
- A wrist-camera glance into the bin after release: works for anything, a Lego included.

## Dimmer nights, fewer lights left on for the robot (parked 1 Oct 2026)
Every bench and round runs the "lit" regime: the testing light, every fixture on, warm and fairly bright, the
same every night (light is a lever easily improved in a real house). The darker night regime already exists in
the generator (`make_nights.py regime()`: ambient 0.06-0.16, the lamp on 60% of nights, one ceiling light) but nothing
is tuned on it. For a future iteration, when the house shouldn't have to leave so many lights on for the robot:
tune and bench on dimmer nights, where the camera runs at full gain and its noise, not the floor, sets the limit.

## Next uses of what's built (1 Oct 2026)
Small add-ons that reuse what exists, before brand-new subsystems.
What already exists to build on: the floor's geometry (DimOS's pose graph, the lidar shape solve, the floor picture and
its change detection against clean nights); the lidar house map that knows where walls and furniture stand, night
over night; the pick, the stow, the tray and the close look; the nightly routine and its scoring.

- **Tidying tables.** The floor's geometry on any flat surface: the plane set at the table's height (the lidar map
  knows where tables stand and how tall), the surface mapped from clean nights with the wrist camera only, tonight's
  compared with it, and how much of the table is clear scored like the floor map error. Reuses the picture builder, the
  stitching, the change detection and the pick; new: holding the wrist camera over the table. (Also "Tables as tidy
  surfaces", 24 Sep.)
- **Dusting, as the logo shows.** A duster (kept at the cleaning station, below) sweeping the surfaces the lidar map knows (tables, shelves, the TV stand),
  in passes planned over each surface's outline, the wrist camera checking coverage. The table maps above could show
  dust settling over weeks (a slow brightening on dark surfaces): the cue to dust. New: a duster tool, sweeps that keep
  contact.
- **Blinds up and down.** A pick: grasp the bottom rail in the middle and lift or lower it; the windows and their blinds
  are in the house map and the sim. New: the pull-and-lift motion, and seeing that the blind moved. Reach is the
  question: a rail at the sill (~0.9 m) likely; the top of a closed blind not.
- **Tucking chairs in, by nudging rather than holding.** Each chair's home under the table is in the house map; the lidar
  sees where it stands tonight (chair moves are tracked night to night already). Short pushes with the body's front or
  the stowed arm's flat side, from the side away from the table, the lidar re-measuring the chair's place and turn
  after each, until it is within a few cm and degrees or N pushes are spent; off-centre pushes on the back turn it.
  Risks: tipping a light chair (push low, near the seat, slowly), scraping hard floors, carpet making pushes
  unpredictable. The sim's chairs are free bodies with friction: testable now, tipping included.

- **Wiping surfaces, from a cleaning station.** Nothing extra rides on the robot between jobs (the payload is too tight
  for permanent attachments): cloths, cleaner and the duster live at a station at a fixed spot in the house map, perhaps
  beside the dock, and the robot holds a tool only while using it. Cleaner goes onto a cloth with no lids or caps to
  work (something like a touchless dispenser: hold the cloth under it); dirty cloths go back to the station. Wiping is
  dusting with a damp tool, so the surface maps, the passes and the wrist camera's coverage check carry over. A mild
  cleaner, never dripping on the floor.
- **Clearing chair seats.** Crumbs and crud on the seats, dusted off onto the floor: each chair's seat is a small
  surface at a known height (the chairs' places are tracked already), checked with the wrist camera as the tables are,
  and swept off its edge with the duster; then the floor below is the vacuum's job (next).
- **A clear floor, then the robot vacuum.** Once the night's finds are put away and a room's floor reads clear (no
  candidates left there, nothing new in its walkways), send the house's robot vacuum to that room to vacuum and mop:
  it no longer eats socks, cords or Lego, and crumbs swept off chairs get picked up. Mopping keeps off rugs and carpet
  (both in the floor map). New: telling the vacuum which room and when, over the house network only (a local
  integration, no cloud: the house's data stays in the house).
- **Bringing a drink: the martini milestone.** Walk a full glass across a room level and hand it to someone seated
  (the channel's cold open: a wing chair by the fire, "Good evening..."). Reuses the pick, the carry and the house map;
  new: a glass held upright through the gait's sway (the arm's flex and backlash, as the sim models them), finding the
  person's hand, and a gentle release only once the glass is taken. Rehearsable in the sim first (a liquid's slosh as a
  tilt budget); a staged shot. In the sim the main thing is teaching it to hold a martini glass and
  move smoothly so it doesn't spill, and the sim's film is fine for the video; the real one is a plastic martini glass
  with water, in case it spills on the carpet.
- **The hat tip, for real.** A small, light bowler (printed, about 30 g est.) held on top of the head by a magnet: a
  steel disc stuck to the head shell and a magnet in the hat's crown, strong enough to stay on through the gait's
  jolts, weak enough for the arm to lift off. A little knob on the crown for the gripper. The arm unfolds, lifts the
  hat a few centimetres, a nod of the body, sets it back, folds: the robot's introduction, and the bedtime story's
  cold open. Reuses the pick and the arm's reach (the folded gripper already sits by the head); new: the hat, the
  disc, and a set-down that finds the disc again (the head camera can't see its own head: the arm's own pose does it).
  The brim stays out of the head camera's view and nowhere near the lidar under the chin. Rehearse in the sim first
  (the demo films' butler costume has the hat).
- **A morning report.** Tonight's comparison already knows what changed: toys put away, toys left (too big), new
  stains, moved chairs, the rug; one short note with a crop of each. Nearly free.
- **Stain alerts.** The detector already finds stains (the sim's nights have them, scored): report them on their own.
- **Doors, cabinet doors and drawers left open.** The lidar sees them stand out against the clean nights (a structure
  change, computed for the costmap already): report first; closing them by pushing later.
- **"Where did I leave my keys?"** Every clean night's floor views are kept with where they were taken: search them for
  an object (a word or a photo; OWLv2 is in DimOS) and say where and when it was last seen.
- **A trip-hazard patrol.** Walkways (doors, hall, stairs) weighted so anything new there is raised first; cords and
  curled rug corners flagged.
- **Pet bowls.** Known places (fixtures in the sim): a close look says whether the water bowl is empty.
- **Lights left on.** The camera's exposure already says how bright each room is at any moment: "the living room light
  is still on at 2 am" for free; the switch itself (~1.2 m) is a reach question, so report first.

More advanced:
- **Standing on its hind legs to reach higher.** Upright, the head rises to roughly 0.9 m and the arm reaches on from
  there: counters (~0.9 m) for wiping and clearing, the upper blinds and high shelves for dusting. Never with anything
  in the tray (it would spill, and its weight high on the back makes balance worse). Open questions: whether the Air's
  firmware stands on two legs and holds it under DimOS's control; balance with the arm's 4-5 kg on the back and moving
  while upright; a fall's cost to the arm, so only in a clear room, and tried in the sim first with the arm's real
  masses.

## Big ideas, parked (3 Oct 2026; the rest of the list went ahead)

- **"Barrymore, where did I leave my keys?"** A household memory: every patch of floor is photographed every night, so
  a question in words (phone or voice) is answered with a picture and a time ("Tuesday 11:42 pm, under the dining
  chair; gone by Wednesday"). All on the house's machines; DimOS's spatial vector store holds frames by place already.
  Needs: a query over the catalog's recordings and the floor pictures, the voice path.
- **"Put this there": teaching by pointing.** Point the phone at an object, then at a place; Barrymore learns where it
  goes, and the house's own tidy rules accumulate from it.
- **Thirty nights untouched.** Self-charging dock, a nightly schedule, recovery from his own mistakes, a month of
  morning logs with no human help: reliability as the headline (follows the self-charging dock).
- **The benchmark, given back.** Release the procedural tidy-house benchmark (scanned toys, real floor photos,
  lighting variety, the realism rules [CaM] [x3f]) and the DimOS extensions, so others measure against it. Parked,
  with its infrastructure built now: what a release needs (licences and attribution per asset, versioned house
  specs, deterministic regeneration, an export that leaves out anything of the house's own) so it is a command
  away when the time comes. Built 3 Oct 2026: docs/specs/benchmark-release.md [WFU], `sim/house/benchmark_release.py`;
  what a release still needs is in the spec (a licence for the project's own work, houses 70 and 95, licence checks).

## DimOS's agent on the Claude subscription instead of OpenAI

Analyse running DimOS's language-model pieces (the agentic blueprints' planner, MCP client, skills; today they expect
OpenAI, and our scripts unset OPENAI_API_KEY and disable mcp-client, speak-skill and perceive-loop-skill) on a
Claude subscription. To find out:
- **What DimOS needs**: which modules call a hosted model, through what client, and whether it takes another provider
  (Anthropic's API, or Claude Code driving DimOS's MCP server, which the tidy already exposes: `mcp-server`).
- **The subscription vs the API**: what a Claude subscription covers for an always-on robot loop (Claude Code over
  MCP, the Agent SDK) against API billing, and its rate limits for a nightly run.
- **Principle [eNx] (external AI on purpose)**: each use named with what it sends; the cheapest is text the robot
  already summarises (the mind view's stage and reasons, the morning log's facts, tool calls) or the user's own
  words; images when the question needs them; never bulk raw RGB ([zGo]).
- **What it would be for**: "Barrymore, come greet our guest"-style requests, planning a night from the morning log,
  explaining a night in conversation.
- **Object recognition**: naming what he found ("a
  sock", not "a plastic object"; the morning log's labels), and deciding what a find is when the local detector is
  unsure. This one sends pictures, so it meets [zGo] (nothing seen in the house leaves it) and [JHu] (children never
  in visual data) head on, not only [eNx]. To weigh: tight crops of the floor around a find only, from screened
  recordings with the people screen run on every crop ([R1X]), never a whole frame; compared, on the sim's own
  toys, against the local models (OWLv2 through DimOS first) to see whether the gain is worth it. Settled by
  [zGo] and [eNx] as amended: external AI is fine when intentional and signed off, named in a spec with what
  it sends and why, each send logged, screened for people, never a child; what stays out is raw RGB and training data in bulk,
  to Dimensional or anyone.

## A splat twin of the real house, after first boot (4 Oct 2026; decision [skp] keeps the floor a grid picture)

The house twin ([lQ1]) is boxes plus a floor texture. After first boot, train a Gaussian splat from one real night
using the poses the stitch already solves (no COLMAP), the lidar map as its seed cloud: render it from the real camera
poses and measure the gap ([iuP] for appearance); only if close, render the twin's rooms with it, MuJoCo meshes (robot,
arm, toys) kept and occluded correctly (Open3D 0.20 renders splats beside meshes; Vulkan, as mujofil already is).
Geometry stays the lidar's; a splat-derived hull (depth fused by TSDF, or 2D Gaussian splatting) may later open up the
spaces under furniture the low camera saw, kept only where the lidar or several views agree (no phantom obstacles).
Training on the laptop's GPU overnight, not onboard. Raw-RGB-derived: stays home ([zGo]); training frames screened
for people, no child in any splat ([JHu]).
