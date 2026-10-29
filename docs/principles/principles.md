# Principles

**ID:** [Dr5]

These constrain every decision in Houndkeeper. Changing one is a project inflection, not an edit made in passing.
Drafted 2026-09-24 from the rules set while the project was a grant proposal; review before relying on them.

---

1. [zGo] **The robot's camera data stays in the house, and is never handed over in bulk.** Raw camera footage, recordings and photos stay on machines in
   the house. What may leave: geometry, telemetry, logs, metrics, and footage deliberately chosen for publication.
   The line is "leaves the house", not "leaves the robot". Named, so it isn't read wider or narrower: lidar scans,
   odometry, joint states, IMU and battery data may leave; any camera's images may not, and neither may the dense depth
   maps the wrist stereo pair makes from them (camera-derived, they can show a person's shape; unless a deliberate
   decision says otherwise). The limit is on what the robot records: raw RGB, recordings and training data. Footage
   deliberately set up and shot (the robot working, a person in the office) may be published, with very selective
   exposure of the house and children never in it ([JHu]).
   **What this guards:** the robot's raw camera data, recordings and training sets are not handed over in bulk, to
   Dimensional (DimOS) or anyone. External AI in general is not ruled out; its use is intentional. Sending images to an
   external AI service (Anthropic, OpenAI) is allowed when intentional and signed off ([eNx]):
   what goes is what the question needs, screened for people, never a child ([JHu]).
2. [JHu] **Children stay out of any visual data and any published material,** and are never named.
   Children at a public event may appear in
   published footage only unidentifiably, so no parent could tell it was their child: from behind, close on hands, or
   super wide, and anything that slips blurred in the house before upload; never in the robot's own footage.
3. [eNx] **External AI is used on purpose.** Local by default; a hosted model (Anthropic, OpenAI) may see house data
   when that use is intentional and signed off: named in a spec or decision with what it sends, why, and how often,
   approved before anything is sent, and each send logged on the laptop. The principle does not bar external AI; it
   makes each use deliberate. Never bulk raw camera data or training sets ([zGo]). No cloud GPUs are rented.
4. [sJQ] **Every number is measured, vendor-published, or marked as an estimate.** "Unknown" is a valid answer; an
   invented number is not. Robot parameters carry their source.
5. [5oU] **The robot never endangers itself or anyone near it.** The arm moves only when the area around it is clear;
   it stays inside the tested tip-safety envelope; when in doubt it stops and asks for review. When the arm may move
   relative to what the base is doing is an operating rule, and each spec that uses the arm states it.
6. [1kb] **It runs on DimOS.** Capabilities are DimOS modules and skills. Changes that belong in DimOS go upstream
   rather than living as private patches.
7. [33f] **The sim is the harness; the real house is the judge.** Every sim scenario that gates a deliverable has a
   real counterpart measuring the same thing, and the gap between them is reported, not assumed away.
8. [CaM] **The sim looks like the house.** Where a realistic asset exists, the sim uses it: scanned real toys and
   household objects, generated floors with real pile and pattern, real proportions and masses. Primitive shapes are
   stand-ins, named as such. Realism covers everything a camera can see, not just the target: whatever a camera sees
   in the sim, the camera on the real robot would see too.
9. [x3f] **The sim is never kinder than the real world.** Where the real robot is imperfect (a printed frame that
   bends, gearbox backlash, a controller's model error, pose noise, lens distortion), the sim models the imperfection
   at an estimated size, or a pessimistic one when unknown, rather than leaving it ideal. A solution that only works
   on a perfect sim is not a solution.
10. [Feg] **Every sample carries the time it was taken, at its source; clocks are compensated, never assumed equal.**
   A camera frame, a lidar scan, an odometry message and an arm reading each keep the time they describe, not the time
   they arrived; where two clocks meet, their offset is estimated and kept up, and state is looked up at a sample's
   own time (interpolated, or rewound). Delay then becomes a correction, not an error. Borrowed from game netcode and
   sensor fusion (waymark [uKM]). What counts is that the robot's own clock is self-consistent: every sample's time
   known relative to every other's, which is what the calculations need. A lag that disagrees with what the
   pipeline is known to do is a fault in the stamping or the estimate, found and fixed, not a number to pick.
11. [fXD] **The robot never assumes it knows exactly where it is; it registers against what it sees.** A pose from
   odometry or the lidar is a starting guess good to centimetres, never millimetres. Where the world shows where the
   robot is (a tile's edge, a carpet's border, a wall), each observation is laid onto the earlier ones before it is
   judged, and what disagrees after that is the change; views of one place from different times are fitted together
   into the one arrangement where they all agree, as a panorama is stitched. Imperfect position is leaned into, not
   fought: finding itself two centimetres off from where it thought it was is normal, and corrected.
12. [0Py] **Nothing flashes in visible light.** Safe for photosensitive people, near the robot or
   watching its films. Any
   visible light the project makes, on the robot or for it (a lamp, a status light, a costume's eyes, a demo film),
   is steady or changes slowly and smoothly, and nothing can make it step or strobe, a fault or a bad command
   included. Light that must pulse is infrared, outside what people see (940 nm).
13. [eM8] **Neither cargo-cult dependencies nor reinvent wheels.** Each tool is adopted for a need, not because it is
   standard, and nothing a library already does well is rebuilt for its own sake. Before adding a dependency, and
   before building something a library already does, four questions, written down: what need it serves here (the
   job, not the tool); what DimOS already offers for it; what writing it ourselves would take and keep costing; what
   the dependency costs (size, platforms including the robot's own ARM computer later, maintenance, licence). Decided
   on measured lift against what we have, the same measures as everything else; standard solutions are where the
   search starts, not where it ends. The register is [eK3].
14. [Yj2] **The project runs from the phone.** As much of the work as can be (reviewing last night's report,
   tuning and adjusting, giving feedback and decisions, managing what runs next, writing illustrations and video
   scripts, reviewing renders) is done from a phone, without sitting down at the desk: progress has to be possible
   remotely, since the desk is not always within reach. Anything that must be seen or decided is readable and
   actionable at phone size; a result that needs the desk to understand is not finished. Private as everything else:
   over the tailnet or the project's own accounts, never a public page ([zGo], [rhP]); shared beyond the project only
   with invited people, and only what was made to be shared (the video app, [kuU]).
15. [MJo] **The field's own words, then the picture that makes them obvious.** Docs, findings, video scripts and
   narration name things by their standard term and then explain it with an everyday image, never the image instead of
   the term: call it a constraint or an edge, then show it as a spring.
   So a pose graph's link between two poses is a constraint (an edge; a factor in a factor graph), drawn and described
   as a spring; its stiffness is the measurement's information; a robust kernel is a spring that goes slack when
   overstretched. A viewer or reader who goes on to the literature finds the same words there. Project nicknames for
   our own parts are kept only where no standard term exists. The terms and their images: `docs/glossary.md`.
   In the video the order is the picture, then the term, then move on: an intuition for how it works, the term for
   whoever wants more, no maths and no detailed system design, and no explicit pointers to look a term up. The term
   is simply used in the narration, as a word the narrator knows, never introduced, defined or pointed at; no
   equations, no architecture diagrams; the detail lives in the docs for whoever goes looking.
16. [JqF] **One impersonal voice.** Specs, waymarks, principles, docs and comments say what the project does and why,
   in the project's own voice: no "the owner", no quoting the person who asked, no account of who decided. The reason
   a rule exists is written as the reason; who asked and when is git's to record. Dates stay where they date evidence (a
   measurement, a run, a finding), not a conversation.
