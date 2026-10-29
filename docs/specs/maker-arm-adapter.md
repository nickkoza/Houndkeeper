# Maker Arm in DimOS

**Type:** Capability spec · **Status:** Draft · **Source:** principle [1kb]; upstream contribution; [41H], [3dt], [JYS] and [wcJ] adapted from MOSS (see Decisions) · **ID:** [oIS]

## Purpose

DimOS can drive a Maker Arm like any other arm it supports, so anyone with a Maker Arm can use DimOS's arm tools,
and this project's arm code is the same in sim and on the real robot. Offered upstream to Dimensional.

## Acceptance criteria

1. [Odx] DimOS lists the Maker Arm among its arms and drives it through the same arm interface as the others (CALM
   node `dimos-arm-interface`).
2. [0h9] Every interface the model gives the adapter (CALM node `maker-arm-adapter`) works on the real arm.
3. [KXa] The same code drives the simulated Maker Arm.
4. [lSw] A lost link or fault stops the arm and reports why; it never keeps moving on stale commands.
5. [uiv] It meets DimOS's contribution rules ([2SB]) and is submitted as a pull request.
6. [41H] **Two independent stops.** The arm host stops the arm when no command has arrived for 300 ms, and the arm
   side stops on a timeout of its own as well, so neither a hung host nor a pulled cable leaves the arm moving.
7. [3dt] **It starts disarmed.** The arm moves only after an explicit arm step: after power-up, after any error reply
   (which also stops it) and after every stop. Reopening a lost link sends a stop before anything else.
8. [JYS] **First real runs are capped.** Joint speed and gripper force start at a commissioning limit and are raised
   only when measurements support it; each raise is recorded with its measurement.
9. [wcJ] **Its tests run without the arm,** against a fake link that replays traffic recorded from the real arm on
   its first bench day, including a pulled cable and error replies.
10. [cqG] **The arm carries its own weight** (gravity assist).
   Each command adds a gravity feed-forward torque per joint, from the arm's model (link masses and centres of mass
   from the URDF, corrected by measurement) and the held item's estimated mass, so the joints' PD springs only correct
   small errors and the arm sags far less at reach. It is capped per joint, ramps in after arming and drops to zero on
   any stop ([41H], [3dt]); a model error makes it imperfect, never unsafe. The share of the weight it actually
   cancels is measured on the real arm (sag at reach, loaded and unloaded) and is what the sim's
   `maker_arm.gravity_ff_fraction` uses ([x3f]); until then the sim keeps its estimate, marked as one.

## Decisions

1. [XkV] **The arm is driven over CAN from a host within USB reach.** Observed 2026-09-24 in maker-arm-sdk 0.1.0 (commit
   b30d05a, 2026-09-05): the follower arm's motors are on a CAN bus reached through a USB CAN adapter (SocketCAN on
   Linux, or SLCAN serial at a verified 25 Hz), and the leader arm is a USB serial device. The Go2 Air has no user
   compute, so the adapter runs on a small computer carried on the robot (CALM node `arm-host`: first a
   Raspberry Pi 3 Model B already on hand; a 2 GB Raspberry Pi 4 if it falls short), which also carries the wrist camera's USB feed onto the house Wi-Fi.

2. [bC0] **Safety and test patterns adapted from MOSS** (good ideas borrowed, with attribution). Criteria
   [41H], [3dt], [JYS] and [wcJ] follow MOSS (github.com/metrox-eth/moss at b2c887a, read 2026-09-25; code Apache-2.0, docs CC BY 4.0), a
   3D-printed litter-picking rover on dimOS: its host-side 300 ms deadman beside the firmware's own command watchdog,
   its disarmed boot with an explicit arm step (any `ERR` stops and disarms; a reopened port sends `stop` first), its
   25 % commissioning limit on the motors, and its cold tests against a fake serial port replaying a recorded bench
   session (`moss_pi/serial_bridge.py`, `tests/fake_serial.py`, `firmware/`). The patterns are re-implemented, not
   copied; any code copied from MOSS keeps its Apache-2.0 notice. The Maker Arm's motors do have a command timeout of
   their own: maker-arm-sdk sets each motor's CAN watchdog to 200 ms and refuses to run with it disabled
   (`maker_arm/config.py`, `profiles/maker_arm_v1.yaml`; observed 2026-09-25 at commit b30d05a). That is the arm-side
   half of [41H]; still to verify on the real arm (November) that a motor actually stops when commands cease.
3. [p2F] **Every motor reports its torque, the gripper included** (motor 7, an RS00 in MIT mode whose grip force is its
   stiffness times the position error), and the SDK has stall detection (`maker_arm/stall.py`: position error over
   0.12 rad, speed under 0.04 rad/s and torque over 0.5 Nm for 0.5 s). Observed 2026-09-25 at b30d05a. The pick routine
   can use these to sense a grip, early contact, the held weight and a drop; DimOS's sim adapter reports the gripper's
   effort as zero today.

4. [bF1] **The SDK speaks motor angles; DimOS plans in the URDF's.** Observed 2026-09-29 at b30d05a: the SDK's joint
   positions are the motors' own (offset 0 in `profiles/maker_arm_v1.yaml`; joint 3 spans 3.88 to 7.96 rad) while the
   URDF's joint 3 spans 0 to 3.14, and the URDF's README leaves "physical zero alignment" unvalidated. Hard-won: an
   adapter that passes the SDK's angles straight through plans fine in simulation (where the two agree) and sends the
   real arm somewhere else. The adapter takes a per-joint zero and sign (`urdf = sign * (motor - zero)`), identity until
   measured: on the real arm, read the motors with the arm in its factory fold (the URDF's zero), checked against a
   second known pose. The fold rests on stops: DimOS's planner rejects a start on or past a limit, so its home is the
   fold backed 0.05 rad off joints 2 and 3 (the SDK profile's own backoff from the stops).

5. [Y2p] **The gravity feed-forward lives in the DimOS Maker Arm adapter** ([cqG]), upstream-shaped, so it is the same
   code on the arm host and in sim ([KXa]): `dimos/hardware/manipulators/maker_arm/gravity.py` (numpy only, no other
   DimOS imports) on the fork's branch `nick/feat/maker-arm-driver` (worktree `../dimos-maker-arm-driver`,
   uncommitted as of 2026-10-03). It reads link masses and centres of mass from the SDK's URDF (fingers and brackets
   lumped into their link), holds the held item as a point mass at `grasp_center`, and works in URDF angles; the
   adapter turns it into motor terms by each joint's sign ([bF1]). Chosen over the SDK (not ours) and the arm host
   (not upstream). Forced: maker-arm-sdk at b30d05a sends tau 0 and takes no torque (`arm.py:316`), so the adapter
   puts the torque on the position target as tau / kp, which the motor's MIT law turns back into exactly that torque.
   RobStride's MIT frame is at the output shaft (the SDK maps torque by direction only), so no gear ratio enters. Two
   costs of that route: the SDK clamps targets to the joint envelope, so near a stop the feed-forward is cut short;
   and on a stop the SDK holds the measured pose with no offset, which is the zero-on-stop [cqG] asks for.
   - Capped at each motor's rated torque: RS02 6 N m (SDK `protocol.py`, b30d05a), RS00 5 N m (RobStride's RS00
     listing, 14 peak, 10:1, observed 2026-10-03). Ramped in over 2 s after the arm step; zero on stop, fault,
     disable or disconnect; re-armed by enabling or clearing a stop. Scale knob 0 to 1, default 0.25 ([JYS], MOSS's
     25 %); `gravity_mass_scale` takes the measured correction. `set_payload` and `set_base_gravity` (the base's tilt;
     upright by default) are extras beyond the adapter protocol.
   - Checked 2026-10-03: on the SDK URDF the torques match MuJoCo's bias forces (`maker_arm.xml`, gravity along the
     base's -y) to 1e-6 N m. A static solve at floor reach (arm base 0.22 m up, jaws 2 cm above the floor 0.45 m out,
     0.3 kg held, SDK kp, 3.7 kg arm; no flex or lash, the pose on joint 2's and 3's stops): PD alone sags 0.057 rad
     at the shoulder, 4.6 cm at the jaws; with the feed-forward 0.019 rad, 0.95 cm (the shoulder's 8.9 N m need is
     capped at its 6 rated); the sim's planned perturbed model (0.8 share, link masses sd 15 %, centres of mass sd
     1 cm, item under-estimated 25 %) 0.018 rad, 1.4 cm.

6. [qaS] **The held item's mass is the pick's own weighing, set once weighed and cleared on every way out** ([cqG]).
   After the lift the pick routine weighs the item from the rise in its joint torques ([XCl]); the feed-forward's
   payload is that mass times `1 + maker_arm.payload_estimate_error` (an estimate: the weighing's error, taken
   short, so a wrong estimate errs to sag rather than lift), held while the item is put back or carried to the bin,
   and set to zero in a `finally` around that path, so it is zero once the item is released and after any failure
   or error. Hard-won: a payload left set after a drop or a failed carry pushes the empty arm upward, and the next
   pick weighs against a baseline that carries it.
   - The pick drives the robot through `set_payload`. The standalone sim's sets the sim arm's feed-forward
     directly. DimosRobot's writes the mass to a small `/dev/shm` file named per launch (`HK_SIM_PAYLOAD`, made in
     `sim/dimos/env.py` beside the clock's), which the DimOS sim's per-frame viewer hook (`sim_hook.viewer`, in the
     process that steps the physics) reads when it changes, before updating the feed-forward. With no such file
     (the real robot) it goes to the adapter's own `set_payload`.
   - The sims turn gravity into the arm base's frame by the simulated trunk's attitude as it is. The real robot uses
     its IMU's attitude estimate instead (`set_base_gravity`), error and all.

7. [Mvl] **The host's stop fires on a stale target, not on a quiet arm** ([41H]). Hard-won: DimOS's coordinator
   writes to an arm only while a task commands, so an arm holding still hears nothing for minutes, and a stop on
   silence alone would latch it after every move. The adapter's watchdog thread (every 50 ms, from the arm step until
   disconnect) stops the arm, holding the measured pose and latched like `write_stop` and reported as a command
   timeout, when no command has come for `command_timeout_s` (0.3 s, never off) while the SDK's rate-limited command
   has not reached the last target (beyond 1 mrad plus the gravity feed-forward's offset, which ramps without a
   command; the target taken inside the envelope the SDK clamps to). A single far target is therefore stopped 0.3 s
   in: DimOS's trajectories stream their points. The arm-side half is the motors' own CAN timeout, which the SDK sets
   at enable; the adapter refuses a profile that turns it off. A reopened link's first frame to each motor is a stop
   because the SDK's connect probes each motor with one (`Motor.probe`, b30d05a, observed 2026-10-18); the fake-link
   test holds that, so an SDK change that breaks it fails there.

8. [g5J] **Commissioning limits are shares of the profile's, applied inside the SDK's own loop** ([JYS]).
   `speed_scale` scales the profile's `max_velocity` (the SDK's rate limiter, which every target walks through: 5.0
   rad/s to 1.25 at 0.25), reported to planners as `get_limits().velocity_max`; `grip_force_scale` scales the gripper
   motor's kp (grip force is kp times position error, [p2F]); both default to 0.25 beside `gravity_scale` ([Y2p]), as
   MOSS commissions its motors at 25 %. Chosen over refusing fast targets in the adapter: the rate limiter acts on
   every frame the SDK sends, its own approach after enabling included, so nothing gets past it. The shares the arm
   host will pass are `maker_arm.commissioning` in `params/robot.yaml`; one above 0.25 is the latest row for it in
   `real/calibration/arm_limits_log.csv` (date, share, from, to, measurement), held by `tests/test_arm_commissioning.py`.

9. [hrT] **The fake link is at the CAN frame level, under the real SDK** ([wcJ]). maker_arm's transports only move
   frames (`CanBackend`: open, close, send, a receive callback; b30d05a), so the fork's `maker_arm/fake_link.py` gives
   three: `SimulatedMotors` (RobStride motors as the SDK sees them: feedback on every frame, parameter read-back,
   targets reached at once, each motor's own CAN timeout, a pulled cable, fault bits), `RecordingLink` (wraps the real
   link on the bench and saves every frame each way as JSON lines) and `ReplayLink` (answers each frame with the
   replies that followed the same motor's same kind of frame in the recording, in order within each enabled spell;
   not by time, since a replay's count of 200 Hz control frames differs from the bench's while their order does
   not; past the recording's end, silence, as on a pulled cable). The tests run the SDK's own `Arm` and the adapter
   on them through the adapter's `can_link`, so an SDK update is under test too; they need the SDK importable and
   skip without it. Until the bench day the recording replayed is synthetic: the simulated motors, recorded.

## Tests

| Criterion | Test | Status |
|---|---|---|
| [Odx] (listed, same interface) | DimOS fork, branch `nick/feat/maker-arm-driver` (410ac5294, 449ef80f6): `dimos/hardware/manipulators/maker_arm/` (a `ManipulatorAdapter`, registered as `maker_arm`) and its `test_adapter.py` against a fake SDK; `dimos/robot/manipulators/maker_arm/` (hardware, planning model, blueprints `coordinator-maker-arm` and `maker-arm-planner-coordinator`) and its `test_config.py`; DimOS's own registry and blueprint checks | Partial: all pass (28, including the joint mapping [bF1]; ruff and mypy strict clean); not on hardware |
| [0h9] (works on the real arm) | bench test with the real arm | Not yet written: arm arrives November |
| [KXa] (same code in sim) | `test_config.py` (the scene generated from the SDK's URDF stands the arm up, holds home, moves the jaws together); the planner blueprint run with `--simulation mujoco` and driven through DimOS's arm SDK (joint move, 5 cm Cartesian drop, gripper, home) | Partial: the scene tests pass; the driven run passed by hand on 2026-09-29 (the joints settle within 0.04 rad of target) and is not yet an automated test |
| [lSw] (faults stop the arm) | fault-injection test with the mock; fake-link tests on the real SDK ([hrT]) | Partial: a motor fault is reported and refuses commands until cleared (`test_a_fault_is_reported_and_cleared`); 18 Oct 2026, on the SDK at b30d05a over simulated motors (`test_fake_link.py`): a pulled cable mid-move stops the arm (the SDK's command frozen, the feed-forward zeroed, commands refused, the motors' own timeouts out) and reports the feedback timeout, and an unplugged adapter reports the control loop's error (`test_a_pulled_cable_stops_the_arm_and_says_why`, `test_an_unplugged_adapter_stops_the_arm_and_says_why`); not on hardware |
| [uiv] (upstream PR) | PR opened | Not yet written |
| [41H] (two independent stops) | fake-link test: commands stop, the arm stops within 300 ms; bench test with the host killed | Partial (18 Oct 2026): the host's stop ([Mvl]) by hand-driven clock (`test_the_host_stops_the_arm_when_commands_stop_mid_move`, `test_a_quiet_arm_holding_still_is_not_stopped`), on its own thread (`test_the_host_stop_runs_by_itself`) and on the real SDK over simulated motors, one far target then silence, held within 0.4 s and not walking on (`test_commands_stop_mid_move_and_the_arm_stops`); the arm side: a profile with the motors' timeout off is refused (`test_the_motors_own_timeout_must_be_on`), enabling writes 200 ms to every motor (`test_enabling_sets_each_motors_own_timeout`), and the simulated motors stop when the host dies (`test_when_the_host_dies_the_motors_stop_by_themselves`, a check of the fake, not the arm); not on hardware: the bench test with the host killed and a real motor's timeout (November) |
| [3dt] (starts disarmed) | fake-link tests: no motion before arm; error reply disarms; reconnect sends stop first | Partial: no motion before enabling, and a latching stop until cleared or re-enabled (`test_connects_disarmed_and_enables_only_when_asked`, `test_stop_latches_until_cleared_or_re_enabled`); 18 Oct 2026, on the real SDK over simulated motors: an error reply stops the arm and, cleared, it stays disarmed until enabled (`test_an_error_reply_stops_and_disarms`), and a reopened link's first frame to every motor is a stop and it comes up disarmed (`test_reopening_a_lost_link_sends_a_stop_first`); not on hardware |
| [JYS] (capped first runs) | the adapter's commissioning shares in the SDK's own loop ([g5J]); houndkeeper `tests/test_arm_commissioning.py` (each share the quarter or its latest logged, measured raise) | Partial (18 Oct 2026): speed and grip force start at a quarter in the SDK's rate limiter and the gripper's stiffness, planners see the capped speed, raises take effect and bad shares are refused (`test_first_runs_are_capped`); the shares in `params/robot.yaml` match the empty raise log, and a raise without its row or measurement is caught (2 pass); not on hardware: no raise measured yet (November) |
| [wcJ] (tests without the arm) | adapter tests replaying the recorded bench session ([hrT]) | Partial (18 Oct 2026): `test_fake_link.py`'s 8 tests run the adapter on the real SDK (b30d05a) over simulated motors, and a bench session (move, a motor's error reply, recover, move, a pulled cable) recorded and replayed through `ReplayLink` gives the adapter the same error reply and lost link (`test_a_recorded_session_replays_the_same`); they skip without the SDK installed; the recording replayed is synthetic: the real arm's, with a pulled cable and error replies, waits for the bench day (November) |
| [cqG] (gravity feed-forward) | DimOS fork, branch `nick/feat/maker-arm-driver` (uncommitted): `maker_arm/test_gravity.py` (two-link torques against the hand calculation and MuJoCo, URDF lumping, payload, cap, ramp, zero on stop, scale knob) and `test_adapter.py`'s feed-forward tests (sign and tau / kp through the motor conversion [bF1], rated caps, ramp after enable, zero on stop, fault and disable, payload); sim: houndkeeper `tests/test_arm_gravity_ff.py` (the sim's arm on the same `gravity.py`, loaded by file path, with the perturbed SDK-URDF model of `maker_arm.gravity_ff_model_error`: its feed-forward equals that model's capped torques, upright and tilted; the error is seeded and nonzero and the model carries `gravity_ff_fraction` of the true moving mass; static sag at floor reach in band; `gravity.py` imports numpy only) and `tests/test_arm_payload_ff.py` (the held item [qaS]: the weighed mass x (1 + `maker_arm.payload_estimate_error`) raises each joint's feed-forward by J^T m g at the grasp point, upright and tilted, and clearing it gives the unloaded torques back; the pick clears it when the carry ends released, failed or raising; DimosRobot's per-launch payload file round trip through the sim process's read); real: sag measured loaded and unloaded (`real/calibration/arm_gravity.md`) | Partial (3 Oct 2026): the unit tests pass (30, ruff and mypy strict clean); the sim now runs this code in place of MuJoCo gravcomp (4 static tests pass, mj_forward only: at floor reach, nothing held, the shoulder sags 0.010 rad and the jaws 0.9 cm, against 0.044 rad and 4.2 cm for PD alone); not yet seen in a stepped sim; 3 Oct 2026: the held item's weighed mass is on it in both sims ([qaS]; 5 static tests pass, mj_forward only, the payload's rise matching MuJoCo's own Jacobian within 2e-3 N m), not yet seen in a stepped pick; real not yet: arm arrives November |
