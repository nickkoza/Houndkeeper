# [nAH] — Where the sim is kinder than the world: the imperfection register

**Status:** Proposed · **Date:** 2026-09-27 · **Source:** a pass over the sim for every other place it
models the world as perfect: an audit of the DimOS fork's sim and houndkeeper's sims, 27 Sep 2026 · **Serves:** [x3f]
(principle), [33f], [FHA] (floor map), [d8d]/[QDc] (the funnel), [BPL] (tip safety) · **ID:** [nAH]

## Context

The live sim (DimOS's MuJoCo process plus houndkeeper's hook) had no imperfection anywhere on 27 Sep: odometry was
the true pose, cameras were perfect pinholes delivering clean frames stamped to the millisecond, the lidar was a
dense exact depth image, and the arm held every commanded angle on a rigid frame. Only the offline mapping bench
could add pose noise and drift. Every number measured on the live sim so far (the floor map's 3.3-3.7 mm, the
detection bench's 98%, the standing look's balance) is therefore a best case. Each entry below says what is ideal,
the real imperfection's size (sourced, or marked est.), what it threatens, and its status.

## Decisions

1. [z7P] **The arm is imperfect: sag, printed-frame flex, backlash** (params/robot.yaml `maker_arm.*`,
   `go1_arm.add_imperfections`). Ideal before: perfect gravity compensation ([Oav]) and rigid links. Real: the SDK
   sends plain PD with no feed-forward (maker-arm-sdk b30d05a, arm.py:316), the frame is 3D printed, gearbox
   backlash unpublished. Modelled from 27 Sep: 80% gravity feed-forward (the controller houndkeeper will send, from
   a mass model taken 20% short), spring hinges at the mount and the elbow (est.; four places along the arm had modes
   too fast for the 4-5 ms step), 0.3 deg free play after every encoder (est.; the wrist's two as a spring, hard stops
   there were unstable). Measured: the wrist camera sits 1.0-2.2 deg and ~8 mm from where the encoders put it
   (stowed, the lifted sweep pose, reaching out): ~3 cm on the floor at the lifted pose. Walking, the flexing arm
   was numerically unstable at the sims' 4-5 ms physics step (the robot fell stepping off the carpet; at a quarter
   step it walked cleanly): every sim now runs physics at 1.25 ms (params `sim.physics_dt_s`, the policy still acting
   every 20 ms; `go1_arm.step`), the motors' damping is joint damping (taken implicitly), and each flex joint's mode
   is held at or below ~10 Hz (its stiffness, so its droop, kept). Tested: `tests/test_arm_imperfect.py`. The kinematic copy stays nominal: the robot believes the rigid arm. Threatens:
   the wrist camera's floor map and flag positions, the grasp's aim. Measure in October.
2. [2Ak] **Odometry is the true pose.** Real: the Go2's leg-and-IMU estimate drifts (size unknown on the Air; est.
   1-3% of distance, ~1-2 deg heading a minute) and jumps at slips; DimOS's lidar SLAM may correct it (unverified).
   Threatens: everything that lays frames on the floor map ([FHA]) and every revisit. Modelled from 27 Sep (the
   fork's `odom_drift.py`, params `go2_air.odom_drift`, on by default, HK_ODOM_DRIFT=0 for comparison): dead
   reckoning with a per-night scale error, noise growing with distance and time, and an uncertain start; the lidar's
   points are placed with the same estimate. Est. size: ~10-15 cm after a 16 m loop, 1-6% of a straight walk.
   Consequence: each night's floor pictures sit in a different drifted frame, so the floor comparison needs the
   frames placed against the house map, not by odometry: DimOS's lidar relocalization (one fix against a prior map;
   `dimos/mapping/relocalization`) plus visual frame-to-map registration (parked on 27 Sep) at the mm level.
3. [kJ6] **The camera image is perfect.** Real Go2 front camera: an equidistant fisheye (DimOS's calibration,
   front_camera_720.yaml), delivered over WebRTC as compressed video (codec artefacts, bitrate drops), with sensor
   noise, motion blur at walking speed, rolling shutter, and auto-exposure that lags. The sim has Filament's
   auto-exposure only. Threatens: the floor comparison's colour thresholds (tc 0.045 is fine-grained), edge
   placement. Modelled from 27 Sep (`camera_imperfect.py`, params `go2_air.camera_imperfections`, on by default,
   HK_CAMERA_IMPERFECT=0 for comparison): blur from the camera's own motion over its exposure (4 ms by day, up to
   33 ms at night before gain rises: a 30 deg/s turn smears 6 px at night), a 20 ms rolling shutter, shot and read
   noise grown by gain, JPEG q75 standing in for H.264; all est. Costs ~15 ms a frame. The fisheye (27 Sep,
   `lens.py`, HK_FISHEYE=0 for comparison): DimOS's calibration of a Go2's front camera (equidistant, 4 coefficients)
   is what the sim's DimOS publishes as camera_info; the head camera renders a 960 x 540 pinhole wide enough to cover
   it (81 deg vertical), warped to the 640 x 360 fisheye, blur before the lens and noise after. Everything that places
   head-camera pixels takes the lens from `lens.head` / the recording's camera_info and straightens frames first; the
   live code had taken it from the kinematic copy's stock 45 deg while frames were 51.8 (floor points 13% out, ~26 cm
   at 2 m, fixed and tested the same day). Not yet: bitrate drops, exposure flicker; the real stream is 1280 x 720
   (the sim's 640 x 360 is coarser). 27-28 Sep: the auto-exposure could reach 12 stops over daylight (gain to 250x);
   now 8 (exposure to 33 ms, then gain to 16x, est. for a phone-class sensor), and a camera swung from the dark under
   the body waits for its exposure to settle before a still is used (`DimosRobot._settled_image`). The noise it
   leaves on dark carpet is answered by a colour threshold that rises with the frame's own measured noise
   (confirm_stream `noise_k`, sim-slice [WFI]).
   28 Sep audit: the noise was the raw sensor's, added after rendering with nothing to reduce it; a real camera
   denoises before it encodes. Now 30% of it survives, as blotches (`isp_noise_keep`, `isp_noise_blur_px`, est.;
   `tests/test_camera_imperfect.py`). The gain itself was fair: see [72t].
   28 Sep audit: the lit regime had one ceiling light for the 165 m2 open plan, so a standing look ran at the camera's
   16x gain ceiling and was still underexposed (frames averaging 29-39 of 255). Now a fixture over each of the plan's
   five parts (`make_nights.LIT_CEILING`), set by measuring the robot's cameras at 48 spots of house 70 on Filament
   1.77.1: a standing look at the carpet 4.9x gain (median; 15% at the ceiling, the dark corners and under furniture),
   a head view 2.4x, both exposed; a room of ~70 lux (a DOE field study of TV rooms: 75th percentile 36-50 lux,
   87.5th 70-108): a lit room. Day and the unlit nights are unchanged.
4. [55b] **The cameras sit exactly where the model says.** Head camera pitch is a placeholder (0 deg, unmeasured) and
   its mount unmeasured; the wrist camera's placement and FOV are estimates. Threatens: floor projection (a 0.5 deg
   pitch error moves a point 2 m out by ~3 cm). Modelled from 27 Sep (params `go2_air.head_camera_mount_error`,
   HK_MOUNT_ERROR=0 for comparison): the sim's camera sits 0.6 deg lower, 0.4 deg turned, 0.3 deg rolled and a few mm
   off the mount DimOS publishes, fixed as one robot's miscalibration is (est.). Calibration
   (real/calibration/head_camera.md) removes it on the robot; the wrist camera's placement error is [z7P]'s.
5. [UMH] **Frames and poses are stamped exactly.** Fixed on 27 Sep to the sim's own state time ([lTq] cause 6). Real:
   WebRTC video arrives 50-200 ms late with jitter (est.), stamped at receipt unless corrected. Threatens: every
   frame's pose while turning. Modelled from 27 Sep (params `go2_air.frame_delay_s`, the fork's
   DIMOS_SIM_FRAME_DELAY, HK_FRAME_DELAY=0 for comparison): every head and wrist frame stamped 80 ms late, 20 ms
   jitter (est.); the offset is found from each recording's turns (`calibrate_lag.py`), as it would be on the robot. Forced (DimOS at a7492431f,
   observed 27 Sep): the Go2 connection stamps every video frame and every odometry message with the host's clock on
   arrival (`time_is_now`), and drops the frame's own presentation timestamp (`pts`, set by the robot's encoder), so
   on the real robot a frame's pose is only as good as two arrival delays. The fix, planned: stamp frames from `pts`
   (exact spacing, no network jitter) and calibrate its fixed offset to the odometry by correlating the image's
   rotation with the odometry's turn rate, kept up during the night (a fork change to offer upstream). Whether
   Unitree's state messages carry a robot-side time is unknown. The sim's odometry arrives late too from 28 Sep (the fork's `odom_delay.py`, params `go2_air.odom_delay_s`: 30 ms mean, 10 ms jitter, est.; in order; stamped on arrival as the connection stamps it, or at source once that fix lands; HK_ODOM_DELAY=1, off until round 10's comparison is done, then on by default). The stream over Wi-Fi (the
   fork's `stream_model.py`, params `go2_air.video_stream`, HK_STREAM=0 for comparison): 3% of frames lost, twice a
   minute a ~0.6 s stall, and twice a minute ~3 s of hard compression (JPEG q30) as the bitrate drops (est.).
   28 Sep audit: frames 150 ms late (30 ms jitter), was 80/20: Unitree states 180 ms for the Go2 Ent's stream, and
   WebRTC is generally 100-300 ms (params `go2_air.frame_delay_s`).
6. [JaG] **The lidar is dense and exact.** Real Go2 L1: ~21,600 points a second over a hemisphere, noisy, with a blind
   zone near the body. Threatens: the costmap, the height map the confirmer's lines of sight and structure band use,
   the toy-vs-structure decision. Modelled from 27 Sep (the fork's `lidar_model.py`, params `go2_air.lidar`,
   HK_LIDAR_MODEL=0 for comparison): 21,600 points a second drawn from the depth cameras evenly over solid angle,
   ~2 cm range noise, nothing nearer than 5 cm (Unitree's figures); returns go missing on dark surfaces (up to 80% on
   black) and on glass, screens and mirrors (70%), by each surface's albedo and material (est., params
   `go2_air.lidar_dropouts`). Not yet: the L1's own scan pattern and its exact hemisphere (the sim's three depth
   cameras cover a band). 27 Sep: the body's pitch and roll while walking lift the floor's far returns ~3 cm, so
   heights are taken from each scan's own fitted floor (`localize.level`); and the sim harness's injected house map
   rides the same lidar stream as one ~24,000-point message, which matchers must not take for a sweep
   (`localize.real_scan`). Lidar placement measured at ~10 cm ([KkQ]): the centimetres come from the floor ([fXD]).
   28 Sep audit: dark-surface dropout only below 5% albedo, half the returns (was 10%, 80%): the L1 manual gives
   10 m range at 10% reflectivity, well past a room (params `go2_air.lidar_dropouts`).
7. [TF8] **The floor is a flat, rigid plane.** Real: carpet stands above the hard floor, rugs have thickness and move,
   thresholds rise at doorways. Threatens: every floor point's projection (the pipeline assumes one plane: seen from
   the head camera's 30 cm, a surface 1 cm up at 2 m is placed ~7 cm off), the carpet-edge precision itself, walking.
   Modelled from 27 Sep (`make_nights.py`, on with photographed floors, `--flat`/RELIEF=0 for comparison): the
   carpeted areas are a slab 12 mm above the hard floor and the rug an 8 mm slab on them (est.), both textured with the
   floor's own image by position (no second texture on the GPU), and the rug shifted each night (sd 2 cm, 1.5 deg,
   est.; rugs do move a little) until it was set still as asking too much (1 Oct 2026,
   [yyJ]). Not yet: doorway thresholds, curled rug corners, carpet the
   feet sink into.
8. [Ghm] **The body is a Go1 walking on a policy trained for it** ([WD6]). Real: a Go2 Air, 15 kg, Unitree's own
   controller, plus 3.7 kg of arm. Threatens: balance numbers, gait sway the cameras see. Plan: unchanged; measure
   on the Air.
9. [Ndp] **Toys are rigid and arrive exactly as placed.** Real: socks and plush deform, toys roll, the robot kicks
   them (seen once in sim). Threatens: grasp, re-finding. Plan: deformable stand-ins for soft items (backlog).
10. [1CB] **Night light is fixed for a night.** Real: headlights sweep through windows, a TV or phone lights up, a
    lamp is switched. Threatens: the floor comparison's "gone"/"present" colour cues. Plan: lighting events during
    a night (backlog; the bench already varies the lamp between nights).
11. [TJu] **Only the toys change.** Real: chairs move, bags and shoes appear, rugs shift; people and pets are out of v1.
    Threatens: false stops, the baseline. Chairs first: from 27 Sep every night, baselines
    included, has six dining chairs, a stool and a kid's chair each tucked in, pulled out 5-25 cm and turned, or moved
    0.5-2 m (`movables.py`, which the PoC had used; `night_tidy.sh` now passes `--movables`, MOVABLES=0 for
    comparison). The chairs are box primitives, stand-ins under [CaM] until scanned chairs replace them. Their thin
    legs sit below the live lidar's 0.35 m "tall" cut, so the floor comparison meets them and their shadows. Later:
    bags and shoes (some of which are to be picked up), rugs shifting (now [TF8]; the rug-in-baseline split is
    [yyJ]). A pet's food and water bowls: two scanned dog bowls by the
    kitchen every night, baselines included, nudged a few cm (est.), never in the pick pool (`items.FIXTURES`), toys
    placed clear of them (PETS=0 for comparison). Not yet: scoring a binned bowl as its own failure.
12. [72t] **The house is lit for testing.** Real nights may be near dark: the camera at full gain, or the robot's
    headlight lighting the floor differently every frame. Testing is with lights on at first; a dark house and a
    headlight are for later, and a real evening's footage in the house settles how dark it is. From 27 Sep test nights
    are `--regime lit` (`night_tidy.sh`'s default): the ceiling light at 0.9 and the lamp on, the same every night
    (light is a lever easily added in real life); dim nights had the camera at high gain, visibly grainy, and a
    lamp on one baseline night and off the other. REGIME=night keeps the dim, varied nights for later.
13. [ipj] **The arm was drawn light grey.** Forced: the Maker Arm's structure is black 3D-printed plastic (MakerMods'
    product page, "its black 3D-printed structure and RobStride actuators", observed 28 Sep 2026: mostly black
    with some silver hardware). The URDF carries no colours, so the sim drew MuJoCo's default grey. Threatens: the
    wrist camera sees the jaws in every look, and pale jaws stand out from dark floors where black ones would not; the
    detector and the robot mask were only ever shown grey jaws. From round 12 the arm is black and matte
    (`go1_arm.paint_arm`, `HK_ARM_BLACK`; albedo and sheen est.), and from 29 Sep in every sim by default, the jaw pads
    too (they had still drawn grey, and the benches had run grey). The head camera doesn't see the stowed arm (0 pixels
    changed), so the baseline pictures are unaffected. Not modelled: the silver hardware (the URDF's parts are unnamed,
    so the actuators' fittings and screws aren't told apart; they stay black).

14. [hkC] **The 28 Sep realism audit** (whether the realism had been overdone). Each estimate set against the
    best published anchor, in `params/robot.yaml` with its source. Changed: the lit regime's lighting ([72t]), the
    camera's own noise reduction ([kJ6]), video delay 80 to 150 ms ([UMH]), walking power 150 to 170 W (a measured 1.4 h
    patrol), lidar dark dropout ([JaG]); the battery 236.8 Wh (Unitree) and the RobStride torque constants (datasheets:
    RS00 1.48, RS01/02 1.22 Nm/A) were already right. Kept, to measure on the real robot: backlash 0.3 deg (RobStride:
    0.17, as a worn unit's margin), the printed frame's 1500 Nm/rad at the base (a printed PLA transmission measured
    319; possibly kind), odometry yaw-rate bias 0.05 deg/s (a standing robot showed 0.008; walking unmeasured). Open:
    the head camera's field of view (Unitree states 120 deg, axis unstated, against DimOS's 51.8 deg vertical), the
    furniture drawn as plain boxes, and one real frame under evening light to settle the noise.


15. [7cU] **The head camera's frames are 640 x 360.** Real: the Go2's front camera delivers 1280 x 720 (DimOS's
    calibration, `front_camera_720.yaml`); the sim renders half that across, so an 8 mm brick covers 1 to 4 pixels where
    the robot will see 2 to 8, and the floor comparison cannot resolve it (detection-baselines [H9w]): harsher than the
    robot, which [x3f] forbids as much as kinder. From 5 Oct 2026 the real size is available, opt-in: `HK_CAMERA_PX=real`
    (sim/dimos/env.py) renders and publishes params `go2_air.head_camera_px` (1280 x 720) through DimOS's frame-size
    option (`DIMOS_SIM_VIDEO_SIZE`, a general fork feature; the lidar's depth cameras keep 640 x 360, so the point cloud
    is unchanged), the fisheye made from a 1920 x 1080 render (1.5 times the frame, as at 640), the camera's
    imperfections applied at the frame's own size. The calibration, camera_info, the rectifier and the floor sampling
    scale with the frame (at 1280 x 720 the calibration is used as measured, unscaled), and so does every pixel figure
    set on 640 x 360 looks (the wrist camera's too, which the same option sizes): the morning log's crop of a find, the
    close-up study's cut and its outline of the gripper, a detection's foot laid on the floor; the frame lag and the mount do
    not depend on it. Unset, the sim renders DimOS's 640 x 360 and a night's render stamp is exactly as before; a
    different size enters the stamp (`sim_hook.render_settings`), so a baseline at one size is never compared with a
    night at another. The default stayed 640 x 360 until the cost and the gain were measured and the change decided
    (5 Oct 2026: first switched on by default, which re-swept shared baselines; made opt-in the same day). From 12 Oct
    2026 the real size is the default (detection-baselines [bFx]); `HK_CAMERA_PX=640x360` is the opt-in for quick loops. Cost and the
    first results: detection-baselines [80w]'s Tests row.

The order of work follows what each entry threatens at the top of the funnel: the arm (1) first, being measured,
then odometry (2), the image (3), the mounts (4) and timing (5), each switched on by default once built, and the
top-of-funnel requirements re-measured with all of them on. A requirement that no longer holds is reported, not
relaxed. [Oav]'s decision to compensate gravity perfectly is superseded by (1).
