# Measuring the head camera's aim

Turns an estimate in `params/robot.yaml` into a measurement ([4hm]): `go2_air.head_camera_pitch_deg`, how far the
Go2's front camera looks down from level. Nobody publishes it: DimOS mounts it level as a placeholder, and Unitree's Go2
model has no camera. It decides how much floor the robot sees and how far ahead, so the sim follows it. Once
measured, update the file with `source: measured` and the date, and let the baselines re-render.

Household tools only: a tape measure, masking tape, a flat floor, and a laptop on the robot's network to save one frame
from the head camera. No people in the frame.

## 1. A line of marks on the floor

1. Stand the robot in its normal walking pose (the height it walks at, not lying down) on a flat, even floor.
2. Straight ahead of the camera, tape small crosses on the floor at 0.5, 1.0, 1.5, 2.0 and 3.0 m, measured from the
   point on the floor directly below the camera lens (a plumb line or a set square from the lens).
3. Measure the lens's height above the floor (mm).

## 2. One frame

Save one head-camera frame at full resolution (1280 x 720) with the robot still. Note the pixel row of each cross's
centre.

## 3. The angle

The lens is a fisheye (equidistant): a point's angle below the optical axis is (row - cy) / fy, in radians, with
cy = 349.3 and fy = 796.5 from DimOS's calibration (`front_camera_720.yaml`). A cross at distance d on the floor is
atan(h / d) below level, with h the lens height. So for each cross:

    pitch = atan(h / d) - (row - cy) / fy        (radians; positive = looking down)

The five should agree within about half a degree. Record their mean in degrees, and the lens height as
`go2_air.head_camera_height_m` (the sim's walking height, 29 cm, is an estimate).

## 4. Where the lens sits (`go2_air.head_camera_pos_m`)

The sim renders from the trunk's origin plus (0.29, 0, 0.01) m (forward, left, up); the real lens is unmeasured.
With the robot in the same standing pose as step 1:

1. The trunk's origin is the centre of the body between the four hip motors (Unitree's model; the hip joints sit at
   +-0.1934 m forward/back and +-0.0465 m to the sides of it). Mark the floor below it: the midpoint of the four
   hip-motor centres, plumbed down.
2. Measure from that mark to the floor point below the lens (step 1.2): forward and sideways (mm).
3. The lens's height (step 1.3) less the trunk origin's height (the hip motors' centres above the floor) is its
   height above the origin.

Write the three as `head_camera_pos_m` (m, forward, left, up), and the lens height as `head_camera_height_m`.

## 5. The mount's small error, from a night (`go2_air.head_camera_mount_error`)

The pitch from steps 1-3 is the camera's aim to about half a degree. The night pipeline then calibrates the mount
from the robot's own frames (`sim/house/calibrate_mount.py`, [KkQ]): far views of the floor agreeing with near ones
(pitch), views from opposite directions agreeing (yaw, roll). Run it on the first real night's recording:

    calibrate_mount.py <real memory.db> --house <its house_lidar.npy> --out real/local/head_mount_real.json

Its pitch should agree with step 3's within half a degree; if not, trust the marks and look for the cause (a flexing
head, the pose not the walking one). The yaw and roll it finds, and the pitch less step 3's, are the mount's error:
write them as `head_camera_mount_error` (source measured). The sim keeps rendering a fixed error of that size so the
pipeline never assumes zero.

### Whether it stays put between nights ([GWP])

A head knocked or a bracket loosened between the mapping nights and tonight shifts tonight's floor against the map:
half a degree of pitch took the launch false alarms from 11.2 to 19.0 a night in the sim (docs/research/
sensitivity-2026-10-13.md). The night pipeline's nightly check ([9tQ], `floor_selfcal.py check`) solves the mount
free from a night's views and warns when it disagrees with the robot's calibration by over 0.1 deg of pitch, 0.3 deg
of yaw or 5 mm of height. On the first two real nights, run it on each night's views alone, then side by side:

    cd sim/house
    $PY floor_selfcal.py check <run>/views_flat_n1.npz --out ../../real/local/mount_check_n1.json
    $PY floor_selfcal.py check <run>/views_flat_n2.npz --out ../../real/local/mount_check_n2.json
    $PY ../../real/calibration/measure.py mount ../../real/local/mount_check_n1.json ../../real/local/mount_check_n2.json

(`<run>` is the real nights' run directory, where the baseline stitch left both nights' views.) `measure.py mount`
says pass when each night is within those bands of the calibration and night 2 within them of night 1, outside
otherwise and which way it moved, and not judged when a night had too little floor (under 2,000 tile registrations:
walk more of the open floor). Later baseline stitches' `calibration_check.json` can be added to the list in night
order. One night's views alone have not yet been checked against a sim house's (the tuning houses' checks are of
both nights together); a not-judged night on the robot is the first thing to watch for. Outside: README.md, day one 1.

