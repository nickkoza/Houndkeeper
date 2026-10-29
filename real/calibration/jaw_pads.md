# Measuring the gripper's squeeze and the pads' grip

Turns three estimates in `params/robot.yaml` into measurements ([4hm]): `maker_arm.grip_force_N`,
`maker_arm.jaw_friction_pad` and `maker_arm.jaw_friction_bare`. They decide whether a held item turns in the jaws
([GU5]). Once measured, update the file with `source: measured` and the date, and re-run the grasp benchmark ([7Xn]).

Household tools only: a kitchen scale (1 g), a 30 cm ruler or a flat wooden stick, a few coins or washers of known
weight, a small bag or clip to hang them from, and a phone camera. Arm on the bench, no robot needed.

## 1. Squeeze (grip_force_N)

1. Zero the kitchen scale, then hold it on its edge (it must be thinner than the jaws' opening, 99 mm with pads).
2. Put the open jaws around it: one jaw on the middle of the weighing platform, the other on the scale's underside.
3. Close the jaws with the same command the pick routine uses (`CLOSED`) and wait 2 s. Read the scale: grams x
   0.00981 = the squeeze in newtons.
4. Log the gripper motor's torque from the SDK (`get_joint_torques()`, motor 7) at the same moment. Newtons per unit
   of motor torque lets the arm estimate its squeeze on every pick from then on.

Repeat 5 times and record the median and the spread. Do it bare and with pads fitted (each pad takes 3 mm of travel).

## 2. Sliding friction (jaw_friction_*[0])

1. Grip the ruler flat between the jaws, pads on, arm holding still.
2. Hang weight from the ruler's end (straight down along the ruler's length, pulling it out of the jaws), adding
   coins until it slips. Weigh the coins and the bag on the kitchen scale.
3. Sliding coefficient = pull at slip / (2 x squeeze), since both jaws hold it.

## 3. Twist resistance (jaw_friction_*[1])

This is the number the sim can't know: how much twist a pad's contact patch resists.

1. Grip the ruler at its middle, flat faces against the pads, ruler horizontal, jaws closing horizontally.
2. Hang weight from one end at a measured distance d (m) from the jaws' centre. Add coins until the ruler starts to
   rotate in the jaws. Film it to catch the moment.
3. Twist torque at slip M = (coin weight in kg) x 9.81 x d. The ruler's own weight is balanced (gripped at its middle).
4. Torsional coefficient for the sim (m) = M / (2 x squeeze), per jaw. Compare with the estimate (0.004 m).

Repeat on a fabric surface (wrap the ruler in a sock) and on smooth plastic (tape over it): household items vary.

## Record

Append each session to `real/calibration/jaw_pads_log.csv`: date, pads (none / v1 / later), surface, squeeze N,
sliding coefficient, twist torque N m, torsional coefficient m, notes.
