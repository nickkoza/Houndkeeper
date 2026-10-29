# Measuring how much of the arm's weight the gravity feed-forward cancels

Turns `maker_arm.gravity_ff_fraction` in `params/robot.yaml` from an estimate into a measurement ([4hm], [cqG]): the
share of the arm's weight the adapter's gravity feed-forward actually carries (decision [Y2p] in the Maker Arm spec [oIS]). It also gives the adapter's `gravity_mass_scale`. Once measured, update the file
with `source: measured`, the date and how, and re-run the grasp benchmark and a night ([7Xn]).

Arm on the bench, base clamped level, the stops tested first ([41H], [3dt]). A tape measure or a ruler fixed upright
beside the jaws, a phone camera, a kitchen scale, a 0.3 kg weight that hangs from the jaws (a bag of coins).

## 1. The pose

Floor reach: the jaws out at their longest reach and low, as for a floor pick (the adapter's command, not by hand).
Mark where `grasp_center` sits on the upright ruler: this is the target.

## 2. Sag without and with the feed-forward

1. `gravity_scale` 0 (PD alone): command the pose, wait 3 s, read every joint's angle (`read_joint_positions`), its
   torque (`read_joint_efforts`) and the jaws' height on the ruler (photo). Sag = target minus reading, per joint.
2. `gravity_scale` 0.25 (the commissioning default, [JYS]): the same. The sag must fall; if it grows, a joint's sign
   is wrong ([bF1]): stop and fix it before going on.
3. Raise to 0.5, then 1.0, only if each step's sag fell as expected; record each.
4. Repeat 1-3 with the 0.3 kg weight hung at the jaws and `set_payload(0.3)`.

## 3. The share

Per joint, the torque the PD carries is kp x sag. Share cancelled = 1 - (kp x sag with the feed-forward) /
(kp x sag without it), at scale 1.0. Take the shoulder (joint 2) as the figure for `gravity_ff_fraction`, the
others alongside. Where the shoulder's need is over its 6 N m cap the share is the cap's, so also do it at a
pose nearer the body. `gravity_mass_scale` = 1 / the unloaded share, if it is under 0.9 or over 1.1.

## Record

Append each session to `real/calibration/arm_gravity_log.csv`: date, pose, scale, payload kg, per-joint sag rad,
per-joint torque N m, jaws' drop mm, share, notes. A raise of `gravity_scale` is also a row in
`real/calibration/arm_limits_log.csv`, with the measurement that supported it, and the new share in
`maker_arm.commissioning` ([JYS]).
