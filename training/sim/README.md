# Sim training pipeline ([Pz2])

1. `sim/arm/record_demos.py --episodes N --seed S [--headless] [--out DIR]` — privileged-teacher demonstrations: the
   scripted grasp, told the item's true mask; records wrist images (128x96), 7 joint states, 7 joint targets at 10 Hz
   from the look-closer handover to the lift. Items from the library, generated floors.
2. `training/sim/make_dataset.py <demos> <dataset>` — LeRobot v3 dataset through DimOS's own dataset writer
   (gripper-frame actions were tried and were not better).
3. `training/.venv/bin/lerobot-train --policy.type=act --dataset.root=<dataset> --dataset.video_backend=pyav ...`
   (LeRobot 0.6.0, the version DimOS's policy runtime uses; `training/.venv` is separate from DimOS's).
4. `sim/arm/eval_policy.py --checkpoint <ckpt> --which policy,scripted --episodes 30 --seed 900 --headless` — policy
   against the scripted grasp on the same held-out episodes.

The checkpoint's features (`observation.images.wrist`, `observation.state`, joint-target `action`) match what DimOS's
LeRobotPolicyModule expects.
