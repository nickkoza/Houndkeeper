#!/usr/bin/env python3
"""GraspGenX served over a pipe for the grasp bench (waymark [uVy]). Run inside DimOS's isolated GraspGenX environment
(`native/python/graspgenx`, launched by pick.GraspGenXClient): loads DimOS's own GraspGenX runtime once, with the
Maker Arm's jaws as a sweep-volume gripper, then answers pickled (N x 3 points) requests with (poses N x 4 x 4,
scores N), best first. The poses are in the points' frame, in DimOS's grasp convention (the grasp centre; z the
approach, y the way the jaws close): GraspGenX's gripper frame (MAKER_ARM_JAWS) carried to it by
MAKER_ARM_GRASP_FRAME_TO_TCP, as DimOS's GraspGenX module applies its grasp_frame_to_tcp (sim-grasp [WuR])."""
import pickle, sys                                                    # a pipe between two of our own processes (pick.py spawns this)

# The Maker Arm's jaws in GraspGenX's gripper frame (origin at the wrist, link 7; z along the approach, x the way the
# jaws close), measured from the SDK's URDF (b30d05a) on 29 Sep 2026: each jaw's inner face is 2.4 cm wide and runs
# from the fingertips, 20.4 cm from the wrist, 6.1 cm back toward it; the jaws open to 10.48 cm.
MAKER_ARM_JAWS = {
    "extents_open": (0.1048, 0.024, 0.061), "offset_open": (0.0, 0.0, 0.1732),
    "extents_half_open": (0.0524, 0.024, 0.061), "offset_half_open": (0.0, 0.0, 0.1732),
    "fingertip_depth": 0.2038, "family": "parallel_2f",
}


# GraspGenX's gripper frame to the grasp centre: 16.96 cm along the approach (the URDF's grasp_center), axes turned so
# y closes the jaws (the same form as DimOS's own xArm transform, XARM_GRASP_FRAME_TO_TCP)
MAKER_ARM_GRASP_FRAME_TO_TCP = ((0.0, 1.0, 0.0, 0.0), (-1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 1.0, 0.1696), (0.0, 0.0, 0.0, 1.0))


def main():
    import os
    out = os.fdopen(os.dup(1), "wb"); os.dup2(2, 1)                        # answers on their own copy of stdout; everything the
    sys.stdout = sys.stderr                                                 # model prints (Python or C) goes to stderr instead
    import numpy as np, torch
    if os.environ.get("HK_GGX_DEVICE") == "cpu":                             # GraspGenX calls .cuda() throughout; on the CPU (the
        torch.Tensor.cuda = lambda self, *a, **k: self                      # laptop's GPU is full with the sim) those become no-ops
        torch.nn.Module.cuda = lambda self, *a, **k: self
        torch.set_num_threads(int(os.environ.get("HK_GGX_THREADS", "4")))
    elif torch.cuda.is_available(): torch.cuda.set_per_process_memory_fraction(0.25)                     # the sim's renderer shares the card: a quarter here
    from dimos.manipulation.grasping.grasp_gen_x.module import GraspGenXConfig, SweepVolumeGripperConfig
    from graspgenx_runtime.backend import GraspGenXRuntime
    rt = GraspGenXRuntime(GraspGenXConfig(gripper=SweepVolumeGripperConfig(**MAKER_ARM_JAWS)))
    pickle.dump({"ready": True}, out); out.flush()
    inp = sys.stdin.buffer
    while True:
        try: pts = pickle.load(inp)
        except EOFError: return
        poses, scores = rt.infer(np.asarray(pts, np.float32))
        if torch.cuda.is_available(): torch.cuda.empty_cache()                                       # hand back what it held: the renderer needs it between asks
        order = np.argsort(-scores.reshape(-1), kind="stable")
        tcp = poses[order] @ np.asarray(MAKER_ARM_GRASP_FRAME_TO_TCP)
        pickle.dump((tcp, scores.reshape(-1)[order]), out); out.flush()


if __name__ == "__main__":
    main()
