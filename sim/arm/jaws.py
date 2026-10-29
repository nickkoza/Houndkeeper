"""The gripper's jaws as the sims and the pick routine see them ([v8J], waymark [wjx]): which gripping surface is fitted (HK_JAWS),
how thick its pads are, and so how wide an item the open jaws take. No dependencies, so any sim or tool can use it."""
import os
from pathlib import Path
import numpy as np

JAWS = os.environ.get("HK_JAWS", "rubber")     # "rubber" (as modelled to 25 Sep), "bare" (the fingers as they come), "pads_v1", "pads_v2"
PADS = Path(__file__).resolve().parents[2] / "cad" / "gripper_pads"      # built by build_pads.py from the finger's CAD
TRAVEL = 0.0524                                # m, each jaw's slide (maker_arm.xml gripper joints: 0 to 0.0524125)
CLEAR = 0.0098                                 # m, the open jaws' margin around the widest item they take (104.8 -> 95 mm bare)


def pad_file(jaws=None):
    jaws = jaws or JAWS
    return np.load(PADS / f"jaw_pad_{jaws[5:]}_sim.npz") if jaws.startswith("pads_") else None


def pad_m(jaws=None):
    """thickness of the pad on each jaw (m): the jaws' reading minus this is the half-gap between the gripping faces"""
    H = pad_file(jaws); return float(H["pad_t"]) if H is not None else 0.0


def max_m(jaws=None):
    """widest item the open jaws take (m): their opening less the pads, less the margin to close around it"""
    return 2 * (TRAVEL - pad_m(jaws)) - CLEAR

