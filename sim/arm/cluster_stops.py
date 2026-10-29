#!/usr/bin/env python3
"""Where the lying robot parks relative to a find ([S7o]): one stop per find, approached head-on, parked so the find
sits where the arm can grip it from the most directions (front_offset). The lying footprint and reach cells are here
too. Grouping several finds into one stop was removed (backlogged; deleted before the first commit, so
not in git; its design and last results are in docs/backlog.md)."""
from pathlib import Path
_HK = Path(__file__).resolve().parents[2]   # this checkout (a worktree too)
import json, math, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(_HK / "sim" / "house"))
HERE = Path(__file__).resolve().parent
from scipy import ndimage as _ndi
RM = json.loads((HERE / "reach_lying.json").read_text()); RX, RY, REACH_FULL = np.array(RM["xs"]), np.array(RM["ys"]), np.array(RM["reach"], bool)
REACH = _ndi.binary_erosion(REACH_FULL)          # plan with a 5 cm margin inside the reach bands (parking lands ~10 cm off)
LIE_SETTLE_M = 0.035              # the body settles this far back as it lies down (sim Go1, measured in DimOS's sim; not
                                 # measured in the standalone sim): a head-on approach parks short by it
FOOT = (0.40, 0.25)                     # lying footprint half-extents incl. splayed legs + 5 cm margin (est.)
OFFS = np.array([(RX[j], RY[i]) for i in range(len(RY)) for j in range(len(RX)) if REACH[i, j]])


def front_offset(max_lean=30, park_sigma=0.04):
    """where a head-on approach should leave an item in the robot's frame: where the share of jaw directions the arm can
    reach (reach_dirs.py; an item's narrow side is wherever it landed) is highest on average over the parking scatter
    (park_sigma, m), so the item's likely spots, not just the target, keep their grips open; cells off the map (under
    the body, out of reach) count as none. Without that map: deepest inside the zone reachable in some direction
    (reach_tilt.py)"""
    import json as _json
    from scipy import ndimage as ndi_
    lean = min((0, 15, 30, 45), key=lambda t: abs(t - max_lean))
    fd = HERE / f"reach_dirs_tilt{lean}.json"
    if fd.exists():                                                                # the most jaw directions open, on average,
        D = _json.loads(fd.read_text()); xs, ys = np.array(D["xs"]), np.array(D["ys"]); share = np.array(D["share"])   # over
        cell = float(xs[1] - xs[0]); pad = int(np.ceil(3 * park_sigma / cell)) + 1                          # where parking
        E = ndi_.gaussian_filter(np.pad(share, pad), park_sigma / cell, mode="constant")[pad:-pad, pad:-pad]   # scatter leaves
        i, j = np.unravel_index(np.argmax(E), E.shape); return np.array([xs[j], ys[i]], float)              # the item
    M = _json.loads((HERE / f"reach_lying_tilt{lean}.json").read_text()); xs, ys = np.array(M["xs"]), np.array(M["ys"])
    R = np.array(M["reach"], bool) & (xs[None, :] >= 0.36); depth = ndi_.distance_transform_edt(R)
    i, j = np.unravel_index(np.argmax(depth), depth.shape); return np.array([xs[j], ys[i]], float)
