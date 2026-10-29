"""The bin on the robot's back, in one place: the arm model builds it from these, the grasp tiers and the robot judge
what fits it, and the scorer asks whether a toy ended up in it (sim-slice [QyZ])."""
import numpy as np

TRAY_X, TRAY_HALF = -0.15, (0.15, 0.11)      # bin 30 x 22 cm inside, centre 15 cm behind trunk centre (est.; the folded arm
                                             # lies over its front half 1.6 cm above the rim: what stands there blocks the stow, [G0m])
BIN_INNER_M = (2 * TRAY_HALF[0], 2 * TRAY_HALF[1])


def footprint_fits(length_m: float, breadth_m: float, tol_m: float = 0.0) -> bool:
    """does a footprint, long side and short side, lie inside the bin (its long side along the bin's)?"""
    a, b = sorted((length_m, breadth_m), reverse=True)
    return a <= BIN_INNER_M[0] + tol_m and b <= BIN_INNER_M[1] + tol_m


def in_bin(p, trunk_pos, trunk_quat, margin: float = 0.05, min_z: float = 0.15) -> bool:
    """is a toy's centre over the bin, within `margin` of its walls, and up off the floor (a shoe resting against one
    side counts; one on the robot's back forward of the bin, or under it, doesn't). The test had been 35 cm from the
    trunk's centre, which counted toys that weren't in it"""
    p, t = np.asarray(p, float), np.asarray(trunk_pos, float)
    if p[2] <= min_z: return False
    w, x, y, z = np.asarray(trunk_quat, float); yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    dx, dy = p[0] - t[0], p[1] - t[1]
    lx, ly = np.cos(yaw) * dx + np.sin(yaw) * dy, -np.sin(yaw) * dx + np.cos(yaw) * dy   # in the trunk's frame
    return bool(abs(lx - TRAY_X) <= TRAY_HALF[0] + margin and abs(ly) <= TRAY_HALF[1] + margin)   # a plain bool: it is counted and written to JSON
