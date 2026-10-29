"""Printability check shared by every printed part: fits the printer, sits flat, prints without supports."""
import numpy as np

BED = 256.0                                            # Bambu Lab X1 Carbon build volume, mm (the project's printer)


def printable(t, name, max_overhang=45.0, bridge_cm2=4.0, slope_cm2=0.2):
    """fits the bed, sits flat on it, and prints without supports: downward faces are either level bridges (the tops of
    holes and nut pockets, short spans) up to bridge_cm2 in all, or sloped ones no steeper than max_overhang from
    vertical (anything steeper, over slope_cm2, fails). Returns (bridge, slope) areas in cm2."""
    assert (t.extents <= BED).all(), f"{name}: {np.round(t.extents)} mm does not fit a {BED:.0f} mm bed"
    z0 = t.bounds[0][2]; on_bed = t.triangles_center[:, 2] < z0 + 0.05
    nz = t.face_normals[:, 2]; a = t.area_faces / 100.0
    level = (nz < -0.985) & ~on_bed                                              # a downward face alpha from vertical has nz = -sin(alpha)
    slope = (nz < -np.sin(np.radians(max_overhang + 1))) & (nz >= -0.985) & ~on_bed   # (exactly 45 deg, e.g. a teardrop roof, is fine)
    b, sl = float(a[level].sum()), float(a[slope].sum())
    assert sl <= slope_cm2, f"{name}: {sl:.2f} cm2 of sloped overhang steeper than {max_overhang:.0f} deg"
    assert b <= bridge_cm2, f"{name}: {b:.2f} cm2 of level bridges (more than short spans)"
    return b, sl
