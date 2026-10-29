"""The printed mount's parts, parametric in build123d ([3JA] [Lsn]); every dimension comes from config.py.

Parts, all in the robot frame (mm, X forward, Y left, Z up, body centre):
  adapter       stays on the robot: bolted to the back's rear holes, carries the module and the bin ([vqz])
  module        the removable plate: the arm's base on top, the arm host and its electronics in bays below ([JW7], [vQO])
  bench_plate   the adapter's interface on a plate for the bench ([JW7])
  bin           on the adapter behind the module, held by magnets on two locating balls ([Frm])
  hat_seat      the hat's magnetic seat, screwed to the module's top ([4JP])
  coupon        test holes for the bolts, inserts, dowels and magnets, printed first ([ngX])
Each is one clean solid, so the finite-element analysis ([tY3]) can take it as it is."""
import math
from build123d import (Align, Box, Cone, Cylinder, Plane, Polygon, Pos, Rot, SlotCenterToCenter, extrude)
import config as C

MIN = (Align.CENTER, Align.CENTER, Align.MIN)
CORNER = (Align.MIN, Align.MIN, Align.MIN)


def box(x0, x1, y0, y1, z0, z1):
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=CORNER)


def cyl(x, y, z0, z1, d):
    return Pos(x, y, z0) * Cylinder(d / 2, z1 - z0, align=MIN)


def one_solid(p, name):
    """the part as a single solid (a fused result can come back as a compound of one)"""
    s = p.solids()
    assert len(s) == 1, f"{name}: {len(s)} solids, expected one"
    out = s[0]; out.label = name
    return out


def interface_features(top_z, depth_avail):
    """what the module's underside meets, cut into a part whose top is the interface plane: four M5 heat-set insert holes
    and two dowel-pin holes (press fit), identical on the adapter and the bench plate ([JW7])"""
    cuts = []
    d, depth = C.INSERT["M5"]
    for x, y in C.IF_BOLTS:
        cuts.append(cyl(x, y, top_z - depth - 3.0, top_z + 1, d))                                       # insert + bolt tip room
    for x, y in C.IF_DOWELS:
        cuts.append(cyl(x, y, top_z - min(12.0, depth_avail - 2), top_z + 1, C.DOWEL_D - 0.05 + C.PRINT_HOLE_ADD))
    return cuts


def adapter():
    """[vqz] Sits on the flat of the back's top cover (Z 57) from its rear edge to where the head rises, bolted through
    the cover's four rear holes; its deck (the interface plane) carries the module in front and the bin behind.
    Printed upside down (deck on the bed): the arm's bending runs along X, in the layers."""
    z0, z1 = C.COVER_Z, C.Z_IF; x0, x1 = C.ADP_X; w, wf = C.ADP_HALF_W, C.ADP_HALF_W_FOOT
    p = box(x0, x1, -w, w, z0, z1)
    zb = C.Z_BIN
    # the underside follows the top cover (Unitree CAD, go2_air.back_mount): flat at Z 57 within |Y| 38; a ridge at
    # |Y| 42-60 (up to 62.5 at the rear, rising into the head's shoulders from X 60); the inner shoulders rise from X 84 (cut from X 62)
    for s in (1, -1):        # upside down on the bed, these undersides print as tops: no support
        p -= box(x0 - 1, x1 + 1, *sorted((s * wf, s * (w + 1))), z0 - 1, z0 + C.ADP_SIDE_LIFT)
        for (xa, za, zb_, y_in, y_out) in ((56.0, z0 + C.ADP_SIDE_LIFT, 80.5, wf, w + 1), (62.0, z0, 72.5, 18.0, wf)):
            wedge = Polygon((xa, z0 - 1), (x1 + 1, z0 - 1), (x1 + 1, zb_), (xa, za), align=None)
            p -= Pos(0, s * y_in if s > 0 else -y_out, 0) * extrude(Plane.XZ * wedge, amount=-(y_out - y_in))
    # the cover's rear lip rises to 58.4 over the last 6 mm
    lip = Polygon((x0 - 1, z0 - 1), (-121.0, z0 - 1), (-121.0, z0), (x0 - 1, 59.6), align=None)     # anticlockwise in (X, Z)
    p -= Pos(0, -(w + 1), 0) * extrude(Plane.XZ * lip, amount=-2 * (w + 1))
    # front ears for the front interface bolts, beside the arm base
    ex0, ex1 = C.EAR["x"]; yo = C.EAR["y_out"]
    for s in (1, -1):
        p += box(ex0 - 2, ex1, *sorted((s * (w - 6), s * yo)), z1 - 18.0, z1)
    # rear bolts through the cover: clearance hole, socket-head counterbore from the deck leaving 8 mm to clamp
    b = C.REAR_BOLT; seat = 8.0
    for x, y in C.BACK["rear_holes_xy"]:
        p -= cyl(x, y, z0 - 1, zb + 1, C.BOLT_CLEAR[b] + C.PRINT_HOLE_ADD)
        p -= cyl(x, y, z0 + seat, zb + 1, C.HEAD_D[b] + 1.0)
    for c in interface_features(z1, z1 - z0):
        p -= c
    # bin: magnet pockets and two pockets for the locating balls, all open at the deck
    m = C.MAGNET
    for x, y in C.MAGNETS_XY:
        p -= cyl(x, y, zb - m["h"] - m["recess"], zb + 1, m["d"] + C.PRINT_HOLE_ADD)
    for x, y in C.CONES_XY:
        p -= cyl(x, y, zb - C.CONE["ball_d"] / 2, zb + 1, C.CONE["ball_d"] + C.PRINT_HOLE_ADD)
    # weight: open pockets in the underside where little load runs (behind the rear bolts; the side wings),
    # leaving the closed box between the bolts and the interface
    for (ax0, ax1, ay0, ay1) in ((x0 + 6, -102.0, -wf + 6, wf - 6), (-50.0, -4.0, 72.0, w - 8), (-50.0, -4.0, -(w - 8), -72.0),
                                 (-120.0, -60.0, 72.0, w - 8), (-120.0, -60.0, -(w - 8), -72.0)):
        p -= box(ax0, ax1, ay0, ay1, z0 - 1, z1 - 6.0)
    return one_solid(p, "adapter")


def module():
    """[JW7] [vQO] The removable module: the arm's base bolts on top (its four M5, from below into the base's nut traps);
    the arm host and its electronics sit in bays open underneath, closed by the adapter's (or the bench plate's) deck
    when it is fitted. Printed top-down (the arm's seat on the bed: the flattest face, and bays need no support)."""
    z0, z1 = C.Z_IF, C.Z_BASE; x0, x1 = C.MOD_X; w = C.MOD_HALF_W
    p = box(x0, x1, -w, w, z0, z1)
    ex0, ex1 = C.EAR["x"]; yo = C.EAR["y_out"]
    for s in (1, -1):
        p += box(ex0, ex1, *sorted((s * (w - 1), s * yo)), z0, z1)
    for (bx, by) in C.BAYS.values():
        p -= box(bx[0], bx[1], by[0], by[1], z0 - 1, z1 - C.MOD_TOP)
    # bosses inside the bays for the interface bolts and the dowels
    for x, y in C.IF_BOLTS + C.IF_DOWELS:
        p += cyl(x, y, z0, z1 - C.MOD_TOP + 0.01, 14.0)
    # interface bolts: clearance through, socket head sunk flush into the top
    for x, y in C.IF_BOLTS:
        p -= cyl(x, y, z0 - 1, z1 + 1, C.BOLT_CLEAR["M5"] + C.PRINT_HOLE_ADD)
        p -= cyl(x, y, z1 - 6.0, z1 + 1, C.HEAD_D["M5"] + 1.0)
    # dowel holes from below: one round, one slot along the line between them (Y)
    (rx, ry), (sx, sy) = C.IF_DOWELS; dd = C.DOWEL_D + 0.1 + C.PRINT_HOLE_ADD; dep = C.DOWEL_PROUD + 1.5
    p -= cyl(rx, ry, z0 - 1, z0 + dep, dd)
    p -= Pos(sx, sy, z0 - 1) * Rot(0, 0, 90) * extrude(SlotCenterToCenter(3.0, dd), dep + 1)
    # the arm's base: four M5 from below; the ribs under them, counterbored to leave 6 mm under the base
    for dx, dy in C.ARM_BASE["m5_nut_traps_xy"]:
        x, y = C.MOUNT_X + dx, dy
        p += cyl(x, y, z0, z1 - C.MOD_TOP + 0.01, 13.0)
        p -= cyl(x, y, z0 - 1, z1 + 1, C.BOLT_CLEAR["M5"] + C.PRINT_HOLE_ADD)
        p -= cyl(x, y, z0 - 1, z1 - 6.0, C.HEAD_D["M5"] + 1.0)
    # Raspberry Pi stand-offs hanging from the top skin (M2.5 self-tapping into 2.2 mm holes)
    (pcx, pcy), (hx, hy) = C.PI["centre"], C.PI["holes"]; (lx, ly) = C.PI["size"]
    x_first = pcx - lx / 2 + C.PI["hole_off"][0]
    for x in (x_first, x_first + hx):
        for y in (-hy / 2, hy / 2):
            p += cyl(x, y, z1 - C.MOD_TOP - C.PI["standoff"], z1 - C.MOD_TOP + 0.01, 6.0)
            p -= cyl(x, y, z1 - C.MOD_TOP - C.PI["standoff"] - 1, z1 - 1.5, 2.2)
    # the Pi's USB/Ethernet end faces the front wall: a window to reach them; vents through the ribs
    p -= box(x1 - 6, x1 + 1, -29.0, 29.0, z0 - 1, z0 + 22.0)
    for xv in range(70, 141, 12):
        for s in (1, -1):
            p -= box(xv, xv + 6, *sorted((s * 30.0, s * 45.0)), z0 + 6, z0 + 18)
    # the two connectors (power, data) in the side walls, by the rear: nothing trails past the module ([pkg])
    for s, (cw, ch) in ((-1, C.CONNECTOR["power"]), (1, C.CONNECTOR["data"])):
        p -= box(14.0, 14.0 + cw, *sorted((s * (w - 4), s * (w + 1))), z0 + 6, z0 + 6 + ch)
    # speaker grille (left wall) and microphone port (top, behind the base)
    for i in range(-2, 3):
        for k in range(-1, 2):
            p -= Pos(80.0 + 7 * i, w - 1.5, z0 + 15 + 7 * k) * Rot(90, 0, 0) * Cylinder(1.5, 6)
    p -= cyl(*C.MIC_XY, z1 - C.MOD_TOP - 1, z1 + 1, 2.0)
    # the hat's seat ([4JP]) screws on with two M3 into heat-set inserts (the module's top stays flat for printing)
    for x, y in hat_screws():
        p += cyl(x, y, z1 - 10.0, z1, 9.0)
        p -= cyl(x, y, z1 - C.INSERT["M3"][1] - 1.5, z1 + 1, C.INSERT["M3"][0])
    return one_solid(p, "module")


def hat_screws():
    (hx, hy), r = C.HAT["centre"], C.HAT["screw_r"]
    return ((hx - r, hy), (hx + r, hy))


def hat_seat():
    """[4JP] The hat's seat: a flange screwed to the module's top behind the arm base, a boss the bowler's crown opening
    drops over (locating it sideways), and a magnet in the boss's top holding it down. Printed upright."""
    h = C.HAT; hx, hy = h["centre"]; z0 = C.Z_BASE; m = C.MAGNET
    p = Pos(hx, hy, z0) * Cylinder(h["flange_d"] / 2, h["flange_h"], align=MIN)
    top = z0 + h["flange_h"] + h["boss_h"]
    p += Pos(hx, hy, z0) * Cylinder(h["boss_d"] / 2, top - z0, align=MIN)
    p -= cyl(hx, hy, top - m["h"] - m["recess"], top + 1, m["d"] + C.PRINT_HOLE_ADD)
    for x, y in hat_screws():
        p -= cyl(x, y, z0 - 1, top + 1, C.BOLT_CLEAR["M3"] + C.PRINT_HOLE_ADD)
    return one_solid(p, "hat_seat")


def bench_plate():
    """[JW7] The module's interface on a plate for the bench: the same inserts and dowels, at the same places, as the
    adapter's deck (interface_features), and four M8 holes (or C-clamps on the flange) to the bench."""
    z1 = C.Z_IF; z0 = z1 - C.BENCH_T; x0, x1 = C.BENCH_X; w = C.BENCH_HALF_W
    p = box(x0, x1, -w, w, z0, z1)
    for c in interface_features(z1, C.BENCH_T):
        p -= c
    for x, y in C.BENCH_HOLES:
        p -= cyl(x, y, z0 - 1, z1 + 1, C.BOLT_CLEAR["M8"] + C.PRINT_HOLE_ADD)
        p -= cyl(x, y, z0 + 8.0, z1 + 1, C.HEAD_D["M8"] + 2.0)
    # weight: pockets from below between the interface bosses (printed top-down, so they open upward)
    for (ax0, ax1, ay0, ay1) in ((x0 + 8, 2.0, -w + 22, w - 22), (24.0, 96.0, -48.0, 48.0), (46.0, 96.0, 56.0, 98.0), (46.0, 96.0, -98.0, -56.0)):
        p -= box(ax0, ax1, ay0, ay1, z0 - 1, z1 - 5.0)
    return one_solid(p, "bench_plate")


def bin_():
    """[Frm] The bin: an open box on the adapter's deck behind the module, under the folded arm's rear segment. Four
    magnet pairs hold it down; it sits on two balls in the deck (a 45 deg cone socket and a V-slot), so a knock rides it up the balls
    and off, and it drops back to the same place. Printed upright (floor on the bed)."""
    x0, x1 = C.BIN_X; w = C.BIN_HALF_W; z0 = C.Z_BIN; z1 = C.BIN_TOP_Z; t, f = C.BIN_WALL, C.BIN_FLOOR
    p = box(x0, x1, -w, w, z0, z1)
    p -= box(x0 + t, x1 - t, -w + t, w - t, z0 + f, z1 + 1)
    pad = C.MAGNET["h"] + C.MAGNET["recess"] + 1.2                        # floor thickened over the magnets and cones
    for x, y in C.MAGNETS_XY:
        p += cyl(x, y, z0, z0 + pad, 16.0)
        p -= cyl(x, y, z0 - 1, z0 + C.MAGNET["h"] + C.MAGNET["recess"], C.MAGNET["d"] + C.PRINT_HOLE_ADD)
    R = C.CONE["d"] / 2                                                    # 45 deg: as deep as its radius
    (rx, ry), (sx, sy) = C.CONES_XY
    for x, y in ((rx, ry), (sx, sy)):
        p += cyl(x, y, z0, z0 + R + 1.6, 2 * R + 12.0)
    p -= Pos(rx, ry, z0 - 1.0) * Cone(R + 1.0, 0.01, R + 1.0, align=MIN)              # the cone socket
    L = 2 * R + 6.0                                                         # the V-slot, along X (the bin's long axis)
    vee = Polygon((sy - R - 1.0, z0 - 1.0), (sy + R + 1.0, z0 - 1.0), (sy, z0 + R), align=None)
    p -= Pos(sx - L / 2, 0, 0) * extrude(Plane.YZ * vee, amount=L)
    return one_solid(p, "bin")


def coupon():
    """[ngX] A strip of test holes, printed first in the chosen filament: M3/M4/M5/M8 clearance, the three insert
    sizes, the dowel press fit, the dowel slip fit and the magnet pocket, each at -0.2, 0 and +0.2 mm of PRINT_HOLE_ADD."""
    holes = [C.BOLT_CLEAR["M3"], C.BOLT_CLEAR["M4"], C.BOLT_CLEAR["M5"], C.BOLT_CLEAR["M8"], C.INSERT["M3"][0], C.INSERT["M4"][0],
             C.INSERT["M5"][0], C.DOWEL_D - 0.05, C.DOWEL_D + 0.1, C.MAGNET["d"]]
    pitch = 16.0; p = box(0, pitch * len(holes), 0, 3 * pitch, 0, 10.0)
    for i, d in enumerate(holes):
        for j, a in enumerate((-0.2, 0.0, 0.2)):
            p -= cyl(pitch * (i + 0.5), pitch * (j + 0.5), -1 if d != C.MAGNET["d"] else 10.0 - C.MAGNET["h"] - C.MAGNET["recess"], 11, d + C.PRINT_HOLE_ADD + a)
    return one_solid(p, "coupon")


PARTS = {"adapter": adapter, "module": module, "bench_plate": bench_plate, "bin": bin_, "hat_seat": hat_seat, "coupon": coupon}
LOAD_BEARING = ("adapter", "module", "bench_plate")
ON_ROBOT = ("adapter", "module", "bin", "hat_seat")


def build(names=None):
    return {k: f() for k, f in PARTS.items() if names is None or k in names}
