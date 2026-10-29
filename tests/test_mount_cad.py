"""The printed mount's parametric parts (printed-mount [3JA]): they fit the X1, the module's interface is the same on the
adapter and the bench plate, the predicted mass stays within the kit budget, and the arm's base lands where [Pvh] put it.
Geometry only, nothing stepped."""
import math
import sys
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "cad" / "mount"))
build123d = pytest.importorskip("build123d")      # the `cad` extra
import config as C            # noqa: E402
import parts                  # noqa: E402
import analysis as A          # noqa: E402


@pytest.fixture(scope="module")
def P():
    return parts.build()


@pytest.fixture(scope="module")
def arm():
    return A.arm_numbers(samples=800)


def holes_at_plane(solid, z, tol=0.05):
    """vertical cylindrical holes that open in the plane Z = z: {(x, y, d)} rounded to 0.01 mm"""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    out = set()
    for f in solid.faces():
        ad = BRepAdaptor_Surface(f.wrapped)
        if ad.GetType() != GeomAbs_Cylinder: continue
        c = ad.Cylinder(); ax = c.Axis(); d = ax.Direction()
        if abs(abs(d.Z()) - 1) > 1e-6: continue
        bb = f.bounding_box()
        if abs(bb.max.Z - z) > tol and abs(bb.min.Z - z) > tol: continue
        p = ax.Location(); out.add((round(p.X(), 2), round(p.Y(), 2), round(2 * c.Radius(), 2)))
    return out


def test_every_part_fits_the_x1_with_margin(P):
    """[ngX]: every piece, in its print orientation, inside the X1's 256 mm cube with 5 mm to spare; no part needs splitting"""
    import build as B
    for k, s in P.items():
        bb = B.on_bed(s, B.ORIENT[k]).bounding_box()
        assert max(bb.size.X, bb.size.Y, bb.size.Z) <= C.BED - 5.0, (k, bb.size)
        assert abs(bb.min.Z) < 1e-6, k


def test_every_part_prints_with_little_or_no_support(P):
    """[ngX]: in its print orientation no part has more than 1 cm2 of overhang steeper than 45 deg, and its level
    downward faces (the tops of holes, pockets and windows: short bridges) total under 12 cm2"""
    import build as B
    sys.path.insert(0, str(HK / "cad")); import printcheck
    for k, s in P.items():
        printcheck.printable(A.to_mesh(B.on_bed(s, B.ORIENT[k]), 0.3), k, bridge_cm2=12.0, slope_cm2=1.0)


def test_load_bearing_parts_are_single_valid_solids(P):
    """[tY3] [ngX]: the adapter, the module and the bench plate are each one clean solid the FEA can take as it is"""
    for k in parts.LOAD_BEARING:
        s = P[k]
        assert s.is_valid and len(s.solids()) == 1 and s.volume > 1000, k


def test_the_module_interface_is_the_same_on_the_adapter_and_the_bench_plate(P):
    """[JW7]: the inserts and dowel holes the module meets sit at the same places and sizes on the adapter's deck and the
    bench plate's top, so the arm keeps its mount geometry on either"""
    a = holes_at_plane(P["adapter"], C.Z_IF); b = holes_at_plane(P["bench_plate"], C.Z_IF)
    want = {(x, y) for x, y in C.IF_BOLTS + C.IF_DOWELS}
    assert {(x, y) for x, y, _ in a} >= want
    assert {h for h in a if (h[0], h[1]) in want} == {h for h in b if (h[0], h[1]) in want}
    # and the module's own holes line up with them
    m = holes_at_plane(P["module"], C.Z_IF)
    assert {(x, y) for x, y in C.IF_BOLTS} <= {(x, y) for x, y, _ in m}


def test_the_arm_base_lands_at_the_decided_position(P, arm):
    """[I9y] [Pvh]: the arm base's four M5 holes in the module are centred on the base axis 100 mm forward of body
    centre, the module's top (the base's seat) at the height the sims assume, and the arm model put there agrees"""
    assert C.MOUNT_X == pytest.approx(1000 * 0.10)
    top = holes_at_plane(P["module"], C.Z_BASE)
    xy = [(x, y) for x, y, d in top if abs(d - (C.BOLT_CLEAR["M5"] + C.PRINT_HOLE_ADD)) < 0.01 and abs(abs(y) - 39.5) < 0.01]
    assert len(xy) == 4
    assert sum(x for x, _ in xy) / 4 == pytest.approx(C.MOUNT_X) and sum(y for _, y in xy) / 4 == pytest.approx(0.0)
    assert P["module"].bounding_box().max.Z == pytest.approx(C.Z_BASE, abs=0.01)
    assert C.Z_BASE == pytest.approx(82.4 + 36.0)                                        # sims' seat + mount_plate_m
    assert arm["base_axis_mm"][0] == pytest.approx(C.MOUNT_X, abs=0.1) and arm["base_axis_mm"][1] == pytest.approx(0, abs=0.1)


def test_predicted_mass_stays_within_the_kit_budget(P):
    """[mgb] [Rrm]: the adapter, module and bin as printed in each candidate filament, plus the electronics and hardware,
    stay within the 1.8 kg budget beyond the arm"""
    hw = sum(C.HARDWARE_G.values()) / 1000
    for k, mt in C.materials().items():
        printed = sum(A.mass_g(P[p], mt["density_g_cm3"])[0] for p in parts.ON_ROBOT) / 1000
        assert printed + hw <= C.KIT_BUDGET, (k, printed, hw)


def test_the_adapter_stays_within_its_walking_fatigue_allowable(P, arm):
    """[vqz] [lUO]: by the beam estimate, the walking stress range in the adapter's skins stays inside every candidate's
    fatigue allowable ([Xtj]), and the stiffness at the arm's base is positive and finite in both bounds"""
    mats = C.materials(); st = A.stiffness(A.to_mesh(P["adapter"], 0.4), mats, A.to_mesh(P["module"], 0.4))
    s = A.stresses(st, A.load_cases(arm)["cases"], arm, mats)
    for k, mt in mats.items():
        assert s["sigma_MPa"]["walking_range"] <= mt["allow"]["bend_xy"]["fatigue_range_allow_1e7_MPa"], k
        b = st["materials"][k]
        assert 0 < b["adapter_alone"]["k_bend_Nm_per_rad"] < b["adapter_and_module_bonded"]["k_bend_Nm_per_rad"]


def test_magnets_keep_their_distance_from_the_encoders_and_the_imu(arm):
    """[Frm] [4JP]: the bin's and the hat's magnets keep the stated distance from every arm motor (its encoder) and from
    the Go2's IMU, where a 10 mm N42 pair adds less than Earth's field at the encoders"""
    m = A.magnets(arm)
    for k in ("bin", "hat"):
        assert m[k]["encoder_mm"] >= C.MAGNET_MIN_MM["encoder"] and m[k]["imu_mm"] >= C.MAGNET_MIN_MM["imu"], (k, m[k])
        assert m[k]["field_at_encoder_mT"] < m["earth_field_mT"]
    assert m["bin"]["margin_walking"] >= 2 and m["hat"]["margin_at_2g"] >= 2


go2 = pytest.importorskip("go2_ref")


@pytest.mark.skipif(not go2.available(), reason="Unitree's Go2 CAD not cached (tools/go2_step_cache.py)")
def test_the_adapter_sits_on_the_cover_without_cutting_into_it(P):
    """[Uaq] [I9y]: against Unitree's CAD of the back (dock removed), the adapter's underside nowhere goes into the cover
    (0.6 mm for the mesh), and the module clears the body"""
    import numpy as np
    body = go2.mesh(exclude=go2.DOCK + tuple(i["id"] for i in go2.index() if A._is_leg(i)))
    adp = A.to_mesh(P["adapter"], 0.4)
    xs = np.arange(C.ADP_X[0] + 1, C.ADP_X[1], 3.0); ys = np.arange(-C.ADP_HALF_W + 1, C.ADP_HALF_W, 3.0)
    O = np.array([[x, y, 0.0] for x in xs for y in ys]); D = np.tile([0, 0, 1.0], (len(O), 1))
    loc, ir, _ = adp.ray.intersects_location(O, D, multiple_hits=False)
    top = go2.top_z(body, xs, ys).ravel()
    gaps = [z - top[i] for i, z in zip(ir, loc[:, 2]) if not math.isnan(top[i])]
    assert min(gaps) > -0.6
    assert A.min_dist(A.to_mesh(P["module"], 0.4), body) > 2.0
