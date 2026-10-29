"""floor_register: the stitch's match weights ([WFI]), each match firm only across what it can see."""
def test_a_ridge_peak_is_firm_across_and_tells_nothing_along():
    """[WFI] the stitch weighs each match by its own sharpness: a stripe's match fixes it across the
    stripe and leaves it free along it"""
    import numpy as np
    from houndkeeper import floor_register as fr
    i, j = np.mgrid[-8:9, -8:9].astype(float)
    ridge = 0.6 - 0.05 * i ** 2                     # sharp across rows, flat along columns
    I = fr.peak_information(ridge, (8, 8))
    w, V = np.linalg.eigh(I)
    assert w[0] < 1e-6 and abs(w[1] - 0.1) < 1e-3
    assert abs(abs(V[0, 1]) - 1) < 1e-6             # the firm direction is the rows'
    spot = 0.6 - 0.05 * i ** 2 - 0.05 * j ** 2
    assert np.allclose(fr.peak_information(spot, (8, 8)), 0.1 * np.eye(2), atol=1e-6)


def test_a_belief_over_the_surface_is_tight_for_a_peak_long_for_a_ridge_wide_for_two_peaks():
    """[WFI] each match's uncertainty from its whole correlation surface (weighed by how well it fits)"""
    import numpy as np
    from houndkeeper import floor_register as fr
    i, j = np.mgrid[-8:9, -8:9].astype(float)
    mu, cov = fr.surface_belief(0.6 - 0.05 * (i ** 2 + j ** 2), 0.05)
    assert np.allclose(mu, [8, 8]) and cov[0, 0] < 1.0 and cov[1, 1] < 1.0
    mu, cov = fr.surface_belief(0.6 - 0.05 * i ** 2, 0.05)                       # ridge along the columns
    assert cov[1, 1] > 10 * cov[0, 0]
    two = np.maximum(0.6 - 0.05 * (i ** 2 + (j - 5) ** 2), 0.59 - 0.05 * (i ** 2 + (j + 5) ** 2))
    mu, cov = fr.surface_belief(two, 0.05)
    assert cov[1, 1] > 20                                                        # two tiles apart: hardly a match


def test_the_mount_carries_the_cameras_place_as_well_as_its_turn():
    """[KkQ] the head camera's calibrated mount: its small turn and, tuned as a constant (a
    static error is treated as tuning), how far it truly sits from where the model puts it, in the body's frame"""
    import numpy as np
    from houndkeeper import calibrate as cal
    R, off = cal.mount({"pitch_deg": 0.0, "yaw_deg": 0.0, "roll_deg": 0.0, "fwd_m": 0.02})
    assert np.allclose(R, np.eye(3)) and np.allclose(off, [0.02, 0.0, 0.0])
    R, off = cal.mount(None)
    assert np.allclose(R, np.eye(3)) and np.allclose(off, 0.0)
    Twb = np.eye(4); c, s = np.cos(np.pi / 2), np.sin(np.pi / 2); Twb[:2, :2] = [[c, -s], [s, c]]
    assert np.allclose(cal.camera_place(np.array([1.0, 2.0, 0.3]), Twb, np.array([0.02, 0.0, 0.0])), [1.0, 2.02, 0.3])   # facing +y
