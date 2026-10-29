"""The real arm's first runs are capped, and each raise is recorded with its measurement ([JYS], decision [g5J] in the
Maker Arm spec [oIS]): the adapter's three commissioning shares live in params/robot.yaml, and a share above the
commissioning quarter is the latest row for it in real/calibration/arm_limits_log.csv."""
import csv
from pathlib import Path

from houndkeeper import params

LOG = Path(__file__).resolve().parents[1] / "real" / "calibration" / "arm_limits_log.csv"
SHARES = {"speed_scale", "grip_force_scale", "gravity_scale"}
COMMISSIONING = 0.25


def _problems(shares, rows):
    """what keeps the shares from being the commissioning quarter or their latest recorded, measured raise"""
    out = [f"shares {sorted(shares)} are not {sorted(SHARES)}"] if set(shares) != SHARES else []
    for r in rows:
        if r["share"] not in SHARES or not r["date"].strip() or not r["measurement"].strip():
            out.append(f"log row {r} needs a known share, a date and its measurement")
        elif not 0 < float(r["to"]) <= 1:
            out.append(f"log row {r} raises outside (0, 1]")
    for name, value in shares.items():
        raises = [r for r in rows if r["share"] == name]
        want = float(raises[-1]["to"]) if raises else COMMISSIONING
        if value != want:
            out.append(f"{name} is {value}, not {want} ({'its latest logged raise' if raises else 'the commissioning share'})")
    return out


def _rows():
    with LOG.open() as f:
        return list(csv.DictReader(f))


def test_the_shares_are_the_quarter_or_a_recorded_raise():   # [JYS]
    assert _problems(params.get("maker_arm.commissioning"), _rows()) == []


def test_a_raise_without_its_measurement_is_caught():   # [JYS] planted errors
    quarter = dict.fromkeys(SHARES, COMMISSIONING)
    assert _problems(quarter, []) == []
    assert _problems({**quarter, "speed_scale": 0.5}, [])    # raised, nothing logged
    row = {"date": "2026-11-02", "share": "speed_scale", "from": "0.25", "to": "0.5", "measurement": ""}
    assert _problems({**quarter, "speed_scale": 0.5}, [row])   # logged without its measurement
    row["measurement"] = "tracking error 0.01 rad at 1.25 rad/s, 20 moves"
    assert _problems({**quarter, "speed_scale": 0.5}, [row]) == []
    assert _problems(quarter, [row])   # the log says 0.5, the params still the quarter
