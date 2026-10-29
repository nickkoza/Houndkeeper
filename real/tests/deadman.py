"""[Mj2] silence stops the robot: log odometry, then measure how fast it stopped after the Cockpit page went quiet.

    deadman.py log ODOM.csv              # Zenoh /odom -> time,x,y,yaw lines until SIGTERM (DimOS venv python)
    deadman.py check ODOM.csv EVENTS.jsonl [--max-stop 0.6] [--sim-log SIM.log]

EVENTS.jsonl comes from cockpit_silence.mjs: one line per case, {"case", "armed", "silence", ...} with Unix times.
Times on both sides are this machine's clock (odometry stamped on receipt, so its delay counts against the stop).

A case counts only when odometry ran at MIN_ODOM_HZ or more around it: a loaded machine runs the sim slower than
real time, and the stop, timed in sim, then looks slow on the wall clock (the deadman times on the wall clock).

A case passes when the robot was really driving before the silence (turning faster than DRIVE_MIN_DEG_S) and, from
some moment no later than --max-stop after it, every WINDOW_S of odometry up to HOLD_S after the silence shows:
  - no more turning the way it was driven: under STILL_DEG_S in that direction;
  - any settling bounded: under SETTLE_DEG_S either way and moving under SETTLE_M_S. Halted mid-turn, the gait
    settles back a few degrees the other way, the body shifting a few centimetres, for up to a second (seen at
    4-7 deg/s and 0.03-0.04 m/s); that is the legs settling, not the command still running. (Turning on the spot
    the body's odometry point itself moves only ~0.05 m/s, so speed bounds the settling rather than marks a stop.)
That moment, never earlier than the first odometry after the silence, is the case's stop time. still_s, reported
beside it, is when every window also fell under STILL_DEG_S either way and STILL_M_S (the robot fully still).
"""
import argparse
import json
import math
import signal
import sys
import time

import numpy as np

WINDOW_S = 0.2          # rate window: 3 odometry samples at 15 Hz
STILL_DEG_S = 3.0       # stopped: turning slower than this ...
STILL_M_S = 0.03        # ... and moving slower than this
SETTLE_DEG_S = 10.0     # settling after the stop: turning back the other way slower than this ...
SETTLE_M_S = 0.05       # ... and shifting slower than this
DRIVE_MIN_DEG_S = 20.0  # driving: median turn rate over the last DRIVE_S before the silence (it turns ~40)
DRIVE_S = 1.5
HOLD_S = 2.0            # stays stopped at least this long after the silence
MAX_STOP_S = 0.6        # [Mj2]
MIN_ODOM_HZ = 11.0      # the sim publishes 15 Hz in real time; slower means a loaded machine: no verdict


def load_odom(path):
    a = np.loadtxt(path, delimiter=",", ndmin=2)
    a = a[np.argsort(a[:, 0], kind="stable")]
    a = a[np.r_[True, np.diff(a[:, 0]) > 0]]            # repeated stamps add nothing
    a[:, 3] = np.unwrap(a[:, 3])
    return a


def window_rates(odom, window=WINDOW_S):
    """For each sample i: (signed turn deg/s, speed m/s) over [t_i, t_i + window], or nan where the log runs out."""
    t = odom[:, 0]
    j = np.searchsorted(t, t + window)
    ok = j < len(t)
    j = np.minimum(j, len(t) - 1)
    dt = np.where(ok, t[j] - t, np.nan)
    turn = np.degrees(odom[j, 3] - odom[:, 3]) / dt
    speed = np.hypot(odom[j, 1] - odom[:, 1], odom[j, 2] - odom[:, 2]) / dt
    return turn, speed


def analyse(odom, silence, max_stop=MAX_STOP_S):
    t = odom[:, 0]
    turn, speed = window_rates(odom)
    before = (t >= silence - DRIVE_S) & (t + WINDOW_S <= silence)
    drive = float(np.median(turn[before])) if before.any() else float("nan")
    after = (t >= silence) & (t + WINDOW_S <= silence + HOLD_S)
    near = (t >= silence - DRIVE_S) & (t <= silence + HOLD_S)
    hz = (near.sum() - 1) / (DRIVE_S + HOLD_S)
    r = {"drive_deg_s": round(abs(drive), 1), "odom_hz": round(float(hz), 1), "stop_s": None, "still_s": None,
         "pass": False, "valid": True, "why": ""}
    if hz < MIN_ODOM_HZ:
        r["valid"] = False
        r["why"] = f"invalid: odometry at {hz:.1f} Hz (sim slower than real time; under {MIN_ODOM_HZ}), machine loaded"
        return r
    if not abs(drive) > DRIVE_MIN_DEG_S:
        r["why"] = f"not driving before the silence ({abs(drive):.1f} deg/s <= {DRIVE_MIN_DEG_S})"
        return r
    if t[-1] < silence + HOLD_S:
        r["why"] = "odometry log ends before the hold"
        return r
    idx = np.flatnonzero(after)
    onward = turn[idx] * np.sign(drive)                 # positive: still turning the way it was driven

    def settled_from(moving):
        # the sample after the last window that still moved (the first sample after the silence if none did)
        return t[idx[0]] if not moving.any() else t[idx[np.flatnonzero(moving)[-1]] + 1]
    stop_t = settled_from((onward >= STILL_DEG_S) | (np.abs(onward) >= SETTLE_DEG_S) | (speed[idx] >= SETTLE_M_S))
    still_t = settled_from((np.abs(onward) >= STILL_DEG_S) | (speed[idx] >= STILL_M_S))
    r["stop_s"] = round(float(stop_t - silence), 3)
    r["still_s"] = round(float(still_t - silence), 3)
    tail = after & (t >= stop_t)
    r["rebound_deg_s"] = round(float(np.abs(turn[tail]).max()), 2) if tail.any() else None
    r["after_speed_m_s"] = round(float(speed[tail].max()), 3) if tail.any() else None
    r["pass"] = r["stop_s"] <= max_stop
    if not r["pass"]:
        r["why"] = f"still moving {r['stop_s']} s after the silence (limit {max_stop})"
    return r


def bridge_zeros(sim_log, day):
    """Unix times of the relay bridge's "zero twist" log lines (stamped HH:MM:SS.mmm UTC) on the given UTC day."""
    import datetime as dt
    out = []
    for line in open(sim_log, errors="replace"):
        if "zero twist" not in line:
            continue
        try:
            hms = dt.datetime.strptime(line[:12], "%H:%M:%S.%f").time()
        except ValueError:
            continue
        out.append(dt.datetime.combine(day, hms, tzinfo=dt.timezone.utc).timestamp())
    return np.array(out)


def check(odom_path, events_path, max_stop=MAX_STOP_S, sim_log=None):
    """Each case's result; with the sim log, also zero_s: when the bridge sent its zero (the deadman, or the
    page's own release), so a slow stop splits into the bridge's part and the robot's own halt after it."""
    import datetime as dt
    odom = load_odom(odom_path)
    zeros = None
    if sim_log:
        day = dt.datetime.fromtimestamp(odom[0, 0], dt.timezone.utc).date()
        zeros = bridge_zeros(sim_log, day)
    results = []
    for line in open(events_path):
        if not line.strip():
            continue
        ev = json.loads(line)
        if "silence" not in ev:
            results.append({"case": ev.get("case"), "pass": False, "why": ev.get("error", "no silence time")})
            continue
        r = {"case": ev["case"], **analyse(odom, ev["silence"], max_stop)}
        if zeros is not None:
            z = zeros[(zeros >= ev["silence"] - 0.05) & (zeros <= ev["silence"] + HOLD_S)]
            r["zero_s"] = round(float(z[0] - ev["silence"]), 3) if len(z) else None
        results.append(r)
    return results


def log(path):
    from dimos.core.transport_factory import transport_topic
    from dimos.protocol.pubsub.impl.lcmpubsub import Topic
    from dimos.protocol.pubsub.impl.zenohpubsub import Zenoh
    out = open(path, "w", buffering=1)
    bus = Zenoh(); bus.start()

    def cb(msg, _topic):
        try:
            q = msg.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            out.write(f"{time.time():.3f},{msg.position.x:.4f},{msg.position.y:.4f},{yaw:.4f}\n")
        except Exception:
            pass
    bus.subscribe(Topic(f"{transport_topic('/odom')}/**"), cb)
    done = []
    signal.signal(signal.SIGTERM, lambda *_: done.append(1))
    signal.signal(signal.SIGINT, lambda *_: done.append(1))
    while not done:
        time.sleep(0.1)
    bus.stop(); out.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("log").add_argument("odom")
    c = sub.add_parser("check"); c.add_argument("odom"); c.add_argument("events")
    c.add_argument("--max-stop", type=float, default=MAX_STOP_S)
    c.add_argument("--sim-log", help="the DimOS log, for when the bridge zeroed the twist")
    a = ap.parse_args(argv)
    if a.cmd == "log":
        log(a.odom); return 0
    results = check(a.odom, a.events, a.max_stop, a.sim_log)
    for r in results:
        print(json.dumps(r))
    ok = bool(results) and all(r["pass"] for r in results)
    invalid = not ok and any(not r.get("valid", True) for r in results)
    verdict = "PASS" if ok else "INVALID (machine loaded; rerun)" if invalid else "FAIL"
    print(f"[Mj2] {verdict}: " + ", ".join(f"{r['case']} {r.get('stop_s')} s (still {r.get('still_s')} s)" for r in results))
    return 0 if ok else 3 if invalid else 1


if __name__ == "__main__":
    sys.exit(main())
