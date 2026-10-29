"""The battery in every real recording ([4hm]): the Go2's low state (charge, pack voltage, current) read once a second
and published as the "battery" stream, so `dimos --record sqlite` keeps it beside odometry and the power procedure
(real/calibration/power_and_pose.md 1, `measure.py power`) runs from the recording.

DimOS's Go2 connection keeps the latest low-state push but does not publish it; it is read through the connection's
`lowstate` RPC (a one-method addition to DimOS's go2/connection.py). The sim has no low state: no battery stream."""
from __future__ import annotations

import threading
from typing import Protocol

from dimos_lcm.sensor_msgs import BatteryState as LCMBatteryState

from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import Out
from dimos.robot.unitree.type.lowstate import LowStateMsg
from dimos.spec.utils import Spec
from dimos.types.timestamped import Timestamped
from dimos.utils import clock   # the robot's clock: sim time in the sim ([3Pv])
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

PERIOD_S = 1.0


class Battery(Timestamped):
    """ts (s), soc (%), voltage (V), current (A, the pack's sign: negative discharging), power (W drawn: positive
    discharging). On the wire a sensor_msgs.BatteryState (percentage 0-1, as ROS keeps it); power is V x -I"""
    msg_name = "sensor_msgs.BatteryState"

    def __init__(self, ts: float | None = None, soc: float = 0.0, voltage: float = 0.0, current: float = 0.0) -> None:
        self.ts = clock.now() if ts is None else ts; self.soc = soc; self.voltage = voltage; self.current = current

    @property
    def power(self) -> float:
        return -self.voltage * self.current

    def lcm_encode(self) -> bytes:
        m = LCMBatteryState(); s = int(self.ts); m.header.stamp.sec, m.header.stamp.nsec = s, int((self.ts - s) * 1e9)
        m.voltage, m.current, m.percentage, m.present = self.voltage, self.current, self.soc / 100, True
        m.power_supply_status = LCMBatteryState.POWER_SUPPLY_STATUS_DISCHARGING if self.current < 0 else LCMBatteryState.POWER_SUPPLY_STATUS_CHARGING if self.current > 0 else LCMBatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        return m.lcm_encode()

    @classmethod
    def lcm_decode(cls, data: bytes) -> Battery:
        m = LCMBatteryState.lcm_decode(data)
        return cls(ts=m.header.stamp.sec + m.header.stamp.nsec / 1e9, soc=round(m.percentage * 100, 3), voltage=m.voltage, current=m.current)

    def __repr__(self) -> str:
        return f"Battery(ts={self.ts:.3f}, soc={self.soc}%, {self.voltage:.2f} V, {self.current:.3f} A, {self.power:.1f} W)"


def battery_from_lowstate(msg: LowStateMsg, ts: float | None = None) -> Battery | None:
    """a Go2 low-state message (dimos LowStateMsg: bms_state.soc %, bms_state.current mA, power_v V) as a Battery;
    None if it carries no battery"""
    try:
        d = msg["data"]; b = d["bms_state"]
        return Battery(ts=ts, soc=float(b["soc"]), voltage=float(d["power_v"]), current=float(b["current"]) / 1000.0)
    except (KeyError, TypeError, ValueError):
        return None


class LowStateSpec(Spec, Protocol):
    def lowstate(self) -> LowStateMsg | None: ...


class BatteryLog(Module):
    battery: Out[Battery]

    _robot: LowStateSpec | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if self._robot is None: logger.warning("battery: no module answers lowstate (DimOS's go2/connection.py lowstate RPC missing?): no battery stream")
        self._stop = threading.Event(); self._thread = threading.Thread(target=self._run, name="battery-log", daemon=True); self._thread.start()

    def _run(self) -> None:
        last = None
        while not self._stop.wait(PERIOD_S):
            if self._robot is None: continue
            try: msg = self._robot.lowstate()
            except Exception: continue
            if msg is None or msg == last: continue                 # nothing yet, or the same push again (the robot went quiet)
            last = msg; b = battery_from_lowstate(msg)
            if b is not None: self.battery.publish(b)

    @rpc
    def stop(self) -> None:
        if getattr(self, "_stop", None): self._stop.set()
        super().stop()
