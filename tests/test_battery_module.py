"""[4hm]: the battery stream every real recording carries (src/houndkeeper/battery_module.py) reads the Go2's low state
in the units the power procedure needs: mA to A, the pack's sign kept, the power drawn positive while discharging."""
import copy


def test_a_low_state_message_becomes_a_battery_sample():
    from dimos.robot.unitree.type.lowstate import raw_odom_msg_sample as sample   # DimOS's recorded push: -2481 mA, 28.33 V, 55%
    from houndkeeper.battery_module import Battery, battery_from_lowstate
    b = battery_from_lowstate(sample, ts=1000.25)
    assert b.soc == 55 and abs(b.voltage - 28.331045) < 1e-6 and abs(b.current + 2.481) < 1e-9
    assert abs(b.power - 28.331045 * 2.481) < 1e-6 and b.power > 0                # discharging: power drawn is positive
    c = copy.deepcopy(sample); c["data"]["bms_state"]["current"] = 1500           # charging: current and power change sign
    assert battery_from_lowstate(c).power < 0
    r = Battery.lcm_decode(b.lcm_encode())                                        # what the recording keeps
    assert r.ts == 1000.25 and r.soc == 55 and abs(r.voltage - b.voltage) < 1e-4 and abs(r.current - b.current) < 1e-6
    assert battery_from_lowstate({"type": "msg", "topic": "rt/lf/lowstate", "data": {}}) is None


def test_the_go2_connection_answers_the_modules_low_state_ref():
    from dimos.robot.unitree.go2.connection import GO2Connection
    from dimos.spec.utils import spec_annotation_compliance
    from houndkeeper.battery_module import LowStateSpec
    assert spec_annotation_compliance(GO2Connection, LowStateSpec)


def test_a_recording_keeps_the_battery_stream(tmp_path):
    from dimos.memory.store.sqlite import SqliteStore
    from houndkeeper.battery_module import Battery
    st = SqliteStore(path=str(tmp_path / "memory.db")); st.start(); s = st.stream("battery", Battery)
    for i in range(3): s.append(Battery(ts=100.0 + i, soc=80 - i, voltage=29.0, current=-5.0), ts=100.0 + i)
    st.stop(); st = SqliteStore(path=str(tmp_path / "memory.db")); st.start()
    got = [o.data for o in sorted(st.stream("battery"), key=lambda o: o.ts)]; st.stop()
    assert [b.soc for b in got] == [80, 79, 78] and abs(got[0].power - 145.0) < 1e-3
