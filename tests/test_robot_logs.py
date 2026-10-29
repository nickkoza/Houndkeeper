"""[VVJ]: the laptop's logs of what the robot tries (real/bringup/robot_logs.py) listen only on the laptop's house
addresses, resolve nothing, and keep every line the router sends."""
import datetime
import socket
import sys
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "real" / "bringup"))


def _env(tmp_path, text):
    p = tmp_path / ".env"; p.write_text(text); return p


def test_env_reads_values_past_comments_and_quotes(tmp_path):
    import robot_logs
    env = robot_logs.read_env(_env(tmp_path, '# head\nA=1   # note\nB="x y"\n\nC=\n'))
    assert env == {"A": "1", "B": "x y", "C": ""}


def test_resolver_answers_nothing_and_listens_only_on_house_addresses(tmp_path):
    import robot_logs
    s = robot_logs.settings(robot_logs.read_env(_env(tmp_path, "LAPTOP_HOUSE_IPS=10.9.8.7, 10.9.9.7\n")))
    a = robot_logs.dnsmasq_args(s)
    assert "--listen-address=10.9.8.7,10.9.9.7" in a
    assert "--address=/#/" in a and "--no-resolv" in a and not any(x.startswith("--server") for x in a)
    assert "--bind-dynamic" in a and "--except-interface=lo" in a
    assert "--log-queries=extra" in a and any(x.endswith("/robot-dns.log") for x in a)
    assert s["log_dir"] == HK / "real" / "local" / "logs" and s["syslog_port"] == 5514


@pytest.mark.parametrize("ips", ["", "127.0.0.53", "0.0.0.0", "10.0.0.1,127.0.0.1"])
def test_loopback_all_or_no_address_refused(tmp_path, ips):
    import robot_logs
    with pytest.raises(SystemExit):
        robot_logs.settings(robot_logs.read_env(_env(tmp_path, f"LAPTOP_HOUSE_IPS={ips}\n")))


def test_router_line_kept_whole_with_time_and_sender():
    import robot_logs
    t = datetime.datetime(2026, 10, 3, 12, 0, 0, 250000, tzinfo=datetime.timezone.utc)
    ln = robot_logs.syslog_line(t, "10.9.8.1", b"<134>robot-out forward: proto UDP, 10.9.7.20:5353->8.8.8.8:53\n")
    assert ln == "2026-10-03T12:00:00.250+00:00 10.9.8.1 <134>robot-out forward: proto UDP, 10.9.7.20:5353->8.8.8.8:53\n"


def test_syslog_binds_an_address_the_laptop_does_not_hold_yet():
    import robot_logs
    socks = robot_logs.syslog_sockets({"ips": ["192.0.2.77"], "syslog_port": 0})   # TEST-NET: never local
    try:
        assert socks[0].getsockname()[0] == "192.0.2.77"
    finally:
        for so in socks:
            so.close()


def test_window_resolver_answers_only_the_exact_allowed_names_from_the_router(tmp_path):
    """[zAo]: during the contained online window only the allowed names resolve, by the router; the rest stays
    "no such domain" and every query is still logged."""
    import robot_logs
    env = robot_logs.read_env(_env(tmp_path, "LAPTOP_HOUSE_IPS=10.9.8.7\nROBOT_DNS_ALLOW=api.vendor.example, cdn.vendor.example.\n"
                                             "ROBOT_DNS_UPSTREAM=10.9.8.1\n"))
    names, ups = robot_logs.window_settings(env)
    a = robot_logs.dnsmasq_window_args(robot_logs.settings(env), names, ups)
    assert [x for x in a if x.startswith("--server")] == ["--server=/api.vendor.example/10.9.8.1",
                                                         "--server=/cdn.vendor.example/10.9.8.1"]
    assert "--address=/#/" in a and "--no-resolv" in a and "--log-queries=extra" in a
    assert not any(x.startswith("--server") for x in robot_logs.dnsmasq_args(robot_logs.settings(env)))


@pytest.mark.parametrize("allow,up", [("", "10.9.8.1"), ("api.vendor.example", ""), ("vendor.example", "10.9.8.1"),
                                      ("*.vendor.example", "10.9.8.1"), ("#", "10.9.8.1"),
                                      ("api.vendor.example", "127.0.0.1")])
def test_window_resolver_refuses_anything_wider_than_exact_names(tmp_path, allow, up):
    import robot_logs
    env = robot_logs.read_env(_env(tmp_path, f"ROBOT_DNS_ALLOW={allow}\nROBOT_DNS_UPSTREAM={up}\n"))
    with pytest.raises(SystemExit):
        robot_logs.window_settings(env)
