"""[VVJ]: the isolation check (real/bringup/isolation_check.py) passes the robot network as designed and names each
way it can come open: a rule moved, disabled or widened, an interface list changed, a service left on, an online
window open. The fixture is the router's own output, saved once, with every address, name and tag replaced by
placeholders (documentation addresses, "Robot-Net")."""
import re
import socket
import sys
import threading
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HK / "real" / "bringup"))
import isolation_check as ic  # noqa: E402

FIX = HK / "tests" / "fixtures" / "router_isolation"
ENV = {"ROUTER_COMMENT_TAG": "Robot-Net", "ROUTER_ON_ROBOT_NET": "203.0.113.1", "ROUTER_GUEST_IF": "br-guest"}


def _out():
    return {k: (FIX / f"{k}.txt").read_text() for k in ic.commands("Robot-Net")}


def _sub(out, key, pattern, repl):
    new = re.sub(pattern, repl, out[key], count=1, flags=re.M)
    assert new != out[key], f"fixture edit did nothing: {pattern}"
    out[key] = new
    return out


def _move_below(text, comment, below):
    lines = text.splitlines()
    i = next(n for n, ln in enumerate(lines) if f"comment={comment} chain" in ln)
    ln = lines.pop(i)
    j = next(n for n, x in enumerate(lines) if f"comment={below} chain" in x)
    lines.insert(j + 1, ln)
    return "\n".join(lines)


def _failed(res, *words):
    assert not res["passed"] and res["summary"].startswith("FAILED: ")
    assert all(w in res["summary"] for w in words), res["summary"]


def test_the_router_as_designed_passes_and_counts_the_drops():
    res = ic.evaluate(_out(), ENV)
    assert res["passed"], res["failures"]
    assert res["summary"].startswith("passed")
    assert res["details"]["dropped"] == {"out": 1656, "router": 28, "in": 0, "ipv6": 0}
    assert "04:13:32" in res["summary"]


def test_the_routers_own_dynamic_rule_above_does_not_count():
    out = _out()
    assert re.search(r"^\s*0 D .*chain=forward", out["ipv4"], re.M)       # the fasttrack dummy sits above
    assert ic.evaluate(out, ENV)["passed"]


def test_a_robot_rule_below_another_rule_fails_with_its_position():
    out = _out()
    out["ipv4"] = _move_below(out["ipv4"], "Robot-Net: robot network goes nowhere else", "Drop guest to LAN")
    _failed(ic.evaluate(out, ENV), "goes nowhere else", "position")


def test_two_robot_rules_swapped_fail():
    out = _out()
    out["ipv4"] = _move_below(out["ipv4"], "Robot-Net: log every attempt out", "Robot-Net: robot network goes nowhere else")
    _failed(ic.evaluate(out, ENV), "log every attempt out", "position")


def test_a_disabled_drop_fails():
    out = _sub(_out(), "ipv4", r"^(\s*9)   (comment=Robot-Net: nothing else to the router)", r"\1 X \2")
    _failed(ic.evaluate(out, ENV), "nothing else to the router", "disabled")


def test_a_missing_rule_fails():
    out = _sub(_out(), "ipv4", r"^.*comment=Robot-Net: nothing else into the robot network.*\n", "")
    _failed(ic.evaluate(out, ENV), "nothing else into the robot network", "missing")


def test_a_drop_narrowed_by_a_matcher_fails():
    out = _sub(_out(), "ipv4", r"(robot network goes nowhere else chain=forward action=drop)", r"\1 protocol=tcp")
    _failed(ic.evaluate(out, ENV), "goes nowhere else", "changed", "protocol")


def test_an_ipv6_drop_not_at_the_top_fails():
    out = _out()
    out["ipv6"] = _move_below(out["ipv6"], "Robot-Net: no IPv6 out", "defconf: drop invalid")
    _failed(ic.evaluate(out, ENV), "IPv6 forward", "no IPv6 out", "position")


def test_an_open_online_window_is_named():
    out = _sub(_out(), "ipv4", r"^(\s*3   comment=Robot-Net: log every attempt out)",
               "99   comment=Robot-Net online window: robot to the vendor chain=forward action=accept "
               "in-interface-list=Robot dst-address-list=robot-online\n\\1")
    _failed(ic.evaluate(out, ENV), "online window open")


def test_a_second_interface_in_the_robot_list_fails():
    out = _out()
    out["members"] += "3 list=Robot interface=br-guest dynamic=no\n"
    _failed(ic.evaluate(out, ENV), "Robot list")


def test_guest_in_the_lan_list_fails_by_setting_and_by_name():
    out = _out()
    out["members"] += "3 list=LAN interface=br-guest dynamic=no\n"
    _failed(ic.evaluate(out, ENV), "guest network in the LAN list")
    _failed(ic.evaluate(out, {k: v for k, v in ENV.items() if k != "ROUTER_GUEST_IF"}), "guest network in the LAN list")


def test_a_service_left_on_or_open_to_all_fails():
    out = _sub(_out(), "services", r"^(\s*3)  X  (name=telnet)", r"\1     \2")
    _failed(ic.evaluate(out, ENV), "service telnet enabled")
    out = _sub(_out(), "services", r"(name=ssh port=22 proto=tcp available-from=)\S+", r"\1")
    _failed(ic.evaluate(out, ENV), "service ssh reachable from anywhere")
    out = _sub(_out(), "services", r"(name=winbox port=8291 proto=tcp available-from=)\S+", r"\g<1>0.0.0.0/0")
    _failed(ic.evaluate(out, ENV), "service winbox")


def test_a_range_in_the_operators_list_fails():
    out = _out()
    out["operators"] += "1   list=robot-operators address=0.0.0.0/0 creation-time=2026-01-01 00:00:00 dynamic=no\n"
    _failed(ic.evaluate(out, ENV), "operators list holds a range")


def test_every_command_sent_only_reads():
    for cmd in ic.commands("Robot-Net").values():
        assert re.match(r"^(/[\w /-]+ print( |$)|:put |:foreach )", cmd), cmd
        words = set(re.findall(r"[\w-]+", cmd))
        assert not words & {"add", "set", "remove", "enable", "disable", "move", "reset-counters", "unset", "edit",
                            "import", "run", "execute", "reboot"}, cmd


def _env_kh(tmp_path, port_host, key=None):
    kh = tmp_path / "known_hosts"
    if key is not None:
        kh.write_text(f"{port_host} {key.get_name()} {key.get_base64()}\n")
    return {"ROUTER_HOST": "127.0.0.1", "ROUTER_USER": "reader", "ROUTER_PASS": "not-sent", "ROUTER_KNOWN_HOSTS": str(kh)}


def test_no_recorded_host_key_refuses_before_connecting(tmp_path):
    with pytest.raises(ic.HostKeyRefused, match="missing"):
        ic.connect(_env_kh(tmp_path, "127.0.0.1"))


@pytest.fixture
def fake_router():
    """A local SSH server with its own key, that records any password offered to it."""
    paramiko = pytest.importorskip("paramiko")
    key = paramiko.RSAKey.generate(1024)
    offered = []

    class Server(paramiko.ServerInterface):
        def check_auth_password(self, username, password):
            offered.append(password); return paramiko.AUTH_FAILED

        def get_allowed_auths(self, username):
            return "password"

    ls = socket.socket(); ls.bind(("127.0.0.1", 0)); ls.listen(1)

    def serve():
        try:
            conn, _ = ls.accept()
            t = paramiko.Transport(conn); t.add_server_key(key); t.start_server(server=Server())
            t.join(5)
        except Exception:
            pass
    th = threading.Thread(target=serve, daemon=True); th.start()
    yield ls.getsockname()[1], key, offered
    ls.close()


def test_a_different_host_key_is_refused_and_the_password_never_sent(tmp_path, fake_router):
    import paramiko
    port, _real, offered = fake_router
    env = _env_kh(tmp_path, f"[127.0.0.1]:{port}", paramiko.RSAKey.generate(1024))   # an impostor's view: other key
    with pytest.raises(ic.HostKeyRefused, match="differs"):
        ic.connect(env, port=port)
    assert offered == []


def test_a_host_not_recorded_is_refused_and_the_password_never_sent(tmp_path, fake_router):
    port, real, offered = fake_router
    env = _env_kh(tmp_path, "[127.0.0.2]:22", real)                                    # recorded for another address
    with pytest.raises(ic.HostKeyRefused, match="not recorded"):
        ic.connect(env, port=port)
    assert offered == []
