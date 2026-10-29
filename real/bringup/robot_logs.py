#!/usr/bin/env python3
"""The laptop's two logs of what the robot tries while it is on the robot network ([VVJ], robot-network.md).

  dns      every name the robot looks up, each answered "no such domain" (dnsmasq, logging only, nothing resolved)
  syslog   the router's firewall log of the robot network ("robot-out " / "robot-in "), sent here by remote syslog,
           because the router itself keeps only ~1,000 lines in memory
  dns-window   the contained online window only (first-boot.md step 1, decision [zAo]): as dns, but the names in
           ROBOT_DNS_ALLOW (and only those) are answered, by the router's resolver (ROBOT_DNS_UPSTREAM), the same
           resolver that fills the router's allow-list, so the robot and the firewall see the same addresses.
           Run by hand in place of hk-robot-dns while the window is open; stop it when the window closes.

Both listen only on the laptop's own house addresses (LAPTOP_HOUSE_IPS in real/.env), never on 0.0.0.0 or the
loopback, so the laptop's own DNS (systemd-resolved on 127.0.0.53/54) is untouched. An address the laptop doesn't
hold right now (it is on one house network at a time) is taken up when it appears: dnsmasq's --bind-dynamic, and
IP_FREEBIND for the syslog socket. The logs go to ROBOT_LOG_DIR (default real/local/logs/, gitignored):
robot-dns.log and router.log.

Settings, all from real/.env (no house specifics in this file):
  LAPTOP_HOUSE_IPS   the laptop's fixed house addresses, comma-separated (the robot network's DNS servers)
  ROBOT_LOG_DIR      where the logs go (optional; relative paths are from the repository)
  ROBOT_SYSLOG_PORT  the UDP port the router sends its log to (optional, default 5514: no privilege needed)
  ROBOT_DNS_ALLOW    dns-window only: the exact names answered, comma-separated (learned from robot-dns.log first)
  ROBOT_DNS_UPSTREAM dns-window only: the router's house-side addresses, comma-separated

Usage: robot_logs.py dns | dns-window | syslog | show      (systemd units: hk-robot-dns.service, hk-robot-syslog.service)"""
import datetime
import os
import shutil
import socket
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[2]
ENV = HK / "real" / ".env"
IP_FREEBIND = getattr(socket, "IP_FREEBIND", 15)        # Linux: bind an address the host doesn't hold yet


def read_env(path=ENV):
    """KEY=value lines; '#' starts a comment (also after a value); surrounding quotes dropped."""
    env = {}
    for ln in Path(path).read_text().splitlines():
        ln = ln.strip()
        if not ln or ln.startswith("#") or "=" not in ln:
            continue
        k, v = ln.split("=", 1)
        v = v.split("#", 1)[0].strip().strip("'\"")
        env[k.strip()] = v
    return env


def settings(env):
    ips = [a.strip() for a in env.get("LAPTOP_HOUSE_IPS", "").replace(" ", ",").split(",") if a.strip()]
    if not ips:
        sys.exit("robot_logs: LAPTOP_HOUSE_IPS is empty in real/.env (the laptop's fixed house addresses)")
    for a in ips:
        socket.inet_aton(a)                                  # an IPv4 address, or this raises
        if a.startswith("127.") or a == "0.0.0.0":
            sys.exit(f"robot_logs: {a} is not a house address; the resolver must not take the loopback or all addresses")
    log_dir = Path(env.get("ROBOT_LOG_DIR") or "real/local/logs")
    if not log_dir.is_absolute():
        log_dir = HK / log_dir
    return {"ips": ips, "log_dir": log_dir, "syslog_port": int(env.get("ROBOT_SYSLOG_PORT") or 5514)}


def dnsmasq_args(s, dnsmasq="dnsmasq"):
    """A resolver that answers nothing and logs everything, on the house addresses only."""
    return [dnsmasq,
            "--keep-in-foreground", "--conf-file=/dev/null",       # no system config, no defaults from elsewhere
            "--no-resolv", "--no-hosts", "--no-poll",               # no upstream: nothing is ever resolved
            "--address=/#/",                                         # every name: "no such domain"
            "--bind-dynamic", "--listen-address=" + ",".join(s["ips"]),
            "--except-interface=lo",                                 # never the loopback (systemd-resolved's)
            "--cache-size=0", "--pid-file=", "--user=", "--group=",  # runs as the unit's user, nothing to drop
            "--log-queries=extra", "--log-facility=" + str(s["log_dir"] / "robot-dns.log")]


def window_settings(env):
    """The online window's allowed names and the resolver that answers them; refuses anything wider than exact names."""
    names = [n.strip().lower().rstrip(".") for n in env.get("ROBOT_DNS_ALLOW", "").split(",") if n.strip()]
    ups = [a.strip() for a in env.get("ROBOT_DNS_UPSTREAM", "").split(",") if a.strip()]
    if not names or not ups:
        sys.exit("robot_logs: dns-window needs ROBOT_DNS_ALLOW (names) and ROBOT_DNS_UPSTREAM (the router) in real/.env")
    for n in names:
        if n.count(".") < 2 or not all(part and all(c.isalnum() or c == "-" for c in part) for part in n.split(".")):
            sys.exit(f"robot_logs: {n!r} is not an exact host name (no wildcards, no bare domains)")
    for a in ups:
        socket.inet_aton(a)
        if a.startswith("127.") or a == "0.0.0.0":
            sys.exit(f"robot_logs: {a} is not the router")
    return names, ups


def dnsmasq_window_args(s, names, ups, dnsmasq="dnsmasq"):
    """As dnsmasq_args, plus: each allowed name (and only it) forwarded to the router; everything else still "no such
    domain", and every query still logged. dnsmasq takes the longest matching domain, so /#/ stays the rest."""
    a = dnsmasq_args(s, dnsmasq)
    i = a.index("--address=/#/") + 1
    return a[:i] + [f"--server=/{n}/{u}" for n in names for u in ups] + a[i:]


def _dnsmasq_exe():
    exe = shutil.which("dnsmasq") or "/usr/sbin/dnsmasq"
    if not Path(exe).exists():
        sys.exit("robot_logs: dnsmasq not found (Debian/Ubuntu: the dnsmasq-base package)")
    return exe


def run_dns(s):
    s["log_dir"].mkdir(parents=True, exist_ok=True)
    exe = _dnsmasq_exe()
    os.execv(exe, dnsmasq_args(s, exe))


def run_dns_window(s, env):
    names, ups = window_settings(env)
    s["log_dir"].mkdir(parents=True, exist_ok=True)
    exe = _dnsmasq_exe()
    print(f"robot_logs: online window resolver: {', '.join(names)} answered; all else refused", flush=True)
    os.execv(exe, dnsmasq_window_args(s, names, ups, exe))


def syslog_line(when, src, data):
    """One line of router.log: the laptop's time (the same internet time as the router, [x20]), who sent it, the
    message as sent (its <priority> and the router's own stamp kept)."""
    text = data.decode("utf-8", errors="replace").rstrip("\r\n\0").replace("\n", " ")
    return f"{when.isoformat(timespec='milliseconds')} {src} {text}\n"


def syslog_sockets(s):
    socks = []
    for a in s["ips"]:
        so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        so.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        so.setsockopt(socket.IPPROTO_IP, IP_FREEBIND, 1)
        so.bind((a, s["syslog_port"]))
        socks.append(so)
    return socks


def run_syslog(s):
    import selectors
    s["log_dir"].mkdir(parents=True, exist_ok=True)
    sel = selectors.DefaultSelector()
    for so in syslog_sockets(s):
        sel.register(so, selectors.EVENT_READ)
    path = s["log_dir"] / "router.log"
    print(f"robot_logs: router log on {','.join(s['ips'])} udp/{s['syslog_port']} -> {path}", flush=True)
    with open(path, "a", buffering=1) as out:
        while True:
            for key, _ in sel.select():
                data, (src, _port) = key.fileobj.recvfrom(8192)
                out.write(syslog_line(datetime.datetime.now().astimezone(), src, data))


def main(argv):
    cmd = argv[1] if len(argv) > 1 else ""
    env = read_env()
    s = settings(env)
    if cmd == "dns":
        run_dns(s)
    elif cmd == "dns-window":
        run_dns_window(s, env)
    elif cmd == "syslog":
        run_syslog(s)                        # every sender is logged with its address (the router has one per house network)
    elif cmd == "show":
        print(" ".join(dnsmasq_args(s)))
        print(f"syslog: {','.join(s['ips'])} udp/{s['syslog_port']} -> {s['log_dir'] / 'router.log'}")
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main(sys.argv)
