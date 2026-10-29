#!/usr/bin/env python3
"""Is the robot network still closed as designed? Read-only, over SSH to the house router ([VVJ], robot-network.md).

It reads the router's interface lists, firewall (IPv4 and IPv6), services and addresses, and checks that:
  - the Robot interface list holds only the robot network's bridge (the interface carrying ROUTER_ON_ROBOT_NET),
    and neither it nor the guest network is in the LAN list;
  - the robot network's rules exist, are enabled, are exactly as designed, and sit above every other rule in their
    chains in the designed order (IPv4 forward and input; the IPv6 drops at the top of theirs);
  - no other rule carries the robot network's tag (an open online window shows up here);
  - the operators list holds single hosts only;
  - the router's services are as left: telnet, ftp, www, www-ssl, api, api-ssl and reverse-proxy disabled; ssh and
    winbox enabled and reachable only from the private network (the LAN list's own subnets);
and reads the counters of the robot network's drop rules: attempts dropped since the router last booted.

It changes nothing on the router: every command it sends is a print or a :put. It writes ISOLATION_JSON (default
real/local/isolation.json, gitignored) with a one-line "summary" ("passed: ..." or "FAILED: <what>") and the details;
the phone cockpit's Status page shows the summary (src/houndkeeper/cockpit_data.py).

Settings, all from real/.env (no house specifics in this file):
  ROUTER_HOST, ROUTER_USER, ROUTER_PASS   the router's SSH login (a read-only user is enough)
  ROUTER_KNOWN_HOSTS                      the router's recorded SSH host key (default real/local/router_known_hosts);
                                          any other key, or none recorded, is refused before the password is sent
  ROUTER_ON_ROBOT_NET                     the router's address on the robot network
  ROUTER_COMMENT_TAG                      the comment prefix of the robot network's items (default: ROBOT_SSID)
  ROUTER_GUEST_IF                         the guest network's interface (optional; default: any with "guest" in its name)
  ISOLATION_JSON                          where the result goes (optional)

Needs paramiko for the SSH (the evaluation itself needs nothing). Usage:
  isolation_check.py            check and write the result; exit 0 if passed, 1 if not
  isolation_check.py --dump DIR save the raw router output to DIR (for test fixtures; replace addresses before use)
Timer: hk-isolation-check.service / .timer beside this file."""
import datetime
import ipaddress
import json
import os
import re
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from robot_logs import ENV, read_env  # noqa: E402

ROBOT_LIST = "Robot"
OPERATORS = "robot-operators"

# The designed rules, top of each chain, in order: comment after "<tag>: ", then every attribute the rule has
# (beyond chain and comment). Exact: a matcher added narrows an accept but can hollow out a drop.
IPV4_DESIGN = {
    "forward": [
        ("operators into the robot network",
         {"action": "accept", "src-address-list": OPERATORS, "out-interface-list": ROBOT_LIST}),
        ("robot network to the operators",
         {"action": "accept", "dst-address-list": OPERATORS, "in-interface-list": ROBOT_LIST}),
        ("log every attempt out",
         {"action": "log", "connection-state": "new", "in-interface-list": ROBOT_LIST, "log-prefix": "robot-out"}),
        ("robot network goes nowhere else", {"action": "drop", "in-interface-list": ROBOT_LIST}),
        ("nothing else into the robot network", {"action": "drop", "out-interface-list": ROBOT_LIST}),
    ],
    "input": [
        ("addresses", {"action": "accept", "protocol": "udp", "in-interface-list": ROBOT_LIST, "dst-port": "67"}),
        ("time", {"action": "accept", "protocol": "udp", "in-interface-list": ROBOT_LIST, "dst-port": "123"}),
        ("log what it tries on the router",
         {"action": "log", "connection-state": "new", "in-interface-list": ROBOT_LIST, "log-prefix": "robot-in"}),
        ("nothing else to the router", {"action": "drop", "in-interface-list": ROBOT_LIST}),
    ],
}
IPV6_DESIGN = {
    "forward": [
        ("no IPv6 out", {"action": "drop", "in-interface-list": ROBOT_LIST}),
        ("no IPv6 in", {"action": "drop", "out-interface-list": ROBOT_LIST}),
    ],
    "input": [("no IPv6 to the router", {"action": "drop", "in-interface-list": ROBOT_LIST})],
}
SERVICES_OFF = ("telnet", "ftp", "www", "www-ssl", "api", "api-ssl", "reverse-proxy")
SERVICES_PRIVATE = ("ssh", "winbox")


def commands(tag):
    """Every command sent: prints and :puts only."""
    def counters(fam):
        p = f"/{fam} firewall filter"
        return (f':foreach i in=[{p} find where comment~"^{tag}"] do={{:put ("{fam}|" . [{p} get $i chain] . "|" . '
                f'[{p} get $i action] . "|" . [{p} get $i packets] . "|" . [{p} get $i comment])}}')
    return {
        "members": "/interface list member print terse",
        "ipv4": "/ip firewall filter print terse",
        "ipv6": "/ipv6 firewall filter print terse",
        "services": "/ip service print terse",
        "addresses": "/ip address print terse",
        "operators": f"/ip firewall address-list print terse where list={OPERATORS}",
        "counters4": counters("ip"),
        "counters6": counters("ipv6"),
        "uptime": ":put [/system resource get uptime]",
    }


_KEY = re.compile(r"\s(?=[\w.-]+=)")


def parse_terse(text):
    """RouterOS 'print terse' lines -> [{'_n': index, '_flags': 'XD..', key: value}]. Values may hold spaces."""
    rows = []
    for ln in text.splitlines():
        m = re.match(r"\s*(\d+)\s+([A-Z* ]*?)\s*(?=[\w.-]+=)", ln)
        if not m:
            continue
        row = {"_n": int(m.group(1)), "_flags": m.group(2).replace(" ", "")}
        for part in _KEY.split(ln[m.end():]):
            if "=" in part:
                k, v = part.split("=", 1)
                row[k] = v.strip()
        rows.append(row)
    return rows


def _attrs(r):
    return {k: v for k, v in r.items() if not k.startswith("_") and k not in ("chain", "comment")}


def check_chains(rules, design, tag, family):
    """The designed rules exist, are enabled, exact, and are the first rules of their chains, in order."""
    fails = []
    static = [r for r in rules if "D" not in r["_flags"]]          # dynamic rules are the router's own (fasttrack dummy)
    designed = {f"{tag}: {c}" for chain in design.values() for c, _ in chain}
    for chain, want in design.items():
        have = [r for r in static if r.get("chain") == chain]
        for i, (c, attrs) in enumerate(want):
            comment = f"{tag}: {c}"
            match = [r for r in have if r.get("comment") == comment]
            if not match:
                fails.append(f"{family} {chain} rule '{c}' missing"); continue
            r = match[0]
            if len(match) > 1:
                fails.append(f"{family} {chain} rule '{c}' appears {len(match)} times")
            if "X" in r["_flags"]:
                fails.append(f"{family} {chain} rule '{c}' disabled")
            if "I" in r["_flags"]:
                fails.append(f"{family} {chain} rule '{c}' invalid")
            if _attrs(r) != attrs:
                fails.append(f"{family} {chain} rule '{c}' changed ({_diff(attrs, _attrs(r))})")
            pos = have.index(r)
            if pos != i:
                above = have[i].get("comment", "(no comment)") if i < len(have) else "?"
                fails.append(f"{family} {chain} rule '{c}' at position {pos + 1}, designed {i + 1} (there: {above})")
    for r in static:
        cm = r.get("comment", "")
        if cm.startswith(tag) and cm not in designed:
            what = "online window open" if "online window" in cm.lower() else "extra robot-network rule"
            fails.append(f"{what}: {family} {r.get('chain')} '{cm[len(tag):].lstrip(': ')}'")
    return fails


def _diff(want, have):
    out = [f"{k}={have.get(k, '-')} not {v}" for k, v in want.items() if have.get(k) != v]
    out += [f"+{k}={v}" for k, v in have.items() if k not in want]
    return ", ".join(out)


def _private_nets(addresses, lan_ifs):
    return sorted({str(ipaddress.ip_interface(a["address"]).network) for a in addresses
                   if a.get("interface") in lan_ifs and "D" not in a["_flags"] and "X" not in a["_flags"]})


def evaluate(out, env):
    """The router's output (a dict of commands() keys -> text) and the settings -> the result dict."""
    tag = env.get("ROUTER_COMMENT_TAG") or env.get("ROBOT_SSID") or ""
    fails, details = [], {}
    if not tag:
        fails.append("no comment tag (ROUTER_COMMENT_TAG or ROBOT_SSID in real/.env)")
    members = [m for m in parse_terse(out["members"]) if "X" not in m["_flags"]]
    addresses = parse_terse(out["addresses"])
    robot_ip = env.get("ROUTER_ON_ROBOT_NET", "").split("/")[0]
    robot_if = next((a.get("interface") for a in addresses if a.get("address", "").split("/")[0] == robot_ip), None)
    if not robot_if:
        fails.append("the router holds no address ROUTER_ON_ROBOT_NET")
    robot_members = sorted(m.get("interface") for m in members if m.get("list") == ROBOT_LIST)
    details["robot_list"] = robot_members
    if robot_members != [robot_if]:
        fails.append(f"Robot list holds {robot_members or 'nothing'}, designed only the robot bridge")
    lan = sorted(m.get("interface") for m in members if m.get("list") == "LAN")
    details["lan_list"] = lan
    guest = env.get("ROUTER_GUEST_IF")
    guests = [i for i in lan if (i == guest if guest else "guest" in i.lower())]
    if guests:
        fails.append(f"guest network in the LAN list ({', '.join(guests)})")
    if robot_if in lan:
        fails.append("robot network in the LAN list")

    if tag:
        fails += check_chains(parse_terse(out["ipv4"]), IPV4_DESIGN, tag, "IPv4")
        fails += check_chains(parse_terse(out["ipv6"]), IPV6_DESIGN, tag, "IPv6")

    ops = parse_terse(out["operators"])
    details["operators"] = len(ops)
    for o in ops:
        net = ipaddress.ip_network(o.get("address", "0.0.0.0/0"), strict=False)
        if net.num_addresses != 1:
            fails.append(f"operators list holds a range ({net.prefixlen}-bit prefix), designed single hosts")

    private = _private_nets(addresses, set(lan) - set(guests) - {robot_if})
    services = {s["name"]: s for s in parse_terse(out["services"]) if "D" not in s["_flags"] and "name" in s}
    details["services"] = {n: ("off" if "X" in s["_flags"] else "on") for n, s in services.items()}
    for n in SERVICES_OFF:
        if n in services and "X" not in services[n]["_flags"]:
            fails.append(f"service {n} enabled")
    for n in SERVICES_PRIVATE:
        s = services.get(n)
        if not s or "X" in s["_flags"]:
            continue                                               # off is closed too
        frm = sorted(x for x in s.get("available-from", "").split(",") if x)
        if not frm or frm != private:
            fails.append(f"service {n} reachable from {'anywhere' if not frm else 'more than the private network'}")

    counters = {}
    for ln in (out.get("counters4", "") + "\n" + out.get("counters6", "")).splitlines():
        p = ln.split("|", 4)
        if len(p) == 5 and p[3].isdigit():
            counters[f"{p[0]} {p[1]} {p[4][len(tag):].lstrip(': ')}"] = {"action": p[2], "packets": int(p[3])}
    details["counters"] = counters
    dropped = {k: v["packets"] for k, v in counters.items() if v["action"] == "drop"}
    out_n = sum(v for k, v in dropped.items() if k.startswith("ip forward") and "nowhere else" in k)
    router_n = sum(v for k, v in dropped.items() if k.startswith("ip input"))
    in_n = sum(v for k, v in dropped.items() if k.startswith("ip forward") and "into the robot" in k)
    v6_n = sum(v for k, v in dropped.items() if k.startswith("ipv6"))
    uptime = out.get("uptime", "").strip()
    details["uptime"] = uptime
    drops = (f"dropped since boot ({uptime or '?'} up): {out_n} going out, {router_n} at the router, "
             f"{in_n} coming in, {v6_n} IPv6")
    details["dropped"] = {"out": out_n, "router": router_n, "in": in_n, "ipv6": v6_n}
    summary = (f"FAILED: {'; '.join(fails)}" if fails else f"passed: robot network closed as designed; {drops}")
    return {"summary": summary, "passed": not fails, "failures": fails, "details": details}


class HostKeyRefused(Exception):
    """The router's SSH host key is not the one recorded: the password is not sent."""


def known_hosts_path(env):
    p = Path(env.get("ROUTER_KNOWN_HOSTS") or "real/local/router_known_hosts")
    return p if p.is_absolute() else HK / p


def ssh_client(env):
    """An SSH client that trusts only the router's recorded host key (ROUTER_KNOWN_HOSTS): a missing file, an unknown
    host or a different key refuses before the password is sent, so an impersonated router learns nothing."""
    kh = known_hosts_path(env)
    if not kh.is_file():
        raise HostKeyRefused(f"router host key not recorded: {kh} missing")
    import paramiko                                                # only for the live read
    c = paramiko.SSHClient()
    c.load_host_keys(str(kh))
    c.set_missing_host_key_policy(paramiko.RejectPolicy())
    return c


def connect(env, port=22):
    c = ssh_client(env)                                            # refuses first when no key is recorded
    import paramiko
    kh = known_hosts_path(env)
    try:
        c.connect(env["ROUTER_HOST"], port=port, username=env["ROUTER_USER"], password=env["ROUTER_PASS"], timeout=10,
                  look_for_keys=False, allow_agent=False)
    except paramiko.BadHostKeyException as e:
        c.close()
        raise HostKeyRefused(f"router host key differs from the one recorded in {kh}") from e
    except paramiko.SSHException as e:
        c.close()
        if "not found in known_hosts" in str(e):
            raise HostKeyRefused(f"router host key not recorded for this address in {kh}") from e
        raise
    return c


def read_router(env):
    tag = env.get("ROUTER_COMMENT_TAG") or env.get("ROBOT_SSID") or ""
    c = connect(env)
    try:
        out = {}
        for k, cmd in commands(tag).items():
            _, so, _ = c.exec_command(cmd, timeout=30)
            out[k] = so.read().decode(errors="replace")
        return out
    finally:
        c.close()


def write_result(res, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(res, indent=1) + "\n")
    os.replace(tmp, path)


def main(argv):
    env = read_env(ENV)
    path = Path(env.get("ISOLATION_JSON") or "real/local/isolation.json")
    path = path if path.is_absolute() else HK / path
    now = datetime.datetime.now().astimezone().isoformat(timespec="seconds")
    try:
        out = read_router(env)
    except HostKeyRefused as e:                                    # refused before the password was sent
        res = {"summary": f"FAILED: router not trusted: {e}", "passed": False,
               "failures": [f"router not trusted: {e}"], "details": {}}
    except Exception as e:                                         # unreadable is not passed
        res = {"summary": f"FAILED: router not read ({type(e).__name__})", "passed": False,
               "failures": [f"router not read: {e}"], "details": {}}
    else:
        if "--dump" in argv:
            d = Path(argv[argv.index("--dump") + 1]); d.mkdir(parents=True, exist_ok=True)
            for k, v in out.items():
                (d / f"{k}.txt").write_text(v)
        res = evaluate(out, env)
    res["checked_at"] = now
    write_result(res, path)
    print(res["summary"])
    return 0 if res["passed"] else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
