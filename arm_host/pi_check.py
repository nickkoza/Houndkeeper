"""The arm host's checks on the real Pi ([JIY], decision [Uyb]): arm_host/pi_check.sh runs both halves from the laptop
and keeps the record under real/local/.
  on the Pi, as root, from the laptop's copy (pi_check.sh ships it; decision [ZyY]):  ... pi_check on-pi [--kill-test]
      [--only UYK,27n]
  on the laptop:  python3 -m arm_host.pi_check from-laptop --host <pi> --on-pi <file> [--expect-unreachable]
  two cards from one commit:  python3 -m arm_host.pi_check compare <record a> <record b>
Each check is a judge over what a command printed, so the laptop's tests feed them recorded output. A result is pass,
fail, "not built" (what it needs is not built yet), "manual" (a step for a person, said in the detail) or "measured"
(a number, not judged). Standard library only."""
import argparse, hashlib, ipaddress, json, os, pwd, re, signal, socket, stat, subprocess, sys, time, urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from arm_host import packages, release

ROOT = Path("/opt/houndkeeper")
CONFIG = Path("/etc/houndkeeper/host.json")
TIMESYNCD_CONF = Path("/etc/systemd/timesyncd.conf.d/10-houndkeeper.conf")
API_PORT, REPORT_PORT = 47100, 47110                       # hostd's defaults; host.json may say otherwise
CLOCK_TOLERANCE_S = 0.010                                  # decision [LSY]
REPORT_FIELDS = {"time", "release", "services", "cpu_temp_c", "throttle", "arm", "speaker"}
DEFAULT_ACCOUNTS = ("pi",)                                 # Raspberry Pi OS's old default user
SERVICE_USER, ADMIN_USER = "hkd", "hk"                     # decision [ZyY]: the services' account; the admin SSH login
NO_LOGIN_SHELLS = ("/usr/sbin/nologin", "/sbin/nologin", "/bin/false", "/usr/bin/false")
ROOTS_PATHS = ("/opt", "/opt/houndkeeper", "/etc/houndkeeper")   # walked: what root runs from or reads
UDP_ALLOWED = {68, 546, 5353}                              # DHCP and mDNS
EPHEMERAL_FROM = 32768                                     # Linux's ip_local_port_range: unbound sending sockets
MEDIA = {".jpg", ".jpeg", ".png", ".bmp", ".yuv", ".rgb", ".raw", ".h264", ".h265", ".hevc", ".mjpeg", ".mjpg", ".mp4",
         ".mkv", ".avi", ".mov", ".webm", ".ts", ".wav", ".pcm", ".flac", ".ogg", ".opus", ".mp3", ".m4a"}
MEDIA_ROOTS = ("/home", "/root", "/opt/houndkeeper", "/tmp", "/var/tmp", "/var/log", "/srv", "/run/houndkeeper",
               "/run/user")
SCAN_PORTS = list(range(1, 1025)) + [1883, 3000, 5000, 5004, 5005, 8000, 8080, 8443, 8554, 8888, 9000, 9090] \
    + list(range(47100, 47112))


def result(id_, check, ok, result=None, detail=""):
    return {"id": id_, "check": check, "result": result or ("pass" if ok else "fail"), "detail": str(detail)}


def verdict(results):
    return 1 if any(r["result"] == "fail" for r in results) else 0


def _run(*args, timeout=10):
    try: return subprocess.run(list(args), capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as e: return subprocess.CompletedProcess(args, 127, "", str(e))


# ---- [UYK] rebuilt from the repository ------------------------------------------------------------------------

def parse_dpkg(text):
    """dpkg-query -W -f '${Package} ${Version}\\n'"""
    return dict(l.split(" ", 1) for l in text.splitlines() if " " in l)


def judge_packages(installed, lk):
    """every locked package installed at its locked version, and every package first boot asks for installed"""
    bad = [f"{p} {installed.get(p, 'missing')} (locked {v})" for p, v in sorted(packages.versions(lk).items())
           if installed.get(p) != v]
    bad += [f"{p} missing" for p in lk.get("packages", "").split() if p not in installed]
    return result("UYK", "packages are the locked versions", not bad, detail="; ".join(bad) or f"{len(lk['debs'])} locked")


def fingerprint(installed, lk, manifest_text):
    """the software a host runs: the locked packages as installed, and its release's manifest"""
    lines = [f"{p} {installed.get(p, 'missing')}" for p in sorted(packages.versions(lk))]
    return hashlib.sha256(("\n".join(lines) + "\n" + manifest_text).encode()).hexdigest()


def compare(a, b):
    """two records (two cards from one commit) run the same software"""
    f = lambda rec: next((r["detail"] for r in rec["results"] if r["id"] == "UYK" and r["check"] == "fingerprint"), None)
    fa, fb = f(a), f(b)
    return result("UYK", "two builds run the same software", fa is not None and fa == fb, detail=f"{fa} / {fb}")


# ---- [BRx] ready from power-up, disarmed ----------------------------------------------------------------------

def judge_services(states):
    """{name: (is-enabled, is-active)}: each enabled (started at power-up) and active"""
    bad = [f"{n} {e}/{a}" for n, (e, a) in sorted(states.items()) if (e, a) != ("enabled", "active")]
    return result("BRx", "services enabled and running", bool(states) and not bad, detail="; ".join(bad) or ", ".join(states))


def judge_restart(old_pid, new_pid, active, waited_s):
    ok = active == "active" and bool(new_pid) and new_pid != old_pid
    return result("BRx", "a killed daemon is restarted", ok, detail=f"pid {old_pid} -> {new_pid}, {active}, {waited_s:.1f} s")


def judge_disarmed(report):
    if "armed" not in report:
        return result("BRx", "the arm disarmed", None, "not built", "the arm adapter reports no armed state yet")
    return result("BRx", "the arm disarmed", report["armed"] is False, detail=f"armed={report['armed']}")


# ---- [aVJ] one clock -------------------------------------------------------------------------------------------

def judge_clock(synced, conf):
    src = next((l.split("=", 1)[1].strip() for l in (conf or "").splitlines() if l.startswith("NTP=")), "")
    fallback = next((l.split("=", 1)[1].strip() for l in (conf or "").splitlines() if l.startswith("FallbackNTP=")), None)
    ok = (synced or "").strip() == "yes" and bool(src) and fallback == ""
    return result("aVJ", "synchronised to the card's time source", ok,
                  detail=f"synchronised={(synced or '').strip() or '?'}, source={'set' if src else 'none'}, "
                         f"fallback={'none' if fallback == '' else 'default (the internet)'}")


def clock_offset(samples):
    """(laptop send, Pi time, laptop receive): the exchange with the shortest round trip; the Pi's time against its
    middle, give or take half the round trip"""
    t0, pi, t1 = min(samples, key=lambda s: s[2] - s[0])
    return pi - (t0 + t1) / 2, (t1 - t0) / 2


def judge_offset(samples, tol=CLOCK_TOLERANCE_S):
    if not samples: return result("aVJ", "offset to the laptop", False, detail="no answer from the host")
    off, unc = clock_offset(samples)
    return result("aVJ", "offset to the laptop", abs(off) <= tol + unc,
                  detail=f"{off * 1000:+.1f} ms +/- {unc * 1000:.1f} ms ({len(samples)} exchanges)")


# ---- [4WJ] the report ------------------------------------------------------------------------------------------

def judge_reports(received, pi_ip, window_s, slack=0.15):
    """(seconds into the window, datagram, sender): from the Pi, every gap within a second (and a little scheduling
    slack) from the window's start to its end, every field in every report"""
    ts, missing = [], set()
    for t, data, addr in received:
        if addr[0] != pi_ip: continue
        try: r = json.loads(data)
        except ValueError: missing.add("valid JSON"); continue
        ts.append(t); missing |= REPORT_FIELDS - set(r)
    edges = [0.0] + sorted(ts) + [window_s]
    gap = max(b - a for a, b in zip(edges, edges[1:]))
    ok = bool(ts) and not missing and gap <= 1.0 + slack
    return result("4WJ", "reports reach the laptop at least once a second", ok,
                  detail=f"{len(ts)} in {window_s:.0f} s, longest gap {gap:.2f} s" + (f", missing {sorted(missing)}" if missing else ""))


# ---- [27n] power cut -------------------------------------------------------------------------------------------

def judge_release_state(root):
    root = Path(root); cur, good = root / "current", root / "known-good"
    if not cur.is_symlink(): return result("27n", "a whole known-good release current", False, detail="no current release")
    if not good.is_symlink(): return result("27n", "a whole known-good release current", False, detail="no known-good link")
    if cur.resolve() != good.resolve():
        return result("27n", "a whole known-good release current", False,
                      detail=f"current {cur.resolve().name} is not the known-good {good.resolve().name}")
    return result("27n", "a whole known-good release current", release.whole(cur.resolve()),
                  detail=f"{cur.resolve().name}" + ("" if release.whole(cur.resolve()) else ": differs from its manifest"))


def judge_cmdline(cmdline):
    return result("27n", "the root file system repaired at boot", "fsck.repair=yes" in cmdline.split(),
                  detail="fsck.repair=yes" if "fsck.repair=yes" in cmdline.split() else "no fsck.repair=yes")


# ---- [CIu] locked down -----------------------------------------------------------------------------------------

def judge_sshd(text):
    """sshd -T: the settings sshd actually runs with"""
    got = {k.lower(): v.strip() for k, _, v in (l.partition(" ") for l in text.splitlines())}
    want = {"passwordauthentication": "no", "kbdinteractiveauthentication": "no", "permitrootlogin": "no"}
    bad = [f"{k} {got.get(k, 'unset')}" for k, v in want.items() if got.get(k) != v]
    return result("CIu", "SSH by key only", not bad, detail="; ".join(bad) or "keys only, no root")


def judge_shadow(text):
    """an account can log in with a password if its field is empty or a hash (not '!' or '*' locked)"""
    bad = [l.split(":")[0] for l in text.splitlines() if l.count(":") >= 1 and not l.split(":")[1].startswith(("!", "*"))]
    return result("CIu", "no password logins", not bad, detail=", ".join(bad) or "every account locked")


def judge_users(passwd_text):
    bad = [l.split(":")[0] for l in passwd_text.splitlines() if l.split(":")[0] in DEFAULT_ACCOUNTS]
    return result("CIu", "no default accounts", not bad, detail=", ".join(bad) or "none")


def judge_service_account(name, passwd_text, sudo_l_text, has_keys):
    """the services' account: no login shell, `sudo -l -U <name>` says it may not run sudo, no SSH key"""
    row = next((l.split(":") for l in passwd_text.splitlines() if l.split(":")[0] == name), None)
    if row is None or len(row) < 7: return result("CIu", "the services' account has no way up", False, detail=f"no account {name}")
    bad = ([f"login shell {row[6]}"] if row[6] not in NO_LOGIN_SHELLS else []) \
        + ([] if "is not allowed to run sudo" in sudo_l_text else ["sudo: " + (sudo_l_text.strip().splitlines() or ["no answer"])[-1].strip()]) \
        + (["an SSH key"] if has_keys else [])
    return result("CIu", "the services' account has no way up", not bad, detail="; ".join(bad) or f"{name}: no login, no sudo, no key")


def judge_unit_users(users, service_user=SERVICE_USER):
    """{unit: systemctl show -p User --value}: each of the profile's services runs as the services' account (an empty
    User is root)"""
    bad = [f"{u} as {who or 'root'}" for u, who in sorted(users.items()) if who != service_user]
    return result("CIu", "the services run as their own account", bool(users) and not bad,
                  detail="; ".join(bad) or f"{', '.join(sorted(users))} as {service_user}")


def judge_not_writable(entries, uid, gids):
    """(path, owner uid, group gid, st_mode): every file and directory root runs or reads is owned by root and
    writable by no other account (the services' least of all); links are judged by their directory"""
    bad = []
    for path, o, g, mode in entries:
        if stat.S_ISLNK(mode): continue
        why = ("owned by uid %d" % o if o != 0 else "") or ("writable by the services' account" if
              (mode & 0o002) or (g in gids and mode & 0o020) else "")
        if why: bad.append(f"{path} {why}")
    return result("CIu", "root's code and settings writable only by root", bool(entries) and not bad,
                  detail="; ".join(bad[:20]) + (" ..." if len(bad) > 20 else "") or f"{len(entries)} paths, all root's")


def roots_entries(paths=ROOTS_PATHS, units_glob="/etc/systemd/system/hk-*.service"):
    """lstat of each path, everything under each directory (links not followed) and the host's unit files"""
    import glob
    seen = []
    def add(x):
        try: s = os.lstat(x); seen.append((x, s.st_uid, s.st_gid, s.st_mode))
        except OSError: pass
    for r in paths:
        add(r)
        if r == "/opt": continue                                  # the directory itself, not every package in it
        for top, dirs, files in os.walk(r):
            for n in dirs + files: add(os.path.join(top, n))
    for u in sorted(glob.glob(units_glob)): add(u)
    return seen


def _ss(line):
    """(state, (local host, port), (peer host, port)) from one ss line, with or without its Netid column"""
    c = line.split()
    if c and c[0] in ("tcp", "udp", "raw", "u_str", "u_dgr"): c = c[1:]
    if len(c) < 5: return None
    def hp(s):
        h, _, p = s.rpartition(":"); h = h.strip("[]").split("%")[0]
        if h.startswith("::ffff:"): h = h[7:]
        return (h, int(p) if p.isdigit() else None)
    return c[0], hp(c[3]), hp(c[4])


def _loopback(h):
    try: return ipaddress.ip_address(h).is_loopback
    except ValueError: return False


def judge_listening(tcp, udp):
    """ss -Hlnt / ss -Hlnu: on the network, only SSH and the API (which answers only the laptop) take connections"""
    bad = []
    for proto, text, allowed in (("tcp", tcp, {22, API_PORT}), ("udp", udp, UDP_ALLOWED)):
        for line in text.splitlines():
            s = _ss(line)
            if not s: continue
            (h, p) = s[1]
            if _loopback(h) or p in allowed or (proto == "udp" and p and p >= EPHEMERAL_FROM): continue
            bad.append(f"{proto} {h}:{p}")
    return result("CIu", "only SSH and the API listen", not bad, detail="; ".join(bad) or "ssh, api")


def judge_open_ports(open_ports, expect_unreachable=False):
    if expect_unreachable:
        return result("CIu", "nothing answers from this network", not open_ports, detail=open_ports or "nothing")
    ok = bool(open_ports) and set(open_ports) <= {22, API_PORT}
    return result("CIu", "only SSH and the API answer the laptop", ok, detail=open_ports or "nothing answered")


def judge_password_refused(rc, stderr):
    m = re.search(r"Permission denied \(([^)]*)\)", stderr)
    methods = set(m.group(1).split(",")) if m else set()
    ok = rc != 0 and bool(methods) and not methods & {"password", "keyboard-interactive"}
    return result("CIu", "a password login refused", ok, detail=stderr.strip().splitlines()[-1] if stderr.strip() else f"exit {rc}")


def scan_ports(host, ports=SCAN_PORTS, timeout=0.5):
    def one(p):
        try:
            with socket.create_connection((host, p), timeout=timeout): return p
        except OSError: return None
    with ThreadPoolExecutor(64) as ex: return sorted(p for p in ex.map(one, ports) if p)


# ---- [Ot6] wrist video stays in the house ----------------------------------------------------------------------

def find_media(roots=MEDIA_ROOTS):
    found = []
    for r in roots:
        for top, dirs, files in os.walk(r):
            found += [os.path.join(top, f) for f in files if os.path.splitext(f)[1].lower() in MEDIA
                      and not os.path.islink(os.path.join(top, f))]
    return found


def judge_media(found):
    return result("Ot6", "no footage stored", not found, detail="; ".join(found[:20]) + (" ..." if len(found) > 20 else "") or "none")


def judge_peers(ss_text, allowed):
    """ss -Htuna: every connection open or being opened is with the laptop, the time source or the host itself"""
    bad = []
    for line in ss_text.splitlines():
        s = _ss(line)
        if not s or s[0] not in ("ESTAB", "SYN-SENT", "SYN-RECV"): continue
        h = s[2][0]
        if not (_loopback(h) or h in allowed): bad.append(f"{s[0]} {h}:{s[2][1]}")
    return result("Ot6", "talks only to the laptop and its time source", not bad, detail="; ".join(bad) or "only those")


def judge_wrist_stream(cfg, observed):
    if observed is None:
        return result("Ot6", "the wrist stream goes only to the laptop", None, "not built", "no wrist camera service yet")
    return judge_peers(observed, {cfg.get("laptop_ip")})


# ---- the two halves --------------------------------------------------------------------------------------------

def _host_cfg():
    try: cfg = json.loads(CONFIG.read_text())
    except (OSError, ValueError): cfg = {}
    try: conf = TIMESYNCD_CONF.read_text()
    except OSError: conf = None
    src = next((l.split("=", 1)[1].strip() for l in (conf or "").splitlines() if l.startswith("NTP=")), "")
    return cfg, conf, src


def _health(host, port, timeout=2):
    with urllib.request.urlopen(f"http://{host}:{port}/health", timeout=timeout) as r: return json.load(r)


def on_pi(only, kill_test):
    cfg, conf, time_src = _host_cfg(); port = cfg.get("http_port", API_PORT); out = []
    want = lambda i: not only or i in only
    cur = ROOT / "current"
    if want("UYK"):
        lk = json.loads(packages.LOCK.read_text())
        inst = parse_dpkg(_run("dpkg-query", "-W", "-f", "${Package} ${Version}\n").stdout)
        try: manifest = (cur / "MANIFEST.sha256").read_text()
        except OSError: manifest = ""
        out += [judge_packages(inst, lk), result("UYK", "fingerprint", bool(manifest), detail=fingerprint(inst, lk, manifest)),
                result("UYK", "release whole", release.whole(cur.resolve()), detail=cur.resolve().name)]
    if want("BRx"):
        names = list(cfg.get("services") or ["hk-hostd"]) + ["hk-release-recover"]
        out.append(judge_services({n: (_run("systemctl", "is-enabled", n).stdout.strip(),
                                       _run("systemctl", "is-active", n).stdout.strip()) for n in names}))
        us = _run("systemctl", "show", "-p", "ActiveEnterTimestampMonotonic", "--value", "hk-hostd").stdout.strip()
        out.append(result("BRx", "power-up to the daemon running", None, "measured",
                          f"{int(us) / 1e6:.1f} s after the kernel started (target under 60 s)" if us.isdigit() and int(us) else "not known"))
        try: out.append(judge_disarmed(_health("127.0.0.1", port)))
        except OSError as e: out.append(result("BRx", "the arm disarmed", False, detail=f"no report: {e}"))
        if kill_test:
            old = int(_run("systemctl", "show", "-p", "MainPID", "--value", "hk-hostd").stdout.strip() or 0)
            if old: os.kill(old, signal.SIGKILL)
            t0 = time.monotonic(); new, active = 0, ""
            while time.monotonic() - t0 < 15:
                time.sleep(0.5); active = _run("systemctl", "is-active", "hk-hostd").stdout.strip()
                new = int(_run("systemctl", "show", "-p", "MainPID", "--value", "hk-hostd").stdout.strip() or 0)
                if active == "active" and new and new != old: break
            out.append(judge_restart(old, new, active, time.monotonic() - t0))
            out.append(result("BRx", "the arm disarmed while its service is down", None, "not built",
                              "the arm adapter's service is not built yet"))
    if want("aVJ"):
        out.append(judge_clock(_run("timedatectl", "show", "-p", "NTPSynchronized", "--value").stdout, conf))
    if want("4WJ"):
        out.append(result("4WJ", "under-voltage raised", None, "manual",
                          "power the Pi from a bench supply, lower it until the Pi flags under-voltage; the report's "
                          "throttle.under_voltage_now turns true and the eyes go dark"))
    if want("27n"):
        out.append(judge_release_state(ROOT))
        try: out.append(judge_cmdline(Path("/proc/cmdline").read_text()))
        except OSError as e: out.append(result("27n", "the root file system repaired at boot", False, detail=e))
        out.append(result("27n", "read-only root", None, "not built", "decision [UcX] waits on the overlayroot package"))
        out.append(result("27n", "20 power cuts", None, "manual",
                          "cut the power at random points, some mid-update (deploy.sh), and run pi_check.sh <host> --only 27n "
                          "after each boot: 20 passes in a row"))
    if want("CIu"):
        out.append(judge_sshd(_run("sshd", "-T").stdout))
        for name, judge in (("/etc/shadow", judge_shadow), ("/etc/passwd", judge_users)):
            try: out.append(judge(Path(name).read_text()))
            except OSError as e: out.append(result("CIu", name, False, detail=e))
        out.append(judge_listening(_run("ss", "-Hlnt").stdout, _run("ss", "-Hlnu").stdout))
        try:
            pw = pwd.getpwnam(SERVICE_USER); gids = set(os.getgrouplist(SERVICE_USER, pw.pw_gid))
            keys = any(os.path.exists(os.path.join(pw.pw_dir, ".ssh", f)) for f in ("authorized_keys", "authorized_keys2"))
            out.append(judge_service_account(SERVICE_USER, Path("/etc/passwd").read_text(),
                                              _run("sudo", "-l", "-U", SERVICE_USER).stdout, keys))
            out.append(judge_not_writable(roots_entries(), pw.pw_uid, gids))
        except KeyError:
            out.append(result("CIu", "the services' account has no way up", False, detail=f"no account {SERVICE_USER}"))
        out.append(judge_unit_users({n: _run("systemctl", "show", "-p", "User", "--value", n).stdout.strip()
                                     for n in cfg.get("services") or ["hk-hostd"]}))
    if want("Ot6"):
        out.append(judge_media(find_media()))
        out.append(judge_peers(_run("ss", "-Htuna").stdout, {cfg.get("laptop_ip"), time_src} - {None, ""}))
        out.append(judge_wrist_stream(cfg, None))
    host = {k: cfg.get(k) for k in ("laptop_ip", "http_port", "report_port", "profile")}
    return {"host": dict(host, release=cur.resolve().name if cur.is_symlink() else None, time_source=bool(time_src)),
            "results": out}


def from_laptop(host, pi_record, only, expect_unreachable, window_s=5.0):
    h = pi_record.get("host", {}); port = h.get("http_port") or API_PORT; out = []
    want = lambda i: not only or i in only
    try: ip = socket.gethostbyname(host)
    except OSError as e: return [result("CIu", "the host found", False, detail=e)]
    if expect_unreachable:
        out.append(judge_open_ports(scan_ports(ip), expect_unreachable=True)); return out
    if want("CIu"):
        out.append(judge_open_ports(scan_ports(ip)))
        r = _run("ssh", "-o", "BatchMode=yes", "-o", "PubkeyAuthentication=no", "-o", "ConnectTimeout=5",
                 "-o", "PreferredAuthentications=password,keyboard-interactive", f"hk@{host}", "true", timeout=20)
        out.append(judge_password_refused(r.returncode, r.stderr))
        out.append(result("CIu", "nothing answers from the guest network", None, "manual",
                          "from a machine on the guest network: arm_host/pi_check.sh <pi address> --expect-unreachable"))
    if want("aVJ"):
        samples = []
        for _ in range(20):
            try: t0 = time.time(); r = _health(ip, port); t1 = time.time(); samples.append((t0, r["time"], t1))
            except (OSError, ValueError, KeyError): pass
            time.sleep(0.05)
        out.append(judge_offset(samples))
    if want("4WJ"):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); got = []
        try:
            s.bind(("", h.get("report_port") or REPORT_PORT)); s.settimeout(0.2); t0 = time.monotonic()
            while time.monotonic() - t0 < window_s:
                try: data, addr = s.recvfrom(65536); got.append((time.monotonic() - t0, data, addr))
                except socket.timeout: pass
            out.append(judge_reports(got, ip, window_s))
        except OSError as e:
            out.append(result("4WJ", "reports reach the laptop at least once a second", False,
                              detail=f"could not listen ({e}): is DimOS holding the report port?"))
        finally: s.close()
        out.append(result("4WJ", "the report shown in DimOS", None, "not built", "DimOS does not read the host's report yet"))
    return out


def _print(results):
    for r in results: print(f"[{r['id']}] {r['result']:<9} {r['check']}: {r['detail']}", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(); sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("on-pi"); p.add_argument("--kill-test", action="store_true"); p.add_argument("--only", default="")
    l = sub.add_parser("from-laptop"); l.add_argument("--host", required=True); l.add_argument("--on-pi", required=True)
    l.add_argument("--only", default=""); l.add_argument("--expect-unreachable", action="store_true")
    c = sub.add_parser("compare"); c.add_argument("a"); c.add_argument("b")
    a = ap.parse_args(); only = set(filter(None, getattr(a, "only", "").split(",")))
    if a.cmd == "on-pi":
        rec = on_pi(only, a.kill_test); print(json.dumps(rec, indent=1)); sys.exit(0)
    if a.cmd == "compare":
        r = compare(json.loads(Path(a.a).read_text()), json.loads(Path(a.b).read_text())); _print([r])
        sys.exit(verdict([r]))
    try: pi = json.loads(Path(a.on_pi).read_text())
    except (OSError, ValueError): pi = {"results": []}
    if not pi["results"] and not a.expect_unreachable:
        pi["results"].append(result("JIY", "the Pi's half", False, detail="no results from the Pi (SSH, or the release)"))
    rec = {"date": time.strftime("%Y-%m-%d %H:%M:%S"), "host": pi.get("host", {}),
           "results": pi["results"] + from_laptop(a.host, pi, only, a.expect_unreachable)}
    print(json.dumps(rec, indent=1)); _print(rec["results"]); sys.exit(verdict(rec["results"]))


if __name__ == "__main__":
    main()
