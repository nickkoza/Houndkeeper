"""Starts the always-on pair the project pages need with nothing else running ([eTc]): the cockpit relay and the
project service (houndkeeper.project). The systemd user units real/bringup/hk-relay.service and hk-project.service
run it; by hand:

    python -m houndkeeper.project_stack relay      # the relay, on the laptop's Tailscale address
    python -m houndkeeper.project_stack project    # the project service, registered with it as "project"

--loopback puts both on 127.0.0.1 (plain HTTP, no auth: tests and trying it out). Otherwise the relay binds the
laptop's Tailscale address with the certificate and auth file in real/local/ ([rhP]), and the project service reaches
it by the certificate's name. Any other address is refused before anything starts ([Dlx], [0Us]): never a house
address, never every address. The project's relay key comes from the auth file at run time (robots.project), into the
environment, never onto the command line."""
from __future__ import annotations

import argparse, ipaddress, json, os, shutil, subprocess, sys
from pathlib import Path

from houndkeeper import workspace

HK = Path(__file__).resolve().parents[2]
DIMOS = workspace.dimos()
TLS = HK / "real" / "local" / "tls"
AUTH = HK / "real" / "local" / "relay-auth.json"
PORT = 7780
ROBOT_ID = "project"
TAILNET = (ipaddress.ip_network("100.64.0.0/10"), ipaddress.ip_network("fd7a:115c:a1e0::/48"))   # Tailscale's ranges


def allowed_host(host: str) -> bool:
    """loopback, or an address in Tailscale's ranges; nothing else (a house address, 0.0.0.0, ::, a name)"""
    if host == "localhost": return True
    try: a = ipaddress.ip_address(host)
    except ValueError: return False
    return a.is_loopback or any(a in n for n in TAILNET)


def tailnet() -> tuple[str, str]:
    """the laptop's Tailscale IPv4 address and its name on the tailnet (the certificate's), from `tailscale status`"""
    st = json.loads(subprocess.run(["tailscale", "status", "--json"], check=True, capture_output=True, text=True).stdout)
    me = st["Self"]; ip = next(a for a in me["TailscaleIPs"] if ":" not in a)
    return ip, me["DNSName"].rstrip(".")


def relay_argv(host: str, port: int = PORT, deno: str = "deno", tls: Path = TLS, auth: Path = AUTH) -> list[str]:
    if not allowed_host(host): raise SystemExit(f"refused: {host} is neither loopback nor a Tailscale address")
    web = DIMOS / "web"
    argv = [deno, "run", "-A", str(web / "relay" / "main.ts"), "--host", host, "--port", str(port),
            "--cockpit-dir", str(web / "cockpit" / "dist"), "--sdk-dir", str(web / "sdk" / "dist"),
            "--extensions-dir", str(HK / "cockpit" / "extensions")]
    if not ipaddress.ip_address("127.0.0.1" if host == "localhost" else host).is_loopback:
        missing = [str(p) for p in (tls / "cockpit.crt", tls / "cockpit.key", auth) if not p.exists()]
        if missing: raise SystemExit("refused: off loopback the relay needs its certificate and auth file: " + ", ".join(missing))
        argv += ["--cert", str(tls / "cockpit.crt"), "--key", str(tls / "cockpit.key"), "--auth-file", str(auth)]
    return argv


def project_argv(url: str, dimos: str | None = None) -> list[str]:
    dimos = dimos or str(Path(sys.executable).with_name("dimos"))
    return [dimos, "--robot-id", ROBOT_ID, "--relay-url", url, "--viewer", "none", "run", "houndkeeper.project"]


def project_env(url: str, auth: Path = AUTH, env: dict | None = None) -> dict:
    """the environment for the project service: its relay key from the auth file when the relay has one"""
    env = dict(os.environ if env is None else env)
    if not url.startswith("http://127.0.0.1") and not url.startswith("http://localhost"):
        keys = json.loads(auth.read_text()).get("robots", {})
        if ROBOT_ID not in keys: raise SystemExit(f'refused: {auth} has no robot key for "{ROBOT_ID}" (real/bringup/make_relay_auth.sh)')
        env["RELAY_KEY"] = keys[ROBOT_ID]
    return env


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("what", choices=("relay", "project"))
    ap.add_argument("--loopback", action="store_true", help="127.0.0.1, plain HTTP, no auth (tests, trying it out)")
    ap.add_argument("--port", type=int, default=PORT)
    a = ap.parse_args(argv)
    host, name = ("127.0.0.1", "127.0.0.1") if a.loopback else tailnet()
    if a.what == "relay":
        cmd = relay_argv(host, a.port, os.environ.get("HK_DENO") or shutil.which("deno") or "deno"); env = None
    else:
        url = f"http://{name}:{a.port}" if a.loopback else f"https://{name}:{a.port}"
        cmd = project_argv(url); env = project_env(url)
    os.chdir(HK)
    os.execvpe(cmd[0], cmd, env if env is not None else dict(os.environ))


if __name__ == "__main__":
    main()
