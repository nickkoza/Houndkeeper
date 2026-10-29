"""Locked down [CIu], decision [ZyY]: a service the network reaches cannot become root. The services run as their own
account (no sudo, no login, no SSH key); the release tree, the settings and the unit files are root's and written only
by root; root runs Python only isolated (-I) from root-owned code; the on-Pi check runs the laptop's own copy of its
code, not the Pi's. Laptop-only: the card's files, first boot, the units and the scripts as written, and pi_check.sh
driven end to end through a stand-in ssh and sudo."""
import json
import os
import re
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

from arm_host import card, pi_check

ARM_HOST = Path(pi_check.__file__).resolve().parent
UNITS = sorted((ARM_HOST / "systemd").glob("*.service"))
SVC, ADMIN = pi_check.SERVICE_USER, pi_check.ADMIN_USER
ROOT_SCRIPTS = ("firstboot.sh", "check.sh")          # run as root throughout: by cloud-init, by release.activate


def _code(name):
    return "\n".join(l.split("#", 1)[0] for l in (ARM_HOST / name).read_text().splitlines())   # comments aside


def _unit(path):
    """[Service] keys of a unit file, the last value of each"""
    out, sec = {}, None
    for line in path.read_text().splitlines():
        line = line.strip()
        if line.startswith("["): sec = line; continue
        if sec == "[Service]" and "=" in line and not line.startswith("#"):
            k, _, v = line.partition("="); out[k.strip()] = v.strip()
    return out


def test_the_service_user_and_the_admin_are_different_accounts():
    assert SVC and ADMIN and SVC != ADMIN and SVC != "root"


def test_no_service_runs_as_the_admin_and_each_profile_service_runs_as_the_service_user():
    """[CIu] the admin (hk) has sudo; a unit running as it hands root to whoever takes the daemon over"""
    users = {p.stem: _unit(p).get("User", "") for p in UNITS}
    assert ADMIN not in users.values(), users
    for p in card.profiles().values():
        for s in p["services"]: assert users[s] == SVC, (s, users[s])


def test_the_daemon_is_confined():
    """[CIu] no way up (no new privileges, no capabilities), nothing of the system writable, its own /tmp"""
    u = _unit(ARM_HOST / "systemd" / "hk-hostd.service")
    assert u["NoNewPrivileges"] == "yes" and u["ProtectSystem"] == "strict" and u["PrivateTmp"] == "yes"
    assert u.get("CapabilityBoundingSet", None) == "" and u.get("ProtectHome") in ("yes", "read-only", "tmpfs")
    assert "PYTHONPATH=/opt/houndkeeper/current" in u["Environment"]


def test_a_unit_run_as_root_runs_isolated_python_from_the_root_owned_tree():
    """[CIu], [27n] the recovery switches links in root's tree, so it is root's; its Python ignores the environment
    and the current directory, and runs from the known-good release"""
    assert _unit(ARM_HOST / "systemd" / "hk-release-recover.service").get("User", "root") == "root"
    for p in UNITS:
        u = _unit(p)
        if u.get("User", "root") != "root": continue
        assert re.search(r"python3 -I ", u["ExecStart"]) and "PYTHONPATH" not in u.get("Environment", ""), p.name
        assert u["ExecStart"].split()[-2].startswith("/opt/houndkeeper/known-good/"), p.name


def _python_calls(text):
    return re.findall(r"python3(?![-\w])[^\n|;&)]*", text)


@pytest.mark.parametrize("name", ROOT_SCRIPTS)
def test_root_scripts_run_python_only_isolated(name):
    """[CIu] root never takes Python's path from the environment or the current directory"""
    calls = _python_calls(_code(name))
    assert calls and all(c.startswith("python3 -I") for c in calls), calls


@pytest.mark.parametrize("name", sorted(p.name for p in ARM_HOST.glob("*.sh")))
def test_nothing_runs_python_under_sudo_with_a_path_it_was_given(name):
    """[CIu] `sudo PYTHONPATH=. python3 -m ...` in a directory the daemon could write was root for the daemon"""
    for line in _code(name).splitlines():
        if "sudo" in line and "python3" in line:
            assert "PYTHONPATH" not in line and re.search(r"python3 -I\b", line), (name, line)


def _owners(text):
    """every owner a script gives: chown U[:G], install -o U"""
    return re.findall(r"chown\s+(?:-\w+\s+)*([\w-]+)(?::[\w-]+)?\s", text) + re.findall(r"install\b[^\n]*?-o\s+([\w-]+)", text)


@pytest.mark.parametrize("name", ["firstboot.sh"])
def test_set_up_gives_the_release_tree_and_settings_only_to_root(name):
    """[CIu] the tree root runs code from, and the settings it reads, belong to root; the only thing the service user
    owns is its own home (for its audio session)"""
    text = _code(name)
    for line in text.splitlines():
        for who in _owners(line + " "):
            if who == "root": continue
            assert who == SVC and "/var/lib/houndkeeper" in line and "/opt" not in line and "/etc" not in line, (name, line)
    assert "-o hk " not in text and "hk:hk" not in text


def test_first_boot_makes_the_tree_roots_before_it_installs_units_from_it():
    """[CIu] first boot installs the units from the release only once the tree is root's (and nothing else ever
    owns it); the release unpacked keeping no owner from the archive"""
    fb = _code("firstboot.sh")
    assert re.search(r"tar\b[^\n]*--no-same-owner[^\n]*release\.tgz", fb)
    root_it = fb.index("chown -R root:root /opt/houndkeeper")
    assert root_it < fb.index("/etc/systemd/system/") and root_it < fb.index("systemctl enable")


@pytest.mark.parametrize("name", ["firstboot.sh"])
def test_set_up_makes_a_service_user_with_no_sudo_no_login_and_no_ssh(name):
    """[CIu] a system account, no login shell, only the device groups the daemon uses; SSH lets only the admin in"""
    text = _code(name)
    m = re.search(rf"useradd\b([^\n]*)\b{SVC}\b", text)
    assert m and "--system" in m.group(1) and "--shell /usr/sbin/nologin" in m.group(1), m and m.group(0)
    groups = re.search(rf"usermod -aG ([\w,]+) {SVC}\b", text).group(1).split(",")
    assert "sudo" not in groups and "adm" not in groups and {"spi", "audio", "bluetooth"} <= set(groups)
    assert f"AllowUsers {ADMIN}" in text
    assert re.search(rf"enable-linger {SVC}\b", text) and not re.search(rf"enable-linger {ADMIN}\b", text)   # its audio


def test_the_card_gives_sudo_and_ssh_keys_only_to_the_admin():
    """[CIu] cloud-init makes one user, the admin; the service user has no key and no sudo (first boot makes it)"""
    ud = json.loads(card.first_boot_files("houndkeeper", "ssh-ed25519 AAAATEST laptop", ("HouseNet", "password1"),
                                          "192.0.2.10", "12-abc")["user-data"].split("\n", 1)[1])
    assert [u["name"] for u in ud["users"]] == [ADMIN]
    assert ud["users"][0]["lock_passwd"] is True and ud["disable_root"] is True


def test_deploy_writes_the_release_tree_only_through_sudo():
    """[CIu], [871] a release is unpacked, renamed and activated by root, from the stream, keeping no owner; nothing
    left in a shared /tmp under a name another account could take first"""
    d = _code("deploy.sh")
    assert "/tmp/hk-release" not in d and "scp" not in d
    assert re.search(r"sudo mktemp -d \\\$R/\.incoming", d) and re.search(r"sudo tar --no-same-owner[^\n]*-xzf -", d)
    assert re.search(r"sudo mv \\\$S/", d) and re.search(r"sudo python3 -I -B \\\$A/arm_host/release\.py activate", d)
    assert "cd \\$A" not in d



def test_deploy_refuses_a_host_without_the_boundary_before_root_runs_its_code():
    """[CIu], decision [ZyY]: users and units change only on a new card, so a deploy first proves the host has the
    service user, the daemon running as it and a tree only root owns and writes, and refuses (reflash) otherwise, before
    any sudo touches the tree or runs its code"""
    d = _code("deploy.sh")
    guard = d.index("id hkd")
    assert "grep -qx 'User=hkd' /etc/systemd/system/hk-hostd.service" in d
    assert "! -user root -print -quit" in d and "-perm /022 -print -quit" in d and "exit 4" in d
    assert guard < d.index("sudo mktemp") and guard < d.index("sudo python3")

# ---- pi_check.sh end to end: the laptop's code, not the Pi's ---------------------------------------------------

STAND_IN_SSH = """#!/bin/bash
# the last argument is the remote command: run it here, with stdin passed through
for a; do :; done; exec bash -c "$a"
"""
STAND_IN_SUDO = """#!/bin/bash
echo "$*" >> "$HK_SUDO_LOG"; exec "$@"
"""


def test_the_on_pi_check_runs_the_laptops_own_code_from_a_fresh_root_dir(tmp_path):
    """[CIu], [Uyb] the Pi's half runs code shipped from the laptop into a new directory made by root and removed
    after, isolated Python; nothing of the Pi's installed release is imported (a compromised tree cannot forge the
    check). Here there is no /opt/houndkeeper at all, which the old command needed."""
    repo = tmp_path / "repo"; shutil.copytree(ARM_HOST, repo / "arm_host", ignore=shutil.ignore_patterns("__pycache__"))
    bin_ = tmp_path / "bin"; bin_.mkdir()
    for name, body in (("ssh", STAND_IN_SSH), ("sudo", STAND_IN_SUDO)):
        (bin_ / name).write_text(body); (bin_ / name).chmod(0o755)
    log = tmp_path / "sudo.log"; scratch = tmp_path / "tmp"; scratch.mkdir()
    env = dict(os.environ, PATH=f"{bin_}:{os.environ['PATH']}", HK_SUDO_LOG=str(log), TMPDIR=str(scratch))
    env.pop("PYTHONPATH", None)
    r = subprocess.run(["/bin/bash", str(repo / "arm_host" / "pi_check.sh"), "localhost", "--only", "NONE"],
                       capture_output=True, text=True, timeout=60, env=env)
    out = sorted((repo / "real" / "local" / "arm-host-checks").glob("*-pi.json"))
    assert "did not finish" not in r.stderr and out and "host" in json.loads(out[-1].read_text()), (r.stdout, r.stderr)
    calls = log.read_text().splitlines()
    assert any(c.startswith("mktemp -d") for c in calls)
    py = [c for c in calls if c.startswith("python3")]
    assert py and all(c.startswith("python3 -I") for c in py) and not any("/opt/houndkeeper" in c for c in py)
    assert any(c.startswith("rm -rf ") for c in calls) and list(scratch.iterdir()) == []     # removed after


def test_the_check_ships_every_module_its_on_pi_half_imports():
    """[Uyb] what pi_check.sh sends is what `pi_check on-pi` needs, so the Pi's copy is never reached for"""
    sh = (ARM_HOST / "pi_check.sh").read_text()
    for f in ("__init__.py", "pi_check.py", "packages.py", "release.py", "pi-packages.lock.json"):
        assert f"arm_host/{f}" in sh, f
    imports = set(re.findall(r"from arm_host import ([\w, ]+)", (ARM_HOST / "pi_check.py").read_text())[0].replace(" ", "").split(","))
    assert {f"{m}.py" for m in imports} <= {"pi_check.py", "packages.py", "release.py"}
