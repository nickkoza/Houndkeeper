"""Rebuilt from the repository [UYK] (packages pinned by a lock kept in the repository; a release built twice from one
commit is the same file, with its own manifest), power cut at any time [27n] (a release is on disk before the link
points at it; at boot an activation the cut interrupted is undone), one clock [aVJ] (the card's time source; the
report says whether the clock is synchronised) and the report's services [4WJ], [BRx] (the profile's, each enabled at
first boot). Laptop-only: no Pi, no card."""
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest

from arm_host import card, health, hostd, packages, release

ARM_HOST = Path(hostd.__file__).resolve().parent
ROOT = ARM_HOST.parent


def _code(name):
    return "\n".join(l.split("#", 1)[0] for l in (ARM_HOST / name).read_text().splitlines())   # comments aside


# ---- [UYK] packages pinned in the repository ------------------------------------------------------------------

def _debs(d, files):
    d.mkdir(parents=True, exist_ok=True)
    for name, body in files.items(): (d / name).write_bytes(body)
    return d


def test_a_lock_names_every_deb_by_its_checksum_and_the_package_list_it_was_fetched_for(tmp_path):
    """[UYK] the lock records the image, firstboot.sh's package list, and each .deb's sha256"""
    d = _debs(tmp_path / "debs", {"pipewire_1.4.2-1+rpt3_arm64.deb": b"a", "libx11-xcb1_2%3a1.8.12-1_arm64.deb": b"b"})
    lk = packages.lock(d, image="img.xz", pkgs="pipewire x")
    assert lk["image"] == "img.xz" and lk["packages"] == "pipewire x"
    assert lk["debs"]["pipewire_1.4.2-1+rpt3_arm64.deb"] == hashlib.sha256(b"a").hexdigest()
    assert packages.versions(lk) == {"pipewire": "1.4.2-1+rpt3", "libx11-xcb1": "2:1.8.12-1"}
    assert packages.check(d, lk, image="img.xz", pkgs="pipewire x") == []


def test_a_card_is_refused_packages_that_differ_from_the_lock(tmp_path):
    """[UYK] a changed, missing or extra .deb, another image or another package list: each named, so no card is made
    from anything but the pinned set"""
    d = _debs(tmp_path / "debs", {"a_1_arm64.deb": b"a", "b_1_arm64.deb": b"b"})
    lk = packages.lock(d, image="img.xz", pkgs="a b")
    (d / "a_1_arm64.deb").write_bytes(b"changed"); (d / "b_1_arm64.deb").unlink(); (d / "c_1_arm64.deb").write_bytes(b"c")
    problems = " ".join(packages.check(d, lk, image="img.xz", pkgs="a b"))
    assert "a_1_arm64.deb" in problems and "b_1_arm64.deb" in problems and "c_1_arm64.deb" in problems
    d2 = _debs(tmp_path / "d2", {"a_1_arm64.deb": b"a"}); lk2 = packages.lock(d2, image="img.xz", pkgs="a")
    assert packages.check(d2, lk2, image="other.xz", pkgs="a") and packages.check(d2, lk2, image="img.xz", pkgs="a chrony")


def test_the_repository_carries_the_lock_for_the_pinned_image_and_firstboots_packages():
    """[UYK] the lock in the repository is for pi-image.json's image and firstboot.sh's package list today"""
    lk = json.loads((ARM_HOST / "pi-packages.lock.json").read_text())
    assert lk["image"] == json.loads((ARM_HOST / "pi-image.json").read_text())["file"]
    assert lk["packages"] == packages.firstboot_pkgs()
    assert lk["debs"] and all(len(h) == 64 for h in lk["debs"].values())
    assert {"pipewire", "wireplumber", "libspa-0.2-bluetooth"} <= set(packages.versions(lk))   # the rest: the image's own


def test_the_card_and_the_fetch_go_through_the_lock():
    """[UYK] flash_card.sh refuses packages the lock does not name; fetch_packages.sh writes the lock it fetched"""
    assert "arm_host.packages check" in _code("flash_card.sh")
    assert "arm_host.packages write" in _code("fetch_packages.sh")


def _repo(tmp_path):
    repo = tmp_path / "repo"; (repo / "arm_host").mkdir(parents=True)
    for f in ("build_release.sh", "check.sh"): shutil.copy(ARM_HOST / f, repo / "arm_host" / f)
    (repo / "arm_host" / "x.py").write_text("a = 1\n")
    git = lambda *a: subprocess.run(["git", "-C", str(repo), *a], check=True, capture_output=True)
    git("init", "-q"); git("add", "."); git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", "x")
    return repo


def _build(repo, out):
    return subprocess.run([str(repo / "arm_host" / "build_release.sh"), str(out)], check=True, capture_output=True,
                          text=True).stdout.strip()


def test_two_builds_of_one_commit_are_the_same_file(tmp_path):
    """[UYK] the release from one commit is byte for byte the same, whenever and wherever it is built"""
    repo = _repo(tmp_path)
    a = _build(repo, tmp_path / "a"); os.utime(repo / "arm_host" / "x.py", (1, 1)); b = _build(repo, tmp_path / "b")
    assert a == b
    assert (tmp_path / "a" / "release.tgz").read_bytes() == (tmp_path / "b" / "release.tgz").read_bytes()


def test_a_release_carries_a_manifest_it_is_checked_against(tmp_path):
    """[UYK], [27n] every file of the release with its sha256; a release missing a file or with one cut short is not
    whole"""
    repo = _repo(tmp_path); n = _build(repo, tmp_path / "out")
    subprocess.run(["tar", "-C", str(tmp_path / "out"), "-xzf", str(tmp_path / "out" / "release.tgz")], check=True)
    rel = tmp_path / "out" / n
    assert "./arm_host/x.py" in (rel / "MANIFEST.sha256").read_text()
    assert release.whole(rel)
    (rel / "arm_host" / "x.py").write_text("")
    assert not release.whole(rel)
    (rel / "arm_host" / "x.py").unlink(); assert not release.whole(rel)
    (rel / "MANIFEST.sha256").unlink(); assert not release.whole(rel)


# ---- [27n] power cut at any time ------------------------------------------------------------------------------

def make(root: Path, n: str, ok: bool):
    d = root / "releases" / n; d.mkdir(parents=True)
    (d / "check.sh").write_text("#!/bin/sh\nexit %d\n" % (0 if ok else 1)); (d / "check.sh").chmod(0o755)
    return d


def test_a_release_that_passes_its_check_becomes_known_good(tmp_path):
    """[27n] the last release that passed its check is kept as a second link; a failed one never becomes it"""
    make(tmp_path, "1", True); make(tmp_path, "2", False)
    assert release.activate(tmp_path, "1", restart=lambda: None)
    assert (tmp_path / "known-good").resolve().name == "1"
    assert not release.activate(tmp_path, "2", restart=lambda: None)
    assert (tmp_path / "known-good").resolve().name == "1" and os.readlink(tmp_path / "known-good") == "releases/1"


def test_an_activation_cut_short_is_undone_at_boot(tmp_path):
    """[27n] power cut after the switch but before the check decided: at boot, the unchecked release is switched back
    to the known-good one"""
    make(tmp_path, "1", True); make(tmp_path, "2", True)
    release.activate(tmp_path, "1", restart=lambda: None)
    release._point(tmp_path, Path("releases/2"))                 # the switch landed; the cut came before the check
    assert release.recover(tmp_path) == "1"
    assert (tmp_path / "current").resolve().name == "1"
    assert release.recover(tmp_path) is None                      # nothing to undo: nothing changed
    assert (tmp_path / "current").resolve().name == "1"


def test_recovery_leaves_a_host_without_a_known_good_release_alone(tmp_path):
    """[27n] a host made before the known-good link (or mid first boot) is not touched"""
    make(tmp_path, "1", True); release._point(tmp_path, Path("releases/1"))
    assert release.recover(tmp_path) is None and (tmp_path / "current").resolve().name == "1"


def test_a_release_is_on_disk_before_the_link_points_at_it(tmp_path, monkeypatch):
    """[27n] every file of the new release, and its directories, are synced before the switch; the switch itself is
    synced after (the link's directory)"""
    make(tmp_path, "1", True); (tmp_path / "releases" / "1" / "sub").mkdir(); (tmp_path / "releases" / "1" / "sub" / "f").write_text("x")
    events, real_fsync, real_replace = [], os.fsync, os.replace
    def fsync(fd): events.append(("fsync", os.readlink(f"/proc/self/fd/{fd}"))); real_fsync(fd)
    def replace(a, b): events.append(("replace", str(b))); real_replace(a, b)
    monkeypatch.setattr(release.os, "fsync", fsync); monkeypatch.setattr(release.os, "replace", replace)
    release.activate(tmp_path, "1", restart=lambda: None)
    first_switch = events.index(("replace", str(tmp_path / "current")))
    synced = {p for e, p in events[:first_switch] if e == "fsync"}
    rel = tmp_path / "releases" / "1"
    assert {str(rel / "check.sh"), str(rel / "sub" / "f"), str(rel / "sub"), str(rel)} <= synced
    assert ("fsync", str(tmp_path)) in events[first_switch:]


def test_the_boot_undoes_a_cut_activation_with_the_known_good_releases_own_code():
    """[27n] a unit runs the recovery at boot, before the daemon, from the known-good release (not the unchecked one);
    first boot installs and enables it and sets the known-good link"""
    unit = (ARM_HOST / "systemd" / "hk-release-recover.service").read_text()
    assert "Before=hk-hostd.service" in unit and "Type=oneshot" in unit and "WantedBy=multi-user.target" in unit
    assert "/opt/houndkeeper/known-good/arm_host/release.py recover" in unit
    fb = _code("firstboot.sh")
    assert "known-good" in fb and "hk-release-recover" in fb


def test_settings_are_on_disk_before_they_replace_the_old_ones():
    """[27n] a profile switch writes host.json beside the old one and syncs it before the rename"""
    sw = _code("switch_profile.sh")
    assert sw.index("sync") < sw.index("mv /etc/houndkeeper/host.json.new")


# ---- [aVJ] one clock -------------------------------------------------------------------------------------------

def _boot(**kw):
    args = dict(profile="houndkeeper", ssh_key="ssh-ed25519 AAAATEST laptop", wifi=("HouseNet", "password1"),
                laptop_ip="192.0.2.10", release="12-abc", offline_packages=True)
    args.update(kw)
    return card.first_boot_files(**args)


def test_the_card_sets_the_time_source_and_never_a_public_one():
    """[aVJ] the clock follows the source given (the router the laptop also follows); no fallback servers on the
    internet ([DzZ]), with a source or without one"""
    conf = _boot(time_server="192.0.2.1")["hk/timesyncd.conf"]
    assert "NTP=192.0.2.1" in conf.splitlines() and "FallbackNTP=" in conf.splitlines()
    bare = _boot()["hk/timesyncd.conf"].splitlines()
    assert "FallbackNTP=" in bare and not any(l.startswith("NTP=") and l != "NTP=" for l in bare)


@pytest.mark.parametrize("bad", ["192.0.2.1\nNTP=pool.ntp.org", "a b", "x;reboot", "1.2.3.4 5.6.7.8"])
def test_a_time_source_that_is_not_one_address_is_refused(bad):
    with pytest.raises(card.Refused): _boot(time_server=bad)


def test_first_boot_installs_the_time_source_and_the_card_passes_it():
    fb, fl = _code("firstboot.sh"), _code("flash_card.sh")
    assert "timesyncd.conf" in fb and "timesyncd.conf.d" in fb
    assert "--time-server" in fl and "ROUTER_ON_ROBOT_NET" in fl


def test_the_report_says_whether_the_clock_is_synchronised(monkeypatch):
    """[aVJ], [4WJ] timedatectl's NTPSynchronized; None off the Pi"""
    def run(args, **kw):
        assert args[:2] == ["timedatectl", "show"]
        return subprocess.CompletedProcess(args, 0, stdout="yes\n", stderr="")
    monkeypatch.setattr(health.subprocess, "run", run)
    assert health.read_clock_synced() is True
    monkeypatch.setattr(health.subprocess, "run", lambda a, **k: subprocess.CompletedProcess(a, 0, stdout="no\n"))
    assert health.read_clock_synced() is False
    def off(*a, **k): raise OSError("no timedatectl")
    monkeypatch.setattr(health.subprocess, "run", off)
    assert health.read_clock_synced() is None
    assert hostd.Host(("127.0.0.1", 9), lambda: False, lambda: False).health()["clock_synced"] is None


# ---- [4WJ], [BRx] the profile's services -----------------------------------------------------------------------

def test_the_host_settings_name_the_profiles_services():
    """[4WJ], [BRx] host.json carries the profile's services: the report gives each one's state, first boot enables
    each one"""
    for name, p in card.profiles().items():
        assert card.host_settings(name, "192.0.2.10")["services"] == p["services"]
    fb = _code("firstboot.sh")
    assert "services" in fb and "systemctl enable" in fb


def test_the_daemon_reports_the_services_its_settings_name(monkeypatch, tmp_path):
    """[4WJ] each service the profile runs, not only the daemon itself"""
    cfg = tmp_path / "host.json"
    cfg.write_text(json.dumps({"services": ["hk-hostd", "hk-arm"], "features": {"voice": False, "eyes": False}}))
    monkeypatch.setattr(hostd, "CONFIG", cfg)
    seen = {}
    class Server:
        def __init__(self, addr, handler): seen["handler"] = handler
        def serve_forever(self): raise SystemExit(0)
    monkeypatch.setattr(hostd, "ThreadingHTTPServer", Server)
    monkeypatch.setattr(hostd.signal, "signal", lambda *a: None)
    monkeypatch.setattr(hostd.threading, "Thread", lambda target, daemon: type("T", (), {"start": lambda s: None})())
    made = {}
    real_host = hostd.Host
    def host(*a, **k): made.update(k); return real_host(*a, **k)
    monkeypatch.setattr(hostd, "Host", host)
    with pytest.raises(SystemExit): hostd.main()
    assert tuple(made["services"]) == ("hk-hostd", "hk-arm")


def test_every_profile_service_has_its_unit_and_restarts_when_it_dies():
    """[BRx] each service a profile names is a unit in the release, started at power-up and restarted if it dies"""
    for name, p in card.profiles().items():
        for s in p["services"]:
            unit = (ARM_HOST / "systemd" / f"{s}.service").read_text()
            assert "Restart=always" in unit and "WantedBy=multi-user.target" in unit, (name, s)
