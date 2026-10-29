"""The on-Pi checks (arm_host/pi_check.py) judged on the laptop, from what the Pi's commands print: rebuilt from the
repository [UYK], ready from power-up [BRx], one clock [aVJ], the report [4WJ], power cut [27n], locked down [CIu],
wrist video stays in the house [Ot6]. Each judge is fed a passing and a failing case (the failing one is the planted
error: a check that cannot fail proves nothing). Addresses are documentation ones (RFC 5737)."""
import json
import subprocess
import stat
from pathlib import Path

import pytest

from arm_host import packages, pi_check, release

LAPTOP, ROUTER, PI = "192.0.2.10", "192.0.2.1", "192.0.2.20"
ARM_HOST = Path(pi_check.__file__).resolve().parent


def ok(r): return r["result"] == "pass"


# ---- [UYK] -----------------------------------------------------------------------------------------------------

LOCK = {"image": "img.xz", "packages": "a", "debs": {"a_1.0-1_arm64.deb": "0" * 64, "liba_2%3a3.1_arm64.deb": "1" * 64}}


def test_installed_packages_are_the_locked_versions():
    """[UYK] every locked .deb installed at its version; one at another version, or missing, fails"""
    good = pi_check.parse_dpkg("a 1.0-1\nliba 2:3.1\nbash 5.2\n")
    assert ok(pi_check.judge_packages(good, LOCK))
    assert not ok(pi_check.judge_packages(pi_check.parse_dpkg("a 1.0-2\nliba 2:3.1\n"), LOCK))
    assert not ok(pi_check.judge_packages(pi_check.parse_dpkg("a 1.0-1\n"), LOCK))


def test_two_hosts_from_one_commit_have_one_fingerprint():
    """[UYK] the fingerprint is the locked packages as installed and the release's manifest: the same on two cards
    from one commit, different when either differs; packages outside the lock (the image's own) do not move it"""
    inst = pi_check.parse_dpkg("a 1.0-1\nliba 2:3.1\nbash 5.2\n")
    f = pi_check.fingerprint(inst, LOCK, "abc  ./x.py\n")
    assert f == pi_check.fingerprint(dict(inst, bash="5.3"), LOCK, "abc  ./x.py\n")
    assert f != pi_check.fingerprint(dict(inst, a="1.0-2"), LOCK, "abc  ./x.py\n")
    assert f != pi_check.fingerprint(inst, LOCK, "abd  ./x.py\n")
    a = {"results": [{"id": "UYK", "check": "fingerprint", "result": "pass", "detail": f}]}
    b = {"results": [{"id": "UYK", "check": "fingerprint", "result": "pass", "detail": "other"}]}
    assert ok(pi_check.compare(a, a)) and not ok(pi_check.compare(a, b))


# ---- [BRx] -----------------------------------------------------------------------------------------------------

def test_every_profile_service_is_enabled_and_running():
    """[BRx] started at power-up means enabled and active; either missing fails"""
    assert ok(pi_check.judge_services({"hk-hostd": ("enabled", "active"), "hk-release-recover": ("enabled", "active")}))
    assert not ok(pi_check.judge_services({"hk-hostd": ("disabled", "active")}))
    assert not ok(pi_check.judge_services({"hk-hostd": ("enabled", "failed")}))
    assert not ok(pi_check.judge_services({}))


def test_a_killed_service_is_back_as_a_new_process():
    """[BRx] after SIGKILL the service is active again under a new process within the wait"""
    assert ok(pi_check.judge_restart(100, 140, "active", 3.0))
    assert not ok(pi_check.judge_restart(100, 100, "active", 3.0))           # never died
    assert not ok(pi_check.judge_restart(100, 0, "failed", 15.0))


def test_the_arm_is_reported_disarmed_or_not_yet_built():
    """[BRx] the arm stays disarmed until DimOS arms it: armed at power-up fails; a report with no armed state (the
    adapter not built) is said, not passed"""
    assert ok(pi_check.judge_disarmed({"arm": "absent", "armed": False}))
    assert not ok(pi_check.judge_disarmed({"arm": "present", "armed": True}))
    assert pi_check.judge_disarmed({"arm": "absent"})["result"] == "not built"


# ---- [aVJ] -----------------------------------------------------------------------------------------------------

def test_the_clock_is_synchronised_to_the_set_source():
    """[aVJ] timesyncd synchronised, to a source the card set, with no fallback"""
    conf = "[Time]\nNTP=192.0.2.1\nFallbackNTP=\n"
    assert ok(pi_check.judge_clock("yes", conf))
    assert not ok(pi_check.judge_clock("no", conf))
    assert not ok(pi_check.judge_clock("yes", "[Time]\nNTP=\nFallbackNTP=\n"))
    assert not ok(pi_check.judge_clock("yes", None))


def test_the_offset_to_the_laptop_is_judged_on_the_quickest_exchange():
    """[aVJ] the Pi's time against the middle of the laptop's send and receive, from the exchange with the shortest
    round trip; within the tolerance plus half that round trip passes, beyond it fails"""
    samples = [(100.0, 100.060, 100.100), (200.0, 200.0105, 200.020), (300.0, 300.2, 300.4)]
    off, unc = pi_check.clock_offset(samples)
    assert off == pytest.approx(0.0005) and unc == pytest.approx(0.010)
    assert ok(pi_check.judge_offset(samples))
    assert not ok(pi_check.judge_offset([(200.0, 200.5, 200.020)]))
    assert not ok(pi_check.judge_offset([]))


# ---- [4WJ] -----------------------------------------------------------------------------------------------------

def _report(**kw):
    r = {"time": 0, "release": "12-abc", "services": {"hk-hostd": "active"}, "cpu_temp_c": 50.0,
         "throttle": {"under_voltage_now": False}, "arm": "absent", "speaker": "connected", "clock_synced": True}
    r.update(kw); return json.dumps(r).encode()


def test_reports_arrive_from_the_pi_at_least_once_a_second():
    """[4WJ] every gap under a second (with a little scheduling slack), every field there, from the Pi's address"""
    good = [(t, _report(), (PI, 40000)) for t in (0.0, 1.0, 2.01, 3.0, 4.0, 5.0)]
    assert ok(pi_check.judge_reports(good, PI, window_s=5.0))
    gap = [(t, _report(), (PI, 40000)) for t in (0.0, 1.0, 3.0, 4.0, 5.0)]
    assert not ok(pi_check.judge_reports(gap, PI, window_s=5.0))
    other = [(t, _report(), ("192.0.2.99", 40000)) for t in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)]
    assert not ok(pi_check.judge_reports(other, PI, window_s=5.0))
    missing = [(t, json.dumps({"time": 0}).encode(), (PI, 40000)) for t in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)]
    assert not ok(pi_check.judge_reports(missing, PI, window_s=5.0))
    late_start = [(t, _report(), (PI, 40000)) for t in (2.5, 3.0, 4.0, 5.0)]
    assert not ok(pi_check.judge_reports(late_start, PI, window_s=5.0))


# ---- [27n] -----------------------------------------------------------------------------------------------------

def _release(root, n, ok_=True):
    d = root / "releases" / n; (d / "arm_host").mkdir(parents=True)
    (d / "arm_host" / "x.py").write_text("x\n"); (d / "check.sh").write_text("#!/bin/sh\nexit %d\n" % (0 if ok_ else 1))
    (d / "check.sh").chmod(0o755)
    subprocess.run("find . -type f ! -name MANIFEST.sha256 -print0 | sort -z | xargs -0 sha256sum > MANIFEST.sha256",
                   shell=True, cwd=d, check=True)
    return d


def test_after_a_cut_the_host_runs_a_whole_known_good_release(tmp_path):
    """[27n] current is the known-good release and every file of it matches its manifest; a current that is not the
    known-good one, or a release cut short, fails"""
    _release(tmp_path, "1"); release.activate(tmp_path, "1", restart=lambda: None)
    assert ok(pi_check.judge_release_state(tmp_path))
    _release(tmp_path, "2"); release._point(tmp_path, Path("releases/2"))
    assert not ok(pi_check.judge_release_state(tmp_path))
    release._point(tmp_path, Path("releases/1")); (tmp_path / "releases" / "1" / "arm_host" / "x.py").write_text("")
    assert not ok(pi_check.judge_release_state(tmp_path))
    assert not ok(pi_check.judge_release_state(tmp_path / "nothing"))


def test_the_root_file_system_is_repaired_at_boot():
    """[27n] the kernel's command line asks for the root file system to be repaired at boot"""
    assert ok(pi_check.judge_cmdline("console=tty1 root=PARTUUID=x rootfstype=ext4 fsck.repair=yes rootwait"))
    assert not ok(pi_check.judge_cmdline("console=tty1 root=PARTUUID=x rootfstype=ext4 rootwait"))


# ---- [CIu] -----------------------------------------------------------------------------------------------------

def test_ssh_takes_keys_only():
    """[CIu] sshd's effective settings: no passwords, no keyboard-interactive, no root"""
    good = "port 22\npasswordauthentication no\nkbdinteractiveauthentication no\npermitrootlogin no\npubkeyauthentication yes\n"
    assert ok(pi_check.judge_sshd(good))
    assert not ok(pi_check.judge_sshd(good.replace("passwordauthentication no", "passwordauthentication yes")))
    assert not ok(pi_check.judge_sshd(good.replace("permitrootlogin no", "permitrootlogin prohibit-password")))
    assert not ok(pi_check.judge_sshd(good.replace("kbdinteractiveauthentication no\n", "")))


def test_no_account_can_log_in_with_a_password():
    """[CIu] every account's password locked or absent; an empty password (log in with none) or a hash fails"""
    good = "root:*:19000:0:99999:7:::\nhk:!:19000:0:99999:7:::\nsshd:!*:19000::::::\n"
    assert ok(pi_check.judge_shadow(good))
    assert not ok(pi_check.judge_shadow(good + "pi:$6$salt$hash:19000:0:99999:7:::\n"))
    assert not ok(pi_check.judge_shadow(good.replace("hk:!:", "hk::")))


def test_no_default_account_is_left():
    """[CIu] the image's old default user is not there to be enabled"""
    assert ok(pi_check.judge_users("root:x:0:0::/root:/bin/bash\nhk:x:1000:1000::/srv/hk:/bin/bash\n"))
    assert not ok(pi_check.judge_users("root:x:0:0::/root:/bin/bash\npi:x:1000:1000::/srv/pi:/bin/bash\n"))


def test_only_ssh_and_the_hosts_api_listen():
    """[CIu] TCP listening: SSH and the API (which answers only the laptop) on the network, anything on loopback;
    UDP: mDNS, DHCP and the kernel's ephemeral range (the report's and the microphone's sending sockets)"""
    tcp = "LISTEN 0 128 0.0.0.0:22 0.0.0.0:*\nLISTEN 0 128 [::]:22 [::]:*\nLISTEN 0 5 0.0.0.0:47100 0.0.0.0:*\n" \
          "LISTEN 0 4096 127.0.0.1:631 0.0.0.0:*\n"
    udp = "UNCONN 0 0 0.0.0.0:5353 0.0.0.0:*\nUNCONN 0 0 0.0.0.0:68 0.0.0.0:*\nUNCONN 0 0 0.0.0.0:41234 0.0.0.0:*\n"
    assert ok(pi_check.judge_listening(tcp, udp))
    assert not ok(pi_check.judge_listening(tcp + "LISTEN 0 64 0.0.0.0:8080 0.0.0.0:*\n", udp))
    assert not ok(pi_check.judge_listening(tcp, udp + "UNCONN 0 0 0.0.0.0:69 0.0.0.0:*\n"))


def test_from_the_laptop_only_ssh_and_the_api_answer():
    assert ok(pi_check.judge_open_ports([22, 47100]))
    assert not ok(pi_check.judge_open_ports([22, 23, 47100]))
    assert ok(pi_check.judge_open_ports([], expect_unreachable=True))           # from the guest network
    assert not ok(pi_check.judge_open_ports([22], expect_unreachable=True))


def test_a_password_login_is_refused_before_any_password_is_asked():
    """[CIu] ssh with keys turned off is told only 'publickey' is accepted"""
    assert ok(pi_check.judge_password_refused(255, "hk@hk-arm.local: Permission denied (publickey).\n"))
    assert not ok(pi_check.judge_password_refused(255, "hk@hk-arm.local: Permission denied (publickey,password).\n"))
    assert not ok(pi_check.judge_password_refused(0, ""))
    assert not ok(pi_check.judge_password_refused(255, "ssh: connect to host x port 22: No route to host\n"))


# ---- [Ot6] -----------------------------------------------------------------------------------------------------

def test_the_host_keeps_no_footage(tmp_path):
    """[Ot6] no picture, video or sound file under the places the host writes; one planted is found"""
    (tmp_path / "home" / "hk").mkdir(parents=True); (tmp_path / "home" / "hk" / "notes.txt").write_text("x")
    assert ok(pi_check.judge_media(pi_check.find_media([tmp_path / "home", tmp_path / "absent"])))
    (tmp_path / "home" / "hk" / "wrist_0001.JPG").write_bytes(b"\xff\xd8")
    (tmp_path / "home" / "hk" / "clip.h264").write_bytes(b"x")
    r = pi_check.judge_media(pi_check.find_media([tmp_path / "home"]))
    assert not ok(r) and "wrist_0001.JPG" in r["detail"] and "clip.h264" in r["detail"]


def test_the_host_talks_only_to_the_laptop_and_its_time_source():
    """[Ot6] every connection open or being opened is with the laptop, the time source or the host itself; one to
    anywhere else (an attempt included) fails"""
    ss = (f"tcp ESTAB 0 0 {PI}:22 {LAPTOP}:51000\nudp ESTAB 0 0 {PI}:40000 {LAPTOP}:47110\n"
          f"udp ESTAB 0 0 {PI}:123 {ROUTER}:123\ntcp LISTEN 0 128 0.0.0.0:22 0.0.0.0:*\n"
          "tcp ESTAB 0 0 127.0.0.1:47100 127.0.0.1:50000\nudp UNCONN 0 0 0.0.0.0:5353 0.0.0.0:*\n"
          f"tcp ESTAB 0 0 [::ffff:{PI}]:47100 [::ffff:{LAPTOP}]:50001\n")
    assert ok(pi_check.judge_peers(ss, {LAPTOP, ROUTER}))
    assert not ok(pi_check.judge_peers(ss + f"tcp SYN-SENT 0 1 {PI}:50100 198.51.100.7:443\n", {LAPTOP, ROUTER}))
    assert not ok(pi_check.judge_peers(ss + f"udp ESTAB 0 0 {PI}:50200 198.51.100.8:5004\n", {LAPTOP, ROUTER}))


def test_the_wrist_stream_is_said_to_be_not_built():
    """[Ot6] until the wrist camera streams, its check says so rather than passing"""
    assert pi_check.judge_wrist_stream({"features": {"wrist_camera": True}}, None)["result"] == "not built"


# ---- the one command -------------------------------------------------------------------------------------------

def test_one_command_runs_both_halves_and_keeps_the_record_out_of_the_repository():
    """the Pi's half over SSH from its current release, the laptop's half, and the record under real/local/
    (gitignored: it holds the house's addresses)"""
    sh = (ARM_HOST / "pi_check.sh").read_text()
    assert '\\"arm_host.pi_check\\"' in sh and " on-pi " in sh and "arm_host.pi_check from-laptop" in sh and "real/local/" in sh
    assert "real/.env" in sh


def test_the_overall_verdict_fails_on_any_failure_and_says_what_is_left():
    rs = [pi_check.result("UYK", "a", True), pi_check.result("BRx", "b", None, "not built"),
          pi_check.result("CIu", "c", None, "manual", "a scan from the guest network")]
    assert pi_check.verdict(rs) == 0
    assert pi_check.verdict(rs + [pi_check.result("27n", "d", False, detail="x")]) == 1


# ---- [CIu] a service the network reaches cannot become root (decision [ZyY]) ----------------------------------

PASSWD = "root:x:0:0:root:/root:/bin/bash\nhk:x:1000:1000::/srv/hk:/bin/bash\n" \
         "hkd:x:998:998::/var/lib/houndkeeper:/usr/sbin/nologin\n"
NO_SUDO = "User hkd is not allowed to run sudo on hk-arm.\n"


def test_the_service_user_has_no_sudo_no_login_and_no_key():
    """[CIu] the account the services run as: a no-login shell, sudo -l refuses it, no authorized_keys; each planted
    way up fails, and so does an account that is not there"""
    assert ok(pi_check.judge_service_account("hkd", PASSWD, NO_SUDO, has_keys=False))
    assert not ok(pi_check.judge_service_account("hkd", PASSWD.replace("/usr/sbin/nologin", "/bin/bash"), NO_SUDO, False))
    assert not ok(pi_check.judge_service_account("hkd", PASSWD, "User hkd may run the following commands on hk-arm:\n"
                                                 "    (ALL) NOPASSWD: ALL\n", False))
    assert not ok(pi_check.judge_service_account("hkd", PASSWD, "", False))          # sudo said nothing: not shown
    assert not ok(pi_check.judge_service_account("hkd", PASSWD, NO_SUDO, has_keys=True))
    assert not ok(pi_check.judge_service_account("hkd", PASSWD.replace("hkd:", "hkx:"), NO_SUDO, False))


def test_each_profile_service_runs_as_the_service_user():
    """[CIu] systemctl show -p User: the profile's services as the service user; one as the admin, or as root (an
    empty User), fails"""
    assert ok(pi_check.judge_unit_users({"hk-hostd": "hkd"}, "hkd"))
    assert not ok(pi_check.judge_unit_users({"hk-hostd": "hk"}, "hkd"))
    assert not ok(pi_check.judge_unit_users({"hk-hostd": ""}, "hkd"))
    assert not ok(pi_check.judge_unit_users({}, "hkd"))


def _entry(path, uid=0, gid=0, mode=0o755, kind=stat.S_IFDIR):
    return (path, uid, gid, kind | mode)


def test_the_release_tree_and_settings_are_roots_and_not_writable_by_the_service_user():
    """[CIu] every file and directory root runs or reads: owned by root, writable by nobody else; one the service user
    owns, one its group can write, one anyone can write, and one owned by another account (who could chmod it) each
    fail. Links are not judged (their directory is)."""
    tree = [_entry("/opt/houndkeeper"), _entry("/opt/houndkeeper/releases"),
            _entry("/opt/houndkeeper/releases/1/arm_host/hostd.py", mode=0o644, kind=stat.S_IFREG),
            _entry("/opt/houndkeeper/current", uid=998, mode=0o777, kind=stat.S_IFLNK),
            _entry("/etc/houndkeeper/host.json", mode=0o644, kind=stat.S_IFREG)]
    assert ok(pi_check.judge_not_writable(tree, uid=998, gids={998, 29}))
    for bad in (_entry("/opt/houndkeeper/releases/1", uid=998),
                _entry("/opt/houndkeeper/releases/1/x.py", gid=29, mode=0o664, kind=stat.S_IFREG),
                _entry("/etc/houndkeeper", mode=0o757),
                _entry("/opt/houndkeeper/releases/2", uid=1000)):
        r = pi_check.judge_not_writable(tree + [bad], uid=998, gids={998, 29})
        assert not ok(r) and bad[0] in r["detail"], bad
    assert not ok(pi_check.judge_not_writable([], uid=998, gids={998}))             # nothing looked at: not shown
