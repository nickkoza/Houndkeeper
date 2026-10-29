"""[871] a release is applied by switching the "current" link; one that fails its start-up check is rolled back."""
from pathlib import Path

from arm_host.release import activate


def make(root: Path, n: str, ok: bool):
    d = root / "releases" / n; d.mkdir(parents=True)
    (d / "check.sh").write_text("#!/bin/sh\nexit %d\n" % (0 if ok else 1)); (d / "check.sh").chmod(0o755)
    return d


def test_good_release_becomes_current(tmp_path):
    make(tmp_path, "1", True); make(tmp_path, "2", True)
    assert activate(tmp_path, "1", restart=lambda: None)
    assert activate(tmp_path, "2", restart=lambda: None)
    assert (tmp_path / "current").resolve().name == "2"


def test_failing_release_rolls_back(tmp_path):
    make(tmp_path, "1", True); make(tmp_path, "2", False)
    activate(tmp_path, "1", restart=lambda: None)
    restarts = []
    assert not activate(tmp_path, "2", restart=lambda: restarts.append(1))
    assert (tmp_path / "current").resolve().name == "1"
    assert len(restarts) == 2                                     # onto the new one, and back


def test_switch_is_atomic(tmp_path):
    make(tmp_path, "1", True)
    activate(tmp_path, "1", restart=lambda: None)
    assert (tmp_path / "current").is_symlink()
    assert not list(tmp_path.glob("current.*"))                   # no temporary link left behind


# ---- red team, 3 Oct 2026 (docs/research/redteam-arm-host-2026-10-03.md) -----------------------------------------------
import fcntl, subprocess

import pytest


def test_the_running_release_is_not_activated_again(tmp_path):
    """[871] two dirty builds of one commit had one number: the second was unpacked over the running release, and
    when it failed its check the 'previous' release was the same broken directory"""
    make(tmp_path, "1", True); activate(tmp_path, "1", restart=lambda: None)
    with pytest.raises(ValueError): activate(tmp_path, "1", restart=lambda: None)


def test_one_activation_at_a_time(tmp_path):
    """[871] two activations racing could each restore the other's 'previous'"""
    make(tmp_path, "1", True)
    with open(tmp_path / ".activate.lock", "w") as f:
        fcntl.flock(f, fcntl.LOCK_EX)
        with pytest.raises(BlockingIOError): activate(tmp_path, "1", restart=lambda: None)


def test_dirty_releases_of_one_commit_get_different_numbers(tmp_path):
    """[871] build_release.sh names a dirty build by its content too"""
    import shutil
    root = Path(__file__).resolve().parents[2]
    repo = tmp_path / "repo"; (repo / "arm_host").mkdir(parents=True)
    for f in ("build_release.sh", "check.sh"): shutil.copy(root / "arm_host" / f, repo / "arm_host" / f)
    (repo / "arm_host" / "x.py").write_text("a = 1\n")
    git = lambda *a: subprocess.run(["git", "-C", str(repo), *a], check=True, capture_output=True)
    git("init", "-q"); git("add", "."); git("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", "x")
    build = lambda: subprocess.run([str(repo / "arm_host" / "build_release.sh"), str(tmp_path / "out")], check=True,
                                   capture_output=True, text=True).stdout.strip()
    clean = build()
    (repo / "arm_host" / "x.py").write_text("a = 2\n"); one = build()
    (repo / "arm_host" / "x.py").write_text("a = 3\n"); two = build()
    assert "dirty" not in clean and "dirty" in one and one != two
