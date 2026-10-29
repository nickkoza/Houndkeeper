"""Tests written to kill mutants that survived the suite (docs/research/test-quality-2026-10-03.md): a release's activation
([871]) when its check hangs, when it is the very first release and fails, when it is not there, and when the tree is
moved (the card is built in one place and runs in another)."""
import shutil
from pathlib import Path

import pytest

from arm_host.release import activate


def make(root: Path, n: str, script: str):
    d = root / "releases" / n; d.mkdir(parents=True)
    (d / "check.sh").write_text("#!/bin/sh\n" + script + "\n"); (d / "check.sh").chmod(0o755)
    return d


def test_a_check_that_hangs_is_a_failed_check(tmp_path):
    """[871] a start-up check that never answers is rolled back, not trusted"""
    make(tmp_path, "1", "exit 0"); make(tmp_path, "2", "sleep 30")
    activate(tmp_path, "1", restart=lambda: None)
    assert not activate(tmp_path, "2", restart=lambda: None, check_timeout=1)
    assert (tmp_path / "current").resolve().name == "1"


def test_the_first_release_failing_leaves_no_current(tmp_path):
    """[871] nothing to roll back to: no current link, rather than one to the broken release"""
    make(tmp_path, "1", "exit 1")
    restarts = []
    assert not activate(tmp_path, "1", restart=lambda: restarts.append(1))
    assert not (tmp_path / "current").is_symlink() and len(restarts) == 2


def test_a_release_that_is_not_there_changes_nothing(tmp_path):
    make(tmp_path, "1", "exit 0"); activate(tmp_path, "1", restart=lambda: None)
    restarts = []
    with pytest.raises(FileNotFoundError): activate(tmp_path, "7", restart=lambda: restarts.append(1))
    assert (tmp_path / "current").resolve().name == "1" and restarts == []


def test_the_current_link_holds_when_the_tree_is_moved(tmp_path):
    """[871] the link is relative to the root: a tree built in one place (a card image) works where it is mounted"""
    root = tmp_path / "built"; make(root, "1", "exit 0"); activate(root, "1", restart=lambda: None)
    moved = tmp_path / "mounted"; shutil.move(str(root), str(moved))
    assert (moved / "current").resolve() == (moved / "releases" / "1").resolve()
