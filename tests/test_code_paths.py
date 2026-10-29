"""Code runs from the checkout it is in ([SdA]: nothing from elsewhere stands in): no Python file names this machine's
checkout, so a worktree's scripts import the worktree's modules, not the main checkout's (3 Oct 2026: 39 files did, and a
worktree's tests silently ran the main checkout's go1_arm); and the sim hook's per-frame viewer adds its path once."""
import subprocess
import sys
from pathlib import Path

HK = Path(__file__).resolve().parents[1]


def test_no_python_file_names_a_checkout_by_its_absolute_path():
    files = subprocess.run(["git", "-C", str(HK), "ls-files", "--cached", "--others", "--exclude-standard", "*.py"],
                           capture_output=True, text=True, check=True).stdout.split()
    home = str(Path.home())
    found = [f for f in files if (HK / f).is_file() and f"{home}/houndkeeper/houndkeeper" in (HK / f).read_text(errors="ignore")]
    assert not found, f"absolute checkout paths in: {found}"


def test_the_viewer_adds_the_arm_path_once():
    sys.path.insert(0, str(HK / "src"))
    from houndkeeper import sim_hook as sh
    sh._arm_path(); n = sys.path.count(str(sh.ARM_SIM))       # (other tests add it too: count what these calls add)
    for _ in range(50): sh._arm_path()
    assert n >= 1 and sys.path.count(str(sh.ARM_SIM)) == n
