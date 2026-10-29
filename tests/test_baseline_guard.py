"""A shared baseline is never re-swept unasked, and never overwritten without a copy (5 Oct 2026: a changed render
stamp silently re-swept houses 72's and 96's shared baselines under other experiments; [SdA])."""
import re
import subprocess
from pathlib import Path

SCRIPT = Path(__file__).resolve().parents[1] / "sim" / "dimos" / "night_tidy.sh"


def test_run_dir_can_be_an_experiments_own():
    assert "RUN=${RUN_DIR:-$SIM/runs/nights_s$SEED}" in SCRIPT.read_text()


def test_a_stale_baseline_is_refused_unless_fresh_is_asked_for():
    s = SCRIPT.read_text()
    guard = s[s.index("for b in 1 2; do"):]
    assert re.search(r'"\$stale" = 1 \] && \[ "\$\{FRESH_BASELINE:-0\}" != 1 \]', guard)
    assert guard.index("exit 1") < guard.index("sweep $b")


def test_keep_old_copies_the_run_directory_before_a_resweep(tmp_path):
    s = SCRIPT.read_text()
    fn = s[s.index("keep_old() {"):s.index("\nfor b in 1 2; do")]
    run = tmp_path / "nights_s1"; run.mkdir()
    (run / "mosaic_n1.npz").write_text("old picture"); (run / "nights_s1_n1.traj.csv").write_text("old path")
    out = subprocess.run(["bash", "-c", f'RUN="{run}"; archived=0\n{fn}\nkeep_old; keep_old; ls "$RUN/archive" | wc -l'],
                         capture_output=True, text=True, check=True).stdout.split()
    assert out[-1] == "1"                                    # once per run, however many nights are redone
    (snap,) = (run / "archive").iterdir()
    assert (snap / "mosaic_n1.npz").read_text() == "old picture"
    assert (snap / "nights_s1_n1.traj.csv").read_text() == "old path"
    assert "previous run directory kept" in (run / "run_notes.txt").read_text()
