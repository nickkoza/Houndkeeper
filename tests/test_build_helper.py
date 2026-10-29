"""The night pipeline's build helper (sim/house/lib/build.sh): a failed step stops the night and nothing from an earlier
run stands in for it ([SdA], decision [gI8]); and the gate that keeps the scripts on it."""
import re
import subprocess
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
LIB = HK / "sim" / "house" / "lib" / "build.sh"
PIPELINE = [HK / "sim/dimos/stitch_baselines.sh", HK / "sim/dimos/near_views.sh", HK / "sim/dimos/night_tidy.sh", HK / "sim/dimos/tidy_run.sh", HK / "sim/house/night_picture.sh",
            HK / "sim/house/sweep_night.sh", HK / "sim/house/real_nights.sh", HK / "sim/house/make_detect_bench.sh"]


def sh(script, cwd):
    return subprocess.run(["bash", "-c", f"set -eu; source {LIB}; {script}"], cwd=cwd, capture_output=True, text=True)


def test_a_build_that_succeeds_leaves_its_output_and_shows_its_last_line(tmp_path):
    """[SdA]"""
    r = sh("build out.txt -- bash -c 'echo noise; echo made > out.txt; echo done-line'", tmp_path)
    assert r.returncode == 0, r.stdout + r.stderr
    assert (tmp_path / "out.txt").read_text() == "made\n" and r.stdout.strip() == "done-line"


def test_a_crashed_build_fails_and_last_runs_output_is_gone(tmp_path):
    """[SdA]: the old output never stands in for a crashed rebuild"""
    (tmp_path / "out.txt").write_text("last run\n")
    r = sh("build out.txt -- bash -c 'echo half > out.txt; echo boom >&2; exit 3'; echo after", tmp_path)
    assert r.returncode != 0 and "after" not in r.stdout          # set -e: the script stops at the failed build
    assert "FAILED" in r.stdout and "boom" in r.stdout and "exit 3" in r.stdout


def test_a_build_that_exits_0_without_its_output_fails(tmp_path):
    """[SdA]: success is the output made again, not the exit code alone"""
    (tmp_path / "out.txt").write_text("last run\n")
    r = sh("build out.txt -- true || echo refused", tmp_path)
    assert "refused" in r.stdout and "not made" in r.stdout and not (tmp_path / "out.txt").exists()
    r = sh("build out.txt -- touch out.txt || echo refused", tmp_path)   # an empty output is no output
    assert "refused" in r.stdout


def test_the_pipeline_scripts_fail_a_pipeline_on_any_stage():
    """[SdA]: pipefail in every night pipeline script that runs on its own: `false | tail -1` is a failure there"""
    assert not [p.name for p in PIPELINE if p.name != "night_picture.sh"
                and not re.search(r"^set -[a-z]*o pipefail|^set -o pipefail", p.read_text(), re.M)]


def test_cleared_directories_and_stdout_outputs(tmp_path):
    """[SdA]: a solver's whole output directory goes before it runs; --stdout takes the command's output as the file"""
    d = tmp_path / "shape"; d.mkdir(); (d / "corrections_2.npz").write_text("old")
    r = sh("build --clear shape shape/corrections_2.npz -- bash -c 'exit 1' || echo refused", tmp_path)
    assert "refused" in r.stdout and not (d / "corrections_2.npz").exists()
    r = sh("build --stdout route.json -- echo '{\"waypoints\": [1]}'", tmp_path)
    assert r.returncode == 0 and (tmp_path / "route.json").read_text().strip() == '{"waypoints": [1]}'
    r = sh("build --stdout route.json -- bash -c 'echo partial; exit 2' || echo refused", tmp_path)
    assert "refused" in r.stdout and not (tmp_path / "route.json").exists()


def test_parallel_builds_are_each_waited_on(tmp_path):
    """[SdA]: one failed background build fails the wait (a bare `wait` returns 0 whatever its jobs did)"""
    r = sh("build_bg a.txt -- bash -c 'echo a > a.txt'; build_bg b.txt -- bash -c 'sleep 0.2; exit 1'; "
           "build_wait || echo refused", tmp_path)
    assert "refused" in r.stdout and (tmp_path / "a.txt").exists()
    r = sh("build_bg a.txt -- bash -c 'echo a > a.txt'; build_bg b.txt -- bash -c 'echo b > b.txt'; build_wait && echo ok", tmp_path)
    assert "ok" in r.stdout


def test_the_night_pipeline_builds_through_the_helper():
    """[SdA], [gI8]: the gate: no bare `wait`, and no python builder piped into `tail`, in the night pipeline's scripts"""
    bad = []
    for p in PIPELINE:
        for k, line in enumerate(p.read_text().splitlines(), 1):
            code = line.split(" #")[0]
            if re.search(r"(^|[;&|]\s*|\bdone;\s*)wait\s*($|;|&|\|)", code.strip()):
                bad.append(f"{p.name}:{k}: bare wait")
            if re.search(r"\.py\b[^|]*\|.*\btail\b", code):
                bad.append(f"{p.name}:{k}: a python step piped into tail")
    assert not bad, "\n".join(bad)
