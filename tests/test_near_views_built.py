"""[BW3], [Ot0]: every stitched house carries the baseline nights' near-grid views (5 mm, out to 1 m), so a live night at
the launch setting runs with its near grid on; made once per stitch, remade when the stitch is. A live sim night ran
with the near grid off (18 Oct 2026): only real_nights.sh's tonight step painted them."""
import os
import re
import subprocess
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
STUB = """#!/usr/bin/env bash
echo "$*" >> "$STUB_LOG"
while [ $# -gt 0 ]; do [ "$1" = --out ] && { echo painted > "$2"; }; shift; done
"""


def _home(tmp_path):
    """a home whose houndkeeper/ is this repository and whose DimOS python only records its calls and writes --out"""
    home = tmp_path / "home"; (home / "houndkeeper" / "dimos" / ".venv" / "bin").mkdir(parents=True)
    (home / "houndkeeper" / "houndkeeper").symlink_to(HK)
    py = home / "houndkeeper" / "dimos" / ".venv" / "bin" / "python"; py.write_text(STUB); py.chmod(0o755)
    return home


def _run(home, run, log):
    env = dict(os.environ, HOME=str(home), STUB_LOG=str(log), HK_DIMOS=str(home / "houndkeeper" / "dimos"))   # [cy4]
    env.pop("RECORDING_CLOCK", None)
    return subprocess.run(["bash", str(HK / "sim" / "dimos" / "near_views.sh"), str(run), "/r/n1.db", "/r/n2.db"],
                          env=env, capture_output=True, text=True)


def test_a_stitched_house_gets_its_near_views_once_per_stitch(tmp_path):
    home = _home(tmp_path); run = tmp_path / "run"; run.mkdir(); log = tmp_path / "calls.txt"
    (run / "stitch.args").write_text("v9 a\n")
    r = _run(home, run, log); assert r.returncode == 0, r.stdout + r.stderr
    calls = log.read_text().splitlines()
    assert len(calls) == 2 and all("baseline_views.py" in c and "--res 0.005" in c and "--rmax 1.0" in c for c in calls)
    assert {re.search(r"baseline_views\.py \S+ (\d)", c).group(1) for c in calls} == {"1", "2"}
    assert (run / "views_near_n1.npz").exists() and (run / "views_near_n2.npz").exists()
    assert (run / "views_near.args").read_text() == "v9 a\n"
    _run(home, run, log); assert len(log.read_text().splitlines()) == 2         # the same stitch: not painted again
    (run / "stitch.args").write_text("v9 b\n"); _run(home, run, log)
    assert len(log.read_text().splitlines()) == 4                                # a new stitch: painted again


def test_the_stitch_paints_them_whether_it_runs_or_was_already_done():
    """the shared builder of every house's map (night_tidy.sh's live nights and real_nights.sh) calls it on both paths"""
    src = (HK / "sim" / "dimos" / "stitch_baselines.sh").read_text()
    assert re.search(r'^\s*gate; near; echo "baselines already stitched', src, re.M)
    stamp = src.index('echo "$STAMP" > "$RUN/stitch.args"')
    assert re.search(r"^near$", src[stamp:], re.M) and "near_views.sh" in src
