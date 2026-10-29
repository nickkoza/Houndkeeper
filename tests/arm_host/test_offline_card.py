"""[DzZ], [XzS]: the Pi never goes online (plugged in, it is ready to go): its first boot
installs only from the card and stops rather than reach for the internet; the laptop fetches the packages, checked
against the image's own archive keys; the card is never made without them."""
import re
import subprocess
from pathlib import Path

ARM_HOST = Path(__file__).resolve().parents[2] / "arm_host"


def _code(name):
    return "\n".join(l.split("#", 1)[0] for l in (ARM_HOST / name).read_text().splitlines())   # comments aside


def test_first_boot_installs_only_from_the_card():
    code = _code("firstboot.sh")
    assert "apt-get update" not in code and "curl" not in code and "wget" not in code
    installs = re.findall(r"apt-get install[^\n]*", code)
    assert installs and all("--no-download" in i for i in installs)


def test_first_boot_without_packages_stops_offline(tmp_path):
    """the branch with no packages on the card fails before anything else runs (the script cut at its first mask)"""
    code = (ARM_HOST / "firstboot.sh").read_text()
    branch = code[code.index('if [ "$OFFLINE" = 1 ]'): code.index("systemctl mask")]
    out = subprocess.run(["bash", "-c", "B=" + str(tmp_path) + "; OFFLINE=1; PKGS=x\n" + branch + "\necho went-on"],
                         capture_output=True, text=True)
    assert out.returncode == 1 and "FAILED: no packages on the card" in out.stdout and "went-on" not in out.stdout


def test_the_card_always_carries_its_packages_and_fetching_checks_signatures():
    card, fetch = _code("flash_card.sh"), _code("fetch_packages.sh")
    assert "fetch_packages.sh" in card and "OFF=--offline" in card and "not making a card" in (ARM_HOST / "flash_card.sh").read_text()
    assert "/usr/share/keyrings" in fetch and "--download-only" in fetch and "AllowUnauthenticated" not in fetch
    assert "AllowInsecureRepositories" not in fetch and "trusted=yes" not in fetch
    assert 'sed -n \'s/^PKGS="' in fetch                                    # one package list: firstboot.sh's
