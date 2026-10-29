"""[Fhn]: nothing git would carry holds the house's network: every value in real/.env (addresses and their /24
subnets, network names, MACs, keys, passwords, the router's user) is searched for in the tracked and the untracked,
unignored files. Skipped where there is no real/.env (a fresh checkout has nothing to leak)."""
import ipaddress
import re
import subprocess
from pathlib import Path

import pytest

HK = Path(__file__).resolve().parents[1]
ENV = HK / "real" / ".env"


def secrets(text):
    """the strings to look for: each value, and for an address its /24 (another house host would share it)"""
    out = set()
    for line in text.splitlines():
        line = line.split("#", 1)[0].strip()
        if "=" not in line: continue
        for v in (x.strip().strip("'\"") for x in line.split("=", 1)[1].split(",")):
            if len(v) < 6 or v.isdigit() or ("/" in v and not re.fullmatch(r"[\d.]+/\d+", v)): continue   # paths, ports
            out.add(v)
            try: ip = ipaddress.ip_address(v.split("/")[0])
            except ValueError: continue
            if ip.version == 4: out.add(".".join(str(ip).split(".")[:3]) + ".")
    return out


def carried():
    r = subprocess.run(["git", "-C", str(HK), "ls-files", "-z", "--cached", "--others", "--exclude-standard"],
                       capture_output=True, check=True)
    return [HK / p for p in r.stdout.decode().split("\0") if p]


def test_secrets_reads_values_and_subnets_not_paths():
    s = secrets("A=10.9.8.7, 10.9.9.7   # note\nB='Some Net'\nP=real/local/x\nPORT=5514\nM=aa:bb:cc:dd:ee:ff\nN=10.9.7.0/24\n")
    assert {"10.9.8.7", "10.9.8.", "10.9.9.", "Some Net", "aa:bb:cc:dd:ee:ff", "10.9.7.0/24", "10.9.7."} <= s
    assert "real/local/x" not in s and "5514" not in s


@pytest.mark.skipif(not ENV.exists(), reason="no real/.env here")
def test_no_carried_file_holds_a_value_from_the_house_env():
    found = []
    needles = [n.encode() for n in secrets(ENV.read_text())]
    for f in carried():
        if not f.is_file() or f.stat().st_size > 5_000_000: continue
        b = f.read_bytes()
        found += [f"{f.relative_to(HK)}: {n.decode()[:3]}..." for n in needles if n in b]   # never print the value
    assert not found, "house network values in files git would carry:\n" + "\n".join(found)
