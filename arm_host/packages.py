"""The card's packages pinned in the repository ([UYK], decision [iwZ]): pi-packages.lock.json names the image,
firstboot.sh's package list and every .deb's sha256. fetch_packages.sh writes it; flash_card.sh refuses a card from
packages it does not name; the on-Pi check compares what is installed with it.
  python3 -m arm_host.packages write --debs <dir>     (after a fetch; commit the lock)
  python3 -m arm_host.packages check --debs <dir>     (exit 1, naming each difference)"""
import argparse, hashlib, json, re, sys
from pathlib import Path
from urllib.parse import unquote

HERE = Path(__file__).resolve().parent
LOCK = HERE / "pi-packages.lock.json"


def firstboot_pkgs():
    """firstboot.sh's PKGS line, the one package list (fetch_packages.sh reads the same line)"""
    m = re.search(r'^PKGS="(.*)"$', (HERE / "firstboot.sh").read_text(), re.M)
    return m.group(1) if m else ""


def image_file():
    return json.loads((HERE / "pi-image.json").read_text())["file"]


def _sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""): h.update(b)
    return h.hexdigest()


def lock(debs, image, pkgs):
    return {"image": image, "packages": pkgs, "debs": {p.name: _sha(p) for p in sorted(Path(debs).glob("*.deb"))}}


def versions(lk):
    """{package: version} from the .deb names (name_version_arch.deb; an epoch's colon is %3a)"""
    out = {}
    for name in lk["debs"]:
        pkg, ver, _arch = name[:-4].split("_")
        out[pkg] = unquote(ver)
    return out


def check(debs, lk, image, pkgs):
    """each way the .debs differ from the lock; empty when they are the pinned set"""
    problems = []
    if lk.get("image") != image: problems.append(f"the lock is for image {lk.get('image')}, not {image}")
    if lk.get("packages") != pkgs: problems.append(f"the lock is for packages '{lk.get('packages')}', not '{pkgs}'")
    have = lock(debs, image, pkgs)["debs"]; want = lk.get("debs", {})
    problems += [f"{n}: not in the lock" for n in sorted(set(have) - set(want))]
    problems += [f"{n}: missing" for n in sorted(set(want) - set(have))]
    problems += [f"{n}: checksum differs from the lock" for n in sorted(set(have) & set(want)) if have[n] != want[n]]
    return problems


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("cmd", choices=["write", "check"]); ap.add_argument("--debs", required=True)
    ap.add_argument("--lock", default=str(LOCK)); a = ap.parse_args()
    if a.cmd == "write":
        Path(a.lock).write_text(json.dumps(lock(a.debs, image_file(), firstboot_pkgs()), indent=1) + "\n")
        print(f"wrote {a.lock}: commit it, so cards are made from these packages"); return
    problems = check(a.debs, json.loads(Path(a.lock).read_text()), image_file(), firstboot_pkgs())
    for p in problems: print(f"packages: {p}", file=sys.stderr)
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
