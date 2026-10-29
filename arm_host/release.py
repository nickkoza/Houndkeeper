"""Releases side by side with a "current" link ([871], decision [WHV]): a release becomes current by an atomic link
switch, the service restarts, and the release's own check.sh decides; a failing release is switched back.
A power cut at any point leaves a whole release to run ([27n], decision [izZ]): the release is on disk before the link
points at it, the release that passed its check is linked as known-good, and at boot an unchecked current is undone.
Root's alone (decision [ZyY]): the tree is root's, and root runs this with isolated Python from it.
On the Pi (deploy.sh): sudo python3 -I -B /opt/houndkeeper/current/arm_host/release.py activate <n> [--root ...]
At boot (hk-release-recover.service): python3 -I -B /opt/houndkeeper/known-good/arm_host/release.py recover"""
import argparse, fcntl, hashlib, os, subprocess, sys
from pathlib import Path

ROOT = Path("/opt/houndkeeper")


def _fsync(path):
    fd = os.open(path, os.O_RDONLY)
    try: os.fsync(fd)
    finally: os.close(fd)


def _sync_tree(d):
    """every file and directory of a release on the card, so a cut after the switch never finds it half written"""
    for top, dirs, files in os.walk(d):
        for f in files:
            p = os.path.join(top, f)
            if not os.path.islink(p): _fsync(p)
        _fsync(top)


def _point(root, target, name="current"):
    tmp = root / f"{name}.tmp-{os.getpid()}"
    if tmp.is_symlink() or tmp.exists(): tmp.unlink()
    tmp.symlink_to(target); os.replace(tmp, root / name)              # rename over the old link: atomic
    _fsync(root)                                                      # and on the card before anything follows


def whole(d):
    """every file in the release's manifest is there with its checksum ([27n]); no manifest is not whole"""
    d = Path(d)
    try: lines = (d / "MANIFEST.sha256").read_text().splitlines()
    except OSError: return False
    for line in lines:
        want, _, name = line.partition("  ")
        try: got = hashlib.sha256((d / name).read_bytes()).hexdigest()
        except OSError: return False
        if got != want: return False
    return bool(lines)


def recover(root=ROOT):
    """at boot, before the daemon: a current that is not the known-good release is an activation a power cut
    interrupted before its check decided, so the known-good one comes back. Returns the release restored, or None."""
    root = Path(root); good, cur = root / "known-good", root / "current"
    if not good.is_symlink(): return None                            # made before this link, or mid first boot
    if cur.is_symlink() and cur.resolve() == good.resolve(): return None
    target = os.readlink(good); _point(root, target); return Path(target).name


def activate(root, version, restart, check_timeout=90):
    """one activation at a time (a second one raises BlockingIOError); never the release already current, which would
    leave nothing to roll back to"""
    root = Path(root)
    with open(root / ".activate.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return _activate(root, version, restart, check_timeout)


def _activate(root, version, restart, check_timeout):
    new = root / "releases" / str(version)
    if not new.is_dir(): raise FileNotFoundError(new)
    cur = root / "current"; prev = os.readlink(cur) if cur.is_symlink() else None    # restored as it was
    if cur.is_symlink() and cur.resolve() == new.resolve(): raise ValueError(f"release {version} is already current")
    _sync_tree(new); _point(root, new.relative_to(root)); restart()
    try: ok = subprocess.run([str(new / "check.sh")], timeout=check_timeout).returncode == 0
    except (OSError, subprocess.TimeoutExpired): ok = False
    if ok:
        _point(root, new.relative_to(root), "known-good"); return True
    if prev is not None: _point(root, prev)
    else: cur.unlink()
    restart(); return False


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("cmd", choices=["activate", "recover"]); ap.add_argument("version", nargs="?")
    ap.add_argument("--root", default=str(ROOT)); a = ap.parse_args()
    if a.cmd == "recover":
        back = recover(a.root)
        print(f"an activation was cut short: release {back} restored" if back else "current is the known-good release")
        return
    if not a.version: ap.error("activate needs a release")
    restart = lambda: subprocess.run(["systemctl", "restart", "hk-hostd"], check=False)
    ok = activate(a.root, a.version, restart)
    print(f"release {a.version}: {'current' if ok else 'failed its check; previous release restored'}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
