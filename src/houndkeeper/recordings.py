"""Recordings and the data they hold ([HqQ]): one catalog of every DimOS recording, real and sim, with what it is,
when, why, and under which code and parameters ([xOp]); finding and using them by id ([5zg]); real ones moved into
the project's data store ([u6u]) and copied, verified, to the house's server ([Zw9], [X7i]); sim ones pruned when
nothing refers to them ([B2O], [YP8]); people screened before footage is used ([R1X]); integrity checked ([T9e]);
nothing outside the house ([fya]).

    hk-data scan                         catalog the sim recordings DimOS has written (no hashing unless --hash)
    hk-data add <memory.db> --real --purpose "..." [--tag t ...] [--firmware v] [--note "..."]
    hk-data ls [--real|--sim] [--tag t] [--purpose text] [--since 2026-10-01] [--house 95]
    hk-data show <id>        hk-data path <id>        hk-data tag <id> t ...        hk-data note <id> "..."
    hk-data screened <id> clear|people [--report file]
    hk-data sync [--dest user@host:/path]            hk-data verify [<id> ...]
    hk-data prune [--apply]                          hk-data drop-local <id>   (only with a verified copy elsewhere)
    hk-data record [--real] --purpose "..." [--tag t ...] [--firmware v] -- <dimos run arguments: blueprints, flags>
"""
from __future__ import annotations

import argparse, hashlib, ipaddress, json, os, re, shlex, shutil, socket, sqlite3, subprocess, sys, time
from contextlib import closing
from pathlib import Path

from houndkeeper import workspace

HK = Path(__file__).resolve().parents[2]
DIMOS = workspace.dimos()
ROOT = Path(os.environ.get("HK_DATA_ROOT", HK / "data" / "recordings"))
DIMOS_REC = DIMOS / "recordings"
RUN_ID = re.compile(r"^(\d{8}-\d{6})-(.+)$")
SAFE_ID = re.compile(r"^[A-Za-z0-9._-]+$")    # a recording id: a folder name, never a shell word or an option
REMOTE_DEST = re.compile(r"^(?:[A-Za-z0-9._-]+@)?[A-Za-z0-9][A-Za-z0-9.-]*:/[A-Za-z0-9._/-]*$")   # user@host:/path, plainly
REAL_ONLY_STREAMS = ("battery",)       # streams only the real robot has (the sim has no low state, [4hm])
SIM_MARKER = "houndkeeper-sim"          # left in a recording's folder by the sim scripts when DimOS stops: a sim run with
                                       # the phone cockpit carries a battery stream too, and was cataloged as real (3 Oct 2026)
SCHEMA = """
create table if not exists recordings (
  id text primary key, kind text not null, path text not null, blueprint text, start real, end real, duration_s real,
  size integer, sha256 text, streams text, firmware text, hk_commit text, fork_patch_sha text, params_sha text,
  purpose text default '', notes text default '', house text, screening text not null, added real not null);
create table if not exists tags (id text not null, tag text not null, primary key (id, tag));
create table if not exists copies (id text not null, location text not null, path text not null, sha256 text,
  verified real, primary key (id, location));
"""


def catalog_path() -> Path:
    return ROOT / "catalog.sqlite"


def db() -> sqlite3.Connection:
    ROOT.mkdir(parents=True, exist_ok=True)
    c = sqlite3.connect(catalog_path()); c.row_factory = sqlite3.Row; c.executescript(SCHEMA); return c


def sha256_of(path: Path, chunk: int = 1 << 22) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while b := f.read(chunk): h.update(b)
    return h.hexdigest()


def read_recording(path: Path) -> dict:
    """what the recording says of itself: its streams (DimOS's _streams) with each one's count, and its span"""
    out, start, end = {}, None, None
    with closing(sqlite3.connect(f"file:{path}?mode=ro", uri=True)) as c:
        names = [r[0] for r in c.execute("select name from _streams")]
        for n in names:
            try: lo, hi, k = c.execute(f'select min(ts), max(ts), count(*) from "{n}"').fetchone()
            except sqlite3.Error: continue
            out[n] = k
            if lo is not None: start = lo if start is None else min(start, lo); end = hi if end is None else max(end, hi)
    return dict(streams=out, start=start, end=end, duration_s=None if start is None else round(end - start, 1))


def _git(*args: str, cwd: Path = HK) -> str:
    try: return subprocess.run(["git", *args], cwd=cwd, capture_output=True, text=True, timeout=20).stdout.strip()
    except Exception: return ""


def provenance() -> dict:
    """the code and parameters in force now: houndkeeper's commit (marked dirty), the fork patch's hash, the params' hash"""
    commit = _git("rev-parse", "--short", "HEAD"); dirty = bool(_git("status", "--porcelain", "--untracked-files=no"))
    patch = HK / "sim" / "dimos" / "dimos_fork_changes.patch"; params = HK / "params" / "robot.yaml"
    return dict(hk_commit=commit + ("+dirty" if dirty and commit else ""),
                fork_patch_sha=sha256_of(patch)[:16] if patch.exists() else None,
                params_sha=sha256_of(params)[:16] if params.exists() else None)


def run_id_of(path: Path) -> str:
    return path.parent.name if path.name.endswith((".db", ".mcap")) else path.name


def house_of(run_id: str) -> str | None:
    """the sim house a run was recorded in, from the night pipeline's seed lists that name it"""
    for f in (HK / "sim" / "house" / "runs").glob("nights_s*/seed_recordings.txt"):
        try:
            if run_id in f.read_text(): return f.parent.name.split("_s")[1].split("_")[0]
        except OSError: pass
    return None


def add(path: Path, real: bool, purpose: str = "", tags: tuple[str, ...] = (), firmware: str | None = None,
        note: str = "", hash_it: bool | None = None, move: bool | None = None) -> str:
    """catalog one recording ([xOp]); a real one is hashed, moved into the data store with a copy of the parameters
    in force, and starts unscreened ([u6u], [R1X])"""
    path = Path(path).resolve(); rid = run_id_of(path); m = RUN_ID.match(rid)
    hash_it = real if hash_it is None else hash_it; move = real if move is None else move
    if not SAFE_ID.match(rid) or rid.startswith("-"):
        raise ValueError(f"{rid!r} is not a recording id (letters, digits, '.', '_' and '-' only)")
    info = read_recording(path)
    if not real and not (path.parent / SIM_MARKER).exists() and any(s in info["streams"] for s in REAL_ONLY_STREAMS):
        raise ValueError(f"{rid} carries the robot's own streams ({', '.join(REAL_ONLY_STREAMS)}): a real recording; add it with --real")
    if move:
        # only a DimOS run folder (<date>-<time>-<blueprint>, nothing but memory.db*) is moved, so a stray memory.db
        # in a busy folder cannot take its neighbours with it
        others = [f.name for f in path.parent.iterdir() if not (f.is_file() and f.name.startswith(path.name))]
        if not m or others or not path.name.endswith(".db"):
            raise ValueError(f"{path.parent} is not a DimOS run folder (<date>-<time>-<blueprint>/memory.db, nothing else)"
                             + (f": also holds {', '.join(sorted(others)[:5])}" if others else ""))
        dest = ROOT / "real" / rid; dest.mkdir(parents=True, exist_ok=True)
        for f in list(path.parent.iterdir()): shutil.move(str(f), dest / f.name)
        try: path.parent.rmdir()
        except OSError: pass
        path = dest / path.name
        params = HK / "params" / "robot.yaml"
        if params.exists(): shutil.copy2(params, dest / "params-robot.yaml")
    prov = provenance()
    with closing(db()) as c, c:
        c.execute("""insert or replace into recordings (id, kind, path, blueprint, start, end, duration_s, size, sha256,
            streams, firmware, hk_commit, fork_patch_sha, params_sha, purpose, notes, house, screening, added)
            values (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                  (rid, "real" if real else "sim", str(path), m.group(2) if m else None, info["start"], info["end"],
                   info["duration_s"], path.stat().st_size, sha256_of(path) if hash_it else None, json.dumps(info["streams"]),
                   firmware, prov["hk_commit"], prov["fork_patch_sha"], prov["params_sha"], purpose, note,
                   None if real else house_of(rid), "unscreened" if real else "n/a", time.time()))
        c.executemany("insert or ignore into tags values (?, ?)", [(rid, t) for t in tags])
        c.execute("insert or replace into copies values (?, 'laptop', ?, ?, ?)",
                  (rid, str(path), None, None))
    return rid


def scan(hash_it: bool = False, min_age_s: float = 0.0) -> list[str]:
    """catalog every recording DimOS has written that the catalog lacks: sim ones (and any the sim scripts marked, SIM_MARKER),
    and any other that carries a stream only the real robot has, as real (moved into the store, unscreened, never pruned); runs written to in the last
    min_age_s are left for later (still recording)"""
    with closing(db()) as c: known = {r[0] for r in c.execute("select id from recordings")}
    added = []; now = time.time()
    for p in sorted(DIMOS_REC.glob("*/memory.db")):
        if p.parent.name in known: continue
        if min_age_s and any(now - f.stat().st_mtime < min_age_s for f in p.parent.iterdir()): continue
        real = not (p.parent / SIM_MARKER).exists() and any(s in read_recording(p)["streams"] for s in REAL_ONLY_STREAMS)
        try: added.append(add(p, real=real, hash_it=hash_it or None))
        except sqlite3.Error as e: print(f"skipped {p.parent.name}: {e}", file=sys.stderr)
    return added


def rows(real: bool | None = None, tag: str | None = None, purpose: str | None = None, since: str | None = None,
         house: str | None = None) -> list[sqlite3.Row]:
    q, a = "select * from recordings where 1=1", []
    if real is not None: q += " and kind=?"; a.append("real" if real else "sim")
    if tag: q += " and id in (select id from tags where tag=?)"; a.append(tag)
    if purpose: q += " and purpose like ?"; a.append(f"%{purpose}%")
    if since: q += " and start >= ?"; a.append(time.mktime(time.strptime(since, "%Y-%m-%d")))
    if house: q += " and house=?"; a.append(str(house))
    with closing(db()) as c: return c.execute(q + " order by start desc", a).fetchall()


def get(rid: str) -> sqlite3.Row:
    with closing(db()) as c:
        r = c.execute("select * from recordings where id=? or id like ? order by id", (rid, rid + "%")).fetchall()
    if len(r) != 1: raise SystemExit(f"{'no' if not r else 'more than one'} recording matches {rid!r}")
    return r[0]


def path_of(rid: str) -> Path:
    """a recording's memory.db by id ([5zg]): the laptop's copy"""
    return Path(get(rid)["path"])


def require_screened(rid: str) -> None:
    """for anything that exports frames: refuses an unscreened or flagged real recording ([R1X])"""
    s = get(rid)["screening"]
    if s not in ("clear", "n/a"): raise PermissionError(f"{rid} is {s}: screen it for people before using its frames (tools/screen_people.py)")


def in_house(dest: str) -> bool:
    """a sync destination on the house's own machines: a local path, or a host that resolves to a private address ([fya])"""
    if dest.startswith("-"): return False                              # rsync and ssh would read it as an option
    if ":" not in dest or dest.startswith("/"): return True
    if not REMOTE_DEST.match(dest): return False                       # user@host:/path and nothing else (no ; spaces $)
    host = dest.split(":", 1)[0].split("@")[-1]
    try: addrs = {ai[4][0] for ai in socket.getaddrinfo(host, None)}
    except socket.gaierror: return False
    ips = [ipaddress.ip_address(a) for a in addrs]                   # the house's LAN: private, not the tailnet's 100.64/10
    return bool(ips) and all(ip.is_private or ip.is_loopback for ip in ips)


def server_dest() -> str | None:
    env = HK / "real" / ".env"
    if env.exists():
        for ln in env.read_text().splitlines():
            if ln.startswith("DATA_SERVER="): return ln.split("=", 1)[1].split("#")[0].strip() or None
    return os.environ.get("HK_DATA_SERVER")


def _remote_sha(dest: str, rel: str) -> str | None:
    if not dest: return None                                          # no destination: never a path relative to here
    if ":" in dest and not dest.startswith("/"):
        host, base = dest.split(":", 1)
        # ssh joins its arguments into one shell line on the server: the path is quoted, so an id cannot be a command
        r = subprocess.run(["ssh", "--", host, "sha256sum", "--", shlex.quote(f"{base.rstrip('/')}/{rel}")], capture_output=True, text=True, timeout=3600)
        return r.stdout.split()[0] if r.returncode == 0 and r.stdout else None
    p = Path(dest) / rel
    return sha256_of(p) if p.is_absolute() and p.exists() else None


def _sha_of_copy(where: str) -> str | None:
    """the hash of one recorded copy, where sync put it ('host:/path/<id>/memory.db' or an absolute local path)"""
    if ":" in where and not where.startswith("/"):
        host, path = where.split(":", 1); return _remote_sha(f"{host}:{str(Path(path).parent.parent)}", "/".join(Path(path).parts[-2:]))
    return _remote_sha(str(Path(where).parent.parent), "/".join(Path(where).parts[-2:]))


def sync(dest: str | None = None) -> list[str]:
    """copy every real recording, and every sim one tagged keep (a demo film's source, say), to the server and verify it
    there ([Zw9]); lists what waits when there is none"""
    dest = dest or server_dest()
    real = rows(real=True) + rows(real=False, tag="keep")
    if not dest:
        print(f"no server set (DATA_SERVER in real/.env): {len(real)} recording(s) to keep, single-copy"); return []
    if dest.startswith("-"): raise SystemExit(f"refusing {dest!r}: not a destination")
    if ":" not in dest or dest.startswith("/"): dest = str(Path(dest).resolve())
    if not in_house(dest): raise SystemExit(f"refusing {dest}: not one of the house's own machines ([fya])")
    done = []
    for r in real:
        src = Path(r["path"]).parent; rel = f"{r['id']}/{Path(r['path']).name}"
        target = f"{dest.rstrip('/')}/{r['id']}/"
        if ":" not in dest or dest.startswith("/"): Path(target).mkdir(parents=True, exist_ok=True)
        subprocess.run(["rsync", "-a", "--partial", "--protect-args", "--", f"{src}/", target], check=True)
        sha = r["sha256"] or sha256_of(Path(r["path"]))
        got = _remote_sha(dest, rel)
        with closing(db()) as c, c:
            c.execute("update recordings set sha256=? where id=?", (sha, r["id"]))
            c.execute("insert or replace into copies values (?, 'server', ?, ?, ?)",
                      (r["id"], f"{dest.rstrip('/')}/{rel}", got, time.time() if got == sha else None))
        print(f"{r['id']}: {'verified' if got == sha else 'MISMATCH'}"); done.append(r["id"])
    return done


def verify(ids: list[str] | None = None) -> list[str]:
    """re-hash every copy against the catalog ([T9e]); returns the problems"""
    problems = []
    with closing(db()) as c:
        targets = ids or [r[0] for r in c.execute("select id from recordings where sha256 is not null")]
        for rid in targets:
            r = get(rid)
            for cp in c.execute("select * from copies where id=?", (r["id"],)).fetchall():
                got = sha256_of(Path(cp["path"])) if cp["location"] == "laptop" and Path(cp["path"]).exists() else \
                    (_sha_of_copy(cp["path"]) if cp["location"] == "server" else None)
                if got is None: problems.append(f"{r['id']} {cp['location']}: missing")
                elif r["sha256"] and got != r["sha256"]: problems.append(f"{r['id']} {cp['location']}: changed")
    return problems


def references() -> str:
    """everything that can name a recording: the run dirs' text files, notes, specs and the films' prep data (HK_DEMOS_DATA)"""
    out = []
    roots = [HK / "sim" / "house" / "runs", HK / "data", HK / "notes", HK / "docs"]
    if os.environ.get("HK_DEMOS_DATA"): roots.append(Path(os.environ["HK_DEMOS_DATA"]))   # the films' prep data, if any
    for root in roots:
        if not root.exists(): continue
        for p in root.rglob("*"):
            if p.is_file() and p.suffix in (".txt", ".json", ".md", ".csv", ".args", ".sh") and p.stat().st_size < 5_000_000:   # not logs: a log names every run it saw
                try: out.append(p.read_text(errors="ignore"))
                except OSError: pass
    return "\n".join(out)


def prunable() -> list[sqlite3.Row]:
    """sim recordings nothing refers to ([B2O]); real ones are never listed ([YP8])"""
    text = references()
    return [r for r in rows(real=False) if r["id"] not in text and not _tagged(r["id"], "keep")]


def _tagged(rid: str, tag: str) -> bool:
    with closing(db()) as c: return c.execute("select 1 from tags where id=? and tag=?", (rid, tag)).fetchone() is not None


def prune(apply: bool = False) -> tuple[int, int]:
    gone, freed = 0, 0
    for r in prunable():
        p = Path(r["path"]); size = sum(f.stat().st_size for f in p.parent.iterdir()) if p.parent.exists() else 0
        if apply and p.parent.exists() and p.parent.parent == DIMOS_REC:
            shutil.rmtree(p.parent)
            with closing(db()) as c, c: c.execute("delete from recordings where id=?", (r["id"],)); c.execute("delete from copies where id=?", (r["id"],))
        gone += 1; freed += size
    return gone, freed


def drop_local(rid: str) -> None:
    """remove the laptop's copy of a real recording: only with a verified server copy ([Zw9])"""
    r = get(rid)
    with closing(db()) as c:
        cp = c.execute("select * from copies where id=? and location='server'", (r["id"],)).fetchone()
        laptop = c.execute("select * from copies where id=? and location='laptop'", (r["id"],)).fetchone()
    if laptop is None or not Path(laptop["path"]).exists():
        raise SystemExit(f"{rid}: no laptop copy to remove")
    here = Path(laptop["path"]).resolve()
    if here.parent.parent != (ROOT / "real").resolve():
        raise SystemExit(f"{rid}: the laptop copy is not in the data store ({here}); not removing")
    if cp is None or cp["verified"] is None or cp["sha256"] != r["sha256"]:
        raise SystemExit(f"{rid}: no verified server copy; run hk-data sync first")
    if not (":" in cp["path"] and not cp["path"].startswith("/")) and Path(cp["path"]).exists() and Path(cp["path"]).resolve() == here:
        raise SystemExit(f"{rid}: the 'server' copy is the laptop's own file; not removing")
    if _sha_of_copy(cp["path"]) != r["sha256"]:
        raise SystemExit(f"{rid}: the server's copy no longer matches (or cannot be reached); not removing")
    shutil.rmtree(here.parent)
    with closing(db()) as c, c:
        c.execute("delete from copies where id=? and location='laptop'", (r["id"],))
        c.execute("update recordings set path=? where id=?", (cp["path"], r["id"]))


FREE_FLOOR_GB = 50.0                   # a session refuses to start with less free than this where DimOS records
ISOLATION_MAX_AGE_S = 3600.0           # and, on the real robot, without an isolation check passed in the last hour


def preflight(real: bool, now: float | None = None) -> list[str]:
    """why a session must not start ([6xc]): too little disk, or (real) no recent passing isolation check"""
    now = time.time() if now is None else now; why = []
    DIMOS_REC.mkdir(parents=True, exist_ok=True)
    free = shutil.disk_usage(DIMOS_REC).free / 1e9
    if free < FREE_FLOOR_GB: why.append(f"only {free:.0f} GB free where DimOS records (floor {FREE_FLOOR_GB:.0f}): hk-data prune")
    if real:
        iso = HK / "real" / "local" / "isolation.json"
        if not iso.exists(): why.append("no isolation check on record (real/bringup/isolation_check.py)")
        else:
            try:
                d = json.loads(iso.read_text()); summary = str(d.get("summary", ""))
                age = now - iso.stat().st_mtime
                if not summary.startswith("passed"): why.append(f"the robot network's isolation check failed: {summary}")
                elif age > ISOLATION_MAX_AGE_S: why.append(f"the isolation check is {age / 60:.0f} min old: run it again")
            except (OSError, ValueError) as e: why.append(f"isolation check unreadable: {e}")
    return why


def session(real: bool, purpose: str, tags: tuple[str, ...], firmware: str | None, dimos_args: list[str]) -> list[str]:
    """a recorded DimOS run that catalogs what it recorded when it ends ([6xc]): the real robot's runs carry the
    battery module, so the power procedure can use them"""
    why = preflight(real)
    if why: raise SystemExit("not starting:\n  " + "\n  ".join(why))
    if not purpose: raise SystemExit("say what the session is for: --purpose")
    args = list(dimos_args)
    if real and "houndkeeper.battery" not in args:
        i = args.index("run") + 2 if "run" in args else len(args); args.insert(i, "houndkeeper.battery")
    before = {p.parent.name for p in DIMOS_REC.glob("*/memory.db")}; t0 = time.time()
    cmd = [str(DIMOS / ".venv" / "bin" / "dimos"), "--record", "sqlite", *args]
    print("recording:", " ".join(cmd), flush=True)
    try: subprocess.run(cmd, cwd=DIMOS)
    except KeyboardInterrupt: pass
    added = []
    for p in sorted(DIMOS_REC.glob("*/memory.db")):
        if p.parent.name in before or p.stat().st_mtime < t0: continue
        added.append(add(p, real=real, purpose=purpose, tags=tags, firmware=firmware))
    print("cataloged:", ", ".join(added) or "nothing (no recording written)")
    return added


def fmt_row(r: sqlite3.Row) -> str:
    when = time.strftime("%Y-%m-%d %H:%M", time.localtime(r["start"])) if r["start"] else "-"
    size = f"{r['size'] / 1e9:.2f} GB" if r["size"] else "-"
    return f"{r['id']:<58} {r['kind']:<4} {when:<16} {(r['duration_s'] or 0) / 60:6.1f} min {size:>9}  {r['house'] or '':<4} {r['purpose'][:40]}"


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(prog="hk-data", description=__doc__.split("\n")[0]); sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("scan"); s.add_argument("--hash", action="store_true"); s.add_argument("--min-age", type=float, default=300.0)
    s = sub.add_parser("add"); s.add_argument("db"); s.add_argument("--real", action="store_true"); s.add_argument("--purpose", default="")
    s.add_argument("--tag", action="append", default=[]); s.add_argument("--firmware"); s.add_argument("--note", default="")
    s = sub.add_parser("ls"); g = s.add_mutually_exclusive_group(); g.add_argument("--real", action="store_true"); g.add_argument("--sim", action="store_true")
    s.add_argument("--tag"); s.add_argument("--purpose"); s.add_argument("--since"); s.add_argument("--house")
    sub.add_parser("show").add_argument("id")
    s = sub.add_parser("path"); s.add_argument("id")
    s.add_argument("--numbers-only", action="store_true", help="an unscreened real recording for an analysis that shows and keeps no pixels")
    s = sub.add_parser("tag"); s.add_argument("id"); s.add_argument("tags", nargs="+")
    s = sub.add_parser("note"); s.add_argument("id"); s.add_argument("text")
    s = sub.add_parser("screened"); s.add_argument("id"); s.add_argument("result", choices=["clear", "people"]); s.add_argument("--report")
    s = sub.add_parser("sync"); s.add_argument("--dest")
    s = sub.add_parser("verify"); s.add_argument("ids", nargs="*")
    s = sub.add_parser("prune"); s.add_argument("--apply", action="store_true")
    sub.add_parser("drop-local").add_argument("id")
    s = sub.add_parser("record"); s.add_argument("--real", action="store_true"); s.add_argument("--purpose", default="")
    s.add_argument("--tag", action="append", default=[]); s.add_argument("--firmware"); s.add_argument("dimos", nargs=argparse.REMAINDER)
    a = ap.parse_args(argv)
    if a.cmd == "scan": print(f"cataloged {len(scan(a.hash, a.min_age))} recording(s)")
    elif a.cmd == "add": print(add(Path(a.db), a.real, a.purpose, tuple(a.tag), a.firmware, a.note))
    elif a.cmd == "ls":
        for r in rows(True if a.real else False if a.sim else None, a.tag, a.purpose, a.since, a.house): print(fmt_row(r))
    elif a.cmd == "show":
        r = get(a.id); d = dict(r); d["streams"] = json.loads(d["streams"] or "{}")
        with closing(db()) as c:
            d["tags"] = [t[0] for t in c.execute("select tag from tags where id=?", (r["id"],))]
            d["copies"] = [dict(x) for x in c.execute("select location, path, verified from copies where id=?", (r["id"],))]
        print(json.dumps(d, indent=1, default=str))
    elif a.cmd == "path":
        if not a.numbers_only: require_screened(a.id)                 # [R1X]: the frames go nowhere until screened
        print(path_of(a.id))
    elif a.cmd == "tag":
        rid = get(a.id)["id"]
        with closing(db()) as c, c: c.executemany("insert or ignore into tags values (?, ?)", [(rid, t) for t in a.tags])
    elif a.cmd == "note":
        r = get(a.id)
        with closing(db()) as c, c: c.execute("update recordings set notes=? where id=?", ((r["notes"] + "\n" if r["notes"] else "") + a.text, r["id"]))
    elif a.cmd == "screened":
        r = get(a.id)
        with closing(db()) as c, c:
            c.execute("update recordings set screening=?, notes=? where id=?", (a.result, (r["notes"] + "\n" if r["notes"] else "") + f"screened {time.strftime('%Y-%m-%d')}: {a.result}" + (f" ({a.report})" if a.report else ""), r["id"]))
    elif a.cmd == "sync": sync(a.dest)
    elif a.cmd == "verify":
        p = verify(a.ids or None); print("\n".join(p) if p else "all copies match")
    elif a.cmd == "prune":
        n, b = prune(a.apply); print(f"{'removed' if a.apply else 'would remove'} {n} unreferenced sim recording(s), {b / 1e9:.1f} GB")
    elif a.cmd == "drop-local": drop_local(a.id); print("removed the laptop's copy")
    elif a.cmd == "record": session(a.real, a.purpose, tuple(a.tag), a.firmware, [x for x in a.dimos if x != "--"])


if __name__ == "__main__":
    main()
