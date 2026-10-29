"""The project map ([KfL]): Barrymore's engineering on one page, made by the Railyard Method's map command with the
notes named as dated ([r5B]), then checked for what must not leave the house before anyone sees it ([NiP]): network
and hardware addresses, email addresses, and every value of real/.env and line of real/local/, found by kind and count
and never printed. Nothing git ignores is read to make it ([Nlv]); a dated document shows only its title, date and the
IDs it cites ([6nG]).

    python tools/project_map.py --out <dir> [--days 14] [--now 2026-10-03T12:00:00Z]

The method is found at HK_RAILYARD_METHOD, else the marker's `repositories.method` ([ZMl]).
"""
from __future__ import annotations

import argparse, json, os, re, shutil, subprocess, sys
from pathlib import Path

HK = Path(__file__).resolve().parents[1]
DATED = ("notes/",)
TITLE = "Houndkeeper"
ENV_MIN, LOCAL_MIN = 6, 12          # [QCx]: shorter values would match ordinary words on any page
EARLIER_NAMES = ("Edgar",)          # [EZt]: the robot is Barrymore

IPV4 = re.compile(r"(?<![\d.])(?:25[0-5]|2[0-4]\d|1?\d?\d)(?:\.(?:25[0-5]|2[0-4]\d|1?\d?\d)){3}(?![\d.])")
MAC = re.compile(r"(?<![0-9A-Fa-f:-])[0-9A-Fa-f]{2}(?:([:-])[0-9A-Fa-f]{2})(?:\1[0-9A-Fa-f]{2}){4}(?![0-9A-Fa-f:-])")
EMAIL = re.compile(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9-]+(?:\.[A-Za-z0-9-]+)*\.[A-Za-z]{2,}")
IPV6 = re.compile(r"(?<![0-9A-Fa-f:])(?:[0-9A-Fa-f]{1,4}:){3,7}[0-9A-Fa-f]{1,4}(?![0-9A-Fa-f:])")


def method_dir() -> Path:
    """[ZMl]: HK_RAILYARD_METHOD, else the marker's repositories.method"""
    if os.environ.get("HK_RAILYARD_METHOD"): return Path(os.environ["HK_RAILYARD_METHOD"]).expanduser()
    marker = json.loads((HK / ".railyard" / "method.json").read_text())
    return Path(marker["repositories"]["method"]).expanduser()


def env_values(path: Path) -> list[str]:
    """the values of a KEY=value file, unquoted, at least ENV_MIN long; read here and compared, never printed"""
    if not path.exists(): return []
    out = []
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line: continue
        v = line.split("=", 1)[1].strip().strip('"').strip("'")
        if len(v) >= ENV_MIN: out.append(v)
    return out


def local_lines(root: Path) -> list[str]:
    """each line of the text files under real/local/, at least LOCAL_MIN long once stripped"""
    if not root.is_dir(): return []
    out = []
    for f in sorted(p for p in root.rglob("*") if p.is_file() and p.stat().st_size < 1 << 20):
        try: text = f.read_text()
        except (UnicodeDecodeError, OSError): continue
        out += [s for s in (l.strip() for l in text.splitlines()) if len(s) >= LOCAL_MIN]
    return out


def published(root: Path = HK) -> str:
    """[QCx]: the text of every file git does not ignore, the repository's own; a value it already carries is not
    something the map could leak from outside it"""
    files = subprocess.run(["git", "ls-files", "-z", "--cached", "--others", "--exclude-standard"], cwd=root,
                           capture_output=True, text=True).stdout.split("\0")
    out = []
    for f in files:
        p = root / f
        if not f or not p.is_file() or p.stat().st_size > 1 << 20: continue
        try: out.append(p.read_text())
        except (UnicodeDecodeError, OSError): continue
    return "\n".join(out)


def data_text(page: str) -> str:
    """the artifacts' text the page carries, decoded from its data: what came from the repository, as a reader sees it
    (escaped JSON would hide a quote or a backslash from a plain search)"""
    m = re.search(r'<script id="map-data" type="application/json">(.*?)</script>', page, flags=re.S)
    if not m: return page
    out: list[str] = []
    def walk(x) -> None:
        if isinstance(x, str): out.append(x)
        elif isinstance(x, dict): [walk(k) or walk(v) for k, v in x.items()]
        elif isinstance(x, list): [walk(v) for v in x]
    walk(json.loads(m.group(1)))
    return "\n".join(out)


def leaks(page: str, secrets: dict[str, list[str]] | None = None, known: str = "") -> dict[str, int]:
    """[NiP]: what the page carries that must not leave, as kind -> count; never the values themselves. Addresses are
    looked for, like values, in what the page carries from the repository, decoded, and in its own words; a value the
    repository's own text already carries is not counted ([QCx])"""
    found: dict[str, int] = {}
    def hit(kind: str, n: int) -> None:
        if n: found[kind] = found.get(kind, 0) + n
    text = data_text(page) + "\n" + own_words(page)
    hit("hardware address", len(MAC.findall(text)))
    hit("network address", len(IPV4.findall(text)) + len(IPV6.findall(MAC.sub(" ", text))))
    hit("email address", len(EMAIL.findall(text)))
    grams = {text[i:i + LOCAL_MIN] for i in range(len(text) - LOCAL_MIN + 1)}
    for kind, values in (secrets or {}).items():
        hit(kind, sum(text.count(v) for v in set(values) if (len(v) < LOCAL_MIN or v[:LOCAL_MIN] in grams) and v in text
                      and v not in known))
    return found


def own_words(page: str) -> str:
    """the page with the artifacts' data taken out: what the map says of its own ([EZt])"""
    return re.sub(r'<script id="map-data" type="application/json">.*?</script>', "", page, flags=re.S)


def ignored(paths: list[str]) -> list[str]:
    """[Nlv]: of these, the paths git ignores"""
    r = subprocess.run(["git", "check-ignore", "--stdin", "-z"], cwd=HK, input="\0".join(paths) + "\0",
                       capture_output=True, text=True)
    return [p for p in r.stdout.split("\0") if p]


def make(out: Path, days: int = 14, now: str | None = None, dated: tuple[str, ...] = DATED) -> int:
    """[r5B]: make the map with the method's command, then check it ([NiP]); a failed check removes the page"""
    refused = ignored(list(dated))
    if refused: print(f"refused: git ignores {', '.join(refused)}", file=sys.stderr); return 2
    method = method_dir()
    cli = method / "bin" / "railyard-map.ts"
    if not cli.exists(): print(f"the method at {method} has no map command (bin/railyard-map.ts)", file=sys.stderr); return 2
    stage = out / f".unchecked-{os.getpid()}"          # [NiP]: nothing sits at the page's path until it is checked
    cmd = ["node", "--experimental-strip-types", str(cli), "--out", str(stage), "--days", str(days), "--title", TITLE]
    for d in dated: cmd += ["--dated", d]
    if now: cmd += ["--now", now]
    try:
        r = subprocess.run(cmd, cwd=HK, capture_output=True, text=True)
        if r.returncode != 0: print(r.stderr.strip() or r.stdout.strip(), file=sys.stderr); return r.returncode or 2
        return check_and_keep(stage / "index.html", out / "index.html")
    finally:
        shutil.rmtree(stage, ignore_errors=True)


def check_and_keep(staged: Path, page_path: Path) -> int:
    """[NiP]: the staged page checked, and moved to its path only when it carries nothing that must not leave"""
    if not staged.exists(): print("the map command wrote no page", file=sys.stderr); return 2
    page = staged.read_text()
    found = leaks(page, {"value from real/.env": env_values(HK / "real" / ".env"),
                         "line from real/local": local_lines(HK / "real" / "local")}, published())
    earlier = sum(own_words(page).count(n) for n in EARLIER_NAMES)
    if earlier: found["earlier name for the robot"] = earlier
    if found:
        staged.unlink()
        print("the map was not kept; it carried " + "; ".join(f"{n} {k}{'' if n == 1 else 's'}" for k, n in sorted(found.items())),
              file=sys.stderr)
        return 1
    os.replace(staged, page_path)
    print(f"the map is at {page_path} ({len(page) // 1024} KB), checked: no addresses, no real/.env or real/local content")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--days", type=int, default=14)
    ap.add_argument("--now", help="the moment the map is made for, e.g. 2026-10-03T12:00:00Z")
    a = ap.parse_args(argv)
    return make(a.out, a.days, a.now)


if __name__ == "__main__":
    sys.exit(main())
