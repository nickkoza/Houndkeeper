#!/usr/bin/env bash
# A numbered release of the arm host from this commit ([871], [WHV], [UYK]): prints its number; writes <out>/release.tgz.
# Usage: arm_host/build_release.sh <out dir>
set -euo pipefail
OUT=$1; ROOT=$(cd "$(dirname "$0")/.." && pwd)
N=$(git -C "$ROOT" rev-list --count HEAD)-$(git -C "$ROOT" rev-parse --short HEAD)
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
mkdir -p "$T/stage"; cp -r "$ROOT/arm_host" "$T/stage/"; cp "$ROOT/arm_host/check.sh" "$T/stage/check.sh"
find "$T/stage" -name __pycache__ -prune -exec rm -rf {} +
if [ -n "$(git -C "$ROOT" status --porcelain arm_host)" ]; then    # a dirty build is named by its content too: two of
  H=$(cd "$T/stage" && find . -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -c1-8)   # one commit never
  N="$N-dirty-$H"; fi                                                                                    # share a directory
# every file with its sha256, which the host checks the release against ([UYK], [27n], decisions [iwZ], [izZ])
(cd "$T/stage" && find . -type f ! -name MANIFEST.sha256 -print0 | LC_ALL=C sort -z | xargs -0 sha256sum > MANIFEST.sha256)
mv "$T/stage" "$T/$N"
# byte for byte the same from one commit: names sorted, the commit's time, no owner, no gzip time stamp ([UYK])
mkdir -p "$OUT"; tar -C "$T" --sort=name --mtime="@$(git -C "$ROOT" log -1 --format=%ct)" --owner=0 --group=0 \
  --numeric-owner --mode=u+rwX,go+rX,go-w -cf - "$N" | gzip -n > "$OUT/release.tgz"; echo "$N"
