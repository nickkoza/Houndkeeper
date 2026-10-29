#!/usr/bin/env bash
# Fetch the scanned household items the item library ([QxY]) uses into models/items/gso (gitignored; ~3 MB per item).
# Source: github.com/kevinzakka/mujoco_scanned_objects (MIT), MuJoCo models of Google Scanned Objects (CC-BY 4.0),
# pinned to COMMIT so every checkout gets identical models. The list of items comes from sim/arm/items.py (GSO_NAMES).
# Usage: tools/fetch_items.sh   (safe to re-run: it replaces models/items/gso)
set -euo pipefail
COMMIT=6ff8d275cebfd5b47e49685e3cfbe64b20e49a3c
HK=$(cd "$(dirname "$0")/.." && pwd); WS=${HK_WORKSPACE:-$(dirname "$HK")}   # the workspace: decision [cy4]
PY=${PY:-${HK_DIMOS:-$WS/dimos}/.venv/bin/python}       # any Python with numpy, mujoco and Pillow (the DimOS venv)
cd "$(dirname "$0")/.."; DEST=models/items/gso; TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
ITEMS=$("$PY" -c "import sys; sys.path.insert(0, 'sim/arm'); import items; print(' '.join(items.GSO_NAMES))")
git init -q "$TMP/mso"; cd "$TMP/mso"
git remote add origin https://github.com/kevinzakka/mujoco_scanned_objects
git config core.sparseCheckout true; printf 'models/%s/\n' $ITEMS > .git/info/sparse-checkout
git fetch -q --depth 1 --filter=blob:none origin "$COMMIT"; git checkout -q FETCH_HEAD
[ "$(git rev-parse HEAD)" = "$COMMIT" ] || { echo "got $(git rev-parse HEAD), wanted $COMMIT" >&2; exit 1; }
cd - >/dev/null
missing=$(for i in $ITEMS; do [ -f "$TMP/mso/models/$i/model.xml" ] || echo "$i"; done)
[ -z "$missing" ] || { echo "not in the source at $COMMIT: $missing" >&2; exit 1; }
rm -rf "$DEST"; mkdir -p "$DEST"; cp -r "$TMP/mso/models/"* "$DEST/"; echo "$COMMIT" > "$DEST/SOURCE_COMMIT"
"$PY" -c "
from PIL import Image; import glob
for f in glob.glob('$DEST/*/texture.png'):
    im = Image.open(f); s = min(1.0, 1024 / max(im.size))
    (im.resize((int(im.size[0] * s), int(im.size[1] * s))) if s < 1 else im).save(f.replace('texture.png', 'texture_1k.png'))"
echo "fetched $(ls -d $DEST/*/ | wc -l) items at $COMMIT into $DEST (with 1024 px texture copies)"
