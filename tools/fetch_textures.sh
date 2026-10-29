#!/usr/bin/env bash
# Fetch the photographed surface textures the indoor house uses (walls, ceiling, furniture wood, sofa fabric, worktops;
# [QMt], principle [CaM]) into models/textures (gitignored). Source: ambientCG, CC0 (public domain), 1K JPG colour maps.
# Each file is pinned by its SHA-256 so every checkout renders the same house; a changed file upstream fails the fetch.
# Usage: tools/fetch_textures.sh   (safe to re-run: skips files already present with the right hash)
set -euo pipefail
cd "$(dirname "$0")/.."; DEST=models/textures; mkdir -p "$DEST"; TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
# asset id, use, SHA-256 of its <id>_Color.jpg (checked 2026-09-25; leathers 2026-09-26)
while read -r ID USE SHA; do
  [ -z "$ID" ] && continue
  F="$DEST/${ID}_Color.jpg"
  if [ -f "$F" ] && [ "$(sha256sum "$F" | cut -c1-64)" = "$SHA" ]; then continue; fi
  curl -sfL -m 120 "https://ambientcg.com/get?file=${ID}_1K-JPG.zip" -o "$TMP/$ID.zip"
  unzip -qo "$TMP/$ID.zip" "*_Color.jpg" -d "$TMP/$ID"; cp "$TMP/$ID/"*_Color.jpg "$F"
  got=$(sha256sum "$F" | cut -c1-64)
  [ "$got" = "$SHA" ] || { echo "$ID ($USE): hash $got, wanted $SHA (changed upstream?)" >&2; rm -f "$F"; exit 1; }
  echo "$ID ($USE)"
done <<'LIST'
PaintedPlaster017 wall ed8a69a6437de9d41d2cf380f7e6a216232fc4754f256c3405985c857547ffef
Plaster001 wall d3fe43282859ab7293ad41f2fa3c29f02df077e5bc326cf90a65cc588a4ac5a4
Wood092 wood ce1a9a76fb043038eef3d6cf21d93595db3751a0d037dfbc6811eb76add4722a
Wood094 wood 25f87429708bd1137561790253c679dcc71f447f4667064c9875d1d959a910b3
Wood095 wood d33072e12e0d5b4002b691689b599ee1eb8ea5fb70eb45b4c96cc4074102197b
Fabric061 fabric de7321c185f7dfe83e71b84ad1f813364107f85f5ed2e605c820b8f7f4fe498b
Fabric081C fabric 9d3f597da359cfebd332cd668e7c57f86388c9e80d81ee665289a34963c061b9
Marble012 stone a34b55c889c6b3819d2c351b6409bfd0baf0a0a55b073441850627dfebcdcd1e
Leather011 leather 8a1c24a984dae5b8fdaf280f18f493e9cf12553ad07dcafbe32fe55da95fae42
Leather037 leather e1033ea8a95a313bdd80ea52aa04b1eee9a8934f1c695430408dacfc8636cc1a
LIST
echo "textures in $DEST: $(ls "$DEST" | wc -l)"
