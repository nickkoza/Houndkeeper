#!/usr/bin/env bash
# Fetch the photographed floors the arm sim's floor patches use (sim/arm/floor_photos.csv: ambientCG, CC0, 2K JPG) into
# models/textures/floors (gitignored), each zip pinned by its SHA-256; a changed file upstream fails the fetch.
# Usage: tools/fetch_floors.sh   (safe to re-run: skips zips already present with the right hash)
set -euo pipefail
cd "$(dirname "$0")/.."; D=models/textures/floors/zips; mkdir -p "$D"
grep -v '^#' sim/arm/floor_photos.csv | tail -n +2 | while IFS=, read -r ID KIND SIZE SRC SPLIT SHA; do
  F="$D/$ID.zip"
  if [ -f "$F" ] && [ "$(sha256sum "$F" | cut -c1-64)" = "$SHA" ]; then continue; fi
  curl -sfL -m 300 "https://ambientcg.com/get?file=${ID}_2K-JPG.zip" -o "$F"
  got=$(sha256sum "$F" | cut -c1-64)
  [ "$got" = "$SHA" ] || { echo "$ID: hash $got, wanted $SHA (changed upstream?)" >&2; rm -f "$F"; exit 1; }
  echo "$ID ($KIND, $SPLIT)"
done
echo "floors in $D: $(ls "$D" | wc -l)"
