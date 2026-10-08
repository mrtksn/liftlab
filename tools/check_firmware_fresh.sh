#!/bin/sh
# Are the bundles in firmware/ built from the firmware source as it is now? Compares what the ESP32 builds compile
# (runner/'s components and the two esp32 projects) at the commit firmware/manifest.json records with this checkout.
# Exit 0: current; 1: older than the source (it lists the commits since); 2: can't tell (no such commit here: a
# shallow clone, or a commit that was never pushed).
cd "$(dirname "$0")/.."
m=$(python3 -c "import json;print(json.load(open('firmware/manifest.json'))['commit'])") || exit 2
base=${m%%+*}
[ "$base" != "$m" ] && echo "firmware/ was built from $base with changes on top (${m#*+}): it can't be matched to a commit exactly"
git cat-file -e "$base^{commit}" 2>/dev/null || { echo "firmware/ was built from $base, which this checkout doesn't have (fetch the history?)"; exit 2; }
set -- ':(glob)runner/*.c' ':(glob)runner/*.h' runner/CMakeLists.txt runner/fc runner/esp_radio runner/ground/esp32 \
  ':(glob)runner/ground/ground_*' runner/ground/rn_builtin_ground.c runner/ground/CMakeLists.txt \
  ':(exclude,glob)runner/**/test_*' ':(exclude,glob)runner/test_*' ':(exclude)runner/fc/testdata' ':(exclude,glob)runner/**/*.wasm' \
  ':(exclude,glob)runner/**/*.sh' ':(exclude)runner/fc/board_wasm.c' ':(exclude)runner/rn_wasm.c'
if git diff --quiet "$base" -- "$@"; then
  [ "$base" = "$m" ] && { echo "firmware/ is current: built from $base, and the firmware source hasn't changed since"; exit 0; }
  exit 1
fi
echo "firmware/ is older than the firmware source: built from $base; changed since:"
git log --oneline "$base"..HEAD -- "$@" | sed 's/^/  /'
git diff --quiet HEAD -- "$@" || echo "  (and uncommitted changes)"
exit 1
