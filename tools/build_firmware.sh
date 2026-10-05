#!/bin/sh
# Builds the ESP32 firmware the page flashes (Computers tab → a board → Install) into firmware/:
#   firmware/esp32-flight/   the flight controller (runner/fc/esp32)
#   firmware/esp32-ground/   the command module (runner/ground/esp32)
# Each holds the three parts at their offsets (bootloader.bin 0x1000, partition-table.bin 0x8000, the app 0x10000),
# and flash_args for esptool (write_flash @flash_args). Written at those offsets they leave the settings the board saved
# (its wiring, the airframe: the nvs partition at 0x9000) as they were. firmware/manifest.json says what was built, from
# which commit, with which ESP-IDF, and each file's size and SHA-256.
# Needs ESP-IDF 5.x (. $IDF_PATH/export.sh first). Usage: sh tools/build_firmware.sh
set -e
cd "$(dirname "$0")/.."
ROOT=$(pwd); OUT=$ROOT/firmware; B=${BUILD_DIR:-/tmp/dfb-firmware-build}
command -v idf.py >/dev/null || { echo "ESP-IDF isn't set up: . \$IDF_PATH/export.sh first"; exit 1; }
mkdir -p "$OUT"
build() {   # name, project dir, app
  name=$1; dir=$2; app=$3; bd=$B/$name
  (cd "$dir" && idf.py -B "$bd" set-target esp32 >/dev/null && idf.py -B "$bd" build >/dev/null)
  rm -rf "$OUT/esp32-$name"; mkdir -p "$OUT/esp32-$name"
  cp "$bd/bootloader/bootloader.bin" "$bd/partition_table/partition-table.bin" "$bd/$app.bin" "$OUT/esp32-$name/"
  printf '%s\n' "$(head -1 "$bd/flash_args")" "0x1000 bootloader.bin" "0x8000 partition-table.bin" "0x10000 $app.bin" > "$OUT/esp32-$name/flash_args"
  echo "built $name"
}
build flight runner/fc/esp32 dfb_flight
build ground runner/ground/esp32 dfb_ground
python - "$OUT" "$(git rev-parse --short HEAD)$(git diff --quiet HEAD -- runner || echo '+changes')" "$(idf.py --version)" <<'EOF'
import hashlib, json, os, sys, datetime
out, commit, idf = sys.argv[1], sys.argv[2], sys.argv[3]
m = {'built': datetime.date.today().isoformat(), 'commit': commit, 'idf': idf, 'chip': 'esp32', 'firmware': {}}
for name, app in (('flight', 'dfb_flight'), ('ground', 'dfb_ground')):
    d = os.path.join(out, 'esp32-' + name); files = {}
    for f in ('bootloader.bin', 'partition-table.bin', app + '.bin'):
        b = open(os.path.join(d, f), 'rb').read(); files[f] = {'size': len(b), 'sha256': hashlib.sha256(b).hexdigest()}
    m['firmware'][name] = {'dir': 'esp32-' + name, 'parts': [[0x1000, 'bootloader.bin'], [0x8000, 'partition-table.bin'], [0x10000, app + '.bin']], 'files': files}
json.dump(m, open(os.path.join(out, 'manifest.json'), 'w'), indent=1)
print('wrote firmware/manifest.json:', commit, idf)
EOF
