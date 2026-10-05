#!/bin/sh
# Build all supported chips/roles outside the checkout; derive offsets from ESP-IDF.
# . $IDF_PATH/export.sh && sh tools/build_firmware.sh
set -e
cd "$(dirname "$0")/.."
ROOT=$(pwd); OUT=$ROOT/firmware; B=${BUILD_DIR:-/tmp/liftlab-firmware-build}
command -v idf.py >/dev/null || { echo "ESP-IDF isn't set up: . \$IDF_PATH/export.sh first"; exit 1; }
mkdir -p "$OUT"
for chip in esp32 esp32s3 esp32c3; do
  for role in flight ground; do
    if [ "$role" = flight ]; then src=runner/fc/esp32; else src=runner/ground/esp32; fi
    bd=$B/$chip-$role
    mkdir -p "$bd"
    (cd "$src" && idf.py -B "$bd" -D SDKCONFIG="$bd/sdkconfig" -D IDF_TARGET="$chip" build > "$bd/build.log" 2>&1) || { tail -60 "$bd/build.log"; exit 1; }
    python3 - "$bd" "$OUT/$chip-$role" <<'PY'
import json,pathlib,shutil,sys
bd,out=map(pathlib.Path,sys.argv[1:]); out.mkdir(parents=True,exist_ok=True)
m=json.loads((bd/'flasher_args.json').read_text())
lines=[]
settings=m['flash_settings']
lines.append('--flash_mode '+settings['flash_mode']+' --flash_freq '+settings['flash_freq']+' --flash_size '+settings['flash_size'])
for addr,name in m['flash_files'].items():
    target=pathlib.Path(name).name
    shutil.copyfile(bd/name,out/target); lines.append(addr+' '+target)
(out/'flash_args').write_text('\n'.join(lines)+'\n')
(out/'parts.json').write_text(json.dumps([[int(a,0),pathlib.Path(n).name] for a,n in m['flash_files'].items()]))
PY
    echo "built $chip $role"
  done
done
python3 - "$OUT" "$(git rev-parse --short HEAD)$(git diff --quiet HEAD -- runner || echo '+changes')" "$(idf.py --version)" <<'PY'
import hashlib,json,pathlib,sys,datetime
out=pathlib.Path(sys.argv[1]); m={'version':2,'built':datetime.date.today().isoformat(),'commit':sys.argv[2],'idf':sys.argv[3],'targets':{}}
for chip in ('esp32','esp32s3','esp32c3'):
    fw={}
    for role in ('flight','ground'):
        d=out/(chip+'-'+role); parts=json.loads((d/'parts.json').read_text()); files={}
        for _,f in parts:
            b=(d/f).read_bytes(); files[f]={'size':len(b),'sha256':hashlib.sha256(b).hexdigest()}
        fw[role]={'dir':d.name,'parts':parts,'files':files}
        (d/'parts.json').unlink()
    m['targets'][chip]={'firmware':fw}
(out/'manifest.json').write_text(json.dumps(m,indent=1)+'\n')
print('wrote firmware/manifest.json:',m['commit'],m['idf'])
PY
