#!/bin/sh
# Build all supported chips/roles outside the checkout; derive offsets from ESP-IDF.
# . $IDF_PATH/export.sh && sh tools/build_firmware.sh
# ONLY=esp32s3-flight builds that one bundle and writes no manifest (CI builds each in a job of its own);
# MANIFEST_ONLY=1 builds nothing: it writes firmware/manifest.json from the six bundles already in firmware/.
# Each build is stamped with DFB_FW_COMMIT (default: this checkout's commit, +changes when runner/ differs from it).
# The boards report it ("version") and the manifest records it, so the installer can tell a board that runs another
# build, and tools/check_firmware_fresh.sh can tell bundles older than the source.
set -e
cd "$(dirname "$0")/.."
ROOT=$(pwd); OUT=$ROOT/firmware; B=${BUILD_DIR:-/tmp/liftlab-firmware-build}
COMMIT=${DFB_FW_COMMIT:-$(git rev-parse --short HEAD)$(git diff --quiet HEAD -- runner || echo '+changes')}
mkdir -p "$OUT"
if [ -z "$MANIFEST_ONLY" ]; then
  command -v idf.py >/dev/null || { echo "ESP-IDF isn't set up: . \$IDF_PATH/export.sh first"; exit 1; }
  IDF_VER=$(idf.py --version)
  for chip in esp32 esp32s3 esp32c3; do
    for role in flight ground; do
      [ -n "$ONLY" ] && [ "$ONLY" != "$chip-$role" ] && continue
      if [ "$role" = flight ]; then src=runner/fc/esp32; else src=runner/ground/esp32; fi
      bd=$B/$chip-$role
      mkdir -p "$bd"
      # (a cache entry, not only the environment: a changed commit makes idf.py configure again, so the stamp follows it)
      (cd "$src" && idf.py -B "$bd" -D SDKCONFIG="$bd/sdkconfig" -D IDF_TARGET="$chip" -D DFB_FW_COMMIT="$COMMIT" build > "$bd/build.log" 2>&1) || { tail -60 "$bd/build.log"; exit 1; }
      python3 - "$bd" "$OUT/$chip-$role" "$IDF_VER" <<'PY'
import json,pathlib,shutil,sys
bd,out=map(pathlib.Path,sys.argv[1:3]); out.mkdir(parents=True,exist_ok=True)
m=json.loads((bd/'flasher_args.json').read_text())
lines=[]
settings=m['flash_settings']
lines.append('--flash_mode '+settings['flash_mode']+' --flash_freq '+settings['flash_freq']+' --flash_size '+settings['flash_size'])
for addr,name in m['flash_files'].items():
    target=pathlib.Path(name).name
    shutil.copyfile(bd/name,out/target); lines.append(addr+' '+target)
(out/'flash_args').write_text('\n'.join(lines)+'\n')
(out/'parts.json').write_text(json.dumps([[int(a,0),pathlib.Path(n).name] for a,n in m['flash_files'].items()]))
(out/'idf.txt').write_text(sys.argv[3])
PY
      echo "built $chip $role ($COMMIT)"
    done
  done
  if [ -n "$ONLY" ]; then echo "no manifest for one bundle: MANIFEST_ONLY=1 writes it once all six are in firmware/"; exit 0; fi
fi
python3 - "$OUT" "$COMMIT" <<'PY'
import hashlib,json,pathlib,sys,datetime
out=pathlib.Path(sys.argv[1]); m={'version':2,'built':datetime.date.today().isoformat(),'commit':sys.argv[2],'idf':'','targets':{}}
for chip in ('esp32','esp32s3','esp32c3'):
    fw={}
    for role in ('flight','ground'):
        d=out/(chip+'-'+role)
        if not (d/'parts.json').exists(): sys.exit(f'{d.name}: no fresh build in firmware/ (parts.json missing): build all six first')
        parts=json.loads((d/'parts.json').read_text()); files={}
        idf=(d/'idf.txt').read_text().strip() if (d/'idf.txt').exists() else ''
        if m['idf'] and idf and idf!=m['idf']: sys.exit(f'{d.name} was built with {idf}, the others with {m["idf"]}')
        m['idf']=m['idf'] or idf
        for _,f in parts:
            b=(d/f).read_bytes(); files[f]={'size':len(b),'sha256':hashlib.sha256(b).hexdigest()}
        fw[role]={'dir':d.name,'parts':parts,'files':files}
    m['targets'][chip]={'firmware':fw}
for chip in ('esp32','esp32s3','esp32c3'):
    for role in ('flight','ground'):
        for f in ('parts.json','idf.txt'): (out/(chip+'-'+role)/f).unlink(missing_ok=True)
(out/'manifest.json').write_text(json.dumps(m,indent=1)+'\n')
print('wrote firmware/manifest.json:',m['commit'],m['idf'])
PY
