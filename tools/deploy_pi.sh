#!/bin/sh
# From this computer to the drone's Raspberry Pi: the Pi's programs built here (tools/build_pi_cross.sh, zig), the
# source and LiftLab's folder there (runner/pi/suite/install.sh: ~/liftlab, the control panel, the services).
#   sh tools/deploy_pi.sh pi@192.168.2.127            build, copy, install; the running programs are restarted
#   sh tools/deploy_pi.sh pi@raspberrypi.local --no-build    (build on the Pi instead: ssh ... liftlab build)
# The design's files are LiftLab's (the Pi's Install / export…, step "Put this design on it"), not copied from here.
set -e
HOST=$1; [ -n "$HOST" ] || { echo "usage: sh tools/deploy_pi.sh USER@HOST [--no-build]"; exit 2; }
cd "$(dirname "$0")/.."
if [ "$2" != "--no-build" ]; then sh tools/build_pi_cross.sh; fi
# the source (under runner/, as it is in this folder: what git tracks and new files it doesn't ignore), then the built
# program beside it. (macOS tar: no Mac metadata, which the Pi's tar would warn about file by file.)
TARX=; if tar --version 2>/dev/null | grep -q bsdtar; then TARX="--no-mac-metadata --no-xattrs"; fi
git ls-files -z -c -o --exclude-standard runner | COPYFILE_DISABLE=1 xargs -0 tar $TARX -c -f - | ssh "$HOST" 'mkdir -p ~/liftlab/src.new && tar -x -f - -C ~/liftlab/src.new && rm -rf ~/liftlab/src.old && { [ ! -d ~/liftlab/src/runner ] || mv ~/liftlab/src ~/liftlab/src.old; } && mv ~/liftlab/src.new ~/liftlab/src && rm -rf ~/liftlab/src.old'
if [ "$2" != "--no-build" ]; then
  scp -q runner/pi/dfb_pi "$HOST":liftlab/src/runner/pi/dfb_pi
  ssh "$HOST" 'mkdir -p ~/liftlab/bin && cp ~/liftlab/src/runner/pi/dfb_pi ~/liftlab/bin/dfb_pi.new && mv ~/liftlab/bin/dfb_pi.new ~/liftlab/bin/dfb_pi'
fi
ssh "$HOST" 'sh ~/liftlab/src/runner/pi/suite/install.sh && for p in $(liftlab status --json | python3 -c "import json,sys; print(\" \".join(p[\"name\"] for p in json.load(sys.stdin)[\"programs\"] if p[\"state\"] == \"active\"))"); do liftlab restart "$p"; done; liftlab status'
