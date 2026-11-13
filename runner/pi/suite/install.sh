#!/bin/sh
# Sets up LiftLab's folder on the drone's computer (a Raspberry Pi), or brings it up to date: the control panel
# (liftlab, also /usr/local/bin/liftlab), liftlab.conf (from the example the first time, or from the earlier layout:
# ~/dfb and dfb.service, ~/liftlab-bench), its programs as systemd services, and the serial port kept from
# ModemManager. Run on the Pi, from a copy of the repository (tools/deploy_pi.sh copies one to ~/liftlab/src):
#   sh ~/liftlab/src/runner/pi/suite/install.sh [FOLDER]        (FOLDER: ~/liftlab)
# Needs sudo (for systemd and /usr/local/bin). Nothing of the earlier layout is deleted.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
RUNNER=$(cd "$HERE/../.." && pwd)                     # (the runner/ this came with)
ROOT=${1:-$HOME/liftlab}
mkdir -p "$ROOT/bin" "$ROOT/design" "$ROOT/apps" "$ROOT/src"
ROOT=$(cd "$ROOT" && pwd)
cp "$HERE/liftlab" "$ROOT/liftlab.new" && chmod +x "$ROOT/liftlab.new" && mv "$ROOT/liftlab.new" "$ROOT/liftlab"
cp "$HERE/liftlab.conf.example" "$ROOT/liftlab.conf.example"
if [ "$RUNNER" != "$ROOT/src/runner" ]; then                       # (the source beside it, for liftlab build)
  rm -rf "$ROOT/src/runner.new" && cp -R "$RUNNER" "$ROOT/src/runner.new" && rm -rf "$ROOT/src/runner" && mv "$ROOT/src/runner.new" "$ROOT/src/runner"
fi
sudo ln -sf "$ROOT/liftlab" /usr/local/bin/liftlab
# The flight controller's serial port isn't a modem: ModemManager leaves it alone (its probes write AT commands).
printf '%s\n' '# LiftLab: the flight controller link, not a modem' 'KERNEL=="ttyAMA0", ENV{ID_MM_DEVICE_IGNORE}="1"' 'KERNEL=="ttyS0", ENV{ID_MM_DEVICE_IGNORE}="1"' |
  sudo tee /etc/udev/rules.d/99-liftlab-serial.rules > /dev/null
sudo udevadm control --reload 2>/dev/null || true
if [ ! -f "$ROOT/liftlab.conf" ] && [ ! -f /etc/systemd/system/dfb.service ] && [ ! -d "$HOME/dfb" ]; then
  cp "$HERE/liftlab.conf.example" "$ROOT/liftlab.conf"
  echo "liftlab.conf: from the example (set [flight]'s command from LiftLab's install guide, then liftlab apply)"
fi
LIFTLAB_HOME="$ROOT" "$ROOT/liftlab" migrate                     # (the earlier layout, if there is one; then apply)
echo "LiftLab is in $ROOT. The panel: liftlab   (over SSH: ssh -t $(id -un)@$(hostname).local liftlab)"
