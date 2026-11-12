#!/bin/sh
# Build the Pi's programs on this computer for a Raspberry Pi (32-bit Raspberry Pi OS: armhf, glibc 2.36, Bookworm),
# with zig as the cross-compiler (brew install zig): seconds instead of minutes on a Pi Zero.
#   sh tools/build_pi_cross.sh                 dfb_pi and dfb_ground for a Pi Zero / Zero W / Pi 1 (ARMv6)
#   PI_CPU=cortex_a7 sh tools/build_pi_cross.sh    for a Pi 2, 3, 4 or Zero 2 W running the 32-bit OS
# Then copy one across, e.g.:  scp runner/pi/dfb_pi pi@raspberrypi.local:dfb/runner/pi/  (and sudo systemctl restart dfb)
set -e
cd "$(dirname "$0")/../runner"
command -v zig >/dev/null || { echo "zig not found: brew install zig (or build on the Pi: sh runner/pi/build.sh)"; exit 1; }
CC="zig cc -target arm-linux-gnueabihf.${PI_GLIBC:-2.36} -mcpu=${PI_CPU:-arm1176jzf_s}"
export CC
sh pi/build.sh
sh ground/build.sh
file pi/dfb_pi ground/dfb_ground 2>/dev/null | sed 's/, with debug_info.*//'
