#!/bin/sh
# Builds the Pi's program: dfb_pi.c with the navigation, the learning and the health supervisor (fc/nav_core.c,
# fc/learn_core.c, fc/super_core.c), the cargo (fc/cargo_core.c, latch_hw.c), the step runner and the Pi's built-in program (rn_builtin_pi.c). On the Pi:
#   sh runner/pi/build.sh && ./runner/pi/dfb_pi --nav drone.dnc --airframe drone.dfa --pi drone.dlc --gps /dev/ttyUSB0
set -e
cd "$(dirname "$0")"
${CC:-cc} -O2 -Wall -Wextra -Wno-unused-parameter -I.. -I../fc -o dfb_pi \
  dfb_pi.c serial_baud.c latch_hw.c ../fc/cargo_core.c ../fc/nav_core.c ../fc/learn_core.c ../fc/super_core.c ../fc/fc_core.c ../fc/tlm_core.c ../fc/tlm_crsf.c ../fc/tlm_sources.c ../fc/crsf.c ../fc/rc_core.c ../fc/pickup_core.c ../rn_host.c ../rn.c ../rn_link.c rn_builtin_pi.c -lm
echo "built dfb_pi"
