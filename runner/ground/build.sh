#!/bin/sh
# Builds the command module for a Mac, a Raspberry Pi or any Linux computer: dfb_ground.c with the ground core
# (ground_core.c), CRSF (../fc/crsf.c, tlm_crsf.c), the step runner and the built-in ground program.
#   sh runner/ground/build.sh && ./runner/ground/dfb_ground --tx /dev/tty.usbserial-XXXX --keys
set -e
cd "$(dirname "$0")"
${CC:-cc} -O2 -Wall -Wextra -Wno-unused-parameter -I.. -I../fc -o dfb_ground \
  dfb_ground.c ground_core.c ground_text.c rn_builtin_ground.c ../pi/serial_baud.c ../fc/tlm_core.c ../fc/tlm_crsf.c ../fc/crsf.c ../fc/rc_core.c ../fc/pickup_core.c ../rn_host.c ../rn.c -lm
echo "built dfb_ground"
