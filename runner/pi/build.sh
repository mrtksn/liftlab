#!/bin/sh
# Builds the Pi's navigation program (pi_nav.c with the navigation and the step runner). On the Pi:
#   sh runner/pi/build.sh && ./runner/pi/pi_nav --config drone.dnc --gps /dev/ttyUSB0
set -e
cd "$(dirname "$0")"
${CC:-cc} -O2 -Wall -Wextra -Wno-unused-parameter -I.. -I../fc -o pi_nav \
  pi_nav.c ../fc/nav_core.c ../rn_host.c ../rn.c ../rn_link.c ../rn_builtin.c -lm
echo "built pi_nav"
