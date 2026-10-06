#!/bin/sh
# Linux integration checks: production ground/Pi programs, fake flight/GPS and radio loss.
set -e
cd "$(dirname "$0")/../runner"
sh ground/build.sh
sh pi/build.sh
TEST_DIR=$(mktemp -d "${TMPDIR:-/tmp}/liftlab-radio-e2e.XXXXXX")
trap 'rm -rf "$TEST_DIR"' EXIT HUP INT TERM
cd ground
for test in test_ground_e2e test_ground_wifi_e2e test_ground_serial_e2e; do
  cc -O2 -I.. -I../fc -o "$TEST_DIR/$test" "$test.c" \
    ../fc/nav_core.c ../fc/fc_core.c ../fc/tlm_core.c ../fc/tlm_sources.c \
    ../fc/learn_core.c ../fc/super_core.c ../fc/rc_core.c ../fc/pickup_core.c \
    ../fc/crsf.c ../rn_host.c ../rn.c ../rn_link.c ../rn_builtin.c -lm -lutil
  "$TEST_DIR/$test"
done
