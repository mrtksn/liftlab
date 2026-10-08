#!/bin/sh
# Host validation of the production hardware coordinator and Pi storage/transport.
set -eu
cd "$(dirname "$0")/.."
TEST_DIR=$(mktemp -d "${TMPDIR:-/tmp}/liftlab-hardware-tuning.XXXXXX")
trap 'rm -rf "$TEST_DIR"' EXIT HUP INT TERM
COMMON="runner/fc/autotune_core.c runner/fc/learn_core.c runner/fc/fc_core.c runner/fc/nav_core.c runner/rn_host.c runner/rn.c"
FLAGS="-pthread -O1 -Wall -Wextra -Werror -Wno-unused-parameter -fsanitize=undefined -Irunner -Irunner/fc"
${CC:-cc} $FLAGS tools/autotune_math_probe.c $COMMON -lm -o "$TEST_DIR/math"
node tools/test_hardware_tuning.js "$TEST_DIR/math"
${CC:-cc} $FLAGS tools/test_hardware_autotune.c $COMMON runner/fc/super_core.c runner/fc/radio_link.c runner/rn_link.c runner/rn_builtin.c -lm -o "$TEST_DIR/flight"
"$TEST_DIR/flight"
${CC:-cc} $FLAGS tools/test_pi_tuning.c runner/rn.c runner/rn_link.c -lm -o "$TEST_DIR/pi"
"$TEST_DIR/pi"
# Compile the actual Pi coordinator on both host OSes. Linux CI also links it with real GPIO drivers.
${CC:-cc} $FLAGS -c runner/pi/dfb_pi.c -o "$TEST_DIR/dfb_pi.o"
PYTHONDONTWRITEBYTECODE=1 python3 tools/test_pi_control.py
