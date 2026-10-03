#!/bin/sh
# Builds a flight computer for the simulator: board.wasm (the flight core, the navigation, the learning and the
# health supervisor: fc_core.c, nav_core.c, learn_core.c, super_core.c, the telemetry and the radio, with the step
# runner; and the command module on the ground: ../ground/ground_core.c), embedded as base64 in js/board-wasm.js. The page runs one instance per board.
# Needs clang with the wasm32 target and wasm-ld.
set -e
cd "$(dirname "$0")"
clang --target=wasm32 -O2 -ffreestanding -nostdlib -fno-builtin-memcpy -Wall -Wextra -Wno-unused-parameter -I.. -I. \
  -Wl,--no-entry -Wl,--strip-all -Wl,--initial-memory=8388608 -Wl,-z,stack-size=262144 \
  -o board.wasm fc_core.c nav_core.c learn_core.c super_core.c tlm_core.c tlm_crsf.c tlm_sources.c crsf.c rc_core.c ../ground/ground_core.c board_wasm.c ../rn_host.c ../rn.c
node -e "const b=require('fs').readFileSync('board.wasm');require('fs').writeFileSync('../../js/board-wasm.js','\'use strict\';\n// A flight computer (runner/fc: fc_core.c, nav_core.c, learn_core.c, super_core.c, the step runner) built to WebAssembly by runner/fc/build_wasm.sh.\nconst BOARD_WASM_B64 = \''+b.toString('base64')+'\';\n');console.log('board.wasm',b.length,'bytes')"
