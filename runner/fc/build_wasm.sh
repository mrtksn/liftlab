#!/bin/sh
# Builds a flight computer for the simulator: board.wasm (the flight core and the navigation, fc_core.c and
# nav_core.c, with the step runner), embedded as base64 in js/board-wasm.js. The page runs one instance per board.
# Needs clang with the wasm32 target and wasm-ld.
set -e
cd "$(dirname "$0")"
clang --target=wasm32 -O2 -ffreestanding -nostdlib -fno-builtin-memcpy -Wall -Wextra -Wno-unused-parameter -I.. \
  -Wl,--no-entry -Wl,--strip-all -Wl,--initial-memory=4194304 -Wl,-z,stack-size=262144 \
  -o board.wasm fc_core.c nav_core.c board_wasm.c ../rn_host.c ../rn.c
node -e "const b=require('fs').readFileSync('board.wasm');require('fs').writeFileSync('../../js/board-wasm.js','\'use strict\';\n// A flight computer (runner/fc: fc_core.c, nav_core.c, the step runner) built to WebAssembly by runner/fc/build_wasm.sh.\nconst BOARD_WASM_B64 = \''+b.toString('base64')+'\';\n');console.log('board.wasm',b.length,'bytes')"
