#!/bin/sh
# Builds the flight controller's portable part for the simulator's firmware-in-the-loop mode: fc.wasm, embedded as
# base64 in js/fc-wasm.js. Needs clang with the wasm32 target and wasm-ld.
set -e
cd "$(dirname "$0")"
clang --target=wasm32 -O2 -ffreestanding -nostdlib -fno-builtin-memcpy -Wall -Wextra -Wno-unused-parameter -I.. \
  -Wl,--no-entry -Wl,--strip-all -Wl,--initial-memory=4194304 -Wl,-z,stack-size=262144 \
  -o fc.wasm fc_core.c fc_wasm.c ../rn_host.c ../rn.c
node -e "const b=require('fs').readFileSync('fc.wasm');require('fs').writeFileSync('../../js/fc-wasm.js','\'use strict\';\n// The flight controller firmware (runner/fc/fc_core.c with the step runner) built to WebAssembly by runner/fc/build_wasm.sh.\nconst FC_WASM_B64 = \''+b.toString('base64')+'\';\n');console.log('fc.wasm',b.length,'bytes')"
