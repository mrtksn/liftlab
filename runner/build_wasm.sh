#!/bin/sh
# Builds the runner for the simulator: runner.wasm, embedded as base64 in js/rn-wasm.js (pages opened from a
# file can't fetch a .wasm next to them). Needs clang with the wasm32 target and wasm-ld.
set -e
cd "$(dirname "$0")"
node gen_ops.js > rn_ops.h
clang --target=wasm32 -O2 -ffreestanding -nostdlib -fno-builtin-memcpy -Wall -Wextra -Wno-unused-parameter \
  -Wl,--no-entry -Wl,--strip-all -Wl,--initial-memory=4194304 -o runner.wasm rn.c rn_wasm.c
node -e "const b=require('fs').readFileSync('runner.wasm');require('fs').writeFileSync('../js/rn-wasm.js','\'use strict\';\n// The C step runner (runner/rn.c) built to WebAssembly by runner/build_wasm.sh.\nconst RN_WASM_B64 = \''+b.toString('base64')+'\';\n');console.log('runner.wasm',b.length,'bytes')"
