#!/bin/sh
# Builds the runner and runs every check: the compiled formulas against the originals (JavaScript runner and C
# runner as WebAssembly), the drone's loading steps (rn_host.c) and the link framing (C and the Pi's Python).
# Needs node, a C compiler, and clang with the wasm32 target for the WebAssembly build.
set -e
cd "$(dirname "$0")"
T=${TMPDIR:-/tmp}/dfb-runner-test; mkdir -p "$T"
./build_wasm.sh
node ../tools/check_formulas.js
node ../tools/host_test_data.js "$T"
cc -O2 -Wall -Wextra -o "$T/test_rnhost" rn.c rn_host.c test_rnhost.c -lm
"$T/test_rnhost" "$T/builtin.rnp" "$T/edit.rnp" "$T/nan.rnp" "$T/trap.rnp" "$T/sig.rnp" "$T/calls.bin"
python3 -c "import sys; sys.path.insert(0, 'pi'); from send_program import frame; open('$T/pyframe.bin', 'wb').write(frame(1, bytes((i * 7 + 3) & 255 for i in range(1000))))"
cc -O2 -Wall -Wextra -o "$T/test_link" rn.c rn_link.c test_link.c -lm
"$T/test_link" "$T/pyframe.bin"
