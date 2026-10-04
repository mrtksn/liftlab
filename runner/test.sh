#!/bin/sh
# Builds the runner and runs every check: the compiled formulas against the originals (JavaScript runner and C
# runner as WebAssembly), the drone's loading steps (rn_host.c), the link framing (C and the Pi's Python) and the
# flight code (fc/, flying airframes exported from the simulator), the navigation, the telemetry and radio (CRSF), and
# the Pi program end to end behind pseudo-terminals, the cargo task's latches, and the command module (runner/ground)
# alone and with the drone.
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
fc/build_wasm.sh
cc -O2 -Wall -Wextra -I. -o "$T/test_fc" fc/test_fc.c fc/fc_core.c rn_host.c rn.c rn_builtin.c -lm
"$T/test_fc" fc/testdata
cc -O2 -Wall -Wextra -I. -o "$T/test_nav" fc/test_nav.c fc/nav_core.c fc/fc_core.c rn_host.c rn.c rn_builtin.c -lm
"$T/test_nav" fc/testdata
cc -O2 -Wall -Wextra -I. -o "$T/test_cargo" fc/test_cargo.c fc/cargo_core.c fc/tlm_core.c fc/tlm_crsf.c fc/tlm_sources.c fc/crsf.c fc/rc_core.c fc/pickup_core.c -lm
"$T/test_cargo"
cc -O2 -Wall -Wextra -I. -o "$T/test_tlm" fc/test_tlm.c fc/tlm_core.c fc/tlm_crsf.c fc/tlm_sources.c fc/crsf.c fc/rc_core.c fc/pickup_core.c -lm
"$T/test_tlm"
(cd pi && sh build.sh && cc -O2 -I.. -I../fc -o "$T/test_dfb_pi" test_dfb_pi.c ../fc/nav_core.c ../fc/fc_core.c ../rn_host.c ../rn.c ../rn_link.c ../fc/tlm_core.c ../fc/rc_core.c ../fc/pickup_core.c ../rn_builtin.c -lm -lutil && "$T/test_dfb_pi")
cc -O2 -Wall -Wextra -Wno-unused-parameter -I. -Ifc -o "$T/test_ground" ground/test_ground.c ground/ground_core.c ground/rn_builtin_ground.c fc/tlm_core.c fc/tlm_crsf.c fc/crsf.c fc/rc_core.c fc/pickup_core.c rn_host.c rn.c -lm
"$T/test_ground"
(cd ground && sh build.sh && cc -O2 -I.. -I../fc -o "$T/test_ground_e2e" test_ground_e2e.c ../fc/nav_core.c ../fc/fc_core.c ../fc/tlm_core.c ../fc/tlm_sources.c ../fc/learn_core.c ../fc/super_core.c ../fc/rc_core.c ../fc/pickup_core.c ../fc/crsf.c ../rn_host.c ../rn.c ../rn_link.c ../rn_builtin.c -lm -lutil && "$T/test_ground_e2e")
