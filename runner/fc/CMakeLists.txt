# The flight code as an ESP-IDF component (fc/esp32 uses it): the flight core, the telemetry task and the pilot's radio.
# board_wasm.c, nav_core.c, learn_core.c, super_core.c and the tests are for the simulator, the Pi and a PC.
idf_component_register(SRCS "fc_core.c" "tlm_core.c" "tlm_crsf.c" "radio_link.c" "plink.c" "pframe.c" "tlm_sources.c" "crsf.c" "rc_core.c" "pickup_core.c" INCLUDE_DIRS "." REQUIRES runner)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wno-unused-parameter)
