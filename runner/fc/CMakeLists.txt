# The flight code as an ESP-IDF component (fc/esp32 uses it). board_wasm.c, nav_core.c and the tests are for the simulator, the Pi and a PC.
idf_component_register(SRCS "fc_core.c" INCLUDE_DIRS "." REQUIRES runner)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wno-unused-parameter)
