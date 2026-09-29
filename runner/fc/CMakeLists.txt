# The flight code as an ESP-IDF component (fc/esp32 uses it). fc_wasm.c and test_fc.c are for the simulator and a PC.
idf_component_register(SRCS "fc_core.c" INCLUDE_DIRS "." REQUIRES runner)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wno-unused-parameter)
