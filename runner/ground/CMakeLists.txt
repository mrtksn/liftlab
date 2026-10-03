# The command module's core as an ESP-IDF component (ground/esp32 uses it). dfb_ground.c, the tests and build.sh are
# for a Mac, a Pi or a PC; the simulator builds ground_core.c into its WebAssembly (fc/build_wasm.sh).
idf_component_register(SRCS "ground_core.c" "ground_text.c" "rn_builtin_ground.c" INCLUDE_DIRS "." REQUIRES runner fc)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wno-unused-parameter)
