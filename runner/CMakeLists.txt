# The step runner as an ESP-IDF component: add this folder to EXTRA_COMPONENT_DIRS (see esp32/CMakeLists.txt).
# The rest of the folder builds the same C for the simulator (build_wasm.sh) and for a PC (the tests).
idf_component_register(SRCS "rn.c" "rn_host.c" "rn_link.c" "rn_builtin.c"
                       INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wno-unused-parameter)
