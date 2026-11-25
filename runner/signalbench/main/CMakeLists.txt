idf_component_register(SRCS "signalbench.c" "../../escsniff/main/sniff_core.c"
  INCLUDE_DIRS "." "../../escsniff/main"
  REQUIRES esp_adc driver esp_timer esp_hw_support esp_system json)
