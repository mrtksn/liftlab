idf_component_register(SRCS "ground.c" "radio_module.c" INCLUDE_DIRS "."
                       REQUIRES ground runner fc esp_radio log nvs_flash esp_driver_uart esp_driver_gpio esp_adc esp_timer esp_system esp_app_format vfs)
