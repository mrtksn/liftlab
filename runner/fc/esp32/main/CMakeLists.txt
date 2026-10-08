idf_component_register(SRCS "flight.c" "hw.c" "config.c" "outputs.c" "radio_elrs.c" INCLUDE_DIRS "."
                       REQUIRES runner fc esp_radio nvs_flash esp_driver_uart esp_driver_i2c esp_driver_ledc esp_driver_gpio esp_adc esp_timer esp_system esp_app_format heap vfs)
