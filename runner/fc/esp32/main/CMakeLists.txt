idf_component_register(SRCS "flight.c" "hw.c" "config.c" "outputs.c" INCLUDE_DIRS "."
                       REQUIRES runner fc nvs_flash esp_driver_uart esp_driver_i2c esp_driver_ledc esp_driver_gpio esp_adc esp_timer esp_system heap vfs)
