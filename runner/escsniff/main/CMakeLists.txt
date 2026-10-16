idf_component_register(SRCS "escsniff.c" "sniff_core.c" INCLUDE_DIRS "."
                       REQUIRES esp_driver_ledc esp_driver_gpio esp_driver_uart esp_driver_usb_serial_jtag esp_hw_support esp_system)
