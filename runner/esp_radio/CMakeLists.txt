# The packet links on an ESP32 (ESP-NOW, Wi-Fi UDP, a serial line) as an ESP-IDF component, for the flight firmware (fc/esp32) and
# the command module's (ground/esp32): see esp_radio.h. plink.c and pframe.c (the packet layer and the framing, portable) are built by the fc
# component. radio_cfg.h and usb_split.c are portable too (test_esp_radio.c tests them on a PC).
idf_component_register(SRCS "radio_packet.c" "radio_espnow.c" "radio_wifi.c" "radio_uart.c" "radio_nrf24.c" "radio_ble.c" "radio_peer.c" "wifi_start.c" "usb_split.c"
                       INCLUDE_DIRS "."
                       REQUIRES runner fc bt esp_driver_uart esp_driver_spi esp_driver_gpio esp_wifi esp_netif esp_event lwip esp_timer esp_hw_support log freertos)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2)
