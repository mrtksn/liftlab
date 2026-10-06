/* Wi-Fi up, for ESP-NOW and UDP alike (inside esp_radio). */
#ifndef WIFI_START_H
#define WIFI_START_H
#include "esp_wifi.h"
/* The network interface for mode (a station's or an access point's), the default event loop, the driver (its
 * settings kept in RAM, not flash), cfg for that interface if given, started; power saving off (it would hold packets
 * back for the beacon interval). */
esp_err_t wifi_start(wifi_mode_t mode, wifi_config_t *cfg);
#endif
