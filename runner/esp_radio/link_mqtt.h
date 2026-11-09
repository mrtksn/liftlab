/* The link to the companion computer over MQTT, beside the UART (which stays: it is also the USB console the installer
 * and fly.py use). The flight controller joins a Wi-Fi network as a station, connects to the broker (fc/mqtt_min.h),
 * takes frames from PREFIX/link/to-fc and publishes its own on PREFIX/link/from-fc, one frame a message. The client runs
 * in its own task; link_task reads what came with link_mqtt_read, as it reads the UART. */
#ifndef LINK_MQTT_H
#define LINK_MQTT_H
#include <stdint.h>
#include "esp_radio.h"
/* ssid/pass: the network (an empty password: an open network); host, port: the broker; prefix: the topics'. 0 ok. */
int link_mqtt_start(const char *ssid, const char *pass, const char *host, int port, const char *prefix, esp_radio_say say);
/* Bytes that came since the last call (up to max). */
int link_mqtt_read(uint8_t *b, int max);
/* 1 once after each (re)connection: a frame cut short by the break isn't to be joined to the next. */
int link_mqtt_resync(void);
/* One whole frame out (dropped while not connected, or when the queue is full). */
void link_mqtt_send(const uint8_t *frame, uint32_t n);
#endif
