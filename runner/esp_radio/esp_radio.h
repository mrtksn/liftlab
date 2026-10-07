/* The packet links on an ESP32 (radio_io.h): ESP-NOW and Wi-Fi (UDP), for the drone (the flight firmware) and the
 * command module (the ground firmware) alike. Each does in our own code what an ExpressLRS module does in its own
 * (plink.h): write() takes the CRSF frames the program writes, read() hands back the frames that came and the link
 * statistics, as a receiver (the drone) or a transmitter module (the command module) would. The program calls read()
 * every 2–4 ms: the packets go and come from there (and write() sends one at once if it's due). Both calls from one
 * task (plink isn't thread-safe): the Wi-Fi task only puts what comes into a queue.
 *
 * ESP-NOW (radio=espnow,CHANNEL[,lr]): Wi-Fi as a station that joins nothing, on the channel set (both ends the
 * same); packets to the broadcast address, so no pairing: they're signed with the binding phrase's key, and the
 * other end drops what isn't. lr: Espressif's long-range mode (WIFI_PROTOCOL_LR: 0.25–0.5 Mbit/s, further), only
 * understood by an ESP32 also in it: both ends lr, or neither.
 * Wi-Fi (radio=wifi,ap,CHANNEL or wifi,sta): UDP, port RLINK_UDP_PORT. The drone makes the network (ap: name and
 * password from wifi=, else LiftLab-XXXX and the default password, radio_cfg.h) or joins one (sta: wifi= needed),
 * and answers whoever sent the last packet it took (a command module, or a laptop's dfb_ground). The command module
 * is always a station: it joins wifi= and sends to the drone's address (drone=).
 *
 * A serial line (radio=serial,BAUD[,half]): a UART to whatever carries its bytes to the other end (a laser or LED
 * and a photodiode, fibre transceivers, an infrared pair, a radio modem in transparent mode, a wire), at that speed,
 * the same at both ends; the packets framed in the byte stream (pframe.h), signed with the binding phrase. The drone's
 * on the receiver's pins (crsf=), the command module's on the module's (tx=); half: the drone answers each packet.
 *
 * say: where the link's news goes (a line of text: joined the network, its address…), from any task. */
#ifndef ESP_RADIO_H
#define ESP_RADIO_H
#include "radio_io.h"
#include "radio_link.h"
#include "plink.h"
#include "radio_mux.h"

/* This start's session number, one for all its links (radio_packet.c). */
uint32_t esp_radio_session(void);

typedef void (*esp_radio_say)(const char *text);
/* Started, or 0 (said why). role: PLINK_GROUND or PLINK_DRONE; bind: the binding phrase. */
radio_io *radio_espnow_start(const rlink_cfg *L, int role, const char *bind, esp_radio_say say);
/* ssid, pass: wifi= (empty: the defaults); drone_ip: the command module's drone= (the drone: ignored). */
radio_io *radio_wifi_start(const rlink_cfg *L, int role, const char *bind, const char *ssid, const char *pass, const char *drone_ip, esp_radio_say say);
/* nRF24L01 (radio=nrf24,RATE): the module on SPI, pins SCK, MOSI, MISO, CSN, CE (nrf24= on either board; the drone's
 * end answers in the acknowledgements: nrf24.h). */
radio_io *radio_nrf24_start(const rlink_cfg *L, int role, const char *bind, const int8_t pins[5], esp_radio_say say);
int radio_nrf24_status(radio_io *R, char *out, int n);         /* 1: R is it (its line in out) */
/* Bluetooth LE (radio=ble): the drone a peripheral, the command module the central (radio_ble.c). And, when the link
 * isn't Bluetooth, its memory given back at start (call it before any other radio starts). */
radio_io *radio_ble_start(const rlink_cfg *L, int role, const char *bind, esp_radio_say say);
void radio_ble_release(void);
/* uart: the UART to use; tx_pin to the line's input, rx_pin from its output. */
radio_io *radio_uart_start(const rlink_cfg *L, int role, const char *bind, int uart, int tx_pin, int rx_pin, esp_radio_say say);
/* The link's counts (plink_counts) and what it hears, for status: a line into out. */
void esp_radio_status(radio_io *R, char *out, int n);
#endif
