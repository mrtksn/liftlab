/* The drone's end of an ExpressLRS link: a receiver (or a Crossfire one) on a UART, CRSF at 420000 baud both ways
 * (setting crsf=rx,tx). The receiver does the radio's part itself: it hands over the channels, the ground station's
 * commands and its link statistics as CRSF frames, and sends down what it's given. See radio_io.h. */
#include "radio_elrs.h"
#include "esp_board.h"
#include "crsf.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include <stdio.h>

#define RADIO LB_RADIO_UART
static int elrs_read(radio_io *R, uint8_t *b, int n, int wait_ms) { (void)R; int k = uart_read_bytes(RADIO, b, (uint32_t)n, pdMS_TO_TICKS(wait_ms)); return k < 0 ? -1 : k; }
static int elrs_write(radio_io *R, const uint8_t *b, int n) { (void)R; int k = uart_write_bytes(RADIO, b, (size_t)n); return k < 0 ? -1 : k; }
static radio_io io = { .name = "ExpressLRS receiver (UART)", .read = elrs_read, .write = elrs_write, .fd = -1 };

radio_io *radio_elrs_start(const hw_config *c) {
  if (c->crsf_rx < 0) return 0;
  uart_config_t uc = { .baud_rate = CRSF_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT };
  uart_driver_install(RADIO, 1024, 1024, 0, NULL, 0); uart_param_config(RADIO, &uc); uart_set_pin(RADIO, c->crsf_tx, c->crsf_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  printf("radio receiver: CRSF on GPIO %d (from its TX) and %d (to its RX); ExpressLRS %d Hz, telemetry 1:%d\n", c->crsf_rx, c->crsf_tx, c->elrs_rate, c->elrs_ratio);
  return &io;
}
