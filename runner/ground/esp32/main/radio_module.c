/* The command module's end of an ExpressLRS link: the transmitter module on a UART (CRSF, 400000 baud by default),
 * as two wires (its input and output) or the one wire of a module bay (tx = rx). The module does the radio's part
 * itself: it takes the channel and command frames, and hands back the drone's telemetry and its own link statistics
 * as CRSF frames. See radio_io.h. */
#include "radio_module.h"
#include "esp_board.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
#include "freertos/FreeRTOS.h"

#define TXU LB_RADIO_UART
static int wire_pin = -1;          /* one wire both ways: its pin (−1: two wires) */
/* One wire both ways (tx = rx, a module bay's CRSF pin), as ExpressLRS's CRSFHandset runs it from the module's side:
 * inverted serial (the UART inverts both ways), so the line idles low; listening, the pin is an input pulled down
 * (as the module's is); talking, it drives the line, with our receiver held at idle so we don't hear ourselves. */
static void wire_listen(void) {
  gpio_set_direction(wire_pin, GPIO_MODE_INPUT); gpio_set_pull_mode(wire_pin, GPIO_PULLDOWN_ONLY);
  esp_rom_gpio_connect_in_signal(wire_pin, LB_RX_SIGNAL, false);
}
static void wire_talk(void) {
  gpio_set_pull_mode(wire_pin, GPIO_FLOATING);
  esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, LB_RX_SIGNAL, false);   /* (0, inverted: idle) */
  gpio_set_level(wire_pin, 0); gpio_set_direction(wire_pin, GPIO_MODE_OUTPUT);           /* (idle low, then the UART's) */
  esp_rom_gpio_connect_out_signal(wire_pin, LB_TX_SIGNAL, false, false);
}
static int module_read(radio_io *R, uint8_t *b, int n, int wait_ms) { (void)R; int k = uart_read_bytes(TXU, b, (uint32_t)n, pdMS_TO_TICKS(wait_ms)); return k < 0 ? -1 : k; }
static int module_write(radio_io *R, const uint8_t *b, int n) {
  (void)R;
  if (wire_pin < 0) { int k = uart_write_bytes(TXU, b, (size_t)n); return k < 0 ? -1 : k; }
  wire_talk(); int k = uart_write_bytes(TXU, b, (size_t)n);
  uart_wait_tx_done(TXU, pdMS_TO_TICKS(10)); wire_listen();     /* the last bit out: let go at once, the module answers now */
  return k < 0 ? -1 : k;
}
static radio_io io = { "ExpressLRS transmitter module (UART)", module_read, module_write, -1, 0 };

radio_io *radio_module_start(int tx, int rx, int baud) {
  uart_config_t uc = { .baud_rate = baud, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT };
  uart_driver_install(TXU, 1024, 1024, 0, NULL, 0); uart_param_config(TXU, &uc);
  uart_set_pin(TXU, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  wire_pin = tx == rx ? tx : -1;
  if (wire_pin >= 0) { uart_set_line_inverse(TXU, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV); wire_listen(); }
  return &io;
}
