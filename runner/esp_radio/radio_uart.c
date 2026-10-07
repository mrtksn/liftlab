/* The serial link on an ESP32 (radio=serial,BAUD[,half]): see esp_radio.h. A UART to whatever carries its bytes to
 * the other end (a laser or LED and a photodiode, fibre transceivers, an infrared pair, a radio modem in transparent
 * mode, a wire); the packets marked out in the byte stream (pframe.h), the packet layer above (radio_packet.c). */
#include "radio_packet.h"
#include "pframe.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
  int uart;
  pframe_rx F;
  uint8_t in[256]; int in_n, in_k;      /* bytes read, not yet framed */
  uint32_t busy;                        /* packets not sent: the last one still going out */
} uart_t;

/* a packet: the bytes already read first, then the UART's (waiting at most wait_ms for the first) */
static int ua_recv(pk_link *K, uint8_t *p, int cap, int *rssi, int wait_ms) {
  uart_t *U = K->t; *rssi = 0;
  for (int pass = 0; pass < 4; pass++) {
    while (U->in_k < U->in_n) { int m = pframe_feed(&U->F, U->in[U->in_k++], p, cap); if (m) return m; }
    int k = uart_read_bytes(U->uart, U->in, sizeof U->in, pass == 0 && wait_ms > 0 ? pdMS_TO_TICKS(wait_ms) : 0);
    if (k <= 0) return 0;
    U->in_n = k; U->in_k = 0;
  }
  return 0;
}
static int ua_send(pk_link *K, const uint8_t *p, int n) {
  uart_t *U = K->t;
  size_t waiting = 0; uart_get_tx_buffer_free_size(U->uart, &waiting);
  uint8_t w[PFRAME_WIRE(PLINK_MTU)]; int m = pframe_encode(p, n, w, sizeof w);
  if (!m || (int)waiting < m) { U->busy++; return -1; }   /* (the line still busy with what went before: this one is dropped, not queued late) */
  return uart_write_bytes(U->uart, w, (size_t)m) == m ? 0 : -1;
}

static pk_link *KP; static uart_t UT;
radio_io *radio_uart_start(const rlink_cfg *L, int role, const char *bind, int uart, int tx_pin, int rx_pin, esp_radio_say say) {
  if (L->kind != RLINK_SERIAL) return 0;
  if (tx_pin < 0 || rx_pin < 0 || tx_pin == rx_pin) { if (say) say("serial line: it needs two pins, one to the line's input and one from its output"); return 0; }
  if (!KP && !(KP = calloc(1, sizeof *KP))) { if (say) say("serial line: no memory"); return 0; }
  pk_init(KP, L->half ? "serial line, one way at a time" : "serial line", role, bind, say);
  plink_cfg C = KP->L.C; plink_cfg_link(&C, L); plink_init(&KP->L, &C, KP->L.session);   /* (the sizes and rates for its speed; one way or both) */
  uart_config_t uc = { .baud_rate = L->baud, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT };
  /* a TX buffer of about two of the biggest packets: a packet is either on its way or not sent (ua_send) */
  esp_err_t e = uart_driver_install(uart, 1024, 2 * PFRAME_WIRE(PLINK_MTU) + 16, 0, NULL, 0);
  if (e == ESP_OK) e = uart_param_config(uart, &uc);
  if (e == ESP_OK) e = uart_set_pin(uart, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (e != ESP_OK) { pk_say(KP, "serial line didn't start: %s", esp_err_to_name(e)); return 0; }
  memset(&UT, 0, sizeof UT); UT.uart = uart; pframe_rx_init(&UT.F);
  KP->t = &UT; KP->recv = ua_recv; KP->send = ua_send;
  pk_say(KP, "radio: a serial line at %d baud%s, GPIO %d out and %d in; %.0f packets a second up", L->baud, L->half ? ", one way at a time (the drone answers each packet)" : "", tx_pin, rx_pin, (double)C.up_hz);
  return &KP->io;
}
