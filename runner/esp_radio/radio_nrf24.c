/* The nRF24L01 link on an ESP32 (radio=nrf24,RATE; the module's pins nrf24=SCK,MOSI,MISO,CSN,CE): see esp_radio.h.
 * The module on SPI (8 MHz, mode 0, the second SPI controller), its CE on a GPIO; the radio's part in nrf24.c, the
 * packet layer in clink.c, both the same as on a Pi and in the simulator. */
#include "esp_radio.h"
#include "radio_cfg.h"
#include "nrf24.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  radio_io io;                   /* (first: a radio_io * is one of these) */
  nrf24_link N;
  spi_device_handle_t dev;
  int ce_pin;
  esp_radio_say say;
  char name[48];
} nrf_t;
static nrf_t *NP;

static int hal_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, int n) {
  nrf_t *K = ctx; spi_transaction_t t; memset(&t, 0, sizeof t);
  t.length = (size_t)n * 8; t.tx_buffer = tx; t.rx_buffer = rx;
  return spi_device_polling_transmit(K->dev, &t) == ESP_OK ? 0 : -1;
}
static void hal_ce(void *ctx, int v) { nrf_t *K = ctx; gpio_set_level(K->ce_pin, v); }
static void hal_delay(void *ctx, int us) { (void)ctx; if (us >= 2000) vTaskDelay(pdMS_TO_TICKS((us + 999) / 1000)); else esp_rom_delay_us((uint32_t)us); }
static double now(void) { return esp_timer_get_time() * 1e-6; }

static int nrf_read(radio_io *R, uint8_t *b, int n, int wait_ms) {
  nrf_t *K = (nrf_t *)R; double t = now();
  nrf24_poll(&K->N, t);
  int k = clink_to_stack(&K->N.L, t, b, n);
  if (!k && wait_ms > 0) vTaskDelay(pdMS_TO_TICKS(wait_ms));    /* (the program's pace: it calls again) */
  return k;
}
static int nrf_write(radio_io *R, const uint8_t *b, int n) {
  nrf_t *K = (nrf_t *)R; double t = now();
  clink_from_stack(&K->N.L, b, n, t);
  nrf24_poll(&K->N, t);                                         /* (the channels up at once, if the beat is due) */
  return n;
}

radio_io *radio_nrf24_start(const rlink_cfg *L, int role, const char *bind, const int8_t pins[5], esp_radio_say say) {
  char s[160];
  if (L->kind != RLINK_NRF24) return 0;
  for (int i = 0; i < 5; i++) if (pins[i] < 0) { if (say) say("nRF24L01: set its pins first: nrf24=SCK,MOSI,MISO,CSN,CE"); return 0; }
  if (!NP && !(NP = calloc(1, sizeof *NP))) { if (say) say("nRF24L01: no memory"); return 0; }
  nrf_t *K = NP; K->say = say; K->ce_pin = pins[4];
  spi_bus_config_t bc = { .sclk_io_num = pins[0], .mosi_io_num = pins[1], .miso_io_num = pins[2], .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 64 };
  spi_device_interface_config_t dc = { .clock_speed_hz = 8 * 1000 * 1000, .mode = 0, .spics_io_num = pins[3], .queue_size = 1 };
  esp_err_t e = spi_bus_initialize(SPI2_HOST, &bc, SPI_DMA_DISABLED);
  if (e == ESP_OK) e = spi_bus_add_device(SPI2_HOST, &dc, &K->dev);
  if (e != ESP_OK) { snprintf(s, sizeof s, "nRF24L01: SPI didn't start: %s", esp_err_to_name(e)); if (say) say(s); return 0; }
  gpio_reset_pin(K->ce_pin); gpio_set_direction(K->ce_pin, GPIO_MODE_OUTPUT); gpio_set_level(K->ce_pin, 0);
  nrf24_hal H = { hal_xfer, hal_ce, hal_delay, K };
  clink_cfg C; clink_cfg_default(&C, role); clink_cfg_link(&C, L); plink_key(rcfg_bind(bind), &C.k0, &C.k1);
  uint32_t ses = 0; while (!ses) ses = esp_random();
  char err[120];
  if (nrf24_start(&K->N, &H, &C, ses, role, L->kbps, err, sizeof err)) { snprintf(s, sizeof s, "nRF24L01: %s", err); if (say) say(s); return 0; }
  snprintf(K->name, sizeof K->name, "nRF24L01 at %d kbit/s", L->kbps);
  K->io.name = K->name; K->io.read = nrf_read; K->io.write = nrf_write; K->io.fd = -1; K->io.ctx = K;
  snprintf(s, sizeof s, "radio: nRF24L01 at %d kbit/s, %.0f packets a second, hopping over channels %d %d %d %d %d %d %d %d (from the binding phrase)",
           L->kbps, (double)C.up_hz, K->N.L.hop[0], K->N.L.hop[1], K->N.L.hop[2], K->N.L.hop[3], K->N.L.hop[4], K->N.L.hop[5], K->N.L.hop[6], K->N.L.hop[7]);
  if (say) say(s);
  return &K->io;
}
int radio_nrf24_status(radio_io *R, char *out, int n) {
  if (!R || !NP || R != &NP->io) return 0;
  const nrf24_link *N = &NP->N; const clink_counts *c = &N->L.N; double t = now();
  snprintf(out, (size_t)n, "%s: %s; hears it at LQ %d%%, heard at LQ %d%%; channel %d; packets sent %lu (unacknowledged %lu), taken %lu, bad %lu, replays %lu, chunks resent %lu, dropped %lu",
           NP->name, clink_connected(&N->L, t) ? "connected" : "not connected", clink_lq(&N->L, t), N->L.peer_lq, N->ch,
           (unsigned long)c->sent, (unsigned long)N->lost, (unsigned long)c->got, (unsigned long)c->bad, (unsigned long)c->replays, (unsigned long)c->resent, (unsigned long)c->dropped);
  return 1;
}
