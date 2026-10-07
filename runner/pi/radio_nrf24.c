/* The nRF24L01 link on a Pi or any Linux computer with SPI (radio_io.h): the drone's end (dfb_pi --radio nrf24,1000
 * --nrf-spi /dev/spidev0.0 --nrf-ce 25) or the command module's (dfb_ground, the same). The module on spidev (8 MHz,
 * mode 0; the kernel drives CSN: CE0 is GPIO 8, CE1 GPIO 7), its CE on a GPIO line of /dev/gpiochip0; the radio's
 * part in ../fc/nrf24.c, the packet layer in ../fc/clink.c, the same as on an ESP32 and in the simulator.
 * On a Pi: SPI on in raspi-config (dtparam=spi=on); the module's VCC on 3.3 V (pin 17), never 5 V, with a 10 µF
 * capacitor across it at the module; SCK pin 23, MOSI 19, MISO 21, CSN pin 24 (CE0), CE e.g. GPIO 25 (pin 22).
 * Not on a Mac (no SPI): radio_nrf24_open says so. */
#define _DEFAULT_SOURCE
#include "radio_nrf24.h"
#include "radio_session.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#ifdef __linux__
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>
#endif

typedef struct { radio_io io; nrf24_link N; int spi, ce; char name[64]; } nrf_t;
static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

#ifdef __linux__
static int hal_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, int n) {
  nrf_t *K = ctx; struct spi_ioc_transfer x; memset(&x, 0, sizeof x);
  x.tx_buf = (unsigned long)tx; x.rx_buf = (unsigned long)rx; x.len = (uint32_t)n; x.speed_hz = 8000000; x.bits_per_word = 8;
  return ioctl(K->spi, SPI_IOC_MESSAGE(1), &x) < 0 ? -1 : 0;
}
static void hal_ce(void *ctx, int v) {
  nrf_t *K = ctx; struct gpiohandle_data d; memset(&d, 0, sizeof d); d.values[0] = (uint8_t)(v ? 1 : 0);
  ioctl(K->ce, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &d);
}
static void hal_delay(void *ctx, int us) { (void)ctx; usleep((useconds_t)us); }
#endif

static int nrf_read(radio_io *R, uint8_t *b, int n, int wait_ms) {
  nrf_t *K = R->ctx; (void)wait_ms;                  /* (the program polls: this never waits) */
  double t = now_s(); nrf24_poll(&K->N, t);
  return clink_to_stack(&K->N.L, t, b, n);
}
static int nrf_write(radio_io *R, const uint8_t *b, int n) {
  nrf_t *K = R->ctx; double t = now_s(); clink_from_stack(&K->N.L, b, n, t); nrf24_poll(&K->N, t); return n;
}

static uint32_t nrf_peer(radio_io *R) { return ((nrf_t *)R->ctx)->N.L.known; }
static void nrf_hear(radio_io *R, int lq, int rssi) { (void)rssi; clink_hear(&((nrf_t *)R->ctx)->N.L, lq); }
radio_io *radio_nrf24_open(int role, const rlink_cfg *L, const char *spidev, int ce_line, const char *phrase, const char *name) {
#ifndef __linux__
  (void)role; (void)L; (void)spidev; (void)ce_line; (void)phrase;
  fprintf(stderr, "%s: an nRF24L01 needs SPI: on a Pi or another Linux computer (not here)\n", name); return 0;
#else
  if (L->kind != RLINK_NRF24) return 0;
  int spi = open(spidev, O_RDWR);
  if (spi < 0) { fprintf(stderr, "%s: %s: %s (SPI on? raspi-config, or dtparam=spi=on in config.txt)\n", name, spidev, strerror(errno)); return 0; }
  uint8_t mode = SPI_MODE_0, bits = 8; uint32_t speed = 8000000;
  ioctl(spi, SPI_IOC_WR_MODE, &mode); ioctl(spi, SPI_IOC_WR_BITS_PER_WORD, &bits); ioctl(spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
  int chip = open("/dev/gpiochip0", O_RDWR);
  if (chip < 0) { fprintf(stderr, "%s: /dev/gpiochip0: %s\n", name, strerror(errno)); close(spi); return 0; }
  struct gpiohandle_request rq; memset(&rq, 0, sizeof rq);
  rq.lineoffsets[0] = (uint32_t)ce_line; rq.lines = 1; rq.flags = GPIOHANDLE_REQUEST_OUTPUT; rq.default_values[0] = 0; snprintf(rq.consumer_label, sizeof rq.consumer_label, "nrf24 ce");
  int r = ioctl(chip, GPIO_GET_LINEHANDLE_IOCTL, &rq); int e = errno; close(chip);
  if (r < 0) { fprintf(stderr, "%s: GPIO %d for CE: %s\n", name, ce_line, strerror(e)); close(spi); return 0; }
  radio_io *R = calloc(1, sizeof *R); nrf_t *K = calloc(1, sizeof *K);
  if (!R || !K) { free(R); free(K); close(spi); close(rq.fd); return 0; }
  K->spi = spi; K->ce = rq.fd;
  nrf24_hal H = { hal_xfer, hal_ce, hal_delay, K };
  clink_cfg C; clink_cfg_default(&C, role); clink_cfg_link(&C, L); plink_key(phrase ? phrase : "liftlab", &C.k0, &C.k1);
  uint32_t ses = radio_session();
  char err[120];
  if (nrf24_start(&K->N, &H, &C, ses, role, L->kbps, err, sizeof err)) { fprintf(stderr, "%s: %s\n", name, err); close(spi); close(rq.fd); free(R); free(K); return 0; }
  snprintf(K->name, sizeof K->name, "nRF24L01 at %d kbit/s", L->kbps);
  R->name = K->name; R->read = nrf_read; R->write = nrf_write; R->fd = -1; R->ctx = K;
  R->peer = nrf_peer; R->hear = nrf_hear;
  return R;
#endif
}
void radio_nrf24_close(radio_io *R) {
  if (!R) return;
  nrf_t *K = R->ctx;
#ifdef __linux__
  hal_ce(K, 0); close(K->spi); close(K->ce);
#endif
  free(K); free(R);
}
int radio_nrf24_is(const radio_io *R) { return R && R->read == nrf_read; }
void radio_nrf24_counts(const radio_io *R, char *out, int n) {
  const nrf_t *K = R->ctx; const clink_counts *c = &K->N.L.N; double t = now_s();
  snprintf(out, (size_t)n, "nrf24,%d: channel %d; packets sent %u (unacknowledged %u), got %u (LQ %d%%, heard at %d%%), bad %u, replays %u; chunks resent %u",
           K->N.kbps, K->N.ch, c->sent, K->N.lost, c->got, clink_lq(&K->N.L, t), K->N.L.peer_lq, c->bad, c->replays, c->resent);
}
