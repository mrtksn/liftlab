/* The nRF24L01(+) radio and the packet layer through it: see nrf24.h. Register numbers and bits from Nordic's
 * nRF24L01+ product specification v1.0. */
#include "nrf24.h"

enum { R_CONFIG = 0x00, R_EN_AA = 0x01, R_EN_RXADDR = 0x02, R_SETUP_AW = 0x03, R_SETUP_RETR = 0x04, R_RF_CH = 0x05,
  R_RF_SETUP = 0x06, R_STATUS = 0x07, R_OBSERVE_TX = 0x08, R_RX_ADDR_P0 = 0x0A, R_TX_ADDR = 0x10, R_FIFO_STATUS = 0x17,
  R_DYNPD = 0x1C, R_FEATURE = 0x1D };
enum { C_R_REGISTER = 0x00, C_W_REGISTER = 0x20, C_R_RX_PAYLOAD = 0x61, C_W_TX_PAYLOAD = 0xA0, C_FLUSH_TX = 0xE1,
  C_FLUSH_RX = 0xE2, C_R_RX_PL_WID = 0x60, C_W_ACK_PAYLOAD = 0xA8, C_ACTIVATE = 0x50, C_NOP = 0xFF };
enum { S_RX_DR = 0x40, S_TX_DS = 0x20, S_MAX_RT = 0x10, F_RX_EMPTY = 0x01, F_TX_EMPTY = 0x10 };
#define CONFIG_BASE 0x0C          /* EN_CRC, CRCO: a 2-byte CRC; the interrupts left on (IRQ unused) */

static uint8_t cmd(const nrf24_hal *H, uint8_t c, const uint8_t *in, uint8_t *out, int n) {
  uint8_t tx[1 + CLINK_MTU], rx[1 + CLINK_MTU];
  tx[0] = c; for (int i = 0; i < n; i++) tx[1 + i] = in ? in[i] : C_NOP;
  if (H->xfer(H->ctx, tx, rx, 1 + n)) return 0xFF;
  if (out) for (int i = 0; i < n; i++) out[i] = rx[1 + i];
  return rx[0];                                          /* (the STATUS register, which comes first in every reply) */
}
static uint8_t rreg(const nrf24_hal *H, uint8_t r) { uint8_t v = 0; cmd(H, C_R_REGISTER | r, 0, &v, 1); return v; }
static void wreg(const nrf24_hal *H, uint8_t r, uint8_t v) { cmd(H, C_W_REGISTER | r, &v, 0, 1); }
static uint8_t status(const nrf24_hal *H) { return cmd(H, C_NOP, 0, 0, 0); }

int nrf24_present(const nrf24_hal *H) {
  wreg(H, R_SETUP_AW, 0x01); uint8_t a = rreg(H, R_SETUP_AW);
  wreg(H, R_SETUP_AW, 0x03); uint8_t b = rreg(H, R_SETUP_AW);
  return a == 0x01 && b == 0x03;                         /* (nothing there reads 0x00 or 0xFF) */
}
static void set_channel(nrf24_link *N, int ch) {
  if (ch == N->ch) return;
  if (N->role == PLINK_DRONE) N->H.ce(N->H.ctx, 0);
  wreg(&N->H, R_RF_CH, (uint8_t)ch); N->ch = ch;
  if (N->role == PLINK_DRONE) N->H.ce(N->H.ctx, 1);
}

int nrf24_start(nrf24_link *N, const nrf24_hal *H, const clink_cfg *C, uint32_t session, int role, int kbps, char *err, int en) {
  uint8_t *z = (uint8_t *)N; for (unsigned i = 0; i < sizeof *N; i++) z[i] = 0;
  N->H = *H; N->role = role; N->kbps = kbps; N->ch = -1;
  const char *why = 0;
  H->ce(H->ctx, 0); H->delay_us(H->ctx, 5000);           /* (power-on reset: 100 ms after VCC; the board waited) */
  wreg(H, R_CONFIG, CONFIG_BASE);                         /* powered down while it's set up */
  if (!nrf24_present(H)) why = "no nRF24L01 answers: check VCC (3.3 V), GND, SCK, MOSI, MISO and CSN";
  if (why) { int k = 0; if (en > 0) { while (why[k] && k < en - 1) { err[k] = why[k]; k++; } err[k] = 0; } return -1; }
  uint8_t act = 0x73; cmd(H, C_ACTIVATE, &act, 0, 1);    /* (the older nRF24L01 wants this before FEATURE; the + ignores it) */
  wreg(H, R_EN_AA, 0x01); wreg(H, R_EN_RXADDR, 0x01); wreg(H, R_SETUP_AW, 0x03);
  /* retries: 3, the wait for an acknowledgement long enough for a 32-byte one at the rate (500 µs; 1500 at 250k) */
  wreg(H, R_SETUP_RETR, (uint8_t)((kbps == 250 ? 5 : 1) << 4 | 3));
  wreg(H, R_RF_SETUP, (uint8_t)((kbps == 250 ? 0x20 : kbps == 2000 ? 0x08 : 0x00) | 0x06));   /* the rate; 0 dBm out */
  clink_init(&N->L, C, session); N->L.C.role = role;
  clink_address(C, N->addr);
  cmd(H, C_W_REGISTER | R_RX_ADDR_P0, N->addr, 0, 5); cmd(H, C_W_REGISTER | R_TX_ADDR, N->addr, 0, 5);
  wreg(H, R_FEATURE, 0x06); wreg(H, R_DYNPD, 0x01);      /* dynamic payload lengths, payloads with the acknowledgements */
  cmd(H, C_FLUSH_TX, 0, 0, 0); cmd(H, C_FLUSH_RX, 0, 0, 0); wreg(H, R_STATUS, S_RX_DR | S_TX_DS | S_MAX_RT);
  wreg(H, R_RF_CH, (uint8_t)clink_channel(&N->L, -1e9)); N->ch = rreg(H, R_RF_CH);
  wreg(H, R_CONFIG, (uint8_t)(CONFIG_BASE | 0x02 | (role == PLINK_DRONE ? 0x01 : 0)));   /* power up; the drone receives */
  H->delay_us(H->ctx, 5000);                             /* (1.5 ms to standby, and a margin) */
  if (role == PLINK_DRONE) H->ce(H->ctx, 1);             /* listening from now */
  return 0;
}

/* a packet the radio has: its width, then the bytes (a width over 32 is a corrupt one: the FIFO flushed) */
static int read_payload(nrf24_link *N, uint8_t *p) {
  uint8_t w = 0; cmd(&N->H, C_R_RX_PL_WID, 0, &w, 1);
  if (w < 1 || w > CLINK_MTU) { cmd(&N->H, C_FLUSH_RX, 0, 0, 0); N->too_long++; return 0; }
  cmd(&N->H, C_R_RX_PAYLOAD, 0, p, w);
  return w;
}

void nrf24_poll(nrf24_link *N, double t) {
  const nrf24_hal *H = &N->H; uint8_t p[CLINK_MTU];
  if (N->role == PLINK_GROUND) {
    if (N->busy) {
      uint8_t s = status(H);
      if (s & S_TX_DS) {                                  /* acknowledged: the drone's answer with it, if it had one */
        if (s & S_RX_DR) { int n = read_payload(N, p); if (n) clink_from_air(&N->L, p, n, 0, t); }
        wreg(H, R_STATUS, S_RX_DR | S_TX_DS | S_MAX_RT); N->busy = 0;
      } else if (s & S_MAX_RT) {                          /* four tries, no acknowledgement: lost */
        cmd(H, C_FLUSH_TX, 0, 0, 0); wreg(H, R_STATUS, S_MAX_RT); N->busy = 0; N->lost++;
      } else if (t - N->t_busy > 0.02) {                  /* (no word from the radio: start it again) */
        cmd(H, C_FLUSH_TX, 0, 0, 0); wreg(H, R_STATUS, S_RX_DR | S_TX_DS | S_MAX_RT); N->busy = 0; N->timeouts++;
      }
      if (N->busy) return;
    }
    set_channel(N, clink_channel(&N->L, t));             /* (the channel for the packet about to go) */
    int n = clink_to_air(&N->L, t, p, sizeof p); if (!n) return;
    cmd(H, C_W_TX_PAYLOAD, p, 0, n);
    H->ce(H->ctx, 1); H->delay_us(H->ctx, 15); H->ce(H->ctx, 0);   /* a pulse of 10 µs or more sends it */
    N->busy = 1; N->t_busy = t;
    return;
  }
  /* the drone */
  uint8_t s = status(H);
  if (s & S_RX_DR) {
    for (int k = 0; k < 3; k++) {                         /* (the receive FIFO holds three) */
      int n = read_payload(N, p); if (n) clink_from_air(&N->L, p, n, 0, t);
      if (rreg(H, R_FIFO_STATUS) & F_RX_EMPTY) break;
    }
    wreg(H, R_STATUS, S_RX_DR);
  }
  if (s & (S_TX_DS | S_MAX_RT)) wreg(H, R_STATUS, S_TX_DS | S_MAX_RT);   /* (an answer went with an acknowledgement) */
  if (rreg(H, R_FIFO_STATUS) & F_TX_EMPTY) {              /* the last answer gone: the next loaded, for the next acknowledgement */
    int n = clink_to_air(&N->L, t, p, sizeof p);
    if (n) cmd(H, C_W_ACK_PAYLOAD | 0, p, 0, n);
  }
  set_channel(N, clink_channel(&N->L, t));               /* (where the ground's next packet comes) */
}
