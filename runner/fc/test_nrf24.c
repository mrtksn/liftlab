/* Tests for the nRF24L01 driver (nrf24.c) against two emulated modules: their registers, FIFOs and Enhanced
 * ShockBurst (a packet, its acknowledgement with the payload loaded for it, up to 3 retries, the channel, address and
 * data rate both ends must share), with the air losing a share of the packets and of the acknowledgements. The
 * driver runs as on a board: polled every 2 ms; through it, clink (clink.c) carries the channels, commands,
 * telemetry and messages.
 *   cc -O2 -I.. -o test_nrf24 test_nrf24.c nrf24.c clink.c plink.c radio_link.c crsf.c tlm_crsf.c tlm_core.c rc_core.c pickup_core.c -lm */
#include "nrf24.h"
#include "tlm_crsf.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)
static uint32_t seed = 123456789u;
static double rnd(void) { seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; return seed / 4294967296.0; }

/* ── an emulated nRF24L01+ ── */
typedef struct { uint8_t b[32]; int n; } slot;
typedef struct chip chip;
struct chip {
  uint8_t reg[32], rx_addr[5], tx_addr[5];
  slot rx[3]; int rx_n; slot tx[3]; int tx_n;
  int ce, present, pid_last, pid; chip *other;
  uint32_t sent, hops, writes;
};
static double loss_pkt, loss_ack; static int cut;
static void push(slot *q, int *n, const uint8_t *b, int len) { if (*n < 3) { memcpy(q[*n].b, b, (size_t)len); q[*n].n = len; (*n)++; } }
static void pop(slot *q, int *n) { for (int i = 1; i < *n; i++) q[i - 1] = q[i]; if (*n) (*n)--; }
static uint8_t fifo_status(chip *c) { return (uint8_t)((c->rx_n == 0 ? 0x01 : 0) | (c->rx_n == 3 ? 0x02 : 0) | (c->tx_n == 0 ? 0x10 : 0) | (c->tx_n == 3 ? 0x20 : 0)); }
static uint8_t chip_status(chip *c) { return (uint8_t)((c->reg[7] & 0x70) | (c->rx_n ? 0 : 0x0E) | (c->tx_n == 3 ? 1 : 0)); }
static int listening(chip *c) { return (c->reg[0] & 0x03) == 0x03 && c->ce; }
static int same_link(chip *a, chip *b) { return a->reg[5] == b->reg[5] && (a->reg[6] & 0x28) == (b->reg[6] & 0x28) && !memcmp(a->tx_addr, b->rx_addr, 5); }
/* the PTX sends its FIFO's head: the PRX takes it (a new PID) and acknowledges with its loaded payload; up to ARC retries */
static void transmit(chip *p) {
  chip *r = p->other; if (!p->tx_n) return;
  int arc = p->reg[4] & 15; p->pid = (p->pid + 1) & 3; p->sent++;
  for (int a = 0; a <= arc; a++) {
    if (cut || rnd() < loss_pkt || !r || !r->present || !listening(r) || !same_link(p, r)) continue;
    if (r->pid_last != p->pid) { r->pid_last = p->pid; if (r->rx_n < 3) { push(r->rx, &r->rx_n, p->tx[0].b, p->tx[0].n); r->reg[7] |= 0x40; } }
    if (rnd() < loss_ack) { if (r->tx_n) { pop(r->tx, &r->tx_n); r->reg[7] |= 0x20; } continue; }   /* (its payload went with the lost acknowledgement) */
    if (r->tx_n) { push(p->rx, &p->rx_n, r->tx[0].b, r->tx[0].n); p->reg[7] |= 0x40; pop(r->tx, &r->tx_n); r->reg[7] |= 0x20; }
    pop(p->tx, &p->tx_n); p->reg[7] |= 0x20; return;
  }
  p->reg[7] |= 0x10;                                       /* MAX_RT: the payload stays until flushed */
}
static int chip_xfer(void *ctx, const uint8_t *tx, uint8_t *rx, int n) {
  chip *c = ctx; if (!c->present) { memset(rx, 0, (size_t)n); return 0; }
  uint8_t op = tx[0]; rx[0] = chip_status(c);
  if (op == 0xFF || op == 0x50) return 0;
  if ((op & 0xE0) == 0x00) {                               /* R_REGISTER */
    int r = op & 31;
    for (int i = 1; i < n; i++) rx[i] = r == 0x0A ? c->rx_addr[i - 1] : r == 0x10 ? c->tx_addr[i - 1] : r == 0x17 ? fifo_status(c) : r == 7 ? chip_status(c) : c->reg[r];
  } else if ((op & 0xE0) == 0x20) {                        /* W_REGISTER */
    int r = op & 31; c->writes++;
    if (r == 0x0A) memcpy(c->rx_addr, tx + 1, 5); else if (r == 0x10) memcpy(c->tx_addr, tx + 1, 5);
    else if (r == 7) c->reg[7] &= (uint8_t)~(tx[1] & 0x70);
    else { if (r == 5 && c->reg[5] != tx[1]) c->hops++; c->reg[r] = tx[1]; }
  } else if (op == 0x60) rx[1] = (uint8_t)(c->rx_n ? c->rx[0].n : 0);
  else if (op == 0x61) { for (int i = 1; i < n; i++) rx[i] = c->rx_n ? c->rx[0].b[i - 1] : 0; pop(c->rx, &c->rx_n); }
  else if (op == 0xA0 || op == 0xA8) push(c->tx, &c->tx_n, tx + 1, n - 1);
  else if (op == 0xE1) c->tx_n = 0;
  else if (op == 0xE2) c->rx_n = 0;
  return 0;
}
static void chip_ce(void *ctx, int v) { chip *c = ctx; int was = c->ce; c->ce = v; if (v && !was && !(c->reg[0] & 1) && (c->reg[0] & 2)) transmit(c); }
static void chip_delay(void *ctx, int us) { (void)ctx; (void)us; }

/* ── the two ends ── */
typedef struct { crsf_parser P; int rc, att, ncmd, ntext, bad; int cmd_seq[2048]; crsf_link last; } reader;
static void read_stack(reader *R, clink *L, double t) {
  uint8_t b[PLINK_OUT]; int n = clink_to_stack(L, t, b, sizeof b);
  for (int i = 0; i < n; i++) {
    int len = crsf_feed(&R->P, b[i]); if (len < 0) R->bad++; if (len <= 0) continue;
    const uint8_t *f = R->P.buf, *p = f + 3;
    if (f[2] == CRSF_RC) R->rc++; else if (f[2] == CRSF_ATTITUDE) R->att++;
    else if (f[2] == CRSF_LINK_STATS) crsf_link_stats_read(p, f[1] - 2, &R->last);
    else if (f[2] == CRSF_EXT && p[0] == CRSF_EXT_CMD && R->ncmd < 2048) R->cmd_seq[R->ncmd++] = p[2];
    else if (f[2] == CRSF_EXT && p[0] == CRSF_EXT_TEXT) R->ntext++;
  }
}
static chip cg, cd; static nrf24_link G, D; static reader rg, rd; static double T; static int cmds, texts; static long tlm;
static int start(int kbps, const char *gp, const char *dp) {
  memset(&cg, 0, sizeof cg); memset(&cd, 0, sizeof cd); memset(&rg, 0, sizeof rg); memset(&rd, 0, sizeof rd); T = 0; cmds = texts = 0; tlm = 0;
  cg.present = cd.present = 1; cg.other = &cd; cd.other = &cg; cd.pid_last = -1;
  nrf24_hal hg = { chip_xfer, chip_ce, chip_delay, &cg }, hd = { chip_xfer, chip_ce, chip_delay, &cd };
  clink_cfg a, b; clink_cfg_default(&a, PLINK_GROUND); clink_cfg_default(&b, PLINK_DRONE);
  rlink_cfg L; rlink_default(&L); rlink_make(&L, RLINK_NRF24, kbps, 0); clink_cfg_link(&a, &L); clink_cfg_link(&b, &L);
  plink_key(gp, &a.k0, &a.k1); plink_key(dp, &b.k0, &b.k1);
  char e1[120], e2[120];
  return nrf24_start(&G, &hg, &a, 0x1001, PLINK_GROUND, kbps, e1, sizeof e1) | nrf24_start(&D, &hd, &b, 0x2002, PLINK_DRONE, kbps, e2, sizeof e2);
}
static void run(double secs, float room) {
  double due = 0; int steps = (int)lround(secs * 1000);
  for (int k = 0; k < steps; k++) {
    if (k % 4 == 0) { uint8_t b[64]; float ch[16] = { -0.5f }; int n = crsf_rc(b, CRSF_ADDR_FC, ch); clink_from_stack(&G.L, b, n, T); }
    if (k % 100 == 50) { uint8_t b[64]; float v[2] = { 1, 1 }; cmds++; int n = tlm_crsf_cmd(b, RC_CMD_LATCH, (cmds - 1) % 255 + 1, v, 2); clink_from_stack(&G.L, b, n, T); }
    if (k % 500 == 250) { uint8_t b[64]; char s[48]; snprintf(s, sizeof s, "a message from the drone, number %d", ++texts); int n = crsf_text(b, 4, s); clink_from_stack(&D.L, b, n, T); }
    due += room * 0.001 * (rd.last.up_lq > 0 ? rd.last.up_lq / 100 : 1);
    while (due >= 10) { uint8_t b[16]; int n = crsf_attitude(b, 0.1f, 0.2f, 0.3f); clink_from_stack(&D.L, b, n, T); due -= n; tlm += n; }
    if (k % 2 == 0) { nrf24_poll(&G, T); nrf24_poll(&D, T + 0.0005); read_stack(&rd, &D.L, T); read_stack(&rg, &G.L, T); }
    T += 0.001;
  }
}
static int in_order(const reader *R, int want) { if (R->ncmd < want - 2 || R->ncmd > want) return 0; for (int i = 0; i < R->ncmd; i++) if (R->cmd_seq[i] != (i % 255) + 1) return 0; return 1; }

int main(void) {
  printf("setting the modules up\n");
  for (int r = 0; r < 3; r++) {
    int kbps = (int[]){ 250, 1000, 2000 }[r];
    int e = start(kbps, "phrase", "phrase");
    uint8_t want_rf = (uint8_t)((kbps == 250 ? 0x20 : kbps == 2000 ? 0x08 : 0) | 6);
    CHECK(!e && cg.reg[6] == want_rf && cd.reg[6] == want_rf && (cg.reg[4] & 15) == 3 && (cg.reg[4] >> 4) == (kbps == 250 ? 5 : 1) && cg.reg[0x1D] == 6 && cg.reg[0x1C] == 1
          && (cg.reg[0] & 3) == 2 && (cd.reg[0] & 3) == 3 && cd.ce && !cg.ce && !memcmp(cg.tx_addr, cd.rx_addr, 5) && !memcmp(cg.rx_addr, cg.tx_addr, 5),
          "%4d kbit/s: RF_SETUP 0x%02X (0 dBm), 3 retries %d µs apart, dynamic payloads and ACK payloads, CRC 2 bytes; the ground a powered-up transmitter, the drone listening; one address at both ends",
          kbps, cg.reg[6], ((cg.reg[4] >> 4) + 1) * 250);
  }
  { memset(&cg, 0, sizeof cg); nrf24_hal hg = { chip_xfer, chip_ce, chip_delay, &cg }; clink_cfg a; clink_cfg_default(&a, PLINK_GROUND); char err[120] = "";
    int e = nrf24_start(&G, &hg, &a, 1, PLINK_GROUND, 1000, err, sizeof err);
    CHECK(e == -1 && strstr(err, "no nRF24L01"), "no module on the bus: said, not flown blind (\"%s\")", err); }

  printf("1 Mbit/s, a clean link\n");
  loss_pkt = loss_ack = 0; cut = 0; start(1000, "phrase", "phrase"); run(5, 1260);
  CHECK(rd.rc >= 5 * 100 * 0.85 && in_order(&rd, cmds), "channels %d in 5 s, commands %d of %d in order", rd.rc, rd.ncmd, cmds);
  CHECK(rg.att * 10 >= tlm * 0.95 && rg.ntext >= texts - 1 && rd.bad == 0 && rg.bad == 0, "telemetry at its room (1260 B/s): %d of %ld frames; messages %d of %d; nothing broken", rg.att, tlm / 10, rg.ntext, texts);
  CHECK(cg.hops > 400 && cd.hops > 400 && G.lost <= 8 && G.timeouts == 0, "both hop, a channel a packet (%u and %u channel changes); unacknowledged only while the drone first waits for the ground's round of channels (%u)", cg.hops, cd.hops, G.lost);
  CHECK(rd.last.up_lq >= 99 && rg.last.down_lq >= 99, "link quality 100%% both ways");

  printf("250 kbit/s, 15%% of packets and 15%% of acknowledgements lost\n");
  loss_pkt = loss_ack = 0.15; start(250, "phrase", "phrase"); run(10, 630);
  CHECK(rd.rc >= 10 * 50 * 0.75 && in_order(&rd, cmds) && rg.ntext >= texts - 1, "channels %d in 10 s (50 a second), commands %d of %d, messages %d of %d (the radio's retries: %u packets lost after 4 tries)", rd.rc, rd.ncmd, cmds, rg.ntext, texts, G.lost);
  CHECK(rd.bad == 0 && rg.bad == 0 && rg.att * 10 >= tlm * 0.85, "nothing broken; telemetry %d of %ld frames", rg.att, tlm / 10);

  printf("the link cut for 2 s\n");
  loss_pkt = loss_ack = 0; start(2000, "phrase", "phrase"); run(2, 1260); cut = 1; run(2, 1260);
  { float up = rg.last.up_lq; cut = 0; int rc0 = rd.rc; run(0.5, 1260);
    CHECK(up == 0 && rd.rc - rc0 >= 25 && in_order(&rd, cmds), "the command module sees uplink 0%%; after, the drone finds the ground's hops again: %d channel frames in 0.5 s; the commands held arrive in order (%d of %d)", rd.rc - rc0, rd.ncmd, cmds); }

  printf("another binding phrase at one end\n");
  loss_pkt = loss_ack = 0; start(1000, "phrase", "another"); run(3, 1260);
  CHECK(rd.rc == 0 && rd.ncmd == 0 && rg.att == 0 && G.lost > 250, "nothing gets through: another address and other channels (%u packets unacknowledged)", G.lost);

  printf(fails ? "%d FAILED\n" : "all passed\n", fails);
  return fails != 0;
}
