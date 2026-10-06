/* Tests for the serial link: the framing (pframe.c) and the packet layer (plink.c) over a simulated serial line, at
 * the line's speed, with bytes damaged or lost on the way, both ways at once or one way at a time.
 *   cc -O2 -I.. -o test_pserial test_pserial.c pframe.c plink.c radio_link.c crsf.c tlm_crsf.c rc_core.c -lm */
#include "pframe.h"
#include "plink.h"
#include "radio_link.h"
#include "tlm_crsf.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)
static uint32_t seed = 2463534242u;
static double rnd(void) { seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; return seed / 4294967296.0; }

/* ── the line: bytes one after another at the line's speed; each damaged (a bit flipped) or lost with a chance ── */
#define LQN 65536
typedef struct { uint8_t b[LQN]; double at[LQN]; int head, tail; double free_at; double start; } wire;   /* one direction */
typedef struct {
  wire w[2];                   /* [0] up (ground → drone), [1] down */
  double byte_s, flip, drop;   /* one byte's time [s]; the chances */
  double delay;                /* [s] more on the way (a radio modem: it takes in the bytes, sends them, hands them out) */
  int half; uint32_t collisions;
} line;
static void line_send(line *L, int dir, const uint8_t *b, int n, double t) {
  wire *W = &L->w[dir], *O = &L->w[!dir];
  double start = t > W->free_at ? t : W->free_at;
  if (L->half && O->tail && O->free_at + L->delay > start) L->collisions++;   /* the other end still talking (its bytes still on the air): both garbled */
  if (start >= W->free_at) W->start = start;
  for (int i = 0; i < n; i++) {
    uint8_t x = b[i]; double at = start + (i + 1) * L->byte_s + L->delay;
    if (L->half && at - L->byte_s < O->free_at && at > O->start) x ^= (uint8_t)(1u << (int)(rnd() * 8));
    if (rnd() < L->drop) continue;
    if (rnd() < L->flip) x ^= (uint8_t)(1u << (int)(rnd() * 8));
    W->b[W->tail % LQN] = x; W->at[W->tail % LQN] = at; W->tail++;
  }
  W->free_at = start + n * L->byte_s;
}

typedef struct { crsf_parser P; int rc, cmd, att, stats; int cmd_seq[4096], ncmd; crsf_link last; } reader;
static void read_stack(reader *R, plink *L, double t) {
  uint8_t b[PLINK_OUT]; int n = plink_to_stack(L, t, b, sizeof b);
  for (int i = 0; i < n; i++) {
    int len = crsf_feed(&R->P, b[i]); if (len <= 0) continue;
    const uint8_t *f = R->P.buf, *p = f + 3;
    if (f[2] == CRSF_RC) R->rc++;
    else if (f[2] == CRSF_LINK_STATS) { R->stats++; crsf_link_stats_read(p, f[1] - 2, &R->last); }
    else if (f[2] == CRSF_ATTITUDE) R->att++;
    else if (f[2] == CRSF_EXT && p[0] == CRSF_EXT_CMD) { if (R->ncmd < 4096) R->cmd_seq[R->ncmd++] = p[2]; R->cmd++; }
  }
}
typedef struct { plink G, D; pframe_rx rg, rd; line L; reader g, d; rlink_cfg RL; double t; int cmds, hold; long tlm_bytes; float room; } world;
static void world_init(world *W, int baud, int half, double flip, double drop) {
  memset(W, 0, sizeof *W);
  rlink_cfg RL; rlink_default(&RL); if (rlink_make(&RL, RLINK_SERIAL, baud, half)) { printf("  (can't set serial,%d)\n", baud); exit(1); }
  W->RL = RL; W->room = rlink_budget(&RL);
  plink_cfg cg, cd; plink_cfg_default(&cg, PLINK_GROUND); plink_cfg_default(&cd, PLINK_DRONE);
  plink_cfg_link(&cg, &RL); plink_cfg_link(&cd, &RL);
  plink_key("phrase", &cg.k0, &cg.k1); plink_key("phrase", &cd.k0, &cd.k1);
  plink_init(&W->G, &cg, 0x1111); plink_init(&W->D, &cd, 0x2222);
  pframe_rx_init(&W->rg); pframe_rx_init(&W->rd);
  W->L.byte_s = 10.0 / baud; W->L.flip = flip; W->L.drop = drop; W->L.half = half;
}
static void send(world *W, int dir, plink *from, double t) {
  uint8_t pk[PLINK_MTU], wbuf[PFRAME_WIRE(PLINK_MTU)];
  int n = plink_to_air(from, t, pk, sizeof pk); if (!n) return;
  int m = pframe_encode(pk, n, wbuf, sizeof wbuf); line_send(&W->L, dir, wbuf, m, t);
}
static void deliver(world *W, int dir, plink *to, pframe_rx *R, double t) {
  wire *Q = &W->L.w[dir]; uint8_t pk[PFRAME_MAX];
  while (Q->head < Q->tail && Q->at[Q->head % LQN] <= t) {
    int n = pframe_feed(R, Q->b[Q->head % LQN], pk, sizeof pk); Q->head++;
    if (n) plink_from_air(to, pk, n, 0, t);
  }
}
/* 1 ms steps; the program at each end runs every 2 ms (as the ESP32's radio task); the ground's stack writes the
 * channels every 4 ms, the drone's the telemetry at the link's room (attitude frames: 10 bytes each), a command
 * every 0.1 s */
static void run(world *W, double secs) {
  double tlm_due = 0; int steps = (int)lround(secs * 1000);
  for (int k = 0; k < steps; k++) {
    double t = W->t;
    deliver(W, 0, &W->D, &W->rd, t); deliver(W, 1, &W->G, &W->rg, t);
    if (k % 4 == 0) { uint8_t b[64]; float ch[16] = { 0.3f }; int n = crsf_rc(b, CRSF_ADDR_FC, ch); plink_from_stack(&W->G, b, n, t); }
    if (k % 100 == 50 && !(W->hold && W->g.last.up_lq == 0)) { uint8_t b[64];   /* (hold: as the command module, none while its link is down) */ float v[2] = { 1, 1 }; W->cmds++; int n = tlm_crsf_cmd(b, RC_CMD_LATCH, (W->cmds - 1) % 255 + 1, v, 2); plink_from_stack(&W->G, b, n, t); }
    tlm_due += W->room * 0.001;
    while (tlm_due >= 10) { uint8_t b[16]; int n = crsf_attitude(b, 0.1f, 0.2f, 0.3f); plink_from_stack(&W->D, b, n, t); tlm_due -= n; W->tlm_bytes += n; }
    if (k % 2 == 0) { send(W, 0, &W->G, t); send(W, 1, &W->D, t); read_stack(&W->d, &W->D, t); read_stack(&W->g, &W->G, t); }
    W->t += 0.001;
  }
}
static int cmds_in_order(const reader *R, int want) { if (R->ncmd < want - 1 || R->ncmd > want) return 0; for (int i = 0; i < R->ncmd; i++) if (R->cmd_seq[i] != (i % 255) + 1) return 0; return 1; }

int main(void) {
  printf("framing (COBS between zeros)\n");
  {
    int ok = 1, maxover = 0; pframe_rx R; pframe_rx_init(&R);
    for (int k = 0; k < 4000 && ok; k++) {
      uint8_t p[PFRAME_MAX], w[PFRAME_WIRE(PFRAME_MAX)], q[PFRAME_MAX]; int n = 1 + (int)(rnd() * PFRAME_MAX);
      if (k < 8) n = (int[]){ 1, 253, 254, 255, 256, 2, 508 / 2, 100 }[k];
      int mode = k % 4;                                              /* random, mostly zeros, no zeros (long blocks), all 0xFF */
      for (int i = 0; i < n; i++) p[i] = mode == 0 ? (uint8_t)(rnd() * 256) : mode == 1 ? (rnd() < 0.7 ? 0 : (uint8_t)(rnd() * 256)) : mode == 2 ? (uint8_t)(1 + rnd() * 255) : 0xFF;
      int m = pframe_encode(p, n, w, sizeof w), zeros = 0; for (int i = 1; i < m - 1; i++) zeros += !w[i];
      if (m - n - 3 > maxover) maxover = m - n - 3;
      int got = 0; for (int i = 0; i < m; i++) { int r = pframe_feed(&R, w[i], q, sizeof q); if (r) { got = r; if (r != n || memcmp(p, q, (size_t)n)) ok = 0; } }
      if (!got || zeros || m > PFRAME_WIRE(n)) ok = 0;
    }
    CHECK(ok && R.bad == 0, "4000 packets of 1–256 bytes (random, mostly zeros, no zeros, all 0xFF) come back whole; no zero inside a frame; at most %d byte%s more than the packet and its two zeros", maxover, maxover == 1 ? "" : "s");
    pframe_rx_init(&R); int good = 0, n_sent = 0;
    for (int k = 0; k < 2000; k++) {                                 /* noise on the line between packets, and damage inside them */
      uint8_t p[80], w[PFRAME_WIRE(80)], q[PFRAME_MAX]; for (int i = 0; i < 80; i++) p[i] = (uint8_t)(rnd() * 256);
      int m = pframe_encode(p, 80, w, sizeof w); n_sent++;
      int noise = (int)(rnd() * 6); for (int i = 0; i < noise; i++) pframe_feed(&R, (uint8_t)(rnd() * 256), q, sizeof q);
      int hurt = k % 5 == 0; if (hurt) { int i = 1 + (int)(rnd() * (m - 2)); if (rnd() < 0.5) w[i] ^= (uint8_t)(1u << (int)(rnd() * 8)); else { memmove(w + i, w + i + 1, (size_t)(m - i - 1)); m--; } }
      for (int i = 0; i < m; i++) { int r = pframe_feed(&R, w[i], q, sizeof q); if (r == 80 && !memcmp(p, q, 80)) good++; }
    }
    CHECK(good >= 1600 * 0.98, "with noise between frames and a fifth of the frames damaged, the undamaged ones still arrive: %d of %d (%u frames didn't decode)", good, n_sent, R.bad);
  }

  printf("speeds: packets and room\n");
  {
    int bauds[] = { 19200, 38400, 57600, 115200, 230400, 460800, 921600 }, ok = 1;
    for (int h = 0; h < 2; h++) for (int i = 0; i < 7; i++) {
      rlink_cfg L; rlink_default(&L); if (rlink_make(&L, RLINK_SERIAL, bauds[i], h)) { if (!(h && bauds[i] < RLINK_BAUD_HALF_MIN)) ok = 0; continue; }
      int mtu, mtu_up, half; float up, dmin, dmax; rlink_sizing(&L, &mtu, &mtu_up, &up, &dmin, &dmax, &half);
      float B = bauds[i] / 10.0f, room = rlink_budget(&L);
      /* the line's busiest: up, every packet as big as it may be; down, every packet full at its most a second (one
       * way at a time: a cycle of the biggest of each) */
      float up_use = up * PFRAME_WIRE(mtu_up) / B, down_use = dmax * PFRAME_WIRE(mtu) / B, cyc = up * (PFRAME_WIRE(mtu_up) + PFRAME_WIRE(mtu)) / B;
      printf("    %7d baud %s: up %5.1f a second (≤ %3d bytes), down %4.1f–%4.1f (≤ %3d bytes), telemetry %4.0f B/s\n", bauds[i], h ? "half" : "full", up, mtu_up, dmin, dmax, mtu, room);
      if (h) printf("        the biggest packet each way, a cycle a beat: %.0f%% of the line (the rest: the turnarounds)\n", 100 * cyc);
      else printf("        up %.0f%%, down %.0f%% of the line\n", 100 * up_use, 100 * down_use);
      if (up < 15 || room <= 0 || mtu < 64 || mtu > 250 || (h ? cyc > 0.95f : up_use > 1 || down_use > 1) || half != h) ok = 0;
    }
    CHECK(ok, "every speed sends the channels at least 15 times a second, leaves the telemetry room and doesn't overfill the line");
    char err[200]; rlink_cfg L; rlink_default(&L);
    CHECK(rlink_parse(&L, "serial,9600", err, sizeof err) && rlink_parse(&L, "serial,19200,half", err, sizeof err) && !rlink_parse(&L, "serial,38400,half", err, sizeof err) && L.half && L.baud == 38400,
          "too slow is refused (9600; 19200 one way at a time), 38400 one way at a time is taken");
    char d[32]; rlink_parse(&L, "serial,921600", err, sizeof err); rlink_describe(&L, d, sizeof d);
    CHECK(!strcmp(d, "serial,921600") && rlink_packets(&L) && !rlink_parse(&L, d, err, sizeof err), "serial,921600 reads back as it was written: %s", d);
    CHECK(rlink_parse(&L, "serial,115200,full", err, sizeof err) && rlink_parse(&L, "serial,", err, sizeof err) && rlink_parse(&L, "serial,5000000", err, sizeof err), "nonsense is refused: %s", err);
  }

  static world W;
  printf("both ways at once, 115200 baud, a clean line\n");
  world_init(&W, 115200, 0, 0, 0); run(&W, 5);
  CHECK(W.d.rc >= 5 * W.G.C.up_hz * 0.97, "the drone gets the channels: %d frames in 5 s (%.0f a second)", W.d.rc, W.G.C.up_hz);
  CHECK(cmds_in_order(&W.d, W.cmds), "every command once, in order: %d of %d", W.d.ncmd, W.cmds);
  CHECK(W.D.N.uq_dropped == 0 && W.g.att * 10 >= W.tlm_bytes * 0.97, "telemetry written at the link's room (%.0f B/s) all comes down: %d of %ld frames, none dropped", W.room, W.g.att, W.tlm_bytes / 10);
  CHECK(W.d.last.up_lq >= 99 && W.g.last.down_lq >= 99, "link quality 100%% both ways (up %.0f, down %.0f)", W.d.last.up_lq, W.g.last.down_lq);

  printf("both ways at once, 115200 baud, 1 byte in 2000 damaged and 1 in 2000 lost\n");
  world_init(&W, 115200, 0, 5e-4, 5e-4); run(&W, 10);
  CHECK(W.d.rc >= 10 * W.G.C.up_hz * 0.9, "channels: %d frames in 10 s", W.d.rc);
  CHECK(cmds_in_order(&W.d, W.cmds), "every command once, in order: %d of %d", W.d.ncmd, W.cmds);
  CHECK(W.d.last.up_lq >= 85 && W.d.last.up_lq < 100, "the drone's link quality shows the losses: %.0f%% (frames that didn't decode %u, failed the signature %u)", W.d.last.up_lq, W.rd.bad, W.D.N.bad);

  printf("both ways at once, 19200 baud (the slowest)\n");
  world_init(&W, 19200, 0, 0, 0); run(&W, 10);
  CHECK(W.d.rc >= 10 * W.G.C.up_hz * 0.97 && cmds_in_order(&W.d, W.cmds), "channels %d in 10 s (%.1f a second), commands %d of %d", W.d.rc, W.G.C.up_hz, W.d.ncmd, W.cmds);
  CHECK(W.D.N.uq_dropped == 0 && W.g.att * 10 >= W.tlm_bytes * 0.95, "telemetry at its room (%.0f B/s) all comes down: %d of %ld frames", W.room, W.g.att, W.tlm_bytes / 10);

  for (int b = 0; b < 2; b++) {
    int baud = b ? 115200 : 38400;
    printf("one way at a time, %d baud\n", baud);
    world_init(&W, baud, 1, 0, 0); run(&W, 10);
    CHECK(W.L.collisions == 0, "the two ends never talk at once (the drone answers each packet; the ground's beat leaves room): %u collisions", W.L.collisions);
    CHECK(W.D.N.sent >= W.G.N.sent - 2 && W.D.N.sent <= W.G.N.sent, "an answer to each packet: %u up, %u answers", W.G.N.sent, W.D.N.sent);
    CHECK(W.d.rc >= 10 * W.G.C.up_hz * 0.97 && cmds_in_order(&W.d, W.cmds), "channels %d in 10 s (%.1f a second), commands %d of %d", W.d.rc, W.G.C.up_hz, W.d.ncmd, W.cmds);
    CHECK(W.D.N.uq_dropped == 0 && W.g.att * 10 >= W.tlm_bytes * 0.95, "telemetry at its room (%.0f B/s) all comes down: %d of %ld frames", W.room, W.g.att, W.tlm_bytes / 10);
    CHECK(W.d.last.up_lq >= 99 && W.g.last.down_lq >= 99, "link quality 100%% both ways (up %.0f, down %.0f)", W.d.last.up_lq, W.g.last.down_lq);
  }
  printf("one way at a time, 57600 baud, through a radio modem that takes 10 ms more each way\n");
  world_init(&W, 57600, 1, 0, 0); W.L.delay = 0.010; run(&W, 10);
  CHECK(W.L.collisions == 0 && cmds_in_order(&W.d, W.cmds), "no collisions (%u): the ground waits for each answer (%.0f ms), its beat stretched by the delay (%u packets up in 10 s, %.0f a second without it); commands %d of %d", W.L.collisions, W.G.rtt * 1000, W.G.N.sent, W.G.C.up_hz, W.d.ncmd, W.cmds);
  CHECK(W.d.rc >= 10 * 15 && W.d.last.up_lq >= 99, "the channels still come %.0f times a second, uplink LQ %.0f%%; of the telemetry written for the undelayed line, the newest comes (%d of %ld frames; %u bytes of older ones dropped)", W.d.rc / 10.0, W.d.last.up_lq, W.g.att, W.tlm_bytes / 10, W.D.N.uq_dropped);
  printf("one way at a time, 57600 baud, 1 byte in 1000 damaged\n");
  world_init(&W, 57600, 1, 1e-3, 0); run(&W, 10);
  CHECK(W.L.collisions == 0 && cmds_in_order(&W.d, W.cmds) && W.d.rc >= 10 * W.G.C.up_hz * 0.85, "no collisions (a lost packet goes unanswered, and the beat goes on); channels %d, commands %d of %d", W.d.rc, W.d.ncmd, W.cmds);

  printf("the line cut for 2 s, then back\n");
  world_init(&W, 115200, 0, 0, 0); W.hold = 1; run(&W, 2);
  W.L.drop = 1; run(&W, 2);
  float lq_cut = W.g.last.up_lq;
  W.L.drop = 0; int rc0 = W.d.rc; run(&W, 1);
  CHECK(lq_cut == 0 && W.d.rc - rc0 >= 0.9 * W.G.C.up_hz && cmds_in_order(&W.d, W.cmds), "the command module sees uplink LQ %.0f while cut; after, the channels flow again (%d in 1 s) and the commands held meanwhile arrive in order (%d of %d)", lq_cut, W.d.rc - rc0, W.d.ncmd, W.cmds);
  printf("the line cut for 3 s with commands written all along (more than the 16 a packet layer holds)\n");
  world_init(&W, 115200, 0, 0, 0); run(&W, 1); W.L.drop = 1; run(&W, 3); W.L.drop = 0; run(&W, 1);
  { int ok = W.d.ncmd > 10 && W.d.cmd_seq[W.d.ncmd - 1] == (W.cmds - 1) % 255 + 1, inc = 1;
    for (int i = 1; i < W.d.ncmd; i++) if (W.d.cmd_seq[i] <= W.d.cmd_seq[i - 1]) inc = 0;
    CHECK(ok && inc && W.D.N.skipped == (uint32_t)(W.cmds - W.d.ncmd), "the oldest the ground had to drop are skipped (%u), the rest arrive in order, the newest last: %d of %d", W.D.N.skipped, W.d.ncmd, W.cmds); }

  printf(fails ? "%d FAILED\n" : "all passed\n", fails);
  return fails != 0;
}
