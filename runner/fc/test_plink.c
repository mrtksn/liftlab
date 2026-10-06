/* Tests for the packet link (plink.c): the two ends over a simulated air with loss, delay and reordering.
 *   cc -O2 -I.. -o test_plink test_plink.c plink.c crsf.c tlm_crsf.c rc_core.c -lm && ./test_plink */
#include "plink.h"
#include "tlm_crsf.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

/* the air: packets in flight with a delay, each lost with probability loss */
typedef struct { uint8_t p[PLINK_MTU]; int n; double at; } flying;
typedef struct { flying q[256]; int n; double loss, delay, jitter; uint32_t seed; } air;
static double rnd(air *A) { A->seed ^= A->seed << 13; A->seed ^= A->seed >> 17; A->seed ^= A->seed << 5; return A->seed / 4294967296.0; }
static void air_send(air *A, const uint8_t *p, int n, double t) {
  if (rnd(A) < A->loss || A->n == 256) return;
  flying *f = &A->q[A->n++]; memcpy(f->p, p, (size_t)n); f->n = n; f->at = t + A->delay + rnd(A) * A->jitter;
}
static void air_deliver(air *A, plink *to, double t) {
  for (int i = 0; i < A->n;) {
    if (A->q[i].at <= t) { plink_from_air(to, A->q[i].p, A->q[i].n, -60, t); A->q[i] = A->q[--A->n]; }   /* (order not kept: jitter reorders) */
    else i++;
  }
}

/* what each stack reads: the frames by type */
typedef struct { crsf_parser P; int rc, cmd, text, att, stats, bad; int cmd_seq[512], ncmd; char texts[64][64]; int ntext; crsf_link last; float ch0; } reader;
static void read_stack(reader *R, plink *L, double t) {
  uint8_t b[PLINK_OUT]; int n = plink_to_stack(L, t, b, sizeof b);
  for (int i = 0; i < n; i++) {
    int len = crsf_feed(&R->P, b[i]); if (len < 0) R->bad++; if (len <= 0) continue;
    const uint8_t *f = R->P.buf, *p = f + 3;
    switch (f[2]) {
      case CRSF_RC: { float ch[16]; crsf_rc_read(p, ch); R->ch0 = ch[0]; R->rc++; break; }
      case CRSF_LINK_STATS: R->stats++; crsf_link_stats_read(p, f[1] - 2, &R->last); break;
      case CRSF_ATTITUDE: R->att++; break;
      case CRSF_EXT:
        if (p[0] == CRSF_EXT_CMD) { if (R->ncmd < 512) R->cmd_seq[R->ncmd++] = p[2]; R->cmd++; }
        else if (p[0] == CRSF_EXT_TEXT) { if (R->ntext < 64) { int k = 0; while (k < 63 && p[2 + k] && 2 + k < f[1] - 2) { R->texts[R->ntext][k] = (char)p[2 + k]; k++; } R->texts[R->ntext][k] = 0; R->ntext++; } R->text++; }
        break;
    }
  }
}

typedef struct { plink G, D; air up, down; reader g, d; double t; int cmds_sent, texts_sent; } world;
static void world_init(world *W, double loss, uint32_t seed, const char *gphrase, const char *dphrase) {
  memset(W, 0, sizeof *W);
  plink_cfg cg, cd; plink_cfg_default(&cg, PLINK_GROUND); plink_cfg_default(&cd, PLINK_DRONE);
  plink_key(gphrase, &cg.k0, &cg.k1); plink_key(dphrase, &cd.k0, &cd.k1);
  plink_init(&W->G, &cg, 0x1111); plink_init(&W->D, &cd, 0x2222);
  W->up = (air){ .loss = loss, .delay = 0.002, .jitter = 0.003, .seed = seed }; W->down = (air){ .loss = loss, .delay = 0.002, .jitter = 0.003, .seed = seed * 7 + 1 };
}
/* one millisecond: the ground's stack writes its channels every 4 ms; each end sends what's due, the air delivers */
static void world_step(world *W, float stick) {
  double t = W->t; uint8_t b[64], pk[PLINK_MTU];
  if (((int)lround(t * 1000)) % 4 == 0) { float ch[16] = { stick }; int n = crsf_rc(b, CRSF_ADDR_FC, ch); plink_from_stack(&W->G, b, n, t); }
  int n = plink_to_air(&W->G, t, pk, sizeof pk); if (n) air_send(&W->up, pk, n, t);
  n = plink_to_air(&W->D, t, pk, sizeof pk); if (n) air_send(&W->down, pk, n, t);
  air_deliver(&W->up, &W->D, t); air_deliver(&W->down, &W->G, t);
  read_stack(&W->d, &W->D, t); read_stack(&W->g, &W->G, t);
  W->t += 0.001;
}
static void cmd(world *W) { uint8_t b[64]; float v[2] = { 1, 1 }; ++W->cmds_sent; int n = tlm_crsf_cmd(b, RC_CMD_LATCH, (W->cmds_sent - 1) % 255 + 1, v, 2); plink_from_stack(&W->G, b, n, W->t); }
static void text(world *W) { uint8_t b[64]; char s[40]; snprintf(s, sizeof s, "message %d", ++W->texts_sent); int n = crsf_text(b, 4, s); plink_from_stack(&W->D, b, n, W->t); }
static void tlm(world *W) { uint8_t b[64]; int n = crsf_attitude(b, 0.1f, 0.2f, 0.3f); plink_from_stack(&W->D, b, n, W->t); }
static int in_order(const reader *R, int want) { if (R->ncmd != want) return 0; for (int i = 0; i < want; i++) if (R->cmd_seq[i] != (i % 255) + 1) return 0; return 1; }
static int texts_in_order(const reader *R, int want) { if (R->ntext != want) return 0; for (int i = 0; i < want; i++) { char s[40]; snprintf(s, sizeof s, "message %d", i + 1); if (strcmp(R->texts[i], s)) return 0; } return 1; }

int main(void) {
  static world W;
  printf("a clean link\n");
  world_init(&W, 0, 12345, "phrase", "phrase");
  for (int k = 0; k < 3000; k++) { if (k % 100 == 50) cmd(&W); if (k % 10 == 0) tlm(&W); if (k % 500 == 7) text(&W); world_step(&W, 0.5f); }
  CHECK(W.d.rc > 250 && fabsf(W.d.ch0 - 0.5f) < 0.01f, "the drone gets the channels: %d frames in 3 s (100 packets a second), stick %.2f", W.d.rc, W.d.ch0);
  CHECK(in_order(&W.d, W.cmds_sent), "the drone takes every command once, in order: %d of %d", W.d.ncmd, W.cmds_sent);
  CHECK(W.g.att > 250 && texts_in_order(&W.g, W.texts_sent), "the ground gets the telemetry (%d attitude frames) and every message once, in order (%d of %d)", W.g.att, W.g.ntext, W.texts_sent);
  CHECK(W.d.stats >= 28 && W.d.last.up_lq >= 99 && W.d.last.down_lq >= 99, "the drone's stack gets link statistics like a receiver's: %d, uplink LQ %.0f, downlink LQ %.0f", W.d.stats, W.d.last.up_lq, W.d.last.down_lq);
  CHECK(W.g.stats >= 28 && W.g.last.up_lq >= 99 && W.g.last.down_lq >= 99 && W.g.last.up_rssi == -60, "the command module's gets them like a transmitter module's: uplink LQ %.0f, downlink %.0f, RSSI %.0f", W.g.last.up_lq, W.g.last.down_lq, W.g.last.up_rssi);
  CHECK(W.G.N.bad == 0 && W.D.N.bad == 0 && W.D.N.replays == 0 && W.d.bad == 0 && W.g.bad == 0, "nothing bad, no replays, every frame whole");

  printf("30%% of the packets lost, the rest reordered\n");
  world_init(&W, 0.3, 777, "phrase", "phrase"); W.up.jitter = W.down.jitter = 0.02;
  for (int k = 0; k < 6000; k++) { if (k % 50 == 25) cmd(&W); if (k % 10 == 0) tlm(&W); if (k % 300 == 7) text(&W); world_step(&W, -0.25f); }
  for (int k = 0; k < 500; k++) world_step(&W, -0.25f);
  CHECK(in_order(&W.d, W.cmds_sent), "every command once, in order: %d of %d (%u sent again)", W.d.ncmd, W.cmds_sent, W.G.N.resent);
  CHECK(texts_in_order(&W.g, W.texts_sent), "every message once, in order: %d of %d (%u sent again)", W.g.ntext, W.texts_sent, W.D.N.resent);
  CHECK(fabsf(W.d.last.up_lq - 70) < 12 && fabsf(W.g.last.down_lq - 70) < 15, "link quality: uplink %.0f%%, downlink %.0f%% (70%% got through)", W.d.last.up_lq, W.g.last.down_lq);
  CHECK(W.d.last.down_lq == W.g.last.down_lq || fabsf(W.d.last.down_lq - W.g.last.down_lq) < 6, "each end tells the other what it hears: the drone knows the downlink at %.0f%% (the ground hears %.0f%%)", W.d.last.down_lq, W.g.last.down_lq);

  printf("the wrong binding phrase\n");
  world_init(&W, 0, 99, "phrase", "another phrase");
  for (int k = 0; k < 1000; k++) { if (k == 100) cmd(&W); if (k % 10 == 0) tlm(&W); world_step(&W, 1); }
  CHECK(W.d.rc == 0 && W.d.ncmd == 0 && W.g.att == 0 && W.D.N.bad > 50 && W.G.N.bad > 10, "nothing taken either way (%u and %u packets refused)", W.D.N.bad, W.G.N.bad);
  CHECK(W.d.stats == 0 && W.g.stats >= 9 && W.g.last.up_lq == 0, "the drone's stack hears no receiver; the command module's module says LQ 0");

  printf("replays\n");
  world_init(&W, 0, 5, "phrase", "phrase");
  uint8_t old[PLINK_MTU]; int oldn = 0;
  for (int k = 0; k < 400; k++) {
    if (k == 100) { cmd(&W); }
    world_step(&W, 0);
    if (!oldn && W.up.n) { oldn = W.up.q[0].n; memcpy(old, W.up.q[0].p, (size_t)oldn); }
  }
  int got = W.d.ncmd; uint32_t r0 = W.D.N.replays;
  plink_from_air(&W.D, old, oldn, -60, W.t); for (int k = 0; k < 20; k++) world_step(&W, 0);
  CHECK(W.D.N.replays == r0 + 1 && W.d.ncmd == got, "an old packet again: refused (replays %u)", W.D.N.replays);
  uint8_t forged[PLINK_MTU]; memcpy(forged, old, (size_t)oldn); forged[2] ^= 0x40;   /* a newer number, same tag */
  uint32_t b0 = W.D.N.bad; plink_from_air(&W.D, forged, oldn, -60, W.t);
  CHECK(W.D.N.bad == b0 + 1, "a packet changed in the air: its tag doesn't match, refused");

  printf("the ground restarts\n");
  world_init(&W, 0, 31, "phrase", "phrase");
  for (int k = 0; k < 500; k++) { if (k == 100) cmd(&W); world_step(&W, 0.3f); }
  plink_cfg cg = W.G.C; int c0 = W.d.ncmd;
  plink_init(&W.G, &cg, 0x3333); W.cmds_sent = 0;                  /* a new session; its commands from 1 again */
  int lost_at = -1, back_at = -1;
  for (int k = 0; k < 1500; k++) {
    if (k == 50) cmd(&W);
    int rc0 = W.d.rc; world_step(&W, 0.3f);
    if (W.d.rc == rc0 && lost_at < 0 && k > 5) lost_at = k;
    if (W.d.rc > rc0 && lost_at >= 0 && back_at < 0 && k > lost_at) back_at = k;
  }
  CHECK(W.D.peer == 0x3333 && back_at > 480 && back_at < 520, "the drone takes the new session once the old one has been quiet half a second: channels again after %d ms", back_at);
  CHECK(W.d.ncmd == c0 + 1 && W.d.cmd_seq[W.d.ncmd - 1] == 1, "its first command, numbered 1 again, taken");
  printf("an old session replayed while the new one talks\n");
  plink_cfg cg2 = W.G.C; static plink Old; plink_init(&Old, &cg2, 0x1111); uint8_t pk[PLINK_MTU];
  Old.t_sent = -1; int on = plink_to_air(&Old, W.t, pk, sizeof pk); uint32_t s0 = W.D.N.stale_sessions;
  plink_from_air(&W.D, pk, on, -60, W.t);
  CHECK(W.D.N.stale_sessions == s0 + 1 && W.D.peer == 0x3333, "refused: the session we have is talking");

  printf("a recording of a whole session played back to a drone that started again\n");
  {
    static world R; world_init(&R, 0, 21, "phrase", "phrase");
    static uint8_t rec[600][PLINK_MTU]; static int rec_n[600]; int nr = 0;
    for (int k = 0; k < 3000; k++) {                         /* the real flight: commands and channels, recorded off the air */
      if (k % 100 == 50) cmd(&R);
      double t = R.t; uint8_t b[64], pk[PLINK_MTU];
      if (k % 4 == 0) { float ch[16] = { 0.9f }; int n = crsf_rc(b, CRSF_ADDR_FC, ch); plink_from_stack(&R.G, b, n, t); }
      int n = plink_to_air(&R.G, t, pk, sizeof pk); if (n) { if (nr < 600) { memcpy(rec[nr], pk, (size_t)n); rec_n[nr++] = n; } air_send(&R.up, pk, n, t); }
      n = plink_to_air(&R.D, t, pk, sizeof pk); if (n) air_send(&R.down, pk, n, t);
      air_deliver(&R.up, &R.D, t); air_deliver(&R.down, &R.G, t); read_stack(&R.d, &R.D, t); read_stack(&R.g, &R.G, t); R.t += 0.001;
    }
    plink_cfg cd = R.D.C; plink_init(&R.D, &cd, 0x7777); memset(&R.d, 0, sizeof R.d);   /* the drone starts again; the ground is off */
    for (int i = 0; i < nr; i++) { uint8_t b[PLINK_OUT]; plink_from_air(&R.D, rec[i], rec_n[i], -60, R.t); plink_to_stack(&R.D, R.t, b, sizeof b); R.t += 0.01; }
    read_stack(&R.d, &R.D, R.t);
    CHECK(R.d.rc == 0 && R.d.ncmd == 0 && !plink_connected(&R.D, R.t), "it takes the recording's session but none of its frames (no channels, no commands): its packets name the drone's old session, not this start's (%d played)", nr);
  }

  printf("the link goes quiet\n");
  world_init(&W, 0, 8, "phrase", "phrase");
  for (int k = 0; k < 1000; k++) world_step(&W, 0);
  W.up.loss = W.down.loss = 1;
  int d_stats_after = 0, d0 = 0; float lq_g = 100;
  for (int k = 0; k < 2000; k++) { if (k == 1100) d0 = W.d.stats; world_step(&W, 0); if (k == 500) lq_g = W.g.last.down_lq; }
  d_stats_after = W.d.stats - d0;
  CHECK(lq_g <= 65 && W.g.last.up_lq == 0 && W.g.last.down_lq == 0, "the command module's module: the downlink LQ falls (%.0f%% after 0.5 s; the drone sends at least 20 a second), then 0 both ways", lq_g);
  CHECK(d_stats_after == 0, "the drone's receiver says nothing once it hasn't heard the ground for a second (as an ExpressLRS one in failsafe)");

  printf("the link goes quiet in flight (telemetry flowing: the drone sends 100 packets a second)\n");
  world_init(&W, 0, 9, "phrase", "phrase");
  for (int k = 0; k < 2000; k++) { if (k % 5 == 0) tlm(&W); world_step(&W, 0); }
  W.up.loss = W.down.loss = 1; float lq_half = -1, lq_up_half = -1;
  for (int k = 0; k < 1200; k++) { world_step(&W, 0); if (k == 500) { lq_half = W.g.last.down_lq; lq_up_half = W.d.last.up_lq; } }
  CHECK(lq_half <= 65 && lq_up_half <= 65, "after half a second (the statistics go every 0.1 s): downlink LQ %.0f%%, uplink LQ %.0f%% (by the rates the packets came at)", lq_half, lq_up_half);
  CHECK(W.g.last.down_lq == 0, "after 1.2 s: 0");

  printf("the ground asked on a coarse tick\n");
  { plink_cfg cg; plink_cfg_default(&cg, PLINK_GROUND); static plink G; plink_init(&G, &cg, 7); uint8_t pk[PLINK_MTU], b[64];
    float ch[16] = { 0 }; int sent = 0, n = crsf_rc(b, CRSF_ADDR_FC, ch);
    for (double t = 0; t < 2.0; t += 0.004) { plink_from_stack(&G, b, n, t); if (plink_to_air(&G, t, pk, sizeof pk)) sent++; }
    CHECK(sent >= 199 && sent <= 201, "asked every 4 ms: %d packets in 2 s (a fixed beat of 100 a second, not one every 12 ms)", sent); }
  printf("the command module stops writing channels\n");
  world_init(&W, 0, 21, "phrase", "phrase");
  for (int k = 0; k < 1000; k++) world_step(&W, 0.4f);
  int rc_before = W.d.rc;
  for (int k = 0; k < 1000; k++) {                                   /* its program stopped: no channels from the stack */
    uint8_t pk[PLINK_MTU]; double t = W.t;
    int n = plink_to_air(&W.G, t, pk, sizeof pk); if (n) air_send(&W.up, pk, n, t);
    n = plink_to_air(&W.D, t, pk, sizeof pk); if (n) air_send(&W.down, pk, n, t);
    air_deliver(&W.up, &W.D, t); air_deliver(&W.down, &W.G, t); read_stack(&W.d, &W.D, t); read_stack(&W.g, &W.G, t); W.t += 0.001;
  }
  CHECK(W.d.rc - rc_before >= 20 && W.d.rc - rc_before <= 27 && W.d.last.up_lq > 90, "the drone gets the last channels for %.2f s, then none (the link itself still up: LQ %.0f%%), so its failsafe acts", (W.d.rc - rc_before) / 100.0, W.d.last.up_lq);

  printf("telemetry room\n");
  world_init(&W, 0, 4, "phrase", "phrase");
  for (int k = 0; k < 2000; k++) { for (int j = 0; j < 3; j++) tlm(&W); world_step(&W, 0); }   /* 3 frames a ms: 3000 a second, 36 KB/s */
  CHECK(W.D.N.uq_dropped > 0 && W.g.att > 1500, "more than the link carries: the oldest go (%u dropped), %d frames a second still arrive", W.D.N.uq_dropped, W.g.att / 2);

  printf(fails ? "%d FAILED\n" : "all passed\n", fails);
  return fails != 0;
}
