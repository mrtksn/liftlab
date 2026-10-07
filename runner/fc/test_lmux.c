/* Tests for two links at once (lmux.c) and one-way links (plink.c's one-way mode): two pairs of packet layers over
 * simulated air, each end's stack behind a merger, as radio_mux.c puts them together (and ties them as it does).
 *   cc -O2 -I.. -o test_lmux test_lmux.c lmux.c plink.c radio_link.c crsf.c tlm_crsf.c tlm_core.c rc_core.c pickup_core.c -lm && ./test_lmux */
#include "lmux.h"
#include "plink.h"
#include "tlm_crsf.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

/* the air one way: packets in flight with a delay, lost with probability loss, none while cut */
typedef struct { uint8_t p[PLINK_MTU]; int n; double at; } flying;
typedef struct { flying q[512]; int n; double loss, delay; uint32_t seed; int cut; } air;
static double rnd(air *A) { A->seed ^= A->seed << 13; A->seed ^= A->seed >> 17; A->seed ^= A->seed << 5; return A->seed / 4294967296.0; }
static void air_send(air *A, const uint8_t *p, int n, double t) {
  if (A->cut || rnd(A) < A->loss || A->n == 512) return;
  flying *f = &A->q[A->n++]; memcpy(f->p, p, (size_t)n); f->n = n; f->at = t + A->delay;
}
/* (a recording: every packet that went, cut or not, while rec is set) */
typedef struct { uint8_t p[PLINK_MTU]; int n; } taped;
static taped tape[2][600]; static int tape_n[2];

/* what a stack reads: the frames by type */
typedef struct { crsf_parser P; int rc, att, stats; int cmds[256]; int texts[256]; double t_rc, gap_rc, t_att, gap_att; crsf_link last; int bad_sent; } reader;
static void read_bytes(reader *R, const uint8_t *b, int n, double t, int watch) {
  for (int i = 0; i < n; i++) {
    int len = crsf_feed(&R->P, b[i]); if (len <= 0) continue;
    const uint8_t *f = R->P.buf, *p = f + 3;
    if (f[2] == CRSF_RC) { if (watch && R->rc && t - R->t_rc > R->gap_rc) R->gap_rc = t - R->t_rc; R->t_rc = t; R->rc++; }
    else if (f[2] == CRSF_ATTITUDE) { if (watch && R->att && t - R->t_att > R->gap_att) R->gap_att = t - R->t_att; R->t_att = t; R->att++; }
    else if (f[2] == CRSF_LINK_STATS) { R->stats++; crsf_link_stats_read(p, f[1] - 2, &R->last); }
    else if (f[2] == CRSF_EXT && p[0] == CRSF_EXT_CMD) R->cmds[p[2]]++;
    else if (f[2] == CRSF_EXT && p[0] == CRSF_EXT_TEXT) { int k = atoi((const char *)p + 2 + 8); if (k >= 0 && k < 256) R->texts[k]++; }
  }
}

/* two links, each a pair of packet layers and its air both ways; a merger at each end */
typedef struct {
  plink G[2], D[2]; air up[2], down[2]; rlink_cfg L[2]; lmux MG, MD; reader g, d; double t;
  int cmds, texts, rec, ground_off, drone_off, watch; int fol_d[3], fol_g[3]; uint32_t gses, dses;
} world;
static void ends(world *W, int ground, uint32_t ses) {
  for (int i = 0; i < 2; i++) {
    plink_cfg C; plink_cfg_default(&C, ground ? PLINK_GROUND : PLINK_DRONE); plink_cfg_link(&C, &W->L[i]); plink_key("two links", &C.k0, &C.k1);
    plink_init(ground ? &W->G[i] : &W->D[i], &C, ses);               /* (one session number for both links at each end, as radio_mux's programs) */
  }
  int up[2] = { rlink_up(&W->L[0]), rlink_up(&W->L[1]) }, down[2] = { rlink_down(&W->L[0]), rlink_down(&W->L[1]) };
  lmux_init(ground ? &W->MG : &W->MD, ground ? LMUX_GROUND : LMUX_DRONE, 2, up, down);
  memset(ground ? &W->g : &W->d, 0, sizeof W->g);
}
static void world_init(world *W, const rlink_cfg *a, const rlink_cfg *b) {
  memset(W, 0, sizeof *W); W->L[0] = *a; W->L[1] = *b; W->gses = 0x1111; W->dses = 0x2222;
  ends(W, 1, W->gses); ends(W, 0, W->dses);
  for (int i = 0; i < 2; i++) { W->up[i] = (air){ .loss = 0.02, .delay = i ? 0.012 : 0.002, .seed = 77u + i }; W->down[i] = (air){ .loss = 0.02, .delay = i ? 0.012 : 0.002, .seed = 991u + i }; }
}
/* as radio_mux.c's tie(): what each end hears over both links, said on each; a one-way link tied to the two-way one */
static void tie(lmux *M, plink *P, const rlink_cfg *L, double t) {
  int up, down, rssi; lmux_lq(M, t, &up, &down, &rssi);
  for (int i = 0; i < 2; i++) {
    plink_hear(&P[i], M->role == LMUX_DRONE ? up : down, rssi);
    int j = 1 - i;
    if (L[i].dir != RLINK_BOTH && L[j].dir == RLINK_BOTH) plink_tie(&P[i], P[j].known);
  }
}
static void step(world *W) {
  double t = W->t; uint8_t b[PLINK_OUT], pk[PLINK_MTU];
  int ms = (int)lround(t * 1000);
  if (!W->ground_off) {
    tie(&W->MG, W->G, W->L, t);
    if (ms % 4 == 0) { float ch[16] = { 0.5f }; int n = crsf_rc(b, CRSF_ADDR_FC, ch); for (int i = 0; i < 2; i++) if (rlink_up(&W->L[i])) plink_from_stack(&W->G[i], b, n, t); }
    if (ms % 300 == 150 && W->cmds < 250) { float v[2] = { 1, 1 }; ++W->cmds; int n = tlm_crsf_cmd(b, RC_CMD_LATCH, W->cmds, v, 2); for (int i = 0; i < 2; i++) if (rlink_up(&W->L[i])) plink_from_stack(&W->G[i], b, n, t); }
    for (int i = 0; i < 2; i++) { int n = plink_to_air(&W->G[i], t, pk, sizeof pk); if (n) { air_send(&W->up[i], pk, n, t); if (W->rec && tape_n[i] < 600) { memcpy(tape[i][tape_n[i]].p, pk, (size_t)n); tape[i][tape_n[i]++].n = n; } } }
  }
  if (!W->drone_off) {
    tie(&W->MD, W->D, W->L, t);
    if (ms % 20 == 0) { int n = crsf_attitude(b, 0.1f, 0.2f, 0.3f); for (int i = 0; i < 2; i++) if (rlink_down(&W->L[i])) plink_from_stack(&W->D[i], b, n, t); }
    if (ms % 700 == 350 && W->texts < 250) { char s[32]; snprintf(s, sizeof s, "message %d", ++W->texts); int n = crsf_text(b, 4, s); for (int i = 0; i < 2; i++) if (rlink_down(&W->L[i])) plink_from_stack(&W->D[i], b, n, t); }
    for (int i = 0; i < 2; i++) { int n = plink_to_air(&W->D[i], t, pk, sizeof pk); if (n) air_send(&W->down[i], pk, n, t); }
  }
  for (int i = 0; i < 2; i++) {                                      /* the air delivers */
    for (int k = 0; k < W->up[i].n;) { flying *f = &W->up[i].q[k]; if (f->at <= t) { if (!W->drone_off) plink_from_air(&W->D[i], f->p, f->n, -60, t); *f = W->up[i].q[--W->up[i].n]; } else k++; }
    for (int k = 0; k < W->down[i].n;) { flying *f = &W->down[i].q[k]; if (f->at <= t) { if (!W->ground_off) plink_from_air(&W->G[i], f->p, f->n, -70, t); *f = W->down[i].q[--W->down[i].n]; } else k++; }
  }
  if (!W->drone_off) { for (int i = 0; i < 2; i++) { int n = plink_to_stack(&W->D[i], t, b, sizeof b); lmux_from_link(&W->MD, i, b, n, t); } int n = lmux_to_stack(&W->MD, t, b, sizeof b); read_bytes(&W->d, b, n, t, W->watch); }
  if (!W->ground_off) { for (int i = 0; i < 2; i++) { int n = plink_to_stack(&W->G[i], t, b, sizeof b); lmux_from_link(&W->MG, i, b, n, t); } int n = lmux_to_stack(&W->MG, t, b, sizeof b); read_bytes(&W->g, b, n, t, W->watch); }
  W->t += 0.001;
}
static void run(world *W, double s) { int n = (int)lround(s * 1000); for (int k = 0; k < n; k++) step(W); }
static int once(const int *c, int n) { for (int i = 1; i <= n; i++) if (c[i] != 1) return 0; return 1; }
static int count(const int *c, int n) { int k = 0; for (int i = 1; i <= n; i++) k += c[i] > 0; return k; }

int main(void) {
  static world W; rlink_cfg A, B; char err[120];
  rlink_parse(&A, "espnow,6", err, sizeof err);

  printf("two links both ways: the first cut for 5 s, then back\n");
  rlink_parse(&B, "serial,115200", err, sizeof err);
  world_init(&W, &A, &B);
  run(&W, 1); W.watch = 1;
  run(&W, 3); int f0 = lmux_followed(&W.MD, W.t);
  W.up[0].cut = W.down[0].cut = 1; run(&W, 2.5); int f1 = lmux_followed(&W.MD, W.t), g1 = lmux_followed(&W.MG, W.t); float lq_cut = W.d.last.up_lq, glq_cut = W.g.last.down_lq;
  run(&W, 2.5); W.up[0].cut = W.down[0].cut = 0; run(&W, 3); int f2 = lmux_followed(&W.MD, W.t);
  CHECK(f0 == 0 && f1 == 1 && g1 == 1 && f2 == 0, "the drone follows the first link, the second while it's cut, the first again (%d %d %d; the ground %d)", f0, f1, f2, g1);
  CHECK(W.d.gap_rc < 0.13, "the channels never stop: the longest gap %.0f ms (the cut: none)", W.d.gap_rc * 1000);
  CHECK(W.g.gap_att < 0.6, "nor the telemetry: the longest gap %.0f ms", W.g.gap_att * 1000);
  CHECK(once(W.d.cmds, W.cmds) && once(W.g.texts, W.texts), "every command (%d) and message (%d) once, though both links carried them (copies dropped: drone %u, ground %u)", W.cmds, W.texts, (unsigned)W.MD.dups, (unsigned)W.MG.dups);
  CHECK(lq_cut > 80 && glq_cut > 80, "the link statistics show the link as a whole during the cut: uplink %.0f%%, downlink %.0f%%", lq_cut, glq_cut);
  CHECK(W.MD.switches >= 2, "the drone switched %u times", (unsigned)W.MD.switches);

  printf("a radio both ways and a laser up only: the radio's uplink cut\n");
  rlink_parse(&B, "serial,115200,up", err, sizeof err);
  CHECK(B.dir == RLINK_UP && rlink_budget(&B) == 0, "serial,115200,up: one way, no telemetry room");
  world_init(&W, &A, &B);
  run(&W, 1); W.watch = 1; int b_before = (int)W.D[1].N.stale_sessions;
  run(&W, 2); W.up[0].cut = 1; run(&W, 5); float glq = W.g.last.up_lq; int f_d = lmux_followed(&W.MD, W.t); W.up[0].cut = 0; run(&W, 2);
  CHECK(W.MD.k[1].frames > 100, "the laser's packets are taken once the radio has the ground confirmed (refused before: %d; frames by it: %u)", b_before, (unsigned)W.MD.k[1].frames);
  CHECK(f_d == 1 && W.d.gap_rc < 0.13, "with the radio's uplink cut the channels come by the laser (followed %d), the longest gap %.0f ms", f_d, W.d.gap_rc * 1000);
  CHECK(once(W.d.cmds, W.cmds), "every command once (%d): by the laser, each sent three times, taken once", W.cmds);
  CHECK(glq > 80, "the command module sees the drone hearing it (uplink %.0f%%): the drone says so over the radio's downlink", glq);
  CHECK(W.G[1].N.sent > 500 && W.D[1].N.sent == 0, "the laser's ground end only sends (%u), its drone end never (%u)", (unsigned)W.G[1].N.sent, (unsigned)W.D[1].N.sent);

  printf("a recording played to a restarted drone\n");
  world_init(&W, &A, &B); W.rec = 1; run(&W, 0.6); W.rec = 0; run(&W, 0.5);
  W.ground_off = 1; W.dses = 0x3333; ends(&W, 0, W.dses); W.up[0].n = W.up[1].n = W.down[0].n = W.down[1].n = 0;   /* the ground gone; the drone restarted */
  for (int k = 0; k < tape_n[0] || k < tape_n[1]; k++) {             /* the recording, both links, at its own pace */
    for (int i = 0; i < 2; i++) if (k < tape_n[i]) air_send(&W.up[i], tape[i][k].p, tape[i][k].n, W.t);
    for (int s = 0; s < 4; s++) step(&W);
  }
  CHECK(tape_n[0] > 40 && tape_n[1] > 40 && W.d.rc == 0 && count(W.d.cmds, 255) == 0, "nothing of it reaches the drone's stack: channels %d, commands %d (the radio's packets don't name its new session; the laser listens only to the sender the radio knows)", W.d.rc, count(W.d.cmds, 255));
  W.ground_off = 0; W.gses = 0x4444; ends(&W, 1, W.gses); run(&W, 1.5);
  CHECK(W.d.rc > 60 && W.MD.k[1].frames > 50, "a new command module: both links carry again (channels %d, frames by the laser %u)", W.d.rc, (unsigned)W.MD.k[1].frames);

  printf("the laser alone, up only\n");
  rlink_cfg N; rlink_parse(&N, "serial,57600,up", err, sizeof err);
  {
    plink_cfg cg, cd; plink_cfg_default(&cg, PLINK_GROUND); plink_cfg_default(&cd, PLINK_DRONE); plink_cfg_link(&cg, &N); plink_cfg_link(&cd, &N);
    static plink G, D; plink_init(&G, &cg, 7); plink_init(&D, &cd, 8); air a = { .loss = 0.1, .delay = 0.003, .seed = 5 };
    reader r; memset(&r, 0, sizeof r); int nc = 0; uint8_t b[PLINK_OUT], pk[PLINK_MTU];
    for (int ms = 0; ms < 6000; ms++) {
      double t = ms * 0.001;
      if (ms % 4 == 0) { float ch[16] = { 0.2f }; int n = crsf_rc(b, CRSF_ADDR_FC, ch); plink_from_stack(&G, b, n, t); }
      if (ms % 200 == 100) { float v[2] = { 1, 1 }; int n = tlm_crsf_cmd(b, RC_CMD_LATCH, ++nc, v, 2); plink_from_stack(&G, b, n, t); }
      int n = plink_to_air(&G, t, pk, sizeof pk); if (n) air_send(&a, pk, n, t);
      for (int k = 0; k < a.n;) { if (a.q[k].at <= t) { plink_from_air(&D, a.q[k].p, a.q[k].n, 0, t); a.q[k] = a.q[--a.n]; } else k++; }
      n = plink_to_stack(&D, t, b, sizeof b); read_bytes(&r, b, n, t, 0);
      if (plink_to_air(&D, t, pk, sizeof pk)) r.bad_sent = 1;
    }
    CHECK(r.rc > 250 && r.stats > 50 && r.last.up_lq > 80, "the drone gets the channels (%d frames) and link statistics (uplink %.0f%%)", r.rc, r.last.up_lq);
    CHECK(count(r.cmds, nc) == nc && once(r.cmds, nc), "with 10%% of packets lost, every command arrives once (%d of %d): each goes three times", count(r.cmds, nc), nc);
    CHECK(!r.bad_sent, "the drone's end never sends");
  }
  printf("%s\n", fails ? "FAILED" : "all passed");
  return fails ? 1 : 0;
}
