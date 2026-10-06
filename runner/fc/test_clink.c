/* Tests for the compact packet layer (clink.c) over a simulated nRF24L01 link: the ground transmits on a fixed beat,
 * the radio retries up to 3 times until acknowledged, the drone's answer rides in the acknowledgement of the
 * ground's next packet; both ends hop channels; each transmission and each acknowledgement may be lost.
 *   cc -O2 -I.. -o test_clink test_clink.c clink.c plink.c radio_link.c crsf.c tlm_crsf.c tlm_core.c rc_core.c pickup_core.c -lm */
#include "clink.h"
#include "tlm_crsf.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)
static uint32_t seed = 88172645u;
static double rnd(void) { seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; return seed / 4294967296.0; }

typedef struct { crsf_parser P; int rc, att, stats, bad; int cmd_seq[4096], ncmd; char texts[256][64]; int ntext; crsf_link last; float ch0; } reader;
static void read_stack(reader *R, clink *L, double t) {
  uint8_t b[PLINK_OUT]; int n = clink_to_stack(L, t, b, sizeof b);
  for (int i = 0; i < n; i++) {
    int len = crsf_feed(&R->P, b[i]); if (len < 0) R->bad++; if (len <= 0) continue;
    const uint8_t *f = R->P.buf, *p = f + 3;
    if (f[2] == CRSF_RC) { float ch[16]; crsf_rc_read(p, ch); R->ch0 = ch[0]; R->rc++; }
    else if (f[2] == CRSF_LINK_STATS) { R->stats++; crsf_link_stats_read(p, f[1] - 2, &R->last); }
    else if (f[2] == CRSF_ATTITUDE) R->att++;
    else if (f[2] == CRSF_EXT && p[0] == CRSF_EXT_CMD) { if (R->ncmd < 4096) R->cmd_seq[R->ncmd++] = p[2]; }
    else if (f[2] == CRSF_EXT && p[0] == CRSF_EXT_TEXT && R->ntext < 256) { int k = 0; while (k < 63 && 2 + k < f[1] - 2 && p[2 + k]) { R->texts[R->ntext][k] = (char)p[2 + k]; k++; } R->texts[R->ntext][k] = 0; R->ntext++; }
  }
}

typedef struct {
  clink G, D; reader g, d; double t;
  double loss_up, loss_ack; int cut;            /* the chances; everything lost */
  uint8_t ackq[CLINK_MTU]; int ackq_n;          /* the drone's answer, loaded for the next acknowledgement */
  int cmds, texts; long tlm_bytes; float room;
  uint32_t mismatched, attempts;
  clink *Gp;                                    /* (the ground in use: a replayer may stand in) */
} world;
static void world_init(world *W, const char *gp, const char *dp, double loss) {
  memset(W, 0, sizeof *W);
  clink_cfg cg, cd; clink_cfg_default(&cg, PLINK_GROUND); clink_cfg_default(&cd, PLINK_DRONE);
  plink_key(gp, &cg.k0, &cg.k1); plink_key(dp, &cd.k0, &cd.k1);
  clink_init(&W->G, &cg, 0x1111); clink_init(&W->D, &cd, 0x2222);
  W->loss_up = W->loss_ack = loss; W->room = 100 * 21 * 0.6f; W->Gp = &W->G;
}
/* one ground beat's exchange (the radio's part), as an nRF24L01 in Enhanced ShockBurst with ACK payloads */
static uint8_t rec[20000][CLINK_MTU]; static int rec_n[20000], nrec;   /* (a recording of the ground's packets) */
static void exchange(world *W, double t) {
  clink *G = W->Gp;
  int ch = clink_channel(G, t);
  uint8_t p[CLINK_MTU]; int n = clink_to_air(G, t, p, sizeof p); if (!n) return;
  if (nrec < 20000 && G == &W->G) { memcpy(rec[nrec], p, (size_t)n); rec_n[nrec++] = n; }
  int got = 0;
  for (int a = 0; a < 4; a++) {                               /* the first try and 3 retries */
    W->attempts++;
    if (W->cut || rnd() < W->loss_up) continue;
    if (clink_channel(&W->D, t) != ch) { W->mismatched++; continue; }
    if (!got) { clink_from_air(&W->D, p, n, 0, t); got = 1; }   /* (a retry of one taken: the radio drops it, acknowledges again) */
    if (rnd() < W->loss_ack) continue;
    if (W->ackq_n) { clink_from_air(G, W->ackq, W->ackq_n, 0, t + 0.0005); W->ackq_n = 0; }
    break;
  }
}
static void drone_loop(world *W, double t) {                  /* the drone's radio task: an answer loaded once the last went */
  if (!W->ackq_n) W->ackq_n = clink_to_air(&W->D, t, W->ackq, sizeof W->ackq);
}
static void run(world *W, double secs, int with_cmds, int with_text) {
  double due = 0; int steps = (int)lround(secs * 1000);
  for (int k = 0; k < steps; k++) {
    double t = W->t;
    if (k % 4 == 0) { uint8_t b[64]; float ch[16] = { 0.4f }; int n = crsf_rc(b, CRSF_ADDR_FC, ch); clink_from_stack(&W->G, b, n, t); }
    if (with_cmds && k % 100 == 50) { uint8_t b[64]; float v[2] = { 1, 1 }; W->cmds++; int n = tlm_crsf_cmd(b, RC_CMD_LATCH, (W->cmds - 1) % 255 + 1, v, 2); clink_from_stack(&W->G, b, n, t); }
    if (with_text && k % 700 == 300) { uint8_t b[64]; char s[80]; snprintf(s, sizeof s, "message %d: the quick brown fox jumps over the lazy dog", ++W->texts); int n = crsf_text(b, 4, s); clink_from_stack(&W->D, b, n, t); }
    due += W->room * 0.001 * (W->d.last.up_lq > 0 ? W->d.last.up_lq / 100 : 1);
    while (due >= 10) { uint8_t b[16]; int n = crsf_attitude(b, 0.1f, 0.2f, 0.3f); clink_from_stack(&W->D, b, n, t); due -= n; W->tlm_bytes += n; }
    exchange(W, t);
    if (k % 2 == 0) { drone_loop(W, t); read_stack(&W->d, &W->D, t); read_stack(&W->g, W->Gp, t); }
    W->t += 0.001;
  }
}
static int cmds_in_order(const reader *R, int want) { if (R->ncmd < want - 2 || R->ncmd > want) return 0; for (int i = 0; i < R->ncmd; i++) if (R->cmd_seq[i] != (i % 255) + 1) return 0; return 1; }
static int texts_in_order(const reader *R, int want) {
  if (R->ntext < want - 1 || R->ntext > want) return 0;
  for (int i = 0; i < R->ntext; i++) { char s[64]; snprintf(s, sizeof s, "message %d: the quick brown fox jumps over the lazy dog", i + 1); if (strncmp(R->texts[i], s, 54)) return 0; }
  return 1;
}

int main(void) {
  static world W;
  printf("the packets\n");
  {
    clink_cfg c; clink_cfg_default(&c, PLINK_GROUND); plink_key("phrase", &c.k0, &c.k1);
    clink L; clink_init(&L, &c, 7); int ok = 1;
    for (int i = 0; i < CLINK_HOPS; i++) { if (L.hop[i] < 2 || L.hop[i] > 81) ok = 0; for (int j = 0; j < i; j++) if (abs(L.hop[i] - L.hop[j]) < 3) ok = 0; }
    uint8_t a[5], b5[5]; clink_address(&c, a); plink_key("other", &c.k0, &c.k1); clink_address(&c, b5);
    for (int i = 0; i < 5; i++) if (a[i] == 0 || a[i] == 0xFF || a[i] == 0x55 || a[i] == 0xAA) ok = 0;
    CHECK(ok && memcmp(a, b5, 5), "8 channels from the phrase, all within 2–81 and 3 apart (%d %d %d %d %d %d %d %d); a 5-byte address of its own, unlike another phrase's",
          L.hop[0], L.hop[1], L.hop[2], L.hop[3], L.hop[4], L.hop[5], L.hop[6], L.hop[7]);
  }

  printf("a clean link, 100 packets a second\n");
  world_init(&W, "phrase", "phrase", 0); run(&W, 5, 1, 1);
  CHECK(W.d.rc >= 5 * 100 * 0.85 && fabsf(W.d.ch0 - 0.4f) < 0.01f, "the drone gets the channels: %d in 5 s (a stream packet in place of some while commands go), stick %.2f", W.d.rc, W.d.ch0);
  CHECK(cmds_in_order(&W.d, W.cmds), "every command once, in order: %d of %d", W.d.ncmd, W.cmds);
  CHECK(W.g.att * 10 >= W.tlm_bytes * 0.95 && W.D.N.dropped == 0 && texts_in_order(&W.g, W.texts), "the telemetry at its room (%.0f B/s) comes down: %d of %ld frames; every message whole and in order (%d of %d, 62 bytes each)", W.room, W.g.att, W.tlm_bytes / 10, W.g.ntext, W.texts);
  CHECK(W.d.last.up_lq >= 99 && W.g.last.down_lq >= 99 && W.mismatched == 0, "link quality 100%% both ways; the two ends always on the same channel");
  CHECK(W.d.bad == 0 && W.g.bad == 0, "no broken frames reach either stack");

  printf("20%% of transmissions and 20%% of acknowledgements lost\n");
  world_init(&W, "phrase", "phrase", 0.2); run(&W, 15, 1, 1);
  CHECK(W.d.rc >= 15 * 100 * 0.75, "channels: %d in 15 s (the radio's retries recover most)", W.d.rc);
  CHECK(cmds_in_order(&W.d, W.cmds), "every command once, in order: %d of %d (the stream sends again what wasn't acknowledged: %u times from the ground)", W.d.ncmd, W.cmds, W.G.N.resent);
  CHECK(texts_in_order(&W.g, W.texts) && W.d.bad == 0 && W.g.bad == 0, "every message whole and in order (%d of %d); no broken frames", W.g.ntext, W.texts);
  CHECK(W.g.att * 10 >= W.tlm_bytes * 0.9, "the telemetry, written at the room the drone's link quality leaves, comes down: %d of %ld frames", W.g.att, W.tlm_bytes / 10);

  printf("another binding phrase at one end\n");
  world_init(&W, "phrase", "not the same", 0); run(&W, 3, 1, 0);
  CHECK(W.d.rc == 0 && W.d.ncmd == 0 && W.g.att == 0 && W.g.last.up_lq == 0, "nothing taken either way (the other phrase's channels and address: it mostly isn't even heard; %u rejected); the command module says uplink %.0f%%", W.D.N.bad, W.g.last.up_lq);

  printf("the ground starts again (a new session), the drone flying on\n");
  world_init(&W, "phrase", "phrase", 0); run(&W, 2, 1, 0);
  { clink_cfg cg = W.G.C; clink_init(&W.G, &cg, 0x3333); W.d.ncmd = 0; W.cmds = 0; int rc0 = W.d.rc; double t0 = W.t;
    run(&W, 2, 1, 0);
    CHECK(W.d.rc - rc0 > 120 && cmds_in_order(&W.d, W.cmds), "the drone takes the new one once the old is quiet half a second: %d channel frames in 2 s; the new start's commands in order from 1 (%d of %d)", W.d.rc - rc0, W.d.ncmd, W.cmds);
    (void)t0; }

  printf("the drone starts again\n");
  world_init(&W, "phrase", "phrase", 0); run(&W, 2, 0, 0);
  { clink_cfg cd = W.D.C; clink_init(&W.D, &cd, 0x4444); W.ackq_n = 0; int rc0 = W.d.rc; run(&W, 2, 0, 0);
    CHECK(W.d.rc - rc0 > 120 && W.g.last.up_lq >= 99, "the ground notices the silence, says hello again, and the channels flow (%d in 2 s); uplink LQ %.0f%%", W.d.rc - rc0, W.g.last.up_lq); }

  printf("a recording of the ground replayed while it is off\n");
  world_init(&W, "phrase", "phrase", 0); nrec = 0; run(&W, 3, 1, 0);
  { int got = nrec; static clink R; clink_cfg cg = W.G.C; clink_init(&R, &cg, 0x5555);
    int rc0 = W.d.rc, cmd0 = W.d.ncmd;
    W.cut = 1; run(&W, 1, 0, 0); W.cut = 0;                     /* (the ground goes quiet) */
    for (int i = 0; i < got; i++) {                            /* the attacker plays it all back, on every channel */
      double t = W.t; clink_from_air(&W.D, rec[i], rec_n[i], 0, t);
      W.ackq_n = clink_to_air(&W.D, t, W.ackq, sizeof W.ackq);
      W.ackq_n = 0;
      read_stack(&W.d, &W.D, t); W.t += 0.01;
    }
    CHECK(W.d.rc == rc0 && W.d.ncmd == cmd0, "the drone takes none of it: no channels, no commands (%d packets replayed; %u failed the tag, %u old numbers)", got, W.D.N.bad, W.D.N.replays); }

  printf("hopping: the link cut for 2 s, then back\n");
  world_init(&W, "phrase", "phrase", 0); run(&W, 2, 0, 0);
  W.cut = 1; run(&W, 2, 0, 0); W.cut = 0;
  { int rc0 = W.d.rc; run(&W, 0.3, 0, 0);
    CHECK(W.d.rc - rc0 >= 15, "the drone, waiting on the first of the 8 channels, finds the ground within a round of them: %d channel frames in the first 0.3 s", W.d.rc - rc0); }
  { int rc0 = W.d.rc; uint32_t mm = W.mismatched; run(&W, 2, 0, 0);
    CHECK(W.d.rc - rc0 >= 195 && W.mismatched == mm, "then follows it channel by channel: %d in 2 s, never on the wrong one", W.d.rc - rc0); }

  printf("50 packets a second (250 kbit/s), 10%% lost\n");
  world_init(&W, "phrase", "phrase", 0.1); W.G.C.up_hz = W.D.C.up_hz = 50; W.room = 50 * 21 * 0.6f; run(&W, 10, 1, 1);
  CHECK(W.d.rc >= 10 * 50 * 0.75 && cmds_in_order(&W.d, W.cmds) && texts_in_order(&W.g, W.texts), "channels %d in 10 s, commands %d of %d, messages %d of %d", W.d.rc, W.d.ncmd, W.cmds, W.g.ntext, W.texts);

  printf("the settings\n");
  { rlink_cfg L; char err[200], d[32]; int ok = 1;
    for (int r = 0; r < 3; r++) { const char *in[] = { "nrf24,250", "nrf24,1000", "nrf24,2000" }; if (rlink_parse(&L, in[r], err, sizeof err)) ok = 0; rlink_describe(&L, d, sizeof d); if (strcmp(d, in[r]) || !rlink_compact(&L) || rlink_packets(&L)) ok = 0; }
    CHECK(ok && rlink_parse(&L, "nrf24,500", err, sizeof err) && rlink_parse(&L, "nrf24", err, sizeof err), "nrf24,250 / 1000 / 2000 read back as written; others refused: %s", err);
    CHECK(!rlink_parse(&L, "ble", err, sizeof err) && L.kind == RLINK_BLE && rlink_packets(&L) && !rlink_compact(&L) && (rlink_describe(&L, d, sizeof d), !strcmp(d, "ble")) && rlink_parse(&L, "ble,2m", err, sizeof err),
          "ble reads back as written (a packet link, plink's packets); ble with settings refused: %s", err);
    rlink_parse(&L, "nrf24,250", err, sizeof err); clink_cfg c; clink_cfg_default(&c, PLINK_GROUND); clink_cfg_link(&c, &L);
    CHECK(c.up_hz == 50 && rlink_budget(&L) > 600 && rlink_budget(&L) < 700, "at 250 kbit/s: 50 packets a second, the telemetry %.0f B/s (ExpressLRS at 250 Hz 1:4: about 280)", rlink_budget(&L)); }

  printf(fails ? "%d FAILED\n" : "all passed\n", fails);
  return fails != 0;
}
