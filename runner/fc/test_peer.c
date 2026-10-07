/* Tests for drones talking to each other (peer.c): several drones over a simulated air (each packet to everyone in
 * range, or to one, lost with a chance, a little late; links cut and back), as they would fly.
 *   cc -O2 -I.. -o test_peer test_peer.c peer.c plink.c radio_link.c crsf.c tlm_crsf.c tlm_core.c rc_core.c pickup_core.c -lm && ./test_peer */
#include "peer.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

#define DRONES 10
typedef struct { uint8_t p[PEER_MTU]; int n, from, to; double at; } flying;   /* to: a drone, or −1 everyone */
static flying air[4096]; static int air_n;
static peer_net D[DRONES]; static int on[DRONES], nd;
static uint8_t addr[DRONES][6];
static double loss = 0.0, delay = 0.002; static int cut[DRONES][DRONES];
static uint32_t seed = 12345;
static double rnd(void) { seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5; return seed / 4294967296.0; }
static int who(const uint8_t a[6]) { for (int i = 0; i < nd; i++) if (!memcmp(a, addr[i], 6)) return i; return -1; }
static flying tape[2000]; static int tape_n, taping = -1;   /* (a recording of what one drone sent) */
static void start(int i, uint32_t session, const char *phrase) {
  addr[i][0] = 0x24; addr[i][1] = 0x6F; addr[i][2] = 0x28; addr[i][3] = 0x10; addr[i][4] = 0; addr[i][5] = (uint8_t)(i + 1);
  char name[16]; snprintf(name, sizeof name, "drone %c", 'A' + i);
  peer_init(&D[i], peer_id_of(addr[i]), session, name, phrase); on[i] = 1;
}
static void step(double t) {
  for (int i = 0; i < nd; i++) {
    if (!on[i]) continue;
    uint8_t p[PEER_MTU], a[6]; int n;
    while ((n = peer_to_air(&D[i], t, a, p, sizeof p)) > 0) {
      int to = memcmp(a, PEER_BROADCAST, 6) ? who(a) : -1;
      if (taping == i && tape_n < 2000) { memcpy(tape[tape_n].p, p, (size_t)n); tape[tape_n].n = n; tape[tape_n].to = to; tape_n++; }
      if (air_n < 4096) { flying *f = &air[air_n++]; memcpy(f->p, p, (size_t)n); f->n = n; f->from = i; f->to = to; f->at = t + delay; }
    }
  }
  for (int k = 0; k < air_n;) {
    flying *f = &air[k];
    if (f->at > t) { k++; continue; }
    for (int j = 0; j < nd; j++) {
      if (j == f->from || !on[j] || (f->to >= 0 && f->to != j) || cut[f->from][j]) continue;
      int tries = f->to >= 0 ? 4 : 1, heard = 0;                      /* (one to one: the radio tries again; to everyone: once) */
      for (int r = 0; r < tries && !heard; r++) heard = rnd() >= loss;
      if (heard) peer_from_air(&D[j], f->from >= 0 ? addr[f->from] : addr[0], f->p, f->n, -60, t);
    }
    air[k] = air[--air_n];
  }
}
static double T;
static void run(double s) { int n = (int)lround(s * 1000); for (int k = 0; k < n; k++) { step(T); T += 0.001; } }
static int state(int i, int j) { int s = peer_find(&D[i], D[j].id); return s < 0 ? -9 : peer_state(&D[i], s, T); }
static const peer_t *of(int i, int j) { int s = peer_find(&D[i], D[j].id); return s < 0 ? 0 : &D[i].P[s]; }

int main(void) {
  printf("three drones find each other\n");
  nd = 3; for (int i = 0; i < 3; i++) start(i, 1000 + i, "fleet phrase");
  run(1.0);
  int all = 1; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) if (i != j && state(i, j) != PEER_CONNECTED) all = 0;
  CHECK(all, "within a second each sees the other two connected");
  CHECK(of(0, 1) && !strcmp(of(0, 1)->name, "drone B") && of(2, 0) && !strcmp(of(2, 0)->name, "drone A"), "by name: %s, %s", of(0, 1) ? of(0, 1)->name : "-", of(2, 0) ? of(2, 0)->name : "-");
  float v[3] = { 1.5f, -2.25f, 42 }; peer_publish(&D[1], v, 3); run(0.3);
  const peer_t *b = of(0, 1);
  CHECK(b && b->nvals == 3 && b->vals[1] == -2.25f && T - b->t_vals < 0.15, "what B publishes reaches A: %d values, %.2f, %.2f s old", b ? b->nvals : 0, b ? b->vals[1] : 0, b ? T - b->t_vals : 0);
  CHECK(peer_lq(&D[0], peer_find(&D[0], D[1].id), T) >= 95 && b->heard_us >= 95, "link quality both ways: %d%%, %d%%", peer_lq(&D[0], peer_find(&D[0], D[1].id), T), b->heard_us);
  peer_ping(&D[0], D[2].id, T); run(0.1);
  CHECK(of(0, 2)->rtt > 0 && of(0, 2)->rtt < 0.03, "a ping from A to C comes back in %.1f ms", of(0, 2)->rtt * 1000);

  printf("messages through a lossy link\n");
  loss = 0.3; int sent = 0, got = 0, order = 1; int next = 0;
  for (int k = 0; k < 400; k++) {
    if (k % 10 == 0 && sent < 30) { char m[32]; int n = snprintf(m, sizeof m, "message %d", sent); if (!peer_send(&D[0], D[1].id, (uint8_t *)m, n)) sent++; }
    run(0.01);
    uint8_t buf[PEER_MSG]; uint32_t from; int n;
    while ((n = peer_recv(&D[1], &from, buf, sizeof buf)) >= 0) { buf[n] = 0; int x = atoi((char *)buf + 8); if (x != next || from != D[0].id) order = 0; next = x + 1; got++; }
  }
  loss = 0;
  CHECK(sent == 30 && got == 30 && order, "with 30%% of tries lost, all %d of %d messages from A arrive at B once, in order (sent again %u)", got, sent, (unsigned)D[0].N.resent);

  printf("out of range and back\n");
  cut[0][1] = cut[1][0] = 1; run(2.0); int s2 = state(0, 1); float age = T - of(0, 1)->t_vals;
  cut[0][1] = cut[1][0] = 0; run(0.3); int back = state(0, 1);
  CHECK(s2 == PEER_STALE && age > 1.9 && back == PEER_CONNECTED, "2 s apart: stale (its values %.1f s old), then connected again by itself", age);
  cut[0][1] = cut[1][0] = 1; run(5.0); int s5 = state(0, 1), s5c = state(0, 2);
  cut[0][1] = cut[1][0] = 0; run(0.5); int back5 = state(0, 1);
  CHECK(s5 == PEER_LOST && s5c == PEER_CONNECTED && back5 == PEER_CONNECTED, "5 s apart: lost (C still connected), then back by its beacons");
  { int q = 0; for (int k = 0; k < 3; k++) { char m[8] = "queued"; q += !peer_send(&D[0], D[1].id, (uint8_t *)m, 6); }
    cut[0][1] = cut[1][0] = 1; run(1.5); cut[0][1] = cut[1][0] = 0; run(0.5);
    int n = 0; uint8_t buf[PEER_MSG]; uint32_t from; while (peer_recv(&D[1], &from, buf, sizeof buf) >= 0) n++;
    CHECK(q == 3 && n == 3, "messages sent while apart arrive once it's back: %d of %d", n, q); }

  printf("a drone restarts\n");
  taping = 0; tape_n = 0; run(1.0); taping = -1;                     /* (A's packets recorded) */
  start(1, 2001, "fleet phrase"); run(1.0);
  CHECK(state(0, 1) == PEER_CONNECTED && of(0, 1)->session == 2001, "A takes B's new start (session %u) and is connected again", (unsigned)of(0, 1)->session);
  { char m[8] = "after"; peer_send(&D[0], D[1].id, (uint8_t *)m, 5); run(0.2); uint8_t buf[PEER_MSG]; uint32_t from; int n = peer_recv(&D[1], &from, buf, sizeof buf);
    CHECK(n == 5 && !memcmp(buf, "after", 5), "and messages flow to the new start"); }
  /* a recording of A played to B after B restarts, A silent: nothing of it gets through */
  on[0] = 0; run(1.0); start(1, 3001, "fleet phrase"); air_n = 0;
  for (int k = 0; k < tape_n; k++) if (tape[k].to == 1 || tape[k].to == -1) { peer_from_air(&D[1], addr[0], tape[k].p, tape[k].n, 0, T); T += 0.002; }
  { uint8_t buf[PEER_MSG]; uint32_t from; int s = peer_find(&D[1], D[0].id);
    CHECK(peer_recv(&D[1], &from, buf, sizeof buf) < 0 && (s < 0 || (D[1].P[s].nvals == 0 && !D[1].P[s].known)), "a recording of A played to B after B restarts: no messages, no values, never connected (%u replays dropped)", (unsigned)D[1].N.replays); }

  printf("another fleet, and a crowd\n");
  nd = 4; start(3, 4001, "another phrase"); on[0] = 1; start(0, 5001, "fleet phrase"); run(1.5);
  CHECK(peer_find(&D[0], D[3].id) < 0 && peer_find(&D[3], D[0].id) < 0 && D[0].N.bad > 0, "a drone with another fleet phrase: neither hears the other (%u bad packets)", (unsigned)D[0].N.bad);
  nd = DRONES; for (int i = 3; i < DRONES; i++) start(i, 6000 + i, "fleet phrase"); run(1.5);
  int used = 0; for (int s = 0; s < PEER_MAX; s++) used += D[0].P[s].used;
  CHECK(used == PEER_MAX && D[0].N.dropped > 0, "ten drones: each keeps %d in its table, the rest wait (%u packets without room)", used, (unsigned)D[0].N.dropped);
  printf("%s\n", fails ? "FAILED" : "all passed");
  return fails ? 1 : 0;
}
