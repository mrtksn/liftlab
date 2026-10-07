/* Tests for the data bus (bus.h, bus.c): publishing, sequence and age, watching, the limits, and two boards copying
 * topics through their frames: periods, on change, subscriptions lapsing, sizes that don't match, malformed frames.
 *   cc -O2 -Wall -Wextra -I.. -o test_bus test_bus.c bus.c */
#include <stdio.h>
#include <string.h>
#include "bus.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static bus A, P;                          /* a flight controller and a Pi */
static float fr[2048];

/* one exchange over the link: the Pi's subscription (when it renews), then what is due for it */
static int link_step(int *sent) {
  int n = bus_sub_pack(&P, fr, 2048); if (n) CHECK(bus_sub_take(&A, 7, fr, n) >= 0, "sub frame refused");
  n = bus_pack(&A, 7, fr, 2048); if (sent) *sent = n;
  return n ? bus_unpack(&P, fr, n) : 0;
}

int main(void) {
  /* ── one board ── */
  bus_init(&A); bus_clock(&A, 10);
  int att = bus_topic(&A, "fc.attitude", 7), st = bus_topic(&A, "fc.state", 3);
  CHECK(att == 0 && st == 1, "ids %d %d", att, st);
  CHECK(bus_topic(&A, "fc.attitude", 7) == att, "registering again finds it");
  CHECK(bus_topic(&A, "fc.attitude", 6) == -1, "another size refused");
  CHECK(bus_topic(&A, "", 3) == -1 && bus_topic(&A, "x", 0) == -1 && bus_topic(&A, "x", BUS_VALS + 1) == -1, "bad name or size refused");
  CHECK(bus_topic(&A, "a.name.that.is.far.too.long.for.it", 3) == -1, "long name refused");
  CHECK(bus_get(&A, att, 0, 0) == 0 && bus_age(&A, att) == -1, "nothing before the first publish");
  uint32_t seen = 0; CHECK(!bus_changed(&A, att, &seen), "no change before the first publish");
  float q[7] = { 1, 0, 0, 0, 0.1f, 0.2f, 0.3f };
  CHECK(bus_pub(&A, att, q, 7) == 0, "publish");
  uint32_t seq; double t; const float *v = bus_get(&A, att, &seq, &t);
  CHECK(v && v[0] == 1 && v[6] == 0.3f && seq == 1 && t == 10, "read back: seq %u t %g", seq, t);
  bus_clock(&A, 10.25); CHECK(bus_age(&A, att) == 0.25, "age %g", bus_age(&A, att));
  CHECK(bus_changed(&A, att, &seen) && seen == 1 && !bus_changed(&A, att, &seen), "watch sees it once");
  float two[2] = { 1, 1 }; CHECK(bus_pub(&A, st, two, 2) == 0 && bus_get(&A, st, 0, 0)[2] == 0, "fewer values: the rest 0");
  CHECK(bus_pub(&A, st, q, 7) == -1 && bus_pub(&A, 99, q, 1) == -1, "too many values or no such topic refused");
  CHECK(bus_find(&A, "fc.state") == st && bus_find(&A, "nope") == -1, "find");
  { bus F; bus_init(&F); int k = 0; char nm[8];
    for (; k < BUS_TOPICS + 2; k++) { snprintf(nm, sizeof nm, "t%d", k); if (bus_topic(&F, nm, 32) < 0) break; }
    CHECK(k == BUS_POOL / 32, "the pool fills: %d topics of 32", k);
    bus_init(&F); for (k = 0; k < BUS_TOPICS + 2; k++) { snprintf(nm, sizeof nm, "s%d", k); if (bus_topic(&F, nm, 1) < 0) break; }
    CHECK(F.nt == BUS_TOPICS, "the table fills: %d", F.nt); }

  /* ── two boards: the Pi subscribes to the flight controller's topics ── */
  bus_init(&P); bus_clock(&P, 3); bus_clock(&A, 20);
  int m_att = bus_want_topic(&P, "fc.attitude", 7, 0.02f), m_st = bus_want_topic(&P, "fc.state", 3, 0);
  CHECK(m_att >= 0 && m_st >= 0 && P.T[m_att].kind == BUS_MIRROR, "mirrors made");
  CHECK(bus_pub(&P, m_att, q, 7) == -1, "a mirror can't be written here (one writer)");
  CHECK(bus_want_topic(&P, "fc.attitude", 7, 0.05f) == m_att && P.nw == 2, "wanting again changes the period, not the list");
  P.W[0].period = 0.02f;
  int sent; CHECK(link_step(&sent) == 2, "both copied at once");
  v = bus_get(&P, m_att, &seq, &t);
  CHECK(v && v[6] == 0.3f && seq == 1 && t == 3 - 10.0, "mirror: values, the writer's seq, its time from the age (%g)", t);
  CHECK(link_step(0) == 0, "nothing new: nothing sent");
  /* period: attitude at most every 20 ms; state on every change */
  int got_att = 0, got_st = 0; uint32_t s_att = P.T[m_att].seq, s_st = P.T[m_st].seq;
  for (int i = 1; i <= 100; i++) {                                  /* 100 ms of a 1 kHz flight core */
    bus_clock(&A, 20 + i * 0.001); bus_clock(&P, 3 + i * 0.001);
    q[4] = (float)i; bus_pub(&A, att, q, 7);
    if (i % 10 == 0) { two[0] = (float)i; bus_pub(&A, st, two, 2); }
    link_step(0);
    if (bus_changed(&P, m_att, &s_att)) got_att++;
    if (bus_changed(&P, m_st, &s_st)) got_st++;
  }
  CHECK(got_att >= 4 && got_att <= 6, "attitude every 20 ms: %d copies in 100 ms", got_att);
  CHECK(got_st == 10, "state on each change: %d", got_st);
  CHECK(bus_get(&P, m_st, 0, 0)[0] == 100, "the latest state");
  /* on change means the values: the same state published every step travels only as a heartbeat */
  got_st = 0;
  for (int i = 1; i <= 2000; i++) { bus_clock(&A, 20.1 + i * 0.001); bus_clock(&P, 3.1 + i * 0.001); bus_pub(&A, st, two, 2); link_step(0); if (bus_changed(&P, m_st, &s_st)) got_st++; }
  CHECK(got_st >= 3 && got_st <= 5, "unchanged for 2 s: %d heartbeats", got_st);
  CHECK(bus_age(&P, m_st) <= BUS_BEAT + 1e-6, "its mirror's age stays under the heartbeat: %g", bus_age(&P, m_st));
  /* lapsing: the Pi stops renewing */
  int nw = P.nw; P.nw = 0;
  bus_clock(&A, 25); q[4] = -1; bus_pub(&A, att, q, 7);
  CHECK(bus_pack(&A, 7, fr, 2048) == 0 && A.ns == 0, "a subscription not renewed for 2 s lapses (%d left)", A.ns);
  P.nw = nw;
  /* another board asking for the same: served separately */
  { float sub[64]; int n = bus_sub_pack(&P, sub, 64); CHECK(bus_sub_take(&A, 9, sub, n) == 2 && bus_sub_take(&A, 7, sub, n) == 2 && A.ns == 4, "two peers: %d subscriptions", A.ns);
    CHECK(bus_pack(&A, 9, fr, 2048) > 0 && bus_pack(&A, 9, fr, 2048) == 0 && bus_pack(&A, 7, fr, 2048) > 0, "each peer gets its own copy once"); }

  /* ── mismatches and malformed frames ── */
  { bus O; bus_init(&O); int w = bus_want_topic(&O, "fc.state", 4, 0);   /* built against another layout */
    float sub[16]; int n = bus_sub_pack(&O, sub, 16); int before = A.n_unknown;
    CHECK(bus_sub_take(&A, 3, sub, n) == 0 && A.n_unknown == (uint32_t)before + 1, "a subscription with another size is left out");
    bus_clock(&A, 26); bus_pub(&A, st, two, 2);
    float d[] = { BUS_VERSION, 1, (float)(A.T[st].hash >> 16), (float)(A.T[st].hash & 0xffff), 5, 0, 3, 1, 2, 3 };
    CHECK(bus_unpack(&O, d, 10) == 0 && O.T[w].bad == 1 && !O.T[w].has, "data of another size is refused and counted"); }
  { float bad1[] = { 2, 0 }; CHECK(bus_unpack(&P, bad1, 2) == -1, "another version refused");
    float bad2[] = { BUS_VERSION, 1, 1, 2, 3, 4, 5 }; CHECK(bus_unpack(&P, bad2, 7) == -1, "a short frame refused");
    float bad3[] = { BUS_VERSION, 1, 0.5f, 2, 3, 4 }; CHECK(bus_sub_take(&A, 1, bad3, 6) == -1, "a hash that isn't whole refused");
    float nanf_ = 0.0f / 0.0f; float bad4[] = { BUS_VERSION, 1, 1, 2, 3, nanf_, 1, 9 }; CHECK(bus_unpack(&P, bad4, 8) == -1, "a NaN age refused");
    float empty[] = { BUS_VERSION, 0 }; CHECK(bus_unpack(&P, empty, 2) == 0, "an empty frame is fine"); }
  CHECK(sizeof(bus) < 16384, "small enough for an ESP32: %u bytes", (unsigned)sizeof(bus));

  printf(fails ? "%d FAILED\n" : "bus: all passed (%u bytes per board)\n", fails ? fails : (int)sizeof(bus));
  return fails != 0;
}
