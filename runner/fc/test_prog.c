/* Tests for programs on the data bus (prog_core.c): headers checked against the formulas, running on a change and by
 * period, waiting for inputs, memory kept between runs, results published, failures counted.
 *   node tools/prog_test_data.js /tmp/prog
 *   cc -O2 -Wall -Wextra -I.. -o test_prog test_prog.c prog_core.c bus.c ../rn_host.c ../rn.c -lm && ./test_prog /tmp/prog/prog.rnp */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "prog_core.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define ACAP 16384
#define CCAP 16384
#define PCAP 1024
static float ar[3][ACAP], pl[3][PCAP]; static int32_t cd[3][CCAP];

/* Apps (prog_add_app): a stand-in for the board's app host. App 0 doubles the barometer every other run (and doesn't
 * publish the others); app 1 traps; the host is told what it was given. */
static int calls[2], last_n_in[2]; static float last_dt[2];
static int host(void *ctx, int app, const float *in, int n_in, float *out, int n_out) {
  (void)ctx; calls[app]++; last_n_in[app] = n_in; last_dt[app] = in[n_in - 1];
  if (app == 1) return -3;
  if (calls[app] % 2) return 0;
  out[0] = 2 * in[0]; out[1] = (float)calls[app]; (void)n_out; return 1;
}
static void apps(bus *B) {
  static prog_state A;
  prog_init(&A, 0, B);
  CHECK(prog_add_app(&A, "twice", "user.twice", 2, "h n", 0, 2) == -1 && strstr(A.why, "no apps"), "a board with no app host refuses apps: %s", A.why);
  prog_apps(&A, host, 0);
  int a = prog_add_app(&A, "twice", "user.twice", 2, "h n", 0, 2), t = prog_add_app(&A, "trap", "user.trap", 1, "x", 0.1f, 2), w = prog_add_app(&A, "wrong", "user.wrong", 1, "x", 0.1f, 3);
  CHECK(a == 0 && t == 1 && w == 2 && A.P[a].app == 0 && A.P[t].app == 1 && A.P[w].app == 2 && A.P[a].fn == -1, "apps added: %d %d %d (%s)", a, t, w, A.why);
  CHECK(prog_add(&A, "smooth", "user.x", 3, "h rate n", 0) == -1 && strstr(A.why, "no formula"), "formulas still need the step runner");
  prog_read(&A, a, "sensor.baro"); prog_trigger(&A, a, "sensor.baro"); prog_read(&A, t, "sensor.baro"); prog_read(&A, w, "sensor.baro");
  CHECK(prog_check(&A, a) == 0 && prog_check(&A, t) == 0, "apps check against the inputs they were compiled for: %s", A.why);
  CHECK(prog_check(&A, w) == -1 && strstr(A.why, "inputs"), "an app compiled for other inputs is refused: %s", A.why);
  int baro = bus_find(B, "sensor.baro"), out = bus_find(B, "user.twice"); uint32_t seen = 0; int pubs = 0;
  double t0 = B->now;
  for (int i = 1; i <= 1000; i++) {
    bus_clock(B, t0 + i * 0.001);
    if (i % 40 == 0) { float h = 5; bus_pub(B, baro, &h, 1); }
    prog_step(&A);
    if (bus_changed(B, out, &seen)) pubs++;
  }
  const float *v = bus_get(B, out, 0, 0);
  /* (26: the reading already there, then 25 more) */
  CHECK(calls[0] == 26 && A.P[a].runs == 26 && pubs == 13, "the app ran on each reading (26: %d, %u runs) and published when it said so (13: %d)", calls[0], A.P[a].runs, pubs);
  CHECK(v && v[0] == 10 && v[1] == 26, "what it published: %g %g", v ? v[0] : -1, v ? v[1] : -1);
  CHECK(last_n_in[0] == 2 && last_dt[0] > 0.0399f && last_dt[0] < 0.0401f && last_dt[1] > 0.0999f && last_dt[1] < 0.1001f, "its inputs: the barometer, then dt (%d floats, dt %g, %g)", last_n_in[0], last_dt[0], last_dt[1]);
  CHECK(A.P[t].fails == 10 && A.P[t].err == -3 && A.P[t].runs == 0 && !bus_get(B, bus_find(B, "user.trap"), 0, 0), "a trapping app: %u failures, error %d, nothing published", A.P[t].fails, A.P[t].err);
}

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "usage: %s prog.rnp\n", argv[0]); return 2; }
  FILE *f = fopen(argv[1], "rb"); if (!f) { perror(argv[1]); return 2; }
  static uint8_t img[1 << 18]; uint32_t len = (uint32_t)fread(img, 1, sizeof img, f); fclose(f);
  float *arenas[3] = { ar[0], ar[1], ar[2] }, *pools[3] = { pl[0], pl[1], pl[2] }; int32_t *codes[3] = { cd[0], cd[1], cd[2] };
  static rn_host H; int e = rn_host_init(&H, img, len, arenas, ACAP, codes, CCAP, pools, PCAP);
  CHECK(!e, "image loads: %d", e); if (e) return 1;

  static bus B; bus_init(&B); bus_clock(&B, 1);
  int baro = bus_topic(&B, "sensor.baro", 1, "height"), imu = bus_want_topic(&B, "sensor.imu", 6, "gyro[3] accel[3]", 0);   /* (the IMU: a mirror) */
  static prog_state G; prog_init(&G, &H, &B);
  int sm = prog_add(&G, "smooth", "user.smooth", 3, "h rate n", 0), tk = prog_add(&G, "ticker", "user.ticker", 1, "t", 0.1f), br = prog_add(&G, "broken", "user.broken", 1, "x", 0.05f);
  CHECK(sm == 0 && tk == 1 && br == 2, "added: %d %d %d (%s)", sm, tk, br, G.why);
  CHECK(prog_add(&G, "nope", "user.nope", 1, "v", 1) == -1 && strstr(G.why, "no formula"), "a program with no formula refused: %s", G.why);
  CHECK(prog_add(&G, "smooth", "fc.thing", 1, "v", 1) == -1 && strstr(G.why, "user."), "only user. topics: %s", G.why);
  CHECK(prog_add(&G, "ticker", "user.smooth", 1, "v", 1) == -1, "a topic another program writes, with another layout, refused");
  CHECK(prog_read(&G, sm, "sensor.baro") == 0 && prog_read(&G, sm, "sensor.imu") == 0, "reads, a mirror among them");
  CHECK(prog_read(&G, sm, "sensor.nothing") == -1 && strstr(G.why, "not on"), "a topic that isn't on the bus refused: %s", G.why);
  CHECK(prog_read(&G, sm, "user.smooth") == -1, "its own topic refused");
  CHECK(prog_check(&G, sm) == -1 && strstr(G.why, "runs on a change"), "on a change needs its trigger: %s", G.why);
  CHECK(prog_trigger(&G, sm, "sensor.imu") == 0 && prog_trigger(&G, sm, "user.ticker") == -1, "the trigger must be a read");
  prog_trigger(&G, sm, "sensor.baro");
  CHECK(prog_check(&G, sm) == 0, "smooth checks: %s", G.why);
  prog_read(&G, tk, "sensor.baro"); CHECK(prog_check(&G, tk) == 0, "ticker checks: %s", G.why);
  prog_read(&G, br, "sensor.baro"); CHECK(prog_check(&G, br) == 0, "broken checks: %s", G.why);
  { prog_state X; prog_init(&X, &H, &B); int k = prog_add(&X, "smooth", "user.other", 3, "h rate n", 0.1f); prog_read(&X, k, "sensor.baro");
    CHECK(prog_check(&X, k) == -1 && strstr(X.why, "inputs"), "a header that reads other than the formula takes is refused: %s", X.why);
    k = prog_add(&X, "ticker", "user.other2", 2, "a b", 0.1f); prog_read(&X, k, "sensor.baro");
    CHECK(prog_check(&X, k) == -1 && strstr(X.why, "returns"), "a topic laid out otherwise than it returns is refused: %s", X.why); }

  /* 2 s at 1 kHz: the barometer at 25 Hz, the IMU mirror arriving after 0.5 s */
  int sm_out = bus_find(&B, "user.smooth"), tk_out = bus_find(&B, "user.ticker"); uint32_t seen = 0; int smooth_new = 0;
  for (int i = 1; i <= 2000; i++) {
    double t = 1 + i * 0.001; bus_clock(&B, t);
    if (i % 40 == 0) { float h = 10 + (float)i * 0.001f; bus_pub(&B, baro, &h, 1); }
    if (i == 500) { float d[] = { BUS_VERSION, 1, (float)(B.T[imu].hash >> 16), (float)(B.T[imu].hash & 0xffff), 1, 0, 6, 0, 0, 0, 0, 0, 9.8f }; bus_unpack(&B, d, 13); }
    prog_step(&G);
    if (bus_changed(&B, sm_out, &seen)) smooth_new++;
  }
  const float *v = bus_get(&B, sm_out, 0, 0);
  CHECK(G.P[sm].waits >= 10 && G.P[sm].waits <= 13, "smooth waited for the IMU: %u", G.P[sm].waits);
  CHECK(G.P[sm].runs == 38 && smooth_new == 38, "smooth ran on each barometer reading once the IMU came: %u runs, %d published", G.P[sm].runs, smooth_new);
  CHECK(v && v[2] == 38 && v[0] > 11.5f && v[0] < 12.1f && v[1] > 0.5f && v[1] < 1.5f, "its memory and result: n %g, h %g, rate %g", v ? v[2] : -1, v ? v[0] : -1, v ? v[1] : -1);
  const float *tv = bus_get(&B, tk_out, 0, 0);
  CHECK(G.P[tk].runs >= 19 && G.P[tk].runs <= 20 && tv && tv[0] > 1.8f && tv[0] < 2.05f, "ticker every 100 ms: %u runs, t %g", G.P[tk].runs, tv ? tv[0] : -1);
  CHECK(G.P[br].fails >= 35 && G.P[br].runs == 0 && !bus_get(&B, bus_find(&B, "user.broken"), 0, 0), "broken: %u failed runs, nothing published", G.P[br].fails);
  apps(&B);

  printf(fails ? "%d FAILED\n" : "programs: all passed\n", fails);
  return fails != 0;
}
