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

  printf(fails ? "%d FAILED\n" : "programs: all passed\n", fails);
  return fails != 0;
}
