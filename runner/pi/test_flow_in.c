/* Checks the camera flow's calibration against the gyro (flow_in.c): a drone rocked by hand (roll and pitch, made up
 * here) seen by a camera whose lens scale, mirroring, turn about its axis and delay are known; flowcal must find them. */
#include "flow_in.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
static unsigned rs = 777;
static float randn(void) { rs = rs * 1103515245u + 12345u; float u = ((rs >> 8) & 0xffff) / 65536.0f + 1e-6f; rs = rs * 1103515245u + 12345u; float v = ((rs >> 8) & 0xffff) / 65536.0f; return sqrtf(-2 * logf(u)) * cosf(6.2831853f * v); }

static void gyro(double t, float w[3]) {     /* rocking: roll and pitch at different rates, ~1 rad/s */
  w[0] = (float)(1.2 * sin(2 * M_PI * 0.7 * t)); w[1] = (float)(0.9 * sin(2 * M_PI * 1.1 * t + 1)); w[2] = 0.1f;
}

static void run(float scale, int mirror, float psi_deg, double delay, const char *want, int telem) {
  static flow_in FI; flow_in_init(&FI);
  char reply[1000], line[256]; double psi = psi_deg * M_PI / 180;
  flow_in_command(&FI, "flowcal start", 0, reply, sizeof reply);
  /* the camera's frames every 1/60 s, each read with the gyro's average over the frame; the gyro at 100 Hz reaching
   * the Pi `delay` after the frame's time (the frames are early against it) */
  double tf = 0, tg = 0, dtf = 1.0 / 60;
  for (int step = 0; step < 6000; step++) {
    double t = step * 0.001;
    if (t >= tg) { float w[3]; gyro(tg - delay, w); for (int i = 0; i < 3; i++) w[i] += 0.01f * randn();
      if (telem) { for (int i = 0; i < 3; i++) w[i] *= 57.29578f; flow_in_gyro_telem(&FI, tg, w); tg += 0.05; } else { flow_in_gyro(&FI, tg, w); tg += 0.01; } }
    if (t >= tf + dtf + 0.015) {                                 /* the reading arrives 15 ms after its exposure */
      tf += dtf; float wa[2] = { 0, 0 };
      for (int j = 0; j < 10; j++) { float w[3]; gyro(tf - dtf * (j + 0.5) / 10, w); wa[0] += w[0] / 10; wa[1] += w[1] / 10; }
      /* camera axes: turned psi about the down axis; flow = J·w_cam, J = [[0,1],[−1,0]], times the lens error, maybe mirrored */
      float cx = (float)(cos(psi) * wa[0] + sin(psi) * wa[1]), cy = (float)(-sin(psi) * wa[0] + cos(psi) * wa[1]);
      float fx = scale * cy, fy = -scale * cx; if (mirror) fx = -fx;
      fx += 0.02f * randn(); fy += 0.02f * randn();
      snprintf(line, sizeof line, "flow %.4f %.4f %.4f %.4f 0.80 0 0 4.0 16 60 0.1 0.1 3.0 %d 0.004000 0", tf, dtf, fx, fy, step);
      CHECK(flow_in_reading(&FI, line, t), "reading not taken");
    }
  }
  flow_in_command(&FI, "flowcal", 6, reply, sizeof reply);
  printf("%s\n  %s\n", want, reply);
  float rad = 0; char *p = strstr(reply, "--rad-px "); if (p) rad = strtof(p + 9, NULL);
  CHECK(p && fabsf(rad - 0.004f / scale) < 0.0001f, "rad/px %.5f, want %.5f", rad, 0.004 / scale);
  CHECK(!!strstr(reply, "MIRRORED") == mirror, "mirroring");
  char want_turn[32]; snprintf(want_turn, sizeof want_turn, "turned %+.0f°", psi_deg);
  CHECK(strstr(reply, want_turn) != NULL, "the camera's turn: want %s", want_turn);
  char want_lag[48]; snprintf(want_lag, sizeof want_lag, "gyro taken %.0f ms later", delay * 1e3);
  CHECK(strstr(reply, want_lag) != NULL, "the delay: want %s", want_lag);
}

int main(void) {
  static flow_in FI; flow_in_init(&FI); char reply[1000];
  CHECK(flow_in_command(&FI, "flow", 0, reply, sizeof reply) && strstr(reply, "no readings"), "flow with nothing: %s", reply);
  CHECK(!flow_in_command(&FI, "flowers", 0, reply, sizeof reply) && !flow_in_reading(&FI, "fly", 0), "other commands left alone");
  {   /* held still: the readings don't count (they say nothing about the lens) */
    flow_in_command(&FI, "flowcal start", 0, reply, sizeof reply); float w0[3] = { 0.02f, -0.01f, 0 }; char line[200];
    for (int i = 0; i < 300; i++) { double t = i * 0.01; flow_in_gyro(&FI, t, w0); if (i % 2) { snprintf(line, sizeof line, "flow %.4f 0.0200 0.01 -0.02 0.9", t - 0.03); flow_in_reading(&FI, line, t); } }
    CHECK(FI.cal_n == 0, "still readings counted: %ld", FI.cal_n);
  }
  run(1.0f, 0, 0, 0.020, "a camera as the design has it, 20 ms late:", 0);
  run(1.3f, 1, 90, 0.040, "a lens 1.3× off, mirrored, turned 90°, 40 ms late:", 0);
  run(0.8f, 0, -45, 0.000, "a lens 0.8×, turned −45°, on time:", 0);
  run(1.2f, 1, 150, 0.020, "disarmed (the telemetry's gyro at 20 Hz): a lens 1.2×, mirrored, turned 150°, 20 ms late:", 1);
  if (fails) { printf("flow_in: %d checks failed\n", fails); return 1; }
  printf("flow_in: all checks passed\n");
  return 0;
}
