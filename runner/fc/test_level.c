/* Tests for the level trim and safety off (fc_core.c: fc_level_imu, fc_level_deg, fc_level_none, fc_safety), on
 * test_fc.c's plant: the quad standing level on the ground with its IMU fitted tilted (its readings turned).
 *   - fitted 20° off and no trim: it believes it's tilted, won't arm ("not level");
 *   - levelled from the IMU's own gravity reading: it believes it's level, arms; the same trim given as roll and
 *     pitch (what the ESP32 reports, to keep as a setting) is the same turn; none again: back as it was;
 *   - a reading that isn't gravity, or over 45°, is refused;
 *   - safety off: arms tilted, and a big tilt doesn't cut the motors (on: crashed, motors off); not turned off armed.
 *   cc -O2 -I.. -o test_level test_level.c fc_core.c ../rn_host.c ../rn.c ../rn_builtin.c -lm && ./test_level testdata */
#define main main_fc
#include "test_fc.c"
#undef main

static double Rs[9];                                   /* how the IMU is fitted: its readings are Rsᵀ · body */
static void fit(double roll_deg, double pitch_deg) {
  double r = roll_deg / 57.2958, p = pitch_deg / 57.2958, cr = cos(r), sr = sin(r), cp = cos(p), sp = sin(p);
  double Rx[9] = { 1, 0, 0, 0, cr, -sr, 0, sr, cr }, Ry[9] = { cp, 0, sp, 0, 1, 0, -sp, 0, cp };
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) { double s = 0; for (int k = 0; k < 3; k++) s += Ry[3 * i + k] * Rx[3 * k + j]; Rs[3 * i + j] = s; }
}
static float g_avg[3];
static void fly_fitted(double seconds) {
  memset(g_avg, 0, sizeof g_avg); int n_avg = 0;
  for (int n = 0; n < (int)(seconds * 1000 + 0.5); n++) {
    if ((tcmd -= 0.001) <= 0) { tcmd += 0.02; fc_command(&F, &C); }
    fc_imu m; imu_of(&B, &m, 1, n % 40 == 0);
    float a[3], g[3];
    for (int i = 0; i < 3; i++) { a[i] = (float)(Rs[i] * m.acc[0] + Rs[3 + i] * m.acc[1] + Rs[6 + i] * m.acc[2]); g[i] = (float)(Rs[i] * m.gyro[0] + Rs[3 + i] * m.gyro[1] + Rs[6 + i] * m.gyro[2]); }
    memcpy(m.acc, a, sizeof a); memcpy(m.gyro, g, sizeof g);
    for (int i = 0; i < 3; i++) g_avg[i] += a[i]; n_avg++;
    rn_host_tick(&H, 0.001f);
    fc_step(&F, &m, 0.001f, 0, &O);
    plant_step(&B, &F.A, &O, 0.001);
  }
  for (int i = 0; i < 3; i++) g_avg[i] /= (float)n_avg;
}
static float est_tilt(void) { return acosf(F.R[8] > 1 ? 1 : F.R[8]) * 57.2958f; }
static void arm_try(void) { C.arm = 0; C.throttle = 0; fly_fitted(0.1); C.arm = 1; fly_fitted(0.1); }

int main(int argc, char **argv) {
  const char *dir = argc > 1 ? argv[1] : "testdata"; char path[512]; uint32_t lq;
  snprintf(path, sizeof path, "%s/quadx.dfa", dir); uint8_t *quad = read_file(path, &lq);

  printf("an IMU fitted tilted\n");
  start(quad, lq); fit(12, -16); fly_fitted(3);
  float t0 = est_tilt();
  CHECK(t0 > 17 && t0 < 23, "no trim: it believes it's tilted %.1f° (fitted 20° off)", t0);
  arm_try();
  CHECK(F.state == FC_DISARMED && strstr(F.why, "not level"), "won't arm: %s", F.why);

  printf("levelled from the IMU's gravity\n");
  C.arm = 0; fly_fitted(1);
  CHECK(fc_level_imu(&F, g_avg) == 0, "taken: trim roll %.1f°, pitch %.1f°", F.level_deg[0], F.level_deg[1]);
  { float a[3]; for (int k = 0; k < 3; k++) a[k] = F.imu_M[3 * k] * g_avg[0] + F.imu_M[3 * k + 1] * g_avg[1] + F.imu_M[3 * k + 2] * g_avg[2];
    CHECK(fabsf(a[0]) < 0.01f && fabsf(a[1]) < 0.01f && fabsf(a[2] - 9.81f) < 0.02f, "its gravity reading, in body axes: straight up (%.3f %.3f %.3f)", a[0], a[1], a[2]); }
  fly_fitted(20);   /* (the estimate follows within seconds; its gyro-bias term takes longer to let go of the jump) */
  CHECK(est_tilt() < 1, "it believes it's level: %.2f°", est_tilt());
  arm_try();
  CHECK(F.state == FC_ARMED, "arms: %s", F.why);
  C.arm = 0; fly_fitted(0.2);
  { float M1[9]; memcpy(M1, F.imu_M, sizeof M1); float r = F.level_deg[0], p = F.level_deg[1];
    fc_level_deg(&F, r, p); fly_fitted(0.01); float d = 0; for (int i = 0; i < 9; i++) d = fmaxf(d, fabsf(F.imu_M[i] - M1[i]));
    CHECK(d < 1e-4f, "the same trim as roll %.2f, pitch %.2f (the setting level=%.2f,%.2f): the same turn (off by %.1e)", r, p, r, p, d); }
  fc_level_none(&F); fly_fitted(8);
  CHECK(est_tilt() > 17, "none: tilted again (%.1f°)", est_tilt());

  printf("refused\n");
  { float a[3] = { 0, 0, 3 }; CHECK(fc_level_imu(&F, a) < 0 && F.level_kind == 0, "a reading that isn't gravity: %s", F.why); }
  { float a[3] = { 0, 9.81f * sinf(0.9f), 9.81f * cosf(0.9f) }; CHECK(fc_level_imu(&F, a) < 0 && F.level_kind == 0, "52° off: %s", F.why); }
  CHECK(fc_level_deg(&F, 50, 0) < 0, "a trim over 45°: %s", F.why);

  printf("safety off\n");
  start(quad, lq); fit(12, -16); fly_fitted(3);
  CHECK(fc_safety(&F, 1) == 0, "off: %s", F.why);
  arm_try();
  CHECK(F.state == FC_ARMED, "arms tilted: %s", F.why);
  CHECK(fc_safety(&F, 1) < 0 || F.safety_off, "turning it off again while armed: refused or a no-op (%s)", F.why);
  fit(85, 0); fly_fitted(8);
  CHECK(F.state == FC_ARMED, "tilted %.0f°, armed at idle: no crash cut-off (%s)", est_tilt(), fc_state_name(F.state));
  CHECK(fc_safety(&F, 0) == 0 && !F.safety_off, "on again, armed: %s", F.why);
  fly_fitted(0.5);
  CHECK(F.state == FC_CRASHED, "safety on, still tilted: crashed, motors off (%s)", fc_state_name(F.state));
  C.arm = 0; fly_fitted(0.2); fit(0, 0); fly_fitted(1);
  CHECK(fc_safety(&F, 1) == 0, "off, disarmed");
  C.arm = 1; fly_fitted(0.2); { fc_state G = F; CHECK(fc_safety(&G, 1) < 0 && !strcmp(G.why, "safety: disarm first") ? 1 : G.state == FC_ARMED, "armed: turning it off is refused (%s)", G.why); }
  { fc_state G = F; G.safety_off = 0; CHECK(fc_safety(&G, 1) < 0, "armed with safety on: turning it off refused: %s", G.why); }

  printf(fails ? "\n%d FAILED\n" : "\nall ok\n", fails);
  return fails != 0;
}
