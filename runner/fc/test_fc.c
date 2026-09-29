/* Tests for the flight code (fc_core.c) on the built-in flight program, with no hardware: airframes exported
 * from the simulator (testdata/NAME.dfa) fly a simple rigid body whose actuators are exactly the airframe's model
 * (thrust per the believed throttle curve, the effect columns at the servos' angles). The simulator's
 * firmware-in-the-loop mode flies the same code against the full physics.
 *   cc -O2 -I.. -o test_fc test_fc.c fc_core.c ../rn_host.c ../rn.c ../rn_builtin.c -lm && ./test_fc testdata */
#include "fc_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern const uint8_t *const rn_builtin_img;
extern const uint32_t rn_builtin_len;

static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

static float arenas_[3][65536], pools_[3][8192];
static int32_t codes_[3][32768];
static rn_host H;
static fc_state F;

static uint8_t *read_file(const char *path, uint32_t *len) {
  FILE *f = fopen(path, "rb"); if (!f) { perror(path); exit(2); }
  static uint8_t buf[4][8192]; static int k = 0; uint8_t *b = buf[k++ & 3];
  *len = (uint32_t)fread(b, 1, 8192, f); fclose(f); return b;
}

/* ── the plant: a rigid body driven by the airframe's own model ── */
typedef struct { double p[3], v[3], q[4], w[3], th[FC_MAX_JOINTS]; double t; int on_ground, dead; double gyro_bias[3], acc_bias[3]; } body;
static void qm(double *R, const double *q) {
  double w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - w * z); R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z); R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y); R[7] = 2 * (y * z + w * x); R[8] = 1 - 2 * (x * x + y * y);
}
static void basis(double *v, const double *ang, int k) {
  int n = 1; v[0] = 1;
  for (int i = 0; i < k; i++) { double f[3] = { 1, cos(ang[i]), sin(ang[i]) }; for (int a = n - 1; a >= 0; a--) { double x = v[a]; v[3 * a] = x * f[0]; v[3 * a + 1] = x * f[1]; v[3 * a + 2] = x * f[2]; } n *= 3; }
}
static double acc_b[3];   /* last body specific force, for the accelerometer */
static void plant_step(body *B, const fc_airframe *A, const fc_out *o, double dt) {
  double a[6] = { 0 };
  for (int i = 0; i < A->n_motors; i++) {
    const fc_motor *M = &A->mot[i]; double ang[FC_MAX_CHAIN], b[FC_MAX_BASIS];
    for (int c = 0; c < M->n_chain; c++) ang[c] = B->th[M->chain[c]];
    basis(b, ang, M->n_chain);
    double u = o->motor[i], k = M->bend, T = i + 1 == B->dead ? 0 : (1 - k) * u + k * u * u;   /* dead: motor i failed */   /* the believed curve: the model is exact here */
    for (int r = 0; r < 6; r++) { double s = 0; for (int n = 0; n < M->n_basis; n++) s += b[n] * M->cols[n][r]; a[r] += s * T; }
  }
  for (int j = 0; j < A->n_joints; j++) {   /* servos move at their rate toward the command */
    double d = o->servo[j] - B->th[j], mx = A->jnt[j].rate * dt; B->th[j] += d > mx ? mx : d < -mx ? -mx : d;
  }
  double R[9]; qm(R, B->q);
  double aw[3]; for (int i = 0; i < 3; i++) aw[i] = R[3 * i] * a[0] + R[3 * i + 1] * a[1] + R[3 * i + 2] * a[2];
  aw[2] -= 9.81;
  for (int i = 0; i < 3; i++) aw[i] -= (i < 2 ? 0.4 : 0.3) * B->v[i];   /* drag (without it an accelerometer can't tell a steady lean from level) */
  /* the ground: pushes back as hard as needed (the accelerometer feels it: resting or the bump of landing),
   * and holds it still and level while it presses down */
  double push = 0;
  B->on_ground = B->p[2] <= 0 && aw[2] + B->v[2] / dt <= 0;
  if (B->p[2] <= 0 && aw[2] * dt + B->v[2] < 0) push = -(aw[2] * dt + B->v[2]) / dt;
  aw[2] += push;
  if (B->on_ground) { aw[0] = aw[1] = 0; B->v[0] = B->v[1] = 0; for (int k = 0; k < 3; k++) { B->w[k] = 0; a[3 + k] = 0; } }
  for (int i = 0; i < 3; i++) { B->v[i] += aw[i] * dt; B->p[i] += B->v[i] * dt; }
  if (B->p[2] < 0) B->p[2] = 0;
  for (int k = 0; k < 3; k++) B->w[k] += a[3 + k] * dt;
  double *q = B->q, w0 = B->w[0], w1 = B->w[1], w2 = B->w[2];
  double dq[4] = { -q[1] * w0 - q[2] * w1 - q[3] * w2, q[0] * w0 + q[2] * w2 - q[3] * w1, q[0] * w1 - q[1] * w2 + q[3] * w0, q[0] * w2 + q[1] * w1 - q[2] * w0 };
  double n = 0; for (int k = 0; k < 4; k++) { q[k] += 0.5 * dq[k] * dt; n += q[k] * q[k]; } n = sqrt(n); for (int k = 0; k < 4; k++) q[k] /= n;
  /* specific force in the body: R^T (a_world + g) */
  double sf[3] = { aw[0], aw[1], aw[2] + 9.81 }; qm(R, B->q);
  for (int i = 0; i < 3; i++) acc_b[i] = R[i] * sf[0] + R[3 + i] * sf[1] + R[6 + i] * sf[2];
  B->t += dt;
}
static void imu_of(const body *B, fc_imu *m, int have_gyro, int baro) {
  if (B->t == 0) { double R[9]; qm(R, B->q); for (int i = 0; i < 3; i++) acc_b[i] = R[6 + i] * 9.81; }   /* at rest before the first step */
  for (int k = 0; k < 3; k++) { m->gyro[k] = (float)(B->w[k] + B->gyro_bias[k]); m->acc[k] = (float)(acc_b[k] + B->acc_bias[k]); }
  m->have_gyro = have_gyro; m->baro_alt = (float)B->p[2] + 100; m->have_baro = baro;
}
static double tilt_deg(const body *B) { double R[9]; qm(R, B->q); return acos(R[8] > 1 ? 1 : R[8]) * 57.29578; }
static void euler(const body *B, double *roll, double *pitch, double *yaw) {
  double R[9]; qm(R, B->q); *roll = atan2(R[7], R[8]) * 57.29578; *pitch = -asin(R[6]) * 57.29578; *yaw = atan2(R[3], R[0]) * 57.29578;
}

/* ── a flight: 1 kHz control, 50 Hz commands (as from the Pi) ── */
static body B; static fc_out O; static fc_cmd C; static int link_up = 1, have_gyro = 1, baro = 1; static float vbatt_in = 0;
static double tcmd; static int trace = 0;
static void fly(double seconds) {
  for (int n = 0; n < (int)(seconds * 1000 + 0.5); n++) {
    if (link_up && (tcmd -= 0.001) <= 0) { tcmd += 0.02; fc_command(&F, &C); }
    fc_imu m; imu_of(&B, &m, have_gyro, baro && n % 40 == 0);   /* the barometer reads at 25 Hz */
    rn_host_tick(&H, 0.001f);
    fc_step(&F, &m, 0.001f, vbatt_in, &O);
    plant_step(&B, &F.A, &O, 0.001);
    if (trace == 2 && n % 200 == 0) { double r, p, y; euler(&B, &r, &p, &y); printf("    t=%.1f true r=%.1f p=%.1f | est r=%.1f p=%.1f | vx=%.2f |f|=%.3f\n", B.t, r, p, atan2(F.R[7], F.R[8]) * 57.3, -asin(F.R[6]) * 57.3, B.v[0], sqrt(acc_b[0]*acc_b[0]+acc_b[1]*acc_b[1]+acc_b[2]*acc_b[2])/9.81); }
    if (trace == 1 && n % 100 == 0) printf("    t=%.1f z=%.2f vz=%.2f | est vz=%.2f alt=%.2f az_f=%.2f trim=%.2f thr=%.2f hold=%d u=%.3f\n", B.t, B.p[2], B.v[2], F.vz_e, F.alt_e - 100, F.az_f, F.iAz, C.throttle, F.holding, O.motor[0]);
  }
}
static void start(const uint8_t *blob, uint32_t len) {
  static fc_state zero; F = zero;
  float *arenas[3] = { arenas_[0], arenas_[1], arenas_[2] }, *pools[3] = { pools_[0], pools_[1], pools_[2] };
  int32_t *codes[3] = { codes_[0], codes_[1], codes_[2] };
  memset(&H, 0, sizeof H);
  int e = rn_host_init(&H, rn_builtin_img, rn_builtin_len, arenas, 65536, codes, 32768, pools, 8192);
  if (e) { printf("rn_host_init: %d\n", e); exit(1); }
  if (fc_init(&F, &H)) { printf("fc_init: %s\n", F.why); exit(1); }
  if (fc_airframe_load(&F, blob, len)) { printf("airframe: %s\n", F.why); exit(1); }
  memset(&B, 0, sizeof B); B.q[0] = 1;
  memset(&C, 0, sizeof C); C.test_motor = -1; link_up = 1; have_gyro = 1; baro = 1; tcmd = 0; vbatt_in = 0;
}
static float max_motor(void) { float m = 0; for (int i = 0; i < F.A.n_motors; i++) if (O.motor[i] > m) m = O.motor[i]; return m; }

int main(int argc, char **argv) {
  const char *dir = argc > 1 ? argv[1] : "testdata"; char path[512]; uint32_t lq, lt, lr;
  snprintf(path, sizeof path, "%s/quadx.dfa", dir); uint8_t *quad = read_file(path, &lq);
  snprintf(path, sizeof path, "%s/tiltquad.dfa", dir); uint8_t *tilt = read_file(path, &lt);
  snprintf(path, sizeof path, "%s/tri.dfa", dir); uint8_t *tri = read_file(path, &lr);

  printf("airframe files\n");
  start(quad, lq);
  CHECK(F.A.n_motors == 4 && F.A.n_joints == 0 && fabsf(F.A.m - 0.87f) < 1e-3f, "quad X: %s", F.why);
  { uint8_t bad[8192]; memcpy(bad, quad, lq); bad[40] ^= 1; fc_state G = F; CHECK(fc_airframe_load(&G, bad, lq) < 0, "a flipped bit is refused: %s", G.why); }
  { fc_state G = F; CHECK(fc_airframe_load(&G, quad, lq - 8) < 0, "a cut-off file is refused: %s", G.why); }
  { fc_state G = F; CHECK(fc_airframe_load(&G, rn_builtin_img, 400) < 0, "a program is not an airframe: %s", G.why); }

  printf("arming\n");
  start(quad, lq); fly(0.3);
  CHECK(F.state == FC_DISARMED && max_motor() == 0, "disarmed: motors at 0");
  C.arm = 1; fly(0.1);
  CHECK(F.state == FC_DISARMED, "not before the attitude settles: %s", F.why);
  C.arm = 0; fly(0.5); C.arm = 1; C.throttle = 0.6f; fly(0.1);
  CHECK(F.state == FC_DISARMED, "not with the throttle up: %s", F.why);
  C.arm = 0; C.throttle = 0; fly(0.1);
  start(quad, lq); have_gyro = 0; fly(1); C.arm = 1; fly(0.1);
  CHECK(F.state == FC_DISARMED, "not without a gyro: %s", F.why);
  start(quad, lq); B.q[0] = cos(0.2); B.q[1] = sin(0.2); fly(1.5); C.arm = 1; fly(0.1);
  CHECK(F.state == FC_DISARMED, "not tilted 23°: %s", F.why);
  start(quad, lq); fly(1); C.arm = 1; fly(0.2);
  CHECK(F.state == FC_ARMED && fabsf(max_motor() - FC_IDLE) < 1e-6f, "armed, stick down: idle (%.2f)", max_motor());
  C.arm = 0; fly(0.1);
  CHECK(F.state == FC_DISARMED && max_motor() == 0, "disarm: motors at 0");

  printf("flying the quad (barometer)\n");
  start(quad, lq); fly(1); C.arm = 1; fly(0.2);
  C.throttle = 0.9f; fly(2); C.throttle = 0.5f; fly(3);
  double h0 = B.p[2]; fly(3);
  CHECK(h0 > 1.5 && fabs(B.p[2] - h0) < 0.25 && fabs(B.v[2]) < 0.1, "takes off and holds its height: %.2f m then %.2f m", h0, B.p[2]);
  CHECK(tilt_deg(&B) < 1, "level in hover (%.2f°)", tilt_deg(&B));
  C.throttle = 0.95f; fly(2.5); CHECK(B.v[2] > 1.5 && B.v[2] < 2.1, "throttle 0.95 → climbs at 2 m/s (a little less against drag): %.2f", B.v[2]);
  C.throttle = 0.2f; fly(2.5); CHECK(B.v[2] < -0.8 && B.v[2] > -1.3, "throttle 0.2 → sinks at 1.1 m/s: %.2f", B.v[2]);
  C.throttle = 0.5f; fly(4); CHECK(fabs(B.v[2]) < 0.1, "stick back to the middle: holds (%.2f m/s)", B.v[2]);
  /* Leaning: the firmware tracks the lean it believes in exactly; the true lean drifts from it by what the
   * attitude estimator (the flight formula) makes of the accelerometer while the drone speeds up or slows down. */
  h0 = B.p[2]; C.pitch = 0.5f; fly(2); { double r, p, y, pe = -asin(F.R[6]) * 57.29578; euler(&B, &r, &p, &y);
    CHECK(fabs(pe - 17.5) < 1.5 && fabs(p - 17.5) < 9 && fabs(r) < 1, "pitch stick 0.5 → 17.5° nose down: believes %.1f°, is %.1f° (roll %.1f°)", pe, p, r);
    CHECK(B.v[0] > 1, "and flies forward (%.1f m/s)", B.v[0]); }
  C.pitch = 0; C.roll = -1; fly(2); { double r, p, y, re = atan2(F.R[7], F.R[8]) * 57.29578; euler(&B, &r, &p, &y);
    CHECK(fabs(re + 35) < 1.5 && fabs(r + 35) < 5, "roll stick −1 → 35° left: believes %.1f°, is %.1f°", re, r); }
  C.roll = 0; fly(3); { double te = acos(F.R[8]) * 57.29578;
    CHECK(te < 1.5 && tilt_deg(&B) < 12 && fabs(B.p[2] - h0) < 1, "levels again at its height: believes %.1f°, is %.1f° while slowing down, %.2f m", te, tilt_deg(&B), B.p[2]); }
  { double v0 = hypot(B.v[0], B.v[1]); fly(8); CHECK(tilt_deg(&B) < 5 && hypot(B.v[0], B.v[1]) < 0.85 * v0, "and keeps slowing down, slowly: the estimator reads drag as lean (%.1f → %.1f m/s, %.1f°)", v0, hypot(B.v[0], B.v[1]), tilt_deg(&B)); }
  { double r, p, y0, y1; float s0 = F.yaw_sp; euler(&B, &r, &p, &y0); C.yaw = 0.5f; fly(1); C.yaw = 0; fly(1.5); euler(&B, &r, &p, &y1);
    CHECK(fabs((F.yaw_sp - s0) * 57.29578 - 57.3) < 0.5 && fabs(y1 - F.yaw_sp * 57.29578) < 2, "yaw stick 0.5 for 1 s → turns 57°: %.1f° (heading within %.1f°)", y1 - y0, fabs(y1 - F.yaw_sp * 57.29578)); }
  C.throttle = 1; fly(2); C.throttle = 0.5f; fly(3);

  printf("failsafe\n");
  { double z0 = B.p[2]; link_up = 0; C.pitch = 1; fly(0.4); CHECK(F.state == FC_ARMED, "0.4 s without commands: still flying");
    fly(0.2); CHECK(F.state == FC_FAILSAFE, "0.6 s: %s", F.why);
    fly(2.5); CHECK(B.v[2] < -0.8 && B.v[2] > -1.2 && tilt_deg(&B) < 2, "descends level at ~1 m/s: %.2f m/s, %.1f°", B.v[2], tilt_deg(&B));
    double tl = B.t; while (F.state == FC_FAILSAFE && B.t - tl < 30) fly(0.1);
    CHECK(F.state == FC_DISARMED && B.p[2] < 0.01 && max_motor() == 0 && strstr(F.why, "landed"), "lands from %.1f m and disarms: %s", z0, F.why); }
  { link_up = 1; C.arm = 0; C.pitch = 0; C.throttle = 0; fly(0.2); C.arm = 1; fly(0.2); C.throttle = 0.9f; fly(2); C.throttle = 0.5f; fly(1);
    link_up = 0; fly(1); link_up = 1; fly(0.1); CHECK(F.state == FC_ARMED, "commands back during the failsafe: the pilot has it again (%s)", F.why); }

  printf("failsafe without a barometer\n");
  start(quad, lq); baro = 0; fly(1); C.arm = 1; fly(0.2); C.throttle = 0.6f; fly(3);
  CHECK(B.v[2] > 2.0 && B.v[2] < 3.2, "throttle 0.6 → climbs faster and faster (1 m/s², less drag): %.2f m/s", B.v[2]);
  C.throttle = 0.5f; fly(2); double v1 = B.v[2]; fly(2);
  CHECK(fabs(B.v[2] - v1) < 0.3 && B.v[2] > 0.5, "throttle 0.5 keeps the vertical speed (drag slows it a little): %.2f → %.2f m/s", v1, B.v[2]);
  C.throttle = 0.3f; fly(2); C.throttle = 0.5f; fly(1);
  { double z0 = B.p[2]; link_up = 0; double tl = B.t; while (F.state != FC_DISARMED && B.t - tl < 40) fly(0.1);
    CHECK(F.state == FC_DISARMED && B.p[2] < 0.01, "comes down from %.1f m and disarms in %.1f s: %s", z0, B.t - tl, F.why); }

  printf("crash and sensor loss\n");
  start(quad, lq); fly(1); C.arm = 1; fly(0.2); C.throttle = 0.9f; fly(1.5); C.throttle = 0.5f; fly(0.5);
  B.dead = 2; fly(1);
  CHECK(F.state == FC_CRASHED && max_motor() == 0, "a motor fails, it flips past 75°: %s", F.why);
  C.arm = 1; fly(0.2); CHECK(F.state == FC_CRASHED, "stays off while the arm switch is still on");
  start(quad, lq); fly(1); C.arm = 1; fly(0.2); C.throttle = 0.9f; fly(1);
  have_gyro = 0; fly(0.1); CHECK(F.state == FC_ARMED, "0.1 s without IMU data: keeps going");
  fly(0.2); CHECK(F.state == FC_CRASHED && max_motor() == 0, "0.3 s: %s", F.why);

  printf("motor test\n");
  start(quad, lq); fly(1); C.test_motor = 2; C.test_throttle = 0.9f; fly(0.5);
  CHECK(F.state == FC_TESTING && O.motor[2] == 0.3f && O.motor[0] == 0 && O.motor[1] == 0 && O.motor[3] == 0, "one motor, capped at 0.3: %s", F.why);
  link_up = 0; fly(0.6); CHECK(F.state == FC_DISARMED && max_motor() == 0, "stops 0.5 s after the commands stop");
  link_up = 1; fly(0.2); CHECK(F.state == FC_DISARMED, "the same test command again doesn't restart it (test off first)");
  C.test_motor = -1; fly(0.1); C.test_motor = 0; fly(2.5); CHECK(F.state == FC_TESTING, "switched off and on: motor 1 runs");
  fly(0.6); CHECK(F.state == FC_DISARMED && max_motor() == 0, "stops %d s after it started, though test commands keep coming", (int)FC_TEST_S);
  C.test_motor = -1; C.arm = 1; fly(0.2); C.test_motor = 1; fly(0.2);
  CHECK(F.state == FC_ARMED && O.motor[1] == FC_IDLE, "not while armed");

  printf("tilt-rotor quad (stays level, servos steer)\n");
  start(tilt, lt); fly(1); C.arm = 1; fly(0.2); C.throttle = 0.9f; fly(2); C.throttle = 0.5f; fly(2);
  C.pitch = 0.5f; fly(2);
  { double r, p, y; euler(&B, &r, &p, &y); float s = 0; for (int j = 0; j < F.A.n_joints; j++) s += fabsf(O.servo[j]);
    CHECK(acos(F.R[8]) * 57.29578 < 1 && fabs(p) < 8 && fabs(r) < 2 && B.v[0] > 1, "pitch stick: flies forward level (believes %.1f°, is %.1f° while speeding up, %.1f m/s)", acos(F.R[8]) * 57.29578, p, B.v[0]);
    CHECK(s / F.A.n_joints > 0.1f, "with the rotors tilted (mean %.0f°)", s / F.A.n_joints * 57.3); }
  { double v0 = B.v[0]; C.pitch = 0; fly(3); CHECK(B.v[0] < 0.4 * v0, "slows down when let go, by drag (%.1f → %.1f m/s)", v0, B.v[0]); }

  printf("tricopter (tail servo)\n");
  start(tri, lr); fly(1); C.arm = 1; fly(0.2); C.throttle = 0.9f; fly(2); C.throttle = 0.5f; fly(2);
  { double r, p, y0, y1; euler(&B, &r, &p, &y0); CHECK(tilt_deg(&B) < 1.5, "hovers level (%.1f°)", tilt_deg(&B));
    C.yaw = -0.5f; fly(1); C.yaw = 0; fly(1.5); euler(&B, &r, &p, &y1); CHECK(fabs((y1 - y0) + 57.3) < 5, "yaw right 57°: %.1f°", y1 - y0); }

  printf("from the review\n");
  /* failsafe while sinking fast: slowing to the descent speed mustn't look like a landing */
  start(quad, lq); fly(1); C.arm = 1; fly(0.2); C.throttle = 1; fly(4); C.throttle = 0.06f; fly(3);
  { double z0 = B.p[2], v0 = B.v[2]; link_up = 0; double lowest_live = 1e9;
    fly(0.6); while (F.state == FC_FAILSAFE && B.t < 90) { fly(0.05); if (max_motor() > 0 && B.p[2] < lowest_live) lowest_live = B.p[2]; }
    CHECK(F.state == FC_DISARMED && B.p[2] < 0.01 && lowest_live < 0.3, "sinking %.1f m/s at %.1f m when the link drops: slows, lands, then disarms (motors ran down to %.2f m)", v0, z0, lowest_live); }
  /* link lost at idle on the ground: disarm, don't spool up for a descent */
  start(quad, lq); fly(1); C.arm = 1; fly(0.2); link_up = 0; fly(0.7);
  CHECK(F.state == FC_DISARMED && max_motor() == 0, "link lost at idle: %s", F.why);
  /* no re-arming by itself */
  link_up = 1; fly(0.2); CHECK(F.state == FC_DISARMED, "the arm switch still on after that: stays disarmed (%s)", F.why);
  C.arm = 0; fly(0.1); C.arm = 1; fly(0.1); CHECK(F.state == FC_ARMED, "switch off, then on: arms");
  /* arming at throttle 0.05 (one step up) is refused: armed means idle */
  C.arm = 0; fly(0.1); C.throttle = 0.05f; C.arm = 1; fly(0.1); CHECK(F.state == FC_DISARMED, "throttle 0.05: %s", F.why);
  /* a motor failing during the failsafe descent: crash cut-off */
  start(quad, lq); fly(1); C.arm = 1; fly(0.2); C.throttle = 1; fly(3); C.throttle = 0.5f; fly(1); link_up = 0; fly(1.5);
  B.dead = 1; fly(1.5); CHECK(F.state == FC_CRASHED && max_motor() == 0, "motor fails in the failsafe: %s", F.why);
  /* a command with NaN is ignored; out-of-range values are clamped */
  start(quad, lq); fly(1); C.arm = 1; fly(0.2); C.throttle = 0.9f; fly(1.5); C.throttle = 0.5f; fly(1);
  C.pitch = NAN; fly(0.3); C.pitch = 0; CHECK(F.state == FC_ARMED && F.cmd.pitch == 0 && tilt_deg(&B) < 2, "NaN pitch: ignored (%.1f°)", tilt_deg(&B));
  C.pitch = NAN; fly(0.7); C.pitch = 0; CHECK(F.state == FC_FAILSAFE, "NaN commands only, for 0.7 s: that's no commands (%s)", F.why);
  fly(0.3); C.roll = 7; fly(1); { double r, p, y; euler(&B, &r, &p, &y); CHECK(F.state == FC_ARMED && fabs(r - 35) < 5, "roll 7 is taken as 1: %.1f°", r); } C.roll = 0; fly(1);
  /* a formula result that isn't finite in flight: hold, then stop */
  { float keep = F.A.Jinv[0]; F.A.Jinv[0] = NAN; fly(0.03); CHECK(F.state == FC_ARMED && max_motor() > 0.3f, "formulas failing for 30 ms: last outputs held");
    fly(0.05); CHECK(F.state == FC_CRASHED && max_motor() == 0, "for 80 ms: %s", F.why); F.A.Jinv[0] = keep; }
  /* battery: a reading that doesn't fit the pack isn't used, and arming wants a plausible one when the wire is set */
  start(quad, lq); F.batt_wired = 1; vbatt_in = 1.3f; fly(1); C.arm = 1; fly(0.1);
  CHECK(F.state == FC_DISARMED, "sense wire loose (1.3 V): %s", F.why);
  F.batt_wired = 0; C.arm = 0; fly(0.1); C.arm = 1; fly(0.2); C.throttle = 0.9f; fly(2); C.throttle = 0.5f; fly(4);
  CHECK(F.state == FC_ARMED && max_motor() < 0.8f && fabs(B.v[2]) < 0.2, "flying with 1.3 V read: no ×12 on the throttles (%.2f)", max_motor());
  /* hours of uptime: the timeouts still work */
  start(quad, lq); F.t = 400000; fly(1); C.arm = 1; fly(0.2); C.throttle = 0.9f; fly(1.5); C.throttle = 0.5f; fly(1);
  link_up = 0; fly(0.6); CHECK(F.state == FC_FAILSAFE, "after 111 h: the command timeout still works (%s)", F.why);

  printf("failsafe with an accelerometer bias that appears in flight\n");
  { const float biases[] = { -0.5f, -0.3f, 0.3f, 0.5f, -0.3f, -0.1f, 0.1f, 0.3f }; const int baros[] = { 1, 1, 1, 1, 0, 0, 0, 0 };
    for (int k = 0; k < 8; k++) {
      start(quad, lq); baro = baros[k]; fly(1); C.arm = 1; fly(0.2);
      if (baro) { C.throttle = 0.8f; fly(3); C.throttle = 0.5f; fly(1); }
      else { C.throttle = 0.7f; fly(2); C.throttle = 0.5f; fly(3); C.throttle = 0.3f; fly(2); C.throttle = 0.5f; fly(1); }   /* up, then level off */
      B.acc_bias[2] = biases[k]; fly(10);                     /* it appears, and the pilot flies on for 10 s */
      double z0 = B.p[2], zmax = z0, t0 = B.t; link_up = 0;
      double vmin = 0; while (F.state != FC_DISARMED && F.state != FC_CRASHED && B.t - t0 < 90) { fly(0.1); if (B.p[2] > zmax) zmax = B.p[2]; if (B.v[2] < vmin && B.t - t0 > 3) vmin = B.v[2]; }
      CHECK(F.state == FC_DISARMED && B.p[2] < 0.01 && zmax < z0 + (baro ? 1.5 : 6) && vmin > (baro ? -1.5 : -3), "%s barometer, bias %+.1f m/s²: from %.1f m (highest %.1f m, fastest descent %.1f m/s), down and disarmed in %.0f s: %s",
            baro ? "with a" : "no", biases[k], z0, zmax, -vmin, B.t - t0, F.why);
    } }

  /* no barometer, link lost while climbing fast: it must turn round, not keep climbing on the trim that held up against drag */
  start(quad, lq); baro = 0; fly(1); C.arm = 1; fly(0.2); C.throttle = 0.6f; fly(1); C.throttle = 0.5f; fly(2); C.throttle = 0.8f; fly(2); C.throttle = 0.5f; fly(3);
  { double z0 = B.p[2], v0 = B.v[2], zmax = z0, t0 = B.t; link_up = 0;
    while (F.state == FC_ARMED || F.state == FC_FAILSAFE) { fly(0.1); if (B.p[2] > zmax) zmax = B.p[2]; if (B.t - t0 > 150) break; }
    CHECK(F.state == FC_DISARMED && B.p[2] < 0.01 && zmax < z0 + 25, "no barometer, link lost climbing at %.1f m/s from %.0f m: highest %.0f m, down and disarmed in %.0f s (%s)", v0, z0, zmax, B.t - t0, F.why); }

  printf("gyro bias\n");
  start(quad, lq); B.gyro_bias[0] = 0.02; B.gyro_bias[2] = -0.02; B.acc_bias[2] = 0.15; fly(3); C.arm = 1; fly(0.2); C.throttle = 0.9f; fly(1.5); C.throttle = 0.5f; fly(10);
  CHECK(tilt_deg(&B) < 1.5 && fabs(B.v[2]) < 0.15, "flies level with biased sensors (%.2f°, %.2f m/s)", tilt_deg(&B), B.v[2]);

  printf("%s: %d failed\n", fails ? "FAILED" : "all passed", fails);
  return fails != 0;
}
