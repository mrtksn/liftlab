/* Tests for the navigation task (nav_core.c) flying the flight core (fc_core.c) as on the drone: the flight core at
 * 1 kHz, the navigation at 100 Hz, joined by a link that delays both ways (as the Pi's serial link does), on the
 * built-in flight program. The plant is test_fc.c's rigid body; GPS, barometer and optical flow are made from it
 * with noise and delay.
 *   cc -O2 -I.. -o test_nav test_nav.c nav_core.c fc_core.c ../rn_host.c ../rn.c ../rn_builtin.c -lm && ./test_nav testdata */
#include "fc_core.h"
#include "nav_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern const uint8_t *const rn_builtin_img;
extern const uint32_t rn_builtin_len;

static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

/* two boards, each with its own host (the Pi loads the same program as the ESP32) */
static float arenas_[2][3][65536], pools_[2][3][8192];
static int32_t codes_[2][3][32768];
static rn_host HF, HN;
static fc_state F; static nav_state N;

static uint8_t *read_file(const char *path, uint32_t *len) {
  FILE *f = fopen(path, "rb"); if (!f) { perror(path); exit(2); }
  static uint8_t buf[2][8192]; static int k = 0; uint8_t *b = buf[k++ & 1];
  *len = (uint32_t)fread(b, 1, 8192, f); fclose(f); return b;
}
static double gauss(void) { double u = (rand() + 1.0) / (RAND_MAX + 2.0), v = (rand() + 1.0) / (RAND_MAX + 2.0); return sqrt(-2 * log(u)) * cos(6.2831853 * v); }

/* ── the plant (test_fc.c's) ── */
typedef struct { double p[3], v[3], q[4], w[3], th[FC_MAX_JOINTS]; double t; double wind[3]; } body;
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
static double acc_b[3];
static void plant_step(body *B, const fc_airframe *A, const fc_out *o, double dt) {
  double a[6] = { 0 };
  for (int i = 0; i < A->n_motors; i++) {
    const fc_motor *M = &A->mot[i]; double ang[FC_MAX_CHAIN], b[FC_MAX_BASIS];
    for (int c = 0; c < M->n_chain; c++) ang[c] = B->th[M->chain[c]];
    basis(b, ang, M->n_chain);
    double u = o->motor[i], k = M->bend, T = 1.04 * ((1 - k) * u + k * u * u);   /* 4% stronger than the model thinks */
    for (int r = 0; r < 6; r++) { double s = 0; for (int n = 0; n < M->n_basis; n++) s += b[n] * M->cols[n][r]; a[r] += s * T; }
  }
  for (int j = 0; j < A->n_joints; j++) { double d = o->servo[j] - B->th[j], mx = A->jnt[j].rate * dt; B->th[j] += d > mx ? mx : d < -mx ? -mx : d; }
  double R[9]; qm(R, B->q);
  double aw[3]; for (int i = 0; i < 3; i++) aw[i] = R[3 * i] * a[0] + R[3 * i + 1] * a[1] + R[3 * i + 2] * a[2];
  aw[2] -= 9.81;
  for (int i = 0; i < 3; i++) aw[i] -= (i < 2 ? 0.4 : 0.3) * (B->v[i] - B->wind[i]);
  int ground = B->p[2] <= 0 && aw[2] * dt + B->v[2] <= 0;
  if (ground) { aw[2] += -(aw[2] * dt + B->v[2]) / dt; aw[0] = aw[1] = 0; B->v[0] = B->v[1] = 0; for (int k = 0; k < 3; k++) { B->w[k] = 0; a[3 + k] = 0; } }
  for (int i = 0; i < 3; i++) { B->v[i] += aw[i] * dt; B->p[i] += B->v[i] * dt; }
  if (B->p[2] < 0) B->p[2] = 0;
  for (int k = 0; k < 3; k++) B->w[k] += a[3 + k] * dt;
  double *q = B->q, w0 = B->w[0], w1 = B->w[1], w2 = B->w[2];
  double dq[4] = { -q[1] * w0 - q[2] * w1 - q[3] * w2, q[0] * w0 + q[2] * w2 - q[3] * w1, q[0] * w1 - q[1] * w2 + q[3] * w0, q[0] * w2 + q[1] * w1 - q[2] * w0 };
  double n = 0; for (int k = 0; k < 4; k++) { q[k] += 0.5 * dq[k] * dt; n += q[k] * q[k]; } n = sqrt(n); for (int k = 0; k < 4; k++) q[k] /= n;
  double sf[3] = { aw[0], aw[1], aw[2] + 9.81 }; qm(R, B->q);
  for (int i = 0; i < 3; i++) acc_b[i] = R[i] * sf[0] + R[3 + i] * sf[1] + R[6 + i] * sf[2];
  B->t += dt;
}

/* ── the link: frames arrive DELAY s after they're sent ── */
#define QN 64
typedef struct { double at; fc_cmd c; } cq_t;
typedef struct { double at; nav_in m; } tq_t;
static cq_t cq[QN]; static int cq_n; static tq_t tq[QN]; static int tq_n;
static double DELAY = 0.006;

static body B; static fc_out O; static nav_sp SP; static int use_fix = 1, use_flow = 0, link_on = 1;
static nav_in latest; static int have_latest;
/* GPS: 5 Hz, 150 ms late, 20 cm noise with a slow wander; barometer 25 Hz; flow 100 Hz with its rangefinder */
static double gps_w[3]; static struct { double at, p[3], v[3]; } gq[8]; static int gq_n;
static void fly(double seconds) {
  for (int n = 0; n < (int)(seconds * 1000 + 0.5); n++) {
    double t = B.t; long ms = lround(t * 1000);
    /* commands arriving at the flight core */
    while (cq_n && cq[0].at <= t) { fc_command(&F, &cq[0].c); memmove(cq, cq + 1, sizeof cq[0] * --cq_n); }
    fc_imu m; for (int k = 0; k < 3; k++) { m.gyro[k] = (float)(B.w[k] + 0.002 * gauss()); m.acc[k] = (float)(acc_b[k] + 0.03 * gauss()); }
    if (B.t == 0) { double R[9]; qm(R, B.q); for (int i = 0; i < 3; i++) m.acc[i] = (float)(R[6 + i] * 9.81); }
    m.have_gyro = 1; m.have_mag = 0; m.have_baro = ms % 40 == 0; m.baro_alt = (float)(B.p[2] + 50 + 0.15 * gauss());
    rn_host_tick(&HF, 0.001f);
    fc_step(&F, &m, 0.001f, 0, &O);
    plant_step(&B, &F.A, &O, 0.001);
    /* every 10 ms the flight core sends its telemetry to the Pi */
    if (ms % 10 == 0 && link_on) {
      nav_in T; memset(&T, 0, sizeof T);
      memcpy(T.q, F.q, sizeof T.q); memcpy(T.w, F.w, sizeof T.w); for (int k = 0; k < 3; k++) T.acc[k] = m.acc[k]; T.have_att = F.att_ok;
      T.have_baro = F.have_alt; T.baro_alt = F.alt_e; T.baro_age = 0;
      if (tq_n < QN) { tq[tq_n].at = t + DELAY; tq[tq_n].m = T; tq_n++; }
    }
    /* the GPS, wired to the Pi */
    if (ms % 200 == 0) {
      for (int k = 0; k < 3; k++) gps_w[k] += (-gps_w[k] * 0.2 + 0.3 * gauss()) * 0.2;
      if (gq_n < 8) { gq[gq_n].at = t + 0.15; for (int k = 0; k < 3; k++) { gq[gq_n].p[k] = B.p[k] + 0.2 * gauss() + gps_w[k]; gq[gq_n].v[k] = B.v[k] + 0.1 * gauss(); } gq_n++; }
    }
    /* the Pi: navigation at 100 Hz on the newest telemetry */
    while (tq_n && tq[0].at <= t) { latest = tq[0].m; have_latest = 1; memmove(tq, tq + 1, sizeof tq[0] * --tq_n); }
    static double fix_t = -1, fix_p[3], fix_v[3], fix_meas_t;
    while (gq_n && gq[0].at <= t) { fix_t = t; fix_meas_t = gq[0].at - 0.15; memcpy(fix_p, gq[0].p, sizeof fix_p); memcpy(fix_v, gq[0].v, sizeof fix_v); memmove(gq, gq + 1, sizeof gq[0] * --gq_n); }
    if (ms % 10 == 5 && have_latest) {
      nav_in in = latest; nav_out o;
      in.baro_age = (float)DELAY;
      if (use_fix && fix_t >= 0 && t - fix_t < 0.3) { in.have_fix = 1; in.fix_age = (float)(t - fix_meas_t); for (int k = 0; k < 3; k++) { in.fix_p[k] = (float)fix_p[k]; in.fix_v[k] = (float)fix_v[k]; } }
      if (use_flow) {   /* a downward camera on the Pi: flow and range now (and the gyro it removes is the telemetry's) */
        double R[9]; qm(R, B.q); double range = B.p[2] + 0.05; range /= R[8] > 0.3 ? R[8] : 0.3;
        double vs[3]; for (int i = 0; i < 3; i++) vs[i] = R[i] * B.v[0] + R[3 + i] * B.v[1] + R[6 + i] * B.v[2];
        in.have_flow = 1; in.range = (float)(range + 0.01 * gauss()); in.flow_q = 1; in.flow_age = 0.02f;
        in.flow[0] = (float)(B.w[1] - vs[0] / range + 0.05 * gauss()); in.flow[1] = (float)(-B.w[0] - vs[1] / range + 0.05 * gauss());
      }
      nav_step(&N, &in, &SP, 0.01f, &o);
      fc_cmd c; memset(&c, 0, sizeof c); c.arm = 1; c.test_motor = -1;
      c.guided = 1; memcpy(c.acc, o.acc, sizeof c.acc); c.heading = o.heading; c.throttle = o.fly ? 1 : 0;
      if (link_on && cq_n < QN) { cq[cq_n].at = t + DELAY; cq[cq_n].c = c; cq_n++; }
    }
  }
}
static uint8_t cfg_blob[8 + 21 * 4 + 4]; static int refs = 3;
static void make_config(float m) {
  float f[21] = { m, 0, 0, 0, 0, 0, 0.05f, 0, 0, -0.02f, 1, 0, 0, 0, 1, 0, 0, 0, 1, 3, (float)refs };
  uint32_t magic = 0x434E4644u, ver = 1; memcpy(cfg_blob, &magic, 4); memcpy(cfg_blob + 4, &ver, 4); memcpy(cfg_blob + 8, f, sizeof f);
  uint32_t crc = rn_crc32(cfg_blob, 8 + sizeof f); memcpy(cfg_blob + 8 + sizeof f, &crc, 4);
}
static void start(const uint8_t *blob, uint32_t len, int flow) {
  static fc_state zf; static nav_state zn; F = zf; N = zn;
  for (int b = 0; b < 2; b++) {
    float *arenas[3] = { arenas_[b][0], arenas_[b][1], arenas_[b][2] }, *pools[3] = { pools_[b][0], pools_[b][1], pools_[b][2] };
    int32_t *codes[3] = { codes_[b][0], codes_[b][1], codes_[b][2] };
    rn_host *H = b ? &HN : &HF; memset(H, 0, sizeof *H);
    if (rn_host_init(H, rn_builtin_img, rn_builtin_len, arenas, 65536, codes, 32768, pools, 8192)) { printf("host init failed\n"); exit(1); }
  }
  if (fc_init(&F, &HF) || fc_airframe_load(&F, blob, len)) { printf("fc: %s\n", F.why); exit(1); }
  refs = flow ? 5 : 3; make_config(F.A.m);
  if (nav_init(&N, &HN) || nav_config_load(&N, cfg_blob, sizeof cfg_blob)) { printf("nav: %s\n", N.why); exit(1); }
  memset(&B, 0, sizeof B); B.q[0] = 1; cq_n = tq_n = gq_n = 0; have_latest = 0; memset(gps_w, 0, sizeof gps_w);
  memset(&SP, 0, sizeof SP); use_fix = !flow; use_flow = flow; link_on = 1; srand(7);
}
static double herr(void) { return hypot(B.p[0] - SP.target[0], B.p[1] - SP.target[1]); }
/* arm on the ground: the arm switch off then on, with the motors idle */
static void arm(void) {
  fc_cmd c; memset(&c, 0, sizeof c); c.test_motor = -1; fc_command(&F, &c); fly(1.0);
  c.arm = 1; fc_command(&F, &c);
}

int main(int argc, char **argv) {
  const char *dir = argc > 1 ? argv[1] : "testdata"; char path[512]; uint32_t lq, lt;
  snprintf(path, sizeof path, "%s/quadx.dfa", dir); uint8_t *quad = read_file(path, &lq);
  snprintf(path, sizeof path, "%s/tiltquad.dfa", dir); uint8_t *tilt = read_file(path, &lt);

  printf("nav config\n");
  start(quad, lq, 0);
  { nav_state G = N; uint8_t bad[sizeof cfg_blob]; memcpy(bad, cfg_blob, sizeof bad); bad[12] ^= 1; CHECK(nav_config_load(&G, bad, sizeof bad) < 0, "a flipped bit is refused: %s", G.why); }
  { nav_state G = N; CHECK(nav_config_load(&G, cfg_blob, 20) < 0, "a short file is refused: %s", G.why); }

  printf("GPS + barometer, the Pi 6 ms away each way\n");
  start(quad, lq, 0); arm(); CHECK(F.state == FC_ARMED, "armed on the ground: %s", F.why);
  fly(0.5); CHECK(B.p[2] < 0.01, "stays on the ground until told to fly (z %.3f)", B.p[2]);
  SP.target[2] = 1.5f; SP.fly = 1; fly(8);
  CHECK(fabs(B.p[2] - 1.5) < 0.25 && herr() < 0.6, "takes off to 1.5 m and holds (z %.2f, %.2f m off)", B.p[2], herr());
  double worst = 0; for (int k = 0; k < 20; k++) { fly(0.5); if (herr() > worst) worst = herr(); }
  CHECK(worst < 0.8, "holds its position for 10 s within %.2f m (GPS noise 20 cm, wander)", worst);
  SP.target[0] = 4; SP.target[1] = -3; SP.target[2] = 2.5f; fly(10);
  CHECK(herr() < 0.6 && fabs(B.p[2] - 2.5) < 0.3, "flies to (4, -3, 2.5): %.2f m off, z %.2f", herr(), B.p[2]);
  SP.heading = 1.2f; fly(4);
  { double R[9]; qm(R, B.q); double yaw = atan2(R[3], R[0]); CHECK(fabs(yaw - 1.2) < 0.1, "turns to its heading (%.2f rad)", yaw); }
  B.wind[0] = 3; fly(12);
  CHECK(herr() < 1.0, "holds against a 3 m/s wind: %.2f m off", herr());
  B.wind[0] = 0;
  for (int k = 0; k < 3; k++) SP.vref[k] = 0;
  SP.vref[0] = 2; for (int k = 0; k < 150; k++) { SP.target[0] += 0.04f; fly(0.02); }
  CHECK(B.v[0] > 1.5, "follows a moving target at 2 m/s (%.2f m/s)", B.v[0]);
  SP.vref[0] = 0; fly(5);
  printf("link lost\n");
  link_on = 0; cq_n = 0; fly(1);
  CHECK(F.state == FC_FAILSAFE, "the flight core goes to its failsafe: %s", F.why);
  fly(8); CHECK(F.state == FC_DISARMED && B.p[2] < 0.05, "and lands (%s, z %.2f)", F.why, B.p[2]);

  printf("optical flow + rangefinder (indoors, no GPS)\n");
  start(quad, lq, 1); arm();
  SP.target[2] = 1.2f; SP.fly = 1; fly(8);
  CHECK(fabs(B.p[2] - 1.2) < 0.25 && herr() < 0.5, "takes off and holds (z %.2f, %.2f m off)", B.p[2], herr());
  SP.target[0] = 2; fly(8);
  CHECK(herr() < 0.5, "moves 2 m on flow alone: %.2f m off", herr());

  printf("tilt-rotor quad\n");
  /* (it gets there in 2 s, then wanders around the target by up to about 1.5 m on GPS: judged on the mean distance) */
  start(tilt, lt, 0); arm(); SP.target[2] = 1.5f; SP.fly = 1; fly(8); SP.target[0] = 3; fly(2);
  { double sum = 0; for (int q = 0; q < 16; q++) { fly(0.5); sum += herr(); }
    CHECK(sum / 16 < 0.9 && fabs(B.p[2] - 1.5) < 0.3, "takes off and flies 3 m: on average %.2f m off over the next 8 s, z %.2f", sum / 16, B.p[2]); }

  printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
  return fails ? 1 : 0;
}
