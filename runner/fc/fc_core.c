/* Drone Force Bench flight controller: see fc_core.h. */
#include "fc_core.h"
#include <stdarg.h>
#if defined(__wasm__)                 /* the simulator's build: no C library, the page's Math */
typedef __SIZE_TYPE__ size_t;
void *memcpy(void *d, const void *s, size_t n); void *memset(void *d, int c, size_t n);
#define FC_IMPORT(n) __attribute__((import_module("env"), import_name(n)))
FC_IMPORT("sin") double fc_js_sin(double); FC_IMPORT("cos") double fc_js_cos(double); FC_IMPORT("tan") double fc_js_tan(double);
FC_IMPORT("acos") double fc_js_acos(double); FC_IMPORT("atan2") double fc_js_atan2(double, double);
#define sinf(x) ((float)fc_js_sin(x))
#define cosf(x) ((float)fc_js_cos(x))
#define tanf(x) ((float)fc_js_tan(x))
#define acosf(x) ((float)fc_js_acos(x))
#define atan2f(y, x) ((float)fc_js_atan2(y, x))
#define sqrtf(x) __builtin_sqrtf(x)
static inline float fminf(float a, float b) { return a < b ? a : b; }
static inline float fmaxf(float a, float b) { return a > b ? a : b; }
#else
#include <math.h>
#include <string.h>
#endif

/* A small formatter for the status line (%s, %d, %.Nf), so the flight code needs no printf. */
static void fc_say(fc_state *F, const char *fmt, ...) {
  char *o = F->why; int n = 0, cap = (int)sizeof F->why - 1; va_list ap; va_start(ap, fmt);
  for (const char *p = fmt; *p && n < cap; p++) {
    if (*p != '%') { o[n++] = *p; continue; }
    int prec = -1; p++;
    if (*p == '.') { prec = p[1] - '0'; p += 2; }
    if (*p == 's') { const char *s = va_arg(ap, const char *); while (*s && n < cap) o[n++] = *s++; }
    else if (*p == 'd' || *p == 'f') {
      double v = *p == 'd' ? (double)va_arg(ap, int) : va_arg(ap, double); if (prec < 0) prec = *p == 'd' ? 0 : 2;
      if (v < 0) { if (n < cap) o[n++] = '-'; v = -v; }
      for (int k = 0; k < prec; k++) v *= 10;
      unsigned long long x = (unsigned long long)(v + 0.5); char d[24]; int m = 0;
      do { d[m++] = (char)('0' + x % 10); x /= 10; } while (x || m <= prec);
      while (m > 0 && n < cap) { if (m == prec && prec > 0) o[n++] = '.'; if (n < cap) o[n++] = d[--m]; }
    } else if (n < cap) o[n++] = *p;
    if (!*p) break;
  }
  o[n] = 0; va_end(ap);
}

#define G_ 9.81f
#define FC_RN_IN 24                 /* actuator inputs the formulas' lists hold (RN_IN in js/rn-sigs.js) */
#define FC_AZ_MAX 5.0f              /* throttle stick: ±this vertical acceleration around hover [m/s²] */
#define FC_YAW_RATE 2.0f
#define FC_K_AZ 1.0f                /* how fast the thrust trim follows the measured vertical acceleration [1/s] */
#define FC_FS_DESCENT 1.0f          /* failsafe descent speed [m/s] */
#define FC_VZ_LEAK 20.0f            /* the accelerometer's vertical speed forgets over this long [s] */
#define FC_AZ_TRIM 3.0f
#define FC_K_ALT 2.0f                /* barometer filter: height and speed corrections [1/s], [1/s²] */
#define FC_K_VZ 1.0f
#define FC_VZ_MAX 2.0f               /* throttle stick with a barometer: at most this climb or sink speed [m/s] */
#define FC_K_V 1.5f                  /* vertical speed → acceleration [1/s] */
#define FC_K_HOLD 1.0f               /* height hold: height error → speed [1/s] */             /* at most this much trim [m/s²] */            /* yaw stick: at most this turn rate [rad/s] */
#define FC_AIRFRAME_MAGIC 0x41424644u   /* 'DFBA' */

const char *fc_state_name(int s) {
  switch (s) { case FC_DISARMED: return "disarmed"; case FC_ARMED: return "armed"; case FC_FAILSAFE: return "failsafe";
    case FC_CRASHED: return "crashed"; case FC_TESTING: return "motor test"; }
  return "?";
}

/* ── small math ── */
static float fabsf_(float x) { return x < 0 ? -x : x; }
static float clampf(float x, float a, float b) { return x < a ? a : x > b ? b : x; }
static void m3v(float *o, const float *M, const float *v) { float x = M[0] * v[0] + M[1] * v[1] + M[2] * v[2], y = M[3] * v[0] + M[4] * v[1] + M[5] * v[2], z = M[6] * v[0] + M[7] * v[1] + M[8] * v[2]; o[0] = x; o[1] = y; o[2] = z; }
static void m3tv(float *o, const float *M, const float *v) { float x = M[0] * v[0] + M[3] * v[1] + M[6] * v[2], y = M[1] * v[0] + M[4] * v[1] + M[7] * v[2], z = M[2] * v[0] + M[5] * v[1] + M[8] * v[2]; o[0] = x; o[1] = y; o[2] = z; }
static void m3mt(float *o, const float *A, const float *B) {   /* A · Bᵀ */
  float t[9]; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) t[3 * i + j] = A[3 * i] * B[3 * j] + A[3 * i + 1] * B[3 * j + 1] + A[3 * i + 2] * B[3 * j + 2];
  memcpy(o, t, sizeof t);
}
static void qmat(float *R, const float *q) {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - w * z); R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z); R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y); R[7] = 2 * (y * z + w * x); R[8] = 1 - 2 * (x * x + y * y);
}
/* math.js frameFrom: axes (h, n × h, n) as columns, h = xref made perpendicular to n */
static void frame_from(float *M, const float *n, const float *xref) {
  float d = xref[0] * n[0] + xref[1] * n[1] + xref[2] * n[2], h[3] = { xref[0] - n[0] * d, xref[1] - n[1] * d, xref[2] - n[2] * d };
  float l = sqrtf(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]);
  if (l < 1e-6f) { d = n[1]; h[0] = -n[0] * d; h[1] = 1 - n[1] * d; h[2] = -n[2] * d; l = sqrtf(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]); }
  h[0] /= l; h[1] /= l; h[2] /= l;
  float k[3] = { n[1] * h[2] - n[2] * h[1], n[2] * h[0] - n[0] * h[2], n[0] * h[1] - n[1] * h[0] };
  M[0] = h[0]; M[1] = k[0]; M[2] = n[0]; M[3] = h[1]; M[4] = k[1]; M[5] = n[1]; M[6] = h[2]; M[7] = k[2]; M[8] = n[2];
}
/* learn.js basisVals / dBasisVals: products of (1, cos θ, sin θ) over a motor's joints, first joint most significant */
static int basis(float *v, const float *ang, int k, int dm) {
  int n = 1; v[0] = 1;
  for (int i = 0; i < k; i++) {
    float f[3] = { 1, cosf(ang[i]), sinf(ang[i]) };
    if (i == dm) { f[0] = 0; f[1] = -sinf(ang[i]); f[2] = cosf(ang[i]); }
    for (int a = n - 1; a >= 0; a--) { float x = v[a]; v[3 * a] = x * f[0]; v[3 * a + 1] = x * f[1]; v[3 * a + 2] = x * f[2]; }
    n *= 3;
  }
  return n;
}
static void col_at(const fc_state *F, int i, int dm, float *out) {   /* motor i's effect now (dm ≥ 0: its change as chain joint dm turns) */
  const fc_motor *M = &F->A.mot[i]; float ang[FC_MAX_CHAIN], b[FC_MAX_BASIS];
  for (int c = 0; c < M->n_chain; c++) ang[c] = F->th_hat[M->chain[c]];
  int n = basis(b, ang, M->n_chain, dm);
  for (int r = 0; r < 6; r++) { float s = 0; for (int k = 0; k < n && k < M->n_basis; k++) s += b[k] * M->cols[k][r]; out[r] = s; }
}

/* ── the airframe ── */
typedef struct { const uint8_t *p; uint32_t n, at; int bad; } rd;
static uint32_t rd_u(rd *r) { if (r->at + 4 > r->n) { r->bad = 1; return 0; } uint32_t v; memcpy(&v, r->p + r->at, 4); r->at += 4; return v; }
static float rd_f(rd *r) { uint32_t u = rd_u(r); float f; memcpy(&f, &u, 4); return f; }
static void rd_fs(rd *r, float *o, int n) { for (int i = 0; i < n; i++) o[i] = rd_f(r); }
static void rd_name(rd *r, char *o) { if (r->at + 16 > r->n) { r->bad = 1; return; } memcpy(o, r->p + r->at, 16); o[15] = 0; r->at += 16; }

int fc_airframe_load(fc_state *F, const uint8_t *blob, uint32_t len) {
  fc_airframe A; memset(&A, 0, sizeof A);
  if (len < 24) { fc_say(F, "airframe: too short"); return -1; }
  uint32_t crc; memcpy(&crc, blob + len - 4, 4);
  if (rn_crc32(blob, len - 4) != crc) { fc_say(F, "airframe: checksum mismatch"); return -1; }
  rd r = { blob, len - 4, 0, 0 };
  if (rd_u(&r) != FC_AIRFRAME_MAGIC || rd_u(&r) != 1) { fc_say(F, "airframe: not an airframe file"); return -1; }
  A.n_motors = (int)rd_u(&r); A.n_joints = (int)rd_u(&r); A.mode = (int)rd_u(&r);
  if (A.n_motors < 1 || A.n_motors > FC_MAX_MOTORS || A.n_joints < 0 || A.n_joints > FC_MAX_JOINTS || A.mode < 0 || A.mode > 2) { fc_say(F, "airframe: %d motors, %d joints: too many for this firmware", A.n_motors, A.n_joints); return -1; }
  A.m = rd_f(&r); rd_fs(&r, A.J, 9); rd_fs(&r, A.Jinv, 9); rd_fs(&r, A.axis, 3); rd_fs(&r, A.imu_R, 9);
  A.allowance = rd_f(&r); A.efficiency = rd_f(&r); A.servo_move = rd_f(&r); A.horizon = rd_f(&r); A.lean_max = rd_f(&r); A.mix_share = rd_f(&r);
  for (int j = 0; j < A.n_joints; j++) {
    fc_joint *J = &A.jnt[j]; rd_name(&r, J->name); J->steer = (int)rd_u(&r);
    J->manual = rd_f(&r); J->range = rd_f(&r); J->rate = rd_f(&r); J->lag = rd_f(&r);
  }
  for (int i = 0; i < A.n_motors; i++) {
    fc_motor *M = &A.mot[i]; rd_name(&r, M->name);
    M->n_chain = (int)rd_u(&r); for (int c = 0; c < FC_MAX_CHAIN; c++) M->chain[c] = (int)rd_u(&r);
    M->n_basis = (int)rd_u(&r);
    if (M->n_chain < 0 || M->n_chain > FC_MAX_CHAIN || M->n_basis < 1 || M->n_basis > FC_MAX_BASIS) { fc_say(F, "airframe: motor %d rides on too many joints", i + 1); return -1; }
    for (int c = 0; c < M->n_chain; c++) if (M->chain[c] < 0 || M->chain[c] >= A.n_joints) { fc_say(F, "airframe: bad joint index"); return -1; }
    for (int k = 0; k < M->n_basis; k++) rd_fs(&r, M->cols[k], 6);
    M->bend = rd_f(&r); M->lag = rd_f(&r); M->power = rd_f(&r);
  }
  if (r.bad || r.at != r.n || !(A.m > 0)) { fc_say(F, "airframe: wrong size or values"); return -1; }
  int inputs = A.n_motors + A.n_joints;
  if (inputs > FC_RN_IN) { fc_say(F, "airframe: %d inputs, the formulas hold %d", inputs, FC_RN_IN); return -1; }
  F->A = A; F->have_airframe = 1; F->state = FC_DISARMED;
  for (int j = 0; j < A.n_joints; j++) F->th_cmd[j] = F->th_hat[j] = A.jnt[j].manual;
  fc_say(F, "airframe: %d motors, %d servo joints, %.2f kg", A.n_motors, A.n_joints, (double)A.m);
  return 0;
}

/* ── calling the formulas: inputs packed flat, in each formula's signature order ── */
typedef struct { float b[512]; int n; } pk;
static void p_f(pk *p, float v) { p->b[p->n++] = v; }
static void p_v(pk *p, const float *v, int k) { for (int i = 0; i < k; i++) p->b[p->n++] = v[i]; }
static void p_list(pk *p, const float *v, int len, int stride) {   /* a list: its length, then FC_RN_IN places */
  p->b[p->n++] = (float)len;
  for (int i = 0; i < FC_RN_IN * stride; i++) p->b[p->n++] = i < len * stride ? v[i] : 0;
}
static int call(fc_state *F, int fn, int inst, pk *in, float *out) {
  int e = rn_host_call(F->H, fn, inst, in->b, out);
  if (e) F->trap = e;
  return e;
}

int fc_init(fc_state *F, rn_host *H) {
  F->H = H;
  const char *names[] = { "attitudeEstimator", "servoPredictor", "thrustAxisTarget", "attitudeError", "attitudeControl", "forceDemand", "allocationPreferences", "allocation", "thrustLinearization", "voltageCompensation" };
  int *slot[] = { &F->f_att, &F->f_srv, &F->f_ta, &F->f_err, &F->f_ctl, &F->f_fd, &F->f_pref, &F->f_alloc, &F->f_lin, &F->f_vc };
  /* what this code passes and expects back, in floats (see js/rn-sigs.js) */
  const int in_sz[] = { 11, 4, 6, 18, 18, 7, 1 + FC_RN_IN * 14 + 3, (1 + FC_RN_IN * 6) + 2 * (1 + FC_RN_IN) + 6 + 1 + (1 + 2 * (1 + FC_RN_IN)), 2, 3 };
  const int out_sz[] = { 7, 1, 3, 3, 3, 3, 2 * (1 + FC_RN_IN), 1 + FC_RN_IN, 1, 1 };
  F->sizes_ok = 1;
  /* a memory per servo, for as many as the firmware takes, so any airframe can be loaded later */
  if (rn_host_instances(H, "servoPredictor", FC_MAX_JOINTS)) { F->sizes_ok = 0; fc_say(F, "can't give servoPredictor %d memories", FC_MAX_JOINTS); return -1; }
  for (int i = 0; i < 10; i++) {
    *slot[i] = rn_host_find(H, names[i]);
    if (*slot[i] < 0 || rn_host_in_size(H, *slot[i]) != in_sz[i] || rn_host_out_size(H, *slot[i]) != out_sz[i]) {
      F->sizes_ok = 0; fc_say(F, "formula %s isn't what this firmware expects", names[i]); return -1;
    }
  }
  F->vref = 16.0f; F->rho = 1;
  F->q[0] = 1; qmat(F->R, F->q);
  return 0;
}

/* how far the nominal thrust axis leans from straight up [deg] */
static float axis_tilt(const fc_state *F) {
  const float *a = F->A.axis, *R = F->R;
  return acosf(clampf(R[6] * a[0] + R[7] * a[1] + R[8] * a[2], -1, 1)) * 57.2958f;
}

void fc_command(fc_state *F, const fc_cmd *c) {
  F->cmd = *c; F->cmd_t = F->t;
  if (!c->arm) { if (F->state != FC_DISARMED) fc_say(F, "disarmed"); F->state = FC_DISARMED; }
  if (c->arm && (F->state == FC_DISARMED || F->state == FC_TESTING)) {
    float tilt = axis_tilt(F);
    const char *no = !F->have_airframe ? "no airframe loaded" : !F->sizes_ok ? "the formulas don't match this firmware" : !F->att_ok ? "no gyro, or the attitude isn't settled yet" :
      tilt > 15 ? "not level" : c->throttle > 0.1f ? "throttle stick not at the bottom" : 0;
    if (no) fc_say(F, "won't arm: %s", no);
    else {
      F->state = FC_ARMED; fc_say(F, "armed");
      F->yaw_sp = atan2f(F->R[3], F->R[0]); memset(F->iAtt, 0, sizeof F->iAtt);
    }
  }
  if (c->arm && F->state == FC_FAILSAFE) { F->state = FC_ARMED; fc_say(F, "commands back: armed"); }
  if (!c->arm && c->test_motor >= 0 && c->test_motor < F->A.n_motors && F->have_airframe) { F->state = FC_TESTING; F->test_t = F->t; fc_say(F, "motor test: %s", F->A.mot[c->test_motor].name); }
}

/* Allocation, as the simulator's allocate(): the servos' angle changes, then every motor's thrust. */
static int allocate(fc_state *F, const float *wa, float *u) {
  const fc_airframe *A = &F->A; float out[64]; pk p;
  static float cols[FC_RN_IN * 6], lo[FC_RN_IN], hi[FC_RN_IN], inp[FC_RN_IN * 14];
  int nm = A->n_motors;
  for (int stage = 0; stage < 2; stage++) {
    int ns = 0, sj[FC_MAX_JOINTS];
    if (stage == 0) { for (int j = 0; j < A->n_joints; j++) if (A->jnt[j].steer) sj[ns++] = j; if (!ns) continue; }
    int n = 0; float amax = 1e-9f;
    for (int i = 0; i < nm; i++, n++) {
      col_at(F, i, -1, cols + 6 * n); lo[n] = 0; hi[n] = 1;
      float *q = inp + 14 * n; memset(q, 0, 14 * sizeof(float));
      q[0] = 0; q[1] = F->v[i]; q[2] = 0; q[3] = 1; q[6] = 1; q[7] = A->mot[i].power;   /* kind thrust, x, lo, hi, (authority), power */
    }
    for (int s = 0; s < ns; s++, n++) {
      const fc_joint *J = &A->jnt[sj[s]]; float th = F->th_hat[sj[s]], reach = J->rate * fmaxf(0.005f, A->horizon - J->lag);
      float d[6] = { 0 };
      for (int i = 0; i < nm; i++) {
        const fc_motor *M = &A->mot[i]; int m = -1; for (int c = 0; c < M->n_chain; c++) if (M->chain[c] == sj[s]) m = c;
        if (m < 0) continue;
        float dc[6]; col_at(F, i, m, dc); float w = fmaxf(F->v[i], 0.02f);
        for (int r = 0; r < 6; r++) d[r] += dc[r] * w;
      }
      memcpy(cols + 6 * n, d, sizeof d);
      lo[n] = fminf(0, fmaxf(-J->range - th, -reach)); hi[n] = fmaxf(0, fminf(J->range - th, reach));
      float *q = inp + 14 * n; memset(q, 0, 14 * sizeof(float));
      q[0] = 1; q[1] = 0; q[2] = lo[n]; q[3] = hi[n]; q[6] = 0; q[8] = 1; q[9] = th; q[10] = 1; q[11] = J->range; q[12] = 1; q[13] = reach;   /* kind servo, x, lo, hi, (authority), power absent, th, range, reach */
    }
    for (int k = 0; k < n; k++) { float a = sqrtf(cols[6 * k + 3] * cols[6 * k + 3] + cols[6 * k + 4] * cols[6 * k + 4] + cols[6 * k + 5] * cols[6 * k + 5]) * (hi[k] - lo[k]); if (a > amax) amax = a; }
    for (int k = 0; k < n; k++) { float *q = inp + 14 * k; q[4] = 1; q[5] = sqrtf(cols[6 * k + 3] * cols[6 * k + 3] + cols[6 * k + 4] * cols[6 * k + 4] + cols[6 * k + 5] * cols[6 * k + 5]) * (hi[k] - lo[k]) / amax; }
    p.n = 0; p_list(&p, inp, n, 14); p_f(&p, A->allowance); p_f(&p, A->efficiency); p_f(&p, A->servo_move);
    float pull[2 * (1 + FC_RN_IN)];
    if (call(F, F->f_pref, 0, &p, pull)) return -1;
    p.n = 0; p_list(&p, cols, n, 6); p_list(&p, lo, n, 1); p_list(&p, hi, n, 1); p_v(&p, wa, 6); p_f(&p, (float)A->mode);
    p_f(&p, 1); p_v(&p, pull, 2 * (1 + FC_RN_IN));
    if (call(F, F->f_alloc, 0, &p, out)) return -1;
    if (stage == 0) for (int s = 0; s < ns; s++) { const fc_joint *J = &A->jnt[sj[s]]; F->th_cmd[sj[s]] = clampf(F->th_hat[sj[s]] + out[1 + nm + s], -J->range, J->range); }
    else for (int i = 0; i < nm; i++) u[i] = out[1 + i];
  }
  return 0;
}

void fc_step(fc_state *F, const fc_imu *imu, float dt, float vbatt, fc_out *o) {
  const fc_airframe *A = &F->A; pk p; float r[8];
  F->t += dt; F->steps++; F->vbatt = vbatt;
  memset(o, 0, sizeof *o);
  for (int j = 0; j < A->n_joints; j++) o->servo[j] = F->th_cmd[j];
  if (!F->have_airframe || !F->sizes_ok) return;

  /* attitude, from the IMU in body axes */
  if (imu->have_gyro) {
    float g[3], a[3]; m3v(g, A->imu_R, imu->gyro); m3v(a, A->imu_R, imu->acc);
    p.n = 0; p_v(&p, g, 3); p_v(&p, a, 3); p_f(&p, 0); p_v(&p, (float[3]){ 0, 0, 0 }, 3); p_f(&p, dt);
    if (call(F, F->f_att, 0, &p, r)) { F->state = FC_DISARMED; fc_say(F, "the attitude estimator failed"); return; }
    float n = sqrtf(r[0] * r[0] + r[1] * r[1] + r[2] * r[2] + r[3] * r[3]);
    for (int k = 0; k < 4; k++) F->q[k] = r[k] / n;
    memcpy(F->w, r + 4, sizeof F->w); qmat(F->R, F->q);
    F->att_t += dt; F->att_ok = F->att_t > 0.5f; F->imu_gap = 0;
    /* vertical acceleration in the world, from the accelerometer (it reads thrust, not gravity), low-passed */
    float azm = F->R[6] * a[0] + F->R[7] * a[1] + F->R[8] * a[2] - G_;
    /* on the ground (disarmed, or armed at idle) and still, it should read 0: what it reads is its bias */
    int still = g[0] * g[0] + g[1] * g[1] + g[2] * g[2] < 0.01f && (F->state == FC_DISARMED || (F->state == FC_ARMED && F->cmd.throttle < 0.05f));
    if (still && F->att_ok) F->az_bias += (azm - F->az_bias) * fminf(1, dt / 1.0f);
    F->az_f += (azm - F->az_bias - F->az_f) * fminf(1, dt * 31.4f);
  } else {
    F->att_ok = 0; F->att_t = 0; F->imu_gap += dt;
    if ((F->state == FC_ARMED || F->state == FC_FAILSAFE) && F->imu_gap > 0.2f) { F->state = FC_CRASHED; fc_say(F, "IMU lost in flight: motors off"); }
  }

  /* height and vertical speed: the barometer and the accelerometer (a complementary filter); without a
   * barometer, the vertical speed is the accelerometer's alone, leaking toward 0 */
  if (imu->have_baro) {
    if (!F->have_alt) { F->alt_e = imu->baro_alt; F->vz_e = F->vz_i; F->have_alt = 1; }
    float e = imu->baro_alt - F->alt_e;
    F->alt_e += (F->vz_e + FC_K_ALT * e) * dt; F->vz_e += (F->az_f + FC_K_VZ * e) * dt;
    F->baro_gap = 0;
  } else if ((F->baro_gap += dt) > 0.5f) F->have_alt = 0;

  /* servos: where they are believed to be */
  for (int j = 0; j < A->n_joints; j++) {
    const fc_joint *J = &A->jnt[j]; float tgt = J->steer ? F->th_cmd[j] : J->manual, th;
    p.n = 0; p_f(&p, tgt); p_f(&p, J->rate); p_f(&p, J->lag); p_f(&p, dt);
    if (!call(F, F->f_srv, j, &p, &th)) F->th_hat[j] = th;
    o->servo[j] = tgt;
  }

  /* safety */
  float tilt = axis_tilt(F);
  if (F->state == FC_TESTING) {
    if (F->t - F->test_t > FC_TEST_S || F->cmd.test_motor < 0) { F->state = FC_DISARMED; return; }
    o->motor[F->cmd.test_motor] = clampf(F->cmd.test_throttle, 0, 0.3f); return;
  }
  if (F->state == FC_ARMED && tilt > FC_CRASH_DEG) { F->state = FC_CRASHED; fc_say(F, "tilted %.0f°: crashed, motors off", (double)tilt); }
  if (F->state == FC_ARMED && F->t - F->cmd_t > FC_CMD_TIMEOUT) { F->state = FC_FAILSAFE; F->fs_t = F->t; F->fs_vz = F->fs_vmin = 0; fc_say(F, "no commands: failsafe descent"); }
  if (F->state == FC_FAILSAFE) {        /* landed: the ground stopped the descent */
    F->fs_vz += F->az_f * dt; if (F->fs_vz < F->fs_vmin) F->fs_vmin = F->fs_vz;
    if (F->t - F->fs_t > 1.5f && F->fs_vz - F->fs_vmin > 0.5f) { F->state = FC_DISARMED; fc_say(F, "failsafe: landed, disarmed"); }
    else if (F->t - F->fs_t > FC_FAILSAFE_S) { F->state = FC_DISARMED; fc_say(F, "failsafe: time's up, disarmed"); }
  }
  if (F->state != FC_ARMED && F->state != FC_FAILSAFE) return;

  /* what the pilot asks for: lean angles, turn rate, and with the throttle stick around its middle (0.5):
   * with a barometer, climb or sink speed (in the middle it holds the height); without, vertical acceleration */
  fc_cmd c = F->cmd;
  float vz = F->have_alt ? F->vz_e : F->vz_i, az;
  if (F->have_alt) {
    float s = c.throttle - 0.5f, vz_cmd;
    if (fabsf_(s) < 0.05f) { if (!F->holding) { F->alt_hold = F->alt_e; F->holding = 1; } vz_cmd = clampf(FC_K_HOLD * (F->alt_hold - F->alt_e), -1, 1); }
    else { F->holding = 0; vz_cmd = (s - (s > 0 ? 0.05f : -0.05f)) / 0.45f * FC_VZ_MAX; }
    az = clampf(FC_K_V * (vz_cmd - vz), -FC_AZ_MAX, FC_AZ_MAX);
  } else { az = (c.throttle - 0.5f) * 2 * FC_AZ_MAX; F->holding = 0; }
  if (F->state == FC_FAILSAFE) { c.roll = c.pitch = c.yaw = 0; az = clampf(FC_K_V * (-FC_FS_DESCENT - vz), -2, 1.5f); }   /* descend at about 1 m/s */
  else if (c.throttle < 0.05f) {                            /* stick at the bottom: idle, nothing to steer with */
    for (int i = 0; i < A->n_motors; i++) { o->motor[i] = FC_IDLE; F->v[i] = 0; }
    memset(F->iAtt, 0, sizeof F->iAtt); F->iAz = 0; F->vz_i = 0; F->holding = 0; F->yaw_sp = atan2f(F->R[3], F->R[0]);
    return;
  }
  const float lean = (A->lean_max > 0 ? A->lean_max : 30) * 0.0174533f;
  /* the thrust for that acceleration: the model's, trimmed by what the accelerometer measures (the model's hover
   * thrust is never exactly right, and without this the drone would drift up or down at "hover") */
  F->vz_i += (F->az_f - F->vz_i / FC_VZ_LEAK) * dt;   /* vertical speed since take-off, from the accelerometer alone: drifts, so it leaks */
  F->iAz = clampf(F->iAz + FC_K_AZ * (az - F->az_f) * dt, -FC_AZ_TRIM, FC_AZ_TRIM);
  float lift = G_ + az + F->iAz;
  float fwd = tanf(clampf(c.pitch, -1, 1) * lean) * lift, left = -tanf(clampf(c.roll, -1, 1) * lean) * lift;
  F->yaw_sp += clampf(c.yaw, -1, 1) * FC_YAW_RATE * dt;
  if (F->yaw_sp > 3.14159265f) F->yaw_sp -= 6.2831853f; else if (F->yaw_sp < -3.14159265f) F->yaw_sp += 6.2831853f;
  float cy = cosf(F->yaw_sp), sy = sinf(F->yaw_sp);
  float Fd[3] = { A->m * (fwd * cy - left * sy), A->m * (fwd * sy + left * cy), A->m * lift };

  /* the attitude wanted, and the torque for it (the simulator's controlStep) */
  float nd[3], Rd[9], F1[9], F2[9], eR[3], tau[3], Fb[3], f[3];
  const float share = A->mode == 1 ? A->mix_share * F->rho : 0;
  p.n = 0; p_v(&p, Fd, 3); p_f(&p, (float)A->mode); p_f(&p, share); p_f(&p, A->lean_max);
  if (call(F, F->f_ta, 0, &p, nd)) return;
  float l = sqrtf(nd[0] * nd[0] + nd[1] * nd[1] + nd[2] * nd[2]); nd[0] /= l; nd[1] /= l; nd[2] /= l;
  frame_from(F1, nd, (float[3]){ cy, sy, 0 }); frame_from(F2, A->axis, (float[3]){ 1, 0, 0 }); m3mt(Rd, F1, F2);
  p.n = 0; p_v(&p, F->R, 9); p_v(&p, Rd, 9);
  if (call(F, F->f_err, 0, &p, eR)) return;
  if (c.throttle > 0.15f) for (int k = 0; k < 3; k++) F->iAtt[k] = clampf(F->iAtt[k] + eR[k] * dt, -0.5f, 0.5f);
  p.n = 0; p_v(&p, eR, 3); p_v(&p, F->w, 3); p_v(&p, F->iAtt, 3); p_v(&p, A->J, 9);
  if (call(F, F->f_ctl, 0, &p, tau)) return;
  m3tv(Fb, F->R, Fd);
  p.n = 0; p_v(&p, Fb, 3); p_v(&p, A->axis, 3); p_f(&p, (float)A->mode);
  if (call(F, F->f_fd, 0, &p, f)) return;

  /* allocation → throttles */
  float wa[6], ta[3]; for (int k = 0; k < 3; k++) wa[k] = f[k] / A->m;
  m3v(ta, A->Jinv, tau); memcpy(wa + 3, ta, sizeof ta);
  float u[FC_MAX_MOTORS];
  if (allocate(F, wa, u)) return;
  if (A->mode == 1) {                  /* how much of the asked-for sideways force the thrusts make (the simulator's steerMix.rho) */
    float dem = sqrtf(wa[0] * wa[0] + wa[1] * wa[1]);
    if (dem > 0.2f) {
      float gx = 0, gy = 0, col[6];
      for (int i = 0; i < A->n_motors; i++) { col_at(F, i, -1, col); gx += col[0] * u[i]; gy += col[1] * u[i]; }
      F->rho += (clampf((gx * wa[0] + gy * wa[1]) / (dem * dem), 0, 1) - F->rho) * fminf(1, dt / 0.3f);
    } else F->rho += (1 - F->rho) * fminf(1, dt / 2);
  }
  for (int i = 0; i < A->n_motors; i++) {
    float want, sent;
    p.n = 0; p_f(&p, u[i]); p_f(&p, A->mot[i].bend);
    if (call(F, F->f_lin, 0, &p, &want)) return;
    want = clampf(want, FC_IDLE, 1);
    sent = want; float eq = want;      /* eq: what it amounts to at the reference voltage (less when the correction runs out) */
    if (vbatt > 1) { p.n = 0; p_f(&p, want); p_f(&p, vbatt); p_f(&p, F->vref); if (call(F, F->f_vc, 0, &p, &sent)) return; eq = fminf(want, sent * vbatt / F->vref); }
    float k = A->mot[i].bend; F->v[i] = (1 - k) * eq + k * eq * eq;
    o->motor[i] = clampf(sent, 0, 1);
  }
}
