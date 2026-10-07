/* LiftLab flight controller: see fc_core.h. */
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

/* A small formatter for the status lines (%s, %d, %.Nf), so the flight code needs no printf. Appends at o + *at. */
void fc_vfmt(char *o, int size, int *at, const char *fmt, va_list ap) {
  int n = *at, cap = size - 1;
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
  o[n] = 0; *at = n;
}
void fc_fmt(char *o, int size, const char *fmt, ...) { int at = 0; va_list ap; va_start(ap, fmt); fc_vfmt(o, size, &at, fmt, ap); va_end(ap); }
void fc_fmt_add(char *o, int size, const char *fmt, ...) { int at = 0; while (o[at] && at < size - 1) at++; va_list ap; va_start(ap, fmt); fc_vfmt(o, size, &at, fmt, ap); va_end(ap); }
static void fc_say(fc_state *F, const char *fmt, ...) { int at = 0; va_list ap; va_start(ap, fmt); fc_vfmt(F->why, (int)sizeof F->why, &at, fmt, ap); va_end(ap); }

#define G_ 9.81f
#define FC_RN_IN 24                 /* actuator inputs the formulas' lists hold (RN_IN in js/rn-sigs.js) */
#define FC_AZ_MAX 5.0f              /* throttle stick: ±this vertical acceleration around hover [m/s²] */
#define FC_YAW_RATE 2.0f            /* yaw stick: at most this turn rate [rad/s] */
#define FC_K_AZ 1.0f                /* how fast the thrust trim follows the measured vertical acceleration [1/s] */
#define FC_AZ_TRIM 3.0f             /* at most this much trim [m/s²] */
#define FC_FS_DESCENT 1.0f          /* failsafe descent speed [m/s] */
#define FC_VZ_LEAK 20.0f            /* the accelerometer's vertical speed forgets over this long [s] */
#define FC_K_ALT 2.1f               /* barometer filter: height, speed and accelerometer-bias corrections [1/s], [1/s²], */
#define FC_K_VZ 1.47f               /* [1/s³] (three equal poles at 0.7 rad/s: the bias settles in about 5 s) */
#define FC_K_B 0.343f
#define FC_FS_NOBARO_S 120.0f       /* failsafe without a barometer: disarm after this long whatever happens (about 80 m of descent) [s] */
#define FC_VZ_MAX 2.0f              /* throttle stick with a barometer: at most this climb or sink speed [m/s] */
#define FC_K_V 1.5f                 /* vertical speed → acceleration [1/s] */
#define FC_K_HOLD 1.0f              /* height hold: height error → speed [1/s] */
#define FC_ERR_HOLD 0.05f           /* a formula failing in flight: hold the last outputs this long, then stop [s] */
#define FC_AIRFRAME_MAGIC 0x41424644u   /* 'DFBA' */

const char *fc_state_name(int s) {
  switch (s) { case FC_DISARMED: return "disarmed"; case FC_ARMED: return "armed"; case FC_FAILSAFE: return "failsafe";
    case FC_CRASHED: return "crashed"; case FC_TESTING: return "motor test"; }
  return "?";
}

/* ── small math ── */
static float fabsf_(float x) { return x < 0 ? -x : x; }
static int fin(float x) { return (x - x) == 0; }   /* finite: false for NaN and ±inf */
static float clampf(float x, float a, float b) { return x > a ? (x < b ? x : b) : a; }   /* NaN → a */
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
int fc_basis(float *v, const float *ang, int k, int dm) {
  int n = 1; v[0] = 1;
  for (int i = 0; i < k; i++) {
    float f[3] = { 1, cosf(ang[i]), sinf(ang[i]) };
    if (i == dm) { f[0] = 0; f[1] = -sinf(ang[i]); f[2] = cosf(ang[i]); }
    for (int a = n - 1; a >= 0; a--) { float x = v[a]; v[3 * a] = x * f[0]; v[3 * a + 1] = x * f[1]; v[3 * a + 2] = x * f[2]; }
    n *= 3;
  }
  return n;
}
/* Motor i's columns as flown: the learned ones, or the description's scaled by the supervisor's effectiveness. */
static float col_k(const fc_state *F, int i, int k, int r) { return F->use_learned ? F->lcols[i][k][r] : F->A.mot[i].cols[k][r] * F->m_eff[i]; }
static void col_at_angles(const fc_state *F, int i, const float *ang, int dm, float *out) {
  const fc_motor *M = &F->A.mot[i]; float b[FC_MAX_BASIS];
  int n = fc_basis(b, ang, M->n_chain, dm);
  for (int r = 0; r < 6; r++) { float s = 0; for (int k = 0; k < n && k < M->n_basis; k++) s += b[k] * col_k(F, i, k, r); out[r] = s; }
}
static void col_at(const fc_state *F, int i, int dm, float *out) {   /* motor i's effect now (dm ≥ 0: its change as chain joint dm turns) */
  const fc_motor *M = &F->A.mot[i]; float ang[FC_MAX_CHAIN];
  for (int c = 0; c < M->n_chain; c++) ang[c] = F->th_hat[M->chain[c]];
  col_at_angles(F, i, ang, dm, out);
}
static const float *axis_of(const fc_state *F) { return F->use_learned ? F->laxis : F->A.axis; }
const float *fc_axis(const fc_state *F) { return axis_of(F); }
static int mode_of(const fc_state *F) { return F->hold_servos ? 0 : F->A.mode; }
static int steers(const fc_state *F, int j) { return F->A.jnt[j].steer && !F->hold_servos && !F->j_off[j]; }
static float j_rate(const fc_state *F, int j) { return F->j_rate[j] > 0 ? F->j_rate[j] : F->A.jnt[j].rate; }
static float j_lag(const fc_state *F, int j) { return F->j_rate[j] > 0 ? F->j_lag[j] : F->A.jnt[j].lag; }
static float m_bend(const fc_state *F, int i) { return F->m_bend[i] >= 0 ? F->m_bend[i] : F->A.mot[i].bend; }

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
  /* a new airframe: the description, nothing learned, every part working */
  F->use_learned = F->hold_servos = 0; F->exc_mode = 0; F->sup_mode = 0; F->lim_lean = 0; F->lim_accel = 0;
  for (int i = 0; i < FC_MAX_MOTORS; i++) { F->m_on[i] = 1; F->m_eff[i] = 1; F->m_cap[i] = 1; F->m_bend[i] = -1; }
  for (int j = 0; j < FC_MAX_JOINTS; j++) { F->j_off[j] = 0; F->j_rate[j] = 0; F->j_lag[j] = 0; }
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
  if (!e) { int n = rn_host_out_size(F->H, fn); for (int k = 0; k < n; k++) if (!fin(out[k])) { e = -1; break; } }   /* a number that isn't finite is a failure too */
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
  F->vref = 16.0f; F->rho = 1; F->cmd.test_motor = -1; F->test_released = 1;
  F->q[0] = 1; qmat(F->R, F->q);
  return 0;
}

/* how far the nominal thrust axis leans from straight up [deg] */
static float axis_tilt(const fc_state *F) {
  const float *a = axis_of(F), *R = F->R;
  return acosf(clampf(R[6] * a[0] + R[7] * a[1] + R[8] * a[2], -1, 1)) * 57.2958f;
}

static int batt_ok(const fc_state *F) { return F->vbatt > 0.6f * F->vref && F->vbatt < 1.35f * F->vref; }

/* ── the data bus (docs/topic-bus.md) ── */
enum { BT_STATE, BT_ATT, BT_IMU, BT_HEIGHT, BT_OUT, BT_TORQUE, BT_CMD };
int fc_bus_attach(fc_state *F, bus *B) {
  static const char *names[] = { "fc.state", "fc.attitude", "fc.imu", "fc.height", "fc.output", "fc.torque", "cmd.pilot" };   /* (BT_ order) */
  static const int sizes[] = { 7, 7, 6, 3, FC_MAX_MOTORS + FC_MAX_JOINTS, 3, 10 };
  F->bus = 0; if (!B) return 0;
  for (int i = 0; i < 7; i++) if ((F->bt[i] = bus_topic(B, names[i], sizes[i])) < 0) return -1;
  F->bus = B; return 0;
}
static void bus_after_step(fc_state *F, const fc_out *o) {
  bus *B = F->bus; if (!B) return;
  float v[FC_MAX_MOTORS + FC_MAX_JOINTS];
  v[0] = (float)F->state; v[1] = (float)F->att_ok; v[2] = (float)F->have_alt; v[3] = (float)F->cmd.guided; v[4] = (float)F->sup_mode; v[5] = (float)F->use_learned; v[6] = (float)F->trap;
  bus_pub(B, F->bt[BT_STATE], v, 7);
  for (int k = 0; k < 4; k++) v[k] = F->q[k];
  for (int k = 0; k < 3; k++) v[4 + k] = F->w[k];
  bus_pub(B, F->bt[BT_ATT], v, 7);
  if (F->have_imu) { for (int k = 0; k < 3; k++) { v[k] = F->fb[k]; v[3 + k] = F->gb[k]; } bus_pub(B, F->bt[BT_IMU], v, 6); }
  v[0] = F->have_alt ? F->alt_e : 0; v[1] = F->have_alt ? F->vz_e : F->vz_i; v[2] = (float)F->have_alt;
  bus_pub(B, F->bt[BT_HEIGHT], v, 3);
  for (int i = 0; i < FC_MAX_MOTORS; i++) v[i] = o->motor[i];
  for (int j = 0; j < FC_MAX_JOINTS; j++) v[FC_MAX_MOTORS + j] = o->servo[j];
  bus_pub(B, F->bt[BT_OUT], v, FC_MAX_MOTORS + FC_MAX_JOINTS);
  if (F->state == FC_ARMED || F->state == FC_FAILSAFE) bus_pub(B, F->bt[BT_TORQUE], F->tau_des, 3);
}

void fc_command(fc_state *F, const fc_cmd *in) {
  /* a command with a number that isn't finite is ignored (it doesn't count as a command either) */
  if (!fin(in->roll) || !fin(in->pitch) || !fin(in->yaw) || !fin(in->throttle) || !fin(in->test_throttle)) return;
  if (in->guided && (!fin(in->acc[0]) || !fin(in->acc[1]) || !fin(in->acc[2]) || !fin(in->heading))) return;
  fc_cmd cc = *in, *c = &cc;
  c->roll = clampf(c->roll, -1, 1); c->pitch = clampf(c->pitch, -1, 1); c->yaw = clampf(c->yaw, -1, 1);
  c->throttle = clampf(c->throttle, 0, 1); c->test_throttle = clampf(c->test_throttle, 0, 0.3f);
  c->guided = c->guided ? 1 : 0;
  if (c->guided) { c->acc[0] = clampf(c->acc[0], -10, 10); c->acc[1] = clampf(c->acc[1], -10, 10); c->acc[2] = clampf(c->acc[2], -FC_AZ_MAX, FC_AZ_MAX); }
  if (c->test_motor < 0 || c->test_motor >= FC_MAX_MOTORS) c->test_motor = -1;
  int prev_test = F->cmd.test_motor;
  F->cmd = *c; F->cmd_t = F->t;
  if (F->bus) { float v[10] = { (float)c->arm, c->roll, c->pitch, c->yaw, c->throttle, (float)c->guided, c->acc[0], c->acc[1], c->acc[2], c->heading }; bus_pub(F->bus, F->bt[BT_CMD], v, 10); }
  if (!c->arm) { F->arm_released = 1; if (F->state == FC_ARMED || F->state == FC_FAILSAFE || F->state == FC_CRASHED) { F->state = FC_DISARMED; fc_say(F, "disarmed"); F->sup_mode = 0; F->sup_landing = 0; } }
  if (c->test_motor < 0) F->test_released = 1;
  if (c->arm && F->state == FC_DISARMED) {   /* arming takes the switch going on: after any disarm it must be seen off first */
    float tilt = axis_tilt(F);
    const char *no = !F->arm_released ? "turn the arm switch off, then on" : !F->have_airframe ? "no airframe loaded" : !F->sizes_ok ? "the formulas don't match this firmware" :
      !F->att_ok ? "no gyro, or the attitude isn't settled yet" : tilt > 15 ? "not level" : c->throttle >= 0.05f ? "throttle stick not at the bottom" :
      F->batt_wired && !batt_ok(F) ? "the battery reading doesn't fit the pack (check the sense wire and vref)" : 0;
    if (no) fc_say(F, "won't arm: %s", no);
    else {
      F->state = FC_ARMED; F->arm_released = 0; F->sup_landing = 0; fc_say(F, "armed");
      F->yaw_sp = atan2f(F->R[3], F->R[0]); memset(F->iAtt, 0, sizeof F->iAtt); F->iAz = 0; F->vz_i = 0; F->err_t = 0;
      memset(&F->last_out, 0, sizeof F->last_out);   /* what a failing first step would hold: idle, servos where they are */
      for (int i = 0; i < F->A.n_motors; i++) F->last_out.motor[i] = FC_IDLE;
      for (int j = 0; j < F->A.n_joints; j++) F->last_out.servo[j] = F->th_cmd[j];
    }
  }
  if (c->arm && F->state == FC_FAILSAFE && !F->sup_landing) { F->state = FC_ARMED; fc_say(F, "commands back: armed"); }
  /* motor test: disarmed only, one motor, for FC_TEST_S from when it starts; another needs the test switched off first */
  if (F->state == FC_TESTING && (c->arm || c->test_motor != prev_test)) { F->state = FC_DISARMED; fc_say(F, "motor test stopped"); }
  if (!c->arm && F->state == FC_DISARMED && c->test_motor >= 0 && c->test_motor < F->A.n_motors && F->have_airframe && F->test_released) {
    F->state = FC_TESTING; F->test_t = F->t; F->test_released = 0; fc_say(F, "motor test: %s", F->A.mot[c->test_motor].name);
  }
}

void fc_keepalive(fc_state *F) { if (F->state == FC_ARMED) F->cmd_t = F->t; }

/* Allocation, as the simulator's allocate(): the servos' angle changes, then every motor's thrust. */
static int allocate(fc_state *F, const float *wa, float *u) {
  const fc_airframe *A = &F->A; float out[64]; pk p;
  static float cols[FC_RN_IN * 6], lo[FC_RN_IN], hi[FC_RN_IN], inp[FC_RN_IN * 14];
  int nm = A->n_motors;
  for (int stage = 0; stage < 2; stage++) {
    int ns = 0, sj[FC_MAX_JOINTS];
    if (stage == 0) { for (int j = 0; j < A->n_joints; j++) if (steers(F, j)) sj[ns++] = j; if (!ns) continue; }
    int n = 0; float amax = 1e-9f;
    for (int i = 0; i < nm; i++, n++) {
      col_at(F, i, -1, cols + 6 * n); lo[n] = 0; hi[n] = F->m_on[i] ? F->m_cap[i] : 0;   /* the supervisor's: removed, or capped */
      float *q = inp + 14 * n; memset(q, 0, 14 * sizeof(float));
      q[0] = 0; q[1] = F->v[i]; q[2] = 0; q[3] = hi[n]; q[6] = 1; q[7] = A->mot[i].power;   /* kind thrust, x, lo, hi, (authority), power */
    }
    for (int s = 0; s < ns; s++, n++) {
      const fc_joint *J = &A->jnt[sj[s]]; float th = F->th_hat[sj[s]], reach = j_rate(F, sj[s]) * fmaxf(0.005f, A->horizon - j_lag(F, sj[s]));
      float d[6] = { 0 };
      for (int i = 0; i < nm; i++) {
        const fc_motor *M = &A->mot[i]; int m = -1; for (int c = 0; c < M->n_chain; c++) if (M->chain[c] == sj[s]) m = c;
        if (m < 0) continue;
        if (!F->m_on[i]) continue;
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
    p.n = 0; p_list(&p, cols, n, 6); p_list(&p, lo, n, 1); p_list(&p, hi, n, 1); p_v(&p, wa, 6); p_f(&p, (float)mode_of(F));
    p_f(&p, 1); p_v(&p, pull, 2 * (1 + FC_RN_IN));
    if (call(F, F->f_alloc, 0, &p, out)) return -1;
    if (stage == 0) for (int s = 0; s < ns; s++) { const fc_joint *J = &A->jnt[sj[s]]; F->th_cmd[sj[s]] = clampf(F->th_hat[sj[s]] + out[1 + nm + s], -J->range, J->range); }
    else for (int i = 0; i < nm; i++) u[i] = out[1 + i];
  }
  return 0;
}

/* The failsafe's descent (commands lost, or the supervisor landing it). */
static void start_descent(fc_state *F) {
  F->state = FC_FAILSAFE; F->fs_t = F->t; F->fs_land_t = 0; F->fs_alt_ref = F->alt_e; F->fs_alt_t = F->t;
  F->fs_bump_t = -1e9; F->fs_vz = F->vz_i;
  /* Without a barometer the thrust trim is frozen, at no more than it was when last hovering calmly: while
   * climbing fast the trim also holds up against drag, and frozen that would keep it climbing. */
  if (!F->have_alt) F->iAz = fminf(F->iAz, F->t - F->calm_t < 30 ? F->iAz_calm : 0);
}

/* One step; returns 0, or −1 when a formula failed (the outputs are then not to be used). */
static int step(fc_state *F, const fc_imu *imu, float dt, float vbatt, fc_out *o) {
  const fc_airframe *A = &F->A; pk p; float r[8];
  F->t += dt; F->steps++; F->vbatt = vbatt;
  memset(o, 0, sizeof *o);
  /* disarmed, the servos go back to their set angles: a helicopter's swashplate is levelled while its rotor runs
   * down (left tilted on the ground, the spinning disc can roll it over), a tilt-rotor's motors point as built */
  if (F->state == FC_DISARMED) for (int j = 0; j < A->n_joints; j++) F->th_cmd[j] = A->jnt[j].manual;
  for (int j = 0; j < A->n_joints; j++) o->servo[j] = F->th_cmd[j];
  if (!F->have_airframe || !F->sizes_ok) return 0;

  /* attitude, from the IMU in body axes */
  if (imu->have_gyro) {
    float g[3], a[3]; m3v(g, A->imu_R, imu->gyro); m3v(a, A->imu_R, imu->acc);
    memcpy(F->fb, a, sizeof a); memcpy(F->gb, g, sizeof g); F->have_imu = 1;
    int hm = imu->have_mag && fin(imu->mag[0]) && fin(imu->mag[1]) && fin(imu->mag[2]);
    p.n = 0; p_v(&p, g, 3); p_v(&p, a, 3); p_f(&p, (float)hm); p_v(&p, hm ? imu->mag : (float[3]){ 0, 0, 0 }, 3); p_f(&p, dt);
    if (call(F, F->f_att, 0, &p, r)) return -1;
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
    F->have_imu = 0; F->imu_gap += dt;
    /* a missed sample or two (an I2C error, a late wake-up) isn't a lost IMU: the attitude holds for 50 ms */
    if (F->imu_gap > 0.05f) { F->att_ok = 0; F->att_t = 0; }
    if ((F->state == FC_ARMED || F->state == FC_FAILSAFE) && F->imu_gap > 0.2f) { F->state = FC_CRASHED; fc_say(F, "IMU lost in flight: motors off"); }
  }

  /* height and vertical speed: the barometer and the accelerometer (a complementary filter); without a
   * barometer, the vertical speed is the accelerometer's alone, leaking toward 0 */
  /* The accelerometer's vertical reading, less the bias the barometer has shown it to have: a bias of a few
   * hundredths of a g appears in flight (vibration, temperature) and would otherwise skew the speed. */
  const float az_c = F->az_f - (F->have_alt ? F->az_b : 0);
  if (F->have_alt) { F->alt_e += F->vz_e * dt; F->vz_e += az_c * dt; }       /* between readings: the accelerometer */
  if (imu->have_baro && fin(imu->baro_alt)) {                              /* a new reading (they come at ~25 Hz) */
    if (!F->have_alt) { F->alt_e = imu->baro_alt; F->vz_e = F->vz_i; F->az_b = 0; F->have_alt = 1; }
    else {
      float e = imu->baro_alt - F->alt_e, h = fminf(F->baro_gap + dt, 0.2f);
      F->alt_e += FC_K_ALT * e * h; F->vz_e += FC_K_VZ * e * h; F->az_b = clampf(F->az_b - FC_K_B * e * h, -2, 2);
    }
    F->baro_gap = 0;
  } else if ((F->baro_gap += dt) > 0.5f) F->have_alt = 0;                 /* none for 0.5 s: fly without it */

  /* the learning task's excitation: added to what the controller asks, or open loop (the throw start) */
  const int exc_live = F->exc_mode && F->t - F->exc_t < FC_EXC_TIMEOUT && F->state == FC_ARMED, open = exc_live && F->exc_mode == 2;
  if (!(exc_live && F->exc_hold_s)) F->held_s = 0;
  else if (!F->held_s) { for (int j = 0; j < A->n_joints; j++) F->hold_th[j] = steers(F, j) ? F->th_cmd[j] : A->jnt[j].manual; F->held_s = 1; }

  /* servos: where they are believed to be (one the supervisor took out is where it says it is) */
  for (int j = 0; j < A->n_joints; j++) {
    const fc_joint *J = &A->jnt[j]; float tgt = steers(F, j) ? F->th_cmd[j] : J->manual, th;
    if (F->j_off[j]) { F->th_hat[j] = F->j_ang[j]; o->servo[j] = F->j_ang[j]; continue; }
    if (exc_live && J->steer) {
      int has = (F->exc_smask >> j) & 1;
      if (open) tgt = has ? F->exc_s[j] : tgt;
      else { if (F->exc_hold_s) tgt = F->hold_th[j]; if (has) tgt += F->exc_s[j]; }
      tgt = clampf(tgt, -J->range, J->range);
    }
    p.n = 0; p_f(&p, tgt); p_f(&p, j_rate(F, j)); p_f(&p, j_lag(F, j)); p_f(&p, dt);
    if (call(F, F->f_srv, j, &p, &th)) return -1;
    F->th_hat[j] = th;
    o->servo[j] = tgt;
  }

  /* safety */
  float tilt = axis_tilt(F);
  float vz = F->have_alt ? F->vz_e : F->vz_i;
  if (F->state == FC_TESTING) {
    if (F->t - F->test_t > FC_TEST_S || F->t - F->cmd_t > FC_CMD_TIMEOUT) { F->state = FC_DISARMED; fc_say(F, "motor test done"); return 0; }
    o->motor[F->cmd.test_motor] = F->cmd.test_throttle; return 0;
  }
  if ((F->state == FC_ARMED || F->state == FC_FAILSAFE) && tilt > FC_CRASH_DEG && !open && !F->open_loop && F->t >= F->recover_t)   /* (open_loop: a throw's open loop ended this step, its grace starts below) */ { F->state = FC_CRASHED; fc_say(F, "tilted %.0f°: crashed, motors off", (double)tilt); }
  if (F->state == FC_ARMED && F->t - F->cmd_t > FC_CMD_TIMEOUT) {
    if (F->cmd.throttle < 0.05f) { F->state = FC_DISARMED; fc_say(F, "no commands, at idle: disarmed"); }   /* on the ground, most likely */
    else { start_descent(F); fc_say(F, "no commands: failsafe descent"); }
  }
  /* the supervisor says land (or go home, which without the navigation is the same): the failsafe's descent */
  if (F->state == FC_ARMED && F->sup_mode >= 2 && !F->cmd.guided && F->cmd.throttle >= 0.05f && !open) { start_descent(F); F->sup_landing = 1; fc_say(F, "supervisor: landing"); }
  float fs_az = 0;
  if (F->state == FC_FAILSAFE) {
    if (F->have_alt) {
      /* With a barometer: a speed loop down at 1 m/s. Landed: asking to sink while the height stays put for 1.5 s
       * (in the air, asking to sink makes it sink). */
      fs_az = clampf(FC_K_V * (-FC_FS_DESCENT - vz), -2, 1.5f);
      if (fabsf_(F->alt_e - F->fs_alt_ref) > 0.15f) { F->fs_alt_ref = F->alt_e; F->fs_alt_t = F->t; }
      if (F->t - F->fs_t > 1 && fs_az < -0.9f && F->t - F->fs_alt_t > 1.5f) { F->state = FC_DISARMED; fc_say(F, "failsafe: landed, disarmed"); }
    } else {
      /* Without one the speed is the accelerometer's guess, which drifts: always ask for at least a little downward
       * acceleration on the thrust trim learned in flight (frozen), so it can't hover or climb on a biased guess;
       * drag then sets the descent speed. Landed: the bump of touching down, then no acceleration and no sinking. */
      F->fs_vz += az_c * dt;
      if (az_c > 5) F->fs_bump_t = F->t;                      /* a bump up: touching down */
      fs_az = clampf(FC_K_V * (-FC_FS_DESCENT - F->fs_vz), -0.6f, -0.2f);
      int still = F->t - F->fs_bump_t < 3 && F->t - F->fs_bump_t > 0.2 && fabsf_(az_c) < 0.5f;   /* (allowing a bias up to 0.5 m/s²) */
      if (F->t - F->fs_t > 1 && still) F->fs_land_t += dt; else F->fs_land_t = 0;
      if (F->fs_land_t > 1) { F->state = FC_DISARMED; fc_say(F, "failsafe: landed, disarmed"); }
      else if (F->t - F->fs_t > FC_FS_NOBARO_S) { F->state = FC_DISARMED; fc_say(F, "failsafe: %.0f s without a barometer, disarmed", (double)FC_FS_NOBARO_S); }
    }
  }
  if (F->state != FC_ARMED && F->state != FC_FAILSAFE) return 0;

  /* open loop (the throw start): the learning task's throttles as they are; nothing else runs */
  if (open) {
    int pulsing = 0; for (int i = 0; i < A->n_motors; i++) if (F->exc_m[i] > 0) pulsing = 1;
    if (F->pulse_dw > 0 && pulsing && !F->pulse_cut && F->t - F->pulse_t > 0.012) {   /* a pulse turned it enough: stop now */
      float d0 = F->gb[0] - F->pulse_w0[0], d1 = F->gb[1] - F->pulse_w0[1], d2 = F->gb[2] - F->pulse_w0[2];
      if (d0 * d0 + d1 * d1 + d2 * d2 > F->pulse_dw * F->pulse_dw) F->pulse_cut = 1;
    }
    for (int i = 0; i < A->n_motors; i++) { float u = F->m_on[i] && !F->pulse_cut ? clampf(F->exc_m[i], 0, 1) : 0, k = m_bend(F, i); o->motor[i] = u; F->v[i] = (1 - k) * u + k * u * u; }
    F->vz_i += (F->az_f - F->vz_i / FC_VZ_LEAK) * dt;   /* (the vertical speed, for the learning's timing) */
    F->open_loop = 1; return 0;
  }
  if (F->open_loop) {                  /* it just ended: catch itself, from whatever attitude it is in (no tilt check for 3 s) */
    F->open_loop = 0; F->recover_t = F->t + 3;
    memset(F->iAtt, 0, sizeof F->iAtt); F->iAz = 0; F->vz_i = 0; F->yaw_sp = atan2f(F->R[3], F->R[0]);
  }

  /* what the pilot asks for: lean angles, turn rate, and with the throttle stick around its middle (0.5):
   * with a barometer, climb or sink speed (in the middle it holds the height); without, vertical acceleration */
  fc_cmd c = F->cmd;
  float az;
  if (c.guided) { az = c.acc[2]; F->holding = 0; }          /* the navigation task asks for the acceleration itself */
  else if (F->have_alt) {
    float s = c.throttle - 0.5f, vz_cmd;
    if (fabsf_(s) < 0.05f) { if (!F->holding) { F->alt_hold = F->alt_e; F->holding = 1; } vz_cmd = clampf(FC_K_HOLD * (F->alt_hold - F->alt_e), -1, 1); }
    else { F->holding = 0; vz_cmd = (s - (s > 0 ? 0.05f : -0.05f)) / 0.45f * FC_VZ_MAX; }
    az = clampf(FC_K_V * (vz_cmd - vz), -FC_AZ_MAX, FC_AZ_MAX);
  } else { az = (c.throttle - 0.5f) * 2 * FC_AZ_MAX; F->holding = 0; }
  if (F->state == FC_FAILSAFE) { c.roll = c.pitch = c.yaw = 0; c.guided = 0; az = fs_az; }   /* level, descending */
  else if (c.throttle < 0.05f) {                            /* stick at the bottom: idle, nothing to steer with */
    for (int i = 0; i < A->n_motors; i++) { o->motor[i] = FC_IDLE; F->v[i] = 0; }
    memset(F->iAtt, 0, sizeof F->iAtt); F->iAz = 0; F->vz_i = 0; F->holding = 0; F->yaw_sp = atan2f(F->R[3], F->R[0]);
    return 0;
  }
  float lean_deg = A->lean_max > 0 ? A->lean_max : 30;
  if (F->lim_lean > 0 && F->lim_lean < lean_deg) lean_deg = F->lim_lean;   /* the supervisor's limit */
  const float lean = lean_deg * 0.0174533f;
  /* the thrust for that acceleration: the model's, trimmed by what the accelerometer measures (the model's hover
   * thrust is never exactly right, and without this the drone would drift up or down at "hover") */
  F->vz_i += (F->az_f - F->vz_i / FC_VZ_LEAK) * dt;   /* vertical speed since take-off, from the accelerometer alone: drifts, so it leaks */
  if (!(F->state == FC_FAILSAFE && !F->have_alt))     /* (frozen in a failsafe without a barometer, see above) */
    F->iAz = clampf(F->iAz + FC_K_AZ * (az - az_c) * dt, -FC_AZ_TRIM, FC_AZ_TRIM);
  if (F->state == FC_ARMED && fabsf_(az) < 0.3f && fabsf_(F->vz_i) < 0.5f) {   /* hovering calmly: remember the trim */
    F->iAz_calm = F->t - F->calm_t > 1 ? F->iAz : F->iAz_calm + (F->iAz - F->iAz_calm) * fminf(1, dt / 2); F->calm_t = F->t;
  }
  float lift = G_ + az + F->iAz;
  float Fd[3];
  if (c.guided) {                                           /* the acceleration asked for, in the world */
    F->yaw_sp = c.heading;
    float ax = c.acc[0], ay = c.acc[1], ah = sqrtf(ax * ax + ay * ay);
    if (F->lim_accel > 0 && ah > F->lim_accel) { ax *= F->lim_accel / ah; ay *= F->lim_accel / ah; }   /* the supervisor's limit */
    Fd[0] = A->m * ax; Fd[1] = A->m * ay; Fd[2] = A->m * lift;
  } else {                                                  /* the sticks: lean angles, heading turned at a rate */
    float fwd = tanf(clampf(c.pitch, -1, 1) * lean) * lift, left = -tanf(clampf(c.roll, -1, 1) * lean) * lift;
    F->yaw_sp += clampf(c.yaw, -1, 1) * FC_YAW_RATE * dt;
    float cy0 = cosf(F->yaw_sp), sy0 = sinf(F->yaw_sp);
    Fd[0] = A->m * (fwd * cy0 - left * sy0); Fd[1] = A->m * (fwd * sy0 + left * cy0); Fd[2] = A->m * lift;
  }
  if (F->yaw_sp > 3.14159265f) F->yaw_sp -= 6.2831853f; else if (F->yaw_sp < -3.14159265f) F->yaw_sp += 6.2831853f;
  float cy = cosf(F->yaw_sp), sy = sinf(F->yaw_sp);

  /* the attitude wanted, and the torque for it (the simulator's controlStep) */
  float nd[3], Rd[9], F1[9], F2[9], eR[3], tau[3], Fb[3], f[3];
  const int mode = mode_of(F);
  const float share = mode == 1 ? A->mix_share * F->rho : 0;
  p.n = 0; p_v(&p, Fd, 3); p_f(&p, (float)mode); p_f(&p, share); p_f(&p, lean_deg);
  if (call(F, F->f_ta, 0, &p, nd)) return -1;
  float l = sqrtf(nd[0] * nd[0] + nd[1] * nd[1] + nd[2] * nd[2]); nd[0] /= l; nd[1] /= l; nd[2] /= l;
  frame_from(F1, nd, (float[3]){ cy, sy, 0 }); frame_from(F2, axis_of(F), (float[3]){ 1, 0, 0 }); m3mt(Rd, F1, F2);
  p.n = 0; p_v(&p, F->R, 9); p_v(&p, Rd, 9);
  if (call(F, F->f_err, 0, &p, eR)) return -1;
  if (c.throttle > 0.15f) for (int k = 0; k < 3; k++) F->iAtt[k] = clampf(F->iAtt[k] + eR[k] * dt, -0.5f, 0.5f);
  p.n = 0; p_v(&p, eR, 3); p_v(&p, F->w, 3); p_v(&p, F->iAtt, 3); p_v(&p, A->J, 9);
  if (call(F, F->f_ctl, 0, &p, tau)) return -1;
  memcpy(F->tau_des, tau, sizeof F->tau_des);
  m3tv(Fb, F->R, Fd);
  p.n = 0; p_v(&p, Fb, 3); p_v(&p, axis_of(F), 3); p_f(&p, (float)mode);
  if (call(F, F->f_fd, 0, &p, f)) return -1;

  /* allocation → throttles */
  float wa[6], ta[3]; for (int k = 0; k < 3; k++) wa[k] = f[k] / A->m;
  m3v(ta, A->Jinv, tau); memcpy(wa + 3, ta, sizeof ta);
  float u[FC_MAX_MOTORS];
  if (allocate(F, wa, u)) return -1;
  /* the learning task's excitation: a pulse on one motor while the others hold, or a little on all of them */
  if (exc_live && F->exc_mode == 1) {
    if (F->exc_hold_m && !F->held_m) { memcpy(F->hold_v, u, sizeof(float) * (size_t)A->n_motors); F->held_m = 1; }
    for (int i = 0; i < A->n_motors; i++) u[i] = clampf((F->exc_hold_m ? F->hold_v[i] : u[i]) + F->exc_m[i], 0, F->m_on[i] ? F->m_cap[i] : 0);
  }
  if (!(exc_live && F->exc_mode == 1 && F->exc_hold_m)) F->held_m = 0;
  if (mode == 1) {                  /* how much of the asked-for sideways force the thrusts make (the simulator's steerMix.rho) */
    float dem = sqrtf(wa[0] * wa[0] + wa[1] * wa[1]);
    if (dem > 0.2f) {
      float gx = 0, gy = 0, col[6];
      for (int i = 0; i < A->n_motors; i++) { col_at(F, i, -1, col); gx += col[0] * u[i]; gy += col[1] * u[i]; }
      F->rho += (clampf((gx * wa[0] + gy * wa[1]) / (dem * dem), 0, 1) - F->rho) * fminf(1, dt / 0.3f);
    } else F->rho += (1 - F->rho) * fminf(1, dt / 2);
  }
  /* throttles: all computed before any is set, so a failure never leaves half of them changed */
  const int vc = batt_ok(F);           /* a reading that doesn't fit the pack (a loose sense wire) isn't used */
  float v[FC_MAX_MOTORS];
  for (int i = 0; i < A->n_motors; i++) {
    float want, sent;
    p.n = 0; p_f(&p, u[i]); p_f(&p, m_bend(F, i));
    if (call(F, F->f_lin, 0, &p, &want)) return -1;
    want = clampf(want, FC_IDLE, 1);
    sent = want; float eq = want;      /* eq: what it amounts to at the reference voltage (less when the correction runs out) */
    if (vc) { p.n = 0; p_f(&p, want); p_f(&p, vbatt); p_f(&p, F->vref); if (call(F, F->f_vc, 0, &p, &sent)) return -1; eq = fminf(want, sent * vbatt / F->vref); }
    float k = m_bend(F, i); v[i] = (1 - k) * eq + k * eq * eq;
    o->motor[i] = clampf(sent, 0, 1);
  }
  memcpy(F->v, v, sizeof(float) * (size_t)A->n_motors);
  return 0;
}

static void ltel_add(fc_state *F, const fc_out *o, float dt) {
  if (F->open_loop && F->sub_n < FC_SUB) {             /* open loop: every step, for the throw's fit */
    float *s = F->sub[F->sub_n++]; int k = 0; s[k++] = dt;
    for (int i = 0; i < 3; i++) s[k++] = F->fb[i];
    for (int i = 0; i < 3; i++) s[k++] = F->gb[i];
    for (int i = 0; i < F->A.n_motors; i++) s[k++] = F->lt_vprev[i];   /* the thrusts it has been flying on (this step's take effect next) */
  }
  memcpy(F->lt_vprev, F->v, sizeof F->lt_vprev);
  if (F->have_imu) for (int k = 0; k < 3; k++) { F->lt_f[k] += F->fb[k]; F->lt_w[k] += F->gb[k]; }
  for (int i = 0; i < F->A.n_motors; i++) { F->lt_u[i] += o->motor[i]; F->lt_v[i] += F->v[i]; }
  for (int j = 0; j < F->A.n_joints; j++) F->lt_tc[j] += o->servo[j];
  F->lt_n++;
}
/* Disarmed after a flight: the supervisor's landing is over (its next SET says what it wants now). */
static void after_flight(fc_state *F, int was) {
  if (F->state == FC_DISARMED && (was == FC_ARMED || was == FC_FAILSAFE || was == FC_CRASHED)) { F->sup_mode = 0; F->sup_landing = 0; }
}
void fc_step(fc_state *F, const fc_imu *imu, float dt, float vbatt, fc_out *o) {
  fc_out n; int was = F->state;
  if (!fin(dt) || dt <= 0) dt = 0.001f;
  int e = step(F, imu, dt, vbatt, &n);
  after_flight(F, was);
  ltel_add(F, e ? &F->last_out : &n, dt);
  int flying = F->state == FC_ARMED || F->state == FC_FAILSAFE;
  if (!e) { *o = n; F->err_t = 0; if (flying) F->last_out = n; bus_after_step(F, o); return; }
  /* A formula failed even after the program slots fell back to the built-in program: hold the last outputs for a
   * moment (a one-off glitch passes), then stop the motors. */
  if (flying && (F->err_t += dt) <= FC_ERR_HOLD) { *o = F->last_out; bus_after_step(F, o); return; }
  if (flying) { F->state = FC_CRASHED; fc_say(F, "the flight formulas failed: motors off"); }
  memset(o, 0, sizeof *o);
  for (int j = 0; j < F->A.n_joints; j++) o->servo[j] = F->th_cmd[j];
  bus_after_step(F, o);
}

/* ── the learning task's and the supervisor's frames ── */
int fc_exc(fc_state *F, const float *p, int n) {
  if (n < 6) return -1;
  int nm = (int)p[4], nj = (int)p[5];
  if (nm != F->A.n_motors || nj != F->A.n_joints || n != 8 + nm + nj) return -1;
  for (int k = 0; k < n; k++) if (!fin(p[k])) return -1;
  int mode = (int)p[0]; if (mode < 0 || mode > 2) return -1;
  F->exc_mode = mode; F->exc_hold_m = p[1] > 0.5f; F->exc_hold_s = p[2] > 0.5f; F->exc_smask = (int)p[3];
  for (int i = 0; i < nm; i++) F->exc_m[i] = clampf(p[6 + i], -1, 1);
  for (int j = 0; j < nj; j++) F->exc_s[j] = clampf(p[6 + nm + j], -3.2f, 3.2f);
  int id = (int)p[6 + nm + nj];
  if (id != F->pulse_id) { F->pulse_id = id; F->pulse_t = F->t; memcpy(F->pulse_w0, F->gb, sizeof F->pulse_w0); F->pulse_cut = 0; }
  F->pulse_dw = p[7 + nm + nj] > 0 ? p[7 + nm + nj] : 0;
  F->exc_t = F->t;
  return 0;
}
/* The nominal thrust axis on the learned model (learn.js ctlAxis): the lifting rotors' force at rest, each weighted
 * by how much it points up (a sideways tail rotor doesn't say which way is up). */
static void learned_axis(fc_state *F) {
  float s[3] = { 0, 0, 0 };
  for (int i = 0; i < F->A.n_motors; i++) {
    if (!F->m_on[i]) continue;
    const fc_motor *M = &F->A.mot[i]; float ang[FC_MAX_CHAIN], c[6];
    for (int k = 0; k < M->n_chain; k++) ang[k] = F->A.jnt[M->chain[k]].steer ? 0 : F->A.jnt[M->chain[k]].manual;
    col_at_angles(F, i, ang, -1, c);
    float l = sqrtf(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]); if (l < 1e-9f) continue;
    float w = fmaxf(0, c[2] / l); for (int k = 0; k < 3; k++) s[k] += c[k] * w;
  }
  float l = sqrtf(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
  if (l > 1e-6f) for (int k = 0; k < 3; k++) F->laxis[k] = s[k] / l; else memcpy(F->laxis, F->A.axis, sizeof F->laxis);
}
int fc_model(fc_state *F, const float *p, int n) {
  const fc_airframe *A = &F->A;
  if (n < 4 || (int)p[2] != A->n_motors || (int)p[3] != A->n_joints) return -1;
  for (int k = 0; k < n; k++) if (!fin(p[k])) return -1;
  int k = 4;
  static float cols[FC_MAX_MOTORS][FC_MAX_BASIS][6];
  for (int i = 0; i < A->n_motors; i++) {
    if (k >= n || (int)p[k] != A->mot[i].n_basis) return -1;
    k++;
    if (k + 6 * A->mot[i].n_basis > n) return -1;
    for (int b = 0; b < A->mot[i].n_basis; b++) for (int r = 0; r < 6; r++) cols[i][b][r] = p[k++];
  }
  if (k + 2 * A->n_joints + A->n_motors != n) return -1;
  int use = p[0] > 0.5f;
  if (use != F->use_learned) memset(F->iAtt, 0, sizeof F->iAtt);   /* the integrators were wound up for the other model */
  memcpy(F->lcols, cols, sizeof cols);
  for (int j = 0; j < A->n_joints; j++) { float r = p[k++], l = p[k++]; F->j_rate[j] = r > 0 ? r : 0; F->j_lag[j] = r > 0 ? clampf(l, 0, 0.5f) : 0; }
  for (int i = 0; i < A->n_motors; i++) { float b = p[k++]; F->m_bend[i] = b >= 0 ? clampf(b, 0, 1) : -1; }
  F->use_learned = use; F->hold_servos = p[1] > 0.5f;
  learned_axis(F);
  return 0;
}
int fc_set(fc_state *F, const float *p, int n) {
  if (n < 6) return -1;
  int nm = (int)p[4], nj = (int)p[5];
  if (nm != F->A.n_motors || nj != F->A.n_joints || n != 6 + 3 * nm + 2 * nj) return -1;
  for (int k = 0; k < n; k++) if (!fin(p[k])) return -1;
  int mode = (int)p[0]; if (mode < 0 || mode > 3) return -1;
  if (mode > F->sup_mode || F->state == FC_DISARMED) F->sup_mode = mode;   /* in the air it only steps up (it may already be landing) */
  F->lim_lean = p[1] > 0 ? p[1] : 0; F->lim_accel = p[2] > 0 ? p[2] : 0;
  for (int i = 0; i < nm; i++) { F->m_on[i] = p[6 + 3 * i] > 0.5f; F->m_eff[i] = clampf(p[7 + 3 * i], 0.05f, 2); F->m_cap[i] = clampf(p[8 + 3 * i], 0, 1); }
  for (int j = 0; j < nj; j++) { F->j_off[j] = p[6 + 3 * nm + 2 * j] > 0.5f; F->j_ang[j] = clampf(p[7 + 3 * nm + 2 * j], -3.2f, 3.2f); }
  if (F->use_learned) learned_axis(F);
  return 0;
}
int fc_ltel(fc_state *F, float *o) {
  const fc_airframe *A = &F->A; float k = F->lt_n ? 1.0f / (float)F->lt_n : 0; int n = 0;
  int flying = (F->state == FC_ARMED && F->cmd.throttle >= 0.05f) || F->state == FC_FAILSAFE;
  o[n++] = (float)(F->t - FC_LTEL_WRAP * (double)(long long)(F->t / FC_LTEL_WRAP)); o[n++] = (float)F->state;   /* (the time modulo FC_LTEL_WRAP: exact to 15 µs) */
  o[n++] = (float)(flying | (F->open_loop ? 2 : 0) | (F->use_learned ? 4 : 0) | (F->held_m ? 8 : 0) | (F->pulse_cut ? 16 : 0));
  for (int i = 0; i < 4; i++) o[n++] = F->q[i];
  for (int i = 0; i < 3; i++) o[n++] = F->lt_f[i] * k;
  for (int i = 0; i < 3; i++) o[n++] = F->lt_w[i] * k;
  o[n++] = F->vbatt; o[n++] = F->alt_e; o[n++] = F->have_alt ? F->vz_e : F->vz_i; o[n++] = (float)F->have_alt;
  o[n++] = (float)A->n_motors; o[n++] = (float)A->n_joints;
  for (int i = 0; i < A->n_motors; i++) o[n++] = F->lt_u[i] * k;
  for (int i = 0; i < A->n_motors; i++) o[n++] = F->lt_v[i] * k;
  for (int j = 0; j < A->n_joints; j++) o[n++] = F->lt_tc[j] * k;
  for (int j = 0; j < A->n_joints; j++) o[n++] = F->th_hat[j];
  o[n++] = (float)F->sub_n;
  for (int s = 0; s < F->sub_n; s++) for (int k = 0; k < 7 + A->n_motors; k++) o[n++] = F->sub[s][k];
  F->sub_n = 0;
  memset(F->lt_f, 0, sizeof F->lt_f); memset(F->lt_w, 0, sizeof F->lt_w); memset(F->lt_u, 0, sizeof F->lt_u); memset(F->lt_v, 0, sizeof F->lt_v); memset(F->lt_tc, 0, sizeof F->lt_tc); F->lt_n = 0;
  return n;
}
