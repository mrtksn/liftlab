/* The navigation task: see nav_core.h. */
#include "nav_core.h"
#if defined(__wasm__)                 /* the simulator's build: no C library */
#define sqrtf(x) __builtin_sqrtf(x)
#define fabsf(x) __builtin_fabsf(x)
void *memcpy(void *d, const void *s, unsigned long n);
void *memset(void *d, int c, unsigned long n);
#else
#include <math.h>
#include <string.h>
#endif

#define G_ 9.81f
#define NAV_MAGIC 0x434E4644u   /* 'DFNC' */

static int fin(float x) { return (x - x) == 0; }
static float clampf(float x, float a, float b) { return x > a ? (x < b ? x : b) : a; }
static void say(nav_state *N, const char *s) { int i = 0; for (; s[i] && i < 63; i++) N->why[i] = s[i]; N->why[i] = 0; }
static void qmat(float *R, const float *q) {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - w * z); R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z); R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y); R[7] = 2 * (y * z + w * x); R[8] = 1 - 2 * (x * x + y * y);
}
static void m3v(float *o, const float *M, const float *v) { float x = M[0] * v[0] + M[1] * v[1] + M[2] * v[2], y = M[3] * v[0] + M[4] * v[1] + M[5] * v[2], z = M[6] * v[0] + M[7] * v[1] + M[8] * v[2]; o[0] = x; o[1] = y; o[2] = z; }
static void m3tv(float *o, const float *M, const float *v) { float x = M[0] * v[0] + M[3] * v[1] + M[6] * v[2], y = M[1] * v[0] + M[4] * v[1] + M[7] * v[2], z = M[2] * v[0] + M[5] * v[1] + M[8] * v[2]; o[0] = x; o[1] = y; o[2] = z; }
static void m3m(float *o, const float *A, const float *B) { float t[9]; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) t[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j]; memcpy(o, t, sizeof t); }
static void cross(float *o, const float *a, const float *b) { float x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2], z = a[0] * b[1] - a[1] * b[0]; o[0] = x; o[1] = y; o[2] = z; }

int nav_config_load(nav_state *N, const uint8_t *blob, uint32_t len) {
  const uint32_t nf = 1 + 3 + 3 + 3 + 9 + 1 + 1;
  if (len != 8 + nf * 4 + 4) { say(N, "nav config: wrong size"); return -1; }
  uint32_t crc; memcpy(&crc, blob + len - 4, 4);
  if (rn_crc32(blob, len - 4) != crc) { say(N, "nav config: checksum mismatch"); return -1; }
  uint32_t magic, ver; memcpy(&magic, blob, 4); memcpy(&ver, blob + 4, 4);
  if (magic != NAV_MAGIC || ver != 1) { say(N, "nav config: not a nav config file"); return -1; }
  float f[1 + 3 + 3 + 3 + 9 + 1 + 1]; memcpy(f, blob + 8, sizeof f);
  for (uint32_t i = 0; i < nf; i++) if (!fin(f[i])) { say(N, "nav config: a value isn't a number"); return -1; }
  nav_config C; int k = 0;
  C.m = f[k++];
  for (int i = 0; i < 3; i++) C.baro_pos[i] = f[k++];
  for (int i = 0; i < 3; i++) C.fix_pos[i] = f[k++];
  for (int i = 0; i < 3; i++) C.flow_pos[i] = f[k++];
  for (int i = 0; i < 9; i++) C.flow_R[i] = f[k++];
  C.speed_max = f[k++];
  C.refs = (int)(f[k++] + 0.5f) & 7;
  if (!(C.m > 0)) { say(N, "nav config: no mass"); return -1; }
  memset(&N->tuning, 0, sizeof N->tuning);
  pid_defaults(N->tuning.accepted, 1);
  N->C = C; N->have_config = 1; say(N, "nav config loaded");
  return 0;
}

int nav_init(nav_state *N, rn_host *H) {
  pid_defaults(N->tuning.accepted, 1);
  N->H = H; N->ok = 0;
  const char *names[] = { "positionEstimator", "flowVelocity", "positionControl" };
  int *slot[] = { &N->f_pe, &N->f_fv, &N->f_pc };
  /* what this code passes and expects back, in floats (js/rn-sigs.js) */
  const int in_sz[] = { 9 + 3 + 3 + 8 + 7 + 1 + 1, 2 + 1 + 3 + 9, 3 + 3 + 3 + 1 + 1 + 7 }, out_sz[] = { 6, 3, 3 };
  for (int i = 0; i < 3; i++) {
    *slot[i] = rn_host_find(H, names[i]);
    if (*slot[i] < 0 || rn_host_in_size(H, *slot[i]) != in_sz[i] || rn_host_out_size(H, *slot[i]) != out_sz[i]) { say(N, "a navigation formula isn't what this code expects"); return -1; }
  }
  N->ok = 1; say(N, "navigation ready");
  return 0;
}

static int call(nav_state *N, int fn, const float *in, float *out) {
  int e = rn_host_call(N->H, fn, 0, in, out);
  if (!e) { int n = rn_host_out_size(N->H, fn); for (int k = 0; k < n; k++) if (!fin(out[k])) return -1; }
  return e;
}

static int step(nav_state *N, const nav_in *in, const nav_sp *sp, float dt, float qn, nav_out *out) {
  const nav_config *C = &N->C;
  float R[9], q[4];
  for (int k = 0; k < 4; k++) q[k] = in->q[k] / qn;
  qmat(R, q);
  float b[64]; int k = 0, e;

  /* the references, each corrected for where its sensor sits */
  int have_baro = in->have_baro && fin(in->baro_alt), have_fix = in->have_fix, have_flow = in->have_flow && in->range > 0;
  float baro_alt = 0, fix_p[3] = { 0 }, fix_v[3] = { 0 }, flow_h = 0, flow_v[2] = { 0 }; int flow_has_v = 0;
  if (have_baro) { float r[3]; m3v(r, R, C->baro_pos); baro_alt = in->baro_alt - r[2]; }
  if (have_fix) {
    float r[3], wr[3], vw[3]; m3v(r, R, C->fix_pos); cross(wr, in->w, C->fix_pos); m3v(vw, R, wr);
    for (int i = 0; i < 3; i++) { fix_p[i] = in->fix_p[i] - r[i]; fix_v[i] = in->fix_v[i] - vw[i]; }
  }
  if (have_flow) {
    float ws[3], Rs[9], o[3]; m3tv(ws, C->flow_R, in->w); m3m(Rs, R, C->flow_R);   /* gyro in the camera's axes; camera → world */
    k = 0; b[k++] = in->flow[0]; b[k++] = in->flow[1]; b[k++] = in->range; for (int i = 0; i < 3; i++) b[k++] = ws[i]; for (int i = 0; i < 9; i++) b[k++] = Rs[i];
    if ((e = call(N, N->f_fv, b, o))) return -1;
    float r[3]; m3v(r, R, C->flow_pos); flow_h = o[2] - r[2];
    if (in->flow_q > 0) { float wr[3], vw[3]; cross(wr, in->w, C->flow_pos); m3v(vw, R, wr); flow_v[0] = o[0] - vw[0]; flow_v[1] = o[1] - vw[1]; flow_has_v = 1; }
  }

  /* where it is: the estimator starts once every reference fitted has spoken (or after 3 s with what there is),
   * so it begins from them rather than from zero */
  N->seen |= (have_baro ? 1 : 0) | (have_fix ? 2 : 0) | (have_flow ? 4 : 0);
  N->t_wait += dt;
  if (!N->t_est && (N->seen & C->refs) != C->refs && N->t_wait < 3) return 0;
  N->t_est += dt;
  /* ready to take off: the estimate has run a moment and stands still (it does, on the ground) */
  if (N->t_est > 1.0f && fabsf(N->v[0]) < 0.3f && fabsf(N->v[1]) < 0.3f && fabsf(N->v[2]) < 0.3f) N->t_still += dt; else N->t_still = 0;
  out->ready = N->t_still > 0.5f || N->have_home;
  k = 0;
  for (int i = 0; i < 9; i++) b[k++] = R[i];
  for (int i = 0; i < 3; i++) b[k++] = in->acc[i];
  b[k++] = (float)have_baro; b[k++] = in->baro_age; b[k++] = baro_alt;
  b[k++] = (float)have_fix; b[k++] = in->fix_age; for (int i = 0; i < 3; i++) b[k++] = fix_p[i]; for (int i = 0; i < 3; i++) b[k++] = fix_v[i];
  b[k++] = (float)have_flow; b[k++] = in->flow_age; b[k++] = (float)have_flow; b[k++] = flow_h; b[k++] = (float)flow_has_v; b[k++] = flow_v[0]; b[k++] = flow_v[1];
  b[k++] = C->m; b[k++] = dt;
  float pv[6];
  if ((e = call(N, N->f_pe, b, pv))) return -1;
  /* home: where it takes off from (the estimate has had the time on the ground to settle on its references) */
  if (!N->have_home && sp->fly && out->ready) { for (int i = 0; i < 3; i++) N->home[i] = pv[i]; N->have_home = 1; N->home_from_fix = have_fix; }
  for (int i = 0; i < 3; i++) { N->p[i] = N->have_home ? pv[i] - N->home[i] : 0; N->v[i] = pv[3 + i]; N->pa[i] = pv[i]; }
  N->have_pa = 1;
  memcpy(out->p, N->p, sizeof out->p); memcpy(out->v, N->v, sizeof out->v); out->have_home = N->have_home;
  N->steps++;

  if (N->landed) { out->landed = 1; return 0; }                    /* the supervisor landed it: it stays down */
  /* "fly" off in the air: it lands where it is first (idling there it would drop); back on, it flies on */
  if (!sp->fly && N->last.fly && !N->fly_land) { N->fly_land = 1; say(N, "fly off in the air: landing"); }
  if (sp->fly && N->fly_land) { N->fly_land = 0; if (N->sup_mode < 2 && !N->rc_rth) { N->auto_on = 0; N->auto_land = 0; } }
  if ((!sp->fly && !N->fly_land) || (!N->have_home && !out->ready)) { memset(N->iPos, 0, sizeof N->iPos); return 0; }   /* on the ground (or not ready to leave it): nothing to steer */

  /* the supervisor's return home and landing, and the landing for "fly" off: it moves the target itself, as the
   * pilot would, within the limits */
  const float *target = sp->target, *vref = sp->vref;
  if ((N->sup_mode >= 2 || N->rc_rth || N->fly_land) && N->have_home) {
    if (!N->auto_on) { N->auto_on = 1; memcpy(N->auto_t, N->p, sizeof N->auto_t); memset(N->auto_v, 0, sizeof N->auto_v); N->land_t = 0; }
    if (N->sup_mode == 3 || N->fly_land) N->auto_land = 1;
    float spd = N->lim_speed > 0 ? N->lim_speed : 1, want[3] = { 0, 0, 0 };
    if (!N->auto_land) {
      float dx = -N->auto_t[0], dy = -N->auto_t[1], d = sqrtf(dx * dx + dy * dy);
      if (d > 0.05f) { float s = (spd < 1.5f * d ? spd : 1.5f * d) / d; want[0] = dx * s; want[1] = dy * s; }
      if (d < 0.15f && sqrtf(N->p[0] * N->p[0] + N->p[1] * N->p[1]) < 0.4f) N->auto_land = 1;
    } else want[2] = N->p[2] > 0.8f ? -0.6f : -0.3f;
    for (int i = 0; i < 3; i++) { N->auto_v[i] += clampf(want[i] - N->auto_v[i], -2 * dt, 2 * dt); N->auto_t[i] += N->auto_v[i] * dt; }
    if (N->auto_t[2] < -0.2f) N->auto_t[2] = -0.2f;
    target = N->auto_t; vref = N->auto_v;
    if (N->auto_land) {                  /* down: settled low and slow for half a second */
      float vv = sqrtf(N->v[0] * N->v[0] + N->v[1] * N->v[1] + N->v[2] * N->v[2]);
      N->land_t = N->p[2] < 0.2f && vv < 0.3f ? N->land_t + dt : 0;
      if (N->land_t > 0.5f && N->fly_land && N->sup_mode < 2 && !N->rc_rth) {   /* down for "fly" off: it idles there */
        N->fly_land = 0; N->auto_on = 0; N->auto_land = 0; memset(N->iPos, 0, sizeof N->iPos); say(N, "landed"); return 0;
      }
      if (N->land_t > 0.5f) { N->landed = 1; N->fly_land = 0; out->landed = 1; say(N, "the supervisor landed it"); return 0; }
    }
  }

  /* where to go: the acceleration toward the target */
  float ep[3], ev[3], vmax = C->speed_max > 0 ? C->speed_max : 6;
  if (N->lim_speed > 0 && N->lim_speed < vmax) vmax = N->lim_speed;   /* the supervisor's limit */
  for (int i = 0; i < 3; i++) {
    ep[i] = target[i] - N->p[i];
    ev[i] = N->v[i] - clampf(vref[i], -vmax, vmax);
    /* the integral only works near the target: on the way to a far one it would wind up and carry the drone past it
     * (a 4 m descent left it 5 m·s, over a metre, too low) */
    if (fabsf(ep[i]) < 1) N->iPos[i] = clampf(N->iPos[i] + ep[i] * dt, i < 2 ? -2 : -5, i < 2 ? 2 : 5);
  }
  k = 0;
  for (int i = 0; i < 3; i++) b[k++] = ep[i];
  for (int i = 0; i < 3; i++) b[k++] = ev[i];
  for (int i = 0; i < 3; i++) b[k++] = N->iPos[i];
  b[k++] = C->m; b[k++] = G_;
  b[k++] = 1;                                                      /* lim: the supervisor's (or the usual 6 m/s², 35°) */
  b[k++] = 1; b[k++] = N->lim_accel > 0 ? N->lim_accel : 6; b[k++] = 1; b[k++] = N->lim_lean > 0 ? N->lim_lean : 35; b[k++] = N->lim_speed > 0; b[k++] = N->lim_speed;
  float Fd[3];
  if ((e = call(N, N->f_pc, b, Fd))) return -1;
  if (N->H->act == 0 && N->H->phase == RN_PH_FLYING && N->H->pending == 0 &&
      (N->tuning.enabled || N->tuning.pending)) {
    const float *g = pid_gains(&N->tuning);
    float want[3], a[3], k = g[0] / g[3];
    for (int i = 0; i < 3; i++)
      want[i] = k * ep[i];
    float xy = sqrtf(want[0] * want[0] + want[1] * want[1]), vh = N->lim_speed > 0 ? N->lim_speed : 6;
    if (xy > vh) {
      want[0] *= vh / xy;
      want[1] *= vh / xy;
    }
    want[2] = clampf(want[2], -1.5f, 3);
    for (int i = 0; i < 3; i++)
      a[i] = g[3] * (want[i] - ev[i]) + g[6] * N->iPos[i];
    xy = sqrtf(a[0] * a[0] + a[1] * a[1]);
    float max = N->lim_accel > 0 ? N->lim_accel : 6;
    if (xy > max) {
      a[0] *= max / xy;
      a[1] *= max / xy;
    }
    a[2] = clampf(a[2], -6, 8);
    for (int i = 0; i < 3; i++)
      Fd[i] = C->m * (a[i] + (i == 2 ? G_ : 0));
  }
  for (int i = 0; i < 3; i++) out->acc[i] = Fd[i] / C->m;
  out->acc[2] -= G_;
  out->heading = sp->heading; out->fly = 1;
  return 0;
}

int nav_test_target(nav_state *N, int axis, float offset) {
  if (axis < 0 || axis > 2 || !fin(offset)) return -1;
  N->test_axis = axis; N->test_offset = clampf(offset, -0.2f, 0.2f); N->test_left = 0.1f; return 0;
}
int nav_step(nav_state *N, const nav_in *in, const nav_sp *sp, float dt, nav_out *out) {
  pid_tick(&N->tuning, dt, N->H && N->H->act == 0 && N->H->phase == RN_PH_FLYING && N->H->pending == 0,
           sp->fly && !N->sup_mode && !N->rc_rth, 1);
  nav_sp test = *sp;
  if (dt > 0 && fin(dt)) N->test_left = N->test_left > dt ? N->test_left - dt : 0;
  if (N->test_left > 0 && !N->sup_mode && !N->rc_rth && sp->fly) test.target[N->test_axis] += N->test_offset;
  sp = &test;
  memset(out, 0, sizeof *out);
  out->heading = sp->heading;
  if (!N->ok || !N->have_config) return 0;
  float qn = sqrtf(in->q[0] * in->q[0] + in->q[1] * in->q[1] + in->q[2] * in->q[2] + in->q[3] * in->q[3]);
  /* nothing to step on (a missed IMU sample, two frames at once): the last output stands, flying or not */
  if (!in->have_att || !(dt > 0) || !(qn > 0.5f)) { *out = N->last; return 1; }
  int e = step(N, in, sp, dt, qn, out);
  N->last = *out;
  if (N->bus) {
    float v[9];
    for (int i = 0; i < 3; i++) { v[i] = out->p[i]; v[3 + i] = out->v[i]; }
    v[6] = (float)out->have_home; v[7] = (float)out->ready; v[8] = (float)out->landed; bus_pub(N->bus, N->bt[0], v, 9);
    for (int i = 0; i < 3; i++) { v[i] = sp->target[i]; v[3 + i] = sp->vref[i]; }
    v[6] = sp->heading; v[7] = (float)sp->fly; bus_pub(N->bus, N->bt[1], v, 8);
    for (int i = 0; i < 3; i++) v[i] = out->acc[i];
    v[3] = out->heading; v[4] = (float)out->fly; bus_pub(N->bus, N->bt[2], v, 5);
  }
  return e;
}
int nav_bus_attach(nav_state *N, bus *B) {
  N->bus = 0; if (!B) return 0;
  if ((N->bt[0] = bus_topic(B, "nav.estimate", 9, "p[3] v[3] hasHome ready landed")) < 0 || (N->bt[1] = bus_topic(B, "nav.setpoint", 8, "target[3] vref[3] heading fly")) < 0
      || (N->bt[2] = bus_topic(B, "nav.command", 5, "acc[3] heading fly")) < 0) return -1;
  N->bus = B; return 0;
}

void nav_set(nav_state *N, const float *p, int n) {
  if (n < 6) return;
  for (int k = 0; k < 4; k++) if (!fin(p[k])) return;
  int mode = (int)p[0]; if (mode < 0 || mode > 3) return;
  if (mode > N->sup_mode || (!N->last.fly && !N->landed)) N->sup_mode = mode;   /* in the air it only steps up */
  N->lim_lean = p[1]; N->lim_accel = p[2]; N->lim_speed = p[3];
}

int nav_tune(nav_state *N, const float *p, int n) {
  return pid_frame(&N->tuning, p, n, 1,
                   N->H && N->H->act == 0 && N->H->phase == RN_PH_FLYING && N->H->pending == 0,
                   N->last.fly && !N->sup_mode && !N->rc_rth, !N->last.fly);
}
