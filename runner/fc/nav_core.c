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
  N->C = C; N->have_config = 1; say(N, "nav config loaded");
  return 0;
}

int nav_init(nav_state *N, rn_host *H) {
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

int nav_step(nav_state *N, const nav_in *in, const nav_sp *sp, float dt, nav_out *out) {
  memset(out, 0, sizeof *out);
  out->heading = sp->heading;
  if (!N->ok || !N->have_config || !in->have_att || !(dt > 0)) return 0;
  const nav_config *C = &N->C;
  float R[9], q[4]; float n = sqrtf(in->q[0] * in->q[0] + in->q[1] * in->q[1] + in->q[2] * in->q[2] + in->q[3] * in->q[3]);
  if (!(n > 0.5f)) return 0;
  for (int k = 0; k < 4; k++) q[k] = in->q[k] / n;
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
  for (int i = 0; i < 3; i++) { N->p[i] = N->have_home ? pv[i] - N->home[i] : 0; N->v[i] = pv[3 + i]; }
  memcpy(out->p, N->p, sizeof out->p); memcpy(out->v, N->v, sizeof out->v); out->have_home = N->have_home;
  N->steps++;

  if (!sp->fly || (!N->have_home && !out->ready)) { memset(N->iPos, 0, sizeof N->iPos); return 0; }   /* on the ground (or not ready to leave it): nothing to steer */

  /* where to go: the acceleration toward the target */
  float ep[3], ev[3], vmax = C->speed_max > 0 ? C->speed_max : 6;
  for (int i = 0; i < 3; i++) {
    ep[i] = sp->target[i] - N->p[i];
    ev[i] = N->v[i] - clampf(sp->vref[i], -vmax, vmax);
    N->iPos[i] = clampf(N->iPos[i] + ep[i] * dt, i < 2 ? -2 : -5, i < 2 ? 2 : 5);
  }
  k = 0;
  for (int i = 0; i < 3; i++) b[k++] = ep[i];
  for (int i = 0; i < 3; i++) b[k++] = ev[i];
  for (int i = 0; i < 3; i++) b[k++] = N->iPos[i];
  b[k++] = C->m; b[k++] = G_;
  b[k++] = 0; for (int i = 0; i < 6; i++) b[k++] = 0;   /* lim: none */
  float Fd[3];
  if ((e = call(N, N->f_pc, b, Fd))) return -1;
  for (int i = 0; i < 3; i++) out->acc[i] = Fd[i] / C->m;
  out->acc[2] -= G_;
  out->heading = sp->heading; out->fly = 1;
  return 0;
}
