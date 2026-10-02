/* The learning task: see learn_core.h. A port of the simulator's learn.js, around the same formulas. */
#include "learn_core.h"
#if defined(__wasm__)                 /* the simulator's build: no C library, the page's Math */
typedef __SIZE_TYPE__ size_t;
void *memcpy(void *d, const void *s, size_t n); void *memset(void *d, int c, size_t n);
#define LN_IMPORT(n) __attribute__((import_module("env"), import_name(n)))
LN_IMPORT("sin") double ln_js_sin(double); LN_IMPORT("acos") double ln_js_acos(double);
#define sinf(x) ((float)ln_js_sin(x))
#define acosf(x) ((float)ln_js_acos(x))
#define sqrtf(x) __builtin_sqrtf(x)
#define fabsf(x) __builtin_fabsf(x)
#else
#include <math.h>
#include <string.h>
#endif

#define G_ 9.81f
#define PI_ 3.14159265f
#define D2R 0.0174533f
#define LEARN_MAGIC 0x434C4644u   /* 'DFLC' */
#define MEM_CAL 4.0f              /* forgetting time while calibrating (at least the whole calibration) [s] */
#define MEM_FLIGHT 30.0f          /* and in flight */
#define DITHER 0.02f              /* the little excitation while learning in flight */
#define THROW_AMP 0.5f            /* pulse throttle */
#define THROW_DW 4.0f             /* a pulse stops once the drone turns this much faster [rad/s] */
#define REFINE_BUDGET 50000.0f    /* operations the background lag search may spend per frame */

static int fin(float x) { return (x - x) == 0; }
static float clampf(float x, float a, float b) { return x > a ? (x < b ? x : b) : a; }
static float maxf(float a, float b) { return a > b ? a : b; }
static float minf(float a, float b) { return a < b ? a : b; }
static void qmat(float *R, const float *q) {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - w * z); R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z); R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y); R[7] = 2 * (y * z + w * x); R[8] = 1 - 2 * (x * x + y * y);
}
static void cross(float *o, const float *a, const float *b) { float x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2], z = a[0] * b[1] - a[1] * b[0]; o[0] = x; o[1] = y; o[2] = z; }
static float nrm3(const float *a) { return sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]); }
#define SAY(...) fc_fmt(L->msg, (int)sizeof L->msg, __VA_ARGS__)
#define SAY_MORE(...) fc_fmt_add(L->msg, (int)sizeof L->msg, __VA_ARGS__)

/* ── calling the formulas: inputs flat, in their signature's order (js/rn-sigs.js) ── */
static float pk[2048], res[512];
static int put_list(float *b, int k, const float *v, int len, int cap) { b[k++] = (float)len; for (int i = 0; i < cap; i++) b[k++] = i < len ? v[i] : 0; return k; }
static int call(learn_state *L, int fn, float *out) {
  int e = rn_host_call(L->H, fn, 0, pk, out);
  if (!e) { int n = rn_host_out_size(L->H, fn); for (int k = 0; k < n; k++) if (!fin(out[k])) return -1; }
  return e;
}

int learn_init(learn_state *L, rn_host *H) {
  L->H = H; L->ok = 0;
  const char *names[] = { "identifyEffectiveness", "identifyMotorResponse", "identifyServoResponse", "identifyThrow" };
  int *slot[] = { &L->f_rls, &L->f_mot, &L->f_srv, &L->f_thr };
  const int LI = 1 + LN_IN, LT = 1 + LN_THROW_IN, W = 1 + 4 * 2 * (1 + LN_WIN);
  const int in_sz[] = { LI + 3 + 3 + 3 + 1 + 6 * LI + 1 + 1 + LI + 1 + 4 * LI, W + 1, W + 1, LT + 3 + 3 + 3 + 1 + 1 + 4 * LT + 1 };
  const int out_sz[] = { 6 * LI + 1 + 3 * LI, 4, 4, 6 * LT + 3 * LT + 3 + 1 + 1 + LT + 6 };
  for (int i = 0; i < 4; i++) {
    *slot[i] = rn_host_find(H, names[i]);
    if (*slot[i] < 0 || rn_host_in_size(H, *slot[i]) != in_sz[i] || rn_host_out_size(H, *slot[i]) != out_sz[i]) { SAY("formula %s isn't what the learning expects", names[i]); return -1; }
  }
  L->keep = 1; L->hold_pulses = 1; L->then_cal = 1;
  L->ok = 1; SAY("learning ready");
  return 0;
}

/* ── the airframe and the inputs ── */
static void reset_learning(learn_state *L) {   /* learn.js resetLearning: back to the description (the test results stay) */
  for (int r = 0; r < 6; r++) for (int j = 0; j < LN_IN; j++) L->prior[r][j] = 0;
  for (int i = 0; i < L->FA.A.n_motors; i++) for (int b = 0; b < L->nb[i]; b++) for (int r = 0; r < 6; r++) L->prior[r][L->col0[i] + b] = L->FA.A.mot[i].cols[b][r] * L->m_eff[i];
  memcpy(L->B, L->prior, sizeof L->B); L->prior_desc = 1;
  rn_host_forget(L->H, L->f_rls);
  L->hold_servos = 0; memcpy(L->imu_r, L->C.imu_pos, sizeof L->imu_r);
}
int learn_airframe(learn_state *L, const uint8_t *blob, uint32_t len) {
  if (fc_airframe_load(&L->FA, blob, len)) { SAY("%s", L->FA.why); return -1; }
  int n = 0;
  for (int i = 0; i < L->FA.A.n_motors; i++) { L->col0[i] = n; L->nb[i] = L->FA.A.mot[i].n_basis; n += L->nb[i]; L->m_eff[i] = 1; L->m_meas[i] = 0; }
  for (int j = 0; j < FC_MAX_JOINTS; j++) L->j_meas[j] = 0;
  if (n > LN_IN) { SAY("this airframe has %d inputs; the learning takes %d", n, LN_IN); return -1; }
  L->n = n; L->use_learned = 0; L->cal = 0; L->thr = LN_THR_NONE; L->refining = 0; L->have_fit = 0;
  reset_learning(L); L->model_dirty = 1;
  SAY("Flying on the airframe description%s.", L->keep ? ", learning in flight" : "");
  return 0;
}
int learn_config_parse(learn_config *C, const uint8_t *blob, uint32_t len) {
  if (len < 16) return -1;
  uint32_t crc; memcpy(&crc, blob + len - 4, 4);
  if (rn_crc32(blob, len - 4) != crc) return -1;
  uint32_t magic, ver; memcpy(&magic, blob, 4); memcpy(&ver, blob + 4, 4);
  if (magic != LEARN_MAGIC || ver != 1) return -1;
  int nf = (int)(len - 12) / 4; float f[6 + 6 * FC_MAX_MOTORS + 3]; if (nf > (int)(sizeof f / 4) || nf < 9) return -1;
  memcpy(f, blob + 8, (size_t)nf * 4);
  for (int k = 0; k < nf; k++) if (!fin(f[k])) return -1;
  memset(C, 0, sizeof *C);
  for (int k = 0; k < 3; k++) C->imu_pos[k] = f[k];
  C->hand_h = f[3]; C->ambient = f[4]; C->n = (int)f[5];
  if (C->n < 0 || C->n > FC_MAX_MOTORS || nf != 6 + 6 * C->n + 3) return -1;
  for (int i = 0; i < C->n; i++) { const float *m = f + 6 + 6 * i; C->coll[i] = m[0] > 0.5f; C->tmax[i] = m[1]; C->rw[i] = m[2]; C->om[i] = m[3]; C->ct[i] = m[4]; C->gf[i] = m[5]; }
  const float *b = f + 6 + 6 * C->n; C->cells = (int)b[0]; C->r_int = b[1]; C->batt_tmax = b[2];
  return 0;
}
int learn_config_load(learn_state *L, const uint8_t *blob, uint32_t len) {
  if (learn_config_parse(&L->C, blob, len)) { SAY("learning config: not a valid config"); return -1; }
  L->have_config = 1; memcpy(L->imu_r, L->C.imu_pos, sizeof L->imu_r);
  return 0;
}

/* The columns as the flight core flies them: the learned model (frozen while a calibration runs), or the
 * description scaled by the supervisor's effectiveness. */
static float flown(const learn_state *L, int i, int b, int r) {
  if (L->use_learned) return (L->fly_frozen ? L->flyB : L->B)[r][L->col0[i] + b];
  return L->FA.A.mot[i].cols[b][r] * L->m_eff[i];
}
static void col_now(const learn_state *L, int i, int dm, float *out) {   /* motor i's effect at the believed angles (dm ≥ 0: its change as chain joint dm turns) */
  const fc_motor *M = &L->FA.A.mot[i]; float ang[FC_MAX_CHAIN], b[FC_MAX_BASIS];
  for (int c = 0; c < M->n_chain; c++) ang[c] = L->thh[M->chain[c]];
  int n = fc_basis(b, ang, M->n_chain, dm);
  for (int r = 0; r < 6; r++) { float s = 0; for (int k = 0; k < n && k < M->n_basis; k++) s += b[k] * flown(L, i, k, r); out[r] = s; }
}
static int chain_pos(const fc_motor *M, int j) { for (int c = 0; c < M->n_chain; c++) if (M->chain[c] == j) return c; return -1; }
static void servo_col(const learn_state *L, int j, float *out) {   /* what turning joint j does, at the thrusts now */
  for (int r = 0; r < 6; r++) out[r] = 0;
  for (int i = 0; i < L->FA.A.n_motors; i++) {
    int m = chain_pos(&L->FA.A.mot[i], j); if (m < 0) continue;
    float d[6]; col_now(L, i, m, d); for (int r = 0; r < 6; r++) out[r] += d[r] * L->v[i];
  }
}
/* Inputs as the identification sees them: each motor's believed thrust × its basis at the believed angles. */
static void inputs(const learn_state *L, float *x, float *mv, float *phi, float *mm, float *coll) {
  for (int i = 0; i < L->FA.A.n_motors; i++) {
    const fc_motor *M = &L->FA.A.mot[i]; float ang[FC_MAX_CHAIN], b[FC_MAX_BASIS];
    for (int c = 0; c < M->n_chain; c++) ang[c] = L->thh[M->chain[c]];
    fc_basis(b, ang, M->n_chain, -1);
    for (int k = 0; k < L->nb[i]; k++) {
      int j = L->col0[i] + k;
      x[j] = L->v[i] * b[k]; mv[j] = L->v[i]; phi[j] = b[k]; mm[j] = (float)i; coll[j] = (float)(i < L->C.n && L->C.coll[i]);
    }
  }
}
/* A measured [f; α] along an effect column: each half weighted by its own size (learn.js project). */
static float project(const float *col, const float *y6) {
  float num = 0, den = 0;
  for (int h = 0; h < 2; h++) {
    float cc = 0, cy = 0; for (int i = 3 * h; i < 3 * h + 3; i++) { cc += col[i] * col[i]; cy += col[i] * y6[i]; }
    if (cc > 1e-6f) { num += cy / cc; den += 1; }
  }
  return den > 0 ? num / den : 0;
}
static float tilt_of(const learn_state *L) {   /* how far the nominal thrust axis leans [rad] */
  const float *a = fc_axis(&L->FA);
  return acosf(clampf(L->R[6] * a[0] + L->R[7] * a[1] + L->R[8] * a[2], -1, 1));
}

/* ── the in-flight learning ── */
static int rls_step(learn_state *L, float memory) {
  float x[LN_IN] = { 0 }, mv[LN_IN] = { 0 }, phi[LN_IN] = { 0 }, mm[LN_IN] = { 0 }, coll[LN_IN] = { 0 }, lags[LN_IN];
  inputs(L, x, mv, phi, mm, coll);
  for (int i = 0; i < L->FA.A.n_motors; i++) for (int k = 0; k < L->nb[i]; k++) lags[L->col0[i] + k] = L->m_meas[i] ? L->m_tau[i] : 0.035f;
  int k = put_list(pk, 0, x, L->n, LN_IN);
  for (int i = 0; i < 3; i++) pk[k++] = L->f[i];
  for (int i = 0; i < 3; i++) pk[k++] = L->w[i];
  for (int i = 0; i < 3; i++) pk[k++] = L->imu_r[i];
  pk[k++] = L->dt;
  for (int r = 0; r < 6; r++) k = put_list(pk, k, L->prior[r], L->n, LN_IN);
  pk[k++] = memory;
  pk[k++] = 1; k = put_list(pk, k, lags, L->n, LN_IN);
  pk[k++] = 1; k = put_list(pk, k, coll, L->n, LN_IN); k = put_list(pk, k, mm, L->n, LN_IN); k = put_list(pk, k, phi, L->n, LN_IN); k = put_list(pk, k, mv, L->n, LN_IN);
  if (call(L, L->f_rls, res)) return -1;
  for (int r = 0; r < 6; r++) for (int j = 0; j < L->n; j++) L->B[r][j] = res[r * (1 + LN_IN) + 1 + j];
  return 0;
}
/* The identification's memory fields e, x, y (for scoring on fresh data). */
static int rls_field(learn_state *L, const char *name, float *out, int want) {
  int32_t sz; float *p = rn_host_field(L->H, L->f_rls, name, &sz); if (!p) return 0;
  if (sz == want) { memcpy(out, p, (size_t)want * 4); return want; }
  int len = (int)p[0]; if (len > want) len = want; if (len < 0 || len + 1 > sz) return 0;
  memcpy(out, p + 1, (size_t)len * 4); return len;
}

/* ── calibration ── */
static void add_seg(learn_state *L, int kind, int who, float amp, float dur) {
  if (L->nseg >= LN_SEGS) return;
  ln_seg *s = &L->seg[L->nseg++]; s->kind = kind; s->who = who; s->amp = amp; s->dur = dur; s->after = 0;
}
static void start_calibration(learn_state *L, int from_throw) {
  if (L->use_learned) { memcpy(L->flyB, L->B, sizeof L->flyB); L->fly_frozen = 1; } else L->fly_frozen = 0;   /* keep flying on this while the test runs */
  /* after a throw there is no description to fall back on: start from, and compete against, the throw model */
  if (from_throw && L->use_learned) { memcpy(L->prior, L->B, sizeof L->prior); L->prior_desc = 0; rn_host_forget(L->H, L->f_rls); }
  else reset_learning(L);
  const fc_airframe *A = &L->FA.A;
  L->nseg = 0; L->nwin = 0; L->win_cur = -1;
  add_seg(L, 0, -1, 0, 1);                                               /* settle */
  for (int p = 0; p < 2; p++) for (int i = 0; i < A->n_motors; i++) add_seg(L, 1, i, p ? 0.16f : 0.06f, 0.6f);   /* each motor, a small then a bigger pulse */
  if (L->nseg > 1) L->seg[L->nseg - 1].after = 1;
  int ns = 0;
  for (int j = 0; j < A->n_joints; j++) if (A->jnt[j].steer) { add_seg(L, 2, j, 0.35f * A->jnt[j].range, 0.8f); ns++; }   /* each servo, one way then the other */
  if (ns) L->seg[L->nseg - 1].after = 2;
  for (int j = 0; j < A->n_joints; j++) if (A->jnt[j].steer) add_seg(L, 3, j, 0.3f * A->jnt[j].range, 2.5f);   /* each servo swept */
  add_seg(L, 4, -1, 0.04f, 3);                                           /* everything together */
  add_seg(L, 5, -1, 0.04f, 2);                                           /* validating, on a fresh signal */
  float t0 = 0; for (int s = 0; s < L->nseg; s++) { L->seg[s].t0 = t0; t0 += L->seg[s].dur; }
  L->total = t0; L->cal_t = 0; L->cur = -1; L->gate = -1; L->held = 0; L->waiting = 0; L->from_throw = from_throw && L->use_learned;
  for (int i = 0; i < 6; i++) L->sums_e[i] = L->sums_d[i] = L->sums_y[i] = L->sums_y2[i] = 0;
  L->sums_n = 0; L->cal = 1; L->msg[0] = 0;
}
static void end_calibration(learn_state *L) { L->cal = 0; L->fly_frozen = 0; L->model_dirty = 1; }
static int pc(float v) { return (int)(v * 100 + 0.5f); }
static void finish_calibration(learn_state *L) {
  float e[2] = { 0, 0 }, d[2] = { 0, 0 }, v[2] = { 0, 0 };
  for (int i = 0; i < 6; i++) {
    int h = i < 3 ? 0 : 1; double n = L->sums_n > 0 ? L->sums_n : 1;
    e[h] += (float)L->sums_e[i]; d[h] += (float)L->sums_d[i]; v[h] += (float)(L->sums_y2[i] - L->sums_y[i] * L->sums_y[i] / n);
  }
  L->fit_force = v[0] > 1e-9f ? clampf(1 - e[0] / v[0], 0, 1) : 0; L->fit_rot = v[1] > 1e-9f ? clampf(1 - e[1] / v[1], 0, 1) : 0;
  L->desc_force = v[0] > 1e-9f ? clampf(1 - d[0] / v[0], 0, 1) : 0; L->desc_rot = v[1] > 1e-9f ? clampf(1 - d[1] / v[1], 0, 1) : 0;
  L->have_fit = 1;
  int better = L->fit_rot + 0.5f * L->fit_force > L->desc_rot + 0.5f * L->desc_force + 0.02f;
  int good = L->fit_force > 0.5f && L->fit_rot > 0.6f && better;
  const char *base = L->from_throw ? "the model from the throw" : "the airframe description";
  if (good) { L->use_learned = 1; L->keep = 1; L->hold_servos = 0; }
  else if (L->from_throw) { memcpy(L->B, L->prior, sizeof L->B); rn_host_forget(L->H, L->f_rls); }   /* keep the throw model */
  else L->use_learned = 0;
  end_calibration(L);
  if (good) SAY("Calibrated. On fresh test moves the learned model explains %d%% of the rotation and %d%% of the force (%s: %d%% and %d%%). Flying on the learned model and still learning.",
    pc(L->fit_rot), pc(L->fit_force), base, pc(L->desc_rot), pc(L->desc_force));
  else SAY("Calibration finished. The learned model explains %d%% of the rotation and %d%% of the force, but %s does %s (%d%%, %d%%), so it keeps flying on %s.",
    pc(L->fit_rot), pc(L->fit_force), base, better ? "nearly as well" : "as well or better", pc(L->desc_rot), pc(L->desc_force), L->from_throw ? "that" : "the description");
}
static void fit_motors(learn_state *L) {
  if (!L->hold_pulses) return;
  for (int i = 0; i < L->FA.A.n_motors; i++) {
    int k = 1, nw = 0; float dt = 0.005f;
    for (int w = 0; w < L->nwin && nw < 4; w++) {
      const ln_win *W = &L->win[w]; if (W->kind != 1 || W->who != i || W->bad || W->n < 30) continue;
      k = put_list(pk, k, W->u, W->n, LN_WIN); k = put_list(pk, k, W->y, W->n, LN_WIN); nw++; dt = W->dt;
    }
    if (!nw) continue;
    pk[0] = (float)nw; for (; k < 1 + 4 * 2 * (1 + LN_WIN); k++) pk[k] = 0;
    pk[k++] = dt;
    float o[4]; if (call(L, L->f_mot, o)) continue;   /* tau, curve, gain, fit */
    L->m_fit[i] = o[3];
    if (o[3] < 0.5f || !(o[2] > 0.3f && o[2] < 3)) continue;   /* a poor fit, or a response nothing like the model's */
    L->m_tau[i] = o[0]; L->m_curve[i] = o[1]; L->m_meas[i] = 1;
  }
}
static void fit_servos(learn_state *L) {
  for (int j = 0; j < L->FA.A.n_joints; j++) {
    int k = 1, nw = 0; float dt = 0.005f;
    for (int w = 0; w < L->nwin && nw < 4; w++) {
      const ln_win *W = &L->win[w]; if (W->kind != 2 || W->who != j || W->bad || W->n < 50) continue;
      k = put_list(pk, k, W->u, W->n, LN_WIN); k = put_list(pk, k, W->y, W->n, LN_WIN); nw++; dt = W->dt;
    }
    if (!nw) continue;
    pk[0] = (float)nw; for (; k < 1 + 4 * 2 * (1 + LN_WIN); k++) pk[k] = 0;
    pk[k++] = dt;
    float o[4]; if (call(L, L->f_srv, o)) continue;   /* rate, lag, gain, fit */
    L->j_fit[j] = o[3];
    if (o[3] < 0.4f || !(o[2] > 0.3f && o[2] < 3)) continue;
    L->j_rate[j] = o[0]; L->j_lag[j] = o[1]; L->j_meas[j] = 1; L->model_dirty = 1;
  }
}
/* What the stage asks for at time t into it: fills the EXC frame; returns the recording window's end (0: none). */
static float seg_now(learn_state *L, const ln_seg *s, float t, float *e) {
  const fc_airframe *A = &L->FA.A; int nm = A->n_motors; float *m = e + 6, *sv = e + 6 + nm;
  e[0] = 1; e[1] = e[2] = e[3] = 0;
  switch (s->kind) {
    case 0: e[0] = 0; return 0;
    case 1: {                                                        /* a motor steps up, then down, the rest holding */
      float a = s->amp; m[s->who] = t < 0.02f ? 0 : t < 0.09f ? a : t < 0.16f ? -a : 0;
      e[1] = e[2] = (float)(L->hold_pulses && t < 0.25f);
      return 0.25f;
    }
    case 2: {                                                        /* a servo swings one way, then the other */
      float d = s->amp; sv[s->who] = t < 0.02f ? 0 : t < 0.17f ? d : t < 0.32f ? -d : 0; e[3] = (float)(1 << s->who);
      e[1] = e[2] = (float)(t < 0.4f);
      return 0.4f;
    }
    case 3: sv[s->who] = s->amp * sinf(2 * PI_ * 0.8f * t); e[3] = (float)(1 << s->who); return 0;   /* rides on what the controller asks */
    default: {                                                       /* everything at once: a multisine */
      float f0 = s->kind == 4 ? 2.3f : 3.1f, df = s->kind == 4 ? 1.7f : 1.3f; int k = 0, mask = 0;
      for (int i = 0; i < nm; i++) m[i] = s->amp * sinf(2 * PI_ * (f0 + df * i) * t + i * 1.3f);
      for (int j = 0; j < A->n_joints; j++) if (A->jnt[j].steer) { sv[j] = 0.3f * A->jnt[j].range * sinf(2 * PI_ * (0.6f + 0.35f * k) * t); mask |= 1 << j; k++; }
      e[3] = (float)mask; return 0;
    }
  }
}
static void record(learn_state *L, const ln_seg *s, int si) {
  if (!L->mf_ok) return;
  if (L->win_cur < 0) {
    if (L->nwin >= LN_TESTS) return;
    ln_win *W = &L->win[L->nwin]; L->win_cur = L->nwin++;
    W->kind = s->kind; W->who = s->who; W->bad = 0; W->n = 0; W->dt = L->dt;
    if (s->kind == 1) col_now(L, s->who, -1, W->col); else servo_col(L, s->who, W->col);
    W->cmd0 = s->kind == 2 ? L->thc[s->who] : 0;
    (void)si;
  }
  ln_win *W = &L->win[L->win_cur]; if (W->n >= LN_WIN) return;
  float y6[6] = { L->mf_f[0], L->mf_f[1], L->mf_f[2], L->mf_a[0], L->mf_a[1], L->mf_a[2] };
  W->u[W->n] = W->kind == 1 ? L->u[W->who] : L->thc[W->who] - W->cmd0;
  W->y[W->n] = project(W->col, y6); W->n++;
}
static void calibration_tick(learn_state *L, float *e) {
  float tilt = tilt_of(L);
  if (tilt > 0.52f || nrm3(L->w) > 4) {                              /* tilted past 30° or spinning: pause the excitation */
    L->held = 1; if (L->win_cur >= 0) L->win[L->win_cur].bad = 1; e[0] = 0; return;
  }
  L->held = 0; L->cal_t += L->dt;
  int si = -1; for (int s = 0; s < L->nseg; s++) if (L->cal_t >= L->seg[s].t0 && L->cal_t < L->seg[s].t0 + L->seg[s].dur) { si = s; break; }
  if (L->cur >= 0 && L->cur != si && L->seg[L->cur].after) { if (L->seg[L->cur].after == 1) fit_motors(L); else fit_servos(L); }   /* a test stage just ended: fit it */
  if (si < 0) { finish_calibration(L); e[0] = 0; return; }
  const ln_seg *s = &L->seg[si];
  if (L->cur != si) { L->cur = si; L->win_cur = -1; L->gate = (s->kind == 1 || s->kind == 2) ? 0 : -1; }
  if (L->gate >= 0) {                                                /* wait (closed loop) until the drone is calm, at most 1.5 s */
    int calm = nrm3(L->w) < 0.25f && tilt < 0.14f;
    if (!calm && L->gate < 1.5f) { L->gate += L->dt; L->cal_t -= L->dt; L->waiting = 1; e[0] = 0; return; }
    L->gate = -1;
  }
  L->waiting = 0;
  float tin = L->cal_t - s->t0, until = seg_now(L, s, tin, e);
  if (until > 0 && tin < until) record(L, s, si);
  if (s->kind == 5 && L->updated) {                                  /* score the learned model and the description on the same fresh data */
    float ee[6], x[2 * LN_IN], y[8];
    if (rls_field(L, "e", ee, 6) == 6 && rls_field(L, "x", x, 2 * LN_IN) >= L->n && rls_field(L, "y", y, 6) == 6) {
      L->sums_n++;
      for (int i = 0; i < 6; i++) {
        float yd = 0; for (int j = 0; j < L->n; j++) yd += L->prior[i][j] * x[j];
        L->sums_e[i] += ee[i] * ee[i]; L->sums_d[i] += (y[i] - yd) * (y[i] - yd); L->sums_y[i] += y[i]; L->sums_y2[i] += y[i] * y[i];
      }
    }
  }
}

/* ── the throw start ── */
/* Every motor pulsed with each steering joint above it at three angles in turn (the middle, one end, the other end),
 * so its (1, cos θ, sin θ) columns can all be told apart; passes go motor by motor so a joint can swing to its next
 * angle while other motors pulse. */
static void throw_plan(learn_state *L) {
  const fc_airframe *A = &L->FA.A; ln_pulse var[FC_MAX_MOTORS][1 + 2 * FC_MAX_CHAIN]; int nv[FC_MAX_MOTORS], most = 0;
  for (int i = 0; i < A->n_motors; i++) {
    const fc_motor *M = &A->mot[i]; int js[FC_MAX_CHAIN], k = 0;
    for (int c = 0; c < M->n_chain; c++) if (A->jnt[M->chain[c]].steer) js[k++] = M->chain[c];
    nv[i] = 0;
    ln_pulse base; memset(&base, 0, sizeof base); base.motor = i;
    for (int c = 0; c < k; c++) { base.joint[c] = js[c]; base.mask |= 1 << c; }
    var[i][nv[i]++] = base;                                         /* the middle first: if it has to stop early, it knows the motors at rest */
    if (k) {
      ln_pulse p = base; p.ang[0] = -0.9f * A->jnt[js[0]].range; var[i][nv[i]++] = p;
      for (int c = 0; c < k; c++) {
        float R = 0.9f * A->jnt[js[c]].range;
        if (c == 0) { ln_pulse q = base; q.ang[0] = R; var[i][nv[i]++] = q; }
        else { ln_pulse q = base; q.ang[c] = -R; var[i][nv[i]++] = q; q.ang[c] = R; var[i][nv[i]++] = q; }
      }
    }
    if (nv[i] > most) most = nv[i];
  }
  L->nplan = 0;
  for (int k = 0; k < most; k++) for (int i = 0; i < A->n_motors; i++) if (k < nv[i] && L->nplan < LN_PLAN) L->plan[L->nplan++] = var[i][k];
}
static float pulse_angle(const ln_pulse *P, int j, int *has) { for (int c = 0; c < FC_MAX_CHAIN; c++) if ((P->mask >> c) & 1 && P->joint[c] == j) { *has = 1; return P->ang[c]; } *has = 0; return 0; }
float learn_throw_plan_time(learn_state *L, int *pulses) {   /* each pulse and rest, plus time for a servo to swing between pulses */
  throw_plan(L); float T = 0;
  for (int i = 0; i < L->nplan; i++) {
    T += 0.13f;
    if (i > 0) for (int c = 0; c < FC_MAX_CHAIN; c++) if ((L->plan[i].mask >> c) & 1) {
      int h; float a = pulse_angle(&L->plan[i - 1], L->plan[i].joint[c], &h);
      if (!h || a != L->plan[i].ang[c]) { T += 0.08f; break; }
    }
  }
  if (pulses) *pulses = L->nplan;
  return T;
}
/* Height it needs to catch itself, moving up at vz: spin up and turn upright (~0.45 s, coasting), then brake at about 0.8 g. */
static float throw_room(float vz) { float t = 0.45f, v1 = minf(0, vz - G_ * t); return 0.3f - (vz * t - 0.5f * G_ * t * t) + v1 * v1 / (2 * 0.8f * G_); }
static int throw_call(learn_state *L, int solve, float budget) {   /* identifyThrow; solve 0 fall, 1 catch, 2 refine */
  float x[LN_IN] = { 0 }, mv[LN_IN] = { 0 }, phi[LN_IN] = { 0 }, mm[LN_IN] = { 0 }, coll[LN_IN] = { 0 }, z[LN_IN] = { 0 };
  inputs(L, x, mv, phi, mm, coll);
  int k = 0, refine = solve == 2;
  k = put_list(pk, k, refine ? z : x, L->n, LN_THROW_IN);
  for (int i = 0; i < 3; i++) pk[k++] = refine ? 0 : L->f[i];
  for (int i = 0; i < 3; i++) pk[k++] = refine ? 0 : L->w[i];
  float vb[3] = { L->R[6] * L->vz, L->R[7] * L->vz, L->R[8] * L->vz };   /* body velocity: what it knows of it, the vertical speed */
  for (int i = 0; i < 3; i++) pk[k++] = refine ? 0 : vb[i];
  pk[k++] = refine ? 0 : L->dt; pk[k++] = (float)solve;
  k = put_list(pk, k, coll, L->n, LN_THROW_IN); k = put_list(pk, k, mm, L->n, LN_THROW_IN); k = put_list(pk, k, phi, L->n, LN_THROW_IN); k = put_list(pk, k, mv, L->n, LN_THROW_IN);
  pk[k++] = budget;
  return call(L, L->f_thr, res);
}
/* The falling fit's update on each control step the frame carries (or the frame itself), then the catch's solve. */
static int throw_fall(learn_state *L, int solve) {
  if (!L->nsub) return throw_call(L, solve, 0);
  float f[3], w[3], v[FC_MAX_MOTORS], dt = L->dt; int nm = L->FA.A.n_motors, e = 0;
  memcpy(f, L->f, sizeof f); memcpy(w, L->w, sizeof w); memcpy(v, L->v, sizeof v);
  for (int s = 0; s < L->nsub && !e; s++) {
    const float *q = L->sub + s * (7 + nm);
    L->dt = q[0] > 0 ? q[0] : 0.001f; for (int k = 0; k < 3; k++) { L->f[k] = q[1 + k]; L->w[k] = q[4 + k]; } for (int i = 0; i < nm; i++) L->v[i] = q[7 + i];
    e = throw_call(L, s == L->nsub - 1 ? solve : 0, 0);
  }
  L->dt = dt; memcpy(L->f, f, sizeof f); memcpy(L->w, w, sizeof w); memcpy(L->v, v, sizeof v);
  return e;
}
/* res as identifyThrow returns it: B (6 lists), B2 (3), r, drag, tau, taus (list), fitF, fitR, refined, improved, progress, spent */
#define TL (1 + LN_THROW_IN)
static float res_B(int r, int j) { return res[r * TL + 1 + j]; }
static float res_at(int k) { return res[9 * TL + k]; }   /* 0..2 r, 3 drag, 4 tau, 5.. taus list, then fitF … */
static float res_after(int k) { return res[9 * TL + 5 + TL + k]; }   /* 0 fitF, 1 fitR, 2 refined, 3 improved, 4 progress, 5 spent */
static void lag_text(learn_state *L) {
  int nt = (int)res_at(5); float lo = 1, hi = 0; for (int g = 0; g < nt; g++) { float t = res_at(6 + g); lo = minf(lo, t); hi = maxf(hi, t); }
  if (nt <= 0) { lo = hi = res_at(4); }
  if ((int)(lo * 1000 + 0.5f) == (int)(hi * 1000 + 0.5f)) SAY_MORE("≈ %d ms", (int)(lo * 1000 + 0.5f)); else SAY_MORE("%d–%d ms", (int)(lo * 1000 + 0.5f), (int)(hi * 1000 + 0.5f));
}
/* Use the throw's fit as the model. Motors not pulsed at every servo angle (cut short) keep what the angles tried can
 * tell; returns 1 if some servo's effect is still unknown. */
static int adopt_throw(learn_state *L) {
  float B[6][LN_IN]; int partial = 0;
  for (int r = 0; r < 6; r++) for (int j = 0; j < L->n; j++) B[r][j] = res_B(r, j);
  if (L->cut >= 0) {
    for (int i = 0; i < L->FA.A.n_motors; i++) {
      const fc_motor *M = &L->FA.A.mot[i]; int c0 = L->col0[i]; if (L->nb[i] < 3) continue;
      int done = 0, all = 0; for (int p = 0; p < L->nplan; p++) if (L->plan[p].motor == i) { all++; if (p < L->cut) done++; }
      if (done >= all) continue;
      float tried = 0; int have = 0;
      for (int p = 0; p < L->cut; p++) if (L->plan[p].motor == i) { int h; float a = pulse_angle(&L->plan[p], M->chain[0], &h); if (h && fabsf(a) > 1e-6f) { tried = a; have = 1; break; } }
      for (int r = 0; r < 6; r++) {
        float b0[FC_MAX_BASIS], b1[FC_MAX_BASIS], z[FC_MAX_CHAIN] = { 0 }, a1[FC_MAX_CHAIN] = { tried };
        fc_basis(b0, z, M->n_chain, -1); fc_basis(b1, a1, M->n_chain, -1);
        float v0 = 0, v1 = 0; for (int k = 0; k < L->nb[i]; k++) { v0 += B[r][c0 + k] * b0[k]; v1 += B[r][c0 + k] * b1[k]; }
        if (M->n_chain == 1 && have) { B[r][c0] = v0; B[r][c0 + 1] = 0; B[r][c0 + 2] = (v1 - v0) / sinf(tried); }   /* the middle and one end: its effect, and how it changes with the servo */
        else for (int k = 0; k < L->nb[i]; k++) B[r][c0 + k] = k == 0 ? v0 : 0;    /* only the middle */
      }
      if (!(M->n_chain == 1 && have)) partial = 1;
    }
  }
  memcpy(L->B, B, sizeof B); memcpy(L->prior, B, sizeof B); L->prior_desc = 0; rn_host_forget(L->H, L->f_rls);
  L->use_learned = 1; L->keep = 1; for (int k = 0; k < 3; k++) L->imu_r[k] = res_at(k);
  L->model_dirty = 1;
  return partial;
}
static void finish_throw(learn_state *L) {
  float fitF = res_after(0), fitR = res_after(1); int ok = fitR > 0.6f && fitF > 0.4f, partial = 0;
  L->thr = LN_THR_RECOVER; L->calm_t = 0; L->zmin = L->z; L->refining = 0;
  if (ok) {
    partial = adopt_throw(L);
    if (res_after(2) < 0.5f) { L->refining = 1; L->refine_t0 = L->t; }   /* each motor's own lag, in the background */
    SAY("Identified in %.2f s of free fall: the fit explains %d%% of the rotation and %d%% of the force, motor lag ", (double)L->thr_t, pc(fitR), pc(fitF));
    lag_text(L); SAY_MORE(", IMU %.1f cm from the balance point.", (double)(100 * nrm3(L->imu_r)));
  } else {
    L->use_learned = 0; L->model_dirty = 1;
    SAY("The free-fall fit was poor (rotation %d%%, force %d%%), so it catches itself on the airframe description instead.", pc(fitR), pc(fitF));
  }
  if (L->cut >= 0) {
    int sj = 0; for (int j = 0; j < L->FA.A.n_joints; j++) if (L->FA.A.jnt[j].steer) sj = 1;
    SAY_MORE(" It stopped after %d of %d pulses to leave room to catch itself%s.", L->cut, L->nplan, sj && partial ? ", so it holds its servos in the middle until a calibration has measured them" : "");
    if (sj && ok && partial) L->hold_servos = 1;
  }
  SAY_MORE(" Recovering…");
}
static void refine_step(learn_state *L) {
  if (!L->refining) return;
  if (throw_call(L, 2, REFINE_BUDGET)) { L->refining = 0; return; }
  if (res_after(2) < 0.5f) return;
  L->refining = 0;
  int nt = (int)res_at(5), differ = 0; for (int g = 1; g < nt; g++) if (res_at(6 + g) != res_at(6)) differ = 1;
  int better = differ && res_after(3) > 0.5f;
  SAY_MORE(" Worked out each motor's own lag in the background (");
  lag_text(L); SAY_MORE(", %.1f s)", (double)(L->t - L->refine_t0));
  if (better && !L->cal && L->use_learned) { int partial = adopt_throw(L); L->hold_servos = L->hold_servos && partial; SAY_MORE("; it now explains %d%% of the fall's rotation and flies on that.", pc(res_after(1))); }
  else SAY_MORE(better ? "; the calibration running now supersedes it." : differ ? "; it didn't fit the fall any better, so nothing changed." : "; all the motors share one lag, so nothing changed.");
}
static void throw_tick(learn_state *L, float *e) {
  const fc_airframe *A = &L->FA.A; int nm = A->n_motors; float *m = e + 6, *sv = e + 6 + nm;
  /* where it is: height above the ground from the barometer (from where the hand held it), else the vertical speed */
  if (L->thr == LN_THR_HAND) { L->h0 = L->have_alt ? L->alt : 0; L->z = L->C.hand_h; }
  else L->z = L->have_alt ? L->C.hand_h + L->alt - L->h0 : L->z + L->vz * L->dt;
  L->zmax = maxf(L->zmax, L->z);
  e[0] = 2; e[1] = e[2] = 0;
  /* each steering joint heads for the angle of the next pulse that needs it */
  int mask = 0, from = L->thr == LN_THR_EXCITE && L->step == 2 ? L->pi + 1 : L->pi;
  for (int j = 0; j < A->n_joints; j++) if (A->jnt[j].steer) {
    float a = 0; for (int p = from; p < L->nplan; p++) { int h; float x = pulse_angle(&L->plan[p], j, &h); if (h) { a = x; break; } }
    sv[j] = a; mask |= 1 << j;
  }
  e[3] = (float)mask;
  if (L->thr == LN_THR_HAND) {                                       /* released: the accelerometer feels nothing (free fall) */
    L->low_t = nrm3(L->f) < 3 ? L->low_t + L->dt : 0;
    if (L->low_t >= 0.01f) { L->thr = LN_THR_FREE; L->thr_t = 0; L->zmax = L->z;
      SAY("Thrown. Near the top of the arc it pulses each motor on its own and fits what each one does from the gyro and accelerometer."); }
    return;
  }
  L->thr_t += L->dt;
  float tplan = L->tplan;
  /* start so the pulses finish just past the top: on the way down it soon needs its height to recover */
  if (L->thr == LN_THR_FREE && L->thr_t > 0.06f && L->vz < G_ * (tplan - 0.1f)) { L->thr = LN_THR_EXCITE; L->ts = L->thr_t; L->pi = 0; L->step = 0; }
  if (L->thr == LN_THR_FREE) { throw_fall(L, 0); return; }
  /* pulsing */
  float el = L->thr_t - L->ts;
  if (L->pi < L->nplan) {
    const ln_pulse *P = &L->plan[L->pi];
    if (L->step == 0) {                                              /* only waits if a joint isn't at its pulse angle yet */
      int ready = 1; for (int c = 0; c < FC_MAX_CHAIN; c++) if ((P->mask >> c) & 1 && fabsf(L->thh[P->joint[c]] - P->ang[c]) > 2 * D2R) ready = 0;
      if (ready || el > 0.15f) { L->step = 1; L->ts = L->thr_t; memcpy(L->w0, L->w, sizeof L->w0); }
    } else if (L->step == 1) {                                       /* pulse until the rotation it causes reaches THROW_DW, 12–80 ms */
      float d[3] = { L->w[0] - L->w0[0], L->w[1] - L->w0[1], L->w[2] - L->w0[2] };
      if (((L->flags & 16) && el > 0.015f) || (el > 0.012f && nrm3(d) > THROW_DW) || el > 0.08f) { L->step = 2; L->ts = L->thr_t; } else m[P->motor] = THROW_AMP;
    } else if (el > 0.035f) { L->pi++; L->step = 0; L->ts = L->thr_t; }
  }
  /* stop early if it has to start catching itself: room to spin the motors up, turn upright and brake */
  e[6 + nm + A->n_joints] = (float)(L->pi * 2 + (L->step == 1));   /* the pulse's number: the flight core cuts it itself once it has turned the drone enough */
  e[7 + nm + A->n_joints] = THROW_DW;
  if (L->pi < L->nplan && L->cut < 0 && L->z < throw_room(L->vz)) L->cut = L->pi;
  int done = L->pi >= L->nplan || L->cut >= 0;
  if (throw_fall(L, done)) { done = 1; res[9 * TL + 5 + TL + 1] = 0; res[9 * TL + 5 + TL] = 0; }
  if (done) { finish_throw(L); e[0] = 0; }
}
static void recover_check(learn_state *L) {                          /* upright and calm for half a second: caught */
  L->z = L->have_alt ? L->C.hand_h + L->alt - L->h0 : L->z + L->vz * L->dt;
  const float *a = fc_axis(&L->FA);
  float up = L->R[6] * a[0] + L->R[7] * a[1] + L->R[8] * a[2];
  L->zmin = minf(L->zmin, L->z);
  L->calm_t = up > 0.9f && nrm3(L->w) < 1.5f && fabsf(L->vz) < 1 ? L->calm_t + L->dt : 0;
  if (L->calm_t <= 0.5f) return;
  SAY_MORE(" Caught itself %.1f s after release (thrown to about %.1f m, lowest point on the way down %.1f m).", (double)L->thr_t, (double)L->zmax, (double)L->zmin);
  L->thr = LN_THR_NONE;
  if (L->use_learned && L->then_cal) { char keep[sizeof L->msg]; memcpy(keep, L->msg, sizeof keep); start_calibration(L, 1); memcpy(L->msg, keep, sizeof keep); SAY_MORE(" Now refining it with a hover calibration…"); }
  else SAY_MORE(" Still learning in flight.");
}

/* ── each telemetry frame ── */
void learn_ltel(learn_state *L, const float *p, int n) {
  const fc_airframe *A = &L->FA.A; int nm = A->n_motors, nj = A->n_joints;
  L->exc_n = 0;
  const int base = FC_LT_N + 2 * nm + 2 * nj;
  if (!L->ok || !L->FA.have_airframe || n < base + 1 || (int)p[FC_LT_N - 2] != nm || (int)p[FC_LT_N - 1] != nj) return;
  L->nsub = (int)p[base]; L->sub = p + base + 1;
  if (L->nsub < 0 || L->nsub > FC_SUB || n != base + 1 + L->nsub * (7 + nm)) return;
  for (int k = 0; k < n; k++) if (!fin(p[k])) return;
  double t = p[0]; L->dt = L->got ? (float)(t - L->t_last) : 0.005f; L->t_last = t; L->got = 1; L->t = (float)t;
  if (!(L->dt > 0) || L->dt > 0.05f) L->dt = 0.005f;
  L->state = (int)p[1]; L->flags = (int)p[2];
  float qn = 0; for (int k = 0; k < 4; k++) { L->q[k] = p[3 + k]; qn += L->q[k] * L->q[k]; }
  if (qn > 0.25f) { qn = sqrtf(qn); for (int k = 0; k < 4; k++) L->q[k] /= qn; } else { L->q[0] = 1; L->q[1] = L->q[2] = L->q[3] = 0; }
  qmat(L->R, L->q);
  for (int k = 0; k < 3; k++) { L->f[k] = p[7 + k]; L->w[k] = p[10 + k]; }
  L->alt = p[14]; L->vz = p[15]; L->have_alt = p[16] > 0.5f;
  for (int i = 0; i < nm; i++) { L->u[i] = p[FC_LT_N + i]; L->v[i] = p[FC_LT_N + nm + i]; }
  for (int j = 0; j < nj; j++) { L->thc[j] = p[FC_LT_N + 2 * nm + j]; L->thh[j] = p[FC_LT_N + 2 * nm + nj + j]; }
  L->frames++;
  /* filtered readings for the tests: 25 Hz, the IMU's swing around the hub taken out */
  { float k = L->dt / (L->dt + 1 / (2 * PI_ * 25));
    if (!L->mf_ok) { memcpy(L->mf_w, L->w, sizeof L->mf_w); memcpy(L->mf_f, L->f, sizeof L->mf_f); L->mf_a[0] = L->mf_a[1] = L->mf_a[2] = 0; L->mf_ok = 1; }
    float wp[3]; memcpy(wp, L->mf_w, sizeof wp);
    for (int i = 0; i < 3; i++) { L->mf_w[i] += k * (L->w[i] - L->mf_w[i]); L->mf_a[i] = (L->mf_w[i] - wp[i]) / L->dt; }
    float ar[3], wr[3], wwr[3]; cross(ar, L->mf_a, L->imu_r); cross(wr, L->mf_w, L->imu_r); cross(wwr, L->mf_w, wr);
    for (int i = 0; i < 3; i++) L->mf_f[i] += k * (L->f[i] - ar[i] - wwr[i] - L->mf_f[i]); }

  float *e = L->exc; for (int k = 0; k < 8 + nm + nj; k++) e[k] = 0;
  e[4] = (float)nm; e[5] = (float)nj;
  int flying = L->flags & 1, crashed = L->state == FC_CRASHED;
  if (crashed && L->cal) { end_calibration(L); SAY("Calibration stopped: the drone crashed."); }
  if (crashed && L->thr) { L->thr = LN_THR_NONE; SAY_MORE(" It crashed."); }
  if (L->thr == LN_THR_HAND || L->thr == LN_THR_FREE || L->thr == LN_THR_EXCITE) {   /* the throw runs open loop, with its own identification */
    if (L->state == FC_ARMED) throw_tick(L, e); else e[0] = 0;
  } else {
    refine_step(L);
    if (L->thr == LN_THR_RECOVER) { L->thr_t += L->dt; recover_check(L); }
    L->updated = 0;
    if ((L->keep || L->cal) && flying && !(L->flags & 2) && L->n) {   /* the in-flight learning, on every frame (200 Hz) */
      float mem = L->cal ? maxf(MEM_CAL, L->total) : MEM_FLIGHT;
      if (!rls_step(L, mem)) L->updated = 1;
    }
    if (L->cal && flying) calibration_tick(L, e);
    else if (L->keep && flying && !L->cal && L->thr == LN_THR_NONE) {   /* a little excitation, so there's always something to learn from */
      for (int i = 0; i < nm; i++) { int c = L->col0[i]; e[6 + i] = DITHER * sinf(2 * PI_ * (2.7f + 1.9f * c) * L->t + c); }
      e[0] = 1;
    }
  }
  if (e[0] > 0) { L->exc_n = 8 + nm + nj; L->exc_was = 1; }
  else if (L->exc_was) { e[0] = 0; L->exc_n = 8 + nm + nj; L->exc_was = 0; }   /* one last frame: excitation over */
}

void learn_set(learn_state *L, const float *p, int n) {
  int nm = L->FA.A.n_motors;
  if (n < 6 || (int)p[4] != nm || n != 6 + 3 * nm + 2 * (int)p[5]) return;
  for (int i = 0; i < nm; i++) {
    float eff = p[7 + 3 * i]; if (!(eff > 0.05f) || fabsf(eff - L->m_eff[i]) < 1e-3f) continue;
    float s = eff / L->m_eff[i]; L->m_eff[i] = eff;
    for (int r = 0; r < 6; r++) for (int k = 0; k < L->nb[i]; k++) { int j = L->col0[i] + k; L->B[r][j] *= s; L->flyB[r][j] *= s; L->prior[r][j] *= s; }
    /* the learning goes on from the rescaled table (its covariance starts again) */
    L->prior_desc = 0; rn_host_forget(L->H, L->f_rls); if (L->use_learned) L->model_dirty = 1;
  }
  fc_set(&L->FA, p, n);
}

int learn_exc_frame(learn_state *L, float *out) { if (!L->exc_n) return 0; memcpy(out, L->exc, (size_t)L->exc_n * 4); return L->exc_n; }

int learn_model_frame(learn_state *L, float *out) {
  const fc_airframe *A = &L->FA.A;
  if (!A->n_motors) return 0;
  int periodic = L->use_learned && !L->fly_frozen && (L->keep || L->cal) && L->t - L->model_t > 0.2;
  if (!L->model_dirty && !periodic) return 0;
  L->model_dirty = 0; L->model_t = L->t;
  int k = 0; float (*M)[LN_IN] = L->fly_frozen ? L->flyB : L->B;
  out[k++] = (float)L->use_learned; out[k++] = (float)L->hold_servos; out[k++] = (float)A->n_motors; out[k++] = (float)A->n_joints;
  for (int i = 0; i < A->n_motors; i++) { out[k++] = (float)L->nb[i]; for (int b = 0; b < L->nb[i]; b++) for (int r = 0; r < 6; r++) out[k++] = M[r][L->col0[i] + b]; }
  for (int j = 0; j < A->n_joints; j++) { out[k++] = L->j_meas[j] ? L->j_rate[j] : 0; out[k++] = L->j_meas[j] ? L->j_lag[j] : 0; }
  for (int i = 0; i < A->n_motors; i++) out[k++] = -1;
  fc_model(&L->FA, out, k);                                          /* (its own copy, for the axis it flies on) */
  return k;
}

int learn_command(learn_state *L, int cmd) {
  int flying = L->flags & 1;
  switch (cmd) {
    case LN_CMD_CALIBRATE:
      if (!L->n) return -1;
      if (L->thr && L->thr != LN_THR_RECOVER) { SAY("It can calibrate once it has caught itself."); return -1; }
      if (!flying) { SAY("It calibrates while hovering: take off first."); return -1; }
      start_calibration(L, 0); return 0;
    case LN_CMD_STOP: if (L->cal) { end_calibration(L); SAY("Calibration stopped. The model keeps what it learned so far."); } return 0;
    case LN_CMD_USE_DESC: if (L->use_learned) { L->use_learned = 0; L->model_dirty = 1; } return 0;
    case LN_CMD_USE_LEARNED: if (!L->use_learned) { L->use_learned = 1; L->model_dirty = 1; } return 0;
    case LN_CMD_KEEP_ON: L->keep = 1; return 0;
    case LN_CMD_KEEP_OFF: L->keep = 0; return 0;
    case LN_CMD_HOLD_PULSES_ON: L->hold_pulses = 1; return 0;
    case LN_CMD_HOLD_PULSES_OFF: L->hold_pulses = 0; return 0;
    case LN_CMD_THEN_CAL_ON: L->then_cal = 1; return 0;
    case LN_CMD_THEN_CAL_OFF: L->then_cal = 0; return 0;
    case LN_CMD_THROW:
      if (L->n > LN_THROW_IN) { SAY("The throw start identifies at most %d inputs; this airframe has %d.", LN_THROW_IN, L->n); return -1; }
      if (L->cal) end_calibration(L);
      L->tplan = learn_throw_plan_time(L, 0); L->thr = LN_THR_HAND; L->pi = 0; L->step = 0; L->cut = -1; L->low_t = 0; L->thr_t = 0; L->refining = 0;
      L->use_learned = 0; L->hold_servos = 0; L->model_dirty = 1; L->zmax = L->zmin = L->C.hand_h;
      rn_host_forget(L->H, L->f_thr);
      SAY("In the hand, motors off. The drone knows its sensors and how many actuators it has, nothing else.");
      return 0;
  }
  return -1;
}

/* For the screen: [0] on the learned model, [1] keep learning, [2] calibrating, [3] its progress 0–1, [4] paused or
 * waiting for calm, [5] stage kind, [6] stage motor/joint, [7] seconds left, [8] throw phase, [9] throw progress, [10]
 * pulse motor, [11] have a validation, [12–15] fit rotation, force, description's rotation, force, [16] background lag
 * search running, [17] inputs, [18] hold pulses, [19] calibrate after a throw, [20] servos held; from 24: per motor
 * measured, lag, curve bend, fit; per joint measured, speed, lag, fit; then the learned columns, 6 rows × inputs. */
int learn_status(const learn_state *L, float *o) {
  const fc_airframe *A = &L->FA.A; int k = 0;
  for (; k < 24; k++) o[k] = 0;
  o[0] = (float)L->use_learned; o[1] = (float)L->keep; o[2] = (float)L->cal; o[3] = L->cal && L->total > 0 ? L->cal_t / L->total : 0;
  o[4] = (float)(L->held || L->waiting); o[5] = L->cal && L->cur >= 0 ? (float)L->seg[L->cur].kind : -1; o[6] = L->cal && L->cur >= 0 ? (float)L->seg[L->cur].who : -1;
  o[7] = L->cal ? maxf(0, L->total - L->cal_t) : 0; o[8] = (float)L->thr;
  o[9] = L->thr == LN_THR_HAND ? 0 : L->thr == LN_THR_FREE ? 0.1f : L->thr == LN_THR_EXCITE ? 0.1f + 0.7f * (float)L->pi / (float)(L->nplan > 0 ? L->nplan : 1) : L->thr == LN_THR_RECOVER ? 0.9f : 0;
  o[10] = L->thr == LN_THR_EXCITE && L->pi < L->nplan ? (float)L->plan[L->pi].motor : -1;
  o[11] = (float)L->have_fit; o[12] = L->fit_rot; o[13] = L->fit_force; o[14] = L->desc_rot; o[15] = L->desc_force;
  o[16] = (float)L->refining; o[17] = (float)L->n; o[18] = (float)L->hold_pulses; o[19] = (float)L->then_cal; o[20] = (float)L->hold_servos;
  o[21] = (float)L->pi; o[22] = (float)L->step; o[23] = (float)L->nplan;
  for (int i = 0; i < A->n_motors; i++) { o[k++] = (float)L->m_meas[i]; o[k++] = L->m_tau[i]; o[k++] = L->m_curve[i]; o[k++] = L->m_fit[i]; }
  for (int j = 0; j < A->n_joints; j++) { o[k++] = (float)L->j_meas[j]; o[k++] = L->j_rate[j]; o[k++] = L->j_lag[j]; o[k++] = L->j_fit[j]; }
  for (int r = 0; r < 6; r++) for (int j = 0; j < L->n; j++) o[k++] = L->B[r][j];
  return k;
}
