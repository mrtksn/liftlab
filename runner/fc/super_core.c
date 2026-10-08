/* The health supervisor: see super_core.h. A port of the simulator's health.js supervisor, around the same formulas. */
#include "super_core.h"
#if defined(__wasm__)
typedef __SIZE_TYPE__ size_t;
void *memcpy(void *d, const void *s, size_t n); void *memset(void *d, int c, size_t n);
#define sqrtf(x) __builtin_sqrtf(x)
#define fabsf(x) __builtin_fabsf(x)
#else
#include <math.h>
#include <string.h>
#endif

#define PI_ 3.14159265f
#define G_ 9.81f
#define SUP_DT 0.1f                       /* it checks 10 times a second */
#define SUP_MEMORY 8.0f                   /* actuatorHealth: how long the normal gap is averaged [s] */

static int fin(float x) { return (x - x) == 0; }
static float clampf(float x, float a, float b) { return x > a ? (x < b ? x : b) : a; }
static float maxf(float a, float b) { return a > b ? a : b; }
static float minf(float a, float b) { return a < b ? a : b; }
static void cross(float *o, const float *a, const float *b) { float x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2], z = a[0] * b[1] - a[1] * b[0]; o[0] = x; o[1] = y; o[2] = z; }
static void qmat(float *R, const float *q) {
  float w = q[0], x = q[1], y = q[2], z = q[3];
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - w * z); R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z); R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y); R[7] = 2 * (y * z + w * x); R[8] = 1 - 2 * (x * x + y * y);
}

static float pk[2304], res[256];
static int call(super_state *S, int fn, float *out) {
  int e = rn_host_call(S->H, fn, 0, pk, out);
  if (!e) { int n = rn_host_out_size(S->H, fn); for (int k = 0; k < n; k++) if (!fin(out[k])) return -1; }
  return e;
}
static void event(super_state *S, int tone, const char *fmt, ...);

int super_init(super_state *S, rn_host *H) {
  S->H = H; S->ok = 0;
  const char *names[] = { "actuatorHealth", "faultDecision", "flightPolicy", "liftMargin", "thermalModel" };
  int *slot[] = { &S->f_ah, &S->f_fd, &S->f_fp, &S->f_lm, &S->f_th };
  const int NM = FC_MAX_MOTORS, NJ = FC_MAX_JOINTS, NI = NM + NJ + 1;
  const int in_sz[] = { 1 + SP_BATCH * ((1 + 6 * NM) + (1 + 6 * NJ) + 6) + 2, (1 + 10 * NM) + (1 + 5 * NJ) + 1, 15 + 5, (1 + 6 * NI) + 2 * (1 + NI), 6 };
  const int out_sz[] = { 2 * (1 + NM) + 2 * (1 + NJ), (1 + 6 * NM) + (1 + 4 * NJ), 7, 3, 1 };
  for (int i = 0; i < 5; i++) {
    *slot[i] = rn_host_find(H, names[i]);
    if (*slot[i] < 0 || rn_host_in_size(H, *slot[i]) != in_sz[i] || rn_host_out_size(H, *slot[i]) != out_sz[i]) { fc_fmt(S->why_text, (int)sizeof S->why_text, "formula %s isn't what the supervisor expects", names[i]); return -1; }
  }
  S->ok = 1; fc_fmt(S->why_text, (int)sizeof S->why_text, "supervisor ready");
  return 0;
}
static void reset_decisions(super_state *S) {
  for (int i = 0; i < FC_MAX_MOTORS; i++) { S->m_on[i] = 1; S->m_eff[i] = 1; S->m_cap[i] = 1; S->m_state[i] = 0; S->m_why[i] = 0; S->m_val[i] = 0; S->m_temp[i] = -999; S->m_eta[i] = 1; S->m_conf[i] = 0; S->logged_cap[i] = 0; S->have_t_est[i] = 0; }
  for (int j = 0; j < FC_MAX_JOINTS; j++) { S->j_off[j] = 0; S->j_ang[j] = 0; S->j_why[j] = 0; S->j_val[j] = 0; }
  S->mode = 0; S->why = 0; S->have_pol = 0; S->rp_bad = 0; S->v_bad = 0; S->lim_speed = 0; S->lim_lean = 35; S->lim_accel = 6;
  S->cells = S->C.cells > 0 ? S->C.cells : 4; S->cell_lost = 0; S->nvh = 0; S->nb = 0; S->margin = 0; S->have_soc = 0;
  rn_host_forget(S->H, S->f_ah); rn_host_forget(S->H, S->f_fd);
}
int super_airframe(super_state *S, const uint8_t *blob, uint32_t len) {
  if (fc_airframe_load(&S->FA, blob, len)) { fc_fmt(S->why_text, (int)sizeof S->why_text, "%s", S->FA.why); return -1; }
  reset_decisions(S);
  return 0;
}
int super_config_load(super_state *S, const uint8_t *blob, uint32_t len) {
  if (learn_config_parse(&S->C, blob, len)) { fc_fmt(S->why_text, (int)sizeof S->why_text, "supervisor config: not a valid config"); return -1; }
  S->have_config = 1; S->cells = S->C.cells > 0 ? S->C.cells : 4;
  return 0;
}
void super_model(super_state *S, const float *p, int n) { if (S->FA.have_airframe) fc_model(&S->FA, p, n); }

/* ── the table as flown ── */
static float col_k(const super_state *S, int i, int k, int r) { return S->FA.use_learned ? S->FA.lcols[i][k][r] : S->FA.A.mot[i].cols[k][r] * S->FA.m_eff[i]; }
static void col_now(const super_state *S, int i, int dm, float *out) {
  const fc_motor *M = &S->FA.A.mot[i]; float ang[FC_MAX_CHAIN], b[FC_MAX_BASIS];
  for (int c = 0; c < M->n_chain; c++) ang[c] = S->thh[M->chain[c]];
  int n = fc_basis(b, ang, M->n_chain, dm);
  for (int r = 0; r < 6; r++) { float s = 0; for (int k = 0; k < n && k < M->n_basis; k++) s += b[k] * col_k(S, i, k, r); out[r] = s; }
}
static int chain_pos(const fc_motor *M, int j) { for (int c = 0; c < M->n_chain; c++) if (M->chain[c] == j) return c; return -1; }
static int steer_list(const super_state *S, int *sj) { int n = 0; for (int j = 0; j < S->FA.A.n_joints; j++) if (S->FA.A.jnt[j].steer) sj[n++] = j; return n; }

/* ── each telemetry frame ── */
void super_ltel(super_state *S, const float *p, int n) {
  const fc_airframe *A = &S->FA.A; int nm = A->n_motors, nj = A->n_joints;
  if (!S->ok || !S->FA.have_airframe || n < FC_LT_N + 2 * nm + 2 * nj + 1 || (int)p[FC_LT_N - 2] != nm) return;
  for (int k = 0; k < n; k++) if (!fin(p[k])) return;
  double t = fc_ltel_unwrap(S->t_last, S->got, p[0]); S->dt = S->got ? (float)(t - S->t_last) : 0.005f; S->t_last = t; S->got = 1; S->t = t;
  if (!(S->dt > 0) || S->dt > 0.05f) S->dt = 0.005f;
  S->state = (int)p[1]; int flags = (int)p[2];
  float qn = 0; for (int k = 0; k < 4; k++) { S->q[k] = p[3 + k]; qn += S->q[k] * S->q[k]; }
  if (qn > 0.25f) { qn = sqrtf(qn); for (int k = 0; k < 4; k++) S->q[k] /= qn; } else { S->q[0] = 1; S->q[1] = S->q[2] = S->q[3] = 0; }
  qmat(S->R, S->q);
  for (int k = 0; k < 3; k++) { S->f[k] = p[7 + k]; S->w[k] = p[10 + k]; }
  S->alt = p[14]; S->have_alt = p[16] > 0.5f;
  for (int i = 0; i < nm; i++) S->v[i] = p[FC_LT_N + nm + i];
  for (int j = 0; j < nj; j++) S->thh[j] = p[FC_LT_N + 2 * nm + nj + j];
  S->open = (flags & 2) != 0; if (S->open) S->open_t = t;
  int flying = (flags & 1) && !S->open;
  if (flying && !S->flying) { S->fly_t0 = t; S->alt0 = S->alt; }
  S->flying = flying;
  /* filtered readings: 25 Hz, the IMU's swing around the hub taken out (as the learning's) */
  { float k = S->dt / (S->dt + 1 / (2 * PI_ * 25)); const float *r = S->C.imu_pos;
    if (!S->mf_ok) { memcpy(S->mf_w, S->w, sizeof S->mf_w); memcpy(S->mf_f, S->f, sizeof S->mf_f); S->mf_a[0] = S->mf_a[1] = S->mf_a[2] = 0; S->mf_ok = 1; }
    float wp[3]; memcpy(wp, S->mf_w, sizeof wp);
    for (int i = 0; i < 3; i++) { S->mf_w[i] += k * (S->w[i] - S->mf_w[i]); S->mf_a[i] = (S->mf_w[i] - wp[i]) / S->dt; }
    float ar[3], wr[3], wwr[3]; cross(ar, S->mf_a, r); cross(wr, S->mf_w, r); cross(wwr, S->mf_w, wr);
    for (int i = 0; i < 3; i++) S->mf_f[i] += k * (S->f[i] - ar[i] - wwr[i] - S->mf_f[i]); }
  /* the stream: 50 times a second while flying (clear of the ground, and not just after a throw or take-off) */
  if (++S->nframe % 4 == 0 && !S->external_load && flying && t - S->fly_t0 > 1.5 && t - S->open_t > 3 && (!S->have_alt || S->alt - S->alt0 > 0.35f) && S->nb < SP_BATCH) {
    sp_sample *s = &S->batch[S->nb++]; int sj[FC_MAX_JOINTS], ns = steer_list(S, sj);
    for (int i = 0; i < nm; i++) { float c[6]; col_now(S, i, -1, c); for (int r = 0; r < 6; r++) s->phi[i][r] = c[r] * S->v[i]; s->cmd[i] = S->v[i]; }
    for (int k = 0; k < ns; k++) {
      for (int r = 0; r < 6; r++) s->psi[k][r] = 0;
      for (int i = 0; i < nm; i++) { int m = chain_pos(&A->mot[i], sj[k]); if (m < 0) continue; float d[6]; col_now(S, i, m, d); for (int r = 0; r < 6; r++) s->psi[k][r] += d[r] * S->v[i]; }
    }
    for (int k = 0; k < 3; k++) { s->y[k] = S->mf_f[k]; s->y[3 + k] = S->mf_a[k]; }
  }
}

void super_health(super_state *S, const float *p, int n) {
  int nm = S->FA.A.n_motors;
  if (n != 1 + 6 * nm + 6 || (int)p[0] != nm) return;
  for (int i = 0; i < nm; i++) { const float *m = p + 1 + 6 * i;
    S->h_has_t[i] = m[0] > 0.5f; S->h_t[i] = m[1]; S->h_has_rpm[i] = m[2] > 0.5f; S->h_rpm[i] = m[3]; S->h_has_i[i] = m[4] > 0.5f; S->h_i[i] = m[5]; }
  const float *b = p + 1 + 6 * nm;
  S->b_has_v = b[0] > 0.5f; S->b_v = b[1]; S->b_has_i = b[2] > 0.5f; S->b_i = b[3]; S->b_has_t = b[4] > 0.5f; S->b_t = b[5];
}

/* ── the check, 10 times a second ── */
static const char *name_m(const super_state *S, int i) { return S->FA.A.mot[i].name; }
void super_why_motor(const super_state *S, int i, char *o, int size) {
  switch (S->m_why[i]) {
    case 1: fc_fmt(o, size, "the ESC reports it has stopped"); break;
    case 2: fc_fmt(o, size, "it no longer moves the drone"); break;
    case 3: fc_fmt(o, size, "it delivers %d%% of what its table says", (int)(S->m_val[i] * 100 + 0.5f)); break;
    case 4: fc_fmt(o, size, "running at %d °C (limit %d)", (int)(S->m_val[i] + 0.5f), (int)(i < S->C.n ? S->C.tmax[i] : 120)); break;
    default: o[0] = 0;
  }
}
static void why_joint(const super_state *S, int j, char *o, int size) {
  if (S->j_why[j] == 1) fc_fmt(o, size, "it reports it isn't following its commands");
  else fc_fmt(o, size, "it isn't where it was told to go (%d° off)", (int)(S->j_val[j] * 57.2958f + (S->j_val[j] < 0 ? -0.5f : 0.5f)));
}
void super_why_mode(const super_state *S, char *o, int size) {
  const char *t[] = { "", "a motor is running hot", "the battery is hot", "", "a motor has failed", "", "a battery cell has failed", "battery below 20%",
    "battery voltage low", "roll and pitch can no longer be held", "not enough lift to stay up", "battery nearly empty", "battery overheating" };
  if (S->why == 3) fc_fmt(o, size, "lift margin %.2f×", (double)S->why_val);
  else if (S->why == 5) fc_fmt(o, size, "lift margin only %.2f×", (double)S->why_val);
  else fc_fmt(o, size, "%s", S->why >= 0 && S->why <= 12 ? t[S->why] : "");
}
static void event(super_state *S, int tone, const char *fmt, ...) {
  for (int k = SP_LOG - 1; k > 0; k--) S->log[k] = S->log[k - 1];
  sp_event *e = &S->log[0]; e->t = S->t; e->tone = tone; int at = 0;
  va_list ap; va_start(ap, fmt); fc_vfmt(e->text, (int)sizeof e->text, &at, fmt, ap); va_end(ap);
  if (S->nlog < SP_LOG) S->nlog++;
  S->log_seq++;
}


/* A LiPo cell's charge from its resting voltage (the usual discharge curve: flat through the middle, falling away
 * below about 15%; the simulator's batteryModel uses the same one). */
static float lipo_charge(float v) {
  static const float c[][2] = { { 0, 3.2f }, { 0.05f, 3.45f }, { 0.1f, 3.6f }, { 0.15f, 3.67f }, { 0.2f, 3.71f }, { 0.3f, 3.75f }, { 0.4f, 3.79f },
    { 0.5f, 3.83f }, { 0.6f, 3.87f }, { 0.7f, 3.92f }, { 0.8f, 3.98f }, { 0.9f, 4.06f }, { 0.95f, 4.13f }, { 1, 4.2f } };
  if (v <= c[0][1]) return 0;
  for (int i = 1; i < (int)(sizeof c / sizeof *c); i++) if (v <= c[i][1]) return c[i - 1][0] + (c[i][0] - c[i - 1][0]) * (v - c[i - 1][1]) / (c[i][1] - c[i - 1][1]);
  return 1;
}

static void tick(super_state *S) {
  const fc_airframe *A = &S->FA.A; int nm = A->n_motors, nj = A->n_joints, sj[FC_MAX_JOINTS], ns = steer_list(S, sj);
  /* 1. how well each motor still does what its column says, from the stream */
  int k = 0; pk[k++] = (float)S->nb;
  for (int b = 0; b < SP_BATCH; b++) {
    const sp_sample *s = &S->batch[b]; int use = b < S->nb;
    pk[k++] = use ? (float)nm : 0; for (int i = 0; i < FC_MAX_MOTORS; i++) for (int r = 0; r < 6; r++) pk[k++] = use && i < nm ? s->phi[i][r] : 0;
    pk[k++] = use ? (float)ns : 0; for (int i = 0; i < FC_MAX_JOINTS; i++) for (int r = 0; r < 6; r++) pk[k++] = use && i < ns ? s->psi[i][r] : 0;
    for (int r = 0; r < 6; r++) pk[k++] = use ? s->y[r] : 0;
  }
  pk[k++] = SUP_DT; pk[k++] = SUP_MEMORY;
  float eta[FC_MAX_MOTORS], conf[FC_MAX_MOTORS], del[FC_MAX_JOINTS], sconf[FC_MAX_JOINTS];
  for (int i = 0; i < FC_MAX_MOTORS; i++) { eta[i] = 1; conf[i] = 0; }
  for (int j = 0; j < FC_MAX_JOINTS; j++) { del[j] = 0; sconf[j] = 0; }
  if (!call(S, S->f_ah, res)) {
    const int LM = 1 + FC_MAX_MOTORS, LJ = 1 + FC_MAX_JOINTS;
    int ne = (int)res[0]; for (int i = 0; i < ne && i < nm; i++) { eta[i] = res[1 + i]; conf[i] = res[LM + 1 + i]; }
    int nd = (int)res[2 * LM]; for (int j = 0; j < nd && j < ns; j++) { del[j] = res[2 * LM + 1 + j]; sconf[j] = res[2 * LM + LJ + 1 + j]; }
  }
  float last_cmd[FC_MAX_MOTORS]; for (int i = 0; i < nm; i++) last_cmd[i] = S->nb ? S->batch[S->nb - 1].cmd[i] : 0;
  S->nb = 0;
  /* 2. what each part reports; temperatures estimated from the ESC's current if there's no sensor */
  k = 0; pk[k++] = (float)nm;
  for (int i = 0; i < FC_MAX_MOTORS; i++) {
    float T = -999; int hasT = 0, hasR = 0; float rr = 0;
    if (i < nm) {
      if (S->h_has_t[i]) { T = S->h_t[i]; hasT = 1; }
      else if (S->h_has_i[i] && i < S->C.n && S->C.ct[i] > 0) {   /* the same heating model as the motor, on the supervisor */
        float x = clampf(S->h_rpm[i] * 2 * PI_ / 60 / maxf(1, S->C.om[i]), 0, 1.2f), t0 = S->have_t_est[i] ? S->t_est[i] : S->C.ambient;
        float b[6] = { t0, S->h_i[i] * S->h_i[i] * S->C.rw[i], S->C.gf[i] * (0.3f + 0.7f * x), S->C.ct[i], S->C.ambient, SUP_DT }, o;
        memcpy(pk + 1000, b, sizeof b);   /* (thermalModel's inputs, out of the way of the batch being built) */
        if (!rn_host_call(S->H, S->f_th, 0, pk + 1000, &o) && fin(o)) { S->t_est[i] = o; S->have_t_est[i] = 1; T = o; hasT = 1; }
      }
      float om = i < S->C.n ? S->C.om[i] : 0, exp = sqrtf(maxf(0, last_cmd[i])) * om * 60 / (2 * PI_);
      if (S->h_has_rpm[i] && exp > 300) { hasR = 1; rr = S->h_rpm[i] / exp; }
    }
    S->m_temp[i] = T; S->m_eta[i] = eta[i]; S->m_conf[i] = conf[i];
    pk[k++] = (float)S->m_on[i]; pk[k++] = S->m_eff[i]; pk[k++] = i < nm ? last_cmd[i] : 0; pk[k++] = (float)hasT; pk[k++] = hasT ? T : 0;
    pk[k++] = i < S->C.n ? S->C.tmax[i] : 120; pk[k++] = (float)hasR; pk[k++] = rr; pk[k++] = eta[i]; pk[k++] = conf[i];
  }
  pk[k++] = (float)ns;
  for (int q = 0; q < FC_MAX_JOINTS; q++) { int j = q < ns ? sj[q] : 0; pk[k++] = q < ns ? S->thh[j] : 0; pk[k++] = del[q]; pk[k++] = sconf[q]; pk[k++] = 0; pk[k++] = 0; }
  pk[k++] = SUP_DT;
  if (!call(S, S->f_fd, res)) {
    int nr = (int)res[0];
    for (int i = 0; i < nr && i < nm; i++) {
      const float *d = res + 1 + 6 * i; int on = d[1] > 0.5f;
      char why[100];
      int was_on = S->m_on[i]; float was_eff = S->m_eff[i];
      S->m_state[i] = (int)d[0]; S->m_why[i] = (int)d[4]; S->m_val[i] = d[5];
      S->m_on[i] = on; S->m_eff[i] = clampf(d[2], 0.05f, 2); S->m_cap[i] = clampf(d[3], 0, 1);
      super_why_motor(S, i, why, (int)sizeof why);
      if (!on && was_on) event(S, 3, "%s: %s. Removed from the controller's table.", name_m(S, i), why);
      else if (on && fabsf(S->m_eff[i] - was_eff) > 0.02f) event(S, 2, "%s: %s. Its column now ×%.2f.", name_m(S, i), why, (double)(S->m_eff[i] / was_eff));
      else if (S->m_cap[i] < 0.95f && !S->logged_cap[i]) { S->logged_cap[i] = 1; event(S, 2, "%s: %s. Throttle capped at %d%%, and lower as it heats.", name_m(S, i), why, (int)(S->m_cap[i] * 100 + 0.5f)); }
    }
    const float *sv = res + 1 + 6 * FC_MAX_MOTORS; int nsv = (int)sv[0];
    for (int q = 0; q < nsv && q < ns; q++) {
      const float *d = sv + 1 + 4 * q; int j = sj[q];
      if (d[0] > 0.5f) {
        if (!S->j_off[j]) { S->j_why[j] = (int)d[2]; S->j_val[j] = d[3]; char why[100]; why_joint(S, j, why, (int)sizeof why);
          event(S, 3, "%s: %s. Left out of the steering; the controller now uses where it really is.", A->jnt[j].name, why); }
        S->j_off[j] = 1; S->j_ang[j] = clampf(d[1], -1.2f * A->jnt[j].range, 1.2f * A->jnt[j].range);
      }
    }
  }
  /* 3. how to fly on what's left: the lift and control margin with the table as flown now */
  k = 0; int ni = 0; float cols[FC_MAX_MOTORS + FC_MAX_JOINTS + 1][6], lo[FC_MAX_MOTORS + FC_MAX_JOINTS + 1], hi[FC_MAX_MOTORS + FC_MAX_JOINTS + 1];
  for (int i = 0; i < nm; i++) if (S->m_on[i]) { col_now(S, i, -1, cols[ni]); lo[ni] = 0; hi[ni] = S->m_cap[i]; ni++; }
  for (int q = 0; q < ns; q++) {   /* steering servos count too: each can turn its rotors across what's left of its travel */
    int j = sj[q]; if (S->j_off[j]) continue;
    float d[6] = { 0 }; int any = 0;
    for (int i = 0; i < nm; i++) { int m = chain_pos(&A->mot[i], j); if (m < 0 || !S->m_on[i]) continue; float c[6]; col_now(S, i, m, c); for (int r = 0; r < 6; r++) d[r] += c[r] * maxf(S->v[i], 0.2f); any = 1; }
    if (!any) continue;
    float th = S->thh[j], R = A->jnt[j].range;
    memcpy(cols[ni], d, sizeof d); lo[ni] = minf(0, -R - th); hi[ni] = maxf(0, R - th); ni++;
  }
  /* A fixed input represents the known external cable wrench. It contributes force/torque,
   * cannot be allocated away, and consumes no actuator slot or control authority. */
  float load = 0;
  for (int r = 0; r < 6; r++) load += fabsf(S->FA.payload[r]);
  if (load > 0) {
    for (int r = 0; r < 3; r++) {
      float f = 0, a = 0;
      for (int q = 0; q < 3; q++) { f += S->R[3*q+r] * S->FA.payload[q]; a += A->Jinv[3*r+q] * S->FA.payload[3+q]; }
      cols[ni][r] = f / A->m; cols[ni][3+r] = a;
    }
    lo[ni] = hi[ni] = 1; ni++;
  }
  const int NI = FC_MAX_MOTORS + FC_MAX_JOINTS + 1;
  pk[k++] = (float)ni; for (int c = 0; c < NI; c++) for (int r = 0; r < 6; r++) pk[k++] = c < ni ? cols[c][r] : 0;
  pk[k++] = (float)ni; for (int c = 0; c < NI; c++) pk[k++] = c < ni ? lo[c] : 0;
  pk[k++] = (float)ni; for (int c = 0; c < NI; c++) pk[k++] = c < ni ? hi[c] : 0;
  float mg[3] = { 0, 0, 0 };
  if (!call(S, S->f_lm, mg)) { /* Report thrust-to-total-supported-weight rather than net acceleration / rigid weight. */
    float support = maxf(0, -S->FA.payload[2]) / (A->m * G_);
    S->margin = (mg[0] + support) / (1 + support); S->rp_ok = mg[1] > 0.5f; S->yaw_ok = mg[2] > 0.5f; }
  /* the battery: its resting voltage (measured, plus the sag its current causes) per working cell gives the charge; a
   * drop of about a cell's worth within two seconds means a cell has failed: count one fewer */
  float soc = -1, vcell = -1;
  if (S->b_has_v) {
    float vrest = S->b_v + S->C.r_int * (S->b_has_i ? S->b_i : 0), per = vrest / (float)(S->cells > 1 ? S->cells : 1), before = per;
    if (S->nvh == 24) { for (int q = 1; q < 24; q++) { S->vh_t[q - 1] = S->vh_t[q]; S->vh_v[q - 1] = S->vh_v[q]; } S->nvh--; }
    S->vh_t[S->nvh] = (float)S->t; S->vh_v[S->nvh++] = per;
    for (int q = 0; q < S->nvh; q++) if (S->vh_t[q] > (float)S->t - 2) before = maxf(before, S->vh_v[q]);
    if (per < 3.3f && before - per > 0.5f && S->cells > 1) { S->cells--; S->nvh = 0; S->cell_lost = 1; event(S, 3, "Battery: its voltage fell by about a cell's worth. Counting %d working cells.", S->cells); }
    soc = lipo_charge(vrest / (float)(S->cells > 1 ? S->cells : 1)); vcell = S->b_v / (float)(S->cells > 1 ? S->cells : 1);
  }
  S->soc = soc; S->have_soc = soc >= 0;
  float hot = -1; int any_failed = 0;
  for (int i = 0; i < nm; i++) { if (S->m_temp[i] > -900 && i < S->C.n && S->C.tmax[i] > 0) hot = maxf(hot, S->m_temp[i] / S->C.tmax[i]); if (S->m_state[i] == 3) any_failed = 1; }
  k = 0;
  pk[k++] = SUP_DT; pk[k++] = S->margin; pk[k++] = (float)S->rp_ok; pk[k++] = (float)S->yaw_ok; pk[k++] = (float)any_failed; pk[k++] = (float)S->cell_lost;
  pk[k++] = hot >= 0; pk[k++] = hot >= 0 ? hot : 0; pk[k++] = soc >= 0; pk[k++] = soc >= 0 ? soc : 0; pk[k++] = vcell >= 0; pk[k++] = vcell >= 0 ? vcell : 0;
  pk[k++] = (float)S->b_has_t; pk[k++] = S->b_has_t ? S->b_t : 0; pk[k++] = S->C.batt_tmax > 0 ? S->C.batt_tmax : 60;
  pk[k++] = (float)S->have_pol; pk[k++] = (float)S->mode; pk[k++] = (float)S->why; pk[k++] = S->rp_bad; pk[k++] = S->v_bad;
  float pol[7];
  if (!call(S, S->f_fp, pol)) {
    int mode = (int)pol[0], was = S->mode;
    S->mode = mode; S->lim_speed = pol[1]; S->lim_lean = pol[2]; S->lim_accel = pol[3]; S->rp_bad = pol[5]; S->v_bad = pol[6]; S->have_pol = 1;
    if ((int)pol[4] != S->why || mode != was) { S->why = (int)pol[4]; S->why_val = S->margin; }
    if (mode != was) {
      const char *label[] = { "normal", "careful (lower speed and lean)", "returning home to land", "landing now" };
      char w[96]; super_why_mode(S, w, (int)sizeof w);
      event(S, mode == 0 ? 1 : mode == 1 ? 2 : 3, "Flight: %s%s%s%s", label[mode & 3], w[0] ? " (" : "", w, w[0] ? ")" : "");
    }
  }
  (void)nj;
  S->sent = 1;
}

int super_set_frame(super_state *S, float *out) {
  const fc_airframe *A = &S->FA.A; int nm = A->n_motors, nj = A->n_joints;
  if (!S->ok || !S->FA.have_airframe || !S->got) return 0;
  if (S->t >= S->next_tick) {
    S->next_tick = S->t + SUP_DT;
    if (S->state != FC_CRASHED && !S->open) tick(S);
  }
  if (!S->sent) return 0;
  S->sent = 0;
  int k = 0;
  out[k++] = (float)S->mode; out[k++] = S->lim_lean; out[k++] = S->lim_accel; out[k++] = S->lim_speed; out[k++] = (float)nm; out[k++] = (float)nj;
  for (int i = 0; i < nm; i++) { out[k++] = (float)S->m_on[i]; out[k++] = S->m_eff[i]; out[k++] = S->m_cap[i]; }
  for (int j = 0; j < nj; j++) { out[k++] = (float)S->j_off[j]; out[k++] = S->j_ang[j]; }
  fc_set(&S->FA, out, k);                                            /* (its own copy of the table as flown) */
  return k;
}

int super_status(const super_state *S, float *o) {
  const fc_airframe *A = &S->FA.A; int k = 0;
  o[k++] = (float)S->mode; o[k++] = (float)S->why; o[k++] = S->why_val; o[k++] = S->margin; o[k++] = (float)S->rp_ok; o[k++] = (float)S->yaw_ok;
  o[k++] = S->have_soc ? S->soc : -1; o[k++] = (float)S->cells; o[k++] = S->lim_speed; o[k++] = S->lim_lean; o[k++] = S->lim_accel; o[k++] = (float)S->log_seq;
  for (int i = 0; i < A->n_motors; i++) { o[k++] = (float)S->m_state[i]; o[k++] = (float)S->m_on[i]; o[k++] = S->m_eff[i]; o[k++] = S->m_cap[i]; o[k++] = (float)S->m_why[i]; o[k++] = S->m_val[i]; o[k++] = S->m_temp[i]; o[k++] = S->m_eta[i]; o[k++] = S->m_conf[i]; }
  for (int j = 0; j < A->n_joints; j++) { o[k++] = (float)S->j_off[j]; o[k++] = S->j_ang[j]; o[k++] = (float)S->j_why[j]; o[k++] = S->j_val[j]; }
  return k;
}
