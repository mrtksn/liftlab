/*
 * What each task puts into the telemetry store (tlm_core.h). Called by whichever board runs the task, after its
 * step; they only read the task's state. Each keeps a little memory (tlm_watch) to turn changes of a task's message
 * into telemetry messages.
 */
#include "tlm_sources.h"
#include "wasm_math.h"

static int str_eq(const char *a, const char *b) { int i = 0; for (; a[i] && b[i]; i++) if (a[i] != b[i]) return 0; return a[i] == b[i]; }
static void str_cp(char *d, const char *s, int n) { int i = 0; for (; s[i] && i < n - 1; i++) d[i] = s[i]; d[i] = 0; }

void tlm_watch_init(tlm_watch *W) { char *p = (char *)W; for (unsigned i = 0; i < sizeof *W; i++) p[i] = 0; W->fc_state = -1; }

void tlm_from_core(tlm_store *T, tlm_watch *W, const fc_state *F, double t) {
  const float *q = F->q;
  float roll = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2]));
  float sp = 2 * (q[0] * q[2] - q[3] * q[1]); sp = sp > 1 ? 1 : sp < -1 ? -1 : sp; float pitch = atan2f(sp, sqrtf(1 - sp * sp));
  float yaw = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]));
  float att[3] = { roll, pitch, yaw }; tlm_put(T, TLM_ATT, att, 3, t);
  float b[1] = { F->vbatt }; tlm_put(T, TLM_BATT, b, 1, t);
  if (F->have_alt) { float a[2] = { F->alt_e, F->vz_e }; tlm_put(T, TLM_ALT, a, 2, t); }
  float st[2] = { (float)F->state, (float)((F->att_ok ? 1 : 0) | (F->have_alt ? 2 : 0) | (F->cmd.guided ? 4 : 0) | (F->open_loop ? 8 : 0)) };
  tlm_put(T, TLM_STATE, st, 2, t);
  int nm = F->A.n_motors, flying = F->state == FC_ARMED || F->state == FC_FAILSAFE;
  float m[1 + FC_MAX_MOTORS]; m[0] = (float)nm; for (int i = 0; i < nm; i++) m[1 + i] = flying ? F->last_out.motor[i] : 0;
  tlm_put(T, TLM_MOTORS, m, 1 + nm, t);
  if (F->state != W->fc_state || !str_eq(F->why, W->fc_why)) {
    if (F->why[0] && !str_eq(F->why, W->fc_why)) tlm_text(T, F->state == FC_CRASHED ? 2 : F->state == FC_FAILSAFE ? 3 : 6, F->why);
    W->fc_state = F->state; str_cp(W->fc_why, F->why, sizeof W->fc_why);
  }
}

void tlm_from_nav(tlm_store *T, tlm_watch *W, const nav_state *N, const nav_out *o, const nav_sp *sp, int level, double t) {
  float p[6] = { o->p[0], o->p[1], o->p[2], o->v[0], o->v[1], o->v[2] };
  if (o->have_home) tlm_put(T, TLM_POS, p, 6, t);
  int bits = (o->ready ? 1 : 0) | (o->fly ? 2 : 0) | (o->have_home ? 4 : 0) | (N->auto_on && !N->auto_land ? 8 : 0) | (N->auto_on && N->auto_land ? 16 : 0)
    | (N->landed ? 32 : 0) | (N->rc_rth ? 64 : 0);
  float n[6] = { sp->target[0], sp->target[1], sp->target[2], sp->heading, (float)bits, (float)level };
  tlm_put(T, TLM_NAV, n, 6, t);
  if (!str_eq(N->why, W->nav_why)) { if (N->why[0]) tlm_text(T, 6, N->why); str_cp(W->nav_why, N->why, sizeof W->nav_why); }
}

void tlm_from_gps(tlm_store *T, double lat, double lon, float alt, float speed, float course, int sats, double t) {
  double la = lat * 1e7, lo = lon * 1e7;
  float hl = (float)floor(la / 65536), ho = (float)floor(lo / 65536);
  float g[8] = { hl, (float)(la - (double)hl * 65536), ho, (float)(lo - (double)ho * 65536), alt, speed, course, (float)sats };
  tlm_put(T, TLM_GPS, g, 8, t);
}

void tlm_from_learn(tlm_store *T, tlm_watch *W, const learn_state *L, double t) {
  float v[7] = { (float)L->cal, L->cal && L->total > 0 ? L->cal_t / L->total : 0, (float)L->use_learned, (float)L->keep, (float)L->thr,
    L->have_fit ? L->fit_rot : 0, L->have_fit ? L->fit_force : 0 };
  tlm_put(T, TLM_LEARN, v, 7, t);
  /* the learning's message, as it changes (only its new part, when it grew) */
  int n = 0; while (L->msg[n]) n++;
  int same = 0; while (same < n && W->learn_msg[same] && W->learn_msg[same] == L->msg[same]) same++;
  if (same < n || W->learn_msg[same]) {
    int grew = W->learn_msg[same] == 0 && same > 0;
    if (n) tlm_text(T, 6, L->msg + (grew ? same : 0));
    str_cp(W->learn_msg, L->msg, sizeof W->learn_msg);
  }
}

void tlm_from_super(tlm_store *T, tlm_watch *W, const super_state *S, double t) {
  float amps = S->b_has_i ? S->b_i : -1;
  if (S->b_has_i) { if (W->t_super > 0 && t > W->t_super) W->mah += S->b_i * (float)(t - W->t_super) / 3.6f; }
  W->t_super = t;
  float v[8] = { (float)S->mode, (float)S->why, S->margin, S->have_soc ? S->soc : -1, (float)S->cells, amps, W->mah };
  tlm_put(T, TLM_SUPER, v, 7, t);
  int nm = S->FA.A.n_motors; float m[1 + FC_MAX_MOTORS]; m[0] = (float)nm;
  for (int i = 0; i < nm; i++) m[1 + i] = S->m_on[i] ? S->m_eff[i] : -1;
  tlm_put(T, TLM_SUPER_M, m, 1 + nm, t);
  for (; W->super_seq < S->log_seq; W->super_seq++) {
    uint32_t back = S->log_seq - 1 - W->super_seq; if (back >= (uint32_t)S->nlog) continue;
    const sp_event *e = &S->log[back];
    tlm_text(T, e->tone == 3 ? 3 : e->tone == 2 ? 4 : 6, e->text);
  }
}

void tlm_from_cargo(tlm_store *T, cargo_state *C, double t) {
  float v[1 + CG_MAX]; v[0] = (float)C->n;
  for (int i = 0; i < C->n; i++) v[1 + i] = (float)cargo_bits(C, i);
  tlm_put(T, TLM_CARGO, v, 1 + C->n, t);
  if (C->said) { C->said = 0; tlm_text(T, 6, C->msg); }
}

void tlm_from_link(tlm_store *T, const rc_input *in, double t) {
  float v[4] = { in->up_rssi, in->up_lq, in->up_snr, rc_link_ok(in, t) ? 0.0f : 1.0f };
  tlm_put(T, TLM_LINK, v, 4, t);
}
