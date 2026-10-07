/* The fleet program on the navigation's board: see fleet.h. */
#include "fleet.h"

#define IN_N (13 + 1 + FLEET_N * 24 + 1 + FLEET_MSG * (2 + FLEET_VALS) + 1)   /* me, others, msg, dt (js/rn-sigs.js) */
#define OUT_N (2 + FLEET_VALS + 10 + 2 + FLEET_MSG * (2 + FLEET_VALS))       /* publish, go, send */

static int fin(float x) { return (x - x) == 0; }
static float clampf(float x, float a, float b) { return x < a ? a : x > b ? b : x; }
static uint32_t join16(float lo, float hi) { return (uint32_t)(lo < 0 ? 0 : lo > 65535 ? 65535 : lo) | (uint32_t)(hi < 0 ? 0 : hi > 65535 ? 65535 : hi) << 16; }
static float id24(uint32_t id) { return (float)(id >> 8); }   /* (the MAC address's last three bytes: the device's own, peer_id_of) */
static void say(fleet_state *F, const char *a, const char *b) {
  int i = 0; for (; a && *a && i < 63; a++) F->msg[i++] = *a;
  for (; b && *b && i < 63; b++) F->msg[i++] = *b;
  F->msg[i] = 0; F->said = 1;
}

int fleet_init(fleet_state *F, rn_host *H) {
  char *p = (char *)F; for (unsigned i = 0; i < sizeof *F; i++) p[i] = 0;
  F->H = H; F->f = rn_host_find(H, "fleetProgram");
  if (F->f < 0) { say(F, "no fleet program in this board's program: the fleet is off", 0); return -1; }
  if (rn_host_in_size(H, F->f) != IN_N || rn_host_out_size(H, F->f) != OUT_N) { F->f = -1; say(F, "the fleet program isn't what this code expects: the fleet is off", 0); return -1; }
  F->ok = 1; F->t_run = -1e9;
  return 0;
}

void fleet_peers(fleet_state *F, const float *p, int n, double t) {
  if (n < 7 || n > FLEET_PACK_MAX || p[0] != 1) return;
  for (int i = 0; i < n; i++) if (!fin(p[i])) return;
  /* walk it once: a bad one is dropped whole, the messages taken */
  int k = 7, ns = (int)p[5], nm = (int)p[6];
  if (ns < 0 || ns > PEER_MAX || nm < 0 || nm > FLEET_MSG) return;
  for (int s = 0; s < ns; s++) { if (k + 8 > n) return; int c = (int)p[k + 7]; if (c < 0 || c > PEER_VALS) return; k += 8 + c; }
  int k_msg = k;
  for (int m = 0; m < nm; m++) { if (k + 3 > n) return; int c = (int)p[k + 2]; if (c < 0 || c > FLEET_VALS || k + 3 + c > n) return; k += 3 + c; }
  for (int i = 0; i < k_msg; i++) F->pk[i] = p[i];
  F->pk[6] = 0; F->pk_n = k_msg; F->t_pk = t;
  for (int m = 0, q = k_msg; m < nm; m++) {
    int c = (int)p[q + 2];
    if (F->in_n == 2 * FLEET_MSG) { for (int i = 1; i < F->in_n; i++) F->in[i - 1] = F->in[i]; F->in_n--; }   /* (the oldest goes) */
    F->in[F->in_n].from = join16(p[q], p[q + 1]); F->in[F->in_n].n = c;
    for (int j = 0; j < c; j++) F->in[F->in_n].v[j] = p[q + 3 + j];
    F->in_n++; F->got++; q += 3 + c;
  }
}

int fleet_engage(fleet_state *F, int on, const nav_state *N, const nav_out *o, const char *why) {
  if (!on) { if (F->engaged) { F->engaged = 0; say(F, "fleet program off: ", why ? why : "asked"); } return 0; }
  if (!F->ok) { say(F, F->msg[0] ? F->msg : "no fleet program here", 0); return -1; }
  if (!o->have_home || !o->fly) { say(F, "fleet program: take off first", 0); return -1; }
  if (F->engaged) return 0;
  for (int i = 0; i < 3; i++) { F->go_p[i] = o->p[i]; F->go_v[i] = 0; }
  F->go_h = o->heading; F->engaged = 1; (void)N;
  say(F, "fleet program flies it: sticks, hold or home take it back", 0);
  return 0;
}

static int run(fleet_state *F, const nav_state *N, const nav_out *o, double t, float dt) {
  float b[IN_N], r[OUT_N]; int k = 0;
  const float *pk = F->pk; int have = F->pk_n >= 7 && t - F->t_pk < 2 * FLEET_STALE_S;
  int shared = N && N->have_pa && (N->seen & 2);
  const float *org = N && N->have_home ? N->home : N ? N->pa : 0;
  /* me */
  b[k++] = have ? id24(join16(pk[1], pk[2])) : 0;
  for (int i = 0; i < 3; i++) b[k++] = o->p[i];
  for (int i = 0; i < 3; i++) b[k++] = o->v[i];
  b[k++] = o->heading; b[k++] = (float)o->fly; b[k++] = have ? pk[4] : -1; b[k++] = (float)shared; b[k++] = (float)F->engaged; b[k++] = (float)t;
  /* the others */
  int at_n = k++, no = 0, q = 7, ns = have ? (int)pk[5] : 0;
  for (int s = 0; s < ns; s++) {
    const float *e = pk + q; int c = (int)e[7]; const float *v = e + 8; q += 8 + c;
    if (no == FLEET_N) continue;
    int fleet = c >= FLEET_HEAD + FLEET_EXT, flags = fleet ? (int)v[FLEET_HEAD + 7] : 0;
    b[k++] = id24(join16(e[0], e[1])); b[k++] = e[2]; b[k++] = e[3]; b[k++] = e[6] < 0 ? 99 : e[6];
    b[k++] = c > 0 && v[0] == 1 && (!fleet || (flags & FLEET_F_FLYING)) ? 1 : 0;      /* flying: armed (and in the air, as it says) */
    b[k++] = c > 1 ? v[1] : -1;
    if (fleet && (flags & FLEET_F_SHARED) && shared) { b[k++] = 1; for (int i = 0; i < 3; i++) b[k++] = v[FLEET_HEAD + i] - org[i]; }
    else { b[k++] = 0; b[k++] = 0; b[k++] = 0; b[k++] = 0; }
    for (int i = 0; i < 3; i++) b[k++] = fleet ? v[FLEET_HEAD + 3 + i] : 0;
    b[k++] = fleet ? v[FLEET_HEAD + 6] : 0; b[k++] = flags & FLEET_F_ENGAGED ? 1 : 0;
    int nv = fleet ? c - FLEET_HEAD - FLEET_EXT : 0; if (nv > FLEET_VALS) nv = FLEET_VALS;
    b[k++] = (float)nv; for (int j = 0; j < FLEET_VALS; j++) b[k++] = j < nv ? v[FLEET_HEAD + FLEET_EXT + j] : 0;
    no++;
  }
  b[at_n] = (float)no;
  for (int s = no; s < FLEET_N; s++) for (int j = 0; j < 24; j++) b[k++] = 0;
  /* the messages */
  int nm = F->in_n < FLEET_MSG ? F->in_n : FLEET_MSG;
  b[k++] = (float)nm;
  for (int m = 0; m < FLEET_MSG; m++) {
    b[k++] = m < nm ? id24(F->in[m].from) : 0; b[k++] = m < nm ? (float)F->in[m].n : 0;
    for (int j = 0; j < FLEET_VALS; j++) b[k++] = m < nm && j < F->in[m].n ? F->in[m].v[j] : 0;
  }
  for (int i = nm; i < F->in_n; i++) F->in[i - nm] = F->in[i];
  F->in_n -= nm;
  b[k++] = dt;
  /* the program */
  F->calls++;
  if (rn_host_call(F->H, F->f, 0, b, r)) return -1;
  for (int i = 0; i < OUT_N; i++) if (!fin(r[i])) return -1;
  /* publish */
  if (r[0] > 0.5f) { int c = (int)r[1]; F->npub = c < 0 ? 0 : c > FLEET_VALS ? FLEET_VALS : c; for (int j = 0; j < F->npub; j++) F->pub[j] = r[2 + j]; }
  /* go */
  const float *g = r + 2 + FLEET_VALS;
  if (g[0] > 0.5f && F->engaged) {
    F->go_p[0] = clampf(g[1], -FLEET_BOX_XY, FLEET_BOX_XY); F->go_p[1] = clampf(g[2], -FLEET_BOX_XY, FLEET_BOX_XY); F->go_p[2] = clampf(g[3], FLEET_ZLO, FLEET_ZHI);
    for (int i = 0; i < 3; i++) F->go_v[i] = g[4] > 0.5f ? clampf(g[5 + i], -FLEET_VMAX, FLEET_VMAX) : 0;
    if (g[8] > 0.5f) F->go_h = g[9];
  } else if (F->engaged) for (int i = 0; i < 3; i++) F->go_v[i] = 0;   /* (nothing said: it holds the last place) */
  /* send: to node numbers as the program knows them (24 bits), back to whole ones from the table */
  const float *sd = g + 10;
  if (sd[0] > 0.5f) {
    int c = (int)sd[1]; if (c > FLEET_MSG) c = FLEET_MSG;
    for (int m = 0; m < c; m++) {
      const float *e = sd + 2 + m * (2 + FLEET_VALS); uint32_t to = 0;
      if (e[0] != 0) {
        for (int s = 0, q2 = 7; s < ns; s++) { uint32_t id = join16(pk[q2], pk[q2 + 1]); if (id24(id) == e[0]) to = id; q2 += 8 + (int)pk[q2 + 7]; }
        if (!to) { F->unknown_to++; continue; }
      }
      if (F->out_n == 2 * FLEET_MSG) { for (int i = 1; i < F->out_n; i++) F->out[i - 1] = F->out[i]; F->out_n--; }
      int nv = (int)e[1]; nv = nv < 0 ? 0 : nv > FLEET_VALS ? FLEET_VALS : nv;
      F->out[F->out_n].to = to; F->out[F->out_n].n = nv; for (int j = 0; j < nv; j++) F->out[F->out_n].v[j] = e[2 + j];
      F->out_n++; F->sent++;
    }
  }
  /* what the navigation publishes with it */
  for (int i = 0; i < 3; i++) { F->ext[i] = N && N->have_pa ? N->pa[i] : 0; F->ext[3 + i] = o->v[i]; }
  F->ext[6] = o->heading; F->ext[7] = (float)((shared ? FLEET_F_SHARED : 0) | (F->engaged ? FLEET_F_ENGAGED : 0) | (o->fly ? FLEET_F_FLYING : 0));
  F->out_new = 1;
  return 0;
}

int fleet_step(fleet_state *F, const nav_state *N, const nav_out *o, double t) {
  if (!F->ok) return 0;
  if (F->engaged && (!o->fly || !o->have_home || (N && N->landed))) fleet_engage(F, 0, N, o, "not flying");
  if (t - F->t_run < 1 / FLEET_HZ - 1e-6) return F->engaged;
  float dt = F->t_last > 0 && t > F->t_last ? (float)(t - F->t_last) : (float)(1 / FLEET_HZ);
  F->t_run = t; F->t_last = t;
  if (run(F, N, o, t, dt)) {
    F->fails++;
    if (F->engaged) fleet_engage(F, 0, N, o, "the program failed (a trap, or a number that isn't one)");
  }
  return F->engaged;
}

void fleet_sp(const fleet_state *F, nav_sp *sp) {
  if (!F->engaged) return;
  for (int i = 0; i < 3; i++) { sp->target[i] = F->go_p[i]; sp->vref[i] = F->go_v[i]; }
  sp->heading = F->go_h;
}

int fleet_out(fleet_state *F, float *o) {
  if (!F->out_new) return 0;
  int k = 0; o[k++] = 1; o[k++] = (float)(FLEET_EXT + F->npub);
  for (int i = 0; i < FLEET_EXT; i++) o[k++] = F->ext[i];
  for (int j = 0; j < F->npub; j++) o[k++] = F->pub[j];
  o[k++] = (float)F->out_n;
  for (int m = 0; m < F->out_n; m++) {
    o[k++] = (float)(F->out[m].to & 0xFFFF); o[k++] = (float)(F->out[m].to >> 16); o[k++] = (float)F->out[m].n;
    for (int j = 0; j < F->out[m].n; j++) o[k++] = F->out[m].v[j];
  }
  F->out_n = 0; F->out_new = 0;
  return k;
}
