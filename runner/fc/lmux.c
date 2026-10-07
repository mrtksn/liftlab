/* Two radio links at once: see lmux.h. */
#include "lmux.h"

static void copy(uint8_t *d, const uint8_t *s, int n) { while (n-- > 0) *d++ = *s++; }
static int same(const uint8_t *a, const uint8_t *b, int n) { for (int i = 0; i < n; i++) if (a[i] != b[i]) return 0; return 1; }

void lmux_init(lmux *M, int role, int n, const int *up, const int *down) {
  uint8_t *p = (uint8_t *)M; for (unsigned i = 0; i < sizeof *M; i++) p[i] = 0;
  M->role = role; M->n = n < 1 ? 1 : n > LMUX_MAX ? LMUX_MAX : n; M->followed = -1; M->t_stats = -1e9;
  for (int i = 0; i < M->n; i++) { M->k[i].up = up[i]; M->k[i].down = down[i]; M->k[i].t_rc = M->k[i].t_tlm = M->k[i].t_stats = -1e9; }
  for (int i = 0; i < LMUX_SEEN; i++) M->seen[i].t = -1e9;
}
static void out(lmux *M, const uint8_t *f, int n) { if (M->out_n + n <= LMUX_OUT) { copy(M->out + M->out_n, f, n); M->out_n += n; } }
static int reliable(const uint8_t *f) { return f[2] == CRSF_EXT && f[1] >= 3 && (f[3] == CRSF_EXT_TEXT || f[3] == CRSF_EXT_CMD); }
/* Is link i bringing the telemetry: frames within 0.3 s? */
static int bringing(const lmux *M, int i, double t) { return M->k[i].down && t - M->k[i].t_tlm < 0.3; }
int lmux_followed(const lmux *M, double t) {
  for (int i = 0; i < M->n; i++) {
    if (M->role == LMUX_DRONE ? M->k[i].up && t - M->k[i].t_rc < 0.25 : bringing(M, i, t)) return i;
  }
  return -1;
}
static void frame(lmux *M, int i, const uint8_t *f, int n, double t) {
  M->k[i].frames++;
  if (f[2] == CRSF_LINK_STATS) { crsf_link_stats_read(f + 3, f[1] - 2, &M->k[i].S); M->k[i].t_stats = t; return; }
  if (f[2] == CRSF_RC) {
    M->k[i].t_rc = t;
    for (int j = 0; j < i; j++) if (M->k[j].up && t - M->k[j].t_rc < LMUX_RC_HOLD) return;   /* (a link before it has them) */
    M->k[i].taken++; out(M, f, n); return;
  }
  if (reliable(f)) {                                                 /* once: a copy from the other link is dropped */
    int best = -1;                                                   /* the oldest the other link brought, not yet matched */
    for (int s = 0; s < LMUX_SEEN; s++)
      if (M->seen[s].n == n && M->seen[s].from != i && !M->seen[s].matched && t - M->seen[s].t < LMUX_DUP_S && same(M->seen[s].f, f, n) && (best < 0 || M->seen[s].t < M->seen[best].t)) best = s;
    if (best >= 0) { M->seen[best].matched = 1; M->dups++; return; }
    int s = M->seen_i; M->seen_i = (s + 1) % LMUX_SEEN; copy(M->seen[s].f, f, n); M->seen[s].n = n; M->seen[s].t = t; M->seen[s].from = i; M->seen[s].matched = M->n < 2;
    M->k[i].taken++; out(M, f, n); return;
  }
  M->k[i].t_tlm = t;
  for (int j = 0; j < i; j++) if (bringing(M, j, t)) return;          /* (the telemetry from the first link bringing it) */
  M->k[i].taken++; out(M, f, n);
}
void lmux_from_link(lmux *M, int i, const uint8_t *b, int n, double t) {
  if (i < 0 || i >= M->n) return;
  for (int k = 0; k < n; k++) { int len = crsf_feed(&M->k[i].P, b[k]); if (len > 0) frame(M, i, M->k[i].P.buf, len, t); }
}
int lmux_lq(const lmux *M, double t, int *up, int *down, int *rssi) {
  int any = 0; *up = *down = 0; *rssi = 0;
  for (int i = 0; i < M->n; i++) {
    if (t - M->k[i].t_stats >= 0.5) continue;
    const crsf_link *S = &M->k[i].S; any = 1;
    if (M->k[i].up && S->up_lq > *up) { *up = (int)S->up_lq; if (M->role == LMUX_DRONE) *rssi = (int)S->up_rssi; }
    if (M->k[i].down && S->down_lq > *down) { *down = (int)S->down_lq; if (M->role == LMUX_GROUND) *rssi = (int)S->down_rssi; }
  }
  return any;
}
int lmux_to_stack(lmux *M, double t, uint8_t *b, int cap) {
  if (t - M->t_stats >= 0.1) {                                       /* the link statistics: the followed link's, the best link quality each way */
    M->t_stats = t;
    int f = lmux_followed(M, t);
    if (f != M->followed) { if (f >= 0 && M->followed >= 0) M->switches++; M->followed = f; }
    int src = f >= 0 && t - M->k[f].t_stats < 0.5 ? f : -1;
    for (int i = 0; src < 0 && i < M->n; i++) if (t - M->k[i].t_stats < 0.5) src = i;
    if (src >= 0) {
      crsf_link S = M->k[src].S;
      for (int i = 0; i < M->n; i++) {
        if (t - M->k[i].t_stats >= 0.5) continue;
        if (M->k[i].up && M->k[i].S.up_lq > S.up_lq) S.up_lq = M->k[i].S.up_lq;
        if (M->k[i].down && M->k[i].S.down_lq > S.down_lq) S.down_lq = M->k[i].S.down_lq;
      }
      uint8_t fr[CRSF_MAX_FRAME]; int n = crsf_link_stats(fr, M->role == LMUX_DRONE ? CRSF_ADDR_FC : CRSF_ADDR_HANDSET, &S);
      if (n) out(M, fr, n);
    }
  }
  int n = M->out_n < cap ? M->out_n : cap;
  copy(b, M->out, n); for (int i = n; i < M->out_n; i++) M->out[i - n] = M->out[i]; M->out_n -= n;
  return n;
}
