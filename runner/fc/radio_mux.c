/* Two radio links at once, as one: see radio_mux.h. */
#include "radio_mux.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  radio_io io, *R[2]; rlink_cfg L[2]; int role; double (*now)(void);
  lmux M; char name[96]; uint8_t buf[512];
} mux_t;

static int sends(const mux_t *X, int i) { return X->role == LMUX_GROUND ? rlink_up(&X->L[i]) : rlink_down(&X->L[i]); }
/* A one-way link beside a two-way packet link: tied to it (radio_mux.h). */
static void tie(mux_t *X, double t) {
  int up, down, rssi; lmux_lq(&X->M, t, &up, &down, &rssi);
  for (int i = 0; i < 2; i++) {
    if (X->R[i]->hear) X->R[i]->hear(X->R[i], X->role == LMUX_DRONE ? up : down, rssi);
    if (X->L[i].dir == RLINK_BOTH || !X->R[i]->tie) continue;
    int j = 1 - i; if (X->L[j].dir != RLINK_BOTH || !X->R[j]->peer) continue;
    X->R[i]->tie(X->R[i], X->R[j]->peer(X->R[j]));
  }
}
static int mux_read(radio_io *R, uint8_t *b, int n, int wait_ms) {
  mux_t *X = R->ctx; double t = X->now(); int got = 0, failed = 0;
  tie(X, t);
  for (int pass = 0; pass < 2 && !got; pass++) {                     /* each link without waiting; then, nothing come, the first with its wait */
    for (int i = 0; i < 2; i++) {
      if (pass && i) break;
      int k = X->R[i]->read(X->R[i], X->buf, sizeof X->buf, pass ? wait_ms : 0);
      if (k < 0) { failed++; continue; }
      if (k) { lmux_from_link(&X->M, i, X->buf, k, X->now()); got = 1; }
    }
    if (!wait_ms) break;
  }
  int m = lmux_to_stack(&X->M, X->now(), b, n);
  return m ? m : failed == 2 ? -1 : 0;
}
static int mux_write(radio_io *R, const uint8_t *b, int n) {
  mux_t *X = R->ctx; int ok = 0;
  for (int i = 0; i < 2; i++) if (sends(X, i) && X->R[i]->write(X->R[i], b, n) >= 0) ok = 1;
  return ok ? n : -1;
}
radio_io *radio_mux_open(radio_io *a, const rlink_cfg *La, radio_io *b, const rlink_cfg *Lb, int role, double (*now)(void)) {
  mux_t *X = calloc(1, sizeof *X); if (!X) return 0;
  X->R[0] = a; X->R[1] = b; X->L[0] = *La; X->L[1] = *Lb; X->role = role; X->now = now;
  int up[2] = { rlink_up(La), rlink_up(Lb) }, down[2] = { rlink_down(La), rlink_down(Lb) };
  lmux_init(&X->M, role, 2, up, down);
  char d0[32], d1[32]; rlink_describe(La, d0, sizeof d0); rlink_describe(Lb, d1, sizeof d1);
  snprintf(X->name, sizeof X->name, "%s + %s", d0, d1);
  X->io.name = X->name; X->io.read = mux_read; X->io.write = mux_write; X->io.fd = -1; X->io.ctx = X;
  return &X->io;
}
int radio_mux_is(const radio_io *R) { return R && R->read == mux_read; }
int radio_mux_followed(radio_io *R) { mux_t *X = R->ctx; return lmux_followed(&X->M, X->now()); }
const rlink_cfg *radio_mux_link(radio_io *R) {                    /* the telemetry's link: the first carrying it down that reports, else the first carrying it */
  mux_t *X = R->ctx; double t = X->now();
  for (int i = 0; i < 2; i++) if (rlink_down(&X->L[i]) && t - X->M.k[i].t_stats < 0.5) return &X->L[i];
  for (int i = 0; i < 2; i++) if (rlink_down(&X->L[i])) return &X->L[i];
  return &X->L[0];
}
void radio_mux_free(radio_io *R) { if (radio_mux_is(R)) free(R->ctx); }
radio_io *radio_mux_part(radio_io *R, int i) { mux_t *X = R->ctx; return i == 0 || i == 1 ? X->R[i] : 0; }
void radio_mux_counts(radio_io *R, char *out, int n) {
  mux_t *X = R->ctx; int f = radio_mux_followed(R), k = 0;
  for (int i = 0; i < 2 && k < n; i++) {
    char d[32]; rlink_describe(&X->L[i], d, sizeof d);
    k += snprintf(out + k, (size_t)(n - k), "%slink %d (%s): %s, %u frames in, %u taken", i ? "; " : "", i + 1, d, f == i ? "followed" : "standing by", (unsigned)X->M.k[i].frames, (unsigned)X->M.k[i].taken);
  }
  if (k < n) snprintf(out + k, (size_t)(n - k), "; copies dropped %u, switches %u", (unsigned)X->M.dups, (unsigned)X->M.switches);
}
