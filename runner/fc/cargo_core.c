/* Cargo: see cargo_core.h. */
#include "cargo_core.h"
#include "rc_core.h"

static void say(cargo_state *C, const char *a, int k, const char *b) {
  int i = 0;
  for (; a[i] && i < 60; i++) C->msg[i] = a[i];
  if (k >= 0 && i < 60) { if (k >= 9) C->msg[i++] = (char)('0' + (k + 1) / 10); C->msg[i++] = (char)('0' + (k + 1) % 10); }
  for (int j = 0; b && b[j] && i < 63; j++) C->msg[i++] = b[j];
  C->msg[i] = 0; C->said = 1; C->nmsg++;
}

void cargo_init(cargo_state *C, int n, uint32_t closed_mask, const float *travel) {
  char *p = (char *)C; for (unsigned i = 0; i < sizeof *C; i++) p[i] = 0;
  C->n = n < 0 ? 0 : n > CG_MAX ? CG_MAX : n;
  for (int i = 0; i < C->n; i++) {
    C->closed[i] = (closed_mask >> i) & 1;
    float tr = travel ? travel[i] : 0; C->travel[i] = tr > 0.005f && tr < 10 ? tr : 0.15f;
  }
}

static void move(cargo_state *C, int i, int closed) {
  if (C->closed[i] == closed) return;
  C->closed[i] = (uint8_t)closed; C->moving[i] = C->travel[i] + CG_SETTLE; C->was_loaded[i] = C->loaded[i];
}

int cargo_command(cargo_state *C, int latch, int action, const char *from) {
  if (action < CG_OPEN || action > CG_TOGGLE) return -2;
  if (latch != CG_ALL && (latch < 0 || latch >= C->n)) return -1;
  if (latch == CG_ALL && !C->n) return -1;
  int lo = latch == CG_ALL ? 0 : latch, hi = latch == CG_ALL ? C->n : latch + 1;
  for (int i = lo; i < hi; i++) move(C, i, action == CG_TOGGLE ? !C->closed[i] : action == CG_CLOSE);
  C->ncmd++;
  const char *what = action == CG_OPEN ? " opening" : action == CG_CLOSE ? " closing" : (latch != CG_ALL && C->closed[lo] ? " closing" : " opening");
  if (latch == CG_ALL) say(C, action == CG_OPEN ? "all latches opening" : action == CG_CLOSE ? "all latches closing" : "all latches toggled", -1, 0);
  else say(C, "latch ", latch, what);
  if (from && from[0]) {                     /* "… (radio)" */
    int i = 0; while (C->msg[i]) i++;
    if (i < 58) { C->msg[i++] = ' '; C->msg[i++] = '('; for (int j = 0; from[j] && i < 62; j++) C->msg[i++] = from[j]; C->msg[i++] = ')'; C->msg[i] = 0; }
  }
  return 0;
}

void cargo_switches(cargo_state *C, uint32_t loaded, uint32_t sw) {
  C->has_sw = sw;
  for (int i = 0; i < C->n; i++) C->loaded[i] = (uint8_t)(((sw & loaded) >> i) & 1);
}

void cargo_step(cargo_state *C, float dt) {
  for (int i = 0; i < C->n; i++) {
    if (C->moving[i] <= 0) continue;
    C->moving[i] -= dt; if (C->moving[i] > 0) continue;
    C->moving[i] = 0;
    if (!((C->has_sw >> i) & 1)) { say(C, "latch ", i, C->closed[i] ? " closed" : " open"); continue; }
    if (C->closed[i]) say(C, "latch ", i, C->loaded[i] ? " closed: holding a load" : " closed: nothing in it");
    else say(C, "latch ", i, C->loaded[i] ? " open but still loaded: stuck?" : C->was_loaded[i] ? " open: load released" : " open: empty");
  }
}

uint32_t cargo_drive(const cargo_state *C) { uint32_t m = 0; for (int i = 0; i < C->n; i++) if (C->closed[i]) m |= 1u << i; return m; }


int cargo_from_rc(cargo_state *C, const struct rc_input *in, double t) {
  if (in->cmd_seq == C->cmd_seen) return 0;
  C->cmd_seen = in->cmd_seq;
  if (in->cmd != RC_CMD_LATCH || t - in->t_cmd > RC_CMD_FRESH_S) return 0;
  int latch = (int)(in->cmd_v[0] < 0 ? in->cmd_v[0] - 0.5f : in->cmd_v[0] + 0.5f), action = (int)(in->cmd_v[1] + 0.5f);
  return cargo_command(C, latch < 0 ? CG_ALL : latch, action, "radio") == 0;
}
