/* Pickup: see pickup_core.h. */
#include "pickup_core.h"
#include "wasm_math.h"

static void say(pickup_state *K, const char *s) { int i = 0; for (; s[i] && i < 63; i++) K->msg[i] = s[i]; K->msg[i] = 0; K->said = 1; }
static float clampf_(float x, float a, float b) { return x < a ? a : x > b ? b : x; }

void pickup_init(pickup_state *K) { char *p = (char *)K; for (unsigned i = 0; i < sizeof *K; i++) p[i] = 0; }

int pickup_start(pickup_state *K, const float spot[3], float heading, int latch, const nav_out *o, double t) {
  for (int k = 0; k < 3; k++) if (!(spot[k] == spot[k]) || spot[k] > 1e4f || spot[k] < -1e4f) { say(K, "pickup: the spot isn't a number"); return -1; }
  if (fabsf(spot[0]) > 25 || fabsf(spot[1]) > 25 || spot[2] < PK_ZMIN || spot[2] > 15) { say(K, "pickup: the spot is outside the box (25 m round home)"); return -1; }
  if (!(heading == heading)) heading = 0;
  K->phase = PK_OVER; K->latch = latch < 0 ? 0 : latch; K->heading = heading; K->done_ok = 0;
  for (int k = 0; k < 3; k++) K->spot[k] = spot[k];
  K->z_over = spot[2] + PK_ABOVE; if (o->p[2] > K->z_over) K->z_over = o->p[2];
  K->tgt[0] = spot[0]; K->tgt[1] = spot[1]; K->tgt[2] = K->z_over;
  K->t_phase = t; K->t_still = -1;
  say(K, "pickup: flying over it");
  return 0;
}

void pickup_cancel(pickup_state *K, const char *why) {
  if (!K->phase) return;
  K->phase = PK_IDLE;
  char b[64] = "pickup stopped: "; int i = 16; for (int j = 0; why && why[j] && i < 63; j++) b[i++] = why[j]; b[i] = 0;
  say(K, b);
}

static float dist_xy(const float *a, const float *b) { float dx = a[0] - b[0], dy = a[1] - b[1]; return sqrtf(dx * dx + dy * dy); }
static float speed(const float *v) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }

int pickup_step(pickup_state *K, const nav_out *o, double t, float dt, nav_sp *sp) {
  if (!K->phase) return 0;
  float vz = 0;
  switch (K->phase) {
    case PK_OVER:                                                  /* over it, high */
      if (dist_xy(o->p, K->tgt) < 0.15f && fabsf(o->p[2] - K->tgt[2]) < 0.2f && speed(o->v) < 0.3f) { K->phase = PK_DOWN; K->t_phase = t; say(K, "pickup: coming down onto it"); }
      break;
    case PK_DOWN:                                                  /* down slowly, then hold still over the spot */
      if (K->tgt[2] > K->spot[2]) { K->tgt[2] -= PK_SINK * dt; vz = -PK_SINK; if (K->tgt[2] <= K->spot[2]) { K->tgt[2] = K->spot[2]; vz = 0; } }
      else {
        int still = dist_xy(o->p, K->spot) < PK_TOL && fabsf(o->p[2] - K->spot[2]) < PK_TOL && speed(o->v) < 0.12f;
        if (!still) K->t_still = -1; else if (K->t_still < 0) K->t_still = t;
        if (K->t_still >= 0 && t - K->t_still >= PK_STILL) {
          K->phase = PK_CLOSE; K->t_phase = t; K->req_latch = K->latch; K->req_act = 1; K->nreq++;
          say(K, "pickup: over it: closing the latch");
        } else if (t - K->t_phase > PK_GIVEUP) { K->phase = PK_UP; K->t_phase = t; K->tgt[2] = K->spot[2]; say(K, "pickup: couldn't hold still over it: climbing back"); }
      }
      break;
    case PK_CLOSE:
      if (t - K->t_phase >= PK_CLOSE_S) { K->phase = PK_UP; K->t_phase = t; K->done_ok = 1; say(K, "pickup: latch closed: climbing"); }
      break;
    case PK_UP:
      if (K->tgt[2] < K->z_over) { K->tgt[2] += PK_RISE * dt; vz = PK_RISE; if (K->tgt[2] >= K->z_over) { K->tgt[2] = K->z_over; vz = 0; } }
      else if (fabsf(o->p[2] - K->z_over) < 0.15f) { K->phase = PK_IDLE; say(K, K->done_ok ? "pickup done" : "pickup gave up"); }
      break;
  }
  for (int k = 0; k < 3; k++) sp->target[k] = K->tgt[k];
  sp->vref[0] = sp->vref[1] = 0; sp->vref[2] = vz;
  sp->heading = K->heading;
  sp->target[2] = clampf_(sp->target[2], PK_ZMIN, 15);
  return 1;
}
