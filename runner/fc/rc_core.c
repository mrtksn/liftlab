/* The pilot's radio: see rc_core.h. */
#include "rc_core.h"
#include "wasm_math.h"

static float clampf_(float x, float a, float b) { return x < a ? a : x > b ? b : x; }
static float dead(float x) { return fabsf(x) < 0.05f ? 0 : (x - (x > 0 ? 0.05f : -0.05f)) / 0.95f; }
static void say(rc_pilot *P, const char *s) { int i = 0; for (; s[i] && i < 63; i++) P->msg[i] = s[i]; P->msg[i] = 0; P->said = 1; }

int rc_stick_cmd(const rc_input *in, double t, fc_cmd *c) {
  if (!rc_link_ok(in, t)) return -1;
  int fresh = t - in->t_ch < RC_STALE_S;                         /* (stale: level and keep the heading, throttle as it was) */
  c->arm = in->ch[RC_ARM] > 0; c->roll = fresh ? in->ch[RC_ROLL] : 0; c->pitch = fresh ? in->ch[RC_PITCH] : 0; c->yaw = fresh ? -in->ch[RC_YAW] : 0;   /* (yaw stick right: turn right, the flight core's negative yaw) */
  c->throttle = clampf_((in->ch[RC_THR] + 1) * 0.5f, 0, 1); c->test_motor = -1; c->test_throttle = 0; c->guided = 0;
  c->acc[0] = c->acc[1] = c->acc[2] = 0; c->heading = 0;
  return 0;
}

float rc_cmd_scale(int cmd, int k) { return cmd == RC_CMD_GOTO ? (k < 3 ? 100.0f : k == 3 ? 1000.0f : 1.0f) : 1.0f; }

void rc_pilot_init(rc_pilot *P) { char *p = (char *)P; for (unsigned i = 0; i < sizeof *P; i++) p[i] = 0; }

/* the speed levels, as the simulator's keys: horizontal [m/s], vertical [m/s], turn [rad/s] */
static const float LEVEL[3][3] = { { 1, 0.6f, 0.785f }, { 3, 1.5f, 1.571f }, { 6, 3, 2.618f } };
#define RC_ACCEL 3.0f                     /* how fast the commanded velocity ramps [m/s²] */
#define BOX_XY 25.0f
#define BOX_ZLO 0.3f
#define BOX_ZHI 15.0f

int rc_pilot_step(rc_pilot *P, const rc_input *in, double t, nav_state *N, const nav_out *o, float dt, nav_sp *sp) {
  int ok = rc_link_ok(in, t);
  /* the link: lost in flight, the navigation flies home and lands; back, it holds where it is */
  if (!ok && in->frames && !P->lost) {
    P->lost = 1;
    if (P->fly && o->have_home && !N->landed) { N->rc_rth = 1; say(P, "radio link lost: flying home to land"); }
    else { P->arm = 0; P->fly = 0; say(P, "radio link lost"); }
  }
  if (ok && P->lost) {
    P->lost = 0;
    if (N->rc_rth) { N->rc_rth = 0; if (N->sup_mode < 2) { N->auto_on = 0; N->auto_land = 0; } }
    for (int k = 0; k < 3; k++) { P->target[k] = o->p[k]; P->vref[k] = 0; }
    if (P->target[2] < BOX_ZLO) P->target[2] = BOX_ZLO;
    say(P, "radio link back: holding here");
  }
  if (ok) {
    P->arm = in->ch[RC_ARM] > 0;
    P->level = in->ch[RC_LEVEL] < -0.33f ? 0 : in->ch[RC_LEVEL] > 0.33f ? 2 : 1;
    int fly = in->ch[RC_FLY] > 0;
    if (fly && !P->fly && !P->have_target) { P->target[0] = o->p[0]; P->target[1] = o->p[1]; P->target[2] = 1.5f; P->heading = sp->heading; P->have_target = 1; }
    P->fly = fly;
    /* a ground-station "go to" */
    if (in->cmd_seq != P->cmd_seen) {
      P->cmd_seen = in->cmd_seq;
      if (in->cmd == RC_CMD_GOTO) {
        P->target[0] = clampf_(in->cmd_v[0], -BOX_XY, BOX_XY); P->target[1] = clampf_(in->cmd_v[1], -BOX_XY, BOX_XY);
        P->target[2] = clampf_(in->cmd_v[2], BOX_ZLO, BOX_ZHI); P->heading = in->cmd_v[3]; P->have_target = 1;
        for (int k = 0; k < 3; k++) P->vref[k] = 0;
      } else if (in->cmd == RC_CMD_LEARN) P->learn_req = (int)in->cmd_v[0];
    }
    int hold = in->ch[RC_HOLD] > 0.5f, home = in->ch[RC_HOME] > 0.5f;
    if (hold && !P->hold_was && o->have_home) { for (int k = 0; k < 3; k++) { P->target[k] = o->p[k]; P->vref[k] = 0; } P->target[2] = clampf_(P->target[2], BOX_ZLO, BOX_ZHI); }
    if (home && !P->home_was) { P->target[0] = P->target[1] = 0; P->target[2] = 1.5f; for (int k = 0; k < 3; k++) P->vref[k] = 0; }
    P->hold_was = hold; P->home_was = home;
  }
  /* the sticks move the target (not while the navigation flies home or lands by itself) */
  float want[3] = { 0, 0, 0 }, yaw = 0;
  int own = N->sup_mode >= 2 || N->rc_rth || N->landed;
  if (ok && !own && t - in->t_ch < RC_STALE_S) {                   /* (stale channels: the sticks count as centred) */
    const float *L = LEVEL[P->level];
    float lim = N->lim_speed > 0 && N->lim_speed < L[0] ? N->lim_speed : L[0];   /* the supervisor's speed limit */
    float f = dead(in->ch[RC_PITCH]), r = dead(in->ch[RC_ROLL]), u = dead(in->ch[RC_THR]);
    yaw = -dead(in->ch[RC_YAW]) * L[2];                              /* stick right turns right (heading decreases: z up) */
    float c = cosf(P->heading), s = sinf(P->heading), hx = f * c + r * s, hy = f * s - r * c, hn = sqrtf(hx * hx + hy * hy);
    if (hn > 1) { hx /= hn; hy /= hn; }
    want[0] = hx * lim; want[1] = hy * lim; want[2] = u * L[1];
  }
  float dv = RC_ACCEL * dt;
  for (int k = 0; k < 3; k++) P->vref[k] += clampf_(want[k] - P->vref[k], -dv, dv);
  if (own) for (int k = 0; k < 3; k++) P->vref[k] = 0;
  for (int k = 0; k < 3; k++) P->target[k] += P->vref[k] * dt;
  for (int k = 0; k < 2; k++) if (fabsf(P->target[k]) > BOX_XY) { P->target[k] = clampf_(P->target[k], -BOX_XY, BOX_XY); P->vref[k] = 0; }
  if (P->target[2] < BOX_ZLO || P->target[2] > BOX_ZHI) { P->target[2] = clampf_(P->target[2], BOX_ZLO, BOX_ZHI); P->vref[2] = 0; }
  P->heading += yaw * dt; if (P->heading > 3.14159265f) P->heading -= 6.2831853f; else if (P->heading < -3.14159265f) P->heading += 6.2831853f;
  for (int k = 0; k < 3; k++) { sp->target[k] = P->target[k]; sp->vref[k] = P->vref[k]; }
  sp->heading = P->heading; sp->fly = P->fly && (P->have_target || o->have_home);
  return P->arm && !(P->lost && !P->fly) && !N->landed;   /* (landed by itself: it stays disarmed) */
}

int rc_pack(const rc_input *in, double t, float *out) {
  int k = 0;
  for (int i = 0; i < 16; i++) out[k++] = in->ch[i];
  out[k++] = in->frames ? (float)(t - in->t_ch) : 1e3f;
  out[k++] = in->up_rssi; out[k++] = in->up_lq; out[k++] = in->up_snr; out[k++] = in->down_rssi; out[k++] = in->down_lq;
  out[k++] = (float)(in->cmd_seq & 0xFFFF); out[k++] = (float)in->cmd; for (int i = 0; i < 6; i++) out[k++] = in->cmd_v[i];
  return k;
}
void rc_unpack(rc_input *in, const float *p, int n, double t) {
  if (n < RC_PACK_N) return;
  int k = 0;
  for (int i = 0; i < 16; i++) in->ch[i] = p[k++];
  float age = p[k++];
  if (age < 100) { in->t_ch = t - age; if (!in->frames) in->frames = 1; else in->frames++; }
  in->up_rssi = p[k++]; in->up_lq = p[k++]; in->up_snr = p[k++]; in->down_rssi = p[k++]; in->down_lq = p[k++];
  uint32_t seq = (uint32_t)p[k++]; int cmd = (int)p[k++];
  if (seq != (in->cmd_seq & 0xFFFF)) { in->cmd_seq = seq; in->cmd = cmd; for (int i = 0; i < 6; i++) in->cmd_v[i] = p[k + i]; }
}
