/* The rope spin test: see spin_core.h. */
#include "spin_core.h"
#if defined(__wasm__)                 /* the simulator's build: no C library */
typedef __SIZE_TYPE__ size_t;
void *memset(void *d, int c, size_t n);
#define fabsf(x) __builtin_fabsf(x)
#define fabs(x) __builtin_fabs(x)
#define sqrt(x) __builtin_sqrt(x)
#define sqrtf(x) __builtin_sqrtf(x)
#else
#include <math.h>
#include <string.h>
#endif

#define SPIN_SETTLE_S 1.5      /* at the base throttle before the steps, and after them */
#define SPIN_LTEL_GAP 0.3      /* no telemetry for this long: stop */
#define SPIN_MAX_YAW 8.0f      /* [rad/s] turning faster: stop (the step is too big for this rope) */
#define SPIN_MAX_TILT 3.0f     /* [rad/s] rolling or pitching faster: stop (it swings into something) */
#define SPIN_SEG 8             /* doublet segments: +T, then 6 of ∓2T, then ±T */
#define SPIN_H 0.0005          /* [s] the fit's own step */

void spin_init(spin_state *S) { memset(S, 0, sizeof *S); }

static float seg_sign(int i) { return i % 2 ? -1.0f : 1.0f; }
static double seg_len(const spin_state *S, int i) { return (i == 0 || i == S->n_seg - 1 ? 1 : 2) * (double)S->half; }

int spin_start(spin_state *S, int n, const float *yaw, float base, float step, float half, double t) {
  if (S->phase == SPIN_SETTLE || S->phase == SPIN_STEP || S->phase == SPIN_END) { fc_fmt(S->msg, (int)sizeof S->msg, "spintest: already running"); return -1; }
  int up = 0, down = 0;
  for (int i = 0; i < n && i < FC_MAX_MOTORS; i++) { S->dir[i] = yaw[i] > 1e-6f ? 1.0f : yaw[i] < -1e-6f ? -1.0f : 0.0f; up += S->dir[i] > 0; down += S->dir[i] < 0; }
  if (n < 2 || n > FC_MAX_MOTORS || !up || !down) { fc_fmt(S->msg, (int)sizeof S->msg, "spintest: needs motors turning both ways (the airframe's spin directions)"); S->phase = SPIN_FAILED; return -1; }
  if (!(base >= 0.05f && base <= 0.7f) || !(step >= 0.01f && step <= 0.25f) || base - step < 0.02f || base + step > 0.95f || !(half >= 0.08f && half <= 1.0f)) {
    fc_fmt(S->msg, (int)sizeof S->msg, "spintest: base 0.05 to 0.7, step 0.01 to 0.25 (and base - step at least 0.02), half period 0.08 to 1 s"); S->phase = SPIN_FAILED; return -1;
  }
  S->n = n; S->base = base; S->step = step; S->half = half; S->n_seg = SPIN_SEG;
  S->delay = S->lag = S->gain = S->kick = S->r2 = S->spring = S->drag = 0;
  S->phase = SPIN_SETTLE; S->t0 = S->seg_t = t; S->seg = 0; S->ns = 0; S->have_ltel = 0; S->last_ltel = t; S->tw = 0; S->ok = 0; S->peak_rate = 0;
  fc_fmt(S->msg, (int)sizeof S->msg, "spintest: settling at %.2f", (double)base);
  return 0;
}

void spin_stop(spin_state *S, const char *why) {
  if (S->phase == SPIN_SETTLE || S->phase == SPIN_STEP || S->phase == SPIN_END) { S->phase = SPIN_FAILED; fc_fmt(S->msg, (int)sizeof S->msg, "spintest: stopped: %s", why); }
}

int spin_tick(spin_state *S, double t, float *out) {
  if (S->phase != SPIN_SETTLE && S->phase != SPIN_STEP && S->phase != SPIN_END) return 0;
  if (t - S->last_ltel > SPIN_LTEL_GAP && t - S->t0 > SPIN_LTEL_GAP) { spin_stop(S, S->have_ltel ? "the flight core's telemetry stopped" : "no telemetry from the flight core (is the learning or the supervisor on?)"); return 0; }
  float d = 0;
  if (S->phase == SPIN_SETTLE && t - S->seg_t >= SPIN_SETTLE_S) { S->phase = SPIN_STEP; S->seg = 0; S->seg_t = t; fc_fmt(S->msg, (int)sizeof S->msg, "spintest: stepping"); }
  if (S->phase == SPIN_STEP) {
    while (S->seg < S->n_seg && t - S->seg_t >= seg_len(S, S->seg)) { S->seg_t += seg_len(S, S->seg); S->seg++; }
    if (S->seg >= S->n_seg) { S->phase = SPIN_END; S->seg_t = t; }
    else d = seg_sign(S->seg);
  }
  if (S->phase == SPIN_END && t - S->seg_t >= SPIN_SETTLE_S * 0.6) {
    S->phase = spin_fit(S) ? SPIN_FAILED : SPIN_DONE;
    return 0;
  }
  for (int i = 0; i < S->n; i++) out[i] = S->base + d * S->dir[i] * S->step;
  return S->n;
}

void spin_ltel(spin_state *S, const float *lt, int n, double t) {
  if (S->phase != SPIN_SETTLE && S->phase != SPIN_STEP && S->phase != SPIN_END) return;
  if (n < 19) return;
  int nm = (int)lt[17];
  if (nm != S->n || n < 19 + nm) { spin_stop(S, "the flight core has another airframe"); return; }
  /* the flight core's time, unwrapped (FC_LTEL_WRAP) */
  double raw = lt[0];
  if (S->have_ltel) {
    double dt = raw - S->prev_raw; if (dt < -FC_LTEL_WRAP / 2) dt += FC_LTEL_WRAP;
    if (dt <= 0) { S->last_ltel = t; return; }                       /* (the same or an older frame: dropped) */
    if (dt > 1) dt = 0.005;                                           /* (a gap: carried on) */
    S->tw += dt;
  }
  S->prev_raw = raw; S->have_ltel = 1; S->last_ltel = t;
  int st = (int)lt[1];
  if (st == FC_ARMED || st == FC_FAILSAFE) { spin_stop(S, "it was armed"); return; }
  float wx = lt[10], wy = lt[11], wz = lt[12];
  if (fabsf(wz) > SPIN_MAX_YAW) { spin_stop(S, "turning too fast: a smaller step"); return; }
  if (fabsf(wx) > SPIN_MAX_TILT || fabsf(wy) > SPIN_MAX_TILT) { spin_stop(S, "swinging: steady the rope"); return; }
  if (fabsf(wz) > S->peak_rate) S->peak_rate = fabsf(wz);
  if (S->ns >= SPIN_MAX) return;
  /* the differential throttle: each motor's change from the base, signed by its yaw, averaged over the motors */
  float D = 0, cnt = 0;
  for (int i = 0; i < nm; i++) if (S->dir[i] != 0) { D += S->dir[i] * (lt[19 + i] - S->base); cnt += 1; }
  if (st != FC_TESTING) D = 0;                                      /* (not running the test: the motors are still) */
  S->st[S->ns] = (float)S->tw; S->sd[S->ns] = cnt > 0 ? D / cnt : 0; S->sw[S->ns] = wz; S->ns++;
}

/* ───────── the fit ───────── */
static float fx[SPIN_MAX], fw[SPIN_MAX], fth[SPIN_MAX], tmp[SPIN_MAX];
static float ia[SPIN_MAX], ix[SPIN_MAX], ith[SPIN_MAX], ft[SPIN_MAX];   /* the integrated fit: ∫a, ∫x, ∫θ, time */
/* a zero-phase smoother (three-point average, twice), the same for every signal so their relation holds */
static void avg3(const float *a, float *o, int n) { for (int i = 0; i < n; i++) { int l = i > 0 ? i - 1 : i, r = i < n - 1 ? i + 1 : i; o[i] = (a[l] + a[i] + a[r]) / 3.0f; } }
static void smooth(const float *a, float *o, int n) { static float mid[SPIN_MAX]; avg3(a, mid, n); avg3(mid, o, n); }
/* The differential throttle after the delay (into fa) and through the motors' spin-up lag too (into fx), each
 * averaged over each telemetry window as the flight core averages (a sample covers the window before its time),
 * then smoothed. The yaw torque is a mix of the two: a motor's reaction torque jumps with its current the moment the
 * throttle steps (fa), then settles to the drag of its new speed, which follows the spin-up lag (fx). */
static float fa[SPIN_MAX], ta[SPIN_MAX];
static void model_x(const spin_state *S, double delay, double lag) {
  int n = S->ns, k = 0; double x = 0, acc = 0, acca = 0; int cnt = 0;
  const double a = SPIN_H / (lag + SPIN_H / 2);
  double t = S->st[0] - 0.02;
  for (int i = 0; i < n; i++) {
    while (t < S->st[i]) {
      double tc = t - delay; while (k < n - 1 && S->st[k] < tc) k++;
      double u = tc < S->st[0] - 0.006 ? 0 : S->sd[k];
      x += a * (u - x); t += SPIN_H; acc += x; acca += u; cnt++;
    }
    tmp[i] = cnt ? (float)(acc / cnt) : (float)x; ta[i] = cnt ? (float)(acca / cnt) : 0; acc = acca = 0; cnt = 0;
  }
  smooth(tmp, fx, n); smooth(ta, fa, n);
  ia[0] = ix[0] = 0;                                              /* (and their integrals, for the fit on ω itself) */
  for (int i = 1; i < n; i++) { const double dt = S->st[i] - S->st[i - 1]; ia[i] = ia[i - 1] + (float)(0.5 * (fa[i] + fa[i - 1]) * dt); ix[i] = ix[i - 1] + (float)(0.5 * (fx[i] + fx[i - 1]) * dt); }
}
/* Least squares on m regressors (cols; 0: a constant) for y over lo..hi: the coefficients into co, the residual back. */
#define SPIN_COLS 6
static double lsq(int m, const float *const *cols, const float *sign, const float *y, int lo, int hi, double *co) {
  double M[SPIN_COLS][SPIN_COLS + 1] = { { 0 } };
  for (int i = lo; i < hi; i++) {
    double v[SPIN_COLS]; for (int p = 0; p < m; p++) v[p] = cols[p] ? sign[p] * cols[p][i] : 1;
    for (int p = 0; p < m; p++) { for (int q = 0; q < m; q++) M[p][q] += v[p] * v[q]; M[p][m] += v[p] * y[i]; }
  }
  for (int a = 0; a < m; a++) {                                   /* (Gauss, partial pivoting) */
    int pv = a; for (int b = a + 1; b < m; b++) if (fabs(M[b][a]) > fabs(M[pv][a])) pv = b;
    if (fabs(M[pv][a]) < 1e-12) return 1e30;
    if (pv != a) for (int j = 0; j <= m; j++) { double x = M[a][j]; M[a][j] = M[pv][j]; M[pv][j] = x; }
    for (int b = 0; b < m; b++) if (b != a) { double f = M[b][a] / M[a][a]; for (int j = a; j <= m; j++) M[b][j] -= f * M[a][j]; }
  }
  for (int a = 0; a < m; a++) co[a] = M[a][m] / M[a][a];
  double e = 0;
  for (int i = lo; i < hi; i++) { double d = y[i]; for (int p = 0; p < m; p++) d -= co[p] * (cols[p] ? sign[p] * cols[p][i] : 1); e += d * d; }
  return e;
}
/* The yaw rate itself, ω = p·∫a + q·∫x − k·∫θ − c·θ + b·t + ω₀ (the rate of change integrated: the gyro's noise isn't
 * differentiated, and b, a steady torque, takes up motors that don't quite cancel at the base throttle): the residual,
 * the gain g = p + q, the kick r = p / g (never against the gain: a reaction torque doesn't jump backwards; if the
 * fit wants it there, the kick is left out). */
static int fit_kick = 1;   /* (0: the model without the kick) */
static double regress(int lo, int hi, double *g, double *r, double *k, double *c) {
  const float *c6[6] = { ia, ix, ith, fth, ft, 0 }; const float s6[6] = { 1, 1, -1, -1, 1, 1 }; double co[6];
  double e = fit_kick ? lsq(6, c6, s6, fw, lo, hi, co) : 1e30;
  if (fit_kick && e < 1e30 && fabs(co[0] + co[1]) > 1e-9 && co[0] / (co[0] + co[1]) >= 0) { *g = co[0] + co[1]; *r = co[0] / *g; *k = co[2]; *c = co[3]; return e; }
  const float *c5[5] = { ix, ith, fth, ft, 0 }; const float s5[5] = { 1, -1, -1, 1, 1 };
  e = lsq(5, c5, s5, fw, lo, hi, co);
  *g = co[0]; *r = 0; *k = co[1]; *c = co[2];
  return e;
}
/* The grid search for (delay, lag): coarse (delay 0–80 ms by 4, lag 5–300 ms by ×1.2), then fine around the best
 * (by 1 ms and ×1.04). 0 with the best residual, delay and lag; −1 if nothing fits. */
static int spin_search(const spin_state *S, int lo, int hi, double *best, double *bd, double *bl) {
  *best = 1e30; *bd = *bl = 0;
  for (double d = 0; d <= 0.0801; d += 0.004) for (double l = 0.005; l <= 0.3; l *= 1.2) {
    model_x(S, d, l); double gg, rr, kk, cc, e = regress(lo, hi, &gg, &rr, &kk, &cc);
    if (e < *best && gg != 0) { *best = e; *bd = d; *bl = l; }
  }
  if (*best >= 1e30) return -1;
  const double cd = *bd, cl = *bl;
  for (double d = cd - 0.004; d <= cd + 0.0041; d += 0.001) for (double l = cl / 1.2; l <= cl * 1.21; l *= 1.04) {
    if (d < 0) continue;
    model_x(S, d, l); double gg, rr, kk, cc, e = regress(lo, hi, &gg, &rr, &kk, &cc);
    if (e <= *best && gg != 0) { *best = e; *bd = d; *bl = l; }
  }
  return 0;
}
int spin_fit(spin_state *S) {
  int n = S->ns; S->ok = 0;
  if (n < 200) { fc_fmt(S->msg, (int)sizeof S->msg, "spintest: too few samples (%d): was the telemetry coming?", n); return -1; }
  /* the yaw rate (lightly smoothed), the angle turned and its integral, the time */
  smooth(S->sw, fw, n);
  fth[0] = ith[0] = 0; ft[0] = 0;
  for (int i = 1; i < n; i++) { const double dt = S->st[i] - S->st[i - 1]; fth[i] = fth[i - 1] + (float)(0.5 * (S->sw[i] + S->sw[i - 1]) * dt); ith[i] = ith[i - 1] + (float)(0.5 * (fth[i] + fth[i - 1]) * dt); ft[i] = S->st[i] - S->st[0]; }
  const int lo = 4, hi = n - 4;
  /* what's left to explain once a steady torque, the rope and the air have had their go: the fit is graded on that */
  double sst; { const float *cn[4] = { ith, fth, ft, 0 }; const float sn[4] = { -1, -1, 1, 1 }; double co[4]; sst = lsq(4, cn, sn, fw, lo, hi, co); }
  /* With the kick and without it; the kick is kept only if it leaves at least 20% less unexplained (with noise, a
   * small kick can stand in for a few milliseconds of delay: the simpler model is the safer one then) */
  double bk, dk, lk, b0, d0, l0;
  fit_kick = 1; if (spin_search(S, lo, hi, &bk, &dk, &lk)) { fc_fmt(S->msg, (int)sizeof S->msg, "spintest: no fit: did it turn at all?"); return -1; }
  fit_kick = 0; if (spin_search(S, lo, hi, &b0, &d0, &l0)) b0 = 1e30;
  const int kick = !(b0 < 1e30) || bk < 0.8 * b0;
  double best = kick ? bk : b0, bd = kick ? dk : d0, bl = kick ? lk : l0, g = 0, r = 0, k = 0, c = 0;
  fit_kick = kick;
  model_x(S, bd, bl); regress(lo, hi, &g, &r, &k, &c); fit_kick = 1;
  S->delay = (float)bd; S->lag = (float)bl; S->gain = (float)g; S->kick = (float)r; S->spring = (float)k; S->drag = (float)c;
  S->r2 = sst > 0 ? (float)(1 - best / sst) : 0;
  /* turning the other way than the airframe's spin directions say: the gain comes out negative */
  const char *why = S->r2 >= 0.6f && g < 0 ? "it turned the other way than the airframe's spin directions say (each motor's direction, or the IMU's heading)" :
                    S->r2 < 0.6f ? "the fit is poor (a bigger step, or a steadier rope)" : bl < 0.0055 || bl > 0.29 ? "the spin-up is at the edge of what it tries" : bd > 0.079 ? "the delay is at the edge of what it tries" : 0;
  S->ok = !why;
  if (why) fc_fmt(S->msg, (int)sizeof S->msg, "spintest: done, but %s: delay %d ms, spin-up %d ms, fit %.2f", why, (int)(bd * 1000 + 0.5), (int)(bl * 1000 + 0.5), (double)S->r2);
  else fc_fmt(S->msg, (int)sizeof S->msg, "spintest: done: delay %d ms, spin-up %d ms, yaw %.1f rad/s2 per unit, fit %.2f", (int)(bd * 1000 + 0.5), (int)(bl * 1000 + 0.5), (double)g, (double)S->r2);
  return S->ok ? 0 : -1;
}

void spin_said(const spin_state *S, char *out, int cap) {
  if (S->phase == SPIN_IDLE) { fc_fmt(out, cap, "spintest: idle (spintest BASE STEP [HALF], e.g. spintest 0.3 0.05)"); return; }
  if (S->phase == SPIN_DONE || (S->phase == SPIN_FAILED && S->ns >= 200 && S->lag > 0)) {
    /* the numbers in a fixed form after the words, for LiftLab: result DELAY_MS LAG_MS GAIN R2 OK PEAK KICK */
    fc_fmt(out, cap, "%s; result %d %d %.2f %.3f %d %.2f %.2f", S->msg, (int)(S->delay * 1000 + 0.5), (int)(S->lag * 1000 + 0.5), (double)S->gain, (double)S->r2, S->ok, (double)S->peak_rate, (double)S->kick);
    return;
  }
  fc_fmt(out, cap, "%s", S->msg);
}

/* ───────── the rope lift test (spin_core.h) ───────── */
#define LIFT_PULSE_S 0.6       /* [s] each pulse, long enough for slow motors to get there (read over its last quarter second) */
#define LIFT_RING 50           /* frames a pulse is read over: its last quarter second */
#define LIFT_BRAKE_S 0.5       /* [s] at base after a pulse, to stop it climbing, before the hold */
#define LIFT_PULSE_DV 0.7f     /* [m/s] a pulse that has sped it up this much ends there (it would climb too far) */
#define LIFT_REST_S 1.5        /* [s] at base after lift-off and between pulses */
#define LIFT_OVER 1.04f        /* a pulse at least this far over the reading at rest lifted it */
#define LIFT_TAU 0.7f          /* [s] the leaky vertical speed's memory (longer than a springy rope's bounce) */
#define LIFT_V 0.35f           /* [m/s] the leaky speed over this: lifted off (a rope's bounce: about ±0.13) */
#define LIFT_VMAX 1.2f         /* [m/s] over this: it's flying up, stop */
void lift_init(lift_state *L) { memset(L, 0, sizeof *L); }
static int lift_running(const lift_state *L) { return L->phase >= LIFT_SETTLE && L->phase <= LIFT_PULSE; }
int lift_start(lift_state *L, int n, float base, float top, float rate, float step, double t) {
  if (lift_running(L)) { fc_fmt(L->msg, (int)sizeof L->msg, "lifttest: already running"); return -1; }
  if (n < 1 || n > FC_MAX_MOTORS || !(base >= 0.05f && base <= 0.7f) || !(top > base && top <= 1) || !(rate >= 0.005f && rate <= 0.2f) || !(step == 0 || (step >= 0.01f && step <= 0.2f))) {
    fc_fmt(L->msg, (int)sizeof L->msg, "lifttest: base 0.05 to 0.7, top above it up to 1, rate 0.005 to 0.2 a second, step 0 (no pulses) or 0.01 to 0.2"); L->phase = LIFT_FAILED; return -1;
  }
  lift_init(L);
  L->n = n; L->base = base; L->top = top; L->rate = rate; L->step = step; L->u = base;
  L->phase = LIFT_SETTLE; L->t0 = L->ph_t = L->last_ltel = t;
  fc_fmt(L->msg, (int)sizeof L->msg, "lifttest: settling at %.2f", (double)base);
  return 0;
}
void lift_stop(lift_state *L, const char *why) {
  if (lift_running(L)) { L->phase = L->lift_u > 0 && L->np ? LIFT_DONE : LIFT_FAILED; L->ok = L->phase == LIFT_DONE; fc_fmt(L->msg, (int)sizeof L->msg, "lifttest: stopped: %s", why); }
}
int lift_tick(lift_state *L, double t, float *out) {
  if (!lift_running(L)) return 0;
  if (t - L->last_ltel > SPIN_LTEL_GAP && t - L->t0 > SPIN_LTEL_GAP) { lift_stop(L, L->have_ltel ? "the flight core's telemetry stopped" : "no telemetry from the flight core (is the learning or the supervisor on?)"); return 0; }
  const double dt = t - L->ph_t;
  if (L->phase == LIFT_SETTLE && dt >= SPIN_SETTLE_S) {
    if (L->n0 < 20) { lift_stop(L, "too little telemetry at rest"); return 0; }
    const double ux = L->fr[0] / L->n0, uy = L->fr[1] / L->n0, uz = L->fr[2] / L->n0, gn = sqrt(ux * ux + uy * uy + uz * uz);
    if (!(gn > 1)) { lift_stop(L, "the accelerometer reads nothing at rest"); return 0; }
    L->up[0] = (float)(ux / gn); L->up[1] = (float)(uy / gn); L->up[2] = (float)(uz / gn);
    L->g_rest = (float)gn; L->alt_rest = (float)(L->a0 / L->n0);
    L->phase = LIFT_RAMP; L->ph_t = t; L->vl = 0; fc_fmt(L->msg, (int)sizeof L->msg, "lifttest: rising from %.2f", (double)L->base);
  }
  if (L->phase == LIFT_RAMP) {
    L->u = L->base + L->rate * (float)(t - L->ph_t);
    if (L->u >= L->top) { L->phase = LIFT_DONE; L->ok = 0; fc_fmt(L->msg, (int)sizeof L->msg, "lifttest: done: it didn't lift off up to %.2f", (double)L->top); return 0; }
  }
  if (L->phase == LIFT_REST && ((L->step == 0 && t - L->ph_t >= LIFT_BRAKE_S + 0.5) || (t - L->ph_t >= LIFT_REST_S && (L->calm >= 60 || t - L->ph_t >= 4.0)))) {   /* (and the bounce died down: calm for 0.3 s) */
    float u = L->lift_u + (float)L->k * L->step;                  /* (from lift-off up: small steps, it climbs little) */
    if (L->step == 0) {                                            /* lift-off only: done once it has braked */
      L->phase = LIFT_DONE; L->ok = 1;
      fc_fmt(L->msg, (int)sizeof L->msg, "lifttest: done: lifted off at %.3f on %.2f V", (double)L->lift_u, (double)L->lift_v);
      return 0;
    }
    if (L->k >= LIFT_PULSES || u > L->top) {
      float top = 0; for (int i = 0; i < L->np; i++) if (L->pf[i] > top) top = L->pf[i];
      L->phase = LIFT_DONE; L->ok = top >= LIFT_OVER;
      if (L->ok) fc_fmt(L->msg, (int)sizeof L->msg, "lifttest: done: lifted off at %.3f on %.2f V, %d pulses", (double)L->lift_u, (double)L->lift_v, L->np);
      else fc_fmt(L->msg, (int)sizeof L->msg, "lifttest: done, but no pulse lifted it (a false lift-off at %.3f?): a bigger step", (double)L->lift_u);
      return 0;
    }
    L->phase = LIFT_PULSE; L->ph_t = t; L->u = u; L->pri = 0; L->dv = 0;
    fc_fmt(L->msg, (int)sizeof L->msg, "lifttest: pulse %d at %.2f", L->k + 1, (double)u);
  }
  if (L->phase == LIFT_PULSE && (t - L->ph_t >= LIFT_PULSE_S || L->dv > LIFT_PULSE_DV)) {
    { const int nr = L->pri < LIFT_RING ? L->pri : LIFT_RING; double f = 0, vv = 0;   /* (its last quarter second, whenever it ended) */
      for (int i = 0; i < nr; i++) { f += L->pring[i]; vv += L->pvring[i]; }
      if (nr >= 20) { L->pu[L->np] = L->u; L->pf[L->np] = (float)(f / nr); L->pv[L->np] = (float)(vv / nr); L->np++; } }
    L->k++; L->phase = LIFT_REST; L->ph_t = t; L->u = L->base; L->calm = 0;   /* (brake at base, then the hold) */
  }
  if (L->phase == LIFT_REST && t - L->ph_t >= LIFT_BRAKE_S) L->u = L->step == 0 ? L->base : L->hold;
  for (int i = 0; i < L->n; i++) out[i] = L->u;
  return L->n;
}
void lift_ltel(lift_state *L, const float *lt, int n, double t) {
  if (!lift_running(L) || n < 19) return;
  L->have_ltel = 1; L->last_ltel = t;
  int st = (int)lt[1];
  if (st == FC_ARMED || st == FC_FAILSAFE) { lift_stop(L, "it was armed"); return; }
  if (fabsf(lt[12]) > SPIN_MAX_YAW) { lift_stop(L, "turning too fast"); return; }
  if (fabsf(lt[10]) > SPIN_MAX_TILT || fabsf(lt[11]) > SPIN_MAX_TILT) { lift_stop(L, "swinging or tipping"); return; }
  const float fx = lt[7], fy = lt[8], fz = lt[9], alt = lt[14], have_alt = lt[16] > 0.5f, v = lt[13];
  int nm = (int)lt[17]; float um = 0; for (int i = 0; i < nm && 19 + i < n; i++) um += lt[19 + i]; um = nm ? um / (float)nm : 0;
  if (L->ns < SPIN_MAX) { int k = L->ns++; L->st[k] = (float)(t - L->t0); L->su[k] = um; L->sf[k][0] = fx; L->sf[k][1] = fy; L->sf[k][2] = fz; L->sa[k] = alt; L->sp[k] = (float)L->phase; }
  if (L->phase == LIFT_SETTLE) { L->fr[0] += fx; L->fr[1] += fy; L->fr[2] += fz; L->a0 += have_alt ? alt : 0; L->n0++; return; }
  /* the accelerometer along its direction at rest (an average of a vibrating vector's length reads high; along a
   * fixed direction the vibration averages out) */
  const float fm = (fx * L->up[0] + fy * L->up[1] + fz * L->up[2]);
  /* the vertical speed, from the accelerometer along its direction at rest less its reading then, leaking away over
   * LIFT_TAU: the rope's bounce averages out, a lift keeps it climbing. (The barometer is no use here: the props'
   * wash moves it tens of centimetres.) */
  { double dt = lt[0] - L->prev_lt; if (dt < -FC_LTEL_WRAP / 2) dt += FC_LTEL_WRAP; if (!(dt > 0 && dt < 0.05)) dt = 0.005; L->prev_lt = lt[0];
    L->vl += ((double)fm - L->g_rest - L->vl / LIFT_TAU) * dt; }
  if (have_alt && alt - L->alt_rest > L->climb) L->climb = alt - L->alt_rest;   /* (kept, not used: see above) */
  if (fabsf(fm / L->g_rest - 1) < 0.04f) L->calm++; else L->calm = 0;   /* (frames in a row within 4% of 1 g: not bouncing) */
  if (L->phase == LIFT_RAMP && L->vl > LIFT_VMAX) { L->u = L->base; lift_stop(L, "it was flying up: a lower step or top"); return; }   /* (rising only: the rope catching it afterwards jerks it up too) */
  if (L->phase == LIFT_RAMP && st == FC_TESTING && L->vl > LIFT_V) {
    L->lift_u = um; L->lift_v = v; L->phase = LIFT_REST; L->ph_t = t; L->k = 0; L->calm = 0;
    L->hold = 0.85f * um > L->base ? 0.85f * um : L->base; L->u = L->base;   /* (brake at base, then just under hover: it settles back onto the rope gently) */
    fc_fmt(L->msg, (int)sizeof L->msg, "lifttest: lifted off at %.3f", (double)um);
  }
  /* a pulse: its second half (the motors have spun up), the accelerometer against its reading at rest */
  if (L->phase == LIFT_PULSE && st == FC_TESTING) {
    L->dv += ((double)fm - L->g_rest) * 0.005;                     /* (the speed it gained in this pulse) */
    if (t - L->ph_t >= 0.15) { L->pring[L->pri % LIFT_RING] = fm / L->g_rest; L->pvring[L->pri % LIFT_RING] = v; L->pri++; }
  }
}
void lift_said(const lift_state *L, char *out, int cap) {
  if (L->phase == LIFT_IDLE) { fc_fmt(out, cap, "lifttest: idle (lifttest BASE TOP [RATE STEP], e.g. lifttest 0.3 0.85)"); return; }
  if ((L->phase == LIFT_DONE || L->phase == LIFT_FAILED) && L->lift_u > 0) {
    fc_fmt(out, cap, "%s; result %.3f %.2f %d", L->msg, (double)L->lift_u, (double)L->lift_v, L->np);
    for (int i = 0; i < L->np; i++) fc_fmt_add(out, cap, " %.3f %.3f %.2f", (double)L->pu[i], (double)L->pf[i], (double)L->pv[i]);
    return;
  }
  fc_fmt(out, cap, "%s", L->msg);
}
