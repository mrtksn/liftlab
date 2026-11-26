/* The rope spin test: see spin_core.h. */
#include "spin_core.h"
#if defined(__wasm__)                 /* the simulator's build: no C library */
typedef __SIZE_TYPE__ size_t;
void *memset(void *d, int c, size_t n);
#define fabsf(x) __builtin_fabsf(x)
#define fabs(x) __builtin_fabs(x)
#define sqrt(x) __builtin_sqrt(x)
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
static float fx[SPIN_MAX], fw[SPIN_MAX], fth[SPIN_MAX], fwd[SPIN_MAX], tmp[SPIN_MAX];
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
}
/* Least squares on m regressors (cols) for fwd over lo..hi: the coefficients into co, the residual back. */
static double lsq(int m, const float *const *cols, const float *sign, int lo, int hi, double *co) {
  double M[5][6] = { { 0 } };
  for (int i = lo; i < hi; i++) {
    double v[5]; for (int p = 0; p < m; p++) v[p] = cols[p] ? sign[p] * cols[p][i] : 1;
    for (int p = 0; p < m; p++) { for (int q = 0; q < m; q++) M[p][q] += v[p] * v[q]; M[p][m] += v[p] * fwd[i]; }
  }
  for (int a = 0; a < m; a++) {                                   /* (Gauss, partial pivoting) */
    int pv = a; for (int b = a + 1; b < m; b++) if (fabs(M[b][a]) > fabs(M[pv][a])) pv = b;
    if (fabs(M[pv][a]) < 1e-12) return 1e30;
    if (pv != a) for (int j = 0; j <= m; j++) { double x = M[a][j]; M[a][j] = M[pv][j]; M[pv][j] = x; }
    for (int b = 0; b < m; b++) if (b != a) { double f = M[b][a] / M[a][a]; for (int j = a; j <= m; j++) M[b][j] -= f * M[a][j]; }
  }
  for (int a = 0; a < m; a++) co[a] = M[a][m] / M[a][a];
  double e = 0;
  for (int i = lo; i < hi; i++) { double d = fwd[i]; for (int p = 0; p < m; p++) d -= co[p] * (cols[p] ? sign[p] * cols[p][i] : 1); e += d * d; }
  return e;
}
/* ω̇ = p·a + q·x − k·θ − c·ω + b on the middle samples: the residual, the gain g = p + q, the kick r = p / g (never
 * below 0: a reaction torque doesn't jump backwards; if the fit wants it there, the kick is left out). */
static double regress(int lo, int hi, double *g, double *r, double *k, double *c) {
  const float *c5[5] = { fa, fx, fth, fw, 0 }; const float s5[5] = { 1, 1, -1, -1, 1 }; double co[5];
  double e = lsq(5, c5, s5, lo, hi, co);
  if (e < 1e30 && co[0] + co[1] > 0 && co[0] >= 0) { *g = co[0] + co[1]; *r = co[0] / *g; *k = co[2]; *c = co[3]; return e; }
  const float *c4[4] = { fx, fth, fw, 0 }; const float s4[4] = { 1, -1, -1, 1 };
  e = lsq(4, c4, s4, lo, hi, co);
  *g = co[0]; *r = 0; *k = co[1]; *c = co[2];
  return e;
}
int spin_fit(spin_state *S) {
  int n = S->ns; S->ok = 0;
  if (n < 200) { fc_fmt(S->msg, (int)sizeof S->msg, "spintest: too few samples (%d): was the telemetry coming?", n); return -1; }
  /* the yaw rate, smoothed; its rate of change; the angle turned (from the rate) */
  smooth(S->sw, fw, n);
  for (int i = 0; i < n; i++) { int l = i > 0 ? i - 1 : i, r = i < n - 1 ? i + 1 : i; double dt = S->st[r] - S->st[l]; fwd[i] = dt > 0 ? (float)((fw[r] - fw[l]) / dt) : 0; }
  { double th = 0; tmp[0] = 0; for (int i = 1; i < n; i++) { th += 0.5 * (S->sw[i] + S->sw[i - 1]) * (S->st[i] - S->st[i - 1]); tmp[i] = (float)th; } smooth(tmp, fth, n); }
  const int lo = 4, hi = n - 4;
  double sst = 0, mean = 0; for (int i = lo; i < hi; i++) mean += fwd[i]; mean /= (hi - lo);
  for (int i = lo; i < hi; i++) sst += (fwd[i] - mean) * (fwd[i] - mean);
  double best = 1e30, bd = 0, bl = 0, g = 0, r = 0, k = 0, c = 0;
  /* coarse: delay 0–80 ms by 4, lag 5–300 ms by ×1.2; then fine around the best: by 1 ms and ×1.04 */
  for (double d = 0; d <= 0.0801; d += 0.004) for (double l = 0.005; l <= 0.3; l *= 1.2) {
    model_x(S, d, l); double gg, rr, kk, cc, e = regress(lo, hi, &gg, &rr, &kk, &cc);
    if (e < best && gg > 0) { best = e; bd = d; bl = l; }
  }
  if (best >= 1e30) { fc_fmt(S->msg, (int)sizeof S->msg, "spintest: no fit: did it turn at all?"); return -1; }
  double cd = bd, cl = bl;
  for (double d = cd - 0.004; d <= cd + 0.0041; d += 0.001) for (double l = cl / 1.2; l <= cl * 1.21; l *= 1.04) {
    if (d < 0) continue;
    model_x(S, d, l); double gg, rr, kk, cc, e = regress(lo, hi, &gg, &rr, &kk, &cc);
    if (e <= best && gg > 0) { best = e; bd = d; bl = l; }
  }
  model_x(S, bd, bl); regress(lo, hi, &g, &r, &k, &c);
  S->delay = (float)bd; S->lag = (float)bl; S->gain = (float)g; S->kick = (float)r; S->spring = (float)k; S->drag = (float)c;
  S->r2 = sst > 0 ? (float)(1 - best / sst) : 0;
  const char *why = S->r2 < 0.6f ? "the fit is poor (a bigger step, or a steadier rope)" : bl < 0.0055 || bl > 0.29 ? "the spin-up is at the edge of what it tries" : bd > 0.079 ? "the delay is at the edge of what it tries" : 0;
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
