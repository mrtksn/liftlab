/* The rope spin test (spin_core.c) against a drone on a rope made up here: each motor's drag torque follows its
 * throttle after a delay and a first-order spin-up, the rope twists back and the air drags, the gyro is noisy, the
 * flight core averages 5 ms windows for its telemetry, and the link delays both ways by a varying amount. The fit
 * must give the delay and the spin-up back; a test that's armed, that spins too fast, or that gets no telemetry
 * stops; and the sequence ends with the rope where it started.
 *   cc -O2 -I. -I.. -o test_spin test_spin.c spin_core.c fc_core.c ../rn_host.c ../rn.c ../rn_builtin.c -lm && ./test_spin */
#include "spin_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } else { printf("ok: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
static unsigned long long rng = 5;   /* its own generator: the same noise on every C library */
static double rnd(void) { rng = rng * 6364136223846793005ULL + 1442695040888963407ULL; return (double)(rng >> 11) / 9007199254740992.0 - 0.5; }

typedef struct { double delay, lag, gain, spring, drag, noise; int armed_at_ms; float wz_limit_kick; double kick; } plant;   /* kick: the reaction torque's jump against its steady change (0: none) */
/* Run one test on the plant: base, step; the result in S. Returns the yaw angle at the end [rad]. */
static double run(spin_state *S, const plant *P, float base, float step, int *stopped) {
  const float yaw[4] = { 1, -1, 1, -1 };
  spin_init(S);
  if (spin_start(S, 4, yaw, base, step, 0.25f, 0)) { printf("start: %s\n", S->msg); return 0; }
  enum { Q = 400 };
  double hist_u[4][4096] = { { 0 } };   /* the throttles applied at the flight core, by ms (for the delay) */
  float sent[4] = { 0 }, applied[4] = { 0 }; double lag_x[4] = { 0 };
  double th = 0, w = 0; int state = FC_TESTING;
  /* the link: commands and telemetry each arrive late by 3–25 ms */
  struct { double at; float v[4]; } cq[Q]; int cqn = 0;
  struct { double at; float lt[40]; } tq[Q]; int tqn = 0;
  double uavg[4] = { 0 }, wavg = 0, last_c = 0, last_t = 0; int navg = 0;
  for (int ms = 0; ms < 16000; ms++) {
    double t = ms * 0.001;
    float out[FC_MAX_MOTORS]; int k = spin_tick(S, t, out);
    if (k && cqn < Q) { double at = t + 0.003 + 0.022 * (rnd() + 0.5); if (at < last_c) at = last_c; last_c = at; cq[cqn].at = at; memcpy(cq[cqn].v, out, sizeof cq[cqn].v); cqn++; }
    for (int i = 0; i < cqn; i++) if (cq[i].at <= t) { memcpy(applied, cq[i].v, sizeof applied); memmove(&cq[i], &cq[i + 1], sizeof cq[0] * (size_t)(cqn - i - 1)); cqn--; i--; }
    if (!k && S->phase != SPIN_SETTLE) memset(applied, 0, sizeof applied);
    if (P->armed_at_ms && ms >= P->armed_at_ms) state = FC_ARMED;
    /* the plant, at 1 ms */
    double Du = 0, r = P->kick;
    for (int i = 0; i < 4; i++) { hist_u[i][ms % 4096] = applied[i]; int dm = (int)lround(P->delay * 1000); double u = ms >= dm ? hist_u[i][(ms - dm) % 4096] : 0; lag_x[i] += (u - lag_x[i]) * (0.001 / P->lag); Du += yaw[i] * (u - base); }
    double D = 0; for (int i = 0; i < 4; i++) D += yaw[i] * (lag_x[i] - base); D /= 4; Du /= 4;
    D = r * Du + (1 - r) * D;                         /* the reaction torque: a jump with the throttle, then the lagged drag */
    double acc = P->gain * D - P->spring * th - P->drag * w + (P->wz_limit_kick && ms > 3000 ? P->wz_limit_kick : 0);
    w += acc * 0.001; th += w * 0.001;
    for (int i = 0; i < 4; i++) uavg[i] += applied[i];
    wavg += w + P->noise * rnd() * 2; navg++;
    (void)sent;
    if (navg == 5) {                                   /* the flight core's LTEL */
      if (tqn < Q) {
        float *lt = tq[tqn].lt; memset(lt, 0, sizeof tq[tqn].lt);
        lt[0] = (float)fmod(t, FC_LTEL_WRAP); lt[1] = (float)state; lt[12] = (float)(wavg / navg); lt[10] = (float)(0.01 * rnd()); lt[11] = (float)(0.01 * rnd());
        lt[17] = 4; lt[18] = 0; for (int i = 0; i < 4; i++) lt[19 + i] = (float)(uavg[i] / navg);
        { double at = t + 0.003 + 0.022 * (rnd() + 0.5); if (at < last_t) at = last_t; last_t = at; tq[tqn].at = at; } tqn++;
      }
      memset(uavg, 0, sizeof uavg); wavg = 0; navg = 0;
    }
    for (int i = 0; i < tqn; i++) if (tq[i].at <= t) { spin_ltel(S, tq[i].lt, 27, t); memmove(&tq[i], &tq[i + 1], sizeof tq[0] * (size_t)(tqn - i - 1)); tqn--; i--; }
    if (S->phase == SPIN_DONE || S->phase == SPIN_FAILED) break;
  }
  *stopped = S->phase == SPIN_FAILED;
  return th;
}

int main(void) {
  spin_state S; int stopped; char said[256];
  /* toy motors: quick and slow, a little and a lot of delay */
  const plant cases[] = {
    { 0.003, 0.028, 80, 0.4, 0.3, 0.02, 0, 0, 0 },
    { 0.012, 0.060, 60, 0.4, 0.3, 0.03, 0, 0, 0 },
    { 0.025, 0.085, 50, 1.5, 0.5, 0.05, 0, 0, 0 },
    { 0.008, 0.140, 40, 0.2, 0.1, 0.03, 0, 0, 0 },
    { 0.010, 0.050, 50, 0.4, 0.3, 0.03, 0, 0, 3.0 },   /* a big kick: the motor's current jumps at once */
    { 0.020, 0.080, 40, 0.4, 0.3, 0.03, 0, 0, 6.0 },
  };
  for (unsigned c = 0; c < sizeof cases / sizeof cases[0]; c++) {
    const plant *P = &cases[c];
    double th = run(&S, P, 0.3f, 0.05f, &stopped);
    spin_said(&S, said, sizeof said);
    printf("  %s\n", said);
    CHECK(!stopped && S.ok, "delay %.0f ms, spin-up %.0f ms: fitted (%s)", P->delay * 1000, P->lag * 1000, S.msg);
    CHECK(fabs(S.delay - P->delay) <= 0.005, "delay %.1f ms against %.1f", S.delay * 1000, P->delay * 1000);
    CHECK(fabs(S.lag / P->lag - 1) <= 0.15, "spin-up %.1f ms against %.1f", S.lag * 1000, P->lag * 1000);
    /* (a big kick: the jump g·r is what the data pins down; how it splits between g and r much less so) */
    if (P->kick > 1) CHECK(fabs(S.gain * S.kick / (P->gain * P->kick) - 1) <= 0.15 && S.kick > 1, "instant yaw torque %.1f against %.1f (kick %.1f, %.1f)", S.gain * S.kick, P->gain * P->kick, S.kick, P->kick);
    else { CHECK(fabs(S.gain / P->gain - 1) <= 0.15, "yaw gain %.1f against %.1f", S.gain, P->gain);
      CHECK(fabs(S.kick - P->kick) <= 0.3, "kick %.2f against %.2f", S.kick, P->kick); }
    CHECK(fabs(th) < 0.6, "the rope ends near where it started (%.2f rad)", th);
    CHECK(strstr(said, "result ") != 0, "the result line has its numbers");
  }
  /* the airframe's spin directions the other way round (or the IMU turned): it says so */
  { plant P = { 0.01, 0.05, -60, 0.4, 0.3, 0.02, 0, 0, 0 }; run(&S, &P, 0.3f, 0.05f, &stopped);
    CHECK(!S.ok && S.gain < 0 && strstr(S.msg, "the other way") && fabs(S.lag - 0.05) < 0.008, "turning the other way: said (%s)", S.msg); }
  /* stops: armed partway; turning too fast; no telemetry at all */
  { plant P = { 0.01, 0.05, 60, 0.4, 0.3, 0.02, 2500, 0, 0 }; run(&S, &P, 0.3f, 0.05f, &stopped); CHECK(stopped && strstr(S.msg, "armed"), "armed: stopped (%s)", S.msg); }
  { plant P = { 0.01, 0.05, 900, 0.0, 0.0, 0.02, 0, 0, 0 }; run(&S, &P, 0.3f, 0.2f, &stopped); CHECK(stopped && strstr(S.msg, "too fast"), "too fast: stopped (%s)", S.msg); }
  { spin_init(&S); const float yaw[4] = { 1, -1, 1, -1 }; spin_start(&S, 4, yaw, 0.3f, 0.05f, 0.25f, 0); float o[12]; int k = 1; for (int ms = 0; ms < 1000 && k; ms++) k = spin_tick(&S, ms * 0.001, o);
    CHECK(S.phase == SPIN_FAILED && strstr(S.msg, "telemetry"), "no telemetry: stopped (%s)", S.msg); }
  /* refusals */
  { spin_init(&S); const float same[4] = { 1, 1, 1, 1 }; CHECK(spin_start(&S, 4, same, 0.3f, 0.05f, 0.25f, 0) && strstr(S.msg, "both ways"), "all one way: refused"); }
  { spin_init(&S); const float yaw[4] = { 1, -1, 1, -1 }; CHECK(spin_start(&S, 4, yaw, 0.3f, 0.5f, 0.25f, 0), "a step too big: refused"); }
  /* the lift test: a drone on a rope that hovers at uh, its thrust ∝ u^1.6 after a 40 ms spin-up; the rope holds it
   * at z = 0 (it can't go lower); the barometer 3 cm noisy */
  for (int c = 0; c < 3; c++) {
    const double uh = c == 0 ? 0.55 : c == 1 ? 0.45 : 0.68, m = 0.29, g = 9.81, lagT = 0.04;
    lift_state L; lift_init(&L);
    CHECK(!lift_start(&L, 4, 0.3f, 0.95f, 0.04f, 0.04f, 0), "lift test starts");
    double T = 0, z = 0, vz = 0, fsum = 0, asum = 0, usum = 0; int na = 0; float applied = 0;
    struct { double at; float u; } q[64]; int qn = 0;
    for (int ms = 0; ms < 30000 && (L.phase != LIFT_DONE && L.phase != LIFT_FAILED); ms++) {
      double t = ms * 0.001; float o[FC_MAX_MOTORS]; int k = lift_tick(&L, t, o);
      if (k && qn < 64) { q[qn].at = t + 0.01; q[qn].u = o[0]; qn++; }
      while (qn && q[0].at <= t) { applied = q[0].u; memmove(q, q + 1, sizeof q[0] * (size_t)--qn); }
      if (!k) applied = 0;
      double Tt = m * g * pow(applied / uh, 1.6); T += (Tt - T) * 0.001 / lagT;
      double a = T / m - g; if (z <= 0 && a < 0) { a = 0; vz = 0; z = 0; }
      vz += a * 0.001; z += vz * 0.001; if (z < 0) { z = 0; vz = 0; }
      double f = a + g; fsum += f; asum += z; usum += applied; na++;
      if (na == 5) {
        /* (the motors shake it: the accelerometer's 5 ms averages scatter, more the faster they spin) */
        const double shake = 2.0 * applied / 0.5;
        float lt[30] = { 0 }; lt[0] = (float)t; lt[1] = FC_TESTING; lt[9] = (float)(fsum / na * 1.003 + 0.02 * rnd() + shake * rnd()); lt[13] = 3.9f;
        lt[7] = (float)(shake * 2 * rnd()); lt[8] = (float)(shake * 2 * rnd());
        lt[14] = (float)(asum / na + 0.03 * rnd()); lt[16] = 1; lt[17] = 4; for (int i = 0; i < 4; i++) lt[19 + i] = (float)(usum / na);
        lift_ltel(&L, lt, 27, t + 0.01); fsum = asum = usum = 0; na = 0;
      }
    }
    char said[300]; lift_said(&L, said, sizeof said); printf("  %s\n", said);
    CHECK(L.ok && fabs(L.lift_u - uh) < 0.03, "hovers at %.2f: lifted off at %.3f", uh, L.lift_u);
    int good = L.np >= 2; for (int i = 0; i < L.np; i++) { double want = pow(L.pu[i] / uh, 1.6); if (fabs(L.pf[i] / want - 1) > 0.04) good = 0; }
    CHECK(good, "each pulse reads its thrust over the weight within 4%% (%d pulses)", L.np);
    CHECK(L.climb < 0.4f, "it rose at most %.2f m", (double)L.climb);
  }
  { lift_state L; lift_init(&L); CHECK(lift_start(&L, 4, 0.3f, 0.2f, 0.04f, 0.04f, 0), "lift: a top below the base is refused"); }
  printf(fails ? "rope spin test: %d FAILED\n" : "rope spin test: ok\n", fails);
  return fails ? 1 : 0;
}
