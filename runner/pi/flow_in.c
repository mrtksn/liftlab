/* The downward camera's optical flow in dfb_pi: see flow_in.h. */
#define _DEFAULT_SOURCE
#include "flow_in.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void flow_in_init(flow_in *FI) { memset(FI, 0, sizeof *FI); }

/* the gyro at time x, between the samples around it (0 if the history doesn't reach it) */
static int gyro_at(const flow_in *FI, double x, float w[3]) {
  for (int k = 0; k + 1 < FI->gn; k++) {
    int b = (FI->gi - 1 - k + FLOW_GYRO_N) % FLOW_GYRO_N, a = (b - 1 + FLOW_GYRO_N) % FLOW_GYRO_N;
    if (FI->gt[a] <= x && x <= FI->gt[b]) {
      double s = FI->gt[b] > FI->gt[a] ? (x - FI->gt[a]) / (FI->gt[b] - FI->gt[a]) : 0;
      for (int i = 0; i < 3; i++) w[i] = (float)(FI->gw[a][i] + s * (FI->gw[b][i] - FI->gw[a][i]));
      return 1;
    }
  }
  return 0;
}

/* a reading against the gyro at each delay: the gyro averaged over the frame interval it covers */
static void accumulate(flow_in *FI, const flow_reading *r) {
  float wm[FLOW_LAGS][2];
  for (int l = 0; l < FLOW_LAGS; l++) {
    double lag = (l - FLOW_LAGS / 2) * FLOW_LAG_STEP; float s[2] = { 0, 0 };
    for (int j = 0; j < 3; j++) { float w[3]; if (!gyro_at(FI, r->t + lag - r->dt * (1 + 2 * j) / 6.0, w)) return; s[0] += w[0] / 3; s[1] += w[1] / 3; }
    wm[l][0] = s[0]; wm[l][1] = s[1];
  }
  for (int l = 0; l < FLOW_LAGS; l++) {
    double *S = FI->S[l]; float wx = wm[l][0], wy = wm[l][1], mx = r->f[0], my = r->f[1];
    S[0] += wx * wx; S[1] += wx * wy; S[2] += wy * wy; S[3] += mx * wx; S[4] += mx * wy; S[5] += my * wx; S[6] += my * wy; S[7] += mx * mx + my * my; S[8] += 1;
  }
  float wl = hypotf(wm[FLOW_LAGS / 2][0], wm[FLOW_LAGS / 2][1]); if (wl > FI->cal_wmax) FI->cal_wmax = wl;
  FI->cal_n++;
}

int flow_in_reading(flow_in *FI, const char *s, double t) {
  flow_reading r; float dx, dy, tex, res, radpx = 0; int mean = 0, mir = 0; unsigned seq;
  if (strncmp(s, "flow ", 5)) return 0;
  int n = sscanf(s + 5, "%lf %f %f %f %f %f %f %f %f %d %f %f %f %u %f %d", &r.t, &r.dt, &r.f[0], &r.f[1], &r.q, &dx, &dy, &FI->exp_ms, &FI->gain, &mean, &tex, &res, &FI->proc, &seq, &radpx, &mir);
  if (n < 5) return 0;
  FI->last = r; FI->t_rx = t; FI->mean = (float)mean; if (radpx > 0) FI->rad_px = radpx;
  if (!FI->n++) FI->t_first = t;
  if (r.q > 0) FI->n_good++;
  if (FI->cal_on && r.q >= 0.3f && r.dt > 0 && r.dt < 0.1) {      /* (waits for the gyro after it: the latest delay tried) */
    if (FI->pn == FLOW_PEND_N) { memmove(FI->pend, FI->pend + 1, sizeof FI->pend[0] * (FLOW_PEND_N - 1)); FI->pn--; }
    FI->pend[FI->pn++] = r;
  }
  return 1;
}

static void gyro_add(flow_in *FI, double t, const float w[3]);
void flow_in_gyro(flow_in *FI, double t, const float w[3]) { FI->t_nav = t; gyro_add(FI, t, w); }
void flow_in_gyro_telem(flow_in *FI, double t, const float w_deg[3]) {
  if (FI->t_nav > 0 && t - FI->t_nav < 0.2) return;
  float w[3] = { w_deg[0] / 57.29578f, w_deg[1] / 57.29578f, w_deg[2] / 57.29578f }; gyro_add(FI, t, w);
}
static void gyro_add(flow_in *FI, double t, const float w[3]) {
  FI->gt[FI->gi] = t; memcpy(FI->gw[FI->gi], w, sizeof FI->gw[0]); FI->gi = (FI->gi + 1) % FLOW_GYRO_N; if (FI->gn < FLOW_GYRO_N) FI->gn++;
  int k = 0;
  for (int i = 0; i < FI->pn; i++) {
    const flow_reading *r = &FI->pend[i];
    if (r->t + (FLOW_LAGS / 2) * FLOW_LAG_STEP + 0.01 < t) accumulate(FI, r);   /* the gyro covers it at every delay now */
    else FI->pend[k++] = *r;
  }
  FI->pn = k;
}

int flow_in_latest(const flow_in *FI, double t, double max_age, flow_reading *r) {
  if (!FI->n || t - FI->t_rx > max_age) return 0;
  *r = FI->last; return 1;
}

static int cal_said(const flow_in *FI, char *reply, int rn) {
  int best = -1; double bestr2 = -1e9, A[2][2] = { { 0 } };
  for (int l = 0; l < FLOW_LAGS; l++) {
    const double *S = FI->S[l]; double det = S[0] * S[2] - S[1] * S[1];
    if (S[8] < 20 || det <= 0 || S[7] <= 0) continue;
    double a[2][2] = { { (S[3] * S[2] - S[4] * S[1]) / det, (S[4] * S[0] - S[3] * S[1]) / det },
                       { (S[5] * S[2] - S[6] * S[1]) / det, (S[6] * S[0] - S[5] * S[1]) / det } };
    double res = S[7] - (a[0][0] * S[3] + a[0][1] * S[4] + a[1][0] * S[5] + a[1][1] * S[6]), r2 = 1 - res / S[7];
    if (r2 > bestr2) { bestr2 = r2; best = l; memcpy(A, a, sizeof A); }
  }
  int k = snprintf(reply, (size_t)rn, "flowcal: %s; %ld readings with the gyro, fastest turn %.1f rad/s", FI->cal_on ? "collecting (rock it: roll and pitch, the camera over the same spot)" : "stopped", FI->cal_n, (double)FI->cal_wmax);
  if (best < 0 || k >= rn) return 1;
  const double *S = FI->S[best]; double n = S[8];
  double lx = S[0] / n, ly = S[2] / n;                               /* how much it turned about each axis (mean square) */
  double det = A[0][0] * A[1][1] - A[0][1] * A[1][0], s = sqrt(fabs(det)); int mirrored = det < 0;
  double B[2][2] = { { (mirrored ? -1 : 1) * A[0][0] / s, (mirrored ? -1 : 1) * A[0][1] / s }, { A[1][0] / s, A[1][1] / s } };
  /* B ≈ J·Rz(−ψ) = [[−sin ψ, cos ψ], [−cos ψ, −sin ψ]]: the camera turned ψ about its axis against the body */
  double sp = -(B[0][0] + B[1][1]) / 2, cp = (B[0][1] - B[1][0]) / 2, psi = floor(atan2(sp, cp) * 57.29578 + 0.5);   /* (whole degrees, no −0) */
  double skew = (fabs(B[0][0] - B[1][1]) + fabs(B[0][1] + B[1][0])) / 2;   /* 0 for a pure turn of the picture */
  k += snprintf(reply + k, (size_t)(rn - k), "; best fit with the gyro taken %.0f ms %s: R² %.2f; the picture moves %.2f× the turn%s, turned %+.0f° about the camera's axis against the body, skew %.2f",
                fabs((best - FLOW_LAGS / 2) * FLOW_LAG_STEP * 1e3), best >= FLOW_LAGS / 2 ? "later" : "earlier", bestr2, s, mirrored ? ", MIRRORED" : "", psi, skew);
  if (k >= rn) return 1;
  if (sqrt(lx) < 0.3 || sqrt(ly) < 0.3 || n < 150) k += snprintf(reply + k, (size_t)(rn - k), "; not enough yet: rock it more about both axes (%.2f and %.2f rad/s rms, want 0.3+; %.0f readings, want 150+)", sqrt(lx), sqrt(ly), n);
  else if (bestr2 < 0.8) k += snprintf(reply + k, (size_t)(rn - k), "; a poor fit: keep the camera over one spot as you rock it, over a textured floor in good light");
  else if (FI->rad_px > 0) k += snprintf(reply + k, (size_t)(rn - k), "; for dfb_flow: --rad-px %.5f%s, and the design's flow camera turned %+.0f° about its axis", FI->rad_px / s, mirrored ? " and toggle --mirror" : "", psi);
  return 1;
}

int flow_in_command(flow_in *FI, const char *s, double t, char *reply, int rn) {
  if (!strcmp(s, "flow")) {
    if (!FI->n) { snprintf(reply, (size_t)rn, "flow: no readings from the camera (is dfb_flow running with --send 127.0.0.1:<this program's port>?)"); return 1; }
    double el = t - FI->t_first; const flow_reading *r = &FI->last;
    snprintf(reply, (size_t)rn, "flow: %ld readings (%.0f a second, %.0f%% good), the last %.2f s ago: %+.3f %+.3f rad/s, quality %.2f; exposure %.1f ms ×%.0f, brightness %.0f; %.1f ms a frame",
             FI->n, el > 0 ? FI->n / el : 0, 100.0 * FI->n_good / FI->n, t - FI->t_rx, (double)r->f[0], (double)r->f[1], (double)r->q, (double)FI->exp_ms, (double)FI->gain, (double)FI->mean, (double)FI->proc);
    return 1;
  }
  if (strncmp(s, "flowcal", 7) || (s[7] && s[7] != ' ')) return 0;
  const char *a = s + 7; while (*a == ' ') a++;
  if (!strcmp(a, "start")) { memset(FI->S, 0, sizeof FI->S); FI->cal_n = 0; FI->cal_wmax = 0; FI->pn = 0; FI->cal_on = 1;
    snprintf(reply, (size_t)rn, "flowcal: collecting; rock it by hand, motors off: roll and pitch, a few times each way, the camera staying over one spot of a textured floor; then flowcal"); return 1; }
  if (!strcmp(a, "stop")) FI->cal_on = 0;
  else if (*a) { snprintf(reply, (size_t)rn, "flowcal start | stop, or flowcal (the fit so far)"); return 1; }
  return cal_said(FI, reply, rn);
}
