#include "autotune_core.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define PI 3.14159265358979323846
static at_complex cx(double r, double i) { return (at_complex){r, i}; }
static at_complex mul(at_complex a, at_complex b) {
  return cx(a.r * b.r - a.i * b.i, a.r * b.i + a.i * b.r);
}
static at_complex divc(at_complex a, at_complex b) {
  double d = b.r * b.r + b.i * b.i;
  return cx((a.r * b.r + a.i * b.i) / d, (a.i * b.r - a.r * b.i) / d);
}
static double mag(at_complex a) { return hypot(a.r, a.i); }
static at_complex plant(double hz, const at_model *m) {
  double w = 2 * PI * hz, a = w * m->delay;
  return mul(divc(cx(-m->gain * cos(a) / (w * w), m->gain * sin(a) / (w * w)),
                  cx(1, w * m->lag)),
             cx(1, w * m->lead));
}
static at_complex response(double hz, const double *g, const at_model *m) {
  double w = 2 * PI * hz;
  at_complex G = plant(hz, m), L = mul(G, cx(g[0], g[1] * w - g[2] / w));
  return divc(mul(G, cx(g[0], -g[2] / w)), cx(1 + L.r, L.i));
}
at_bin autotune_bin(const double (*s)[3], int n, double hz, double amp,
                    int column, int rate) {
  at_bin b = {0};
  b.hz = hz;
  b.n = n;
  double M[4][5] = {{0}}, w = 2 * PI * hz, t0 = n ? s[0][0] : 0;
  for (int k = 0; k < n; k++) {
    double x[4] = {sin(w * s[k][0]), cos(w * s[k][0]), 1, s[k][0] - t0};
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++)
        M[i][j] += x[i] * x[j];
      M[i][4] += x[i] * s[k][column];
    }
  }
  for (int i = 0; i < 4; i++) {
    int p = i;
    for (int k = i + 1; k < 4; k++)
      if (fabs(M[k][i]) > fabs(M[p][i]))
        p = k;
    for (int j = 0; j < 5; j++) {
      double v = M[i][j];
      M[i][j] = M[p][j];
      M[p][j] = v;
    }
    double d = M[i][i];
    if (fabs(d) < 1e-10)
      return b;
    for (int j = i; j < 5; j++)
      M[i][j] /= d;
    for (int k = 0; k < 4; k++)
      if (k != i) {
        double v = M[k][i];
        for (int j = i; j < 5; j++)
          M[k][j] -= v * M[i][j];
      }
  }
  double err = 0;
  for (int k = 0; k < n; k++) {
    double e = s[k][column] - M[0][4] * sin(w * s[k][0]) -
               M[1][4] * cos(w * s[k][0]) - M[2][4] - M[3][4] * (s[k][0] - t0);
    err += e * e;
  }
  double signal = hypot(M[0][4], M[1][4]), noise = err / fmax(1, n);
  b.quality = signal > 0 ? signal * signal / (signal * signal + 2 * noise) : 0;
  at_complex H = cx(M[0][4] / amp, M[1][4] / amp);
  b.T = rate ? cx(H.i / w, -H.r / w) : H;
  return b;
}
/* Attitude fits commanded acceleration -> angle. Position fits a measured
 * closed loop with verified attitude. */
int autotune_fit(const at_bin *b, int n, at_model *out, const double *inner_g,
                 const at_model *inner, const double *gains) {
  double best = 1e30;
  int usable = 0;
  for (int i = 0; i < n; i++)
    if (b[i].quality > .65 && b[i].n >= 60 && mag(b[i].T) > .015)
      usable++;
  if (usable < 3)
    return 0;
  for (int l = 0; l <= 16; l++)
    for (int d = 0; d <= 10; d++)
      for (int g = 0; g <= 16; g++)
        for (int lead = 0; lead < (inner ? 1 : 6); lead++) {
          const double leads[] = {0, .02, .04, .08, .12, .2};
          at_model m = {.6 + .05 * g, .01 * l, .004 * d, leads[lead], 0};
          double err = 0;
          for (int i = 0; i < n; i++)
            if (b[i].quality > .65 && b[i].n >= 60 && mag(b[i].T) > .015) {
              at_complex t = plant(b[i].hz, &m), want = b[i].plant;
              if (inner) {
                at_complex G = mul(t, response(b[i].hz, inner_g, inner));
                double w = 2 * PI * b[i].hz;
                at_complex L =
                    mul(G, cx(gains[0], gains[1] * w - gains[2] / w));
                t = divc(mul(G, cx(gains[0], -gains[2] / w)), cx(1 + L.r, L.i));
                want = b[i].T;
              }
              at_complex diff = cx(t.r - want.r, t.i - want.i);
              err += b[i].quality * mag(diff) * mag(diff) /
                     fmax(inner ? .04 : 1e-8, mag(want) * mag(want));
            }
          err /= usable;
          if (err < best) {
            best = err;
            m.error = err;
            *out = m;
          }
        }
  return best < .15;
}
static int margins(const double *g, const at_model *m, const double *ig,
                   const at_model *im) {
  double last = 1e30, margin = -1e30, peak = 0, sensitivity = 0;
  for (int k = 0; k < 220; k++) {
    double hz = .02 * pow(2500, (double)k / 219), w = 2 * PI * hz;
    at_complex G = plant(hz, m);
    if (im)
      G = mul(G, response(hz, ig, im));
    at_complex f = cx(g[0], g[1] * w - g[2] / w), L = mul(G, f);
    double a = mag(L);
    if (last >= 1 && a < 1) {
      double phase = -PI - atan(w * m->lag) - w * m->delay + atan(w * m->lead) +
                     atan2(f.i, f.r);
      if (im) {
        at_complex r = response(hz, ig, im);
        phase += atan2(r.i, r.r);
      }
      margin = (PI + phase) * 180 / PI;
    }
    last = a;
    at_complex den = cx(1 + L.r, L.i);
    peak = fmax(peak, mag(divc(mul(G, cx(g[0], -g[2] / w)), den)));
    sensitivity = fmax(sensitivity, 1 / mag(den));
  }
  return g[2] < g[0] * g[1] * .35 && margin >= 45 && peak <= 1.25 &&
         sensitivity <= 2;
}
int autotune_recommend(const float *before, int pos, const at_model *models,
                       float *out, const at_model *inner) {
  memcpy(out, before, 12 * sizeof(float));
  int ia = before[0] <= before[1] ? 0 : 1;
  double ig[3] = {before[ia], before[3 + ia], before[6 + ia]};
  for (int axis = 0; axis < (pos ? 1 : 3); axis++) {
    int k = pos ? 9 : axis;
    double oldP = before[k], oldI = before[pos ? 11 : 6 + axis],
           integ = fmin(oldI / oldP, pos ? .3 : .8),
           ceiling = fmin(pos ? 1 : 4, sqrt(oldP) / (2 * PI) * 1.25);
    if (pos)
      ceiling = fmin(ceiling, sqrt(fmin(before[0], before[1])) / (2 * PI * 4));
    int found = 0;
    double best[3] = {0};
    for (double hz = pos ? .08 : .3; hz <= ceiling + 1e-8;
         hz += pos ? .01 : .025) {
      for (int z = 0; z < 4; z++) {
        double wn = 2 * PI * hz,
               g[3] = {wn * wn, 2 * (.8 + .1 * z) * wn, wn * wn * integ};
        if (margins(g, &models[axis], pos ? ig : 0, pos ? inner : 0)) {
          memcpy(best, g, sizeof best);
          found = 1;
          break;
        }
      }
    }
    if (!found)
      return 0;
    for (int i = 0; i < 3; i++)
      out[pos ? 9 + i : axis + 3 * i] = (float)best[i];
  }
  return 1;
}
static uint32_t model_crc(autotune_state *A) {
  uint32_t parts[] = {
      rn_crc32((const uint8_t *)A->L->flyB, sizeof A->L->flyB),
      rn_crc32((const uint8_t *)A->L->m_curve, sizeof A->L->m_curve),
      rn_crc32((const uint8_t *)A->L->m_eff, sizeof A->L->m_eff)};
  return rn_crc32((const uint8_t *)parts, sizeof parts);
}
static void message(autotune_state *A, const char *s) {
  snprintf(A->message, sizeof A->message, "%s", s);
  A->events++;
}
int autotune_busy(const autotune_state *A) {
  return A->phase == AT_MEASURE || A->phase == AT_STAGE ||
         A->phase == AT_VERIFY || A->phase == AT_COMMIT ||
         A->phase == AT_ANALYZE;
}
static int tune_send(autotune_state *A, int action) {
  float p[PID_FRAME] = {PID_VERSION, (float)action, (float)A->session};
  for (int i = 0; i < 9; i++)
    p[i + 3] = A->candidate[i];
  A->send(A->ctx, RN_LINK_TUNE, p, PID_FRAME);
  if (A->loop && action != 2) {
    for (int i = 0; i < 9; i++)
      p[3 + i] = A->candidate[9 + i / 3];
    return nav_tune(A->N, p, PID_FRAME);
  }
  return 0;
}
static void exc(autotune_state *A, float angle) {
  float p[8 + FC_MAX_MOTORS + FC_MAX_JOINTS] = {0};
  int nm = A->L->FA.A.n_motors, nj = A->L->FA.A.n_joints;
  p[0] = angle ? 3 : 0;
  p[4] = (float)nm;
  p[5] = (float)nj;
  p[6 + nm + nj] = (float)(A->loop ? 0 : A->index / 4);
  p[7 + nm + nj] = angle;
  A->send(A->ctx, RN_LINK_EXC, p, 8 + nm + nj);
  if (!angle)
    A->N->test_left = 0;
}
void autotune_stop(autotune_state *A, const char *why) {
  if (!autotune_busy(A) && A->phase != AT_REVIEW)
    return;
  exc(A, 0);
  if (A->phase == AT_STAGE || A->phase == AT_VERIFY || A->phase == AT_COMMIT)
    tune_send(A, 0);
  learn_command(A->L, A->keep ? LN_CMD_KEEP_ON : LN_CMD_KEEP_OFF);
  int committed = A->phase == AT_COMMIT;
  A->phase = AT_STOPPED;
  message(
      A,
      committed
          ? "stopped during acceptance: check autotune status for current gains"
          : why);
}
void autotune_init(autotune_state *A, nav_state *N, learn_state *L,
                   super_state *S,
                   void (*send)(void *, uint8_t, const float *, int),
                   void *ctx) {
  memset(A, 0, sizeof *A);
  A->N = N;
  A->L = L;
  A->S = S;
  A->send = send;
  A->ctx = ctx;
  pid_defaults(A->current, 0);
  A->current[9] = 4;
  A->current[10] = 3.6f;
  A->current[11] = 1;
  message(A, "idle; calibrate while hovering, then autotune attitude");
}
void autotune_guard(autotune_state *A, double now, int arm, const nav_sp *sp,
                    const nav_out *o, double nav_at, int blocked) {
  A->now = now;
  A->ready = A->have_status && A->status[15] == 1 && arm && sp->fly && o->fly &&
             o->ready && now - nav_at < .1 && !A->N->sup_mode &&
             !A->N->rc_rth && A->N->H->act == 0 &&
             A->N->H->phase == RN_PH_FLYING && A->N->H->pending == 0 &&
             !A->S->mode && !A->L->cal && !A->L->thr && A->L->adapt != 2 &&
             A->L->have_fit && A->L->accepted && !A->L->external_load &&
             !blocked;
  for (int i = 0; i < 3; i++)
    if (fabsf(sp->vref[i]) > .01f)
      A->ready = 0;
  if (!autotune_busy(A) && A->phase != AT_REVIEW) {
    A->target = *sp;
    return;
  }
  if (!A->ready || model_crc(A) != A->model_crc ||
      A->L->use_learned != A->model_use ||
      fabsf(sp->heading - A->target.heading) > .001f) {
    autotune_stop(A, "stopped: flight/model changed; previous gains restored");
    return;
  }
  for (int i = 0; i < 3; i++)
    if (fabsf(sp->target[i] - A->target.target[i]) > .001f) {
      autotune_stop(A,
                    "stopped: pilot target changed; previous gains restored");
      return;
    }
  if (now - A->last_packet > .04) {
    autotune_stop(
        A, "stopped: tuning telemetry interrupted; previous gains restored");
    return;
  }
  if (autotune_busy(A) && hypot(hypot(o->v[0], o->v[1]), o->v[2]) > 1.5) {
    autotune_stop(A, "stopped: excess velocity; previous gains restored");
    return;
  }
  if (autotune_busy(A) &&
      hypot(hypot(o->p[0] - sp->target[0], o->p[1] - sp->target[1]),
            o->p[2] - sp->target[2]) > .75) {
    autotune_stop(A, "stopped: position excursion; previous gains restored");
    return;
  }
}
static void plan(autotune_state *A) {
  A->index = A->ns = 0;
  A->start = A->stamp;
  A->settle = 0;
  A->max_motor = 0;
}
static const double freqs[2][4] = {{.8, 1.6, 3.2, 6.4}, {.18, .32, .55, 0}};
static int target_same(const float *a, const float *b) {
  for (int i = 0; i < 9; i++)
    if (fabsf(a[i] - b[i]) > 1e-4f)
      return 0;
  return 1;
}
void autotune_sample(autotune_state *A, const float *p, int n, double now) {
  if (n < 19 + AT_TAIL || n > AT_LTEL_MAX)
    return;
  for (int i = 0; i < n; i++)
    if (!isfinite(p[i])) {
      autotune_stop(A, "stopped: non-finite telemetry");
      return;
    }
  if (p[1] < 0 || p[1] > FC_CRASHED || (int)p[1] != p[1] || p[2] < 0 ||
      p[2] > 31 || (int)p[2] != p[2] || p[17] != (float)A->L->FA.A.n_motors ||
      p[18] != (float)A->L->FA.A.n_joints) {
    autotune_stop(A, "stopped: bad telemetry layout");
    return;
  }
  int base = n - AT_TAIL;
  const float *alpha = p + base, *st = alpha + 4;
  if (st[0] != PID_VERSION || !(st[1] >= 0 && st[1] <= 16777215) ||
      (uint32_t)st[1] != st[1] || !(st[2] >= 0 && st[2] <= 2) ||
      (int)st[2] != st[2] || fabsf(alpha[0] - p[0]) > .001f ||
      (st[15] != 0 && st[15] != 1) || st[3] != 1) {
    A->have_status = 0;
    autotune_stop(A, "stopped: unsupported controller or mismatched samples");
    return;
  }
  if (st[13] < 0 || st[13] > 65535 || st[14] < 0 || st[14] > 65535 ||
      (uint32_t)st[13] != st[13] || (uint32_t)st[14] != st[14] ||
      ((uint32_t)st[13] | ((uint32_t)st[14] << 16)) != A->L->FA.airframe_crc) {
    A->have_status = 0;
    autotune_stop(A, "stopped: Pi and flight controller airframes differ");
    return;
  }
  if (!pid_valid(st + 4, 0)) {
    A->have_status = 0;
    return;
  }
  memcpy(A->status, st, PID_STATUS * sizeof(float));
  A->have_status = 1;
  A->last_packet = now;
  double stamp = fc_ltel_unwrap(A->stamp, A->stamp > 0, p[0]);
  if (autotune_busy(A) && (stamp <= A->stamp || stamp - A->stamp > .04)) {
    autotune_stop(A, "stopped: tuning sample clock interrupted");
    return;
  }
  A->stamp = stamp;
  const nav_out *stable = &A->N->last;
  A->settled = hypot(hypot(stable->v[0], stable->v[1]), stable->v[2]) < .12 &&
               hypot(hypot(stable->p[0] - A->target.target[0],
                           stable->p[1] - A->target.target[1]),
                     stable->p[2] - A->target.target[2]) < .2 &&
               hypot(hypot(p[10], p[11]), p[12]) < .4;
  if (!autotune_busy(A) && A->phase != AT_REVIEW) {
    if (!target_same(st + 4, A->current))
      A->att_verified = 0;
    A->session = (uint32_t)st[1];
    memcpy(A->current, st + 4, 9 * sizeof(float));
    const float *pg = pid_gains(&A->N->tuning);
    for (int i = 0; i < 3; i++)
      A->current[9 + i] = pg[3 * i];
    return;
  }
  if (!autotune_busy(A)) {
    int same = target_same(st + 4, A->before);
    const float *ng = pid_gains(&A->N->tuning);
    for (int i = 0; i < 3; i++)
      if (fabsf(ng[3 * i] - A->before[9 + i]) > 1e-4f)
        same = 0;
    if (!same)
      autotune_stop(A, "stopped: gains changed while reviewing; measure again");
    return;
  }
  if (st[15] != 1 || p[1] != FC_ARMED || !((int)p[2] & 1) ||
      !!((int)p[2] & 4) != A->model_use ||
      1 - 2 * (p[4] * p[4] + p[5] * p[5]) < .94 ||
      hypot(hypot(p[10], p[11]), p[12]) > 3) {
    autotune_stop(A, "stopped: attitude/state limits; previous gains restored");
    return;
  }
  int nm = (int)p[17];
  if (nm != A->L->FA.A.n_motors || base < 19 + 2 * nm) {
    autotune_stop(A, "stopped: bad telemetry layout");
    return;
  }
  for (int i = 0; i < nm; i++) {
    A->max_motor = fmax(A->max_motor, p[19 + i]);
    if (p[19 + i] > .93) {
      autotune_stop(
          A, "stopped: insufficient motor headroom; previous gains restored");
      return;
    }
  }
  if (A->phase == AT_STAGE) {
    if (tune_send(A, 1)) {
      autotune_stop(A, "stopped: navigation refused trial gains");
      return;
    }
    if (A->stamp - A->stage_start > 2) {
      autotune_stop(A, "stopped: trial gains not acknowledged");
      return;
    }
    if (st[1] == A->session && st[2] == 1 &&
        target_same(st + 4, A->candidate)) {
      A->phase = AT_VERIFY;
      plan(A);
      message(A, "verifying trial gains; stop restores previous gains");
    }
    return;
  }
  if (A->phase == AT_COMMIT) {
    tune_send(A, 2);
    if (st[1] == A->session && st[2] == 2 &&
        target_same(st + 4, A->candidate)) {
      if (A->loop) {
        float f[PID_FRAME] = {PID_VERSION, 2, (float)A->session};
        for (int i = 0; i < 9; i++)
          f[3 + i] = A->candidate[9 + i / 3];
        if (nav_tune(A->N, f, PID_FRAME)) {
          autotune_stop(A, "stopped: navigation commit failed");
          return;
        }
      }
      memcpy(A->current, A->candidate, sizeof A->current);
      A->phase = AT_DONE;
      if (!A->loop) {
        A->att_verified = 1;
        A->verified_crc = A->model_crc;
        A->verified_use = A->model_use;
        memcpy(A->att_models, A->models, sizeof A->models);
      }
      learn_command(A->L, A->keep ? LN_CMD_KEEP_ON : LN_CMD_KEEP_OFF);
      message(A, "verified and accepted; autotune save persists gains for this "
                 "airframe");
    } else if (A->stamp - A->stage_start > .2) {
      autotune_stop(
          A, "stopped: commit acknowledgement missing; check autotune status");
    }
    return;
  }
  if (A->phase == AT_VERIFY) {
    if (st[1] != A->session || st[2] != 1 ||
        !target_same(st + 4, A->candidate)) {
      autotune_stop(A, "stopped: controller reverted trial gains");
      return;
    }
    if (tune_send(A, 1)) {
      autotune_stop(A, "stopped: navigation refused trial gains");
      return;
    }
  } else if (!target_same(st + 4, A->before)) {
    autotune_stop(A, "stopped: gains changed during measurement");
    return;
  }
  if (A->phase == AT_ANALYZE)
    return;
  if (A->settle) {
    const nav_out *o = &A->N->last;
    double v = hypot(hypot(o->v[0], o->v[1]), o->v[2]), err = 0;
    for (int i = 0; i < 3; i++)
      err += pow(o->p[i] - A->target.target[i], 2);
    if (v < .12 && sqrt(err) < .2 && hypot(hypot(p[10], p[11]), p[12]) < .4) {
      A->settle = 0;
      A->start = A->stamp;
    } else {
      if (A->stamp - A->settle > 20)
        autotune_stop(A, "stopped: aircraft did not settle");
      return;
    }
  }
  double hz = freqs[A->loop][A->loop ? A->index : A->index % 4],
         elapsed = A->stamp - A->start, warm = fmax(2, ceil(3 * hz)) / hz,
         duration = warm + fmax(4, ceil(2 * hz)) / hz,
         amp = A->loop    ? .18
               : hz < 1.2 ? .034907
                          : .05236;
  if (A->loop)
    nav_test_target(A->N, 0, (float)(amp * sin(2 * PI * hz * elapsed)));
  else
    exc(A, (float)(amp * sin(2 * PI * hz * elapsed)));
  if (elapsed > warm) {
    if (A->ns >= AT_SAMPLES) {
      autotune_stop(A, "stopped: sample capacity exceeded");
      return;
    }
    A->samples[A->ns][0] = elapsed;
    A->samples[A->ns][1] = A->loop ? A->N->last.v[0] : p[10 + A->index / 4];
    A->samples[A->ns++][2] = alpha[1 + (A->loop ? 0 : A->index / 4)];
  }
  if (elapsed < duration)
    return;
  exc(A, 0);
  at_bin b = autotune_bin(A->samples, A->ns, hz, amp, 1, 1);
  b.axis = A->loop ? 0 : A->index / 4;
  if (!A->loop) {
    at_bin cmd = autotune_bin(A->samples, A->ns, hz, amp, 2, 0);
    b.plant = divc(b.T, cmd.T);
    b.quality = fmin(b.quality, cmd.quality);
  } else {
    const float *g = A->phase == AT_VERIFY ? A->candidate : A->before;
    double w = 2 * PI * hz;
    at_complex C = cx(g[9], g[10] * w - g[11] / w), ref = cx(g[9], -g[11] / w);
    b.plant = divc(b.T, cx(ref.r - mul(b.T, C).r, ref.i - mul(b.T, C).i));
  }
  A->bins[A->index++] = b;
  A->ns = 0;
  A->start = A->stamp;
  if (A->index < (A->loop ? 3 : 12)) {
    A->settle = A->stamp;
    return;
  }
  if (A->phase == AT_MEASURE) {
    A->phase = AT_ANALYZE;
    message(A, "analysing response in background; flight control continues");
  } else {
    int ok = A->max_motor < .9;
    for (int i = 0; i < A->index; i++) {
      at_bin *q = &A->bins[i], *old = &A->baseline[i];
      ok &= q->quality > .65 && mag(q->T) <= fmax(1.25, mag(old->T) * 1.02);
      if (q->hz < (A->loop ? .4 : 1.2))
        ok &= mag(cx(q->T.r - 1, q->T.i)) <=
              fmax(1, mag(cx(old->T.r - 1, old->T.i)) * 1.1);
    }
    if (!ok) {
      autotune_stop(A, "verification failed; previous gains restored");
      return;
    }
    A->phase = AT_COMMIT;
    A->stage_start = A->stamp;
    tune_send(A, 2);
    message(A, "verification passed; awaiting controller acceptance");
  }
}
void autotune_analysis_input(const autotune_state *A, autotune_analysis *J) {
  memset(J, 0, sizeof *J);
  J->measurement = A->measurement;
  J->loop = A->loop;
  memcpy(J->before, A->before, sizeof J->before);
  memcpy(J->bins, A->bins, sizeof J->bins);
  memcpy(J->att_models, A->att_models, sizeof J->att_models);
}
void autotune_analyze(autotune_analysis *J) {
  int ia = J->before[0] <= J->before[1] ? 0 : 1;
  double ig[3] = {J->before[ia], J->before[3 + ia], J->before[6 + ia]},
         pg[3] = {J->before[9], J->before[10], J->before[11]};
  int ok = 1;
  for (int axis = 0; axis < (J->loop ? 1 : 3); axis++)
    ok &= autotune_fit(J->bins + (J->loop ? 0 : axis * 4), J->loop ? 3 : 4,
                       &J->models[axis], J->loop ? ig : 0,
                       J->loop ? &J->att_models[ia] : 0, pg);
  if (ok)
    ok = autotune_recommend(J->before, J->loop, J->models, J->candidate,
                            J->loop ? &J->att_models[ia] : 0);
  J->ok = ok;
}
void autotune_analysis_finish(autotune_state *A, const autotune_analysis *J) {
  if (A->phase != AT_ANALYZE || A->measurement != J->measurement)
    return;
  if (!J->ok) {
    autotune_stop(
        A, "insufficient signal or poor response fit; kept previous gains");
    return;
  }
  if (!A->ready || A->now - A->last_packet > .04 ||
      A->model_crc != model_crc(A) || A->model_use != A->L->use_learned) {
    autotune_stop(A, "stopped: flight changed during analysis");
    return;
  }
  memcpy(A->models, J->models, sizeof A->models);
  memcpy(A->candidate, J->candidate, sizeof A->candidate);
  memcpy(A->baseline, A->bins, sizeof A->bins);
  A->phase = AT_REVIEW;
  message(A, "recommendation ready; autotune apply repeats tests with leased "
             "trial gains");
}
int autotune_save_ready(autotune_state *A) {
  if (A->phase != AT_DONE || !A->have_status || A->now - A->last_packet > .04 ||
      A->N->tuning.pending || A->model_crc != model_crc(A) ||
      A->model_use != A->L->use_learned)
    return 0;
  for (int i = 0; i < 12; i++)
    if (fabsf(A->current[i] - A->candidate[i]) > 1e-4f)
      return 0;
  return 1;
}
int autotune_command(autotune_state *A, const char *s, char *reply,
                     unsigned cap) {
  if (strncmp(s, "autotune", 8) || (s[8] && s[8] != ' '))
    return 0;
  const char *cmd = s + 8;
  while (*cmd == ' ')
    cmd++;
  if (!*cmd || !strcmp(cmd, "status")) {
    if (!A->have_status || A->now - A->last_packet > .04) {
      snprintf(reply, cap,
               "tuning telemetry unavailable: update ESP32 firmware and check "
               "the Pi/flight-controller link; last test: %s",
               A->message);
      return 1;
    }
    snprintf(
        reply, cap,
        "autotune %s (%s %d/%d): %s; current attitude P %.2f %.2f %.2f D %.2f "
        "%.2f "
        "%.2f I %.2f %.2f %.2f; position %.2f %.2f %.2f",
        (const char *[]){"idle", "measuring", "review", "staging", "verifying",
                         "accepting", "done", "stopped", "analysing"}[A->phase],
        A->loop ? "position" : "attitude", A->index, A->loop ? 3 : 12,
        A->message, A->current[0], A->current[1], A->current[2], A->current[3],
        A->current[4], A->current[5], A->current[6], A->current[7],
        A->current[8], A->current[9], A->current[10], A->current[11]);
    if (A->phase == AT_REVIEW) {
      size_t used = strlen(reply);
      if (used < cap)
        snprintf(reply + used, cap - used,
                 "; proposed attitude P %.2f %.2f %.2f D %.2f %.2f %.2f I %.2f "
                 "%.2f %.2f; position %.2f %.2f %.2f",
                 A->candidate[0], A->candidate[1], A->candidate[2],
                 A->candidate[3], A->candidate[4], A->candidate[5],
                 A->candidate[6], A->candidate[7], A->candidate[8],
                 A->candidate[9], A->candidate[10], A->candidate[11]);
    }
    return 1;
  }
  if (!strcmp(cmd, "stop")) {
    autotune_stop(A, "stopped by pilot; previous gains restored");
  } else if (!strcmp(cmd, "attitude") || !strcmp(cmd, "position")) {
    if (autotune_busy(A)) {
      snprintf(reply, cap, "test already running; autotune stop first");
      return 1;
    }
    if (A->phase == AT_REVIEW)
      autotune_stop(A, "replacing recommendation");
    if (!A->ready || !A->L->accepted || A->L->external_load ||
        !A->L->have_fit || A->L->cal || A->L->thr || A->L->adapt == 2 ||
        !A->settled || !A->have_status || A->now - A->last_packet > .04 ||
        A->status[3] != 1 || A->N->H->act != 0) {
      snprintf(reply, cap,
               "needs calibrated, stable guided hover on bundled standard "
               "firmware with current tuning telemetry");
      return 1;
    }
    int pos = !strcmp(cmd, "position");
    if (pos && (!A->att_verified || A->verified_crc != model_crc(A) ||
                A->verified_use != A->L->use_learned)) {
      snprintf(reply, cap,
               "verify attitude tuning on this accepted flight model first");
      return 1;
    }
    A->measurement++;
    A->loop = pos;
    A->keep = A->L->keep;
    A->model_crc = model_crc(A);
    A->model_use =
        A->L->use_learned; /* guard captured the unmodified pilot target */
    memcpy(A->before, A->current, sizeof A->before);
    A->phase = AT_MEASURE;
    learn_command(A->L, LN_CMD_KEEP_OFF);
    plan(A);
    message(A, "measuring hardware response; autotune stop cancels");
  } else if (!strcmp(cmd, "apply")) {
    if (A->phase != AT_REVIEW || !A->ready || A->model_crc != model_crc(A) ||
        A->model_use != A->L->use_learned || !A->have_status ||
        A->now - A->last_packet > .04) {
      snprintf(reply, cap, "no current recommendation ready to verify");
      return 1;
    }
    A->session = A->session % 16777214 + 1;
    A->phase = AT_STAGE;
    A->stage_start = A->stamp;
    if (tune_send(A, 1)) {
      autotune_stop(A, "stopped: navigation refused trial gains");
      return 1;
    }
    message(A, "staging leased trial gains on the flight controller");
  } else {
    snprintf(reply, cap,
             "autotune attitude | position | apply | stop | status | save");
    return 1;
  }
  snprintf(reply, cap, "%s", A->message);
  return 1;
}
