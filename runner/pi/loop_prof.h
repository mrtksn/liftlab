/*
 * Where dfb_pi's main loop spends its processor time: each part of a pass is charged the main thread's CPU time
 * (CLOCK_THREAD_CPUTIME_ID: not time waiting, nor time another program had the processor) while it runs. A part
 * entered inside another (the learning inside the flight core's frames) is charged its own time only.
 *
 *   loop_prof_at(&LP, LP_X)     the pass goes on with part X (at the loop's top level)
 *   loop_prof_in(&LP, LP_X) ... loop_prof_out(&LP)     part X inside the current one
 *
 * Off until asked (a clock read per part costs ~5% of a Pi Zero): the first `cpu` starts it; each `cpu` after says,
 * since it was last asked: each part's share of the processor and how often it ran, the main thread's total, and the
 * process's (the main thread plus the fitting threads). `cpu off` stops it.
 */
#ifndef LOOP_PROF_H
#define LOOP_PROF_H
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/resource.h>

enum { LP_WAIT, LP_IO, LP_RADIO, LP_CMD, LP_FLOWIN, LP_LINK, LP_LEARN, LP_SUPER, LP_TUNE, LP_NAV, LP_NAVSTEP, LP_TLM, LP_OUT, LP_N };
static const char *const LP_NAME[LP_N] = {
  "waiting (poll)", "rangefinder, link flush, fit results", "radio packets", "text commands", "camera readings",
  "flight core's frames", "learning", "supervisor", "autotune and rope tests", "navigation (around its step)",
  "navigation step (its formulas)", "telemetry", "commands to the ESP32" };

typedef struct { int on; double t0, acc[LP_N], win0, ru0; long calls[LP_N], passes; int stk[8], sp; } loop_prof;

static inline double lp_cpu(void) { struct timespec ts; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts); return (double)ts.tv_sec + ts.tv_nsec * 1e-9; }
static inline double lp_wall(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return (double)ts.tv_sec + ts.tv_nsec * 1e-9; }
static inline double lp_proc(void) { struct rusage u; getrusage(RUSAGE_SELF, &u); return (double)u.ru_utime.tv_sec + u.ru_utime.tv_usec * 1e-6 + (double)u.ru_stime.tv_sec + u.ru_stime.tv_usec * 1e-6; }

static inline void loop_prof_init(loop_prof *P) { memset(P, 0, sizeof *P); }
static inline void lp_charge(loop_prof *P) { double n = lp_cpu(); P->acc[P->stk[P->sp]] += n - P->t0; P->t0 = n; }
static inline void loop_prof_at(loop_prof *P, int k) { if (!P->on) return; lp_charge(P); P->sp = 0; P->stk[0] = k; P->calls[k]++; if (k == LP_WAIT) P->passes++; }
static inline void loop_prof_in(loop_prof *P, int k) { if (!P->on) return; lp_charge(P); if (P->sp < 7) P->stk[++P->sp] = k; P->calls[k]++; }
static inline void loop_prof_out(loop_prof *P) { if (!P->on) return; lp_charge(P); if (P->sp > 0) P->sp--; }

/* the command (cpu, cpu off), asked in part `here` of the pass: the report since the last one, then a new window */
static inline void loop_prof_said(loop_prof *P, const char *arg, int here, char *out, size_t n) {
  if (!strcmp(arg, "off")) { P->on = 0; snprintf(out, n, "cpu: stopped measuring"); return; }
  if (!P->on) { memset(P, 0, sizeof *P); P->on = 1; P->stk[0] = here; P->t0 = lp_cpu(); P->win0 = lp_wall(); P->ru0 = lp_proc();
    snprintf(out, n, "cpu: measuring from now (it costs a few %% itself); ask cpu again in a few seconds, cpu off to stop"); return; }
  lp_charge(P);
  double w = lp_wall() - P->win0, pr = lp_proc() - P->ru0, main = 0; if (w <= 0) w = 1e-9;
  for (int k = 0; k < LP_N; k++) main += P->acc[k];
  int ord[LP_N]; for (int k = 0; k < LP_N; k++) ord[k] = k;
  for (int a = 0; a < LP_N; a++) for (int b = a + 1; b < LP_N; b++) if (P->acc[ord[b]] > P->acc[ord[a]]) { int x = ord[a]; ord[a] = ord[b]; ord[b] = x; }
  size_t k = (size_t)snprintf(out, n, "cpu over %.1f s: the process %.1f%% (main loop %.1f%%, the fitting threads %.1f%%), %.0f passes a second; the main loop's parts:",
                              w, 100 * pr / w, 100 * main / w, 100 * (pr - main > 0 ? pr - main : 0) / w, P->passes / w);
  for (int i = 0; i < LP_N && k < n; i++) { int j = ord[i]; if (P->acc[j] / w < 0.0005) continue;
    k += (size_t)snprintf(out + k, n - k, " %s %.1f%% (%.0f/s, %.0f µs each);", LP_NAME[j], 100 * P->acc[j] / w, P->calls[j] / w, P->calls[j] ? 1e6 * P->acc[j] / P->calls[j] : 0); }
  int top = P->stk[P->sp]; double t0 = P->t0;
  memset(P, 0, sizeof *P); P->on = 1; P->t0 = t0; P->stk[0] = top; P->win0 = lp_wall(); P->ru0 = lp_proc();
}

#endif
