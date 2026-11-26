/* Checks who flies it on a board with the navigation (rc_core.h rc_who): the navigation while the pilot wants it and
 * it knows where it is, the sticks otherwise; the handovers' timing, what they say, and the set point while the sticks
 * fly it. */
#include "rc_core.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static int run(rc_who *W, nav_state *N, int want, int pos, float seconds, const char **said) {
  int s = W->sticks; *said = 0;
  for (int i = 0; i < (int)(seconds / 0.01f + 0.5f); i++) { const char *m; s = rc_who_step(W, want, pos, 0.01f, N, &m); if (m) *said = m; }
  return s;
}

int main(void) {
  static rc_who W; static nav_state N; const char *m;
  /* it starts on the sticks (a restart in the air passes the radio's sticks on) */
  CHECK(run(&W, &N, 0, 0, 0.01f, &m) == 1 && !m, "starts on the sticks, quietly");
  /* the fly switch on, position good: the navigation after a second, its integral cleared */
  N.iPos[0] = 3;
  CHECK(run(&W, &N, 1, 1, 0.95f, &m) == 1, "position good for under a second: still the sticks");
  CHECK(run(&W, &N, 1, 1, 0.1f, &m) == 0 && m && strstr(m, "holding here") && N.iPos[0] == 0, "then the navigation holds (said: %s; iPos %.1f)", m ? m : "nothing", (double)N.iPos[0]);
  /* a short dropout doesn't hand it over; 0.3 s does */
  CHECK(run(&W, &N, 1, 0, 0.25f, &m) == 0, "position gone for 0.25 s: still the navigation");
  CHECK(run(&W, &N, 1, 1, 0.05f, &m) == 0, "and back: it stays");
  CHECK(run(&W, &N, 1, 0, 0.31f, &m) == 1 && m && strstr(m, "position lost"), "gone for 0.3 s: the sticks (said: %s)", m ? m : "nothing");
  /* a motor or rope test asks for the sticks: at once, quietly */
  run(&W, &N, 1, 1, 1.1f, &m);
  CHECK(run(&W, &N, 0, 1, 0.01f, &m) == 1 && !m, "a test asks for the sticks: at once, quietly");
  /* what it wants: on the ground the navigation; flying on the sticks, back only with the fly switch on; a test the sticks */
  { rc_who V = { 0 }; V.sticks = 1;
    CHECK(rc_who_wants(&V, 0, 0, 0) && rc_who_wants(&V, 1, 1, 0) && !rc_who_wants(&V, 1, 0, 0), "on the sticks: the ground, or the fly switch on, ask for the navigation; flying with it off, not");
    V.sticks = 0; CHECK(rc_who_wants(&V, 1, 0, 0), "the navigation flying, fly off: it keeps it (it lands it)");
    CHECK(!rc_who_wants(&V, 0, 1, 1), "a test: the sticks"); }
  /* the set point while the sticks fly it: where the drone is, flying while armed; the radio's target with it */
  { rc_pilot P; rc_pilot_init(&P); nav_out o; memset(&o, 0, sizeof o); o.p[0] = 1; o.p[1] = -2; o.p[2] = 0.05f;
    nav_sp sp; memset(&sp, 0, sizeof sp); sp.target[0] = 9; sp.vref[1] = 1;
    rc_who_sp(&P, &o, 1, &sp);
    CHECK(sp.fly && sp.target[0] == 1 && sp.target[1] == -2 && fabsf(sp.target[2] - 0.15f) < 1e-6f && sp.vref[1] == 0, "armed: the target is where it is (height at least 0.15 m): %.2f %.2f %.2f", (double)sp.target[0], (double)sp.target[1], (double)sp.target[2]);
    CHECK(P.have_target && P.target[1] == -2, "and the radio's pilot keeps it");
    memset(&sp, 0, sizeof sp); sp.fly = 1; sp.target[0] = 9; rc_who_sp(&P, &o, 0, &sp);
    CHECK(!sp.fly && sp.target[0] == 9, "disarmed: not flying, the target left alone"); }
  if (fails) { printf("who: %d checks failed\n", fails); return 1; }
  printf("who: all checks passed\n");
  return 0;
}
