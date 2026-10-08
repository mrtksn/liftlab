/* Real host/learning state, with synthetic fresh telemetry features: acceptance, excitation gates and rollback.
 * Include the implementation to exercise its internal validation separately from the flight physics. */
#include "../runner/fc/learn_core.c"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "../runner/fc/nav_core.h"
extern const uint8_t *const rn_builtin_pi_img;
extern const uint32_t rn_builtin_pi_len;
static rn_host H;
static float arenas[3][131072], pools[3][8192];
static int32_t codes[3][65536];
static learn_state L;
static void setup(void) {
  memset(&L, 0, sizeof L);
  float *a[3] = { arenas[0], arenas[1], arenas[2] }, *p[3] = { pools[0], pools[1], pools[2] };
  int32_t *c[3] = { codes[0], codes[1], codes[2] };
  assert(!rn_host_init(&H, rn_builtin_pi_img, rn_builtin_pi_len, a, 131072, c, 65536, p, 8192));
  assert(!learn_init(&L, &H));
  FILE *f = fopen("runner/fc/testdata/quadx.dfa", "rb"); assert(f);
  uint8_t blob[8192]; size_t len = fread(blob, 1, sizeof blob, f); fclose(f); assert(!learn_airframe(&L, blob, (uint32_t)len));
  L.state = FC_ARMED; L.flags = 1; L.keep = 1; L.dt = 0.005f; L.R[0] = L.R[4] = L.R[8] = 1;
  for (int i = 0; i < L.FA.A.n_motors; i++) L.u[i] = 0.4f;
  float x[LN_IN] = { 0 };
  int k = put_list(pk, 0, x, L.n, LN_IN);
  for (int j = 0; j < 6; j++) pk[k++] = j == 2 ? 9.81f : 0;
  for (int j = 0; j < 3; j++) pk[k++] = 0;
  pk[k++] = L.dt;
  for (int r = 0; r < 6; r++) k = put_list(pk, k, L.prior[r], L.n, LN_IN);
  pk[k++] = 30; pk[k++] = 0; k = put_list(pk, k, x, 0, LN_IN);
  pk[k++] = 1;
  for (int j = 0; j < 4; j++) k = put_list(pk, k, x, L.n, LN_IN);
  /* Warm the real formula until its x/y fields have real list lengths. */
  for (int j = 0; j < 310; j++) assert(!call(&L, L.f_rls, res));
  for (int r = 0; r < 6; r++) for (int j = 0; j < L.n; j++) L.B[r][j] = L.prior[r][j] * 0.96f;
}
static void observations(int steps, int kind, float scale) {
  int32_t nx, ny; float *x = rn_host_field(&H, L.f_rls, "x", &nx), *y = rn_host_field(&H, L.f_rls, "y", &ny); assert(x && y);
  for (int i = 0; i < steps; i++) {
    L.t += L.dt;
    x[0] = (float)(2 * L.n);
    for (int j = 0; j < L.n; j++) x[j + 1] = (kind == 2 ? 0.00001f : 0.1f) * sinf(2 * PI_ * (kind == 1 ? 2 : 1.2f + 0.7f * j) * L.t + (kind == 1 ? 0 : j));
    for (int j = L.n; j < 2 * L.n; j++) x[j + 1] = 0;
    for (int r = 0; r < 6; r++) { y[r] = 0; for (int j = 0; j < L.n; j++) y[r] += scale * L.prior[r][j] * x[j + 1]; }
    adapt_step(&L);
  }
}
int main(void) {
  setup(); assert(learn_command(&L, LN_CMD_USE_LEARNED) == -1);
  observations(610, 1, 0.96f); assert(!L.use_learned && !L.adapt_updates);
  setup(); observations(610, 2, 0.96f); assert(!L.use_learned && !L.adapt_updates);
  setup(); observations(610, 0, -1); assert(!L.use_learned && !L.adapt_updates);
  setup(); observations(610, 0, 0.96f); assert(L.use_learned && L.accepted && L.adapt == 2 && L.adapt_updates == 1);
  for (int r = 0; r < 6; r++) for (int j = 0; j < L.n; j++) assert(fabsf(L.flyB[r][j] - L.prior[r][j]) <= fabsf(L.prior[r][j]) * 0.051f + 1e-6f);
  observations(410, 0, 1); assert(!L.use_learned && !L.accepted && L.adapt_rollbacks == 1);
  setup(); observations(610, 0, 0.96f); adapt_reset(&L); L.adapt = 2;
  observations(410, 2, 0.96f); assert(!L.use_learned && L.adapt_rollbacks == 1); /* Missing probation signal cannot approve the update. */
  setup(); observations(610, 0, 0.96f); observations(410, 0, 0.96f);
  assert(L.use_learned && L.accepted && !L.adapt_rollbacks); /* Informative probation retains a good model. */
  setup(); observations(610, 0, 0.96f); assert(!learn_command(&L, LN_CMD_KEEP_OFF)); assert(!L.keep && L.adapt_rollbacks == 1);
  setup(); L.model_dirty = 0; float out[FC_MODEL_MAX]; assert(!learn_model_frame(&L, out));
  for (int r = 0; r < 6; r++) for (int j = 0; j < L.n; j++) L.B[r][j] *= 1.5f;
  assert(!learn_model_frame(&L, out)); /* Changing the estimator never streams an unaccepted model. */
  fc_state F = { 0 }; F.A.n_motors = 4; float exc[12] = { 3, 1, 1, 255, 4, 0, 0, 0, 0, 0, 1, 1 };
  assert(!fc_exc(&F, exc, 12) && F.exc_mode == 3 && F.exc_angle < 0.07f && !F.exc_hold_m && !F.exc_hold_s);
  exc[10] = 3; assert(fc_exc(&F, exc, 12) == -1);
  F.t = 0.02; F.lt_n = 2; F.lt_alpha[0] = 8; F.lt_alpha[1] = 4; F.lt_alpha[2] = 2;
  float tel[FC_LTEL_MAX], sample[4]; fc_ltel(&F, tel); assert(fc_tuning_sample(&F, sample) == 4);
  assert(sample[0] == tel[0] && sample[1] == 4 && sample[2] == 2 && sample[3] == 1 && F.lt_alpha[0] == 0);
  nav_state N = { 0 }; assert(!nav_test_target(&N, 0, 10) && N.test_offset == 0.2f && N.test_left == 0.1f);
  nav_in in = { 0 }; nav_sp sp = { 0 }; nav_out no;
  nav_step(&N, &in, &sp, 0.11f, &no); assert(!N.test_left);assert(nav_test_target(&N, 3, 0) == -1);
  puts("Fresh-data acceptance, correlated/weak/bad-data rejection, bounded updates, probation rollback, keep-off rollback and expiring reference tests passed");
  return 0;
}
