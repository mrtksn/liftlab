/* Native test of the loading steps (rn_host.c): the built-in program flies; a good edit is loaded, shadowed,
 * blended and swapped in; an edit that gives NaN in flight is rejected in the background; an edit that traps
 * after it took over falls back to the previous program with the memory carried across; a corrupted image and
 * one whose formulas take other inputs are rejected.
 *   cc -O2 -Wall -Wextra -o test_rnhost rn.c rn_host.c test_rnhost.c -lm
 *   ./test_rnhost /tmp/h_builtin.rnp /tmp/h_edit.rnp /tmp/h_nan.rnp /tmp/h_trap.rnp /tmp/h_sig.rnp /tmp/h_calls.bin
 * (the images and calls come from the simulator: tools/host_test_data.js) */
#include "rn_host.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t *slurp(const char *path, uint32_t *len) {
  FILE *f = fopen(path, "rb"); if (!f) { perror(path); exit(2); }
  fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
  uint8_t *b = malloc((size_t)n); if (fread(b, 1, (size_t)n, f) != (size_t)n) { perror(path); exit(2); } fclose(f);
  *len = (uint32_t)n; return b;
}
#define ACAP 65536
#define CCAP 65536
#define PCAP 4096
static float ar[3][ACAP], pl[3][PCAP]; static int32_t cd[3][CCAP];
static const char *EV[] = { "", "loaded", "rejected", "swapped", "fell back", "built-in failed" };
static int events[8];
static void on_event(void *ctx, int code, const char *what) { (void)ctx; events[code]++; printf("    event: %s%s%s\n", EV[code], what ? ": " : "", what ? what : ""); }

static int32_t *calls; static uint32_t ncall_words, at;
static int step_calls(rn_host *H, int steps, int *errors) {
  static float out[4096];
  for (int s = 0; s < steps; s++) {
    for (;;) {
      if (at >= ncall_words) at = 0;
      int32_t fi = calls[at++], inst = calls[at++], n = calls[at++];
      if (fi < 0) break;
      int e = rn_host_call(H, fi, inst, (const float *)(calls + at), out); at += (uint32_t)n;
      if (e) { (*errors)++; }
    }
    rn_host_tick(H, 0.001f);
  }
  return 0;
}
static int fails;
#define CHECK(c, ...) do { if (c) printf("  ok   " __VA_ARGS__); else { printf("  FAIL " __VA_ARGS__); fails++; } printf("\n"); } while (0)

int main(int argc, char **argv) {
  if (argc < 7) { fprintf(stderr, "usage: %s builtin edit nan trap sig calls\n", argv[0]); return 2; }
  uint32_t lb, le, ln, lt, ls, lc;
  uint8_t *b = slurp(argv[1], &lb), *ed = slurp(argv[2], &le), *nan = slurp(argv[3], &ln), *tr = slurp(argv[4], &lt), *sg = slurp(argv[5], &ls);
  calls = (int32_t *)slurp(argv[6], &lc); ncall_words = lc / 4;
  float *arenas[3] = { ar[0], ar[1], ar[2] }, *pools[3] = { pl[0], pl[1], pl[2] }; int32_t *codes[3] = { NULL, cd[1], cd[2] };   /* the built-in program runs from its image, as from flash */
  rn_host H; memset(&H, 0, sizeof H); H.event = on_event;
  int e = rn_host_init(&H, b, lb, arenas, ACAP, codes, CCAP, pools, PCAP);
  printf("built-in program: %s (%d formulas, %d self-tests)\n", rn_error_text(e), H.slot[0].P.n_fn, H.slot[0].P.n_tests);
  if (e) return 1;
  e = rn_host_instances(&H, "servoPredictor", 4); CHECK(!e, "servoPredictor: 4 instances");
  int errors = 0, att = rn_host_find(&H, "attitudeEstimator");

  printf("1. the built-in program flies\n");
  step_calls(&H, 300, &errors); CHECK(errors == 0 && H.act == 0, "300 control steps, no traps");

  printf("2. a good edit: loaded, shadowed, blended, swapped in\n");
  int32_t qn; float q0[4]; float *q = rn_host_field(&H, att, "q", &qn); memcpy(q0, q, sizeof q0);
  e = rn_host_stage(&H, ed, le); CHECK(!e && H.phase == RN_PH_SHADOW, "staged: %s", rn_error_text(e));
  float *qc = 0; { int j = -1; for (int32_t i = 0; i < H.slot[H.cand].P.n_fn; i++) if (!strcmp(H.slot[H.cand].P.fn[i].name, "attitudeEstimator")) j = i;
    rn_fn *f = &H.slot[H.cand].P.fn[j]; for (int32_t k = 0; k < f->n_state; k++) if (!strcmp(f->state[k].name, "q")) qc = H.slot[H.cand].arena + f->state[k].addr; }
  CHECK(qc && !memcmp(qc, q0, sizeof q0), "the candidate starts with the flying program's memory (attitude estimate)");
  step_calls(&H, 1010, &errors); CHECK(H.phase == RN_PH_BLEND, "after 1 s: blending");
  step_calls(&H, 400, &errors); CHECK(H.phase == RN_PH_FLYING && H.act != 0 && H.prev == 0 && events[RN_EV_SWAPPED] == 1, "after 1.3 s: swapped in (largest difference %.3g)", H.max_diff);
  int edit_slot = H.act;

  printf("3. an edit that gives NaN in flight: rejected in the background\n");
  e = rn_host_stage(&H, nan, ln); CHECK(!e, "staged (it has no self-tests to catch it)");
  step_calls(&H, 50, &errors); CHECK(H.cand < 0 && H.act == edit_slot && events[RN_EV_REJECTED] == 1, "rejected; the good edit keeps flying");

  printf("4. an edit that traps after it took over: the previous program takes over\n");
  e = rn_host_stage(&H, tr, lt); CHECK(!e, "staged: %s", rn_error_text(e));
  step_calls(&H, 1400, &errors); CHECK(H.act != edit_slot && H.prev == edit_slot, "swapped in");
  int before = errors;
  { int sp = rn_host_find(&H, "servoPredictor"); int32_t n; float *c = rn_host_field(&H, sp, "n", &n); printf("    servoPredictor calls counted by the edit: %g\n", c ? c[0] : -1.0); }
  step_calls(&H, 3000, &errors); CHECK(events[RN_EV_FELL_BACK] == 1 && H.act == edit_slot && errors == before, "it trapped; the good edit took over and answered the same call");

  printf("5. images the loader must refuse\n");
  uint8_t *bad = malloc(le); memcpy(bad, ed, le); bad[le / 3] ^= 4;
  e = rn_host_stage(&H, bad, le); CHECK(e == RN_E_CRC, "corrupted: %s", rn_error_text(e));
  e = rn_host_stage(&H, sg, ls); CHECK(e == RN_E_SIGNATURE, "a formula with different inputs: %s", e == RN_E_SIGNATURE ? "rejected (signature)" : rn_error_text(e));
  step_calls(&H, 100, &errors); CHECK(H.act == edit_slot && errors == before, "still flying the good edit");

  printf("6. two slots (a board short of RAM): a new program loads while the built-in one flies\n");
  { rn_host H2; memset(&H2, 0, sizeof H2); H2.event = on_event;
    float *ar2[3] = { ar[0], ar[1], NULL };
    e = rn_host_init(&H2, b, lb, ar2, ACAP, codes, CCAP, pools, PCAP); if (!e) e = rn_host_instances(&H2, "servoPredictor", 4);
    CHECK(!e, "set up with two slots");
    int sw0 = events[RN_EV_SWAPPED], fb0 = events[RN_EV_FELL_BACK];
    e = rn_host_stage(&H2, ed, le); step_calls(&H2, 1400, &errors); CHECK(!e && H2.act == 1 && events[RN_EV_SWAPPED] == sw0 + 1, "an edit swapped in");
    e = rn_host_stage(&H2, ed, le); CHECK(!e && H2.cand == 1, "another program: the built-in one flies while it loads");
    step_calls(&H2, 1400, &errors); CHECK(H2.act == 1 && events[RN_EV_SWAPPED] == sw0 + 2, "and it swapped in");
    e = rn_host_stage(&H2, tr, lt); step_calls(&H2, 4400, &errors); CHECK(!e && H2.act == 0 && events[RN_EV_FELL_BACK] == fb0 + 1, "a program that traps falls back to the built-in one");
  }
  printf("%s (%d failures)\n", fails ? "FAILED" : "all passed", fails);
  return fails != 0;
}
