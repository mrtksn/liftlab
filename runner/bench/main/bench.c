/*
 * Bench firmware: the step runner on the real chip. Nothing drives motors or servos.
 *
 * At boot it prints the chip and memory, loads the built-in flight program (its self-tests run on this chip),
 * and times every flight formula on the inputs its self-tests carry. Then it runs a 1 kHz loop on core 1 that
 * calls every formula once per step through the program slots (rn_host), and prints the loop's load every 2 s.
 * On core 0 it listens on the same USB serial port for programs (pi/send_program.py) and reports the loading
 * steps: checked, flying in the background, swapped, rejected or fell back.
 *
 * The output is plain text at 115200 baud (esptool-js's Console, or any serial monitor); the program frames
 * and the replies to them are rn_link frames on the same port, which a serial monitor shows as a few odd bytes.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_flash.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "rn_host.h"
#include "rn_link.h"

extern const uint8_t *const rn_builtin_img;
extern const uint32_t rn_builtin_len;

#define CODE_CAP 8192           /* code words for a loaded program */
#define POOL_CAP 256            /* floats of instance memory per slot */
#define IMG_CAP (40 * 1024)     /* largest program accepted over the link (tools/export_program.js --link) */
#define N_SERVOS 4
#define LINK UART_NUM_0

static rn_host H;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static void host_lock(void *c, int on) { if (on) portENTER_CRITICAL(&mux); else portEXIT_CRITICAL(&mux); }

/* Events, passed from whichever core to the link task, which does all the printing once the loop runs. */
#define NEV 16
static char ev_text[NEV][72]; static volatile uint32_t ev_w, ev_r;
static const char *EVN[] = { "", "loaded, flying in the background", "rejected", "swapped in: the new program is flying", "fell back to the previous program", "the built-in program failed" };
static void host_event(void *ctx, int code, const char *what) {
  uint32_t i = ev_w % NEV; snprintf(ev_text[i], sizeof ev_text[i], "%s%s%s", EVN[code], what ? ": " : "", what ? what : ""); ev_w++;
}

/* Every formula's inputs, flat, taken from the built-in program's first self-test for it. */
static float *fn_in[RN_FN_MAX]; static int fn_out_n[RN_FN_MAX];
static int32_t rd32(const uint8_t *p) { int32_t v; memcpy(&v, p, 4); return v; }

/* Time every formula on its self-test inputs: min and average microseconds per call, and steps. */
static void bench_formulas(rn_prog *P, const uint8_t *img, uint32_t len) {
  static double sum_us[RN_FN_MAX]; static int64_t min_us[RN_FN_MAX]; static int calls[RN_FN_MAX], steps[RN_FN_MAX];
  for (int i = 0; i < RN_FN_MAX; i++) { min_us[i] = 1 << 30; }
  uint32_t at = P->tests_at;
  for (int32_t t = 0; t < P->n_tests; t++) {
    int32_t fi = rd32(img + at); at += 4;
    uint32_t in_at = at;
    /* skip inputs and outputs to find where this test ends */
    for (int part = 0; part < 2; part++) for (;;) { int32_t a = rd32(img + at), n = rd32(img + at + 4); at += 8; (void)a; if (!n) break; at += 4u * (uint32_t)n; }
    const rn_fn *f = &P->fn[fi];
    if (!fn_in[fi]) {                                  /* flat inputs for the loop: each argument's region, in order */
      int total = 0; for (int k = 0; k < f->n_args; k++) total += f->arg_size[k];
      fn_in[fi] = calloc((size_t)(total ? total : 1), sizeof(float)); fn_out_n[fi] = f->ret_size;
      int off = 0;
      for (int k = 0; k < f->n_args; k++) {
        if (!f->arg_size[k]) continue;
        for (uint32_t q = in_at;;) { int32_t a = rd32(img + q), n = rd32(img + q + 4); q += 8; if (!n) break; if (a == f->arg_addr[k] && n == f->arg_size[k]) memcpy(fn_in[fi] + off, img + q, (size_t)n * 4); q += 4u * (uint32_t)n; }
        off += f->arg_size[k];
      }
    }
    for (int rep = 0; rep < 20; rep++) {
      rn_clear(P);
      for (uint32_t q = in_at;;) { int32_t a = rd32(img + q), n = rd32(img + q + 4); q += 8; if (!n) break; memcpy(P->arena + a, img + q, (size_t)n * 4); q += 4u * (uint32_t)n; }
      int64_t t0 = esp_timer_get_time();
      int e = rn_run(P, fi);
      int64_t dt = esp_timer_get_time() - t0;
      if (e) { printf("  %s: trapped (%s)\n", f->name, rn_error_text(e)); break; }
      sum_us[fi] += (double)dt; calls[fi]++; steps[fi] = P->steps; if (dt < min_us[fi]) min_us[fi] = dt;
    }
  }
  rn_clear(P);
  printf("\nTime per call on this chip (inputs from the self-tests):\n  %-24s %8s %8s %8s\n", "formula", "min us", "avg us", "steps");
  for (int32_t i = 0; i < P->n_fn; i++) if (calls[i]) printf("  %-24s %8lld %8.1f %8d\n", P->fn[i].name, (long long)min_us[i], sum_us[i] / calls[i], steps[i]);
}

/* The 1 kHz loop: every formula once per step, servoPredictor once per servo. */
static volatile int64_t loop_sum, loop_max; static volatile int loop_n, loop_err, loop_late;
static int f_learn;
static void flight_task(void *arg) {
  float *out = malloc(1024 * sizeof(float));
  int nfn = H.slot[0].P.n_fn, sp = rn_host_find(&H, "servoPredictor");
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, 1);
    int64_t t0 = esp_timer_get_time();
    for (int i = 0; i < nfn; i++) {
      if (!fn_in[i] || i == f_learn) continue;                  /* the learning runs on core 0 */
      int insts = i == sp ? N_SERVOS : 1;
      for (int s = 0; s < insts; s++) if (rn_host_call(&H, i, s, fn_in[i], out)) loop_err++;
    }
    rn_host_tick(&H, 0.001f);
    int64_t dt = esp_timer_get_time() - t0;
    loop_sum += dt; loop_n++; if (dt > loop_max) loop_max = dt; if (dt > 1000) loop_late++;
  }
}

/* The in-flight learning at 200 Hz on core 0, as flight controllers usually run it: it has its own working
 * space in the program (ownPool), so it can run while the flight loop does. */
static volatile int64_t learn_sum, learn_max; static volatile int learn_n, learn_err;
static void learn_task(void *arg) {
  float *out = malloc(1024 * sizeof(float));
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, 5);
    int64_t t0 = esp_timer_get_time();
    if (fn_in[f_learn] && rn_host_call(&H, f_learn, 0, fn_in[f_learn], out)) learn_err++;
    int64_t dt = esp_timer_get_time() - t0;
    learn_sum += dt; learn_n++; if (dt > learn_max) learn_max = dt;
  }
}

static uint8_t *img_buf;
static void link_send(uint8_t type, const char *text) {
  static uint8_t fr[160]; uint32_t n = rn_link_frame(fr, sizeof fr, type, (const uint8_t *)text, (uint32_t)strlen(text));
  if (n) uart_write_bytes(LINK, fr, n);
}
static void say(const char *text) { printf("%s\n", text); link_send(RN_LINK_EVENT, text); }
static void link_task(void *arg) {
  static rn_link L; rn_link_init(&L, img_buf, img_buf ? IMG_CAP : 0);
  static uint8_t rx[256];
  int64_t next = esp_timer_get_time() + 2000000;
  for (;;) {
    while (ev_r != ev_w) { say(ev_text[ev_r % NEV]); ev_r++; }
    int n = uart_read_bytes(LINK, rx, sizeof rx, pdMS_TO_TICKS(20));
    for (int i = 0; i < n; i++) {
      int type = rn_link_feed(&L, rx[i]);
      if (type == RN_LINK_PROGRAM) {
        char s[96]; snprintf(s, sizeof s, "received a program: %u bytes; checking it", (unsigned)L.len); say(s);
        int64_t t0 = esp_timer_get_time();
        int e = rn_host_prepare(&H, img_buf, L.len);
        if (!e) { snprintf(s, sizeof s, "checked and self-tested in %lld ms", (long long)((esp_timer_get_time() - t0) / 1000)); say(s); }
      } else if (type == RN_LINK_STATUS) {
        char s[120]; snprintf(s, sizeof s, "flying slot %d (0 = built-in), candidate %d, phase %d, previous %d, free heap %u", H.act, H.cand, H.phase, H.prev, (unsigned)esp_get_free_heap_size());
        link_send(RN_LINK_REPORT, s); printf("%s\n", s);
      } else if (type < 0) say("dropped a damaged or oversized frame");
    }
    if (esp_timer_get_time() > next) {
      next += 2000000;
      int nn = loop_n; int64_t sum = loop_sum, mx = loop_max; loop_n = 0; loop_sum = 0; loop_max = 0;
      if (nn) printf("flight loop, core 1: %d steps, %.0f us per step on average (%.1f%% of the core at 1 kHz), longest %lld us, over 1 ms: %d, traps: %d, flying slot %d\n",
                     nn, (double)sum / nn, (double)sum / nn / 10.0, (long long)mx, loop_late, loop_err, H.act);
      int ln = learn_n; int64_t ls = learn_sum, lm = learn_max; learn_n = 0; learn_sum = 0; learn_max = 0;
      if (ln) printf("learning, core 0: %d updates, %.0f us each on average (%.1f%% of the core at 200 Hz), longest %lld us, traps: %d\n",
                     ln, (double)ls / ln, (double)ls / ln / 50.0, (long long)lm, learn_err);
    }
  }
}

void app_main(void) {
  esp_chip_info_t ci; esp_chip_info(&ci);
  uint32_t flash = 0; esp_flash_get_size(NULL, &flash);
  printf("\n\nLiftLab: step runner bench\nchip: %s rev %d.%d, %d cores, flash %u MB\n", CONFIG_IDF_TARGET, ci.revision / 100, ci.revision % 100, ci.cores, (unsigned)(flash >> 20));
  printf("free heap %u bytes, largest block %u bytes\n", (unsigned)esp_get_free_heap_size(), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

  /* The built-in program alone first (in slot 0, before the slots are set up): its size decides the slots' size. */
  rn_prog *pp = &H.slot[0].P;
#define probe (*pp)
  float *a0 = NULL; uint32_t acap = 0;
  { const uint8_t *p = rn_builtin_img; int32_t asz = rd32(p + 8); acap = (uint32_t)asz + 512; }
  a0 = heap_caps_malloc(acap * sizeof(float), MALLOC_CAP_8BIT);
  /* Steps in IRAM (only 32-bit access, which is all the runner does with them): flash is behind a small cache.
   * IRAM is otherwise unused here, so this costs no data memory. */
  int32_t cwords = rd32(rn_builtin_img + 16);
  int32_t *c0 = heap_caps_malloc((size_t)cwords * 4, MALLOC_CAP_EXEC | MALLOC_CAP_32BIT);
  int e = a0 ? rn_load(&probe, rn_builtin_img, rn_builtin_len, a0, acap, c0, c0 ? (uint32_t)cwords : 0) : RN_E_TOO_BIG;
  printf("built-in program: %s, %u bytes, %d formulas, arena %d floats (%d KB), %d step words (in %s)\n", rn_error_text(e), (unsigned)rn_builtin_len, (int)probe.n_fn, (int)probe.arena_size, (int)(probe.arena_size * 4 / 1024), (int)probe.code_len, c0 ? "IRAM" : "flash");
  if (e) return;
  float worst = 0; int64_t t0 = esp_timer_get_time();
  e = rn_selftest(&probe, rn_builtin_img, rn_builtin_len, 1e-2f, &worst);
  printf("self-tests on this chip: %s, %d tests in %lld ms, largest difference from the simulator %.2g%%\n", rn_error_text(e), (int)probe.n_tests, (long long)((esp_timer_get_time() - t0) / 1000), worst * 100);
  bench_formulas(&probe, rn_builtin_img, rn_builtin_len);

  /* A slot for programs sent over the link and a buffer to receive them, then a third slot if there's room. */
  uint8_t *img = heap_caps_malloc(IMG_CAP, MALLOC_CAP_8BIT);
  float *a1 = img ? heap_caps_malloc(acap * sizeof(float), MALLOC_CAP_8BIT) : NULL;
  int32_t *c1 = a1 ? heap_caps_malloc(CODE_CAP * 4, MALLOC_CAP_EXEC | MALLOC_CAP_32BIT) : NULL;
  if (!c1) { free(a1); a1 = NULL; free(img); img = NULL; }
  float *a2 = img ? heap_caps_malloc(acap * sizeof(float), MALLOC_CAP_8BIT) : NULL;
  int32_t *c2 = a2 ? heap_caps_malloc(CODE_CAP * 4, MALLOC_CAP_EXEC | MALLOC_CAP_32BIT) : NULL;
  if (a2 && !c2) { free(a2); a2 = NULL; }
  float *p0 = calloc(POOL_CAP, 4), *p1 = calloc(POOL_CAP, 4), *p2 = calloc(POOL_CAP, 4);
  printf("\nprogram slots: built-in, %s\n", !img ? "none for loaded programs (not enough memory): the built-in program only" :
         a2 ? "two for loaded programs (flying and next, plus the previous one to fall back to)" :
         "one for loaded programs (a new one loads while the built-in one flies)");
  float *arenas[3] = { a0, img ? a1 : NULL, a2 }, *pools[3] = { p0, p1, p2 };
  int32_t *codes[3] = { c0, c1, c2 };
  H.event = host_event; H.lock = host_lock;
  e = rn_host_init(&H, rn_builtin_img, rn_builtin_len, arenas, acap, codes, CODE_CAP, pools, POOL_CAP);
  if (!e) e = rn_host_instances(&H, "servoPredictor", N_SERVOS);
  printf("program slots set up: %s\n", rn_error_text(e));
  if (e) return;
  img_buf = img;
  printf("free heap now %u bytes, largest block %u bytes\n", (unsigned)esp_get_free_heap_size(), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

  uart_driver_install(LINK, 4096, 2048, 0, NULL, 0);
  uart_vfs_dev_use_driver(LINK);
  printf("\n1 kHz loop on core 1: every formula once per step (servoPredictor for %d servos), except the learning: 200 Hz on core 0. Listening for programs on this port.\n\n", N_SERVOS);
  xTaskCreatePinnedToCore(link_task, "link", 6144, NULL, 5, NULL, 0);
  f_learn = rn_host_find(&H, "identifyEffectiveness");
  xTaskCreatePinnedToCore(learn_task, "learn", 6144, NULL, 10, NULL, 0);
  xTaskCreatePinnedToCore(flight_task, "flight", 6144, NULL, configMAX_PRIORITIES - 1, NULL, 1);
}
