/* The calibration's test-stage fits (learn_core.h ln_fit_job) on a worker thread: a second or more of arithmetic on
 * a Pi Zero, which on the main loop would leave the flight core without commands (and in its failsafe) mid-hover.
 * The worker has its own host with the same built-in program (this companion never loads another into its main
 * host), so it shares no formula memory or scratch with the main loop's calls (rn.h RN_TLS). The learning holds
 * while the job is out and goes on when learn_fit_poll() hands it back; the results are the ones it would have got
 * at once. Main owns the learning; the worker touches only the job, between start and done. */
#ifndef PI_LEARN_FIT_WORKER_H
#define PI_LEARN_FIT_WORKER_H
#include "learn_core.h"
#include <pthread.h>
#include <stdatomic.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
typedef struct {
  rn_host H;
  ln_fit_job *job;
  pthread_t thread;
  atomic_int done;
  int running, ready;
} learn_fit_worker;
static float lfw_arena[131072]; static int32_t lfw_code[65536];
static void *learn_fit_work(void *ctx) {
  learn_fit_worker *W = ctx;
#ifdef SYS_gettid
  setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), 10);   /* below the main loop: it yields the core whenever that wakes */
#endif
  learn_fit_run(&W->H, W->job);
  atomic_store_explicit(&W->done, 1, memory_order_release);
  return 0;
}
static int learn_fit_start(void *ctx, ln_fit_job *J) {          /* learn_state.fit_start: 0 started, else the learning fits at once */
  learn_fit_worker *W = ctx;
  if (!W->ready || W->running) return -1;
  W->job = J; atomic_store_explicit(&W->done, 0, memory_order_relaxed);
  if (pthread_create(&W->thread, 0, learn_fit_work, W)) return -1;
  W->running = 1;
  return 0;
}
/* Set up once the main host is (its formula table is shared: rn_host.c fmap), and hook it into the learning. */
static void learn_fit_worker_init(learn_fit_worker *W, learn_state *L, const uint8_t *img, uint32_t len) {
  float *arenas[3] = { lfw_arena, 0, 0 }, *pools[3] = { 0, 0, 0 }; int32_t *codes[3] = { lfw_code, 0, 0 };
  W->ready = !rn_host_init(&W->H, img, len, arenas, sizeof lfw_arena / sizeof *lfw_arena, codes, sizeof lfw_code / sizeof *lfw_code, pools, 0);
  if (W->ready) { L->fit_start = learn_fit_start; L->fit_ctx = W; }
}
/* Nonblocking, every pass of the main loop. */
static void learn_fit_poll(learn_fit_worker *W, learn_state *L) {
  if (W->running && atomic_load_explicit(&W->done, memory_order_acquire)) {
    pthread_join(W->thread, 0);
    W->running = 0;
    learn_fit_done(L);
  }
}
#endif
