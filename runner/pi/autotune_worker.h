/* The worker owns only an immutable numerical job. Main owns all flight/tuner
 * state. */
#ifndef PI_AUTOTUNE_WORKER_H
#define PI_AUTOTUNE_WORKER_H
#include "autotune_core.h"
#include <pthread.h>
#include <stdatomic.h>
typedef struct {
  autotune_analysis job;
  pthread_t thread;
  atomic_int done;
  int running;
} autotune_worker;
static void *autotune_work(void *ctx) {
  autotune_worker *W = ctx;
  autotune_analyze(&W->job);
  atomic_store_explicit(&W->done, 1, memory_order_release);
  return 0;
}
/* Nonblocking poll, called beside navigation even if the telemetry stops. */
static void autotune_work_poll(autotune_worker *W, autotune_state *A) {
  if (W->running && atomic_load_explicit(&W->done, memory_order_acquire)) {
    pthread_join(W->thread, 0);
    W->running = 0;
    autotune_analysis_finish(A, &W->job);
  }
  if (!W->running && A->phase == AT_ANALYZE) {
    autotune_analysis_input(A, &W->job);
    atomic_store_explicit(&W->done, 0, memory_order_relaxed);
    if (pthread_create(&W->thread, 0, autotune_work, W)) {
      autotune_stop(A, "stopped: could not start response analysis");
      return;
    }
    W->running = 1;
  }
}
#endif
