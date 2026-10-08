/* Measured hardware tuning on the Pi; no true plant state or configured motor
 * lag. */
#ifndef AUTOTUNE_CORE_H
#define AUTOTUNE_CORE_H
#include "learn_core.h"
#include "nav_core.h"
#include "rn_link.h"
#include "super_core.h"
#define AT_SAMPLES 8192
#define AT_BINS 12
#define AT_TAIL (4 + PID_STATUS)
#define AT_LTEL_MAX (FC_LTEL_MAX + AT_TAIL)
enum {
  AT_IDLE,
  AT_MEASURE,
  AT_REVIEW,
  AT_STAGE,
  AT_VERIFY,
  AT_COMMIT,
  AT_DONE,
  AT_STOPPED,
  AT_ANALYZE
};
typedef struct {
  double r, i;
} at_complex;
typedef struct {
  double hz, quality;
  at_complex T, plant;
  int n, axis;
} at_bin;
typedef struct {
  double gain, lag, delay, lead, error;
} at_model;
typedef struct {
  nav_state *N;
  learn_state *L;
  super_state *S;
  void (*send)(void *, uint8_t, const float *, int);
  void *ctx;
  int phase, loop, index, ns, keep, ready, settled, have_status, att_verified,
      verified_use, model_use;
  uint32_t session, model_crc, verified_crc, measurement;
  double now, last_packet, stamp, start, settle, stage_start, max_motor;
  nav_sp target;
  float current[12], before[12], candidate[12], status[PID_STATUS];
  double samples[AT_SAMPLES][3];
  at_bin bins[AT_BINS], baseline[AT_BINS];
  at_model models[3], att_models[3];
  char message[160];
  unsigned events;
} autotune_state;
/* Immutable worker input/result: fitting never reads live flight state. */
typedef struct {
  uint32_t measurement;
  int loop, ok;
  float before[12], candidate[12];
  at_bin bins[AT_BINS];
  at_model models[3], att_models[3];
} autotune_analysis;
void autotune_analysis_input(const autotune_state *, autotune_analysis *);
void autotune_analyze(autotune_analysis *);
void autotune_analysis_finish(autotune_state *, const autotune_analysis *);
void autotune_init(autotune_state *, nav_state *, learn_state *, super_state *,
                   void (*)(void *, uint8_t, const float *, int), void *);
/* Always called, including when NAV/LTEL stops. A changed target/control or
 * missing telemetry cancels tests. */
void autotune_guard(autotune_state *, double, int, const nav_sp *,
                    const nav_out *, double, int);
void autotune_sample(autotune_state *, const float *, int, double);
int autotune_command(autotune_state *, const char *, char *, unsigned);
int autotune_busy(const autotune_state *);
int autotune_save_ready(autotune_state *);
void autotune_stop(autotune_state *, const char *);
/* Numerical primitives also exercised against independent simulator
 * implementations. */
at_bin autotune_bin(const double (*)[3], int, double, double, int, int);
int autotune_fit(const at_bin *, int, at_model *, const double *,
                 const at_model *, const double *);
int autotune_recommend(const float *, int, const at_model *, float *,
                       const at_model *);
#endif
