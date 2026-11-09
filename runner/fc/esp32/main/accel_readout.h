/* Console diagnostics for a cached IMU sample; no I2C access or allocation. */
#ifndef LIFTLAB_ACCEL_READOUT_H
#define LIFTLAB_ACCEL_READOUT_H
#include <math.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
  float acc[3];
  int64_t us;
  uint32_t seq;
  int valid;
} accel_readout;

static inline void accel_readout_format(char *out, size_t cap, int present,
                                       const accel_readout *sample, int64_t now) {
  if (!present) { snprintf(out, cap, "accel: no accelerometer detected; check wiring and reboot"); return; }
  if (!sample->seq) { snprintf(out, cap, "accel: waiting for the first sensor sample"); return; }
  if (!sample->valid) { snprintf(out, cap, "accel: sensor read failed; check wiring"); return; }
  if (now < sample->us || now - sample->us > 100000) {
    snprintf(out, cap, "accel: sensor sample is stale"); return;
  }
  const float *a = sample->acc;
  if (!isfinite(a[0]) || !isfinite(a[1]) || !isfinite(a[2])) {
    snprintf(out, cap, "accel: invalid sensor values"); return;
  }
  float g = hypotf(hypotf(a[0], a[1]), a[2]) / 9.80665f;
  snprintf(out, cap, "accel: x=%+.3f y=%+.3f z=%+.3f m/s^2; magnitude=%.3f g; age=%lld ms (sensor axes)",
           (double)a[0], (double)a[1], (double)a[2], (double)g,
           (long long)((now - sample->us) / 1000));
}
#endif
