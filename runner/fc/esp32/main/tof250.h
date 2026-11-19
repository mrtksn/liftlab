#ifndef LIFTLAB_TOF250_H
#define LIFTLAB_TOF250_H
#include <stdint.h>

/* HCJ TOF250: register 0x00 starts a big-endian distance in centimetres. */
static inline float lb_tof250_metres(const uint8_t data[2]) {
  const unsigned cm = (unsigned)data[0] << 8 | data[1];
  if (cm < 5 || cm > 250) return -1;   /* includes the no-return code 8888 */
  return cm / 100.0f;
}
#endif
