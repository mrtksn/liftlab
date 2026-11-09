/* A diagnostic must distinguish gravity from missing, failed or stale readings. */
#include <assert.h>
#include <string.h>
#include "../runner/fc/esp32/main/accel_readout.h"

int main(void) {
  char out[256];
  accel_readout s = { .acc = {0, 0, 9.80665f}, .us = 1000000, .seq = 1, .valid = 1 };
  accel_readout_format(out, sizeof out, 1, &s, 1002000);
  assert(strstr(out, "z=+9.807") && strstr(out, "magnitude=1.000 g") && strstr(out, "age=2 ms"));
  s.acc[0] = -9.80665f; s.acc[2] = 0;
  accel_readout_format(out, sizeof out, 1, &s, 1002000);
  assert(strstr(out, "x=-9.807") && strstr(out, "magnitude=1.000 g"));
  accel_readout_format(out, sizeof out, 0, &s, 1002000);
  assert(strstr(out, "no accelerometer"));
  s.seq = 0;
  accel_readout_format(out, sizeof out, 1, &s, 1002000);
  assert(strstr(out, "waiting"));
  s.seq = 1; s.valid = 0;
  accel_readout_format(out, sizeof out, 1, &s, 1002000);
  assert(strstr(out, "read failed") && !strstr(out, "x="));
  s.valid = 1;
  accel_readout_format(out, sizeof out, 1, &s, 1100001);
  assert(strstr(out, "stale") && !strstr(out, "x="));
  accel_readout_format(out, sizeof out, 1, &s, 999999);
  assert(strstr(out, "stale"));
  s.acc[0] = NAN;
  accel_readout_format(out, sizeof out, 1, &s, 1002000);
  assert(strstr(out, "invalid sensor"));
  struct { char text[8]; char guard; } tiny = {{0}, 'X'};
  accel_readout_format(tiny.text, sizeof tiny.text, 0, &s, 1002000);
  assert(tiny.text[7] == 0 && tiny.guard == 'X');
  puts("Acceleration readout: gravity, axis signs, freshness, read failures and bounds passed.");
  return 0;
}
