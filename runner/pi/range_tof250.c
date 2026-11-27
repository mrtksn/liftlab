/* The TOF250 rangefinder on the Pi's I²C: see range_tof250.h. */
#define _DEFAULT_SOURCE
#include "range_tof250.h"
#include "esp32/main/tof250.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#ifdef __linux__
#include <linux/i2c-dev.h>
#else
#define I2C_SLAVE 0x0703
#endif

int tof250_parse(tof250_pi *T, const char *spec) {
  memset(T, 0, sizeof *T); T->fd = -1;
  if (strncmp(spec, "tof250:", 7)) { snprintf(T->why, sizeof T->why, "--range KIND:BUS:ADDRESS, KIND tof250 (e.g. tof250:/dev/i2c-1:0x52)"); return -1; }
  const char *s = spec + 7, *c = strrchr(s, ':');
  if (!c || c == s) { snprintf(T->why, sizeof T->why, "--range tof250:BUS:ADDRESS (e.g. tof250:/dev/i2c-1:0x52)"); return -1; }
  char *e; long a = strtol(c + 1, &e, 0);
  if (*e || a < 8 || a > 0x77) { snprintf(T->why, sizeof T->why, "--range: the address is 8 to 0x77 (the TOF250's is 0x52)"); return -1; }
  int len = (int)(c - s);
  if (s[0] >= '0' && s[0] <= '9') snprintf(T->dev, sizeof T->dev, "/dev/i2c-%.*s", len, s);
  else snprintf(T->dev, sizeof T->dev, "%.*s", len, s);
  T->addr = (int)a;
  return 0;
}

static uint64_t mono_ns(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec; }

static void fail(tof250_pi *T, double t, const char *what) {
  T->fails++; T->range = 0; T->t = t; T->phase = 0; T->next = t + 1.0;   /* not answering: try again in a second */
  snprintf(T->why, sizeof T->why, "%s: %s", what, strerror(errno));
  if (!T->said) { T->said = 1; printf("rangefinder (TOF250 at 0x%02x on %s): %s\n", T->addr, T->dev, T->why); fflush(stdout); }
}

void tof250_poll(tof250_pi *T, double t) {
  if (!T->dev[0]) return;
  if (T->fd < 0) {
    if (t < T->t_try) return;
    T->t_try = t + 1.0;
    T->fd = open(T->dev, O_RDWR);
    if (T->fd < 0) { fail(T, t, "can't open the bus"); return; }
    if (ioctl(T->fd, I2C_SLAVE, T->addr) < 0) { fail(T, t, "can't address it"); close(T->fd); T->fd = -1; return; }
  }
  if (T->phase == 0) {
    if (t < T->next) return;
    const unsigned char reg = 0;                                    /* the distance register */
    if (write(T->fd, &reg, 1) != 1) { fail(T, t, "no answer"); return; }
    T->ask_ns = mono_ns();
    T->phase = 1; T->read_at = t + 0.002; T->next = t + 0.1;       /* (the datasheet's preparation time; its 10 Hz) */
    return;
  }
  if (t < T->read_at) return;
  unsigned char d[2];
  if (read(T->fd, d, 2) != 2) { fail(T, t, "no answer"); return; }
  T->phase = 0; T->range = lb_tof250_metres(d); T->t = t; T->n++; T->raw = d[0] << 8 | d[1]; T->read_ns = mono_ns();
  if (T->range < 0) T->n_far++;
  if (T->said) { T->said = 0; printf("rangefinder: answering again (%.2f m)\n", (double)T->range); fflush(stdout); }
}

int tof250_latest(const tof250_pi *T, double t, double max_age, float *range, float *age) {
  if (!T->n || T->range == 0 || t - T->t > max_age) return 0;
  *range = T->range; *age = (float)(t - T->t) + 0.05f;              /* (the module measures over its 100 ms) */
  return 1;
}

void tof250_said(const tof250_pi *T, double t, char *out, int n) {
  if (!T->dev[0]) { snprintf(out, (size_t)n, "rangefinder: none (--range tof250:/dev/i2c-1:0x52)"); return; }
  if (!T->n || t - T->t > 1) { snprintf(out, (size_t)n, "rangefinder (TOF250 at 0x%02x on %s): not answering%s%s", T->addr, T->dev, T->why[0] ? ": " : "", T->why); return; }
  if (T->range < 0) snprintf(out, (size_t)n, "rangefinder: out of its reach (no return; 2.5 m at most), %ld readings, %ld out of reach", T->n, T->n_far);
  else snprintf(out, (size_t)n, "rangefinder: %.2f m, %.2f s ago; %ld readings, %ld out of reach", (double)T->range, t - T->t, T->n, T->n_far);
}
