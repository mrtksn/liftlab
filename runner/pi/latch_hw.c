/* The latches' outputs on a Raspberry Pi (dfb_pi --latch), for the cargo task (fc/cargo_core.h):
 *   pwmN    a servo on hardware PWM channel N of /sys/class/pwm/pwmchip0 (GPIO 18 or 12 for channel 0, 19 or 13 for 1,
 *           with dtoverlay=pwm-2chan in config.txt), 50 Hz, the closed and open pulses from --latch-us
 *   gpioN   an on/off line, line N of /dev/gpiochip0 (the GPIO number), driven high for closed: the driver of a
 *           solenoid's or an electromagnet's latch
 *   dry     drives nothing (to try the commands without the hardware)
 * Linux only: the PWM by sysfs, the line by the kernel's GPIO character device. */
#define _DEFAULT_SOURCE
#include "latch_hw.h"
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <linux/gpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

static int put(const char *path, const char *val) {
  int fd = open(path, O_WRONLY); if (fd < 0) return -errno;
  ssize_t n = write(fd, val, strlen(val)); int e = n < 0 ? -errno : 0; close(fd); return e;
}

int latch_hw_open(latch_out *o, const char *spec, int closed, int us_closed, int us_open, char *err, int errn) {
  memset(o, 0, sizeof *o); o->fd = -1; o->us_closed = us_closed; o->us_open = us_open;
  char *end; long n;
  if (!strcmp(spec, "dry")) { o->kind = LATCH_DRY; return 0; }
  if (!strncmp(spec, "pwm", 3)) {
    n = strtol(spec + 3, &end, 10); if (end == spec + 3 || *end || n < 0 || n > 15) { snprintf(err, errn, "%s: pwm and a channel number (pwm0)", spec); return -1; }
    o->kind = LATCH_PWM; o->ch = (int)n;
    char p[96], v[24];
    snprintf(v, sizeof v, "%d", o->ch); int e = put("/sys/class/pwm/pwmchip0/export", v);
    if (e && e != -EBUSY) { snprintf(err, errn, "%s: no PWM channel %d (%s); is dtoverlay=pwm-2chan in config.txt?", spec, o->ch, strerror(-e)); return -1; }
    snprintf(p, sizeof p, "/sys/class/pwm/pwmchip0/pwm%d/period", o->ch);
    for (int k = 0; k < 20 && (e = put(p, "20000000")) == -EACCES; k++) usleep(50000);   /* (udev sets the permissions a moment after the export) */
    if (e) { snprintf(err, errn, "%s: %s: %s", spec, p, strerror(-e)); return -1; }
    latch_hw_set(o, closed);
    snprintf(p, sizeof p, "/sys/class/pwm/pwmchip0/pwm%d/enable", o->ch);
    if ((e = put(p, "1"))) { snprintf(err, errn, "%s: %s: %s", spec, p, strerror(-e)); return -1; }
    return 0;
  }
  if (!strncmp(spec, "gpio", 4)) {
    n = strtol(spec + 4, &end, 10); if (end == spec + 4 || *end || n < 0 || n > 63) { snprintf(err, errn, "%s: gpio and a line number (gpio17)", spec); return -1; }
    o->kind = LATCH_GPIO; o->ch = (int)n;
    int chip = open("/dev/gpiochip0", O_RDWR);
    if (chip < 0) { snprintf(err, errn, "%s: /dev/gpiochip0: %s", spec, strerror(errno)); return -1; }
    struct gpiohandle_request rq; memset(&rq, 0, sizeof rq);
    rq.lineoffsets[0] = (uint32_t)o->ch; rq.lines = 1; rq.flags = GPIOHANDLE_REQUEST_OUTPUT; rq.default_values[0] = (uint8_t)(closed ? 1 : 0);
    snprintf(rq.consumer_label, sizeof rq.consumer_label, "dfb_pi latch");
    int r = ioctl(chip, GPIO_GET_LINEHANDLE_IOCTL, &rq); int e = errno; close(chip);
    if (r < 0) { snprintf(err, errn, "%s: line %d: %s", spec, o->ch, strerror(e)); return -1; }
    o->fd = rq.fd; o->last = closed;
    return 0;
  }
  snprintf(err, errn, "%s: pwmN, gpioN or dry", spec); return -1;
}

void latch_hw_set(latch_out *o, int closed) {
  o->last = closed;
  if (o->kind == LATCH_PWM) {
    char p[96], v[24]; snprintf(p, sizeof p, "/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", o->ch);
    snprintf(v, sizeof v, "%d", (closed ? o->us_closed : o->us_open) * 1000);
    if (put(p, v)) perror(p);
  } else if (o->kind == LATCH_GPIO && o->fd >= 0) {
    struct gpiohandle_data d; memset(&d, 0, sizeof d); d.values[0] = (uint8_t)(closed ? 1 : 0);
    if (ioctl(o->fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &d) < 0) perror("latch line");
  }
}

const char *latch_hw_name(const latch_out *o) {
  static char b[4][24]; static int k; char *s = b[k++ & 3];
  if (o->kind == LATCH_PWM) snprintf(s, 24, "PWM %d", o->ch); else if (o->kind == LATCH_GPIO) snprintf(s, 24, "GPIO %d", o->ch); else snprintf(s, 24, "no output");
  return s;
}
