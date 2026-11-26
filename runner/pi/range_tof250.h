/*
 * A downward rangefinder on the Pi's I²C: the HCJ TOF250 (laser time of flight, ~5 cm to 2.5 m, 10 readings a
 * second). dfb_pi passes its distance to the navigation as the flow camera's rangefinder: the height above the
 * ground the flow is scaled by, and the height reference the barometer's offset is learned from (nav_core.h).
 *
 * Polled without waiting: each poll does at most one short I²C transfer (asking for the distance register, then, a
 * couple of milliseconds later, reading it), so the flight program's loop never stalls on it. A module that doesn't
 * answer is tried again once a second.
 *
 *   --range tof250:/dev/i2c-1:0x52     (the bus and the module's 7-bit address; tof250:1:0x52 is the same)
 */
#ifndef RANGE_TOF250_H
#define RANGE_TOF250_H

typedef struct {
  char dev[64]; int addr, fd;
  int phase;                             /* 0 ask for the distance next, 1 read it next */
  double next, read_at, t_try;
  float range; double t;                 /* the latest: > 0 metres, −1 out of reach (no return), 0 not answering; when */
  long n, n_far, fails; int said;
  char why[96];
} tof250_pi;

/* "tof250:/dev/i2c-1:0x52" (or "tof250:1:0x52"): 0, or −1 if it isn't one (why says). It opens on its first poll. */
int tof250_parse(tof250_pi *T, const char *spec);
/* Each pass of the program's loop, with the time now [s]. */
void tof250_poll(tof250_pi *T, double t);
/* The latest distance, if it is fresh (no older than max_age): 1 with range (> 0 m, or −1 out of reach) and its age. */
int tof250_latest(const tof250_pi *T, double t, double max_age, float *range, float *age);
/* How it is, for a status line. */
void tof250_said(const tof250_pi *T, double t, char *out, int n);

#endif
