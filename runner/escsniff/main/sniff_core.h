/*
 * ESC signal sniffer: what a captured motor line carries. Plain C, no ESP-IDF (tools/test_escsniff.c runs it on the host).
 *
 * A capture is every change of up to SNIFF_CH input lines: the time (in ticks, CPU cycles on the chip) and all lines'
 * levels after it. sniff_analyze() reads one line out of it and names what it looks like: no pulses, servo-style PWM,
 * OneShot, duty PWM, DShot (decoded, CRC checked), or UART serial (bytes decoded), with the numbers behind the guess.
 */
#ifndef SNIFF_CORE_H
#define SNIFF_CORE_H
#include <stdint.h>

#define SNIFF_CH 4

typedef struct {
  const uint32_t *t;     /* tick of each change (wraps; only differences from t0 are used) */
  const uint8_t *lv;     /* bit c: line c's level after that change */
  int n;                 /* changes */
  uint8_t lv0;           /* levels at the start */
  uint32_t t0, t_end;    /* the window */
  float tick_per_us;
} sniff_cap;

enum { SN_STEADY, SN_FEW, SN_SERVO, SN_ONESHOT125, SN_ONESHOT42, SN_SHORT, SN_DUTY, SN_PERIODIC, SN_DSHOT, SN_SERIAL, SN_IRREGULAR };

typedef struct {
  int kind;
  int level;                    /* SN_STEADY: the level; otherwise the starting level */
  int edges, pulses;            /* edges on this line; complete high pulses */
  float window_us;
  float period_us, hz;          /* median high-to-high period */
  float high_us, high_min, high_max, duty;
  float shortest_us;            /* shortest complete high or low stretch */
  /* DShot */
  int dshot_rate, dshot_inverted, frames, bad_crc, throttle, thr_min, thr_max, telem_req;
  float frame_hz;
  /* serial */
  int pwm_uart_ambiguous;        /* the pulse train also decodes as a repeated UART byte (reported as a note) */
  int baud, nbytes, bad_bytes;
  uint8_t bytes[16];
} sniff_res;

/* scratch: room for c->n + 2 values each */
void sniff_analyze(const sniff_cap *c, int ch, uint32_t *scratch_a, uint32_t *scratch_b, sniff_res *r);
/* one line, no newline: the reading and the guess */
int sniff_line(const sniff_res *r, char *buf, int size);
const char *sniff_kind_name(int kind);
#endif
