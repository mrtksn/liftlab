/* The latches' outputs on a Raspberry Pi: see latch_hw.c. */
#ifndef LATCH_HW_H
#define LATCH_HW_H
enum { LATCH_DRY = 0, LATCH_PWM, LATCH_GPIO };
typedef struct { int kind, ch, fd, us_closed, us_open, last; } latch_out;
/* spec: pwmN, gpioN or dry; closed: how to start it. Returns 0, or −1 with why in err. */
int latch_hw_open(latch_out *o, const char *spec, int closed, int us_closed, int us_open, char *err, int errn);
void latch_hw_set(latch_out *o, int closed);
const char *latch_hw_name(const latch_out *o);
#endif
