/* A packet link's end over a serial line, on Linux or a Mac (radio_io.h): see radio_pserial.c. */
#ifndef RADIO_PSERIAL_H
#define RADIO_PSERIAL_H
#include "radio_io.h"
#include "plink.h"
/* role: PLINK_DRONE (dfb_pi) or PLINK_GROUND (dfb_ground); L: the link (serial,BAUD[,half]); phrase: the binding
 * phrase (both ends the same). 0 if the port can't be opened (said on stderr). */
radio_io *radio_pserial_open(int role, const char *dev, const rlink_cfg *L, const char *phrase, const char *name);
void radio_pserial_close(radio_io *R);
int radio_pserial_is(const radio_io *R);
const plink *radio_pserial_plink(const radio_io *R);
/* One line of counts, for a status line: "serial line: packets sent 1200 …". */
void radio_pserial_counts(const radio_io *R, char *out, int n);
#endif
