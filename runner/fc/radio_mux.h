/* Two radio links at once, as one (../radio_io.h): what the stack writes goes on each link that carries that way,
 * what comes is merged by lmux.h. The drone's ESP32, the command module's, dfb_pi and dfb_ground open their links as
 * usual and, with a second one set (radio2=), wrap the two in this; the program reads and writes it as it would one.
 *
 * A one-way link beside a two-way packet link is tied to it: it listens only to the sender the two-way link knows
 * (the program's links share one session number at each end), and a sending end says in its packets what its
 * program hears of the other end, so both ends' link statistics show the link as a whole (plink.h plink_tie). */
#ifndef RADIO_MUX_H
#define RADIO_MUX_H
#include "radio_io.h"
#include "radio_link.h"
#include "lmux.h"

/* a and b with their links as set (La, Lb), for this end (LMUX_GROUND, LMUX_DRONE); now() the time [s]. 0: no
 * memory (the two are left as they were). */
radio_io *radio_mux_open(radio_io *a, const rlink_cfg *La, radio_io *b, const rlink_cfg *Lb, int role, double (*now)(void));
int radio_mux_is(const radio_io *R);
/* The link followed now (lmux_followed): 0 or 1, −1 none. */
int radio_mux_followed(radio_io *R);
/* The link the telemetry goes by now (for its room, on the drone): the first that carries it down and reports (a
 * receiver's link statistics: it hears the ground), else the first that carries it down. */
const rlink_cfg *radio_mux_link(radio_io *R);
/* The wrapper freed (its two links are the caller's to close). */
void radio_mux_free(radio_io *R);
/* Each link: its radio_io (i 0 or 1), or 0. */
radio_io *radio_mux_part(radio_io *R, int i);
/* A line about both: "link 1 (espnow,6): followed; link 2 (serial,57600,down): frames 1234 …". */
void radio_mux_counts(radio_io *R, char *out, int n);
#endif
