/* This program's session number for its packet links: one random number for all of them (the other end tells our
 * restarts by it; with two links at once, a one-way link listens to the sender its two-way partner knows, by its
 * session: fc/radio_mux.h). */
#ifndef RADIO_SESSION_H
#define RADIO_SESSION_H
#include <stdint.h>
uint32_t radio_session(void);
#endif
