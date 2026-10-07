/* Two radio links at once: the frames each brings, merged for the stack (the drone's rc_core/tlm_core, the command
 * module's ground_core), so it sees one link. Any two kinds (radio_link.h): ExpressLRS beside ESP-NOW, an nRF24L01
 * beside Bluetooth LE, a laser that carries the channels up beside a radio modem that brings the telemetry down.
 * What the stack writes goes on every link that carries that way (radio_mux.h does that); what comes is merged here:
 *   - the channels from the first link (in the order set: the first is the main one) that brought some within
 *     LMUX_RC_HOLD seconds, so a second link fills in only while the first has none (its channels would be older or
 *     newer by its own delay: taking both in turns would jitter the sticks);
 *   - commands and messages (what plink and the modules send reliably) once: a frame the same as one the other link
 *     brought within LMUX_DUP_S, not yet matched, is its copy (each copy matches one: the same message said twice
 *     comes twice; a command carries its own number, so two commands are never the same frame);
 *   - the rest (the telemetry) from the first link that is bringing it: one that brought some within 0.3 s;
 *   - the link statistics: one frame ten times a second, the followed link's (the one the channels or the telemetry
 *     come by), with the link quality each way the best of the links that carry that way, so the failsafe, the
 *     telemetry's room and the command module's alerts see the link as a whole: up while either carries.
 * No C library: it also builds for the simulator. */
#ifndef LMUX_H
#define LMUX_H
#include <stdint.h>
#include "crsf.h"

#define LMUX_MAX 2
#define LMUX_SEEN 32             /* reliable frames remembered, to drop the other link's copy */
#define LMUX_DUP_S 60.0          /* [s] (a link back after a cut brings what it held) */
#define LMUX_RC_HOLD 0.1         /* [s] */
#define LMUX_OUT 2048
enum { LMUX_GROUND = 0, LMUX_DRONE = 1 };   /* (as plink's roles) */

typedef struct {
  int role, n;
  struct {
    int up, down;              /* does it carry the channels and commands up, the telemetry down */
    double t_rc, t_tlm, t_stats;
    crsf_link S;               /* its last link statistics */
    crsf_parser P;             /* its bytes, into frames */
    uint32_t frames, taken;
  } k[LMUX_MAX];
  struct { uint8_t f[CRSF_MAX_FRAME]; int n, from, matched; double t; } seen[LMUX_SEEN]; int seen_i;
  uint8_t out[LMUX_OUT]; int out_n;
  double t_stats;
  int followed;                /* the link followed when the statistics last went: −1 none */
  uint32_t dups, switches;
} lmux;

/* n links; up[i], down[i]: which ways link i carries (radio_link.h rlink_up, rlink_down). */
void lmux_init(lmux *M, int role, int n, const int *up, const int *down);
/* What link i has for the stack (any split of its frames). */
void lmux_from_link(lmux *M, int i, const uint8_t *b, int n, double t);
/* What the stack should read now: bytes, ≤ cap. */
int lmux_to_stack(lmux *M, double t, uint8_t *b, int cap);
/* The link followed now (the drone: the one its channels come by; the command module: its telemetry): −1 none. */
int lmux_followed(const lmux *M, double t);
/* The best link quality each way among the links reporting [%], and the signal this end hears on the best (the
 * drone: of the uplink; the command module: of the downlink) [dBm]: 1, or 0 if none reports. */
int lmux_lq(const lmux *M, double t, int *up, int *down, int *rssi);
#endif
