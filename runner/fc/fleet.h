/* The fleet program: each drone's own program for working with the others (js/laws.js fleetProgram, compiled into
 * the navigation's program like the other flight formulas, and edited the same way). It runs 10 times a second
 * beside the navigation, on what this drone knows of itself and what it hears from the others over its peer link
 * (peer.h), and can publish numbers, send messages and, while the pilot has engaged it, say where the drone flies.
 *
 * Two halves, because the peer link is on the board with the radio (the flight controller: ESP-NOW is the ESP32's)
 * and the program beside the navigation (on the Pi, or the same board):
 *
 *   fleet_link (the peer end's board): what the drone publishes is the flight core's three (state, battery %,
 *     height: FLEET_HEAD), then the navigation's (FLEET_EXT: where it is in the fleet's frame, its velocity and
 *     heading, flags), then its program's (FLEET_VALS). Ten times a second it packs the peer table and the
 *     program's messages that came (fleet_link_pack) for the navigation's board, and takes back what the program
 *     publishes and sends (fleet_link_apply). On one board these are calls; between two, frames on their link
 *     (rn_link.h RN_LINK_PEER, RN_LINK_PEER_OUT).
 *
 *   fleet_core (the navigation's board): the table and messages in (fleet_peers), the program run on them every
 *     0.1 s (fleet_step), what it publishes and sends out (fleet_out). Engaged (fleet_engage: the pilot's FLEET
 *     command, rc_core.h; a button or a text command) it says where to fly: the target goes to the navigation as
 *     the pilot's would, within the same box (25 m from home, 0.5–15 m up), and the pilot's sticks, hold, home, a
 *     go-to, the link lost or the fly switch take it back (rc_core.c ends it; the caller tells fleet_engage).
 *
 * The fleet's frame: positions are shared only when a drone's navigation has GPS (its estimate is then in the GPS
 * frame, the same for every drone whose GPS has the same origin: in the simulator the world's; on a Pi, dfb_pi
 * --fleet-origin LAT,LON). The program sees the others' positions in its own frame (from its home), and only when
 * both drones have it; without, they can still talk and publish.
 *
 * Node numbers: the program sees each drone's top 24 bits (its MAC address's last three: a float holds them exactly);
 * fleet_core maps them back.
 * Messages: up to FLEET_VALS numbers, tagged FLEET_MSG_TAG on the air (peer.h messages are bytes); to node 0: every
 * drone connected. No C library. */
#ifndef FLEET_H
#define FLEET_H
#include <stdint.h>
#include "peer.h"
#include "nav_core.h"

#define FLEET_N 8                   /* drones the program sees (js/rn-sigs.js RN_FLEET_N) */
#define FLEET_VALS 8                /* numbers it publishes, or sends in a message (RN_FLEET_VALS) */
#define FLEET_MSG 4                 /* messages a call, each way (RN_FLEET_MSG) */
#define FLEET_HEAD 3                /* the flight core's: state, battery %, height */
#define FLEET_EXT 8                 /* the navigation's: position (the fleet's frame) ×3, velocity ×3, heading, flags */
#define FLEET_PUB (FLEET_HEAD + FLEET_EXT + FLEET_VALS)
#define FLEET_F_SHARED 1            /* flags: its position is in the fleet's frame */
#define FLEET_F_ENGAGED 2           /*        the program flies it */
#define FLEET_F_FLYING 4            /*        in the air */
#define FLEET_MSG_TAG 0x46          /* 'F': a program's message on the air: the tag, n, n floats */
#define FLEET_HZ 10.0
#define FLEET_STALE_S 1.0           /* the navigation's part, the table: older than this, not used */
#define FLEET_BOX_XY 25.0f
#define FLEET_ZLO 0.5f
#define FLEET_ZHI 15.0f
#define FLEET_VMAX 3.0f             /* the target's velocity fed forward, at most [m/s] each way */
/* RN_LINK_PEER: version, my id lo, hi, my state, battery, slots, messages; each slot: id lo, hi, state, lq, its lq of
 * us, since heard, its values' age (−1 none), n, n values; each message: from lo, hi, n, n values */
#define FLEET_PACK_MAX (7 + PEER_MAX * (8 + PEER_VALS) + FLEET_MSG * (3 + FLEET_VALS))
/* RN_LINK_PEER_OUT: version, n, n values (the navigation's and the program's), messages; each: to lo, hi, n, values */
#define FLEET_OUT_MAX (3 + FLEET_EXT + FLEET_VALS + 2 * FLEET_MSG * (3 + FLEET_VALS))
#if FLEET_PUB > PEER_VALS
#error "peer.h PEER_VALS must hold what the fleet publishes"
#endif

/* ── the peer end's board ── */
typedef struct { float ext[FLEET_EXT + FLEET_VALS]; int n_ext; double t_ext; } fleet_link;
void fleet_link_init(fleet_link *K);
/* What this drone publishes now: head (the flight core's), then the navigation's last (within FLEET_STALE_S). */
void fleet_link_publish(fleet_link *K, peer_net *N, const float head[FLEET_HEAD], double t);
/* The table and the program's messages that came (taken from N), for the navigation's board: its length. */
int fleet_link_pack(peer_net *N, double t, const float head[FLEET_HEAD], float *out);
/* What the navigation's board sends back: kept to publish; its messages queued (peer_send). Messages sent, −1 bad. */
int fleet_link_apply(fleet_link *K, peer_net *N, const float *in, int n, double t);

/* ── the navigation's board ── */
typedef struct {
  rn_host *H; int f, ok;
  float pk[FLEET_PACK_MAX]; int pk_n; double t_pk;   /* the table, as last sent */
  struct { uint32_t from; int n; float v[FLEET_VALS]; } in[2 * FLEET_MSG]; int in_n;   /* messages for the next call */
  int engaged;
  float go_p[3], go_v[3], go_h;                     /* where it flies while engaged (from home) */
  double t_run, t_last;
  float pub[FLEET_VALS]; int npub;
  float ext[FLEET_EXT];
  struct { uint32_t to; int n; float v[FLEET_VALS]; } out[2 * FLEET_MSG]; int out_n;
  int out_new;                                      /* something to send since fleet_out */
  uint32_t calls, fails, sent, got, unknown_to;
  char msg[64]; int said;                           /* what to tell the pilot (said: new) */
} fleet_state;

/* On a host with the navigation's program: 0, or −1 when it has no fleetProgram (or not the one this code passes
 * to): the fleet is off, the rest flies as ever. */
int fleet_init(fleet_state *F, rn_host *H);
/* A table came (RN_LINK_PEER, or fleet_link_pack on the same board). */
void fleet_peers(fleet_state *F, const float *p, int n, double t);
/* Engage (1) or not (0). Engaging needs the program, the drone flying with a home, and the table: else it says why
 * and stays off. o: where the navigation is (engaged, it holds there until the program says otherwise). 0 or −1. */
int fleet_engage(fleet_state *F, int on, const nav_state *N, const nav_out *o, const char *why);
/* At the navigation's rate, after its step (o: its output): the program, every 1/FLEET_HZ s. Returns 1 while engaged:
 * then go_p, go_v, go_h are the target. A program that fails (a trap, a number that isn't one) disengages. */
int fleet_step(fleet_state *F, const nav_state *N, const nav_out *o, double t);
/* While engaged: the target into sp (the navigation's set point), as the pilot's would be. */
void fleet_sp(const fleet_state *F, nav_sp *sp);
/* For the peer end: what the program publishes, and its messages (RN_LINK_PEER_OUT). Its length; 0 nothing new. */
int fleet_out(fleet_state *F, float *out);
#endif
