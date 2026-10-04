/*
 * Pickup: fly a latch onto something on the ground and grab it, by itself. Part of the navigation's pilot (the
 * radio's, rc_core.c; or the text commands' and the simulator's, without a radio).
 *
 * The drone doesn't know where things are, or where its hook is: the one who asks does. A PICKUP command (rc_core.h
 * RC_CMD_PICKUP, or pickup_start) says where the hub must be for the hook to reach the thing (from home), which way
 * to face for that, and which latch to close. In the simulator that comes from the nearest loose thing and the
 * hook's place on the airframe; on the drone, from the pilot (dfb_ground or dfb_pi: "pickup X Y Z", the hook's
 * offset set there).
 *
 * Then it flies (the latch should already be open; this never opens one):
 *   1. over: to that spot, PK_ABOVE higher (at least as high as it is), facing that way;
 *   2. down: slowly (PK_SINK), so it doesn't dip under the spot near the ground and clip a prop; it may go below
 *      the pilot's floor (rc_core.c BOX_ZLO) for this, down to the spot;
 *   3. still: it waits until it holds the spot within PK_TOL, nearly still, for PK_STILL s;
 *   4. close: it asks the cargo task to close the latch (pickup_req: whichever board runs it), and waits for it;
 *   5. up: back to the height of step 1. Done.
 * Any stick, hold or home, a new go-to, or the link lost, ends it where it is (the caller cancels it). Not still
 * over the spot within PK_GIVEUP s of getting there: it gives up and climbs back, without closing anything.
 */
#ifndef PICKUP_CORE_H
#define PICKUP_CORE_H
#include <stdint.h>
#include "nav_core.h"

#define PK_ABOVE 0.8f                  /* [m] how high over the spot it comes in */
#define PK_SINK 0.35f                  /* [m/s] how fast it comes down onto it */
#define PK_RISE 0.8f                   /* [m/s] and goes back up */
#define PK_TOL 0.05f                   /* [m] how close to the spot it must hold */
#define PK_STILL 0.6f                  /* [s] for this long */
#define PK_GIVEUP 25.0f                /* [s] */
#define PK_CLOSE_S 0.8f                /* [s] the latch's time to close, and its switch to settle */
#define PK_ZMIN (-5.0f)                /* [m] the lowest spot taken (from home) */

enum { PK_IDLE = 0, PK_OVER, PK_DOWN, PK_CLOSE, PK_UP };
typedef struct {
  int phase, latch;
  float spot[3], heading, z_over;      /* where the hub goes to grab, from home; facing; how high it comes in */
  float tgt[3];                        /* the target it flies now */
  double t_phase, t_still;
  uint32_t nreq; int req_latch, req_act;   /* the latest request to the cargo task (nreq counts them) */
  int done_ok;                         /* the last pickup went through to the end (it closed the latch) */
  char msg[64]; int said;              /* what to tell the pilot (said: new) */
} pickup_state;

void pickup_init(pickup_state *K);
/* Start one. spot: hub position, from home [m]; heading [rad]; latch 0…; o: where the navigation is now. Returns 0,
 * or −1 (said in msg) for a spot that isn't a number or is out of reach of the box (25 m, PK_ZMIN…15 m). */
int pickup_start(pickup_state *K, const float spot[3], float heading, int latch, const nav_out *o, double t);
void pickup_cancel(pickup_state *K, const char *why);
static inline int pickup_active(const pickup_state *K) { return K->phase != PK_IDLE; }
/* At the navigation's rate, while active: the target to fly (into sp: target, vref, heading). Returns 1 while active. */
int pickup_step(pickup_state *K, const nav_out *o, double t, float dt, nav_sp *sp);

#endif
