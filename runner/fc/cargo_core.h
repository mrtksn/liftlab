/*
 * Cargo: the latches that hold what the drone carries (a hook, a gripper, an electromagnet) and let it go. Any board
 * can run it (the simulator's Cargo task; on a Pi, dfb_pi --latch): it takes the pilot's commands, drives each latch
 * open or closed, and reports what the latches and their load switches say.
 *
 * Commands come from:
 *   - the radio: a ground-station LATCH command (rc_core.h RC_CMD_LATCH: latch number 0… or −1 for all, action);
 *     on the receiver's board straight from its rc_input, on another board from the RN_LINK_RC frames it is sent;
 *   - a text command or a script on the board itself ("latch 1 open"), or another board (RN_LINK_CARGO).
 * Each latch is driven open or closed and takes its travel time to get there. A latch may have a load switch (a
 * microswitch in the hook): then the task knows whether something hangs from it, says when a release didn't drop
 * anything (stuck) or a close caught something, and the telemetry carries it.
 *
 * It never decides on its own to let go: only a command opens a latch. A board that restarts drives its latches to
 * how they were set up (closed, normally), so a reset in flight doesn't drop the load.
 */
#ifndef CARGO_CORE_H
#define CARGO_CORE_H
#include <stdint.h>

#define CG_MAX 8
enum { CG_OPEN = 0, CG_CLOSE = 1, CG_TOGGLE = 2 };
#define CG_ALL (-1)
#define CG_SETTLE 0.1f                /* after a move, how long until its load switch is believed [s] */

typedef struct {
  int n;                              /* latches */
  uint8_t closed[CG_MAX];             /* driven: 1 closed, 0 open */
  uint8_t loaded[CG_MAX];             /* what its load switch says (only with has_sw) */
  uint32_t has_sw;                    /* bit per latch with a load switch */
  float travel[CG_MAX];               /* seconds from open to closed */
  float moving[CG_MAX];               /* seconds left until it has got there and settled (travel + CG_SETTLE) */
  uint8_t was_loaded[CG_MAX];         /* the switch when the move started */
  uint32_t cmd_seen;                  /* the radio's last command, seen (rc_input.cmd_seq) */
  uint32_t ncmd;                      /* commands taken */
  char msg[64]; int said;             /* the latest thing to tell the pilot (said: new since the telemetry took it) */
  uint32_t nmsg;                      /* counts them (for a reader other than the telemetry) */
} cargo_state;

/* closed_mask: bit per latch closed at the start; travel: per latch [s] (0 or a null pointer: 0.15 s) */
void cargo_init(cargo_state *C, int n, uint32_t closed_mask, const float *travel);
/* latch: 0…n−1 or CG_ALL; action CG_OPEN, CG_CLOSE, CG_TOGGLE; from: who asked ("radio", "text"…), for the message.
 * Returns 0, −1 no such latch, −2 no such action. */
int cargo_command(cargo_state *C, int latch, int action, const char *from);
/* What the load switches say now: bit per latch loaded; sw: bit per latch that has a switch. */
void cargo_switches(cargo_state *C, uint32_t loaded, uint32_t sw);
/* Advance the latches' moves; says when a move finished and what the switch found. */
void cargo_step(cargo_state *C, float dt);
/* What to drive: bit per latch closed. */
uint32_t cargo_drive(const cargo_state *C);
/* For the telemetry (TLM_CARGO): per latch bits 1 closed, 2 loaded, 4 moving, 8 has a load switch. (Inline: the
 * telemetry's code reads it on every board, with or without the cargo task.) */
static inline int cargo_bits(const cargo_state *C, int i) {
  if (i < 0 || i >= C->n) return 0;
  return (C->closed[i] ? 1 : 0) | (C->loaded[i] ? 2 : 0) | (C->moving[i] > 0 ? 4 : 0) | (((C->has_sw >> i) & 1) ? 8 : 0);
}

/* A LATCH command that came up the radio: takes a new, fresh one (rc_core.h rc_input; RC_CMD_FRESH_S). Returns 1 if
 * it took one. (A separate header so a board without the radio's code can still run the cargo task.) */
struct rc_input;
int cargo_from_rc(cargo_state *C, const struct rc_input *in, double t);

#endif
