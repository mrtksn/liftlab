/*
 * The command module's text commands, shared by dfb_ground.c (a Mac or a Pi: the terminal and UDP) and the ESP32
 * command module (its USB serial port): what a script or a person types to fly. One command per line:
 *   press NAME, release NAME, tap NAME     a button: right left fwd back up down yawr yawl arm fly hold home
 *                                          gentle normal sport cal
 *   stick roll|pitch|throttle|yaw V        an analog stick, −1…1 (off: back to its buttons)
 *   goto X Y Z [HEADING]                   fly to X north, Y west, Z up [m] from home (each within ±327 m: what the
 *                                          command carries), facing HEADING [°]
 *   calibrate                              the learning's hover calibration
 *   fleet on|off                           let the drone's fleet program fly it (fc/fleet.h), or take it back
 *   latch N|all open|close|toggle          the cargo task's latches (N from 1): drop what one holds, or grab
 *   pickup X Y Z [LATCH]                   fly the hook onto a thing whose top is at X Y Z [m] from home and close
 *                                          the latch (1 by default; open it first): gnd_pickup, gnd_config.hook
 *   cmd ID V1 V2 …                         any command (rc_core.h): ID 1–255, up to 6 values
 * A value that isn't a number (or is out of range) is refused with a reply saying why; nothing is sent.
 *   status, messages, quit
 * Sticks and the stick buttons set this way lapse a second after they were last sent, like a radio's channels: a
 * script keeps sending them while it holds them, so if it stops, the sticks centre. Hold, home and cal are momentary
 * (as on the drone): a press is a tap. The arm, fly and speed switches stay as set. A latching button (gnd_config.latch:
 * a push button as the arm or fly switch) is set by press and release whatever the button last did, so a script can
 * always disarm; tap toggles it, as a push would. Arm and fly reply with their state.
 * Not in the simulator's build (it has no C library); the simulator's own keys go straight to ground_core.
 */
#ifndef GROUND_TEXT_H
#define GROUND_TEXT_H
#include "ground_core.h"

extern const char *const gnd_button_names[GB_N];
int gnd_button(const char *name);                    /* GB_… or −1 */

typedef struct {
  uint32_t held;                                     /* switches and buttons pressed until released */
  double until[GB_N];                                /* buttons held until then (taps, lapsing presses, terminal keys) */
  float axis[GND_AXES]; uint32_t has; double axis_until[GND_AXES];
} gnd_text_in;

/* A line. Returns 1 if it was a command here (the reply, maybe empty, in reply), 0 if not; 2 for "quit". */
int gnd_text(gnd_state *G, gnd_text_in *I, char *line, double t, char *reply, int rn);
/* What the text commands hold now, added to in. */
void gnd_text_inputs(const gnd_text_in *I, double t, gnd_input *in);
/* A one-line summary: telemetry, link, flight mode, battery, height, the alert. */
void gnd_status(const gnd_state *G, double t, char *out, int n);

#endif
