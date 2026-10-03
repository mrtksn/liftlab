/*
 * The command module: the pilot's side of the radio. The same C runs on an ESP32 with buttons and sticks wired to it
 * (runner/ground/esp32), on a Raspberry Pi or a Mac driving an ExpressLRS transmitter module over a serial port
 * (runner/ground/dfb_ground.c), and in the simulator, built to WebAssembly, on the far side of the simulated link.
 *
 * Each step it takes what the pilot does (gnd_input: analog sticks, buttons, keys) and turns it into what goes up
 * the radio, as CRSF frames for the transmitter module:
 *   - 16 channels (rc_core.h's order), every step. The sticks go through the stickInput formula (deadband and expo
 *     for a real stick, easing in for buttons); switches can be latching (a push button that toggles);
 *   - ground-station commands (go to, calibrate…), queued and sent one at a time, a little apart, so the drone (which
 *     keeps the latest) acts on each.
 * It sends intent, not control: what a stick position makes the drone do, its limits and its failsafes, are the
 * drone's. If the link drops, the drone's rules take over, whatever is on this side.
 *
 * What the transmitter module hands back (the drone's telemetry, and the link statistics the module makes) is
 * decoded into a view (gnd_view): what the Ground station shows, and what the groundAlerts formula warns about.
 *
 * Formulas (js/laws.js, run by the step runner like the drone's): stickInput, groundAlerts. Built-in program:
 * rn_builtin_ground.c (node tools/export_program.js --tasks ground --c runner/ground/rn_builtin_ground.c). If a
 * formula fails and even the built-in program can't answer, the raw inputs go up unshaped: the pilot keeps control.
 */
#ifndef GROUND_CORE_H
#define GROUND_CORE_H
#include <stdint.h>
#include "rn_host.h"
#include "crsf.h"
#include "tlm_core.h"

enum { GND_ROLL = 0, GND_PITCH, GND_THR, GND_YAW, GND_AXES };
/* Buttons (bit numbers in gnd_input.held). The stick buttons move a stick (right/left roll, forward/back pitch,
 * up/down throttle, turn right/left yaw); the others are switches and actions. */
enum {
  GB_RIGHT = 0, GB_LEFT, GB_FWD, GB_BACK, GB_UP, GB_DOWN, GB_YAWR, GB_YAWL,
  GB_ARM, GB_FLY, GB_HOLD, GB_HOME, GB_GENTLE, GB_NORMAL, GB_SPORT, GB_CAL,
  GB_N
};
#define GB(b) (1u << (b))

typedef struct {
  float axis[GND_AXES];      /* analog sticks, −1…1: roll right, pitch forward, throttle up, yaw right are + */
  uint32_t has_axis;         /* bit per axis that has an analog stick (the others come from buttons) */
  uint32_t held;             /* buttons held now (GB bits) */
} gnd_input;

typedef struct {
  uint32_t latch;            /* buttons that toggle on each press (push buttons used as the arm and fly switches) */
  float rc_period;           /* seconds between channel frames (0.004: 250 per second) */
  float cmd_gap;             /* seconds between commands (0.15): the drone keeps only the latest one */
} gnd_config;
void gnd_config_default(gnd_config *c);

#define GND_QN 8             /* commands waiting */
#define GND_MSGS 16          /* messages kept */
enum { GND_WHY_N = 10 };
extern const char *const gnd_why_text[GND_WHY_N];   /* groundAlerts' reasons, as words */

/* The drone as the telemetry shows it. t_*: when each last came (−1: never). */
typedef struct {
  crsf_link link; double t_link;                       /* what the transmitter module reports */
  float roll, pitch, yaw; double t_att;
  float volts, amps, mah; int pct; double t_batt;
  double lat, lon; float speed, course, alt; int sats; double t_gps;
  float baro_alt, vz; double t_baro;
  char mode[16]; double t_mode;
  struct { int n; float v[TLM_NV]; double t; } item[TLM_ITEMS];   /* the drone's own items (tlm_core.h), unscaled */
  struct { int sev; char s[TLM_TEXT + 1]; double t; } msg[GND_MSGS]; uint32_t nmsg;   /* a ring; nmsg counts them all */
  double t_any; uint32_t bytes, frames, bad;               /* the drone's telemetry: bytes and frames; frames dropped */
} gnd_view;

typedef struct {
  rn_host *H; int ok, f_stick, f_alert, shaped;
  gnd_config C;
  uint32_t held_was, latched;
  int level;                                           /* 0 gentle, 1 normal, 2 sport */
  float stick[GND_AXES], ch[16];
  struct { int cmd, n; float v[6]; } q[GND_QN]; int qh, qn, seq; double t_cmd, t_rc;
  int alert, alert_why; double t_alert_step;
  crsf_parser P; gnd_view V;
  char why[96];
} gnd_state;

/* Set up with a step runner holding the ground program. Returns 0, or −1 (why says why) if its formulas aren't
 * what this code expects: then it still sends the raw sticks. */
int gnd_init(gnd_state *G, rn_host *H, const gnd_config *c);
/* One step: the pilot's inputs → channels; writes into out the frames due now (channels every rc_period, a queued
 * command every cmd_gap) and returns their bytes. */
int gnd_step(gnd_state *G, const gnd_input *in, double t, float dt, uint8_t *out, int cap);
/* Bytes from the transmitter module: the drone's telemetry and the link statistics. */
void gnd_from_radio(gnd_state *G, const uint8_t *b, int n, double t);
/* Queue a command (rc_core.h RC_CMD_*). Returns 0, or −1 when the queue is full. */
int gnd_command(gnd_state *G, int cmd, const float *v, int n);
int gnd_goto(gnd_state *G, float x, float y, float z, float heading);
/* The latest alert: level 0 fine, 1 warning, 2 alarm; why (gnd_why_text). */
static inline int gnd_alert(const gnd_state *G, int *why) { if (why) *why = G->alert_why; return G->alert; }
/* The view packed as floats for a display (the simulator's Ground station): see ground_core.c gnd_view_pack. */
int gnd_view_pack(const gnd_state *G, double t, float *out, int cap);

#endif
