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
 *     keeps the latest) acts on each. A command goes instead of that beat's channels, never next to them: on a module
 *     bay's single wire the module answers each frame as soon as it has it, and would talk over a second one. And
 *     only while the link is up: a transmitter module drops what isn't channels while it has no link, so commands
 *     wait for it (and are dropped, said in why, if they waited more than GND_CMD_WAIT).
 * Switch warning (as EdgeTX's): arm and fly aren't sent on until each has been seen off since the start, so a switch
 * left on, a stuck button or a floating pin doesn't arm the drone when the link comes up (why: GND_WHY_SWITCH).
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
  uint32_t held;             /* buttons held now (GB bits): hardware buttons, which may latch (gnd_config.latch) */
  uint32_t sw;               /* switch states already decided (text commands, a key that toggles): never latched */
} gnd_input;

typedef struct {
  uint32_t latch;            /* buttons that toggle on each press (push buttons used as the arm and fly switches) */
  float rc_period;           /* seconds between channel frames (0.004: 250 per second) */
  float cmd_gap;             /* seconds between commands (0.15): the drone keeps only the latest one */
  uint8_t seq0;              /* the first command's sequence number − 1: something random, so a restarted command
                                module's first command isn't taken by the drone for a repeat of its last one */
  uint32_t resume;           /* after a restart in flight: the buttons that were on (gnd_state.on, kept by the caller
                                across it). Latching ones start latched on, and none is held back by the switch warning */
} gnd_config;
void gnd_config_default(gnd_config *c);

#define GND_QN 8             /* commands waiting */
#define GND_MSGS 16          /* messages kept */
#define GND_CMD_WAIT 10.0    /* [s] a command that waited this long for the link is dropped, not sent late */
enum { GND_WHY_SWITCH = 11, GND_WHY_N = 12 };
extern const char *const gnd_why_text[GND_WHY_N];   /* groundAlerts' reasons, and the switch warning's, as words */

/* The drone as the telemetry shows it. t_*: when each last came (−1: never). */
typedef struct {
  crsf_link link; double t_link; int linked;           /* what the transmitter module reports; linked: it has connected once */
  float roll, pitch, yaw; double t_att;
  float volts, amps, mah; int pct; double t_batt;
  double lat, lon; float speed, course, alt; int sats; double t_gps;
  float baro_alt, vz; double t_baro, t_vz;             /* (the climb comes in the barometer frame or a vario frame) */
  char mode[16]; double t_mode;
  struct { int n; float v[TLM_NV]; double t; } item[TLM_ITEMS];   /* the drone's own items (tlm_core.h), unscaled */
  struct { int sev; char s[TLM_TEXT + 1]; double t; } msg[GND_MSGS]; uint32_t nmsg;   /* a ring; nmsg counts them all */
  double t_any; uint32_t bytes, frames, bad;               /* the drone's telemetry: bytes and frames; frames dropped */
} gnd_view;

typedef struct {
  rn_host *H; int ok, f_stick, f_alert, shaped;
  gnd_config C;
  uint32_t held_was, all_was, latched, on;              /* on: the buttons sent on in the last step */
  uint32_t seen_off, blocked;                          /* the switch warning: arm, fly seen off since the start; held back now */
  int stepped, level;                                  /* level: 0 gentle, 1 normal, 2 sport */
  float stick[GND_AXES], ch[16];
  struct { int cmd, n; float v[6]; double t; } q[GND_QN]; int qh, qn, seq; double t_cmd, t_rc_next, t_now;
  uint32_t dropped;                                    /* commands dropped after waiting GND_CMD_WAIT for the link */
  int alert, alert_why, f_lvl, f_why; double t_alert_step;   /* f_: what groundAlerts said */
  crsf_parser P; gnd_view V;
  char why[96];
} gnd_state;

/* Set up with a step runner holding the ground program. Returns 0, or −1 (why says why) if its formulas aren't
 * what this code expects: then it still sends the raw sticks. */
int gnd_init(gnd_state *G, rn_host *H, const gnd_config *c);
/* One step: the pilot's inputs → channels; writes into out the frame due now (one a beat: the channels every
 * rc_period, or instead a queued command, cmd_gap apart, while the link is up) and returns its bytes. cap below
 * CRSF_MAX_FRAME: nothing goes, and the beat waits for a step that has room (a port still busy with the last one). */
int gnd_step(gnd_state *G, const gnd_input *in, double t, float dt, uint8_t *out, int cap);
/* Bytes from the transmitter module: the drone's telemetry and the link statistics. */
void gnd_from_radio(gnd_state *G, const uint8_t *b, int n, double t);
/* Queue a command (rc_core.h RC_CMD_*, 1–255; up to 6 values, each within what its 16 bits hold at rc_cmd_scale).
 * Returns 0, −1 when the queue is full, −2 when the command or a value can't go (out of range, not a number). */
int gnd_command(gnd_state *G, int cmd, const float *v, int n);
/* A go-to replaces one still waiting to go (only the newest target matters). x, y, z within ±GND_GOTO_MAX m (what
 * the command's centimetres hold), heading [rad] any finite angle. Returns as gnd_command. */
#define GND_GOTO_MAX 327.0f
int gnd_goto(gnd_state *G, float x, float y, float z, float heading);
/* Set a latching button (gnd_config.latch) on or off, as pressing it would toggle it: for a script or an emergency,
 * whatever the hardware button last did. Returns the state now (0, 1), or −1 if b doesn't latch. */
int gnd_latch(gnd_state *G, int b, int on);
/* The link as the transmitter module reports it: 1 up, 0 down (statistics older than a second, or no uplink), −1
 * no statistics ever (no module that sends them: taken as up). */
int gnd_link_up(const gnd_state *G, double t);
/* The latest alert: level 0 fine, 1 warning, 2 alarm; why (gnd_why_text). */
static inline int gnd_alert(const gnd_state *G, int *why) { if (why) *why = G->alert_why; return G->alert; }
/* The view packed as floats for a display (the simulator's Ground station): see ground_core.c gnd_view_pack. */
int gnd_view_pack(const gnd_state *G, double t, float *out, int cap);

#endif
