/*
 * The pilot's radio, as the drone sees it: the channels the receiver passes on (16, −1…1), the link statistics it
 * reports, and ground-station commands that came up the same link. Whatever the radio (rc_input is filled by a
 * transport: tlm_crsf.c for ExpressLRS), this turns them into what the flight code takes:
 *   - without navigation, the flight core's stick command (angle mode), as long as the channels keep coming; when
 *     they stop, nothing is sent, and the flight core's own failsafe levels and lands it;
 *   - with navigation, the target it flies to: the sticks move it (at up to the speed level's speed, ramped),
 *     switches arm it, take off or land, hold where it is and fly home, and a ground-station "go to" sets it. Lose
 *     the link in flight for a second and the navigation flies home and lands by itself; when the link comes back,
 *     it holds where it is and the pilot has it again.
 *
 * Channels (AETR, then switches; −1 off/low, +1 on/high):
 *   1 roll  2 pitch  3 throttle  4 yaw  5 arm  6 speed level (−1 gentle, 0 normal, +1 sport)  7 fly (take off / land)
 *   8 hold (momentary)  9 home (momentary)
 * With navigation the throttle stick is centred: up climbs, down sinks; without, it is the throttle.
 */
#ifndef RC_CORE_H
#define RC_CORE_H
#include <stdint.h>
#include "fc_core.h"
#include "nav_core.h"

enum { RC_ROLL = 0, RC_PITCH, RC_THR, RC_YAW, RC_ARM, RC_LEVEL, RC_FLY, RC_HOLD, RC_HOME };
/* Ground-station commands (CRSF 0x80/0xD1: command, sequence 1–255, up to 6 values as 16-bit integers × rc_cmd_scale):
 *   GOTO   x y z [m from home], heading [rad]: fly there
 *   LEARN  code: a learning command (learn_core.h: 1 calibrate, 2 stop, 3 fly on the description, 4 on the learned) */
enum { RC_CMD_GOTO = 1, RC_CMD_LEARN = 2 };
float rc_cmd_scale(int cmd, int k);
#define RC_LOST_S 1.0                     /* no channels for this long: the link is lost */

typedef struct rc_input {
  float ch[16]; double t_ch; uint32_t frames;          /* the channels, and when they last came */
  float up_rssi, up_lq, up_snr, down_rssi, down_lq; int rf_mode, tx_power; double t_link;
  int cmd; float cmd_v[6]; uint32_t cmd_seq;           /* the latest ground-station command (cmd_seq counts them) */
} rc_input;
static inline int rc_link_ok(const rc_input *in, double t) { return in->frames > 0 && t - in->t_ch < RC_LOST_S; }

/* Angle mode: the stick command. Returns 0 (and fills c) while the channels come, −1 when they don't. */
int rc_stick_cmd(const rc_input *in, double t, fc_cmd *c);

typedef struct {
  float target[3], vref[3], heading;     /* the navigation's target, from home [m]; its velocity; heading [rad] */
  int have_target, fly, arm, level;
  int hold_was, home_was; uint32_t cmd_seen;
  int learn_req;                          /* a LEARN command came: its code, for the learning (the board passes it on) */
  int lost;                               /* the link is lost (flying home if it was flying) */
  char msg[64]; int said;                 /* something to tell the pilot (said: new since last read) */
} rc_pilot;
void rc_pilot_init(rc_pilot *P);
/* At the navigation's rate: the pilot's set point from the radio. o: the navigation's last output. Returns whether
 * the pilot has it armed. */
int rc_pilot_step(rc_pilot *P, const rc_input *in, double t, nav_state *N, const nav_out *o, float dt, nav_sp *sp);

/* Between boards (RN_LINK_RC): the receiver's board sends what it got to the navigation's board, 50 times a second.
 * 16 channels, age of the channels [s], the link (up RSSI, LQ, SNR, down RSSI, LQ), the command (seq, cmd, 6 values). */
#define RC_PACK_N (16 + 1 + 5 + 8)
int rc_pack(const rc_input *in, double t, float *out);
void rc_unpack(rc_input *in, const float *p, int n, double t);

#endif
