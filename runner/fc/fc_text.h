/*
 * The flight core's text commands: what the ground's command buttons (LiftLab's drone card) send up the radio for
 * it, on a board that runs the flight core itself (the simulator's boards: board_wasm.c). On the drone's ESP32 the
 * same lines are its settings lines (esp32/main/flight.c setting()), which the Pi passes on (dfb_pi: level, safety,
 * esp32 LINE); they answer the same way.
 *   level              as it sits now is level (acc: the IMU's reading, averaged while it stood still; still: it did)
 *   level none         the IMU's mount as the airframe says
 *   level R,P[,Y]      a trim: roll, pitch, yaw [deg] (fc_level_deg)
 *   level yaw Y        the yaw alone: the board turned about straight up
 *   safety off|on      no level check when arming and no crash cut-off (disarmed only), or back on; safety: which
 *   status             its state, attitude, level trim and safety
 * Each changes nothing while armed, but safety on and the questions. Returns 1 with reply filled, or 0: not one of
 * these (reply untouched).
 */
#ifndef FC_TEXT_H
#define FC_TEXT_H
#include "fc_core.h"

int fc_text(fc_state *F, const char *line, const float *acc, int still, char *reply, int cap);

#endif
