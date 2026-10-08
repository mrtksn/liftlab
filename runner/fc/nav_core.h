/*
 * LiftLab navigation task: holds and moves the drone's position, on top of the flight core.
 *
 * Portable C, like fc_core.c. It runs on the Raspberry Pi (pi/dfb_pi.c, talking to the ESP32 over the serial link) or
 * on the flight controller itself (beside fc_core, without a link), and the simulator flies the same code built to
 * WebAssembly. It only ever talks to the flight core through guided commands (fc_cmd.guided): the acceleration
 * wanted and the heading. The flight core still does everything fast and everything about safety.
 *
 * Each step (about 100 Hz):
 *   the flight core's attitude, body rates and accelerometer (on the Pi: from its telemetry, a few ms old);
 *   the position references that are there: GPS (antenna position fix), optical flow with its rangefinder,
 *     the barometer. Each reading says how old it is; positionEstimator compares it with its own estimate from
 *     that moment. A sensor away from the hub is corrected for where it sits (nav_config);
 *   positionEstimator → position and velocity, relative to home (where it started);
 *   positionControl → the acceleration toward the target, with an integral that trims out steady pushes (wind, a
 *     hover thrust the model doesn't know exactly).
 * Position is relative to home: where it took off (set when it is first told to fly). x and y lie along the
 * world's axes as the flight core's attitude estimate has them.
 */
#ifndef NAV_CORE_H
#define NAV_CORE_H
#include <stdint.h>
#include "rn_host.h"
#include "bus.h"

/* What the navigation needs to know about the drone (exported by the simulator with the airframe). */
typedef struct {
  float m;                               /* mass the controller flies with [kg] */
  float baro_pos[3], fix_pos[3];         /* where the barometer and the GPS antenna sit, from the hub, body axes [m] */
  float flow_pos[3], flow_R[9];          /* the flow camera: where, and its mount (sensor → body) */
  float speed_max;                       /* the fastest it is asked to fly [m/s] */
  int refs;                              /* the references fitted: 1 barometer, 2 GPS, 4 optical flow */
} nav_config;

typedef struct {
  float q[4], w[3], acc[3];              /* flight core: attitude (body → world), body rates [rad/s], specific force (body) [m/s²] */
  int have_att;
  int have_baro; float baro_alt, baro_age;
  int have_fix; float fix_p[3], fix_v[3], fix_age;               /* GPS: position [m] in its own frame, velocity [m/s] */
  int have_flow; float flow[2], range, flow_q, flow_age;          /* flow [rad/s], range [m], tracking quality 0–1 */
} nav_in;

typedef struct {
  float target[3];                       /* where to be, relative to home [m] */
  float vref[3];                         /* velocity to move the target at (fed forward) [m/s] */
  float heading;                         /* [rad] */
  int fly;                               /* 0: stay on the ground (idle; turned to 0 in the air, it lands where it is
                                          * first); 1: fly to the target */
} nav_sp;

typedef struct {
  float acc[3], heading; int fly;        /* the guided command for the flight core */
  float p[3], v[3];                      /* the estimate, relative to home */
  int have_home;
  int ready;                             /* the estimate has settled on its references: it may take off */
  int landed;                            /* the supervisor had it land, and it is down: it stays on the ground */
} nav_out;

typedef struct {
  rn_host *H; int f_pe, f_fv, f_pc, ok;
  nav_config C; int have_config;
  float home[3]; int have_home, home_from_fix;
  float iPos[3];
  int test_axis; float test_offset, test_left;
  float p[3], v[3];
  float pa[3]; int have_pa;              /* the estimate as it is (not from home): in the GPS frame once GPS is heard (fleet.h) */
  int seen;                              /* references heard from since start (bits as nav_config.refs) */
  float t_est, t_wait, t_still;                   /* how long the estimator has run; how long it has waited for its references */
  /* the health supervisor's mode and limits (fc_core.h SET): it flies home, or lands, by itself */
  int sup_mode; float lim_speed, lim_accel, lim_lean;
  int rc_rth;                            /* the pilot's radio link is lost in flight: it flies home and lands (rc_core.h) */
  int fly_land;                          /* "fly" went off in the air: it lands where it is, then idles */
  int auto_on, auto_land, landed; float auto_t[3], auto_v[3], land_t;
  nav_out last;                          /* the last step's output (held while a step can't be made) */
  char why[64];
  uint32_t steps;
  bus *bus; int bt[3];                   /* the board's data bus, if it has one (nav_bus_attach), and this code's topics */
} nav_state;

/* A nav_config blob: magic 'DFNC', version 1, the fields above as floats (refs too), CRC32. Returns 0 or −1. */
int nav_config_load(nav_state *N, const uint8_t *blob, uint32_t len);
/* Set up on a host that has the flight program. Checks the formulas' sizes. */
int nav_init(nav_state *N, rn_host *H);
/* One step of dt seconds. Returns 0; 1 if there was nothing to step on (no attitude, or no time since the last step):
 * out is then the last step's output, and the caller sends no new command (the flight core flies on the last one for
 * a moment, and a gap that lasts ends in its failsafe); or −1 if a formula failed (then out->fly is 0: the caller
 * stops sending commands, and the flight core's failsafe lands it). */
int nav_step(nav_state *N, const nav_in *in, const nav_sp *sp, float dt, nav_out *out);
/* Bounded reference excitation for measured position tuning; expires after 100 ms without a refresh. */
int nav_test_target(nav_state *N, int axis, float offset);
/* Publish on the board's data bus (docs/topic-bus.md) after each step: nav.estimate, nav.setpoint, nav.command.
 * Without a bus (B NULL) nothing is published. 0, or −1 if the bus has no room. */
int nav_bus_attach(nav_state *N, bus *B);
/* The supervisor's settings (a SET frame): mode 2 flies home at its speed limit and lands, 3 lands where it is. In the
 * air the mode only steps up; on the ground it is as sent. */
void nav_set(nav_state *N, const float *p, int n);
/* It landed by itself (nav_out.landed) and has been disarmed: it may fly again. The supervisor's mode goes with it
 * (its next SET raises it again if it still holds). */
static inline void nav_land_reset(nav_state *N) {
  N->landed = 0; N->auto_on = 0; N->auto_land = 0; N->land_t = 0; N->rc_rth = 0; N->fly_land = 0; N->sup_mode = 0;
  for (int i = 0; i < 3; i++) N->iPos[i] = 0;
}

#endif
