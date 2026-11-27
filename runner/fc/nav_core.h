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
#include "bus.h"
#include "pid_tuning.h"
#include "rn_host.h"
#include "map_core.h"
#include <stdint.h>

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
  int have_flow; float flow[2], range, flow_q, flow_age;          /* flow [rad/s], range [m] (≤ 0: out of its reach: the
                                                                  * fused height scales the flow), tracking quality 0–1 */
} nav_in;

typedef struct {
  float target[3];                       /* where to be, relative to home [m] */
  float vref[3];                         /* velocity to move the target at (fed forward) [m/s] */
  float heading;                         /* [rad] */
  int fly;                               /* 0: stay on the ground (idle; turned to 0 in the air, it lands where it is
                                          * first); 1: fly to the target */
} nav_sp;

#define NAV_LIFTOFF_S 3.0f                /* lift-off: how long it may climb before the flow must speak [s] */
typedef struct {
  float acc[3], heading; int fly;        /* the guided command for the flight core */
  float p[3], v[3];                      /* the estimate, relative to home */
  int have_home;
  int ready;                             /* the estimate has settled on its references: it may take off */
  int landed;                            /* the supervisor had it land, and it is down: it stays on the ground */
  int pos_ok;                            /* it knows where it is across the ground: a horizontal reference (the flow's
                                          * speed, the GPS) spoke in the last 0.3 s, or it is on the ground, or lifting
                                          * off (its first NAV_LIFTOFF_S, below 0.3 m). Without one the estimate drifts */
  int guard;                             /* the map's guard this step: 0 nothing, 1 slowed (near something, or into the
                                          * unknown), 2 the target cut short (the caller moves its own target there) */
  float target[3];                       /* the target it flew to, relative to home (cut short by the guard) */
} nav_out;

/* The map and where the drone is in it (docs/map-navigation.md, map_core.h). The map's axes turned by psi are the
 * navigation's; a point of the navigation (its own estimate, pa) is at Rz(−psi)·pa + off in the map. The first fix
 * anchors off (so nothing jumps); after it the fixes correct the estimate (a fix in the estimator, without velocity:
 * the drift goes) and off stays, moved only to keep the drone's place when psi changes, or when the map jumps (over
 * 2 m: re-solved). Without fixes for NAV_FIX_STALE the guard stands down (where it is in the map is no longer known). */
#define NAV_FIX_STALE 10.0f              /* [s] */
#define NAV_GUARD_BRAKE 1.0f             /* [m/s²] the braking the guard counts on (well inside what it can do) */
#define NAV_DRIFT_SHARE 0.15f            /* the guard's margin grows by this share of the distance flown since a fix (up to 1 m) */
typedef struct {
  map_grid *M;                           /* the host's map (nav_map_attach), or NULL */
  float radius, v_unknown;               /* the guard: the drone's radius with its margin [m]; its speed near the unknown [m/s] */
  int aligned; float psi, off[3], t_fix, sigma;
  int fuse;                              /* the fixes correct the estimate (1, the default; map fuse off: they only place it) */
  int anchored;                          /* placed by hand (mapanchor): the guard doesn't stand down without fixes (the flow
                                          * carries where it is from there) */
  float yaw;                             /* the navigation's heading, the last step [rad] (for an anchor) */
  float odo;                             /* how far it has flown since the last fix [m]: its position in the map is that much
                                          * less sure (the flow's scale), so the guard's margin grows with it */
  float fix_a[3], fix_age;               /* the last fix in the navigation's own frame, and how old it was when it came */
  uint32_t n_fix;
  int guard_was;                         /* (for what it says when the guard starts and stops) */
} nav_mapstate;

typedef struct {
  rn_host *H; int f_pe, f_fv, f_pc, ok;
  nav_config C; int have_config;
  float home[3]; int have_home, home_from_fix;
  float iPos[3];
  pid_tuning tuning;
  int test_axis; float test_offset, test_left;
  float p[3], v[3];
  float pa[3]; int have_pa;              /* the estimate as it is (not from home): in the GPS frame once GPS is heard (fleet.h) */
  int seen;                              /* references heard from since start (bits as nav_config.refs) */
  float t_est, t_wait, t_still;                   /* how long the estimator has run; how long it has waited for its references */
  float t_noref;                         /* how long since a horizontal reference spoke (nav_out.pos_ok) */
  float t_fly;                           /* how long it has been flying (the last step's fly): its lift-off (nav_out.pos_ok) */
  /* the health supervisor's mode and limits (fc_core.h SET): it flies home, or lands, by itself */
  int sup_mode; float lim_speed, lim_accel, lim_lean;
  int rc_rth;                            /* the pilot's radio link is lost in flight: it flies home and lands (rc_core.h) */
  int fly_land;                          /* "fly" went off in the air: it lands where it is, then idles */
  int auto_on, auto_land, landed; float auto_t[3], auto_v[3], land_t;
  nav_out last;                          /* the last step's output (held while a step can't be made) */
  nav_mapstate map;                      /* the map, where it is in it, and the guard */
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
int nav_tune(nav_state *N, const float *p, int n);
/* Publish on the board's data bus (docs/topic-bus.md) after each step: nav.estimate, nav.setpoint, nav.command.
 * Without a bus (B NULL) nothing is published. 0, or −1 if the bus has no room. */
int nav_bus_attach(nav_state *N, bus *B);
/* The map (map_core.h) this host keeps for the navigation: attach once (radius 0.3 m, 0.3 m/s near the unknown by
 * default); a map file loaded into it with nav_map_load (grown by the guard's radius). */
void nav_map_attach(nav_state *N, map_grid *M);
int nav_map_load(nav_state *N, const uint8_t *buf, uint32_t len, char *reply, int rn);
/* A fix: where the hub was age seconds ago, in the map [m]; psi the turn from the map's axes to the navigation's [rad];
 * sigma how sure [m]. 0, or −1 if the estimate hasn't started (nothing to align). */
int nav_mapfix(nav_state *N, float age, const float p[3], float psi, float sigma);
/* Placed by hand: the hub is at p in the map, facing heading [rad, from the map's +X toward +Y]. An anchor doesn't go
 * stale (fixes, when they come, correct it). 0, or −1 before the estimate has started. */
int nav_mapanchor(nav_state *N, const float p[3], float heading);
/* The map's text commands (docs/map-navigation.md): mapfix T X Y Z PSI [SIGMA] (T on the clock `now` is on: the
 * host's: dfb_pi's CLOCK_MONOTONIC, the clock its camera's pictures carry; the simulator's board clock; T ≤ 0: −T
 * seconds ago), mapanchor X Y Z HEADING_DEG, mapguard RADIUS [V_UNKNOWN], map (how it is),
 * map clear, map fuse on|off. 1 if it was one (reply set), else 0. */
int nav_map_command(nav_state *N, const char *s, double now, char *reply, int rn);

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
