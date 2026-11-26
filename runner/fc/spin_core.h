/* The rope spin test: how quickly the motors answer, measured with the drone hanging on a rope (or a line), free to
 * turn about its yaw axis. Tune → Rope spin test in LiftLab; dfb_pi's "spintest"; the simulator's Pi board.
 *
 * Disarmed, through the flight core's motor test, every motor runs at a base throttle (below lift: the rope holds
 * the weight). Then the motors that turn one way step up and the others step down by the same amount: the thrust
 * stays, the props' drag torques no longer cancel, and the drone starts to turn. The steps alternate in doublets
 * (+T, −2T, +2T, ... , ±T) so the turn rate swings around zero and the rope ends about where it started.
 *
 * The flight core's telemetry to the Pi (RN_LINK_LTEL, 200 times a second) has, averaged over the same few
 * milliseconds, what each motor was sent and what the gyro read; so the timing comes from the drone's own clock,
 * whatever the link's delay. The fit: the yaw rate follows
 *     ω̇ = g·(r·a + (1 − r)·x) − k·θ − c·ω + b,   a = the differential throttle D(t − delay), x = a through a
 * first-order lag (the motors' spin-up). A motor's reaction torque jumps with its current the moment its throttle
 * steps, then settles to the drag at its new speed, which follows the same spin-up as its thrust: r says how big that
 * jump is against the steady change (0: none; a voltage-driven brushless motor's is several). g the yaw acceleration per unit of differential throttle, k and c the
 * rope's twist and the air's drag, b a bias; linear in g·r, g·(1 − r), k, c, b once delay and lag are chosen: a least-squares fit for each (delay, lag) on a grid, coarse then
 * fine, keeps the best. delay is everything before the thrust starts to change (the ESC, the gyro's filter, the
 * averaging), lag the motors' spin-up time constant. */
#ifndef SPIN_CORE_H
#define SPIN_CORE_H
#include "fc_core.h"

#define SPIN_MAX 2600          /* samples kept (13 s at 200 Hz) */
enum { SPIN_IDLE, SPIN_SETTLE, SPIN_STEP, SPIN_END, SPIN_DONE, SPIN_FAILED };

typedef struct {
  int phase, n, seg, n_seg, have_ltel;
  float dir[FC_MAX_MOTORS], base, step, half;  /* each motor's yaw sign (+1/−1, 0: no yaw effect) */
  double t0, seg_t, last_ltel, prev_raw;       /* (prev_raw: the last LTEL time, before unwrapping) */
  double tw;                                   /* the LTEL time unwrapped */
  int ns; float st[SPIN_MAX], sd[SPIN_MAX], sw[SPIN_MAX];   /* samples: time from the start, differential throttle, yaw rate */
  /* the result */
  float delay, lag, gain, kick, r2, spring, drag, peak_rate; int ok;   /* kick: the instant part of the yaw torque against the steady one */
  char msg[200];
} spin_state;

void spin_init(spin_state *S);
/* Start: n motors, yaw[i] each motor's yaw effect (its sign is what matters), base and step throttles, half the
 * doublet's period [s], now. 0, or −1 with S->msg saying why not. */
int spin_start(spin_state *S, int n, const float *yaw, float base, float step, float half, double t);
/* While it runs: the throttles to send now (out: n values, as a motor test) → n; 0 when not running. */
int spin_tick(spin_state *S, double t, float *out);
/* Every LTEL frame from the flight core (fc_ltel's layout), and when it came. */
void spin_ltel(spin_state *S, const float *lt, int n, double t);
void spin_stop(spin_state *S, const char *why);
/* The fit on the recorded samples (spin_tick calls it at the end; tests call it directly). 0 ok, −1 not. */
int spin_fit(spin_state *S);
/* What it says now (a line for the radio): "spintest: ..." */
void spin_said(const spin_state *S, char *out, int cap);

/* The rope lift test: how much the motors lift, with the drone hanging on the rope. While the rope is taut the
 * accelerometer reads 1 g whatever the motors do (the rope takes the difference); once their thrust passes the
 * weight the drone rises and the accelerometer reads thrust over mass. So: every motor at the same throttle, from
 * base, rising slowly (rate per second) until it lifts off the rope (the accelerometer 4% over its reading at rest,
 * or the height 8 cm up): that throttle holds the weight. Back to base at once; then short pulses (a quarter second) at
 * lift-off + step, + 2 step, ... (up to top), resting at base between: each reads thrust = mass × the accelerometer.
 * It stops when armed, swinging, turning, 40 cm up, or the telemetry stops. */
#define LIFT_PULSES 3
enum { LIFT_IDLE, LIFT_SETTLE, LIFT_RAMP, LIFT_REST, LIFT_PULSE, LIFT_DONE, LIFT_FAILED };
typedef struct {
  int phase, n, k, have_ltel, over;
  float base, top, rate, step, u;                 /* u: the throttle asked now */
  double t0, ph_t, last_ltel;
  double f0, a0; int n0;                          /* at rest: the accelerometer's reading [m/s²] and the height, summed */
  float alt_rest, g_rest, v_sum; int v_n;         /* their means; the pack's voltage while it lifted */
  float lift_u, lift_v;                           /* lift-off: the throttle (as the flight core sent it), the voltage */
  float pu[LIFT_PULSES], pf[LIFT_PULSES], pv[LIFT_PULSES]; int np;   /* each pulse: throttle, thrust / weight, voltage */
  double ps; int pn; float pvs;                   /* the pulse being read: sum of the accelerometer's ratio, count, volts */
  float climb;                                    /* the most it rose [m] */
  int ok; char msg[200];
} lift_state;
void lift_init(lift_state *L);
int lift_start(lift_state *L, int n, float base, float top, float rate, float step, double t);
int lift_tick(lift_state *L, double t, float *out);
void lift_ltel(lift_state *L, const float *lt, int n, double t);
void lift_stop(lift_state *L, const char *why);
/* "lifttest: ...; result LIFT_U VOLTS NP U1 F1 V1 U2 F2 V2 ..." (F: thrust / weight, V: the pack's volts) once it lifted. */
void lift_said(const lift_state *L, char *out, int cap);
#endif
