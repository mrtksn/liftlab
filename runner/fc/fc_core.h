/*
 * LiftLab flight controller: the flight code around the formulas.
 *
 * Portable C, no hardware: the ESP32 firmware (fc/esp32) feeds it IMU samples and commands and drives the ESCs
 * and servos with what it returns, and the simulator flies the same code built to WebAssembly ("firmware in the
 * loop"), so what flies in the simulator is what flies on the drone. The formulas themselves run on the step
 * runner (rn_host), so they can be edited and reloaded in flight as before.
 *
 * What it does each control step (the simulator's controlStep, js/sim.js, in angle mode):
 *   attitude estimate (attitudeEstimator) from the gyro and accelerometer, in body axes;
 *   the servos' believed angles (servoPredictor, one memory per joint);
 *   the pilot's command → the force wanted: lean angles set where the thrust points, the throttle stick asks
 *     for climb or sink speed with a barometer (0.5 holds the height), else vertical acceleration around hover
 *     (0.5 keeps the vertical speed); either way the accelerometer trims the thrust until the drone accelerates
 *     as asked, so the model's weight needn't be exact;
 *   thrustAxisTarget, the attitude wanted, attitudeError, attitudeControl, forceDemand;
 *   allocation in two stages as in the simulator: the steering servos' angle changes, then every motor's thrust
 *     (allocationPreferences, allocation), thrustLinearization and voltageCompensation → throttles 0–1.
 * It holds no position itself: the navigation task (nav_core.c) does, with GPS or optical flow, and sends
 * guided commands (an acceleration and a heading) in place of the sticks.
 *
 * Safety:
 *   - it arms only when asked, with the airframe loaded, a gyro, the drone level (< 15°), the throttle stick
 *     low and the formulas loaded;
 *   - no command for FC_CMD_TIMEOUT s while armed: at idle it disarms; otherwise failsafe. It levels and descends
 *     at about 1 m/s. With a barometer it disarms once it asks to sink but the height stays put for 1.5 s (landed);
 *     the barometer also measures the accelerometer's bias, so the speed is right. Without one it always asks for a
 *     little downward acceleration on the thrust it learned hovers (drag sets the speed), disarms after the bump of
 *     touching down, and in any case after 120 s;
 *   - IMU data missing for 0.2 s while flying: motors off (there is nothing to fly on); a sample or two missed
 *     changes nothing (the attitude counts as settled until 50 ms pass without one);
 *   - tilted past FC_CRASH_DEG while flying (failsafe included), or the formulas failing: motors off (crashed);
 *   - arming needs the arm switch seen off first, so nothing re-arms by itself after a disarm;
 *   - disarmed, the motors get throttle 0 (the ESC's minimum pulse); a motor test spins one motor at a set
 *     throttle (at most 0.3) only while disarmed, for FC_TEST_S s from when it starts.
 */
#ifndef FC_CORE_H
#define FC_CORE_H
#include <stdint.h>
#include <stdarg.h>
#include "rn_host.h"
#include "bus.h"

#define FC_MAX_MOTORS 12
#define FC_MAX_JOINTS 8
#define FC_MAX_CHAIN 2          /* joints a motor may ride on */
#define FC_MAX_BASIS 9          /* 3^FC_MAX_CHAIN */
#define FC_CMD_TIMEOUT 0.5f
#define FC_CRASH_DEG 75.0f
#define FC_TEST_S 3.0f
#define FC_IDLE 0.06f           /* throttle while armed with the stick at the bottom */

/* The airframe, from the simulator (Airframe tab → Export for the flight controller): the controller's model
 * of the drone, as it flies on it in the simulator. */
typedef struct {
  char name[16];
  int n_chain, chain[FC_MAX_CHAIN];      /* joints it rides on, nearest first */
  int n_basis; float cols[FC_MAX_BASIS][6];   /* its effect per full thrust, per basis term of the joint angles */
  float bend, lag, power;                /* throttle curve bend, motor lag [s], power at full thrust [W] */
} fc_motor;
typedef struct {
  char name[16];
  int steer;                             /* the allocation moves it (else it holds its set angle) */
  float manual, range, rate, lag;        /* set angle, ±range [rad], speed [rad/s], lag [s] */
} fc_joint;
typedef struct {
  int n_motors, n_joints, mode;          /* mode: 0 tilt body, 1 mixed, 2 stay level */
  float m, J[9], Jinv[9], axis[3];       /* the model the controller computes with, and the nominal thrust axis */
  float imu_R[9];                        /* IMU → body rotation */
  float allowance, efficiency, servo_move, horizon;   /* allocation preferences */
  float lean_max;                        /* most lean [deg] */
  float mix_share;                       /* mixed steering: the servos' share of the sideways force */
  fc_motor mot[FC_MAX_MOTORS];
  fc_joint jnt[FC_MAX_JOINTS];
} fc_airframe;

typedef struct {
  float gyro[3], acc[3]; int have_gyro;  /* IMU sensor axes, rad/s and m/s² (the accelerometer reads +g up when level) */
  float baro_alt; int have_baro;         /* barometer height [m] from any reference, if there is one */
  float mag[3]; int have_mag;            /* compass, body axes (any scale), if there is one: it holds the heading */
} fc_imu;
typedef struct {
  int arm;                               /* 1 arm, 0 disarm */
  float roll, pitch, yaw, throttle;      /* sticks: roll/pitch −1…1 (right, forward), yaw rate −1…1 (left turn +), throttle 0…1 */
  int test_motor; float test_throttle;   /* motor test (disarmed only): index, or −1 */
  /* Guided (from the navigation task, nav_core.c, on this board or the Pi): instead of the sticks, the world
   * acceleration wanted [m/s², x north/forward at take-off, y left, z up; gravity not included] and the heading
   * [rad]. The throttle still says whether to fly: below 0.05 the motors idle (on the ground, before take-off). */
  int guided; float acc[3], heading;
} fc_cmd;
typedef struct { float motor[FC_MAX_MOTORS]; float servo[FC_MAX_JOINTS]; } fc_out;   /* throttles 0–1, servo angles [rad] */

/* From the learning task (learn_core.h), over the link or on the same board. Each is a flat list of floats, the
 * same on the link (rn_link.h) and in the simulator:
 *   EXC    excitation: mode (0 none, 1 added to what the controller asks, 2 open loop: these are the throttles and
 *          servo angles), hold motors (1: the motors keep the thrust they had when the hold began, plus mval),
 *          hold servos (likewise), servo mask (bit j: sval[j] applies), nm, nj, mval[nm], sval[nj].
 *          Then a pulse number and a rate limit [rad/s] (open loop): when the number changes the flight core notes the
 *          body rates, and once they have changed by more than the limit (after 12 ms) it cuts the motors to 0 until
 *          the next number, so a pulse stops at once rather than a link's round trip later.
 *          Mode 3 is a small attitude-reference test: the final two values are axis (0–2) and angle [rad],
 *          bounded to ±4°. Motors remain under closed-loop control; hold flags and motor/servo values are ignored.
 *          It lapses FC_EXC_TIMEOUT after the last one (the Pi stopped): the controller simply flies on.
 *   MODEL  the model it flies on: use learned (1) or the airframe description (0), hold servos (1: fly as a plain
 *          multirotor with the steering servos at rest, until they are measured), nm, nj; per motor its number of
 *          basis terms and its learned columns (acceleration per full thrust); per joint its speed and lag from the
 *          servo tests (0: the description's); per motor the throttle-curve bend to linearize with (−1: the
 *          description's).
 * From the health supervisor (super_core.h):
 *   SET    mode (0 normal, 1 careful, 2 return home, 3 land; in the air it only steps up, disarmed it is as sent,
 *          and a disarm ends it), lean limit [deg], acceleration limit [m/s²], speed limit [m/s] (0: none; the
 *          navigation's), nm, nj; per motor on (0/1), effectiveness (its columns ×), throttle ceiling; per joint out of
 *          the steering (0/1) and the angle it is really at [rad].
 * To both (fc_ltel), 200 times a second: LTEL  t (the time modulo FC_LTEL_WRAP: a float keeps that exact for its 5 ms
 *          steps, where the time since power-on would be rounded to a millisecond after a few hours; the reader unwraps
 *          it), state, flags (1 flying, 2 open loop, 4 on the learned model,
 *          8 motors held), attitude q (4), specific force (3) and body rates (3) averaged since the last one (body
 *          axes), battery volts, height and vertical speed (barometer and accelerometer), have height, nm, nj,
 *          throttles sent u[nm], believed thrusts v[nm], servo commands[nj], servo
 *          angles believed[nj]; then how many single control steps follow, each (dt, specific force (3), body rates
 *          (3), believed thrusts v[nm]): every step's, while in open loop (the throw's fit needs them all), else none.
 *          Flag 16: the open-loop pulse was cut (below). */
#define FC_EXC_TIMEOUT 0.1f
#define FC_LTEL_WRAP 256.0              /* [s] */
#define FC_SUB 8                        /* IMU samples an LTEL frame carries in open loop */
#define FC_LTEL_MAX (19 + 2 * FC_MAX_MOTORS + 2 * FC_MAX_JOINTS + 1 + FC_SUB * (7 + FC_MAX_MOTORS))
#define FC_LT_N 19                      /* where the per-motor values start in LTEL */
#define FC_MODEL_MAX (4 + FC_MAX_MOTORS * (1 + 6 * FC_MAX_BASIS) + 2 * FC_MAX_JOINTS + FC_MAX_MOTORS)

enum { FC_DISARMED = 0, FC_ARMED, FC_FAILSAFE, FC_CRASHED, FC_TESTING };

typedef struct {
  fc_airframe A; int have_airframe;
  rn_host *H; int f_att, f_srv, f_ta, f_err, f_ctl, f_fd, f_pref, f_alloc, f_lin, f_vc; int sizes_ok;
  int state; char why[64];
  double t, cmd_t;                       /* time since start, when the last command came [s] (double: exact for years) */
  fc_cmd cmd;
  float q[4], R[9], w[3]; int att_ok; float att_t;
  float yaw_sp, iAtt[3];
  float payload[6];                     /* known external load: world force [N], body torque [N m]; zero by default */
  double fs_t, test_t;                   /* when the failsafe or the motor test began */
  float fs_land_t, err_t, fs_vz, fs_alt_ref;   /* how long it has looked landed; how long formulas have failed; failsafe speed, height */
  double fs_alt_t, fs_bump_t, calm_t;   /* failsafe landing: when the height last moved; the touchdown bump; last calm hover */
  float iAz_calm;                        /* the thrust trim while hovering calmly */
  int arm_released, test_released;       /* the arm switch / motor test seen off since the last arming / test */
  int batt_wired;                        /* a battery sense wire is configured: arming needs a plausible reading */
  fc_out last_out;
  float az_f, az_bias, az_b, iAz, vz_i;  /* measured vertical acceleration (filtered), its bias on the ground and (from the
                                          * barometer) in flight, thrust trim [m/s²], vertical speed from it alone [m/s] */
  float imu_gap;                         /* time without IMU data */
  float alt_e, vz_e, alt_hold, baro_gap; int have_alt, holding;   /* barometer: height, vertical speed, height held */
  float rho;                             /* mixed steering: how much of what the servos were asked for they made */
  float th_cmd[FC_MAX_JOINTS], th_hat[FC_MAX_JOINTS];
  float v[FC_MAX_MOTORS];                /* believed thrust fraction each motor was last asked for */
  float vbatt, vref;                     /* battery voltage (0: not measured), the voltage the tables are for */
  int trap;                              /* last formula error */
  float tau_des[3];                      /* the torque the attitude control last asked for (body) [N·m] */
  uint32_t steps;
  /* the learning task's excitation and model (see EXC, MODEL above) */
  int exc_mode, exc_hold_m, exc_hold_s, exc_smask; float exc_m[FC_MAX_MOTORS], exc_s[FC_MAX_JOINTS]; double exc_t;
  int exc_axis; float exc_angle;
  int held_m, held_s; float hold_v[FC_MAX_MOTORS], hold_th[FC_MAX_JOINTS];
  int open_loop; double recover_t;       /* open loop now; until when it is catching itself (no tilt check) */
  int pulse_id, pulse_cut; float pulse_dw, pulse_w0[3]; double pulse_t;
  int sub_n; float sub[FC_SUB][7 + FC_MAX_MOTORS], lt_vprev[FC_MAX_MOTORS];   /* open loop: each step's sample for LTEL */
  int use_learned, hold_servos; float lcols[FC_MAX_MOTORS][FC_MAX_BASIS][6], laxis[3];
  float j_rate[FC_MAX_JOINTS], j_lag[FC_MAX_JOINTS], m_bend[FC_MAX_MOTORS];   /* from the actuator tests (0 / −1: the description's) */
  /* the supervisor's settings (see SET above) */
  int sup_mode, sup_landing; float lim_lean, lim_accel;
  int m_on[FC_MAX_MOTORS]; float m_eff[FC_MAX_MOTORS], m_cap[FC_MAX_MOTORS];
  int j_off[FC_MAX_JOINTS]; float j_ang[FC_MAX_JOINTS];
  /* what goes to the learning and the supervisor (LTEL): sums since the last frame */
  float lt_f[3], lt_w[3], lt_u[FC_MAX_MOTORS], lt_v[FC_MAX_MOTORS], lt_tc[FC_MAX_JOINTS]; int lt_n;
  float lt_alpha[3], test_alpha[3];      /* commanded angular acceleration, averaged with the LTEL gyro */
  float fb[3], gb[3]; int have_imu;      /* this step's accelerometer and gyro, body axes */
  bus *bus; int bt[7];                   /* the board's data bus, if it has one (fc_bus_attach), and this code's topics on it */
} fc_state;

/* Parse an airframe blob. Returns 0 or −1 with F->why set. */
int fc_airframe_load(fc_state *F, const uint8_t *blob, uint32_t len);
/* Set up: the host runs the formulas. Checks every formula takes what this code passes (the signatures), and
 * gives servoPredictor a memory per joint. Call after rn_host_init, before any program is staged. */
int fc_init(fc_state *F, rn_host *H);
/* Publish on the board's data bus (bus.h, docs/topic-bus.md): fc.state, fc.attitude, fc.imu, fc.height, fc.output,
 * fc.torque after each step, cmd.pilot with each command taken. Without a bus (B NULL) nothing is published.
 * Returns 0, or −1 if the bus has no room for them. */
int fc_bus_attach(fc_state *F, bus *B);
/* A command from the pilot (the Pi link now, a radio receiver later). Values are clamped; one that isn't finite is
 * ignored. Arming needs the arm switch seen off since the last disarm. */
void fc_command(fc_state *F, const fc_cmd *c);
/* The link is alive but can't pass commands for a moment (a program arriving): keep flying on the last command.
 * Only while armed: it never takes the drone out of its failsafe. */
void fc_keepalive(fc_state *F);
/* One control step: IMU sample in (dt since the last), outputs out. vbatt: battery volts, 0 if not measured; it is
 * used only when it fits the pack (0.6–1.35 × vref). If a formula fails while flying, the last outputs are held for
 * 50 ms, then the motors stop (crashed). */
void fc_step(fc_state *F, const fc_imu *imu, float dt, float vbatt, fc_out *out);
const char *fc_state_name(int s);
/* The learning and supervisor frames (above). Each returns 0, or −1 if the frame doesn't fit this airframe. */
int fc_exc(fc_state *F, const float *p, int n);
int fc_payload(fc_state *F, const float *p, int n); /* simulator support input; no physical transport */
int fc_model(fc_state *F, const float *p, int n);
int fc_set(fc_state *F, const float *p, int n);
/* The LTEL frame since the last call (out: FC_LTEL_MAX floats). Returns its length. */
int fc_ltel(fc_state *F, float *out);
/* Companion tuning sample, after fc_ltel: matching timestamp and averaged commanded angular acceleration.
 * Together with LTEL gyro it identifies actuator dynamics inside the closed loop. Does not change LTEL. */
int fc_tuning_sample(const fc_state *F, float *out);
/* The reader's side: an LTEL frame's time, unwrapped, from the last one (got: there was one). */
static inline double fc_ltel_unwrap(double last, int got, double raw) {
  if (!got) return raw;
  double d = raw - (last - FC_LTEL_WRAP * (double)(long long)(last / FC_LTEL_WRAP));
  return last + (d < -FC_LTEL_WRAP / 2 ? d + FC_LTEL_WRAP : d);
}
/* learn.js basisVals / dBasisVals: products of (1, cos θ, sin θ) over k joint angles (dm ≥ 0: the derivative with
 * respect to joint dm). Returns 3^k. For the Pi's tasks too. */
int fc_basis(float *v, const float *ang, int k, int dm);
/* The nominal thrust axis it flies on (body), from the description or the learned model. */
const float *fc_axis(const fc_state *F);
/* Text without printf (%s, %d, %.Nf), for status lines on boards with no C library: fc_fmt writes, fc_fmt_add appends. */
void fc_fmt(char *o, int size, const char *fmt, ...);
void fc_fmt_add(char *o, int size, const char *fmt, ...);
void fc_vfmt(char *o, int size, int *at, const char *fmt, va_list ap);

#endif
