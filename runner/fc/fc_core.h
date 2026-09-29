/*
 * Drone Force Bench flight controller: the flight code around the formulas.
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
 * There is no horizontal position hold: that needs GPS or optical flow, which come later.
 *
 * Safety:
 *   - it arms only when asked, with the airframe loaded, a gyro, the drone level (< 15°), the throttle stick
 *     low and the formulas loaded;
 *   - no command for FC_CMD_TIMEOUT s while armed: failsafe. It levels and descends at about 1 m/s, and
 *     disarms when the accelerometer feels the ground stop it (or after FC_FAILSAFE_S s). Without a barometer
 *     the descent speed comes from the accelerometer alone and is rough (it may even hover or climb slowly);
 *   - IMU data missing for 0.2 s while flying: motors off (there is nothing to fly on);
 *   - tilted past FC_CRASH_DEG while armed: disarms (crashed);
 *   - disarmed, the motors get throttle 0 (the ESC's minimum pulse); a motor test spins one motor at a set
 *     throttle only while disarmed, for FC_TEST_S s at most.
 */
#ifndef FC_CORE_H
#define FC_CORE_H
#include <stdint.h>
#include "rn_host.h"

#define FC_MAX_MOTORS 12
#define FC_MAX_JOINTS 8
#define FC_MAX_CHAIN 2          /* joints a motor may ride on */
#define FC_MAX_BASIS 9          /* 3^FC_MAX_CHAIN */
#define FC_CMD_TIMEOUT 0.5f
#define FC_FAILSAFE_S 30.0f
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
} fc_imu;
typedef struct {
  int arm;                               /* 1 arm, 0 disarm */
  float roll, pitch, yaw, throttle;      /* sticks: roll/pitch −1…1 (right, forward), yaw rate −1…1 (left turn +), throttle 0…1 */
  int test_motor; float test_throttle;   /* motor test (disarmed only): index, or −1 */
} fc_cmd;
typedef struct { float motor[FC_MAX_MOTORS]; float servo[FC_MAX_JOINTS]; } fc_out;   /* throttles 0–1, servo angles [rad] */

enum { FC_DISARMED = 0, FC_ARMED, FC_FAILSAFE, FC_CRASHED, FC_TESTING };

typedef struct {
  fc_airframe A; int have_airframe;
  rn_host *H; int f_att, f_srv, f_ta, f_err, f_ctl, f_fd, f_pref, f_alloc, f_lin, f_vc; int sizes_ok;
  int state; char why[64];
  float t, cmd_t;                        /* time, when the last command came */
  fc_cmd cmd;
  float q[4], R[9], w[3]; int att_ok; float att_t;
  float yaw_sp, iAtt[3], fs_t, test_t;
  float az_f, az_bias, iAz, vz_i;                       /* measured vertical acceleration (filtered), thrust trim [m/s²], vertical speed [m/s] */
  float fs_vz, fs_vmin, imu_gap;         /* failsafe: vertical speed change since it began, its lowest; time without IMU data */
  float alt_e, vz_e, alt_hold, baro_gap; int have_alt, holding;   /* barometer: height, vertical speed, height held */
  float rho;                             /* mixed steering: how much of what the servos were asked for they made */
  float th_cmd[FC_MAX_JOINTS], th_hat[FC_MAX_JOINTS];
  float v[FC_MAX_MOTORS];                /* believed thrust fraction each motor was last asked for */
  float vbatt, vref;                     /* battery voltage (0: not measured), the voltage the tables are for */
  int trap;                              /* last formula error */
  uint32_t steps;
} fc_state;

/* Parse an airframe blob. Returns 0 or −1 with F->why set. */
int fc_airframe_load(fc_state *F, const uint8_t *blob, uint32_t len);
/* Set up: the host runs the formulas. Checks every formula takes what this code passes (the signatures), and
 * gives servoPredictor a memory per joint. Call after rn_host_init, before any program is staged. */
int fc_init(fc_state *F, rn_host *H);
/* A command from the pilot (the Pi link now, a radio receiver later). now: time [s]. */
void fc_command(fc_state *F, const fc_cmd *c);
/* One control step: IMU sample in (dt since the last), outputs out. vbatt: battery volts, 0 if not measured. */
void fc_step(fc_state *F, const fc_imu *imu, float dt, float vbatt, fc_out *out);
const char *fc_state_name(int s);

#endif
