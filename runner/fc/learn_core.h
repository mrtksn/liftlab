/*
 * LiftLab learning task: learns what each motor and servo really does, on a Raspberry Pi.
 *
 * Portable C like fc_core.c and nav_core.c: the Pi runs it (runner/pi/dfb_pi.c) beside the navigation, talking to
 * the ESP32 over the serial link, and the simulator flies the same code built to WebAssembly. Its formulas
 * (identifyEffectiveness, identifyMotorResponse, identifyServoResponse, identifyThrow) run on the step runner, so
 * they can be edited and reloaded like the flight core's. It never touches the motors itself: it asks the flight
 * core for excitation (fc_core.h EXC) and tells it which model to fly on (MODEL). The flight core lets an
 * excitation lapse 0.1 s after the last one, so if this task stops, the drone just flies on.
 *
 * What it does, on the flight core's telemetry (LTEL, 200 times a second):
 *   in-flight learning: recursive least squares (identifyEffectiveness) on what each input does to the drone's
 *     acceleration and rotation, with a little excitation (dither) so there is always something to learn from;
 *   calibration (while hovering): settle; pulse each motor on its own while the others hold (6%, then 16%) and fit
 *     its lag and throttle curve (identifyMotorResponse); step each steering servo while everything else holds
 *     and fit its real speed and lag (identifyServoResponse); sweep the servos; excite everything together; then
 *     score the learned model and the description on fresh moves and fly on whichever explains them better;
 *   throw start: thrown with the motors off and knowing nothing about its geometry, it pulses each motor in free
 *     fall, fits its model (identifyThrow), sends it, and catches itself on it; then (optionally) calibrates.
 *
 * Inputs (as learn.js): a motor on the frame is one input, its thrust fraction; a motor on servo joints is several,
 * its thrust × each product of (1, cos θ, sin θ) over its joints (3 for one joint, 9 for two).
 */
#ifndef LEARN_CORE_H
#define LEARN_CORE_H
#include "fc_core.h"

#define LN_IN 24                         /* inputs (RN_IN in js/rn-sigs.js) */
#define LN_THROW_IN 12                   /* inputs the throw start can identify (RN_THROW_IN) */
#define LN_WIN 128                       /* samples in a test window (RN_WIN) */
#define LN_SEGS 64                       /* calibration stages */
#define LN_TESTS 40                      /* recorded test windows */
#define LN_PLAN 64                       /* throw pulses */

/* What the Pi's tasks need besides the airframe (exported by the simulator with it; the supervisor uses it too):
 * where the IMU sits from the hub (body axes), how high the throw starts above the ground [m], the air temperature
 * [°C]; per motor whether it is a collective-pitch rotor (a helicopter's: it holds its speed), its temperature limit
 * [°C], winding resistance [Ω], full speed [rad/s], heat capacity [J/K] and cooling at full speed [W/K] (for
 * estimating its temperature from the ESC's current); the battery's cells, internal resistance [Ω] and temperature
 * limit [°C]. Blob: magic 'DFLC', version 1, those as floats (imu_pos[3], hand_h, ambient, n, n × (coll, tmax, rw,
 * om, ct, gf), cells, r_int, batt_tmax), CRC32. */
typedef struct {
  float imu_pos[3], hand_h, ambient; int n;
  int coll[FC_MAX_MOTORS]; float tmax[FC_MAX_MOTORS], rw[FC_MAX_MOTORS], om[FC_MAX_MOTORS], ct[FC_MAX_MOTORS], gf[FC_MAX_MOTORS];
  int cells; float r_int, batt_tmax;
} learn_config;
int learn_config_parse(learn_config *C, const uint8_t *blob, uint32_t len);

typedef struct {                         /* one calibration stage */
  int kind;                              /* 0 settle, 1 motor test, 2 servo test, 3 servo sweep, 4 excite all, 5 validate */
  int who; float amp;                    /* the motor or joint, the size */
  float t0, dur;
  int after;                             /* 1: fit the motors when it ends, 2: the servos */
} ln_seg;
typedef struct { int kind, who, bad, n; float col[6], cmd0, dt; float u[LN_WIN], y[LN_WIN]; } ln_win;
typedef struct { int motor, mask; float ang[FC_MAX_CHAIN]; int joint[FC_MAX_CHAIN]; } ln_pulse;

enum { LN_THR_NONE = 0, LN_THR_HAND, LN_THR_FREE, LN_THR_EXCITE, LN_THR_RECOVER };

typedef struct {
  rn_host *H; int f_rls, f_mot, f_srv, f_thr, ok;
  fc_state FA;                           /* the airframe (parsed as the flight core does), and the model it flies */
  learn_config C; int have_config;
  int n, col0[FC_MAX_MOTORS], nb[FC_MAX_MOTORS];   /* inputs; each motor's first input and how many it has */
  /* the models: rows ax ay az αx αy αz × inputs, acceleration per full thrust */
  float prior[6][LN_IN], B[6][LN_IN], flyB[6][LN_IN];
  int prior_desc, fly_frozen;            /* the prior is the description; flying on flyB while a calibration runs */
  int use_learned, keep, hold_servos, hold_pulses, then_cal;
  float imu_r[3];                        /* the IMU's offset it uses (the throw measures it) */
  float m_eff[FC_MAX_MOTORS];            /* the supervisor's effectiveness, as last told */
  /* what the tests measured */
  int m_meas[FC_MAX_MOTORS], j_meas[FC_MAX_JOINTS];
  float m_tau[FC_MAX_MOTORS], m_curve[FC_MAX_MOTORS], m_fit[FC_MAX_MOTORS], j_rate[FC_MAX_JOINTS], j_lag[FC_MAX_JOINTS], j_fit[FC_MAX_JOINTS];
  /* the telemetry, as last received */
  int nsub; const float *sub;            /* this frame's single steps (open loop) */
  float t, dt, q[4], R[9], f[3], w[3], u[FC_MAX_MOTORS], v[FC_MAX_MOTORS], thc[FC_MAX_JOINTS], thh[FC_MAX_JOINTS], alt, vz; int state, flags, have_alt;
  double t_last; int got;
  float mf_w[3], mf_f[3], mf_a[3]; int mf_ok;   /* filtered readings for the tests (25 Hz, lever arm removed) */
  int updated;                           /* the in-flight learning ran this frame */
  /* calibration */
  int cal; ln_seg seg[LN_SEGS]; int nseg, cur, from_throw; float cal_t, total, gate; int held, waiting;
  ln_win win[LN_TESTS]; int nwin, win_cur;
  double sums_e[6], sums_d[6], sums_y[6], sums_y2[6]; int sums_n;
  float fit_force, fit_rot, desc_force, desc_rot; int have_fit;
  /* the throw */
  int thr; ln_pulse plan[LN_PLAN]; int nplan, pi, step, cut; float tplan, thr_t, ts, w0[3], low_t, h0, z, zmax, zmin, calm_t, t_free;
  float res_fitR, res_fitF, res_taus[LN_THROW_IN], res_r[3]; int res_ok, res_nt;
  int refining; double refine_t0;
  /* what goes out */
  float exc[8 + FC_MAX_MOTORS + FC_MAX_JOINTS]; int exc_n, exc_was;   /* this frame's excitation (exc_was: one was sent) */
  int model_dirty; double model_t;
  char msg[480];
  uint32_t frames;
} learn_state;

/* Set up on a host that has the learning formulas; then the airframe (as the flight core's .dfa) and the config. */
int learn_init(learn_state *L, rn_host *H);
int learn_airframe(learn_state *L, const uint8_t *blob, uint32_t len);
int learn_config_load(learn_state *L, const uint8_t *blob, uint32_t len);
/* A telemetry frame from the flight core (fc_core.h LTEL). Runs everything that happens on it. */
void learn_ltel(learn_state *L, const float *p, int n);
/* The supervisor's settings (fc_core.h SET): a motor's effectiveness changed, so the learned columns are rescaled. */
void learn_set(learn_state *L, const float *p, int n);
/* What to send the flight core after a telemetry frame: an EXC frame (returns its length, 0: nothing), and a
 * MODEL frame when the model changed (or 5 times a second while it learns in flight). */
int learn_exc_frame(learn_state *L, float *out);
int learn_model_frame(learn_state *L, float *out);
/* Commands: calibrate (or stop one), which model to fly on, keep learning in flight, get ready for a throw. */
enum { LN_CMD_CALIBRATE = 1, LN_CMD_STOP, LN_CMD_USE_DESC, LN_CMD_USE_LEARNED, LN_CMD_KEEP_ON, LN_CMD_KEEP_OFF, LN_CMD_THROW,
  LN_CMD_HOLD_PULSES_ON, LN_CMD_HOLD_PULSES_OFF, LN_CMD_THEN_CAL_ON, LN_CMD_THEN_CAL_OFF };
int learn_command(learn_state *L, int cmd);
/* For the screen: a few numbers (see learn_core.c learn_status) and the message. */
int learn_status(const learn_state *L, float *out);
/* The throw plan's length in pulses and its rough duration [s] (to say how high to throw it). */
float learn_throw_plan_time(learn_state *L, int *pulses);

#endif
