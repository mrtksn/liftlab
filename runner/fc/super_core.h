/*
 * LiftLab health supervisor: watches for failing, weakening or overheating parts and changes how the drone
 * flies, on a Raspberry Pi.
 *
 * Portable C like the other tasks; the Pi runs it (runner/pi/dfb_pi.c) and the simulator flies the same code built to
 * WebAssembly. Its formulas (actuatorHealth, faultDecision, flightPolicy, liftMargin, thermalModel) run on the step
 * runner. It only talks to the flight core through settings (fc_core.h SET), 10 times a second: a failed motor is
 * taken out of the allocation, a weakened one has its column scaled to what it really does, a hot one is capped, a
 * stuck servo is left out of the steering with its real angle; and a flight mode with limits (careful, return home,
 * land) for the flight core and the navigation.
 *
 * On the flight core's telemetry (LTEL): 50 times a second while flying, what the table says each motor did (its
 * column × its thrust) and what the IMU says the drone did; actuatorHealth finds what no longer matches. With the
 * health sensors wired to its board (motor temperatures, ESC rpm and current, battery voltage, current and
 * temperature; super_health), faultDecision and flightPolicy decide.
 */
#ifndef SUPER_CORE_H
#define SUPER_CORE_H
#include "learn_core.h"

#define SP_BATCH 16                      /* samples per check (the formula's batch) */
#define SP_LOG 12                        /* events kept */

typedef struct { double t; int tone; char text[160]; } sp_event;   /* tone: 0 info, 1 good, 2 warn, 3 bad */
typedef struct { float phi[FC_MAX_MOTORS][6], psi[FC_MAX_JOINTS][6], y[6], cmd[FC_MAX_MOTORS]; } sp_sample;

typedef struct {
  rn_host *H; int f_ah, f_fd, f_fp, f_lm, f_th, ok;
  fc_state FA;                           /* the airframe, and the table as the flight core flies it (MODEL, SET) */
  learn_config C; int have_config;
  /* telemetry */
  double t, t_last, next_tick, fly_t0; int got, flying, open, state; float dt, q[4], R[9], f[3], w[3], v[FC_MAX_MOTORS], thh[FC_MAX_JOINTS], alt, alt0; int have_alt;
  float mf_w[3], mf_f[3], mf_a[3]; int mf_ok, nframe; double open_t;
  sp_sample batch[SP_BATCH]; int nb;
  /* what the health sensors say (super_health) */
  int h_has_t[FC_MAX_MOTORS], h_has_rpm[FC_MAX_MOTORS], h_has_i[FC_MAX_MOTORS]; float h_t[FC_MAX_MOTORS], h_rpm[FC_MAX_MOTORS], h_i[FC_MAX_MOTORS];
  int b_has_v, b_has_i, b_has_t; float b_v, b_i, b_t;
  float t_est[FC_MAX_MOTORS]; int have_t_est[FC_MAX_MOTORS];
  /* the battery */
  int cells, cell_lost; float vh_t[24], vh_v[24]; int nvh;
  /* decisions */
  int m_on[FC_MAX_MOTORS], m_state[FC_MAX_MOTORS], m_why[FC_MAX_MOTORS], logged_cap[FC_MAX_MOTORS]; float m_eff[FC_MAX_MOTORS], m_cap[FC_MAX_MOTORS], m_val[FC_MAX_MOTORS], m_temp[FC_MAX_MOTORS], m_eta[FC_MAX_MOTORS], m_conf[FC_MAX_MOTORS];
  int j_off[FC_MAX_JOINTS], j_why[FC_MAX_JOINTS]; float j_ang[FC_MAX_JOINTS], j_val[FC_MAX_JOINTS];
  int mode, why, have_pol; float rp_bad, v_bad, lim_speed, lim_lean, lim_accel, why_val;
  float margin, soc; int rp_ok, yaw_ok, have_soc;
  int sent;                              /* a SET frame is ready (super_set_frame) */
  sp_event log[SP_LOG]; int nlog; uint32_t log_seq;
  char why_text[96];
} super_state;

int super_init(super_state *S, rn_host *H);
int super_airframe(super_state *S, const uint8_t *blob, uint32_t len);
int super_config_load(super_state *S, const uint8_t *blob, uint32_t len);
/* The flight core's telemetry (LTEL), and the model it flies (MODEL, from the learning task). */
void super_ltel(super_state *S, const float *p, int n);
void super_model(super_state *S, const float *p, int n);
/* The health sensors: nm, per motor (has temperature, °C, has rpm, rpm, has current, A), then the battery (has
 * voltage, V, has current, A, has temperature, °C). */
void super_health(super_state *S, const float *p, int n);
/* After a telemetry frame: the SET frame to send, when one is due (10 Hz). Returns its length or 0. */
int super_set_frame(super_state *S, float *out);
/* For the screen: [0] mode, [1] why, [2] the number with it, [3] lift margin, [4] roll/pitch held, [5] yaw held, [6] battery
 * charge (−1 unknown), [7] working cells, [8] speed limit, [9] lean limit, [10] acceleration limit, [11] events so far;
 * from 12, per motor: state, on, eff, cap, why, val, temperature (−999 unknown), eta, conf; per joint: out, angle, why, val. */
int super_status(const super_state *S, float *out);
/* The text of a motor's or servo's reason, and of the mode's (for the screen and the log). */
void super_why_motor(const super_state *S, int i, char *o, int size);
void super_why_mode(const super_state *S, char *o, int size);

#endif
