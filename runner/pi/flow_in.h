/*
 * The downward camera's optical flow in dfb_pi: the readings dfb_flow sends (one "flow ..." datagram a frame, to
 * dfb_pi's command port), and their calibration against the gyro.
 *
 * Calibration (flowcal): with the motors off, hold the drone over a textured floor and rock it by hand (roll and
 * pitch, a few times each way, the camera staying about where it is). Turning alone sweeps the picture by exactly the
 * angle turned, so fitting the camera's flow against the gyro gives the lens's scale (radians per pixel), whether the
 * picture comes mirrored, how the camera is turned about its axis against the body, and how late its frames are
 * against the gyro. The fit: measured flow m (camera axes) ≈ A · gyro (body x, y), A 2×2, at each of a range of
 * delays; the delay that fits best wins.
 *
 *   flow                 the camera's readings: how many, how often, how good, the latest
 *   flowcal start|stop   start (afresh) or stop collecting; flowcal: the fit so far
 */
#ifndef FLOW_IN_H
#define FLOW_IN_H

#define FLOW_GYRO_N 256                  /* the gyro's history (~2.5 s at 100 Hz) */
#define FLOW_PEND_N 64                   /* readings waiting for the gyro after them */
#define FLOW_LAGS 41                     /* delays tried: −80 … +80 ms */
#define FLOW_LAG_STEP 0.004

typedef struct { double t; float dt, f[2], q; } flow_reading;

typedef struct {
  flow_reading last; double t_rx, t_first; long n, n_good;
  float exp_ms, gain, mean, proc, tex, resid; float rad_px;   /* (as dfb_flow reports them) */
  double gt[FLOW_GYRO_N]; float gw[FLOW_GYRO_N][3]; int gn, gi; double t_nav;
  flow_reading pend[FLOW_PEND_N]; int pn;
  int cal_on; long cal_n; float cal_wmax;
  double S[FLOW_LAGS][9];                /* per delay: Σ w wᵀ (xx, xy, yy), Σ m wᵀ (4), Σ |m|², count */
} flow_in;

void flow_in_init(flow_in *FI);
/* A datagram from dfb_flow ("flow T DT FX FY Q ..."): 1 if it was one (taken), 0 if not. t: now. */
int flow_in_reading(flow_in *FI, const char *s, double t);
/* The gyro (body rates [rad/s]) as the ESP32's NAV frame brings it (100 Hz, only while the Pi sends it commands), at t. */
void flow_in_gyro(flow_in *FI, double t, const float w[3]);
/* The same from its telemetry (body rates [deg/s], ~20 Hz, always): taken only while no NAV frame comes (disarmed,
 * no pilot: a calibration by hand). */
void flow_in_gyro_telem(flow_in *FI, double t, const float w_deg[3]);
/* The text commands flow and flowcal ...: 1 with reply filled, or 0 if s isn't one. */
int flow_in_command(flow_in *FI, const char *s, double t, char *reply, int rn);
/* The latest reading, if there is a fresh one (no older than max_age): 1, else 0. */
int flow_in_latest(const flow_in *FI, double t, double max_age, flow_reading *r);

#endif
