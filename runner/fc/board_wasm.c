/* One flight computer for the simulator: the portable flight code (fc_core.c, nav_core.c, learn_core.c, super_core.c,
 * with rn_host.c and rn.c)
 * built to WebAssembly. The page makes one instance per board (an ESP32, a Raspberry Pi, …) and runs on it the
 * tasks the board has: the flight core, the navigation, or both. What the page passes between instances is what
 * the boards pass over their link (commands, telemetry), with the link's delay; on one board they pass directly.
 * So what flies in the simulator is the code that flies on the drone.
 *
 * Use: put the flight program (the image the simulator compiled) in img, call host_setup(); then fc_setup() with an
 * airframe in blob and/or nav_setup() with a navigation config in ncfg. Each control step: write the IMU sample in
 * io, call fc_tick(); each navigation step: write nio, call nav_tick(), read its outputs. Commands go through cmd
 * and fc_command().
 * Built by fc/build_wasm.sh. */
#include "fc_core.h"
#include "nav_core.h"
#include "super_core.h"

#define ARENA_CAP 131072
#define CODE_CAP 65536
#define POOL_CAP 8192
#define IMG_CAP (512u << 10)
#define BLOB_CAP 8192
static float arenas_[3][ARENA_CAP], pools_[3][POOL_CAP];
static int32_t codes_[3][CODE_CAP];
static uint8_t img[IMG_CAP], blob[BLOB_CAP], ncfg[256], lcfg[512];
static rn_host H; static int host_ok;
static fc_state F;
static nav_state N;
static learn_state LS;
static super_state SS;
static float fr[1024];                 /* frames in and out: EXC, MODEL, SET, LTEL, the health readings, status */
static char txt[512];
/* io: in gyro[3] acc[3] have_gyro dt vbatt baro_alt have_baro mag[3] have_mag | out motor[12] servo[8] |
 *     state q[4] w[3] trap rho att_ok yaw_sp az_f iAz vz alt have_alt tau_des[3] */
#define IO_IN 15
static float io[IO_IN + FC_MAX_MOTORS + FC_MAX_JOINTS + 19];
static float cmd[12];                /* arm roll pitch yaw throttle test_motor test_throttle guided ax ay az heading */
/* nio: in q[4] w[3] acc[3] have_att | have_baro alt age | have_fix p[3] v[3] age | have_flow flow[2] range q age |
 *      target[3] vref[3] heading fly | dt   →   out acc[3] heading fly p[3] v[3] have_home ready landed */
#define NIO_IN 41
static float nio[NIO_IN + 14];

#define EXPORT(n) __attribute__((export_name(n)))
EXPORT("img_ptr") uint8_t *img_ptr(void) { return img; }
EXPORT("img_cap") uint32_t img_cap(void) { return IMG_CAP; }
EXPORT("blob_ptr") uint8_t *blob_ptr(void) { return blob; }
EXPORT("blob_cap") uint32_t blob_cap(void) { return BLOB_CAP; }
EXPORT("ncfg_ptr") uint8_t *ncfg_ptr(void) { return ncfg; }
EXPORT("io_ptr") float *io_ptr(void) { return io; }
EXPORT("cmd_ptr") float *cmd_ptr(void) { return cmd; }
EXPORT("nio_ptr") float *nio_ptr(void) { return nio; }
EXPORT("why_ptr") char *why_ptr(void) { return F.why; }
EXPORT("lcfg_ptr") uint8_t *lcfg_ptr(void) { return lcfg; }
EXPORT("fr_ptr") float *fr_ptr(void) { return fr; }
EXPORT("txt_ptr") char *txt_ptr(void) { return txt; }
EXPORT("nav_why_ptr") char *nav_why_ptr(void) { return N.why; }
EXPORT("state") int state(void) { return F.state; }

static void set_why(char *w, const char *s) { int i = 0; for (; s[i] && i < 63; i++) w[i] = s[i]; w[i] = 0; }

/* The board's step runner with the flight program as its built-in program. */
EXPORT("host_setup") int host_setup(uint32_t img_len) {
  static fc_state zf; static nav_state zn; static learn_state zl; static super_state zs; F = zf; N = zn; LS = zl; SS = zs; host_ok = 0;
  float *arenas[3] = { arenas_[0], arenas_[1], arenas_[2] }, *pools[3] = { pools_[0], pools_[1], pools_[2] };
  int32_t *codes[3] = { codes_[0], codes_[1], codes_[2] };
  int e = rn_host_init(&H, img, img_len, arenas, ARENA_CAP, codes, CODE_CAP, pools, POOL_CAP);
  if (e) { set_why(F.why, "the flight program didn't load"); set_why(N.why, F.why); return 100 + e; }
  host_ok = 1; return 0;
}
/* The flight core, with an airframe. */
EXPORT("fc_setup") int fc_setup(uint32_t blob_len) {
  if (!host_ok) return 3;
  if (fc_init(&F, &H)) return 1;
  if (fc_airframe_load(&F, blob, blob_len)) return 2;
  return 0;
}
/* The navigation, with its config. */
EXPORT("nav_setup") int nav_setup(uint32_t cfg_len) {
  if (!host_ok) return 3;
  if (nav_init(&N, &H)) return 1;
  if (nav_config_load(&N, ncfg, cfg_len)) return 2;
  return 0;
}
/* A new program while flying: through the host's loading steps, as on the drone. */
EXPORT("stage") int stage(uint32_t img_len) { return rn_host_stage(&H, img, img_len); }
EXPORT("host_event") int host_event(void) { int e = H.last_event; H.last_event = 0; return e; }
EXPORT("host_phase") int host_phase(void) { return H.phase + 10 * H.act; }
/* Advance the host's loading steps: once per step of the board's fastest task. */
EXPORT("host_tick") void host_tick(float dt) { rn_host_tick(&H, dt); }

EXPORT("fc_command") void command(void) {
  fc_cmd c = { (int)cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], (int)cmd[5], cmd[6], cmd[7] > 0.5f, { cmd[8], cmd[9], cmd[10] }, cmd[11] };
  fc_command(&F, &c);
}
EXPORT("fc_tick") void fc_tick(void) {
  fc_imu m = { { io[0], io[1], io[2] }, { io[3], io[4], io[5] }, io[6] > 0.5f, io[9], io[10] > 0.5f, { io[11], io[12], io[13] }, io[14] > 0.5f };
  float dt = io[7];
  fc_out o;
  fc_step(&F, &m, dt, io[8], &o);
  float *p = io + IO_IN;
  for (int i = 0; i < FC_MAX_MOTORS; i++) *p++ = o.motor[i];
  for (int j = 0; j < FC_MAX_JOINTS; j++) *p++ = o.servo[j];
  for (int k = 0; k < 4; k++) *p++ = F.q[k];
  for (int k = 0; k < 3; k++) *p++ = F.w[k];
  *p++ = (float)F.trap; *p++ = F.rho; *p++ = (float)F.att_ok; *p++ = F.yaw_sp; *p++ = F.az_f; *p++ = F.iAz; *p++ = F.have_alt ? F.vz_e : F.vz_i; *p++ = F.alt_e; *p++ = (float)F.have_alt;
  for (int k = 0; k < 3; k++) *p++ = F.tau_des[k];
}
/* The learning task: the airframe in blob, the config in lcfg. */
EXPORT("learn_setup") int learn_setup(uint32_t af_len, uint32_t cfg_len) {
  if (!host_ok) return 3;
  if (learn_init(&LS, &H)) return 1;
  if (learn_config_load(&LS, lcfg, cfg_len)) return 2;
  if (learn_airframe(&LS, blob, af_len)) return 4;
  return 0;
}
EXPORT("learn_msg_ptr") char *learn_msg_ptr(void) { return LS.msg; }
EXPORT("learn_ltel") void learn_ltel_(int n) { learn_ltel(&LS, fr, n); }
EXPORT("learn_set") void learn_set_(int n) { learn_set(&LS, fr, n); }
EXPORT("learn_exc") int learn_exc(void) { return learn_exc_frame(&LS, fr); }
EXPORT("learn_model") int learn_model(void) { return learn_model_frame(&LS, fr); }
EXPORT("learn_cmd") int learn_cmd(int c) { return learn_command(&LS, c); }
EXPORT("learn_status") int learn_status_(void) { return learn_status(&LS, fr); }
EXPORT("learn_plan") float learn_plan(void) { int n = 0; float t = learn_throw_plan_time(&LS, &n); fr[0] = (float)n; return t; }
/* The health supervisor. */
EXPORT("super_setup") int super_setup(uint32_t af_len, uint32_t cfg_len) {
  if (!host_ok) return 3;
  if (super_init(&SS, &H)) return 1;
  if (super_config_load(&SS, lcfg, cfg_len)) return 2;
  if (super_airframe(&SS, blob, af_len)) return 4;
  return 0;
}
EXPORT("super_why_ptr") char *super_why_ptr(void) { return SS.why_text; }
EXPORT("super_ltel") void super_ltel_(int n) { super_ltel(&SS, fr, n); }
EXPORT("super_model") void super_model_(int n) { super_model(&SS, fr, n); }
EXPORT("super_health") void super_health_(int n) { super_health(&SS, fr, n); }
EXPORT("super_set") int super_set(void) { return super_set_frame(&SS, fr); }
EXPORT("super_status") int super_status_(void) { return super_status(&SS, fr); }
EXPORT("super_log_n") int super_log_n(void) { return SS.nlog; }
EXPORT("super_log_text") char *super_log_text(int i) { return SS.log[i].text; }
EXPORT("super_log_tone") int super_log_tone(int i) { return SS.log[i].tone; }
EXPORT("super_log_t") float super_log_t(int i) { return (float)SS.log[i].t; }
EXPORT("super_motor_why") char *super_motor_why(int i) { super_why_motor(&SS, i, txt, (int)sizeof txt); return txt; }
EXPORT("super_mode_why") char *super_mode_why(void) { super_why_mode(&SS, txt, (int)sizeof txt); return txt; }
/* The flight core's side of those frames. */
EXPORT("fc_exc") int fc_exc_(int n) { return fc_exc(&F, fr, n); }
EXPORT("fc_model") int fc_model_(int n) { return fc_model(&F, fr, n); }
EXPORT("fc_set") int fc_set_(int n) { return fc_set(&F, fr, n); }
EXPORT("fc_ltel") int fc_ltel_(void) { return fc_ltel(&F, fr); }
EXPORT("nav_set") void nav_set_(int n) { nav_set(&N, fr, n); }

EXPORT("nav_tick") int nav_tick(void) {
  const float *a = nio; nav_in in; nav_sp sp; nav_out o; int k = 0;
  for (int i = 0; i < 4; i++) in.q[i] = a[k++];
  for (int i = 0; i < 3; i++) in.w[i] = a[k++];
  for (int i = 0; i < 3; i++) in.acc[i] = a[k++];
  in.have_att = a[k++] > 0.5f;
  in.have_baro = a[k++] > 0.5f; in.baro_alt = a[k++]; in.baro_age = a[k++];
  in.have_fix = a[k++] > 0.5f; for (int i = 0; i < 3; i++) in.fix_p[i] = a[k++]; for (int i = 0; i < 3; i++) in.fix_v[i] = a[k++]; in.fix_age = a[k++];
  in.have_flow = a[k++] > 0.5f; in.flow[0] = a[k++]; in.flow[1] = a[k++]; in.range = a[k++]; in.flow_q = a[k++]; in.flow_age = a[k++];
  for (int i = 0; i < 3; i++) sp.target[i] = a[k++];
  for (int i = 0; i < 3; i++) sp.vref[i] = a[k++];
  sp.heading = a[k++]; sp.fly = a[k++] > 0.5f;
  float dt = a[k++];
  int e = nav_step(&N, &in, &sp, dt, &o);
  float *p = nio + NIO_IN;
  for (int i = 0; i < 3; i++) *p++ = o.acc[i];
  *p++ = o.heading; *p++ = (float)o.fly;
  for (int i = 0; i < 3; i++) *p++ = o.p[i];
  for (int i = 0; i < 3; i++) *p++ = o.v[i];
  *p++ = (float)o.have_home; *p++ = (float)o.ready; *p++ = (float)o.landed;
  return e;
}

/* The few C library functions the code uses (there is no C library in this build). */
typedef __SIZE_TYPE__ size_t;
void *memcpy(void *d, const void *s, size_t n) { uint8_t *a = d; const uint8_t *b = s; while (n--) *a++ = *b++; return d; }
void *memmove(void *d, const void *s, size_t n) {
  uint8_t *a = d; const uint8_t *b = s;
  if (a == b || !n) return d;
  if (a < b) while (n--) *a++ = *b++; else { a += n; b += n; while (n--) *--a = *--b; }
  return d;
}
void *memset(void *d, int c, size_t n) { uint8_t *a = d; while (n--) *a++ = (uint8_t)c; return d; }
size_t strlen(const char *s) { size_t n = 0; while (s[n]) n++; return n; }
int strncmp(const char *a, const char *b, size_t n) { for (; n; n--, a++, b++) { if (*a != *b) return (unsigned char)*a - (unsigned char)*b; if (!*a) return 0; } return 0; }
