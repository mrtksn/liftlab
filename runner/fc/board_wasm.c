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
 * The telemetry task and the pilot's radio (tlm_core.h, rc_core.h): radio_in/radio_out pass the CRSF bytes of the
 * receiver's UART; tlm_publish has the board's tasks put their items; tlm_pack/tlm_unpack and rc_pack/rc_unpack are
 * the RN_LINK_TLM and RN_LINK_RC frames between boards; radio_stick gives the angle-mode stick command; nav_tick_radio
 * runs a navigation step on the radio's set point (rc_pilot).
 * The command module (runner/ground/ground_core.h), the pilot's side of the radio, is an instance of this too, with
 * the ground program: gnd_setup, then gnd_tick with the pilot's inputs (its CRSF frames for the transmitter module
 * come out in rbuf), gnd_from_radio with what the module hands back, gnd_view for the Ground station.
 * The cargo task (cargo_core.h): cargo_setup, then cargo_tick with the load switches; it takes the radio's LATCH
 * commands from this board's rc_input (the receiver's, or the RN_LINK_RC frames it is sent) and cargo_cmd's.
 * Built by fc/build_wasm.sh. */
#include "fc_core.h"
#include "nav_core.h"
#include "super_core.h"
#include "tlm_sources.h"
#include "tlm_crsf.h"
#include "ground/ground_core.h"
#include "cargo_core.h"

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
static float nio[NIO_IN + 14 + 7];

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

/* ── the telemetry task and the pilot's radio ── */
static tlm_store TS; static tlm_watch TW; static rc_input RCI; static crsf_parser CP; static rc_pilot RP;
static int tlm_local, elrs_rate = 250, elrs_ratio = 4;
static pickup_state PK; static double nav_clock; static uint32_t pk_seen_r, pk_seen_l;   /* the pickup without a radio (below) */
static nav_out last_o; static nav_sp last_sp; static int have_nav_out;
static uint8_t rbuf[2048];                 /* the receiver's UART, both ways */
EXPORT("rbuf_ptr") uint8_t *rbuf_ptr(void) { return rbuf; }
/* local: this board has the radio receiver (runs the telemetry task); rate, ratio: the ExpressLRS link's */
EXPORT("tlm_setup") void tlm_setup(int local, int rate, int ratio) {
  tlm_init(&TS); tlm_watch_init(&TW); rc_pilot_init(&RP); pickup_init(&PK); pk_seen_r = pk_seen_l = 0;
  char *p = (char *)&RCI; for (unsigned i = 0; i < sizeof RCI; i++) p[i] = 0;
  p = (char *)&CP; for (unsigned i = 0; i < sizeof CP; i++) p[i] = 0;
  tlm_local = local; elrs_rate = rate; elrs_ratio = ratio; have_nav_out = 0;
}
/* the radio's rate and telemetry ratio changed, in flight: the budget follows, nothing else is touched */
EXPORT("tlm_link") void tlm_link(int rate, int ratio) { elrs_rate = rate; elrs_ratio = ratio; }
/* n bytes from the receiver, in rbuf */
EXPORT("radio_in") void radio_in(int n, double t) { for (int i = 0; i < n; i++) tlm_crsf_input(&CP, rbuf[i], &RCI, t); }
/* what goes to the receiver now (into rbuf): returns the bytes */
EXPORT("radio_out") int radio_out(double t) { return tlm_service(&TS, &tlm_crsf, t, tlm_crsf_budget_now(elrs_rate, elrs_ratio, &RCI, t), rbuf, (int)sizeof rbuf); }
static cargo_state CG;
/* the board's tasks put their items: tasks bits 1 flight core, 2 navigation, 4 learning, 8 supervisor, 16 cargo */
EXPORT("tlm_publish") void tlm_publish(int tasks, double t) {
  if (tasks & 16) tlm_from_cargo(&TS, &CG, t);
  if (tasks & 1) tlm_from_core(&TS, &TW, &F, t);
  if ((tasks & 2) && have_nav_out) tlm_from_nav(&TS, &TW, &N, &last_o, &last_sp, RP.level, t);
  if ((tasks & 4) && LS.ok) tlm_from_learn(&TS, &TW, &LS, t);
  if ((tasks & 8) && SS.ok) tlm_from_super(&TS, &TW, &SS, t);
  if (tlm_local) tlm_from_link(&TS, &RCI, t);
}
/* the GPS, as the navigation's board reads it */
EXPORT("tlm_gps") void tlm_gps(double lat, double lon, float alt, float speed, float course, int sats, double t) { tlm_from_gps(&TS, lat, lon, alt, speed, course, sats, t); }
EXPORT("tlm_pack") int tlm_pack_(void) { return tlm_pack(&TS, fr, TLM_PACK_MAX); }
EXPORT("tlm_unpack") void tlm_unpack_(int n, double t) { tlm_unpack(&TS, fr, n, t); }
EXPORT("tlm_stats") int tlm_stats(void) { fr[0] = (float)TS.bytes_sent; fr[1] = (float)TS.frames_sent; fr[2] = (float)TS.qn; return 3; }
EXPORT("rc_pack") int rc_pack_(double t) { return rc_pack(&RCI, t, fr); }
EXPORT("rc_unpack") void rc_unpack_(int n, double t) { rc_unpack(&RCI, fr, n, t); }
EXPORT("rc_link_ok") int rc_link_ok_(double t) { return rc_link_ok(&RCI, t); }
/* angle mode: the stick command into cmd (then fc_command, here or on the flight core's board). 0: there is one */
EXPORT("radio_stick") int radio_stick(double t) {
  fc_cmd c; if (rc_stick_cmd(&RCI, t, &c)) return -1;
  cmd[0] = (float)c.arm; cmd[1] = c.roll; cmd[2] = c.pitch; cmd[3] = c.yaw; cmd[4] = c.throttle; cmd[5] = -1; cmd[6] = 0; cmd[7] = 0;
  cmd[8] = cmd[9] = cmd[10] = 0; cmd[11] = 0;
  return 0;
}
EXPORT("rc_msg_ptr") char *rc_msg_ptr(void) { return RP.msg; }
/* a LEARN command came up the radio (its code, once): the board passes it to the learning */
EXPORT("rc_learn_req") int rc_learn_req(void) { int r = RP.learn_req; RP.learn_req = 0; return r; }

/* ── the cargo task ── */
/* n latches, closed: bit per latch closed at the start; each latch's travel [s] in fr */
EXPORT("cargo_setup") void cargo_setup(int n, int closed) { cargo_init(&CG, n, (uint32_t)closed, fr); }
/* latch (−1 all), action (0 open, 1 close, 2 toggle), from a text command or another board: 0, −1, −2 */
EXPORT("cargo_cmd") int cargo_cmd(int latch, int action) { return cargo_command(&CG, latch, action, "board"); }
/* a step: the radio's LATCH commands, the load switches (bits loaded, sw), the moves. Returns what to drive: bit per latch closed. */
EXPORT("cargo_tick") int cargo_tick(float dt, double t, int loaded, int sw) {
  if (RCI.frames || RCI.cmd_seq) cargo_from_rc(&CG, &RCI, t);
  cargo_switches(&CG, (uint32_t)loaded, (uint32_t)sw);
  cargo_step(&CG, dt);
  return (int)cargo_drive(&CG);
}
/* what it said last, and how many things it has said */
EXPORT("cargo_msg_ptr") char *cargo_msg_ptr(void) { return CG.msg; }
EXPORT("cargo_nmsg") int cargo_nmsg(void) { return (int)CG.nmsg; }

/* ── the command module ── */
static gnd_state GND;
/* latch: the buttons that toggle (GB bits). resume (may be left out: 0): the buttons on before, for a command module
 * set up again in flight (gnd_config.resume), so its switch warning doesn't turn arm and fly off. Uses the step runner
 * set up by host_setup (with the ground program). */
EXPORT("gnd_setup") int gnd_setup(int latch, int resume) { gnd_config c; gnd_config_default(&c); c.latch = (uint32_t)latch; c.resume = (uint32_t)resume; return gnd_init(&GND, host_ok ? &H : 0, &c); }
EXPORT("gnd_why_ptr") char *gnd_why_ptr(void) { return GND.why; }
/* one step: the buttons held (GB bits), the analog sticks (has: a bit per axis); frames for the module into rbuf */
EXPORT("gnd_tick") int gnd_tick(int held, int has, float roll, float pitch, float thr, float yaw, double t, float dt) {
  gnd_input in = { .axis = { roll, pitch, thr, yaw }, .has_axis = (uint32_t)has, .held = (uint32_t)held };   /* (the rest, sw, zero) */
  return gnd_step(&GND, &in, t, dt, rbuf, (int)sizeof rbuf);
}
EXPORT("gnd_from_radio") void gnd_from_radio_(int n, double t) { gnd_from_radio(&GND, rbuf, n, t); }
/* 0, −1 too many waiting, −2 out of range or not a number (nothing queued) */
EXPORT("gnd_goto") int gnd_goto_(float x, float y, float z, float h) { return gnd_goto(&GND, x, y, z, h); }
/* a pickup: the hook's place on the drone (body axes), then the thing's top from home and the latch: as gnd_pickup */
EXPORT("gnd_pickup") int gnd_pickup_(float hx, float hy, float hz, float x, float y, float z, int latch, double t) {
  float top[3] = { x, y, z }; GND.C.hook[0] = hx; GND.C.hook[1] = hy; GND.C.hook[2] = hz; return gnd_pickup(&GND, top, latch, t);
}
EXPORT("gnd_command") int gnd_command_(int cmd, int n) { return gnd_command(&GND, cmd, fr, n); }   /* values in fr */
/* a latching button (gnd_setup's latch) set on or off, whatever its button last did: the state, or −1 */
EXPORT("gnd_latch") int gnd_latch_(int b, int on) { return gnd_latch(&GND, b, on); }
/* the buttons sent on in the last step (GB bits): what to pass back as gnd_setup's resume */
EXPORT("gnd_on") int gnd_on(void) { return (int)GND.on; }
EXPORT("gnd_view") int gnd_view_(double t) { return gnd_view_pack(&GND, t, fr, (int)(sizeof fr / sizeof *fr)); }
EXPORT("gnd_mode_ptr") char *gnd_mode_ptr(void) { return GND.V.mode; }
EXPORT("gnd_msg_text") char *gnd_msg_text(int i) { return GND.V.msg[i % GND_MSGS].s; }
EXPORT("gnd_msg_sev") int gnd_msg_sev(int i) { return GND.V.msg[i % GND_MSGS].sev; }
EXPORT("gnd_msg_t") double gnd_msg_t(int i) { return GND.V.msg[i % GND_MSGS].t; }
EXPORT("gnd_why_text") const char *gnd_why_text_(int w) { return w >= 0 && w < GND_WHY_N ? gnd_why_text[w] : ""; }

/* ── the pickup (pickup_core.h): the radio's (rc_pilot) or, without a radio, the simulator's pilot's ── */
/* start one without a radio: the hub's spot from home, heading [rad], latch. 0, or −1 (pk_msg_ptr says why) */
EXPORT("pickup_cmd") int pickup_cmd(float x, float y, float z, float heading, int latch) {
  float s[3] = { x, y, z };
  if (!have_nav_out || !last_o.have_home) { PK.phase = 0; return -1; }
  return pickup_start(&PK, s, heading, latch, &last_o, nav_clock);
}
EXPORT("pickup_stop") void pickup_stop(void) { pickup_cancel(&PK, "the pilot"); pickup_cancel(&RP.pk, "the pilot"); }
/* the one under way (the radio's, or ours): phase (0 none), then in fr the target x y z and the latch */
EXPORT("pk_view") int pk_view(void) {
  const pickup_state *K = pickup_active(&RP.pk) ? &RP.pk : &PK;
  fr[0] = K->tgt[0]; fr[1] = K->tgt[1]; fr[2] = K->tgt[2]; fr[3] = (float)K->latch; return K->phase;
}
/* a request to the cargo task, once: 1 + 4 latch + action (1 close), or 0 */
EXPORT("pk_req") int pk_req(void) {
  if (RP.pk.nreq != pk_seen_r) { pk_seen_r = RP.pk.nreq; return 1 + 4 * RP.pk.req_latch + RP.pk.req_act; }
  if (PK.nreq != pk_seen_l) { pk_seen_l = PK.nreq; return 1 + 4 * PK.req_latch + PK.req_act; }
  return 0;
}
/* what ours said (the radio's goes with the radio's messages): 1 if new, the text at pk_msg_ptr */
EXPORT("pk_said") int pk_said(void) { int s = PK.said; PK.said = 0; return s; }
EXPORT("pk_msg_ptr") char *pk_msg_ptr(void) { return PK.msg; }

static int nav_run(nav_sp *sp_radio);
EXPORT("nav_tick") int nav_tick(void) { return nav_run(0); }
/* A navigation step on the radio's set point; nio's set point is not used. Then nio's outputs, and after them: the
 * target (from home) x y z, heading, armed by the radio, the radio link up, a new message (rc_msg_ptr). */
EXPORT("nav_tick_radio") int nav_tick_radio(double t) {
  nav_sp sp; float dt = nio[36];                 /* (the step size, as nav_run reads nio) */
  sp.heading = RP.heading;
  int arm = rc_pilot_step(&RP, &RCI, t, &N, &last_o, dt, &sp);
  if (RP.said) { RP.said = 0; tlm_text(&TS, 4, RP.msg); nio[NIO_IN + 14 + 6] = 1; } else nio[NIO_IN + 14 + 6] = 0;
  int e = nav_run(&sp);
  float *p = nio + NIO_IN + 14;
  for (int i = 0; i < 3; i++) *p++ = sp.target[i];
  *p++ = sp.heading; *p++ = (float)arm; *p++ = (float)rc_link_ok(&RCI, t);
  return e;
}
static int nav_run(nav_sp *sp_radio) {
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
  nav_clock += dt;
  if (sp_radio) sp = *sp_radio;
  else if (pickup_active(&PK)) { if (!sp.fly) pickup_cancel(&PK, "not flying"); else pickup_step(&PK, &last_o, nav_clock, dt, &sp); }
  int e = nav_step(&N, &in, &sp, dt, &o);
  last_o = o; last_sp = sp; have_nav_out = 1;
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
