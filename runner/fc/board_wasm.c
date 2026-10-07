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
 * The data bus (bus.h, docs/topic-bus.md): every board has one; its flight core and navigation publish on it, the page
 * copies topics between boards with bus_sub_out/bus_sub_in and bus_out/bus_in (the RN_LINK_BUS_SUB and RN_LINK_BUS
 * frames), and reads it all with bus_list for the Live data view.
 * Built by fc/build_wasm.sh. */
#include "fc_core.h"
#include "nav_core.h"
#include "super_core.h"
#include "tlm_sources.h"
#include "tlm_crsf.h"
#include "radio_link.h"
#include "plink.h"
#include "pframe.h"
#include "clink.h"
#include "lmux.h"
#include "peer.h"
#include "fleet.h"
#include "ground/ground_core.h"
#include "cargo_core.h"
#include "bus.h"
#include "prog_core.h"

#define ARENA_CAP 131072
#define CODE_CAP 65536
#define POOL_CAP 8192
#define IMG_CAP (512u << 10)
#define BLOB_CAP 8192
static float arenas_[3][ARENA_CAP], pools_[3][POOL_CAP];
static int32_t codes_[3][CODE_CAP];
static uint8_t img[IMG_CAP], blob[BLOB_CAP], ncfg[256], lcfg[512];
static rn_host H; static int host_ok;
static bus BUS;                        /* this board's data bus */
static float busv[BUS_TOPICS * (5 + BUS_VALS)];
static prog_state PG;                  /* this board's programs */
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
  bus_init(&BUS);
  float *arenas[3] = { arenas_[0], arenas_[1], arenas_[2] }, *pools[3] = { pools_[0], pools_[1], pools_[2] };
  int32_t *codes[3] = { codes_[0], codes_[1], codes_[2] };
  int e = rn_host_init(&H, img, img_len, arenas, ARENA_CAP, codes, CODE_CAP, pools, POOL_CAP);
  if (e) { set_why(F.why, "the flight program didn't load"); set_why(N.why, F.why); return 100 + e; }
  host_ok = 1; prog_init(&PG, &H, &BUS); return 0;
}
/* The flight core, with an airframe. */
EXPORT("fc_setup") int fc_setup(uint32_t blob_len) {
  if (!host_ok) return 3;
  if (fc_init(&F, &H)) return 1;
  if (fc_airframe_load(&F, blob, blob_len)) return 2;
  fc_bus_attach(&F, &BUS);
  return 0;
}
static fleet_state FL;                    /* the fleet program, beside the navigation (fleet.h) */
/* The navigation, with its config. */
EXPORT("nav_setup") int nav_setup(uint32_t cfg_len) {
  if (!host_ok) return 3;
  if (nav_init(&N, &H)) return 1;
  if (nav_config_load(&N, ncfg, cfg_len)) return 2;
  nav_bus_attach(&N, &BUS);
  fleet_init(&FL, &H);                            /* (no fleet program in it: the fleet is off, the rest flies) */
  return 0;
}
/* ── the data bus ── */
EXPORT("bus_time") void bus_time(double t) { bus_clock(&BUS, t); }
EXPORT("bus_reset") void bus_reset(void) { bus_init(&BUS); }   /* (a board with no flight program: host_setup does it for the others) */
/* every topic, into busv: kind (0 this board's, 1 a mirror), floats n, updates here (mod 2^24: publishes, or copies taken),
 * age [s] (−1 never), copies refused, then its n values. Returns how many topics; their names from bus_name_ptr. */
EXPORT("bus_list_ptr") float *bus_list_ptr(void) { return busv; }
EXPORT("bus_list") int bus_list(void) {
  int k = 0;
  for (int i = 0; i < BUS.nt; i++) {
    const bus_entry *T = &BUS.T[i];
    busv[k++] = T->kind; busv[k++] = T->n; busv[k++] = (float)(T->got % 16777216u); busv[k++] = (float)bus_age(&BUS, i); busv[k++] = (float)T->bad;
    const float *v = BUS.pool + T->off; for (int j = 0; j < T->n; j++) busv[k++] = T->has ? v[j] : 0;
  }
  return BUS.nt;
}
EXPORT("bus_name_ptr") const char *bus_name_ptr(int i) { return i >= 0 && i < BUS.nt ? BUS.T[i].name : ""; }
EXPORT("bus_layout_ptr") const char *bus_layout_ptr(int i) { return i >= 0 && i < BUS.nt ? BUS.T[i].layout : ""; }
/* txt holds a topic's name, a 0, then its layout (may be empty) */
static const char *txt_layout(void) { int i = 0; while (i < (int)sizeof txt - 1 && txt[i]) i++; return txt + i + 1; }
static int starts(const char *s, const char *p) { for (int i = 0; p[i]; i++) if (s[i] != p[i]) return 0; return 1; }
/* its counts, into fr: publishes, topics sent, taken, refused, unknown; subscriptions served, asked for */
EXPORT("bus_stats") int bus_stats(void) { fr[0] = (float)BUS.n_pub; fr[1] = (float)BUS.n_sent; fr[2] = (float)BUS.n_got; fr[3] = (float)BUS.n_bad; fr[4] = (float)BUS.n_unknown; fr[5] = (float)BUS.ns; fr[6] = (float)BUS.nw; return 7; }
/* ask for another board's topic (txt: its name, 0, its layout): n floats, every period [s] (0: on change). Its index
 * here, or −1. */
EXPORT("bus_want") int bus_want(int n, float period) { return bus_want_topic(&BUS, txt, n, txt_layout(), period); }
/* publish a topic of this board's (txt: its name, 0, its layout; the values in fr): a program's own, so its name starts
 * with "user." (the flight code's topics have their one writer). 0, or −1. */
EXPORT("bus_put") int bus_put(int n) {
  if (!starts(txt, "user.")) return -1;
  int id = bus_topic(&BUS, txt, n, txt_layout()); return id < 0 ? -1 : bus_pub(&BUS, id, fr, n);
}
/* the sensor drivers' side (the page plays the drivers): declare a sensor's topic (sensor.…), then publish its readings */
EXPORT("bus_declare") int bus_declare(int n) { return starts(txt, "sensor.") ? bus_topic(&BUS, txt, n, txt_layout()) : -1; }
EXPORT("bus_driver_put") int bus_driver_put(int id, int n) { return id >= 0 && id < BUS.nt && starts(BUS.T[id].name, "sensor.") ? bus_pub(&BUS, id, fr, n) : -1; }   /* (values in fr) */
/* ── programs (prog_core.h) ── */
EXPORT("prog_reset") void prog_reset(void) { prog_init(&PG, host_ok ? &H : 0, &BUS); }
/* a program: txt = its formula's name, 0, the topic it writes, 0, that topic's layout; it writes n floats; it runs
 * every period [s], or (0) on a change of the read prog_on names. Its index, or −1 (prog_why_ptr) */
EXPORT("prog_add") int prog_add_(int n, float period) { const char *t = txt_layout(), *l = t; while (*l) l++; return prog_add(&PG, txt, t, n, l + 1, period); }
EXPORT("prog_read") int prog_read_(int i) { return prog_read(&PG, i, txt); }      /* a topic it reads: txt */
EXPORT("prog_on") int prog_on(int i) { return prog_trigger(&PG, i, txt); }         /* the read whose change runs it: txt */
EXPORT("prog_check") int prog_check_(int i) { return prog_check(&PG, i); }
EXPORT("prog_why_ptr") char *prog_why_ptr(void) { return PG.why; }
EXPORT("prog_tick") void prog_tick(void) { prog_step(&PG); }
/* into fr, per program: checked, runs, failed runs, runs skipped waiting for inputs, the last error */
EXPORT("prog_list") int prog_list(void) { int k = 0; for (int i = 0; i < PG.n; i++) { fr[k++] = (float)PG.P[i].ok; fr[k++] = (float)PG.P[i].runs; fr[k++] = (float)PG.P[i].fails; fr[k++] = (float)PG.P[i].waits; fr[k++] = (float)PG.P[i].err; } return PG.n; }
EXPORT("bus_sub_out") int bus_sub_out(void) { return bus_sub_pack(&BUS, fr, 1024); }              /* RN_LINK_BUS_SUB, into fr */
EXPORT("bus_sub_in") int bus_sub_in(int peer, int n) { return bus_sub_take(&BUS, peer, fr, n); }   /* one that came, in fr */
EXPORT("bus_out") int bus_out(int peer) { return bus_pack(&BUS, peer, fr, 1024); }                /* RN_LINK_BUS due for peer, into fr */
EXPORT("bus_in") int bus_in(int n) { return bus_unpack(&BUS, fr, n); }                             /* one that came, in fr */

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
static int tlm_local; static rlink_cfg RL = { .kind = RLINK_ELRS, .rate_hz = 250, .ratio = 4, .channel = 1 };   /* the radio link: which, and its settings (radio_link.h) */
static pickup_state PK; static double nav_clock; static uint32_t pk_seen_r, pk_seen_l;   /* the pickup without a radio (below) */
static nav_out last_o; static nav_sp last_sp; static int have_nav_out;
static uint8_t rbuf[2048];                 /* the receiver's UART, both ways */
EXPORT("rbuf_ptr") uint8_t *rbuf_ptr(void) { return rbuf; }
/* local: this board has the radio receiver (runs the telemetry task) */
EXPORT("tlm_setup") void tlm_setup(int local) {
  tlm_init(&TS); tlm_watch_init(&TW); rc_pilot_init(&RP); pickup_init(&PK); pk_seen_r = pk_seen_l = 0;
  char *p = (char *)&RCI; for (unsigned i = 0; i < sizeof RCI; i++) p[i] = 0;
  p = (char *)&CP; for (unsigned i = 0; i < sizeof CP; i++) p[i] = 0;
  tlm_local = local; have_nav_out = 0;
}
/* the radio link: which (radio_link.h RLINK_*) and its settings (ExpressLRS: a packet rate, b telemetry ratio). At
 * any time, in flight too: the telemetry's budget follows, nothing else is touched. 0, or −1 (unknown, unchanged). */
EXPORT("radio_link") int radio_link(int kind, int a, int b) { return rlink_make(&RL, kind, a, b); }
/* ── a packet link's end (plink.h): the drone's on the receiver's board, the ground's on the command module's ──
 * The board's stack writes and reads CRSF as with a module (radio_out/radio_in, gnd_tick/gnd_from_radio); these take
 * what it wrote, give what it should read, and make and take the packets the simulated air carries. */
/* Two links at once (lmux.h): two of each, plink_sel picks the one the calls below work on (the link models step
 * each in turn); the stack's side (plink_stack_in, plink_stack_out) then goes through the merger, both links. */
static plink PLs[2]; static int pl_ons[2]; static uint8_t pbuf[PLINK_MTU]; static pframe_rx PFs[2];
/* a compact packet link (the nRF24L01: clink.h) in the same calls, when the link is one */
static clink CLs[2]; static int cl_ons[2]; static int SEL;
#define PL PLs[SEL]
#define pl_on pl_ons[SEL]
#define PF PFs[SEL]
#define CL CLs[SEL]
#define cl_on cl_ons[SEL]
static lmux MX; static int mx_on, mx_role; static rlink_cfg RL2;
EXPORT("plink_sel") void plink_sel(int i) { SEL = i == 1; }
EXPORT("plink_off") void plink_off(void) { pl_on = cl_on = 0; }   /* (the selected one isn't a packet link: ExpressLRS's modules) */
/* Two links (n 2) or one (n 1): which ways each carries, this board's role (0 the ground, 1 the drone), and the
 * second's settings (kind, a, b: the telemetry's room follows the link that carries it). A link that isn't a packet
 * link here (ExpressLRS: the model's modules) hands its frames in with link_in. */
EXPORT("link_mux") void link_mux(int n, int up0, int down0, int up1, int down1, int role, int kind2, int a2, int b2) {
  mx_on = n == 2; mx_role = role ? LMUX_DRONE : LMUX_GROUND;
  int up[2] = { up0, up1 }, down[2] = { down0, down1 };
  lmux_init(&MX, mx_role, 2, up, down); rlink_default(&RL2); rlink_make(&RL2, kind2, a2, b2);
  if (!mx_on) { pl_ons[1] = cl_ons[1] = 0; SEL = 0; }
}
EXPORT("link_in") void link_in(int i, int n, double t) { if (mx_on) lmux_from_link(&MX, i, rbuf, n, t); }
EXPORT("pbuf_ptr") uint8_t *pbuf_ptr(void) { return pbuf; }
/* role 0 the ground, 1 the drone; the binding phrase: n bytes in rbuf; session: this start's own number (not 0);
 * kind, a, b: the link (radio_link.h rlink_make) */
EXPORT("plink_setup") int plink_setup(int role, int n, int session, int kind, int a, int b) {
  plink_cfg C; plink_cfg_default(&C, role ? PLINK_DRONE : PLINK_GROUND);
  rlink_cfg L; rlink_default(&L); if (!rlink_make(&L, kind, a, b)) plink_cfg_link(&C, &L);   /* (the link as set: a serial line's sizes from its speed) */
  char ph[64]; int k = 0; for (; k < n && k < 63; k++) ph[k] = (char)rbuf[k]; ph[k] = 0;
  plink_key(ph, &C.k0, &C.k1); plink_init(&PL, &C, (uint32_t)session); pframe_rx_init(&PF); pl_on = 1;
  cl_on = rlink_compact(&L);
  if (cl_on) { clink_cfg K; clink_cfg_default(&K, C.role); clink_cfg_link(&K, &L); K.k0 = C.k0; K.k1 = C.k1; clink_init(&CL, &K, (uint32_t)session); }
  return 0;
}
static void stack_in1(int s, int n, double t) { if (cl_ons[s]) clink_from_stack(&CLs[s], rbuf, n, t); else if (pl_ons[s]) plink_from_stack(&PLs[s], rbuf, n, t); }
static int stack_out1(int s, double t, uint8_t *b, int cap) { return cl_ons[s] ? clink_to_stack(&CLs[s], t, b, cap) : pl_ons[s] ? plink_to_stack(&PLs[s], t, b, cap) : 0; }
static uint32_t known1(int s) { return cl_ons[s] ? CLs[s].known : pl_ons[s] ? PLs[s].known : 0; }
/* what the stack wrote, in rbuf (two links: on each that carries this way) */
EXPORT("plink_stack_in") void plink_stack_in(int n, double t) {
  if (!mx_on) { stack_in1(SEL, n, t); return; }
  for (int s = 0; s < 2; s++) if (mx_role == LMUX_GROUND ? MX.k[s].up : MX.k[s].down) stack_in1(s, n, t);
}
/* for the stack, into rbuf (two links: merged; each told what this end hears over both, a one-way one tied to the other: radio_mux.c) */
EXPORT("plink_stack_out") int plink_stack_out(double t) {
  if (!mx_on) return stack_out1(SEL, t, rbuf, (int)sizeof rbuf);
  int up, down, rssi; lmux_lq(&MX, t, &up, &down, &rssi);
  for (int s = 0; s < 2; s++) {
    int lq = mx_role == LMUX_DRONE ? up : down;
    if (cl_ons[s]) clink_hear(&CLs[s], lq); else if (pl_ons[s]) plink_hear(&PLs[s], lq, rssi);
    int two_way = MX.k[s].up && MX.k[s].down, other = MX.k[1 - s].up && MX.k[1 - s].down;
    if (!two_way && other && pl_ons[s]) plink_tie(&PLs[s], known1(1 - s));
    uint8_t b[PLINK_OUT]; int m = stack_out1(s, t, b, (int)sizeof b); if (m) lmux_from_link(&MX, s, b, m, t);
  }
  return lmux_to_stack(&MX, t, rbuf, (int)sizeof rbuf);
}
EXPORT("link_followed") int link_followed(double t) { return mx_on ? lmux_followed(&MX, t) : 0; }
EXPORT("plink_air_out") int plink_air_out(double t) { return cl_on ? clink_to_air(&CL, t, pbuf, (int)sizeof pbuf) : pl_on ? plink_to_air(&PL, t, pbuf, (int)sizeof pbuf) : 0; }   /* a packet due, into pbuf */
EXPORT("plink_air_in") int plink_air_in(int n, int rssi, double t) { return cl_on ? clink_from_air(&CL, pbuf, n, rssi, t) : pl_on ? plink_from_air(&PL, pbuf, n, rssi, t) : 0; }   /* a packet that came, in pbuf */
/* Bluetooth LE from a browser (js/live.js): this end's biggest packet once the connection's MTU is known (the
 * packet layer's own, plink_cfg.mtu: 64 at least, PLINK_MTU at most), and the binding phrase's mark in the drone's
 * advertising (in rbuf, n bytes: the same as radio_ble.c's, the SipHash of "adv" under the phrase's key; its first
 * byte lowest). */
EXPORT("plink_mtu") int plink_mtu(int m) { if (!pl_on) return 0; if (m > PLINK_MTU) m = PLINK_MTU; if (m < 64) m = 64; PL.C.mtu = m; return m; }
EXPORT("ble_mark") uint32_t ble_mark(int n) {
  char ph[64]; int k = 0; for (; k < n && k < 63; k++) ph[k] = (char)rbuf[k]; ph[k] = 0;
  uint64_t k0, k1; plink_key(ph, &k0, &k1); const uint8_t m[3] = { 'a', 'd', 'v' };
  return (uint32_t)plink_siphash(k0, k1, m, 3);
}
EXPORT("plink_channel") int plink_channel(double t) { return cl_on ? clink_channel(&CL, t) : -1; }   /* the compact link's radio channel now */
/* A serial line (pframe.h): the packet due as the line's bytes (into sbuf; the packet itself stays in pbuf, its
 * length from plink_pkt_n), and the bytes that came off the line (in sbuf) through the deframer to the packet layer:
 * the packets taken. */
static uint8_t sbuf[PFRAME_WIRE(PLINK_MTU)]; static int pkt_n;
EXPORT("sbuf_ptr") uint8_t *sbuf_ptr(void) { return sbuf; }
EXPORT("plink_pkt_n") int plink_pkt_n(void) { return pkt_n; }
EXPORT("plink_serial_out") int plink_serial_out(double t) {
  pkt_n = pl_on ? plink_to_air(&PL, t, pbuf, (int)sizeof pbuf) : 0;
  return pkt_n ? pframe_encode(pbuf, pkt_n, sbuf, (int)sizeof sbuf) : 0;
}
EXPORT("plink_serial_in") int plink_serial_in(int n, double t) {
  int took = 0; uint8_t p[PFRAME_MAX];
  for (int i = 0; i < n && pl_on; i++) { int m = pframe_feed(&PF, sbuf[i], p, (int)sizeof p); if (m) took += plink_from_air(&PL, p, m, 0, t); }
  return took;
}
/* its numbers: sent, got, bad, replays, stale sessions, dropped, sent again, LQ heard, signal heard, LQ the other end
 * hears, bytes waiting, reliable frames waiting, connected; a serial line: frames that didn't decode; reliable frames
 * the other end dropped (skipped). (A compact link: the same, its stream bytes not yet sent and its chunks in flight
 * for the two waiting, and its hellos last) */
EXPORT("plink_stats") int plink_stats(double t) {
  float *o = fr; int k = 0;
  if (cl_on) {                                                       /* (the compact link's, in the same places) */
    const clink_counts *c = &CL.N;
    o[k++] = (float)c->sent; o[k++] = (float)c->got; o[k++] = (float)c->bad; o[k++] = (float)c->replays; o[k++] = 0;
    o[k++] = (float)c->dropped; o[k++] = (float)c->resent; o[k++] = (float)clink_lq(&CL, t); o[k++] = 0; o[k++] = (float)CL.peer_lq;
    o[k++] = (float)(CL.tx.n - CL.tx.sent); o[k++] = (float)(uint8_t)(CL.tx.hi - CL.tx.base); o[k++] = (float)clink_connected(&CL, t); o[k++] = 0; o[k++] = (float)c->hellos;
    return k;
  }
  const plink_counts *c = &PL.N;
  o[k++] = (float)c->sent; o[k++] = (float)c->got; o[k++] = (float)c->bad; o[k++] = (float)c->replays; o[k++] = (float)c->stale_sessions;
  o[k++] = (float)c->uq_dropped; o[k++] = (float)c->resent; o[k++] = (float)plink_lq(&PL, t); o[k++] = (float)PL.rssi; o[k++] = (float)PL.peer_lq;
  o[k++] = (float)PL.uq_n; o[k++] = (float)PL.rq_n; o[k++] = (float)plink_connected(&PL, t); o[k++] = (float)PF.bad; o[k++] = (float)c->skipped;
  return k;
}
/* n bytes from the receiver, in rbuf */
/* ── drones talking to each other (peer.h): this board's end, when it's the one with the radio (the simulator's
 * js/peer-air.js carries the packets between the fleet's drones) ── */
static peer_net PN; static fleet_link FK; static int pn_on; static uint8_t paddr[6]; static uint32_t pfrom;
EXPORT("peer_addr_ptr") uint8_t *peer_addr_ptr(void) { return paddr; }
/* id: the drone's node number; session: this start's; rbuf: the fleet phrase, a 0, the drone's name */
EXPORT("peer_setup") void peer_setup(int on, int id, int session) {
  pn_on = on; fleet_link_init(&FK); if (!on) return;
  char ph[64], nm[PEER_NAME]; int k = 0, j = 0;
  while (k < 63 && rbuf[k]) { ph[k] = (char)rbuf[k]; k++; } ph[k] = 0;
  for (k++; j < PEER_NAME - 1 && rbuf[k]; k++, j++) nm[j] = (char)rbuf[k]; nm[j] = 0;
  peer_init(&PN, (uint32_t)id, (uint32_t)session, nm, ph);
}
EXPORT("peer_publish") void peer_publish_(int n) { if (pn_on) peer_publish(&PN, fr, n); }   /* the values, in fr */
EXPORT("peer_air_out") int peer_air_out(double t) { return pn_on ? peer_to_air(&PN, t, paddr, pbuf, (int)sizeof pbuf) : 0; }   /* a packet due: in pbuf, its address in paddr */
EXPORT("peer_air_in") int peer_air_in(int n, int rssi, double t) { return pn_on ? peer_from_air(&PN, paddr, pbuf, n, rssi, t) : 0; }   /* one that came: in pbuf, from paddr */
EXPORT("peer_ping") int peer_ping_(int id, double t) { return pn_on ? peer_ping(&PN, (uint32_t)id, t) : -1; }
EXPORT("peer_send") int peer_send_(int id, int n) { return pn_on ? peer_send(&PN, (uint32_t)id, rbuf, n) : -1; }   /* a message: rbuf */
EXPORT("peer_recv") int peer_recv_(void) { return pn_on ? peer_recv(&PN, &pfrom, rbuf, (int)sizeof rbuf) : -1; }   /* the next message, into rbuf; its sender: peer_from */
EXPORT("peer_from") int peer_from(void) { return (int)pfrom; }
EXPORT("peer_id") int peer_id_(int i) { return i >= 0 && i < PEER_MAX ? (int)PN.P[i].id : 0; }   /* (a float can't hold a node number) */
EXPORT("peer_name_ptr") const char *peer_name_ptr(int i) { return i >= 0 && i < PEER_MAX ? PN.P[i].name : ""; }
/* the table, into fr: per slot, 8 numbers then its values: state (−1 empty), node number, link quality ours, its
 * of us, since heard [s], since its values [s], the last round trip [s] (−1 none), how many values; and the counts */
EXPORT("peer_list") int peer_list(double t) {
  float *o = fr; int k = 0;
  for (int i = 0; i < PEER_MAX; i++) {
    const peer_t *P = &PN.P[i]; int st = pn_on ? peer_state(&PN, i, t) : -1;
    o[k++] = (float)st; o[k++] = (float)P->id; o[k++] = (float)(st >= 0 ? peer_lq(&PN, i, t) : 0); o[k++] = (float)P->heard_us;
    o[k++] = (float)(t - P->t_heard); o[k++] = (float)(P->nvals ? t - P->t_vals : -1); o[k++] = P->rtt; o[k++] = (float)P->nvals;
    for (int v = 0; v < PEER_VALS; v++) o[k++] = v < P->nvals ? P->vals[v] : 0;
  }
  const peer_counts *c = &PN.N;
  o[k++] = (float)c->sent; o[k++] = (float)c->beacons; o[k++] = (float)c->got; o[k++] = (float)c->bad; o[k++] = (float)c->replays; o[k++] = (float)c->resent; o[k++] = (float)c->dropped;
  return k;
}
/* The fleet program's link, on the peer end's board (fleet.h): fr[0..2] the flight core's state, battery %, height. */
EXPORT("fleet_publish") void fleet_publish(double t) { if (pn_on) fleet_link_publish(&FK, &PN, fr, t); }
EXPORT("fleet_pack") int fleet_pack(double t) { float h[FLEET_HEAD] = { fr[0], fr[1], fr[2] }; return pn_on ? fleet_link_pack(&PN, t, h, fr) : 0; }   /* the table, into fr */
EXPORT("fleet_apply") int fleet_apply(int n, double t) { return pn_on ? fleet_link_apply(&FK, &PN, fr, n, t) : -1; }   /* what the program says back, in fr */
/* … and on the navigation's board: the table in, what it says out, the pilot's switch, what it is doing */
EXPORT("fleet_in") void fleet_in(int n) { fleet_peers(&FL, fr, n, nav_clock); }
EXPORT("fleet_take") int fleet_take(void) { return fleet_out(&FL, fr); }
EXPORT("fleet_cmd") int fleet_cmd(int on) { return fleet_engage(&FL, on, &N, &last_o, "the pilot"); }
EXPORT("fleet_said") int fleet_said(void) { int s = FL.said; FL.said = 0; return s; }
EXPORT("fleet_msg_ptr") char *fleet_msg_ptr(void) { return FL.msg; }
/* into fr: has the program, engaged, its target (from home) x y z, heading, calls, fails, messages sent, got, what it
 * publishes: n, then the values */
EXPORT("fleet_view") int fleet_view(void) {
  int k = 0; fr[k++] = (float)FL.ok; fr[k++] = (float)FL.engaged; for (int i = 0; i < 3; i++) fr[k++] = FL.go_p[i]; fr[k++] = FL.go_h;
  fr[k++] = (float)FL.calls; fr[k++] = (float)FL.fails; fr[k++] = (float)FL.sent; fr[k++] = (float)FL.got; fr[k++] = (float)FL.npub;
  for (int j = 0; j < FLEET_VALS; j++) fr[k++] = j < FL.npub ? FL.pub[j] : 0;
  return k;
}
EXPORT("radio_in") void radio_in(int n, double t) { for (int i = 0; i < n; i++) tlm_crsf_input(&CP, rbuf[i], &RCI, t); }
/* what goes to the receiver now (into rbuf): returns the bytes */
static const rlink_cfg *budget_link(double t) {                     /* (two links: as radio_mux.c radio_mux_link) */
  if (!mx_on) return &RL;
  const rlink_cfg *L[2] = { &RL, &RL2 };
  for (int i = 0; i < 2; i++) if (MX.k[i].down && t - MX.k[i].t_stats < 0.5) return L[i];
  for (int i = 0; i < 2; i++) if (MX.k[i].down) return L[i];
  return &RL;
}
EXPORT("radio_out") int radio_out(double t) { return tlm_service(&TS, &tlm_crsf, t, rlink_budget_now(budget_link(t), &RCI, t), rbuf, (int)sizeof rbuf); }
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
  int fleet_was = RP.mis_on = FL.engaged;                     /* the fleet program's target, flown as the pilot's (fleet.h) */
  if (FL.engaged) { for (int i = 0; i < 3; i++) { RP.mis_t[i] = FL.go_p[i]; RP.mis_v[i] = FL.go_v[i]; } RP.mis_h = FL.go_h; }
  int arm = rc_pilot_step(&RP, &RCI, t, &N, &last_o, dt, &sp);
  if (fleet_was && !RP.mis_on) fleet_engage(&FL, 0, &N, &last_o, RP.mis_why);
  if (RP.fleet_req) { fleet_engage(&FL, RP.fleet_req == 1, &N, &last_o, "the pilot's radio"); RP.fleet_req = 0; }
  if (FL.said && !RP.said) { FL.said = 0; for (int i = 0; i < 64; i++) RP.msg[i] = FL.msg[i]; RP.said = 1; }
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
  else fleet_sp(&FL, &sp);                       /* (without a radio: the simulator's pilot; engaged, the program's target) */
  int e = nav_step(&N, &in, &sp, dt, &o);
  fleet_step(&FL, &N, &o, nav_clock);
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
