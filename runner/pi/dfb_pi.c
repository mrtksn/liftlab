/*
 * LiftLab on the Raspberry Pi: the navigation, the learning and the health supervisor.
 *
 * The same code the simulator runs for a Pi board (fc/nav_core.c, fc/learn_core.c, fc/super_core.c with the step
 * runner and the Pi's built-in program, rn_builtin_pi.c: the formulas of those three tasks). It talks to the ESP32
 * flight controller over the serial link (rn_link.h):
 *   - navigation (--nav): the ESP32 sends RN_LINK_NAV 100 times a second while guided commands come; for each one this
 *     runs a navigation step and sends a guided command (RN_LINK_CMD, 12 floats): the acceleration wanted and the
 *     heading. If they stop (this program stops, the cable comes out), the ESP32 goes to its failsafe within 0.5 s
 *     and lands. A GPS on its own serial port (NMEA: GGA and RMC, as the NEO-6M sends; a sentence whose checksum
 *     doesn't match is dropped) gives position and velocity; home is where it took off;
 *   - learning and supervisor (--airframe and --pi): it asks for the ESP32's LTEL telemetry (RN_LINK_WANT, twice a
 *     second) and answers with the learning's excitation and model (RN_LINK_EXC, RN_LINK_MODEL) and the
 *     supervisor's settings (RN_LINK_SET). The supervisor's settings also reach the navigation here (it flies home or
 *     lands) and the learning (it rescales what it learned). This Pi has no health sensor drivers yet: the
 *     supervisor works from the flight core's data stream (a failed or weakened motor, a stuck servo, the lift left).
 *
 *   - the pilot's radio and the telemetry (rc_core.h, tlm_core.h): with an ExpressLRS (or Crossfire) receiver on a
 *     serial port here (--crsf /dev/ttyAMA1; CRSF at 420000 baud; --elrs 250,4: the link's packet rate and telemetry
 *     ratio), its channels fly the navigation (the sticks move the target; switches arm, take off, hold, fly home; a
 *     second without channels in flight and it flies home and lands), and this program sends the telemetry of all
 *     the tasks, the ESP32's included (it asks for them: RN_LINK_WANT bit 2), down the radio. With the receiver on
 *     the ESP32 instead, the channels come from it (RN_LINK_RC) and this program sends its tasks' telemetry there
 *     (RN_LINK_TLM) when asked.
 *   - the cargo (--latch pwm0,gpio17: fc/cargo_core.h): the latches wired to this Pi, a servo on a hardware PWM
 *     channel or an on/off line (latch_hw.c; --latch-us 1000,2000: a servo's pulse closed and open, µs). They start
 *     closed; the radio's LATCH commands (the ground station's buttons) and the text commands open and close them,
 *     and their state goes down the radio with the rest.
 *
 * The pilot's commands are lines of text, on standard input or UDP (port 14560 by default; fly.py or a phone can
 * send them):
 *   arm | disarm | takeoff [height m] | land | goto X Y Z | move VX VY VZ (target velocity, m/s; 0 0 0 stops)
 *   heading DEG | hold | home | status
 *   calibrate | stop | learned | description | keep on | keep off | throw | learning | health
 *   latch N open | latch N close | latch N toggle | latch all open | latches   (with --latch; N from 1)
 *   pickup X Y Z [N]: fly the hook onto a thing whose top is at X Y Z [m] from home, close latch N (1) and climb
 *     (fc/pickup_core.h; --hook DX,DY,DZ: where the hook is from the hub, body axes, 0,0,-0.06 by default)
 * (throw: hold it level and arm it first; throw it upward and it flies itself from there.)
 *
 * Build:  sh runner/pi/build.sh
 * Run:    ./dfb_pi --link /dev/serial0 --baud 921600 --nav drone.dnc [--gps /dev/ttyUSB0] [--airframe drone.dfa --pi drone.dlc]
 *                  [--crsf /dev/ttyAMA1 --elrs 250,4] [--latch pwm0,gpio17 [--latch-us 1000,2000]]
 * While the radio's channels come, they fly it; the text commands are for when there is no radio.
 * (the files: the simulator's Computers tab -> Export, on the Pi board.)
 */
#define _DEFAULT_SOURCE
#include "nav_core.h"
#include "super_core.h"
#include "tlm_sources.h"
#include "tlm_crsf.h"
#include "rn_link.h"
#include "latch_hw.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

extern const uint8_t *const rn_builtin_img;
extern const uint32_t rn_builtin_len;

int serial_custom_baud(int fd, int baud);
static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

static int open_serial(const char *dev, int baud) {
  int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) { perror(dev); return -1; }
  struct termios t; tcgetattr(fd, &t); cfmakeraw(&t);
  speed_t sp = baud == 9600 ? B9600 : baud == 38400 ? B38400 : baud == 57600 ? B57600 : baud == 230400 ? B230400 : baud == 460800 ? B460800 : baud == 921600 ? B921600 : B115200;
  cfsetispeed(&t, sp); cfsetospeed(&t, sp); t.c_cflag |= CLOCAL | CREAD; t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &t); tcflush(fd, TCIOFLUSH);
  return fd;
}

/* ── GPS: NMEA sentences → position (x north, y west, z up) [m] around the first fix, and velocity ── */
typedef struct { int have_origin; double lat0, lon0, alt0, lat, lon, alt; float p[3], v[3]; double t; int fix, sats; char line[128]; int n; } gps_t;
static double nmea_deg(const char *f, const char *hemi) {   /* ddmm.mmmm or dddmm.mmmm */
  double x = atof(f); int d = (int)(x / 100); double v = d + (x - d * 100) / 60;
  return (*hemi == 'S' || *hemi == 'W') ? -v : v;
}
static int split(char *s, char **f, int max) { int n = 0; f[n++] = s; for (; *s && n < max; s++) if (*s == ',' || *s == '*') { *s = 0; f[n++] = s + 1; } return n; }
static int hexv(char c) { return c >= '0' && c <= '9' ? c - '0' : c >= 'A' && c <= 'F' ? c - 'A' + 10 : c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1; }
/* the sentence's checksum (after '*', two hex digits: the XOR of what lies between '$' and '*'); one without is refused */
static int nmea_ok(const char *s) {
  int x = 0; const char *p = s; for (; *p && *p != '*'; p++) x ^= (unsigned char)*p;
  if (*p != '*') return 0;
  int h = hexv(p[1]), l = h < 0 ? -1 : hexv(p[2]);
  return l >= 0 && (p[3] == 0 || p[3] == '\r' || p[3] == '\n') && (h << 4 | l) == x;
}
static void gps_line(gps_t *G, char *s) {
  if (!nmea_ok(s)) return;                                    /* (a damaged sentence: a position off by kilometres) */
  char *f[24]; int n = split(s, f, 24);
  if (n > 9 && !strcmp(f[0] + 2, "GGA")) {                   /* time, lat, N/S, lon, E/W, quality, sats, hdop, altitude */
    if (atoi(f[6]) < 1 || !*f[2] || !*f[4]) { G->fix = 0; return; }
    double lat = nmea_deg(f[2], f[3]), lon = nmea_deg(f[4], f[5]), alt = atof(f[9]);
    if (!G->have_origin) { G->lat0 = lat; G->lon0 = lon; G->alt0 = alt; G->have_origin = 1; }
    G->lat = lat; G->lon = lon; G->alt = alt; G->sats = atoi(f[7]);
    const double R = 6371000.0, k = M_PI / 180;
    G->p[0] = (float)((lat - G->lat0) * k * R); G->p[1] = (float)(-(lon - G->lon0) * k * R * cos(G->lat0 * k)); G->p[2] = (float)(alt - G->alt0);
    G->fix = 1; G->t = now_s();
  } else if (n > 8 && !strcmp(f[0] + 2, "RMC") && *f[2] == 'A') {   /* speed over ground [knots], course [deg] */
    double v = atof(f[7]) * 0.514444, c = atof(f[8]) * M_PI / 180;
    G->v[0] = (float)(v * cos(c)); G->v[1] = (float)(-v * sin(c)); G->v[2] = 0;
  }
}
static void gps_read(gps_t *G, int fd) {
  char b[256]; ssize_t n = read(fd, b, sizeof b);
  for (ssize_t i = 0; i < n; i++) {
    if (b[i] == '$') G->n = 0;
    if (b[i] == '\n' || b[i] == '\r') { if (G->n > 6) { G->line[G->n] = 0; gps_line(G, G->line + 1); } G->n = 0; }
    else if (G->n < (int)sizeof G->line - 1) G->line[G->n++] = b[i];
  }
}

/* ── the pilot ── */
typedef struct { int arm, fly, landing; nav_sp sp; } pilot_t;
static void pilot_line(pilot_t *P, nav_state *N, const nav_out *o, char *s, char *reply, size_t rn) {
  float a, b, c; reply[0] = 0;
  if (!strncmp(s, "arm", 3)) {
    if (N->landed) { nav_land_reset(N); P->fly = 0; P->landing = 0; P->sp.fly = 0; }   /* it landed by itself, and is disarmed: it may fly again */
    P->arm = 1; snprintf(reply, rn, "arming");
  }
  else if (!strncmp(s, "disarm", 6)) { P->arm = 0; P->fly = 0; snprintf(reply, rn, "disarmed"); }
  else if (!strncmp(s, "takeoff", 7)) {
    float h = 1.5f; sscanf(s + 7, "%f", &h);
    if (!P->arm) snprintf(reply, rn, "arm first");
    else { P->fly = 1; P->landing = 0; P->sp.target[0] = o->p[0]; P->sp.target[1] = o->p[1]; P->sp.target[2] = h; P->sp.fly = 1; snprintf(reply, rn, "taking off to %.1f m", h); }
  }
  else if (!strncmp(s, "land", 4)) { P->landing = 1; memset(P->sp.vref, 0, sizeof P->sp.vref); snprintf(reply, rn, "landing"); }
  else if (sscanf(s, "goto %f %f %f", &a, &b, &c) == 3) { P->sp.target[0] = a; P->sp.target[1] = b; P->sp.target[2] = c; memset(P->sp.vref, 0, sizeof P->sp.vref); snprintf(reply, rn, "going to %.1f %.1f %.1f", a, b, c); }
  else if (sscanf(s, "move %f %f %f", &a, &b, &c) == 3) { P->sp.vref[0] = a; P->sp.vref[1] = b; P->sp.vref[2] = c; snprintf(reply, rn, "moving"); }
  else if (sscanf(s, "heading %f", &a) == 1) { P->sp.heading = a * (float)M_PI / 180; snprintf(reply, rn, "heading %.0f", a); }
  else if (!strncmp(s, "hold", 4)) { memcpy(P->sp.target, o->p, sizeof P->sp.target); memset(P->sp.vref, 0, sizeof P->sp.vref); snprintf(reply, rn, "holding"); }
  else if (!strncmp(s, "home", 4)) { P->sp.target[0] = P->sp.target[1] = 0; memset(P->sp.vref, 0, sizeof P->sp.vref); snprintf(reply, rn, "going home"); }
  else if (!strncmp(s, "status", 6)) snprintf(reply, rn, "%s; at %.2f %.2f %.2f, speed %.2f %.2f %.2f%s", P->fly ? "flying" : P->arm ? "armed" : "disarmed", o->p[0], o->p[1], o->p[2], o->v[0], o->v[1], o->v[2], o->ready ? "" : "; position not settled yet");
  else snprintf(reply, rn, "? arm | disarm | takeoff [h] | land | goto x y z | move vx vy vz | heading deg | hold | home | status | calibrate | stop | learned | description | keep on | keep off | throw | learning | health");
}
/* The learning's and the supervisor's commands. Returns 1 if it was one of theirs. */
static int task_line(learn_state *L, int have_learn, super_state *S, int have_super, pilot_t *P, const nav_out *o, char *s, char *reply, size_t rn) {
  static const struct { const char *w; int c; } cmds[] = { { "calibrate", LN_CMD_CALIBRATE }, { "stop", LN_CMD_STOP }, { "learned", LN_CMD_USE_LEARNED },
    { "description", LN_CMD_USE_DESC }, { "keep on", LN_CMD_KEEP_ON }, { "keep off", LN_CMD_KEEP_OFF }, { "throw", LN_CMD_THROW } };
  for (unsigned i = 0; i < sizeof cmds / sizeof *cmds; i++) if (!strncmp(s, cmds[i].w, strlen(cmds[i].w))) {
    if (!have_learn) { snprintf(reply, rn, "the learning isn't running (start with --airframe and --pi)"); return 1; }
    if (cmds[i].c == LN_CMD_THROW) {
      if (!P->arm) { snprintf(reply, rn, "arm first, holding it level"); return 1; }
      P->fly = 1; P->landing = 0;                   /* the navigation flies it once it has caught itself (there: see main) */
      memcpy(P->sp.target, o->p, sizeof P->sp.target); memset(P->sp.vref, 0, sizeof P->sp.vref);
    }
    learn_command(L, cmds[i].c);
    if (cmds[i].c == LN_CMD_CALIBRATE && L->cal) snprintf(reply, rn, "calibrating: about %.0f s of tests while it hovers (\"stop\" ends it)", (double)L->total);
    else if (L->msg[0]) snprintf(reply, rn, "%s", L->msg);
    else snprintf(reply, rn, "ok");
    return 1;
  }
  if (!strncmp(s, "learning", 8)) {
    if (!have_learn) snprintf(reply, rn, "the learning isn't running");
    else if (L->cal) snprintf(reply, rn, "calibrating: %.0f%% done", (double)(L->total > 0 ? 100 * L->cal_t / L->total : 0));
    else snprintf(reply, rn, "%s%s", L->use_learned ? "on the learned model. " : "on the description. ", L->msg);
    return 1;
  }
  if (!strncmp(s, "health", 6)) {
    if (!have_super) { snprintf(reply, rn, "the supervisor isn't running"); return 1; }
    const char *mode[] = { "normal", "careful", "returning home", "landing" }; char why[96]; super_why_mode(S, why, sizeof why);
    int k = snprintf(reply, rn, "%s%s%s; lift margin %.2fx", mode[S->mode & 3], why[0] ? ": " : "", why, (double)S->margin);
    for (int i = 0; i < S->nlog && i < 3 && k < (int)rn; i++) k += snprintf(reply + k, rn - k, " | %.1f s %s", S->log[i].t, S->log[i].text);
    return 1;
  }
  return 0;
}
/* The cargo's commands (with --latch). Returns 1 if it was one of them. */
static int cargo_line(cargo_state *C, latch_out *out, char *s, char *reply, size_t rn) {
  if (strncmp(s, "latch", 5)) return 0;
  if (!C->n) { snprintf(reply, rn, "no latches (start with --latch pwm0,gpio17)"); return 1; }
  if (!strncmp(s, "latches", 7)) {
    int k = 0; for (int i = 0; i < C->n && k < (int)rn; i++) { int b = cargo_bits(C, i); k += snprintf(reply + k, rn - k, "%slatch %d (%s): %s%s", i ? "; " : "", i + 1, latch_hw_name(&out[i]), b & 1 ? "closed" : "open", b & 4 ? ", moving" : ""); }
    return 1;
  }
  char which[16], act[16]; int i;
  if (sscanf(s + 5, "%15s %15s", which, act) != 2) { snprintf(reply, rn, "latch N open | close | toggle, latch all open, latches"); return 1; }
  int a = !strcmp(act, "open") ? CG_OPEN : !strcmp(act, "close") ? CG_CLOSE : !strcmp(act, "toggle") ? CG_TOGGLE : -1;
  int l = !strcmp(which, "all") ? CG_ALL : sscanf(which, "%d", &i) == 1 ? i - 1 : -2;
  int e = a < 0 ? -2 : l == -2 ? -1 : cargo_command(C, l, a, "text");
  if (e) snprintf(reply, rn, e == -1 ? "no latch %s (1 to %d, or all)" : "%s? open, close or toggle", e == -1 ? which : act, C->n);
  else snprintf(reply, rn, "%s", C->msg);
  return 1;
}

#define ARENA_CAP 131072
#define CODE_CAP 65536
static float arenas_[3][ARENA_CAP], pools_[3][8192];
static int32_t codes_[3][CODE_CAP];
static uint8_t *read_file(const char *path, uint32_t *n) {
  FILE *f = fopen(path, "rb"); if (!f) { perror(path); return 0; }
  static uint8_t bufs[3][16384]; static int k; uint8_t *b = bufs[k++ % 3];
  *n = (uint32_t)fread(b, 1, sizeof bufs[0], f); fclose(f); return b;
}
static void send_frame(int fd, uint8_t type, const void *p, uint32_t n) {
  static uint8_t fr[FC_MODEL_MAX * 4 + 32]; uint32_t len = rn_link_frame(fr, sizeof fr, type, (const uint8_t *)p, n);
  if (len && write(fd, fr, len) < 0 && errno != EAGAIN) perror("link");
}

int main(int argc, char **argv) {
  const char *link_dev = "/dev/serial0", *gps_dev = 0, *cfg_path = 0, *af_path = 0, *pi_path = 0, *crsf_dev = 0; int baud = 921600, gps_baud = 9600, port = 14560, no_learn = 0, no_super = 0;
  int elrs_rate = 250, elrs_ratio = 4; const char *latch_spec = 0; int us_closed = 1000, us_open = 2000; float hook[3] = { 0, 0, -0.06f };
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--no-learning")) { no_learn = 1; continue; }
    if (!strcmp(argv[i], "--no-supervisor")) { no_super = 1; continue; }
    if (i + 1 >= argc) break;
    if (!strcmp(argv[i], "--link")) link_dev = argv[++i]; else if (!strcmp(argv[i], "--baud")) baud = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--gps")) gps_dev = argv[++i]; else if (!strcmp(argv[i], "--gps-baud")) gps_baud = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--nav") || !strcmp(argv[i], "--config")) cfg_path = argv[++i]; else if (!strcmp(argv[i], "--port")) port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--airframe")) af_path = argv[++i]; else if (!strcmp(argv[i], "--pi")) pi_path = argv[++i];
    else if (!strcmp(argv[i], "--crsf")) crsf_dev = argv[++i]; else if (!strcmp(argv[i], "--elrs")) sscanf(argv[++i], "%d,%d", &elrs_rate, &elrs_ratio);
    else if (!strcmp(argv[i], "--hook")) sscanf(argv[++i], "%f,%f,%f", &hook[0], &hook[1], &hook[2]);
    else if (!strcmp(argv[i], "--latch")) latch_spec = argv[++i]; else if (!strcmp(argv[i], "--latch-us")) sscanf(argv[++i], "%d,%d", &us_closed, &us_open);
  }
  if (!cfg_path) { fprintf(stderr, "usage: dfb_pi --nav drone.dnc [--airframe drone.dfa --pi drone.dlc] [--no-learning] [--no-supervisor]\n"
    "              [--link /dev/serial0] [--baud 921600] [--gps /dev/ttyUSB0] [--port 14560] [--crsf /dev/ttyAMA1 --elrs 250,4]\n"
    "              [--latch pwm0,gpio17 (or dry) --latch-us 1000,2000]\n"
    "(the pilot's commands go through the navigation, so it always runs; the learning and the supervisor need the airframe and the Pi config)\n"); return 2; }

  setvbuf(stdout, NULL, _IOLBF, 0);   /* a line at a time, also into a pipe or a log file */
  static rn_host H; static nav_state N; static learn_state LS; static super_state SS;
  float *arenas[3] = { arenas_[0], arenas_[1], arenas_[2] }, *pools[3] = { pools_[0], pools_[1], pools_[2] };
  int32_t *codes[3] = { codes_[0], codes_[1], codes_[2] };
  int e = rn_host_init(&H, rn_builtin_img, rn_builtin_len, arenas, ARENA_CAP, codes, CODE_CAP, pools, 8192);
  if (e) { fprintf(stderr, "flight program didn't load (%d)\n", e); return 1; }
  if (nav_init(&N, &H)) { fprintf(stderr, "%s\n", N.why); return 1; }
  { uint32_t n; uint8_t *blob = read_file(cfg_path, &n); if (!blob) return 1;
    if (nav_config_load(&N, blob, n)) { fprintf(stderr, "%s\n", N.why); return 1; } }
  int have_learn = 0, have_super = 0;
  if (af_path && pi_path) {
    uint32_t na, np; uint8_t *af = read_file(af_path, &na), *pc = read_file(pi_path, &np); if (!af || !pc) return 1;
    if (!no_learn) { if (learn_init(&LS, &H) || learn_config_load(&LS, pc, np) || learn_airframe(&LS, af, na)) fprintf(stderr, "learning: %s\n", LS.msg); else have_learn = 1; }
    if (!no_super) { if (super_init(&SS, &H) || super_config_load(&SS, pc, np) || super_airframe(&SS, af, na)) fprintf(stderr, "supervisor: %s\n", SS.why_text); else have_super = 1; }
  } else if (af_path || pi_path) fprintf(stderr, "the learning and the supervisor need both --airframe and --pi\n");

  static cargo_state CG; static latch_out LO[CG_MAX]; uint32_t cg_drive = 0, cg_said = 0;
  if (latch_spec) {                                                  /* the latches: each closed to start with */
    char spec[128], err[160]; snprintf(spec, sizeof spec, "%s", latch_spec); int n = 0;
    if (us_closed < 500 || us_closed > 2500 || us_open < 500 || us_open > 2500) { fprintf(stderr, "--latch-us: pulses from 500 to 2500 µs\n"); return 2; }
    for (char *tok = strtok(spec, ","); tok && n < CG_MAX; tok = strtok(0, ","), n++)
      if (latch_hw_open(&LO[n], tok, 1, us_closed, us_open, err, sizeof err)) { fprintf(stderr, "latch %d: %s\n", n + 1, err); return 1; }
    cargo_init(&CG, n, (1u << n) - 1, 0); cg_drive = cargo_drive(&CG); cg_said = CG.nmsg;
  }
  int link = open_serial(link_dev, baud); if (link < 0) return 1;
  int crsf = -1;
  if (crsf_dev) { crsf = open_serial(crsf_dev, 115200); if (crsf < 0) return 1; if (serial_custom_baud(crsf, CRSF_BAUD)) fprintf(stderr, "%s: couldn't set %d baud\n", crsf_dev, CRSF_BAUD); }
  static tlm_store TS; static tlm_watch TW; static rc_input RCI; static crsf_parser CP; static rc_pilot RP;
  tlm_init(&TS); tlm_watch_init(&TW); rc_pilot_init(&RP);
  double tlm_want = -10, next_pub = 0, next_radio = 0, next_pack = 0; nav_sp last_sp; memset(&last_sp, 0, sizeof last_sp);
  int gps = gps_dev ? open_serial(gps_dev, gps_baud) : -1;
  int udp = socket(AF_INET, SOCK_DGRAM, 0);
  struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons((uint16_t)port), .sin_addr.s_addr = htonl(INADDR_ANY) };
  if (udp >= 0 && bind(udp, (struct sockaddr *)&addr, sizeof addr)) { perror("udp"); close(udp); udp = -1; }
  fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
  printf("navigation: %s, link %s at %d, GPS %s, commands on stdin%s; learning %s, supervisor %s; radio %s; latches %d\n", N.why, link_dev, baud, gps_dev ? gps_dev : "none", udp >= 0 ? " and UDP" : "",
         have_learn ? "on" : "off", have_super ? "on" : "off", crsf_dev ? crsf_dev : "on the ESP32, if it has one", CG.n);

  static uint8_t rxbuf[4096]; rn_link L; rn_link_init(&L, rxbuf, sizeof rxbuf);
  gps_t G; memset(&G, 0, sizeof G);
  pilot_t P; memset(&P, 0, sizeof P);
  nav_out o; memset(&o, 0, sizeof o);
  double last_nav = 0, last_send = 0, last_fix_t = 0, last_want = 0, fc_state_t = 0, t_start = now_s(); float fc_state = 0;
  int in_control = 0;   /* this program flies it: the drone was on the ground when we started, or a pilot (radio, text) took over since */ uint32_t seen_log = 0;
  int in_fd = 0, nav_got = 0; float nav_v[16];
  static pickup_state PKt; pickup_init(&PKt); uint32_t pk_seen[2] = { 0, 0 };   /* a pickup from a text command (the radio's is in RP) */
  for (;;) {
    struct pollfd pf[5] = { { link, POLLIN, 0 }, { gps, POLLIN, 0 }, { in_fd, POLLIN, 0 }, { udp, POLLIN, 0 }, { crsf, POLLIN, 0 } };
    poll(pf, 5, 5);
    double t = now_s();
    if (crsf >= 0 && (pf[4].revents & POLLIN)) { uint8_t b[256]; ssize_t n = read(crsf, b, sizeof b); for (ssize_t i = 0; i < n; i++) tlm_crsf_input(&CP, b[i], &RCI, t); }
    if (gps >= 0 && (pf[1].revents & POLLIN)) gps_read(&G, gps);
    for (int k = 2; k < 4; k++) if (pf[k].fd >= 0 && (pf[k].revents & POLLIN)) {   /* the pilot */
      static char in[512]; static int in_n; char dg[512]; struct sockaddr_in from; socklen_t fl = sizeof from; ssize_t n;
      char *buf = k == 2 ? in : dg; int have = k == 2 ? in_n : 0, cap = k == 2 ? (int)sizeof in : (int)sizeof dg;
      if (k == 2) n = read(0, buf + have, (size_t)(cap - 1 - have)); else n = recvfrom(udp, buf, (size_t)cap - 1, 0, (struct sockaddr *)&from, &fl);
      if (n == 0 && k == 2) in_fd = -1;                                 /* the end of standard input (a service, /dev/null): no more of it */
      if (n <= 0) continue;
      have += (int)n; buf[have] = 0;
      if (k == 3 && buf[have - 1] != '\n') buf[have++] = '\n';            /* a datagram is a whole line (or several) */
      char *s = buf, *nl;
      while ((nl = memchr(s, '\n', (size_t)(buf + have - s)))) {      /* one command per line */
        *nl = 0; if (nl > s && nl[-1] == '\r') nl[-1] = 0;
        if (*s) {
          char reply[600]; float px, py, pz; int pl = 1;
          if (sscanf(s, "pickup %f %f %f %d", &px, &py, &pz, &pl) >= 3) {
            float hd = P.sp.heading, c = cosf(hd), sn = sinf(hd), spot[3] = { px - (c * hook[0] - sn * hook[1]), py - (sn * hook[0] + c * hook[1]), pz - hook[2] + 0.03f };
            if (!P.fly) snprintf(reply, sizeof reply, "pickup: take off first");
            else if (pickup_start(&PKt, spot, hd, pl - 1, &o, t)) snprintf(reply, sizeof reply, "%s", PKt.msg);
            else { PKt.said = 0; snprintf(reply, sizeof reply, "pickup: flying over it, then down onto it; any other command stops it"); }
            in_control = 1;
          } else if (!cargo_line(&CG, LO, s, reply, sizeof reply) && !task_line(&LS, have_learn, &SS, have_super, &P, &o, s, reply, sizeof reply)) { pilot_line(&P, &N, &o, s, reply, sizeof reply); in_control = 1; pickup_cancel(&PKt, "another command"); PKt.said = 0; }
          if (k == 2) printf("%s\n", reply); else sendto(udp, reply, strlen(reply), 0, (struct sockaddr *)&from, fl);
        }
        s = nl + 1;
      }
      if (k == 2) { in_n = (int)(buf + have - s); if (in_n >= (int)sizeof in - 1) in_n = 0; memmove(in, s, (size_t)in_n); }
    }
    uint8_t rx[512]; ssize_t n = (pf[0].revents & POLLIN) ? read(link, rx, sizeof rx) : 0;
    for (ssize_t i = 0; i < n; i++) {
      int type = rn_link_feed(&L, rx[i]);
      if (type == RN_LINK_EVENT || type == RN_LINK_REPORT) printf("esp32: %.*s\n", (int)L.len, (char *)L.buf);
      if (type == RN_LINK_RC) { rc_unpack(&RCI, (const float *)L.buf, (int)(L.len / 4), t); continue; }          /* the ESP32's radio */
      if (type == RN_LINK_TLM && crsf >= 0) { tlm_unpack(&TS, (const float *)L.buf, (int)(L.len / 4), t); continue; }   /* the ESP32's telemetry, for our radio */
      if (type == RN_LINK_WANT && L.len == 4) { float w; memcpy(&w, L.buf, 4); if ((int)w & 2) tlm_want = t; continue; }
      if ((type == RN_LINK_TELEM && L.len == 144) || (type == RN_LINK_LTEL && L.len >= 8)) { memcpy(&fc_state, L.buf + 4, 4); fc_state_t = t; }   /* the flight core's state */
      if (type == RN_LINK_LTEL && (have_learn || have_super)) {   /* the learning and the supervisor, on every frame */
        static float lt[FC_LTEL_MAX], fo[FC_MODEL_MAX]; int n = (int)(L.len / 4); if (n > FC_LTEL_MAX) continue;
        memcpy(lt, L.buf, (size_t)n * 4);
        if (have_learn) {
          static char was[sizeof LS.msg]; memcpy(was, LS.msg, sizeof was);
          learn_ltel(&LS, lt, n);
          int k = learn_exc_frame(&LS, fo); if (k) send_frame(link, RN_LINK_EXC, fo, (uint32_t)k * 4);
          k = learn_model_frame(&LS, fo); if (k) { send_frame(link, RN_LINK_MODEL, fo, (uint32_t)k * 4); if (have_super) super_model(&SS, fo, k); }
          if (strcmp(was, LS.msg) && LS.msg[0]) printf("learning: %s\n", LS.msg);
          static double said; if (LS.cal && t - said > 10) { if (said > 0) printf("learning: calibrating, %.0f%% done\n", (double)(LS.total > 0 ? 100 * LS.cal_t / LS.total : 0)); said = t; }
          if (!LS.cal) said = 0;
          static int thr_was; if (thr_was && !LS.thr && P.fly) { memcpy(P.sp.target, o.p, sizeof P.sp.target); memset(P.sp.vref, 0, sizeof P.sp.vref); }   /* caught itself: hold there */
          thr_was = LS.thr;
        }
        if (have_super) {
          super_ltel(&SS, lt, n);
          int k = super_set_frame(&SS, fo);
          if (k) { send_frame(link, RN_LINK_SET, fo, (uint32_t)k * 4); nav_set(&N, fo, k); if (have_learn) learn_set(&LS, fo, k); }
          for (; seen_log < SS.log_seq; seen_log++) { uint32_t back = SS.log_seq - 1 - seen_log; if (back < SP_LOG) printf("supervisor: %.1f s %s\n", SS.log[back].t, SS.log[back].text); }
        }
        continue;
      }
      if (type == RN_LINK_NAV && L.len == 64) { memcpy(nav_v, L.buf, 64); nav_got = 1; fc_state = nav_v[1]; fc_state_t = t; }
    }
    /* the navigation: a step on the newest NAV frame (two in one read: the older is passed over, so no step is made of
     * no time), and a guided command from it */
    if (nav_got) {
      nav_got = 0; const float *v = nav_v;
      nav_in in; memset(&in, 0, sizeof in);
      memcpy(in.q, v + 2, 16); memcpy(in.w, v + 6, 12); memcpy(in.acc, v + 9, 12); in.have_att = v[14] > 0.5f;
      in.have_baro = v[13] > 0.5f; in.baro_alt = v[12]; in.baro_age = 0.005f;
      if (G.fix && t - G.t < 1.0) { in.have_fix = 1; memcpy(in.fix_p, G.p, sizeof in.fix_p); memcpy(in.fix_v, G.v, sizeof in.fix_v); in.fix_age = (float)(t - G.t) + 0.1f; last_fix_t = G.t; }
      float dt = last_nav > 0 ? (float)(t - last_nav) : 0.01f; last_nav = t; if (dt > 0.1f) dt = 0.1f;
      /* landing: sink at 0.5 m/s; on the ground (height near home and not sinking any more), idle and disarm */
      if (P.landing && P.fly) { P.sp.vref[2] = -0.5f; P.sp.target[2] = o.p[2] - 0.3f; if (o.p[2] < 0.15f && fabsf(o.v[2]) < 0.1f) { P.fly = 0; P.landing = 0; P.arm = 0; P.sp.fly = 0; printf("landed\n"); } }
      P.sp.fly = P.fly;
      /* the radio's channels fly it while they come (the text commands are for when there is no radio) */
      int radio = rc_link_ok(&RCI, t) || RP.lost, radio_arm = 0; nav_sp rsp = P.sp;
      if (RCI.frames) { radio_arm = rc_pilot_step(&RP, &RCI, t, &N, &o, dt, &rsp); if (RP.said) { RP.said = 0; printf("radio: %s\n", RP.msg); tlm_text(&TS, 4, RP.msg); }
        if (RP.learn_req) { int c = RP.learn_req; RP.learn_req = 0; if (have_learn) { learn_command(&LS, c); printf("radio: learning command %d\n", c); } } }
      if (radio) { P.arm = radio_arm; P.fly = rsp.fly; P.sp = rsp; pickup_cancel(&PKt, "the radio has it"); PKt.said = 0; }
      else if (pickup_active(&PKt)) { if (!P.fly) pickup_cancel(&PKt, "not flying"); else pickup_step(&PKt, &o, t, dt, &P.sp); }
      if (PKt.said) { PKt.said = 0; printf("%s\n", PKt.msg); tlm_text(&TS, 6, PKt.msg); }
      for (int w = 0; w < 2; w++) {                                    /* the pickups' requests to the cargo task */
        const pickup_state *K = w ? &PKt : &RP.pk; if (K->nreq == pk_seen[w]) continue; pk_seen[w] = K->nreq;
        if (CG.n) cargo_command(&CG, K->req_latch, K->req_act, "pickup"); else printf("pickup: no latches here (--latch): it can't close one\n");
      }
      int e = nav_step(&N, &in, &P.sp, dt, &o);
      if (e < 0) { printf("navigation formula failed: stopping commands (the ESP32 lands)\n"); P.fly = 0; }
      else {
        { static int was_landed; if (o.landed && !was_landed) printf("%s\n", N.rc_rth ? "landed by itself (the radio link is lost): disarmed" : "the supervisor landed it: disarmed"); was_landed = o.landed; }
        if (o.landed && P.arm) { P.arm = P.fly = 0; P.sp.fly = 0; }
        if (P.fly && !o.fly && !o.ready) { static double said; if (t - said > 2) { printf("waiting for the position to settle before taking off\n"); said = t; } }
        if (!radio && !pickup_active(&PKt)) for (int k = 0; k < 3; k++) P.sp.target[k] += P.sp.vref[k] * dt;   /* the target moves at the commanded velocity (the radio's pilot moves its own) */
        last_sp = P.sp;
        /* nothing to step on (no attitude in this frame): in the air no new command, the ESP32 flies on the last one */
        /* started (again) while the drone flies: our pilot knows nothing yet (disarmed), and a command from it would
         * disarm it in the air. Silent until the radio's channels or a text command come; meanwhile its failsafe lands it. */
        int core_flying = fc_state == FC_ARMED || fc_state == FC_FAILSAFE;
        if (!in_control && (!core_flying || (radio && RCI.frames))) in_control = 1;
        if (!in_control) { static int said; if (!said) { said = 1; printf("the drone is flying and this program just started: not commanding it until the radio's channels or a command come (its failsafe lands it meanwhile)\n"); } }
        else if (!(e > 0 && o.fly)) {
          float c[12] = { (float)P.arm, 0, 0, 0, o.fly ? 1.0f : 0.0f, -1, 0, 1, o.acc[0], o.acc[1], o.acc[2], o.heading };
          uint8_t fr[96]; uint32_t len = rn_link_frame(fr, sizeof fr, RN_LINK_CMD, (uint8_t *)c, sizeof c);
          if (write(link, fr, len) < 0 && errno != EAGAIN) perror("link");
          last_send = t;
        }
      }
    }
    if (CG.n) {                                                       /* the cargo: the radio's LATCH commands, the moves, the outputs */
      static double cg_t; float cdt = cg_t > 0 ? (float)(t - cg_t) : 0; cg_t = t;
      if (RCI.frames || RCI.cmd_seq) cargo_from_rc(&CG, &RCI, t);
      cargo_step(&CG, cdt);
      uint32_t d = cargo_drive(&CG);
      for (int i = 0; i < CG.n; i++) if (((d ^ cg_drive) >> i) & 1) latch_hw_set(&LO[i], (d >> i) & 1);
      cg_drive = d;
      if (CG.nmsg != cg_said) { cg_said = CG.nmsg; printf("cargo: %s\n", CG.msg); }
    }
    if ((have_learn || have_super || crsf >= 0) && t - last_want > 0.5) { float w = (float)((have_learn || have_super ? 1 : 0) | (crsf >= 0 ? 2 : 0)); send_frame(link, RN_LINK_WANT, &w, 4); last_want = t; }   /* LTEL, and the ESP32's telemetry for our radio, please */
    /* the telemetry: our tasks' items, then down our radio, or to the ESP32's when it asks */
    if (t >= next_pub) {
      next_pub = t + 0.01;
      tlm_from_nav(&TS, &TW, &N, &o, &last_sp, RP.level, t);
      if (have_learn) tlm_from_learn(&TS, &TW, &LS, t);
      if (have_super) tlm_from_super(&TS, &TW, &SS, t);
      if (CG.n) tlm_from_cargo(&TS, &CG, t);
      if (G.fix && G.t > last_fix_t - 1) { static double gps_t; if (G.t != gps_t) { gps_t = G.t; tlm_from_gps(&TS, G.lat, G.lon, (float)G.alt, sqrtf(G.v[0] * G.v[0] + G.v[1] * G.v[1]), atan2f(-G.v[1], G.v[0]) * 57.29578f + (G.v[1] > 0 ? 360 : 0), G.sats, t); } }
      if (crsf >= 0) tlm_from_link(&TS, &RCI, t);
    }
    if (crsf >= 0 && t >= next_radio) {
      next_radio = t + 0.005; static uint8_t out[512];
      int n = tlm_service(&TS, &tlm_crsf, t, tlm_crsf_budget_now(elrs_rate, elrs_ratio, &RCI, t), out, sizeof out);
      if (n && write(crsf, out, (size_t)n) < 0 && errno != EAGAIN) perror("crsf");
    } else if (crsf < 0 && t - tlm_want < 1 && t >= next_pack) {
      next_pack = t + 0.05; static float pk[TLM_PACK_MAX]; int n = tlm_pack(&TS, pk, TLM_PACK_MAX); if (n) send_frame(link, RN_LINK_TLM, pk, (uint32_t)n * 4);
    }
    if (P.arm && t - last_nav > 0.3) P.arm = P.fly = 0;   /* lost the drone's telemetry: nothing to fly on */
    /* No navigation telemetry (the ESP32 sends it while guided commands come): announce ourselves, disarmed. Never
     * while it may be flying: commands that stop land it (its failsafe), a disarm would drop it. Every frame it sends
     * says its state (NAV, LTEL, its telemetry); with none for longer than its failsafe takes to land, or none at all
     * 2 s after starting, it counts as on the ground. */
    int flying = (fc_state == FC_ARMED || fc_state == FC_FAILSAFE) && fc_state_t > 0 && t - fc_state_t < 150;
    if (t - last_send > 0.1 && t - last_nav > 0.1 && !flying && (fc_state_t > 0 || t - t_start > 2)) {
      float c[12] = { 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0 };
      uint8_t fr[96]; uint32_t len = rn_link_frame(fr, sizeof fr, RN_LINK_CMD, (uint8_t *)c, sizeof c);
      if (write(link, fr, len) < 0 && errno != EAGAIN) perror("link");
      last_send = t;
    }
    (void)last_fix_t;
  }
}
