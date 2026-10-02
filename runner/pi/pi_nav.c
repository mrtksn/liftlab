/*
 * Drone Force Bench navigation on the Raspberry Pi: holds and moves the drone's position.
 *
 * The same code the simulator runs for a Pi board: nav_core.c with the step runner and the built-in flight program
 * (the one the ESP32 has). It talks to the ESP32 flight controller over the serial link (rn_link.h):
 *   - the ESP32 sends RN_LINK_NAV 100 times a second while guided commands come: attitude, rates, accelerometer,
 *     barometer height;
 *   - for each one, this runs a navigation step and sends a guided command (RN_LINK_CMD, 12 floats): the
 *     acceleration wanted and the heading. If they stop (this program stops, the cable comes out), the ESP32 goes to
 *     its failsafe within 0.5 s and lands.
 * A GPS on its own serial port (NMEA: GGA and RMC, as the NEO-6M sends) gives position and velocity; home is where
 * it took off. Without a GPS it navigates on the barometer and the accelerometer alone (it drifts; a downward camera
 * for optical flow is the next step).
 *
 * The pilot's commands are lines of text, on standard input or UDP (port 14560 by default; fly.py or a phone can
 * send them):
 *   arm | disarm | takeoff [height m] | land | goto X Y Z | move VX VY VZ (target velocity, m/s; 0 0 0 stops)
 *   heading DEG | hold | home | status
 *
 * Build:  sh runner/pi/build.sh    Run:  ./pi_nav --link /dev/serial0 --baud 115200 --config drone.dnc [--gps /dev/ttyUSB0]
 * (drone.dnc: the simulator's Computers tab → Export for this board.)
 */
#define _DEFAULT_SOURCE
#include "nav_core.h"
#include "rn_link.h"
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
typedef struct { int have_origin; double lat0, lon0, alt0; float p[3], v[3]; double t; int fix; char line[128]; int n; } gps_t;
static double nmea_deg(const char *f, const char *hemi) {   /* ddmm.mmmm or dddmm.mmmm */
  double x = atof(f); int d = (int)(x / 100); double v = d + (x - d * 100) / 60;
  return (*hemi == 'S' || *hemi == 'W') ? -v : v;
}
static int split(char *s, char **f, int max) { int n = 0; f[n++] = s; for (; *s && n < max; s++) if (*s == ',' || *s == '*') { *s = 0; f[n++] = s + 1; } return n; }
static void gps_line(gps_t *G, char *s) {
  char *f[24]; int n = split(s, f, 24);
  if (n > 9 && !strcmp(f[0] + 2, "GGA")) {                   /* time, lat, N/S, lon, E/W, quality, sats, hdop, altitude */
    if (atoi(f[6]) < 1 || !*f[2] || !*f[4]) { G->fix = 0; return; }
    double lat = nmea_deg(f[2], f[3]), lon = nmea_deg(f[4], f[5]), alt = atof(f[9]);
    if (!G->have_origin) { G->lat0 = lat; G->lon0 = lon; G->alt0 = alt; G->have_origin = 1; }
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
static void pilot_line(pilot_t *P, const nav_out *o, char *s, char *reply, size_t rn) {
  float a, b, c; reply[0] = 0;
  if (!strncmp(s, "arm", 3)) { P->arm = 1; snprintf(reply, rn, "arming"); }
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
  else snprintf(reply, rn, "? arm | disarm | takeoff [h] | land | goto x y z | move vx vy vz | heading deg | hold | home | status");
}

static float arenas_[3][65536], pools_[3][8192];
static int32_t codes_[3][32768];

int main(int argc, char **argv) {
  const char *link_dev = "/dev/serial0", *gps_dev = 0, *cfg_path = 0; int baud = 115200, gps_baud = 9600, port = 14560;
  for (int i = 1; i + 1 < argc; i += 2) {
    if (!strcmp(argv[i], "--link")) link_dev = argv[i + 1]; else if (!strcmp(argv[i], "--baud")) baud = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--gps")) gps_dev = argv[i + 1]; else if (!strcmp(argv[i], "--gps-baud")) gps_baud = atoi(argv[i + 1]);
    else if (!strcmp(argv[i], "--config")) cfg_path = argv[i + 1]; else if (!strcmp(argv[i], "--port")) port = atoi(argv[i + 1]);
  }
  if (!cfg_path) { fprintf(stderr, "usage: pi_nav --config drone.dnc [--link /dev/serial0] [--baud 115200] [--gps /dev/ttyUSB0] [--port 14560]\n"); return 2; }

  static rn_host H; static nav_state N;
  float *arenas[3] = { arenas_[0], arenas_[1], arenas_[2] }, *pools[3] = { pools_[0], pools_[1], pools_[2] };
  int32_t *codes[3] = { codes_[0], codes_[1], codes_[2] };
  int e = rn_host_init(&H, rn_builtin_img, rn_builtin_len, arenas, 65536, codes, 32768, pools, 8192);
  if (e) { fprintf(stderr, "flight program didn't load (%d)\n", e); return 1; }
  if (nav_init(&N, &H)) { fprintf(stderr, "%s\n", N.why); return 1; }
  { static uint8_t blob[256]; FILE *f = fopen(cfg_path, "rb"); if (!f) { perror(cfg_path); return 1; }
    uint32_t n = (uint32_t)fread(blob, 1, sizeof blob, f); fclose(f);
    if (nav_config_load(&N, blob, n)) { fprintf(stderr, "%s\n", N.why); return 1; } }

  int link = open_serial(link_dev, baud); if (link < 0) return 1;
  int gps = gps_dev ? open_serial(gps_dev, gps_baud) : -1;
  int udp = socket(AF_INET, SOCK_DGRAM, 0);
  struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons((uint16_t)port), .sin_addr.s_addr = htonl(INADDR_ANY) };
  if (udp >= 0 && bind(udp, (struct sockaddr *)&addr, sizeof addr)) { perror("udp"); close(udp); udp = -1; }
  fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
  printf("navigation: %s, link %s at %d, GPS %s, commands on stdin%s\n", N.why, link_dev, baud, gps_dev ? gps_dev : "none", udp >= 0 ? " and UDP" : "");

  static uint8_t rxbuf[4096]; rn_link L; rn_link_init(&L, rxbuf, sizeof rxbuf);
  gps_t G; memset(&G, 0, sizeof G);
  pilot_t P; memset(&P, 0, sizeof P);
  nav_out o; memset(&o, 0, sizeof o);
  double last_nav = 0, last_send = 0, last_fix_t = 0; float fc_state = 0;
  for (;;) {
    struct pollfd pf[4] = { { link, POLLIN, 0 }, { gps, POLLIN, 0 }, { 0, POLLIN, 0 }, { udp, POLLIN, 0 } };
    poll(pf, 4, 10);
    double t = now_s();
    if (gps >= 0 && (pf[1].revents & POLLIN)) gps_read(&G, gps);
    for (int k = 2; k < 4; k++) if (pf[k].fd >= 0 && (pf[k].revents & POLLIN)) {   /* the pilot */
      char line[200]; struct sockaddr_in from; socklen_t fl = sizeof from; ssize_t n;
      if (k == 2) n = read(0, line, sizeof line - 1); else n = recvfrom(udp, line, sizeof line - 1, 0, (struct sockaddr *)&from, &fl);
      if (n <= 0) continue;
      line[n] = 0;
      char reply[200]; pilot_line(&P, &o, line, reply, sizeof reply);
      if (k == 2) { printf("%s\n", reply); fflush(stdout); }
      else sendto(udp, reply, strlen(reply), 0, (struct sockaddr *)&from, fl);
    }
    uint8_t rx[512]; ssize_t n = (pf[0].revents & POLLIN) ? read(link, rx, sizeof rx) : 0;
    for (ssize_t i = 0; i < n; i++) {
      int type = rn_link_feed(&L, rx[i]);
      if (type == RN_LINK_EVENT || type == RN_LINK_REPORT) printf("esp32: %.*s\n", (int)L.len, (char *)L.buf);
      if (type != RN_LINK_NAV || L.len != 64) continue;
      float v[16]; memcpy(v, L.buf, 64); fc_state = v[1];
      nav_in in; memset(&in, 0, sizeof in);
      memcpy(in.q, v + 2, 16); memcpy(in.w, v + 6, 12); memcpy(in.acc, v + 9, 12); in.have_att = v[14] > 0.5f;
      in.have_baro = v[13] > 0.5f; in.baro_alt = v[12]; in.baro_age = 0.005f;
      if (G.fix && t - G.t < 1.0) { in.have_fix = 1; memcpy(in.fix_p, G.p, sizeof in.fix_p); memcpy(in.fix_v, G.v, sizeof in.fix_v); in.fix_age = (float)(t - G.t) + 0.1f; last_fix_t = G.t; }
      float dt = last_nav > 0 ? (float)(t - last_nav) : 0.01f; last_nav = t; if (dt > 0.1f) dt = 0.1f;
      /* landing: sink at 0.5 m/s; on the ground (height near home and not sinking any more), idle and disarm */
      if (P.landing && P.fly) { P.sp.vref[2] = -0.5f; P.sp.target[2] = o.p[2] - 0.3f; if (o.p[2] < 0.15f && fabsf(o.v[2]) < 0.1f) { P.fly = 0; P.landing = 0; P.arm = 0; P.sp.fly = 0; printf("landed\n"); } }
      P.sp.fly = P.fly;
      if (nav_step(&N, &in, &P.sp, dt, &o)) { printf("navigation formula failed: stopping commands (the ESP32 lands)\n"); P.fly = 0; continue; }
      if (P.fly && !o.fly && !o.ready) { static double said; if (t - said > 2) { printf("waiting for the position to settle before taking off\n"); said = t; } }
      for (int k = 0; k < 3; k++) P.sp.target[k] += P.sp.vref[k] * dt;   /* the target moves at the commanded velocity */
      float c[12] = { (float)P.arm, 0, 0, 0, o.fly ? 1.0f : 0.0f, -1, 0, 1, o.acc[0], o.acc[1], o.acc[2], o.heading };
      uint8_t fr[96]; uint32_t len = rn_link_frame(fr, sizeof fr, RN_LINK_CMD, (uint8_t *)c, sizeof c);
      if (write(link, fr, len) < 0 && errno != EAGAIN) perror("link");
      last_send = t;
    }
    /* no navigation telemetry yet (the ESP32 sends it once guided commands come): announce ourselves, disarmed */
    if (t - last_send > 0.1) {
      float c[12] = { 0, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0 };
      if (P.arm && t - last_nav > 0.3) P.arm = P.fly = 0;   /* lost the drone's telemetry: nothing to fly on */
      uint8_t fr[96]; uint32_t len = rn_link_frame(fr, sizeof fr, RN_LINK_CMD, (uint8_t *)c, sizeof c);
      if (write(link, fr, len) < 0 && errno != EAGAIN) perror("link");
      last_send = t;
    }
    (void)fc_state; (void)last_fix_t;
  }
}
