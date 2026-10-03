/* End-to-end test of the command module with the drone's code, on a PC, in real time:
 *   ./dfb_ground  ⇄ pty ⇄  [this test: the transmitter module, the air, the receiver]  ⇄ pty ⇄  ../pi/dfb_pi --crsf
 * and behind dfb_pi a fake ESP32 (the real flight core flying test_nav.c's plant, its telemetry items when asked) and
 * a fake GPS, as in pi/test_dfb_pi.c. The test plays only what the two ExpressLRS modules do: the newest channel frame
 * goes to the receiver 250 times a second, commands and telemetry pass whole, both sides get link statistics.
 * It drives dfb_ground over UDP as a script would ("press arm", then "press fly", "goto", "press fwd", "calibrate",
 * "status"), and checks the drone takes off, goes there, follows the stick, starts the learning's calibration from
 * the radio, that the telemetry comes back to the command module, and that when the radio goes quiet the drone flies
 * home and lands by itself, and the command module raises its alarm.
 *   sh build.sh && sh ../pi/build.sh && cc -O2 -I.. -I../fc -o test_ground_e2e test_ground_e2e.c ../fc/nav_core.c ../fc/fc_core.c \
 *     ../fc/tlm_core.c ../fc/tlm_sources.c ../fc/learn_core.c ../fc/super_core.c ../fc/rc_core.c ../fc/crsf.c ../rn_host.c ../rn.c ../rn_link.c ../rn_builtin.c -lm -lutil && ./test_ground_e2e */
#define _DEFAULT_SOURCE
#define main main_orig
#include "../fc/test_nav.c"
#undef main
#include "rn_link.h"
#include "tlm_sources.h"
#include "crsf.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pty.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static void write_file(const char *path, const void *p, size_t n) { FILE *f = fopen(path, "wb"); fwrite(p, 1, n, f); fclose(f); }
static void write_pi_config(const char *path, int nm) {   /* as pi/test_dfb_pi.c */
  float f[6 + 6 * FC_MAX_MOTORS + 3]; int n = 0;
  f[n++] = 0; f[n++] = 0; f[n++] = 0; f[n++] = 1.2f; f[n++] = 25; f[n++] = (float)nm;
  for (int i = 0; i < nm; i++) { f[n++] = 0; f[n++] = 90; f[n++] = 0.1f; f[n++] = 2500; f[n++] = 25; f[n++] = 0.4f; }
  f[n++] = 4; f[n++] = 0.02f; f[n++] = 60;
  static uint8_t b[8 + sizeof f + 4]; uint32_t magic = 0x434C4644u, ver = 1;
  memcpy(b, &magic, 4); memcpy(b + 4, &ver, 4); memcpy(b + 8, f, (size_t)n * 4);
  uint32_t crc = rn_crc32(b, 8 + (uint32_t)n * 4); memcpy(b + 8 + n * 4, &crc, 4);
  write_file(path, b, 12 + (size_t)n * 4);
}
static void send_frame(int fd, uint8_t type, const void *p, uint32_t n) {
  static uint8_t fr[8192]; uint32_t len = rn_link_frame(fr, sizeof fr, type, (const uint8_t *)p, n); if (len) (void)!write(fd, fr, len);
}
static void step_ms(void) {
  fc_imu m; memset(&m, 0, sizeof m); for (int j = 0; j < 3; j++) { m.gyro[j] = (float)B.w[j]; m.acc[j] = (float)acc_b[j]; }
  if (B.t == 0) m.acc[2] = 9.81f;
  m.have_gyro = 1; m.have_baro = (lround(B.t * 1000) % 40) == 0; m.baro_alt = (float)(B.p[2] + 50);
  rn_host_tick(&HF, 0.001f); fc_step(&F, &m, 0.001f, 0, &O); plant_step(&B, &F.A, &O, 0.001);
}
static int udp; static struct sockaddr_in to;
static void say(const char *s) { sendto(udp, s, strlen(s), 0, (struct sockaddr *)&to, sizeof to); }
static pid_t spawn(const char *prog, char *const argv[], int *out_fd) {
  int p[2]; if (pipe(p)) return -1;
  pid_t pid = fork();
  if (!pid) { int nul = open("/dev/null", O_RDONLY); dup2(nul, 0); dup2(p[1], 1); dup2(p[1], 2); execv(prog, argv); _exit(1); }
  close(p[1]); fcntl(p[0], F_SETFL, O_NONBLOCK); *out_fd = p[0];
  return pid;
}
static int lines(int fd, char *buf, int *n, const char *tag, void (*on)(const char *)) {
  ssize_t r = read(fd, buf + *n, 8191 - (size_t)*n); if (r > 0) *n += (int)r;
  int got = 0;
  for (char *nl; (nl = memchr(buf, '\n', (size_t)*n)); got++) {
    *nl = 0; char *s = buf; while (*s == '\r') s++; char *e = s + strlen(s); while (e > s && e[-1] == '\r') *--e = 0;
    if (*s) { printf("    %s | %s\n", tag, s); fflush(stdout); on(s); }
    int k = (int)(nl - buf) + 1; memmove(buf, nl + 1, (size_t)(*n - k)); *n -= k;
  }
  return got;
}
static int learn_line, cal_line, lost_line, landed_line, alarm_line, prog_line;
static void on_pi(const char *s) { if (strstr(s, "learning command 1")) learn_line = 1; if (strstr(s, "calibrating")) cal_line = 1; if (strstr(s, "radio link lost: flying home")) lost_line = 1; if (strstr(s, "landed by itself")) landed_line = 1; }
static void on_gnd(const char *s) { if (strstr(s, "ALARM: no telemetry")) alarm_line = 1; if (strstr(s, "drone: ")) prog_line++; }

int main(void) {
  uint32_t lq; uint8_t *quad = read_file("../fc/testdata/quadx.dfa", &lq);
  start(quad, lq, 0);
  write_file("/tmp/test_ground.dnc", cfg_blob, sizeof cfg_blob); write_file("/tmp/test_ground.dfa", quad, lq); write_pi_config("/tmp/test_ground.dlc", F.A.n_motors);
  struct termios t; cfmakeraw(&t);
  int lm, ls, gm, gs, cm, cs, tm, ts; char lname[64], gname[64], cname[64], tname[64];
  if (openpty(&lm, &ls, lname, &t, NULL) || openpty(&gm, &gs, gname, &t, NULL) || openpty(&cm, &cs, cname, &t, NULL) || openpty(&tm, &ts, tname, &t, NULL)) { perror("openpty"); return 1; }
  int pi_out, gnd_out;
  char *pa[] = { "dfb_pi", "--link", lname, "--nav", "/tmp/test_ground.dnc", "--airframe", "/tmp/test_ground.dfa", "--pi", "/tmp/test_ground.dlc", "--port", "14598", "--gps", gname, "--crsf", cname, 0 };
  char *ga[] = { "dfb_ground", "--tx", tname, "--port", "14597", 0 };
  pid_t pi = spawn("../pi/dfb_pi", pa, &pi_out), gnd = spawn("./dfb_ground", ga, &gnd_out);
  fcntl(lm, F_SETFL, O_NONBLOCK); fcntl(cm, F_SETFL, O_NONBLOCK); fcntl(tm, F_SETFL, O_NONBLOCK);
  udp = socket(AF_INET, SOCK_DGRAM, 0); to.sin_family = AF_INET; to.sin_port = htons(14597); to.sin_addr.s_addr = htonl(INADDR_LOOPBACK); fcntl(udp, F_SETFL, O_NONBLOCK);
  static uint8_t lbuf[8192]; rn_link L; rn_link_init(&L, lbuf, sizeof lbuf);
  static tlm_store TS; static tlm_watch TW; tlm_init(&TS); tlm_watch_init(&TW);
  crsf_parser from_gnd, from_rx; memset(&from_gnd, 0, sizeof from_gnd); memset(&from_rx, 0, sizeof from_rx);
  float ch[16]; int have_ch = 0; for (int i = 0; i < 16; i++) ch[i] = -1;
  double t0 = now_s(), next = t0, want_tlm = -10, next_pack = 0, next_rc = 0, next_stats = 0, next_gps = 0;
  int radio = 1, cmds_up = 0, tlm_down = 0; char pbuf[8192], gbuf[8192]; int pn = 0, gn = 0;
  double armed_at = 0, goto_at = 0, fwd_at = 0, cal_at = 0, quiet_at = 0; float x0 = 0, x1 = 0; char status[600] = "";
  int ok_takeoff = 0, ok_goto = 0;
  while (now_s() - t0 < 85) {
    double el = now_s() - t0;
    lines(pi_out, pbuf, &pn, "pi", on_pi); lines(gnd_out, gbuf, &gn, "ground", on_gnd);
    { char r[600]; struct sockaddr_in f; socklen_t fl = sizeof f; ssize_t n = recvfrom(udp, r, sizeof r - 1, 0, (struct sockaddr *)&f, &fl); if (n > 0) { r[n] = 0; if (strstr(r, "uplink")) snprintf(status, sizeof status, "%s", r); } }
    /* the fake ESP32: commands from the Pi; its telemetry items when asked */
    uint8_t rx[1024]; ssize_t n = read(lm, rx, sizeof rx);
    for (ssize_t i = 0; i < n; i++) {
      int type = rn_link_feed(&L, rx[i]);
      if (type == RN_LINK_CMD && L.len == 48) { float v[12]; memcpy(v, L.buf, 48); fc_cmd c = { v[0] > 0.5f, v[1], v[2], v[3], v[4], -1, v[6], v[7] > 0.5f, { v[8], v[9], v[10] }, v[11] }; fc_command(&F, &c); }
      else if (type == RN_LINK_WANT && L.len == 4) { float w; memcpy(&w, L.buf, 4); if ((int)w & 2) want_tlm = el; }
      else if (type == RN_LINK_EXC) fc_exc(&F, (const float *)L.buf, (int)(L.len / 4));
      else if (type == RN_LINK_MODEL) fc_model(&F, (const float *)L.buf, (int)(L.len / 4));
      else if (type == RN_LINK_SET) fc_set(&F, (const float *)L.buf, (int)(L.len / 4));
    }
    while (now_s() < next) usleep(200);
    next += 0.01;
    for (int k = 0; k < 10; k++) { step_ms(); if (k % 5 == 4 && el - want_tlm < 1.0) { static float lt[FC_LTEL_MAX]; int m = fc_ltel(&F, lt); send_frame(lm, RN_LINK_LTEL, lt, (uint32_t)m * 4); } }
    float nb[16] = { (float)F.t, (float)F.state, F.q[0], F.q[1], F.q[2], F.q[3], F.w[0], F.w[1], F.w[2], (float)acc_b[0], (float)acc_b[1], (float)acc_b[2], F.alt_e, (float)F.have_alt, (float)F.att_ok, 0 };
    send_frame(lm, RN_LINK_NAV, nb, 64);
    tlm_from_core(&TS, &TW, &F, el);
    if (el - want_tlm < 1.0 && el >= next_pack) { next_pack = el + 0.05; static float pk[2048]; int m = tlm_pack(&TS, pk, 2048); if (m) send_frame(lm, RN_LINK_TLM, pk, (uint32_t)m * 4); }
    if (el >= next_gps) {
      next_gps += 0.2;
      double lat = 41.0 + (B.p[0] + 0.2 * gauss()) / 6371000.0 * 180 / M_PI, lon = 29.0 - (B.p[1] + 0.2 * gauss()) / (6371000.0 * cos(41.0 * M_PI / 180)) * 180 / M_PI;
      int la = (int)lat, lo = (int)lon; char g[200], rm[200];
      snprintf(g, sizeof g, "$GPGGA,120000.00,%02d%08.5f,N,%03d%08.5f,E,1,08,1.0,%.1f,M,0,M,,*00\r\n", la, (lat - la) * 60, lo, (lon - lo) * 60, 100 + B.p[2]);
      double sp = hypot(B.v[0], B.v[1]) / 0.514444, crs = atan2(-B.v[1], B.v[0]) * 180 / M_PI; if (crs < 0) crs += 360;
      snprintf(rm, sizeof rm, "$GPRMC,120000.00,A,%02d%08.5f,N,%03d%08.5f,E,%.3f,%.1f,021026,,,A*00\r\n", la, (lat - la) * 60, lo, (lon - lo) * 60, sp, crs);
      (void)!write(gm, g, strlen(g)); (void)!write(gm, rm, strlen(rm));
    }
    /* the two radio modules and the air between them */
    uint8_t b[1024];
    n = read(tm, b, sizeof b);                                         /* from the command module */
    for (ssize_t i = 0; i < n; i++) if (crsf_feed(&from_gnd, b[i]) > 0) {
      if (crsf_type(&from_gnd) == CRSF_RC) { crsf_rc_read(crsf_payload(&from_gnd), ch); have_ch = 1; }
      else if (radio && crsf_type(&from_gnd) == CRSF_EXT) { uint8_t f[64]; int len = crsf_frame(f, CRSF_ADDR_FC, CRSF_EXT, crsf_payload(&from_gnd), crsf_payload_len(&from_gnd)); (void)!write(cm, f, (size_t)len); cmds_up++; }
    }
    if (radio && have_ch && el >= next_rc) { next_rc = el + 0.004; uint8_t f[64]; int len = crsf_rc(f, CRSF_ADDR_FC, ch); (void)!write(cm, f, (size_t)len); }
    n = read(cm, b, sizeof b);                                         /* from the drone's receiver port */
    for (ssize_t i = 0; i < n; i++) { int len = crsf_feed(&from_rx, b[i]); if (len > 0 && radio) { (void)!write(tm, from_rx.buf, (size_t)len); tlm_down++; } }
    if (radio && el >= next_stats) {
      next_stats = el + 0.1; crsf_link Lk = { -55, 100, 9, -56, 100, 8, 2, 100, 0 }; uint8_t f[64];
      int len = crsf_link_stats(f, CRSF_ADDR_FC, &Lk); (void)!write(cm, f, (size_t)len);
      len = crsf_link_stats(f, CRSF_ADDR_HANDSET, &Lk); (void)!write(tm, f, (size_t)len);
    }
    /* the pilot's script, over UDP */
    if (el > 2 && !armed_at) { say("press arm"); armed_at = el; }                 /* arm, then take off, as a pilot does */
    { static int flew; if (armed_at && !flew && el > armed_at + 1) { say("press fly"); flew = 1; } }
    if (armed_at && !goto_at && el > armed_at + 13) { ok_takeoff = B.p[2] > 1.2 && F.state == FC_ARMED; printf("  after take-off: z %.2f, %s\n", B.p[2], fc_state_name(F.state)); say("goto 2 1 2 0"); goto_at = el; }
    if (goto_at && !fwd_at && el > goto_at + 10) { ok_goto = hypot(B.p[0] - 2, B.p[1] - 1) < 0.5 && fabs(B.p[2] - 2) < 0.4; printf("  after the go-to: (%.2f %.2f %.2f)\n", B.p[0], B.p[1], B.p[2]); x0 = (float)B.p[0]; fwd_at = el; }
    if (fwd_at && el < fwd_at + 2) { static double nf; if (el >= nf) { nf = el + 0.05; say("press fwd"); } }
    if (fwd_at && !cal_at && el > fwd_at + 5) { x1 = (float)B.p[0]; say("status"); say("calibrate"); cal_at = el; }
    if (cal_at && !quiet_at && el > cal_at + 5) { quiet_at = el; radio = 0; printf("  the radio goes quiet at (%.2f %.2f %.2f)\n", B.p[0], B.p[1], B.p[2]); }
    if (quiet_at && el > quiet_at + 22) break;
  }
  kill(gnd, SIGTERM); kill(pi, SIGTERM); usleep(300000);
  lines(pi_out, pbuf, &pn, "pi", on_pi); lines(gnd_out, gbuf, &gn, "ground", on_gnd);
  printf("  status from the command module: %s\n", status);
  printf("  commands up %d, telemetry frames down %d; the stick moved it %.2f m forward\n", cmds_up, tlm_down, x1 - x0);
  printf("  after the radio went quiet: at (%.2f %.2f %.2f), %s; Pi: lost line %d, landed %d; command module alarm %d\n", B.p[0], B.p[1], B.p[2], fc_state_name(F.state), lost_line, landed_line, alarm_line);
  int ok = ok_takeoff && ok_goto && x1 - x0 > 1.5 && learn_line && cal_line && strstr(status, "telemetry |") && strstr(status, "uplink 100%") && strstr(status, "POSHOLD")
        && tlm_down > 200 && prog_line > 0 && lost_line && landed_line && alarm_line && hypot(B.p[0], B.p[1]) < 1.0 && B.p[2] < 0.2 && F.state == FC_DISARMED;
  printf(ok ? "command module end-to-end: ok\n" : "command module end-to-end: FAIL (take-off %d, goto %d, stick %d, learning %d/%d, status %d, telemetry %d, lost %d, landed %d, alarm %d)\n",
         ok_takeoff, ok_goto, x1 - x0 > 1.5, learn_line, cal_line, strstr(status, "POSHOLD") != 0, tlm_down, lost_line, landed_line, alarm_line);
  return ok ? 0 : 1;
}
