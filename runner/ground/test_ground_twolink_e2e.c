/* End-to-end test of two links at once (../fc/radio_mux.c, ../fc/lmux.c), on a PC, in real time: the command module
 * and the drone's Pi over two serial lines (two ptys each way, this test passing the bytes at each line's speed):
 *   ./dfb_ground --radio serial,115200 --tx PTY_A --radio2 serial,57600 --tx2 PTY_B
 *     ⇄  [this test: line A, line B]  ⇄  ../pi/dfb_pi --radio serial,115200 --radio-dev PTY_A --radio2 serial,57600 --radio2-dev PTY_B
 * Behind dfb_pi a fake ESP32 (the real flight core flying test_nav.c's plant) and a fake GPS, as in
 * test_ground_serial_e2e.c. First both lines both ways: take off; line A cut both ways: nothing is lost (no failsafe,
 * the go-to sent then arrives once, the telemetry keeps coming by line B); A back. Then both programs again with line B
 * one way only (serial,57600,up: a laser up): line A's uplink cut, the channels come by the laser and the command
 * module still sees the drone hearing it (by A's downlink); then A cut both ways: the command module has no telemetry
 * (its alarm), but the drone, still getting its channels by the laser, doesn't fail safe.
 *   sh build.sh && sh ../pi/build.sh && cc -O2 -I.. -I../fc -o test_ground_twolink_e2e test_ground_twolink_e2e.c ../fc/nav_core.c ../fc/fc_core.c \
 *     ../fc/tlm_core.c ../fc/tlm_sources.c ../fc/learn_core.c ../fc/super_core.c ../fc/rc_core.c ../fc/pickup_core.c ../fc/crsf.c ../rn_host.c ../rn.c ../rn_link.c ../rn_builtin.c -lm -lutil && ./test_ground_twolink_e2e */
#define _DEFAULT_SOURCE
#define main main_orig
#include "../fc/test_nav.c"
#undef main
#include "rn_link.h"
#include "tlm_sources.h"
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

enum { GND_CMD_PORT = 14585, PI_CMD_PORT = 14586 };

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
static void nmea(char *out, size_t n, const char *body) { int x = 0; for (const char *p = body; *p; p++) x ^= (unsigned char)*p; snprintf(out, n, "$%s*%02X\r\n", body, x); }
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
static void lines(int fd, char *buf, int *n, const char *tag, void (*on)(const char *)) {
  ssize_t r = read(fd, buf + *n, 8191 - (size_t)*n); if (r > 0) *n += (int)r;
  for (char *nl; (nl = memchr(buf, '\n', (size_t)*n));) {
    *nl = 0; char *s = buf; while (*s == '\r') s++; char *e = s + strlen(s); while (e > s && e[-1] == '\r') *--e = 0;
    if (*s) { printf("    %s | %s\n", tag, s); fflush(stdout); on(s); }
    int k = (int)(nl - buf) + 1; memmove(buf, nl + 1, (size_t)(*n - k)); *n -= k;
  }
}
static int lost_line, alarm_line;
static void on_pi(const char *s) { lost_line |= strstr(s, "radio link lost") != 0; }
static void on_gnd(const char *s) { alarm_line |= strstr(s, "ALARM: no telemetry") != 0; }

/* ── two lines: bytes each way at each one's speed; each way cut or not ── */
#define LQN (1 << 16)
typedef struct { uint8_t b[LQN]; double at[LQN]; long head, tail; double free_at; } wire;
typedef struct { wire up, down; int g, d; double byte_s; int cut_up, cut_down; long up_bytes, down_bytes; } line;
static line LN[2];
static void line_in(line *X, wire *W, int from, double t, int cut, long *count) {
  uint8_t b[4096]; ssize_t n;
  while ((n = read(from, b, sizeof b)) > 0)
    for (ssize_t i = 0; i < n; i++) {
      double start = t > W->free_at ? t : W->free_at; W->free_at = start + X->byte_s;
      if (cut) continue;
      if (W->tail - W->head < LQN) { W->b[W->tail % LQN] = b[i]; W->at[W->tail % LQN] = W->free_at; W->tail++; (*count)++; }
    }
}
static void line_out(wire *W, int to_fd, double t) {
  uint8_t b[4096]; int k = 0;
  while (W->head < W->tail && W->at[W->head % LQN] <= t && k < (int)sizeof b) { b[k++] = W->b[W->head % LQN]; W->head++; }
  if (k) (void)!write(to_fd, b, (size_t)k);
}
static void air(void) {
  double t = now_s();
  for (int i = 0; i < 2; i++) {
    line *X = &LN[i];
    line_in(X, &X->up, X->g, t, X->cut_up, &X->up_bytes); line_in(X, &X->down, X->d, t, X->cut_down, &X->down_bytes);
    line_out(&X->up, X->d, t); line_out(&X->down, X->g, t);
  }
}
static void lines_reset(void) { for (int i = 0; i < 2; i++) { LN[i].up.head = LN[i].up.tail = LN[i].down.head = LN[i].down.tail = 0; LN[i].up.free_at = LN[i].down.free_at = 0; LN[i].cut_up = LN[i].cut_down = 0; } }

int main(void) {
  srand(9);
  uint32_t lq; uint8_t *quad = read_file("../fc/testdata/quadx.dfa", &lq);
  start(quad, lq, 0);
  write_file("/tmp/test_two.dnc", cfg_blob, sizeof cfg_blob); write_file("/tmp/test_two.dfa", quad, lq); write_pi_config("/tmp/test_two.dlc", F.A.n_motors);
  struct termios t; cfmakeraw(&t);
  int lm, ls, gm, gs, s0, s1, s2, s3; char lname[64], gname[64], ga_n[64], da_n[64], gb_n[64], db_n[64];
  if (openpty(&lm, &ls, lname, &t, NULL) || openpty(&gm, &gs, gname, &t, NULL) || openpty(&LN[0].g, &s0, ga_n, &t, NULL) || openpty(&LN[0].d, &s1, da_n, &t, NULL)
      || openpty(&LN[1].g, &s2, gb_n, &t, NULL) || openpty(&LN[1].d, &s3, db_n, &t, NULL)) { perror("openpty"); return 1; }
  for (int i = 0; i < 2; i++) { fcntl(LN[i].g, F_SETFL, O_NONBLOCK); fcntl(LN[i].d, F_SETFL, O_NONBLOCK); }
  LN[0].byte_s = 10.0 / 115200; LN[1].byte_s = 10.0 / 57600;
  char ports[2][8]; snprintf(ports[0], 8, "%d", PI_CMD_PORT); snprintf(ports[1], 8, "%d", GND_CMD_PORT);
  char *pa[] = { "dfb_pi", "--link", lname, "--nav", "/tmp/test_two.dnc", "--airframe", "/tmp/test_two.dfa", "--pi", "/tmp/test_two.dlc", "--port", ports[0], "--gps", gname,
                 "--radio", "serial,115200", "--radio-dev", da_n, "--radio2", "serial,57600", "--radio2-dev", db_n, "--bind", "two links", 0 };
  char *ga[] = { "dfb_ground", "--radio", "serial,115200", "--tx", ga_n, "--radio2", "serial,57600", "--tx2", gb_n, "--bind", "two links", "--port", ports[1], "--status", 0 };
  int pi_out, gnd_out;
  pid_t pi = spawn("../pi/dfb_pi", pa, &pi_out), gnd = spawn("./dfb_ground", ga, &gnd_out);
  fcntl(lm, F_SETFL, O_NONBLOCK);
  udp = socket(AF_INET, SOCK_DGRAM, 0); to.sin_family = AF_INET; to.sin_port = htons(GND_CMD_PORT); to.sin_addr.s_addr = htonl(INADDR_LOOPBACK); fcntl(udp, F_SETFL, O_NONBLOCK);
  static uint8_t lbuf[8192]; rn_link L; rn_link_init(&L, lbuf, sizeof lbuf);
  static tlm_store TS; static tlm_watch TW; tlm_init(&TS); tlm_watch_init(&TW);
  double t0 = now_s(), next = t0, want_tlm = -10, next_pack = 0, next_gps = 0;
  char pbuf[8192], gbuf[8192]; int pn = 0, gn = 0;
  enum { ARM, FLY, UP, CUT_A, BACK, RESTART, ARM2, FLY2, UP2, CUT_A_UP, CUT_A_ALL, DONE } ph = ARM; double ph_t = 0;
  char status[600] = "", st_cut[600] = "", st_laser[600] = "", st_all[600] = "";
  int ok_up = 0, ok_cut = 0, ok_goto = 0, ok_up2 = 0, ok_laser = 0, ok_all = 0, lost_cut = 0, lost_laser = 0, alarm_cut = 0; long down_b0 = 0;
  while (now_s() - t0 < 120 && ph != DONE) {
    double el = now_s() - t0;
    lines(pi_out, pbuf, &pn, "pi", on_pi); lines(gnd_out, gbuf, &gn, "ground", on_gnd);
    { char r[600]; struct sockaddr_in f; socklen_t fl = sizeof f; ssize_t n = recvfrom(udp, r, sizeof r - 1, 0, (struct sockaddr *)&f, &fl); if (n > 0) { r[n] = 0; if (strstr(r, "uplink")) snprintf(status, sizeof status, "%s", r); } }
    uint8_t rx[1024]; ssize_t n = read(lm, rx, sizeof rx);
    for (ssize_t i = 0; i < n; i++) {
      int type = rn_link_feed(&L, rx[i]);
      if (type == RN_LINK_CMD && L.len == 48) { float v[12]; memcpy(v, L.buf, 48); fc_cmd c = { v[0] > 0.5f, v[1], v[2], v[3], v[4], -1, v[6], v[7] > 0.5f, { v[8], v[9], v[10] }, v[11] }; fc_command(&F, &c); }
      else if (type == RN_LINK_WANT && L.len == 4) { float w; memcpy(&w, L.buf, 4); if ((int)w & 2) want_tlm = el; }
      else if (type == RN_LINK_EXC) fc_exc(&F, (const float *)L.buf, (int)(L.len / 4));
      else if (type == RN_LINK_MODEL) fc_model(&F, (const float *)L.buf, (int)(L.len / 4));
      else if (type == RN_LINK_SET) fc_set(&F, (const float *)L.buf, (int)(L.len / 4));
    }
    while (now_s() < next) { air(); usleep(200); }
    next += 0.01;
    for (int k = 0; k < 10; k++) { step_ms(); if (k % 5 == 4 && el - want_tlm < 1.0) { static float lt[FC_LTEL_MAX]; int m = fc_ltel(&F, lt); send_frame(lm, RN_LINK_LTEL, lt, (uint32_t)m * 4); } }
    float nb[16] = { (float)F.t, (float)F.state, F.q[0], F.q[1], F.q[2], F.q[3], F.w[0], F.w[1], F.w[2], (float)acc_b[0], (float)acc_b[1], (float)acc_b[2], F.alt_e, (float)F.have_alt, (float)F.att_ok, 0 };
    send_frame(lm, RN_LINK_NAV, nb, 64);
    tlm_from_core(&TS, &TW, &F, el);
    if (el - want_tlm < 1.0 && el >= next_pack) { next_pack = el + 0.05; static float pk[2048]; int m = tlm_pack(&TS, pk, 2048); if (m) send_frame(lm, RN_LINK_TLM, pk, (uint32_t)m * 4); }
    if (el >= next_gps) {
      next_gps += 0.2;
      double lat = 41.0 + (B.p[0] + 0.2 * gauss()) / 6371000.0 * 180 / M_PI, lon = 29.0 - (B.p[1] + 0.2 * gauss()) / (6371000.0 * cos(41.0 * M_PI / 180)) * 180 / M_PI;
      int la = (int)lat, lo = (int)lon; char b[200], g[210], rm[210];
      snprintf(b, sizeof b, "GPGGA,120000.00,%02d%08.5f,N,%03d%08.5f,E,1,08,1.0,%.1f,M,0,M,,", la, (lat - la) * 60, lo, (lon - lo) * 60, 100 + B.p[2]); nmea(g, sizeof g, b);
      double sp = hypot(B.v[0], B.v[1]) / 0.514444, crs = atan2(-B.v[1], B.v[0]) * 180 / M_PI; if (crs < 0) crs += 360;
      snprintf(b, sizeof b, "GPRMC,120000.00,A,%02d%08.5f,N,%03d%08.5f,E,%.3f,%.1f,021026,,,A", la, (lat - la) * 60, lo, (lon - lo) * 60, sp, crs); nmea(rm, sizeof rm, b);
      (void)!write(gm, g, strlen(g)); (void)!write(gm, rm, strlen(rm));
    }
    double in = el - ph_t; static int asked;
    switch (ph) {
      case ARM: if (el > 2) { say("press arm"); ph = FLY; ph_t = el; } break;
      case FLY: if (in > 1) { say("press fly"); ph = UP; ph_t = el; } break;
      case UP: if (in > 12) { ok_up = B.p[2] > 1.2 && F.state == FC_ARMED; printf("  both lines, after take-off: z %.2f, %s\n", B.p[2], fc_state_name(F.state));
                 LN[0].cut_up = LN[0].cut_down = 1; lost_line = 0; down_b0 = LN[1].down_bytes; say("goto 2 1 2 0"); ph = CUT_A; ph_t = el; asked = 0; printf("  line A cut both ways; a go-to sent\n"); } break;
      case CUT_A: if (in > 7 && !asked) { say("status"); asked = 1; }
        if (in > 8) { snprintf(st_cut, sizeof st_cut, "%s", status); ok_goto = hypot(B.p[0] - 2, B.p[1] - 1) < 0.5 && fabs(B.p[2] - 2) < 0.4; lost_cut = lost_line;
          ok_cut = !lost_line && F.state == FC_ARMED && LN[1].down_bytes - down_b0 > 5000 && strstr(st_cut, "telemetry |") && strstr(st_cut, "uplink 100%");
          printf("  line A cut: at (%.2f %.2f %.2f), %s; telemetry by line B %ld bytes\n", B.p[0], B.p[1], B.p[2], fc_state_name(F.state), LN[1].down_bytes - down_b0);
          LN[0].cut_up = LN[0].cut_down = 0; ph = BACK; ph_t = el; } break;
      case BACK: if (in > 3) {
          kill(gnd, SIGTERM); kill(pi, SIGTERM); waitpid(gnd, 0, 0); waitpid(pi, 0, 0);
          lines(pi_out, pbuf, &pn, "pi", on_pi); lines(gnd_out, gbuf, &gn, "ground", on_gnd); close(pi_out); close(gnd_out); pn = gn = 0;
          ph = RESTART; ph_t = el; } break;
      case RESTART: if (in > 4) {                                     /* (the drone has landed by itself meanwhile, its link gone) */
          lines_reset(); pa[18] = "serial,57600,up"; ga[6] = "serial,57600,up";
          pi = spawn("../pi/dfb_pi", pa, &pi_out); gnd = spawn("./dfb_ground", ga, &gnd_out);
          printf("  both again, line B one way: serial,57600,up (a laser)\n"); ph = ARM2; ph_t = el; } break;
      case ARM2: if (in > 2 && F.state == FC_DISARMED) { say("press arm"); ph = FLY2; ph_t = el; } else if (in > 2) { say("release arm"); } break;
      case FLY2: if (in > 1) { say("press fly"); ph = UP2; ph_t = el; } break;
      case UP2: if (in > 12) { ok_up2 = B.p[2] > 1.2 && F.state == FC_ARMED; printf("  after take-off: z %.2f, %s\n", B.p[2], fc_state_name(F.state));
                  LN[0].cut_up = 1; lost_line = 0; ph = CUT_A_UP; ph_t = el; asked = 0; printf("  line A's uplink cut: the channels by the laser only\n"); } break;
      case CUT_A_UP: if (in > 5 && !asked) { say("status"); asked = 1; }
        if (in > 6) { snprintf(st_laser, sizeof st_laser, "%s", status); lost_laser = lost_line;
          ok_laser = !lost_line && F.state == FC_ARMED && B.p[2] > 1.2 && strstr(st_laser, "telemetry |") && strstr(st_laser, "uplink 100%");
          LN[0].cut_down = 1; alarm_line = 0; ph = CUT_A_ALL; ph_t = el; asked = 0; printf("  line A cut both ways: no telemetry, the channels by the laser\n"); } break;
      case CUT_A_ALL: if (in > 5 && !asked) { say("status"); asked = 1; }
        if (in > 6) { snprintf(st_all, sizeof st_all, "%s", status); alarm_cut = alarm_line;
          ok_all = !lost_line && F.state == FC_ARMED && B.p[2] > 1.2 && alarm_line && strstr(st_all, "NO TELEMETRY");
          printf("  after: (%.2f %.2f %.2f), %s\n", B.p[0], B.p[1], B.p[2], fc_state_name(F.state)); ph = DONE; } break;
      case DONE: break;
    }
  }
  kill(gnd, SIGTERM); kill(pi, SIGTERM); usleep(300000);
  lines(pi_out, pbuf, &pn, "pi", on_pi); lines(gnd_out, gbuf, &gn, "ground", on_gnd);
  printf("  status, line A cut: %s\n  status, by the laser: %s\n  status, A cut both ways: %s\n", st_cut, st_laser, st_all);
  int ok = ok_up && ok_cut && ok_goto && ok_up2 && ok_laser && ok_all;
  printf(ok ? "two links end-to-end: ok\n" : "two links end-to-end: FAIL (take-off %d; A cut: carried on %d (lost %d), go-to %d; again: take-off %d, by the laser %d (lost %d), A cut both ways: drone on %d, alarm %d)\n",
         ok_up, ok_cut, lost_cut, ok_goto, ok_up2, ok_laser, lost_laser, ok_all, alarm_cut);
  return ok ? 0 : 1;
}
