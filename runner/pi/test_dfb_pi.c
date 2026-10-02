/* End-to-end test of dfb_pi: a fake ESP32 (the real flight core with test_nav.c's plant, LTEL at 200 Hz while asked)
 * and a fake GPS (NMEA at 5 Hz) behind two pseudo-terminals, in real time. It starts ./dfb_pi on them with the
 * navigation, the learning and the supervisor, types "arm", "takeoff 1.5", "calibrate" (and waits for it to finish),
 * "health" and "goto 2 1 2" on its input, checks the calibration ran on the link (EXC and MODEL frames; the plant is the
 * description 4% stronger, so it may well keep flying on the description), that the supervisor answers and that it
 * gets there, then stops dfb_pi and checks the ESP32 goes to its failsafe.
 *   sh build.sh && cc -O2 -I.. -I../fc -o test_dfb_pi test_dfb_pi.c ../fc/nav_core.c ../fc/fc_core.c ../rn_host.c ../rn.c ../rn_link.c ../rn_builtin.c -lm -lutil && ./test_dfb_pi */
#define _DEFAULT_SOURCE
#define main main_orig
#include "../fc/test_nav.c"
#undef main
#include "rn_link.h"
#include <pty.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <signal.h>
#include <sys/wait.h>
static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static void write_file(const char *path, const void *p, size_t n) { FILE *f = fopen(path, "wb"); fwrite(p, 1, n, f); fclose(f); }
/* the Pi config (learn_core.h learn_config) for a quad: IMU at the hub, thrown from 1.2 m, 25 °C, four motors, 4S */
static void write_pi_config(const char *path, int nm) {
  float f[6 + 6 * FC_MAX_MOTORS + 3]; int n = 0;
  f[n++] = 0; f[n++] = 0; f[n++] = 0; f[n++] = 1.2f; f[n++] = 25; f[n++] = (float)nm;
  for (int i = 0; i < nm; i++) { f[n++] = 0; f[n++] = 90; f[n++] = 0.1f; f[n++] = 2500; f[n++] = 25; f[n++] = 0.4f; }
  f[n++] = 4; f[n++] = 0.02f; f[n++] = 60;
  static uint8_t b[8 + sizeof f + 4]; uint32_t magic = 0x434C4644u, ver = 1;   /* 'DFLC' */
  memcpy(b, &magic, 4); memcpy(b + 4, &ver, 4); memcpy(b + 8, f, (size_t)n * 4);
  uint32_t crc = rn_crc32(b, 8 + (uint32_t)n * 4); memcpy(b + 8 + n * 4, &crc, 4);
  write_file(path, b, 12 + (size_t)n * 4);
}
static void send_frame(int fd, uint8_t type, const void *p, uint32_t n) {
  static uint8_t fr[8192]; uint32_t len = rn_link_frame(fr, sizeof fr, type, (const uint8_t *)p, n); if (len) (void)!write(fd, fr, len);
}
static void step_ms(void) {
  fc_imu m; memset(&m, 0, sizeof m); for (int j = 0; j < 3; j++) { m.gyro[j] = (float)B.w[j]; m.acc[j] = (float)acc_b[j]; }
  if (B.t == 0) { m.acc[2] = 9.81f; }
  m.have_gyro = 1; m.have_baro = (lround(B.t * 1000) % 40) == 0; m.baro_alt = (float)(B.p[2] + 50);
  rn_host_tick(&HF, 0.001f); fc_step(&F, &m, 0.001f, 0, &O); plant_step(&B, &F.A, &O, 0.001);
}
int main(int argc, char **argv) {
  uint32_t lq; uint8_t *quad = read_file("../fc/testdata/quadx.dfa", &lq);
  start(quad, lq, 0);
  write_file("/tmp/test_dfb_pi.dnc", cfg_blob, sizeof cfg_blob);
  write_file("/tmp/test_dfb_pi.dfa", quad, lq);
  write_pi_config("/tmp/test_dfb_pi.dlc", F.A.n_motors);
  int mfd, sfd; char name[64]; struct termios t; cfmakeraw(&t);
  if (openpty(&mfd, &sfd, name, &t, NULL)) { perror("openpty"); return 1; }
  int gm, gs; char gname[64]; if (openpty(&gm, &gs, gname, &t, NULL)) { perror("openpty"); return 1; }
  int inp[2], outp[2]; if (pipe(inp) || pipe(outp)) return 1;
  pid_t pid = fork();
  if (!pid) {
    dup2(inp[0], 0); dup2(outp[1], 1);
    execl("./dfb_pi", "dfb_pi", "--link", name, "--nav", "/tmp/test_dfb_pi.dnc", "--airframe", "/tmp/test_dfb_pi.dfa", "--pi", "/tmp/test_dfb_pi.dlc",
          "--port", "14599", "--gps", gname, (char *)0);
    _exit(1);
  }
  fcntl(mfd, F_SETFL, O_NONBLOCK); fcntl(outp[0], F_SETFL, O_NONBLOCK);
  static uint8_t buf[8192]; rn_link L; rn_link_init(&L, buf, sizeof buf);
  double t0 = now_s(), next = t0, want_t = -10, cal_at = 0, cal_done = 0, goto_at = 0;
  int sent_arm = 0, sent_to = 0, ncmd = 0, nguided = 0, nexc = 0, nmodel = 0, nset = 0, nltel = 0, health_ok = 0, learned_ok = 0, ready_line = 0;
  double zmin_cal = 1e9, zmax_cal = -1e9;
  char out[8192]; int on = 0;
  while (now_s() - t0 < 120) {
    double el = now_s() - t0;
    /* what dfb_pi prints */
    ssize_t r = read(outp[0], out + on, sizeof out - 1 - on);
    if (r > 0) on += (int)r;
    for (char *nl; (nl = memchr(out, '\n', (size_t)on)); ) {
      *nl = 0; printf("    pi | %s\n", out); fflush(stdout);
      if (strstr(out, "learning on, supervisor on")) ready_line = 1;
      if (strstr(out, "Calibrated.") || strstr(out, "Calibration finished.")) { cal_done = el; learned_ok = strstr(out, "Flying on the learned model") != 0; }
      if (strstr(out, "lift margin")) health_ok = 1;
      int k = (int)(nl - out) + 1; memmove(out, nl + 1, (size_t)(on - k)); on -= k;
    }
    /* frames from the Pi */
    uint8_t rx[1024]; ssize_t n = read(mfd, rx, sizeof rx);
    for (ssize_t i = 0; i < n; i++) {
      int type = rn_link_feed(&L, rx[i]);
      if (type == RN_LINK_CMD && L.len == 48) {
        float v[12]; memcpy(v, L.buf, 48); fc_cmd c = { v[0] > 0.5f, v[1], v[2], v[3], v[4], -1, v[6], v[7] > 0.5f, { v[8], v[9], v[10] }, v[11] };
        fc_command(&F, &c); ncmd++; if (c.guided) nguided++;
      } else if (type == RN_LINK_WANT) want_t = el;
      else if (type == RN_LINK_EXC) { if (!fc_exc(&F, (const float *)L.buf, (int)(L.len / 4))) nexc++; }
      else if (type == RN_LINK_MODEL) { if (!fc_model(&F, (const float *)L.buf, (int)(L.len / 4))) nmodel++; }
      else if (type == RN_LINK_SET) { if (!fc_set(&F, (const float *)L.buf, (int)(L.len / 4))) nset++; }
    }
    /* 10 ms of flight in real time (LTEL every 5 ms while asked), then the navigation telemetry */
    while (now_s() < next) usleep(200);
    next += 0.01;
    for (int k = 0; k < 10; k++) {
      step_ms();
      if (k % 5 == 4 && el - want_t < 1.0) { static float lt[FC_LTEL_MAX]; int m = fc_ltel(&F, lt); send_frame(mfd, RN_LINK_LTEL, lt, (uint32_t)m * 4); nltel++; }
    }
    float nb[16] = { (float)F.t, (float)F.state, F.q[0], F.q[1], F.q[2], F.q[3], F.w[0], F.w[1], F.w[2], (float)acc_b[0], (float)acc_b[1], (float)acc_b[2], F.alt_e, (float)F.have_alt, (float)F.att_ok, 0 };
    send_frame(mfd, RN_LINK_NAV, nb, 64);
    static double next_gps = 0;
    if (el >= next_gps) {   /* the GPS: 5 Hz NMEA, 0.2 m noise, around 41.0 N 29.0 E (x north, y west) */
      next_gps += 0.2;
      double lat = 41.0 + (B.p[0] + 0.2 * gauss()) / 6371000.0 * 180 / M_PI, lon = 29.0 - (B.p[1] + 0.2 * gauss()) / (6371000.0 * cos(41.0 * M_PI / 180)) * 180 / M_PI;
      int la = (int)lat, lo = (int)lon; char g[200], rm[200];
      snprintf(g, sizeof g, "$GPGGA,120000.00,%02d%08.5f,N,%03d%08.5f,E,1,08,1.0,%.1f,M,0,M,,*00\r\n", la, (lat - la) * 60, lo, (lon - lo) * 60, 100 + B.p[2]);
      double sp = hypot(B.v[0], B.v[1]) / 0.514444, crs = atan2(-B.v[1], B.v[0]) * 180 / M_PI; if (crs < 0) crs += 360;
      snprintf(rm, sizeof rm, "$GPRMC,120000.00,A,%02d%08.5f,N,%03d%08.5f,E,%.3f,%.1f,021026,,,A*00\r\n", la, (lat - la) * 60, lo, (lon - lo) * 60, sp, crs);
      (void)!write(gm, g, strlen(g)); (void)!write(gm, rm, strlen(rm));
    }
    if (el > 1.0 && !sent_arm) { (void)!write(inp[1], "arm\n", 4); sent_arm = 1; }
    if (el > 2.5 && !sent_to) { (void)!write(inp[1], "takeoff 1.5\n", 12); sent_to = 1; }
    if (el > 13 && !cal_at) {
      printf("  after take-off: z %.2f, state %s, commands %d (guided %d), LTEL %d\n", B.p[2], fc_state_name(F.state), ncmd, nguided, nltel); fflush(stdout);
      (void)!write(inp[1], "calibrate\n", 10); cal_at = el;
    }
    if (cal_at && !cal_done) { if (B.p[2] < zmin_cal) zmin_cal = B.p[2]; if (B.p[2] > zmax_cal) zmax_cal = B.p[2]; }
    if (cal_done && !goto_at && el > cal_done + 1) { (void)!write(inp[1], "health\n", 7); (void)!write(inp[1], "goto 2 1 2\n", 11); goto_at = el; }
    if (goto_at && el > goto_at + 8) break;
  }
  printf("  calibration: %s in %.0f s, height %.2f..%.2f m; frames EXC %d, MODEL %d, SET %d; on the learned model: %s\n", cal_done ? "done" : "NOT done",
         cal_done - cal_at, zmin_cal, zmax_cal, nexc, nmodel, nset, learned_ok ? "yes" : "no");
  printf("  after goto: at (%.2f %.2f %.2f), state %s, flight core on the learned model: %s\n", B.p[0], B.p[1], B.p[2], fc_state_name(F.state), F.use_learned ? "yes" : "no");
  int ok1 = fabs(B.p[0] - 2) < 0.5 && fabs(B.p[1] - 1) < 0.5 && fabs(B.p[2] - 2) < 0.4;
  int ok2 = ready_line && cal_done && nexc > 100 && nmodel > 0 && nset > 0 && zmin_cal > 1.0 && zmax_cal < 2.0 && health_ok && learned_ok == F.use_learned;
  kill(pid, SIGTERM); usleep(300000);
  /* with dfb_pi gone, the ESP32 must go to its failsafe */
  for (int k = 0; k < 1000; k++) step_ms();
  printf("  dfb_pi stopped: %s\n", fc_state_name(F.state));
  int ok = ok1 && ok2 && F.state == FC_FAILSAFE;
  printf(ok ? "end-to-end: ok\n" : "end-to-end: FAIL (goto %d, learning %d)\n", ok1, ok2);
  return ok ? 0 : 1;
}
