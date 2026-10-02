/* End-to-end test of pi_nav: a fake ESP32 (the real flight core with test_nav.c's plant) and a fake GPS (NMEA at
 * 5 Hz) behind two pseudo-terminals, in real time. It starts ./pi_nav on them, types "arm", "takeoff 1.5" and
 * "goto 2 1 2" on its input, checks it gets there, then stops pi_nav and checks the ESP32 goes to its failsafe.
 *   sh build.sh && cc -O2 -I.. -I../fc -o test_pi_nav test_pi_nav.c ../fc/nav_core.c ../fc/fc_core.c ../rn_host.c ../rn.c ../rn_link.c ../rn_builtin.c -lm -lutil && ./test_pi_nav */
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
#include <sys/wait.h>
static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
int main(int argc, char **argv) {
  uint32_t lq; uint8_t *quad = read_file("../fc/testdata/quadx.dfa", &lq);
  start(quad, lq, 0);
  FILE *f = fopen("/tmp/test_pi_nav.dnc", "wb"); fwrite(cfg_blob, 1, sizeof cfg_blob, f); fclose(f);
  int mfd, sfd; char name[64]; struct termios t; cfmakeraw(&t);
  if (openpty(&mfd, &sfd, name, &t, NULL)) { perror("openpty"); return 1; }
  int gm, gs; char gname[64]; if (openpty(&gm, &gs, gname, &t, NULL)) { perror("openpty"); return 1; }
  int inp[2]; pipe(inp);
  pid_t pid = fork();
  if (!pid) { dup2(inp[0], 0); execl("./pi_nav", "pi_nav", "--link", name, "--config", "/tmp/test_pi_nav.dnc", "--port", "14599", "--gps", gname, (char *)0); _exit(1); }
  fcntl(mfd, F_SETFL, O_NONBLOCK);
  static uint8_t buf[4096]; rn_link L; rn_link_init(&L, buf, sizeof buf);
  double t0 = now_s(), next = t0; int sent_arm = 0, sent_to = 0, sent_goto = 0, ncmd = 0, nguided = 0;
  while (now_s() - t0 < 16) {
    /* commands from the Pi */
    uint8_t rx[512]; ssize_t n = read(mfd, rx, sizeof rx);
    for (ssize_t i = 0; i < n; i++) if (rn_link_feed(&L, rx[i]) == RN_LINK_CMD && (L.len == 48)) {
      float v[12]; memcpy(v, L.buf, 48); fc_cmd c = { v[0] > 0.5f, v[1], v[2], v[3], v[4], -1, v[6], v[7] > 0.5f, { v[8], v[9], v[10] }, v[11] };
      fc_command(&F, &c); ncmd++; if (c.guided) nguided++;
    }
    /* 10 ms of flight in real time, then the navigation telemetry */
    while (now_s() < next) usleep(200);
    next += 0.01;
    for (int k = 0; k < 10; k++) {
      fc_imu m; memset(&m, 0, sizeof m); for (int j = 0; j < 3; j++) { m.gyro[j] = (float)B.w[j]; m.acc[j] = (float)acc_b[j]; }
      if (B.t == 0) { m.acc[2] = 9.81f; }
      m.have_gyro = 1; m.have_baro = (lround(B.t * 1000) % 40) == 0; m.baro_alt = (float)(B.p[2] + 50);
      rn_host_tick(&HF, 0.001f); fc_step(&F, &m, 0.001f, 0, &O); plant_step(&B, &F.A, &O, 0.001);
    }
    float nb[16] = { (float)F.t, (float)F.state, F.q[0], F.q[1], F.q[2], F.q[3], F.w[0], F.w[1], F.w[2], (float)acc_b[0], (float)acc_b[1], (float)acc_b[2], F.alt_e, (float)F.have_alt, (float)F.att_ok, 0 };
    if (B.t == 0.01 || 1) { uint8_t fr[128]; uint32_t len = rn_link_frame(fr, sizeof fr, RN_LINK_NAV, (uint8_t *)nb, 64); write(mfd, fr, len); }
    double el = now_s() - t0;
    static double next_gps = 0;
    if (el >= next_gps) {   /* the GPS: 5 Hz NMEA, 0.2 m noise, around 41.0 N 29.0 E (x north, y west) */
      next_gps += 0.2;
      double lat = 41.0 + (B.p[0] + 0.2 * gauss()) / 6371000.0 * 180 / M_PI, lon = 29.0 - (B.p[1] + 0.2 * gauss()) / (6371000.0 * cos(41.0 * M_PI / 180)) * 180 / M_PI;
      int la = (int)lat, lo = (int)lon; char g[200], r[200];
      snprintf(g, sizeof g, "$GPGGA,120000.00,%02d%08.5f,N,%03d%08.5f,E,1,08,1.0,%.1f,M,0,M,,*00\r\n", la, (lat - la) * 60, lo, (lon - lo) * 60, 100 + B.p[2]);
      double sp = hypot(B.v[0], B.v[1]) / 0.514444, crs = atan2(-B.v[1], B.v[0]) * 180 / M_PI; if (crs < 0) crs += 360;
      snprintf(r, sizeof r, "$GPRMC,120000.00,A,%02d%08.5f,N,%03d%08.5f,E,%.3f,%.1f,021026,,,A*00\r\n", la, (lat - la) * 60, lo, (lon - lo) * 60, sp, crs);
      write(gm, g, strlen(g)); write(gm, r, strlen(r));
    }
    if (el > 1.0 && !sent_arm) { write(inp[1], "arm\n", 4); sent_arm = 1; }
    if (el > 2.5 && !sent_to) { write(inp[1], "takeoff 1.5\n", 12); sent_to = 1; }
    if (el > 8 && !sent_goto) { write(inp[1], "goto 2 1 2\n", 11); sent_goto = 1; printf("  after take-off: z %.2f, state %s, commands %d (guided %d)\n", B.p[2], fc_state_name(F.state), ncmd, nguided); fflush(stdout); }
  }
  printf("  after goto: at (%.2f %.2f %.2f), state %s\n", B.p[0], B.p[1], B.p[2], fc_state_name(F.state));
  int ok1 = fabs(B.p[0] - 2) < 0.5 && fabs(B.p[1] - 1) < 0.5 && fabs(B.p[2] - 2) < 0.4;
  kill(pid, SIGTERM); usleep(300000);
  /* with pi_nav gone, the ESP32 must go to its failsafe */
  for (int k = 0; k < 1000; k++) { fc_imu m; memset(&m, 0, sizeof m); for (int j = 0; j < 3; j++) { m.gyro[j] = (float)B.w[j]; m.acc[j] = (float)acc_b[j]; } m.have_gyro = 1; fc_step(&F, &m, 0.001f, 0, &O); plant_step(&B, &F.A, &O, 0.001); }
  printf("  pi_nav stopped: %s\n", fc_state_name(F.state));
  printf(ok1 && F.state == FC_FAILSAFE ? "end-to-end: ok\n" : "end-to-end: FAIL\n");
  return 0;
}
