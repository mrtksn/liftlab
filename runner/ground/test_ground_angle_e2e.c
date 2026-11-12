/* End-to-end test of angle mode on a Pi (dfb_pi --no-navigation): a drone with no position sensor, flown on the sticks.
 *   ./dfb_ground --radio wifi --drone 127.0.0.1:14598  ⇄ UDP ⇄  ../pi/dfb_pi --no-navigation --radio wifi,sta --radio-port 14598
 * and behind dfb_pi a fake ESP32 (the real flight core flying test_nav.c's plant), no GPS. It drives dfb_ground over
 * UDP as a script would ("press arm", then "press fwd" and "press up") and checks that the ESP32 gets the pilot's
 * stick commands (RN_LINK_CMD of 28 bytes, not guided ones: dfb_pi's navigation doesn't fly it), that the arm switch
 * and the pitch and throttle sticks reach it as sent (the keys rest the throttle at its middle, so the flight core
 * refuses to arm: "throttle stick not at the bottom", its own rule; LiftLab's page sends the bottom until Take off), that dfb_pi says it is in angle mode and never waits for a
 * position, and that when the command module stops, the commands stop and the ESP32 falls back on its failsafe.
 *   sh build.sh && sh ../pi/build.sh && cc -O2 -I.. -I../fc -o test_ground_angle_e2e test_ground_angle_e2e.c ../fc/nav_core.c ../fc/fc_core.c \
 *     ../fc/tlm_core.c ../fc/tlm_sources.c ../fc/learn_core.c ../fc/super_core.c ../fc/rc_core.c ../fc/pickup_core.c ../fc/crsf.c ../rn_host.c ../rn.c ../rn_link.c ../rn_builtin.c -lm -lutil && ./test_ground_angle_e2e */
#define _DEFAULT_SOURCE
#define main main_orig
#include "../fc/test_nav.c"
#undef main
#include "rn_link.h"
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

#define PI_CMD_PORT 14597
#define DRONE_PORT 14598
#define GND_CMD_PORT 14599
static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static void write_file(const char *path, const void *p, size_t n) { FILE *f = fopen(path, "wb"); fwrite(p, 1, n, f); fclose(f); }
static void send_frame(int fd, uint8_t type, const void *p, uint32_t n) {
  static uint8_t fr[8192]; uint32_t len = rn_link_frame(fr, sizeof fr, type, (const uint8_t *)p, n); if (len) (void)!write(fd, fr, len);
}
static void step_ms(void) {                                    /* the plant and the flight core, 1 ms (as test_ground_wifi_e2e.c) */
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
  close(p[1]); fcntl(p[0], F_SETFL, O_NONBLOCK); *out_fd = p[0]; return pid;
}
static int angle_line, wait_line;
static void lines(int fd, char *buf, int *n, const char *tag) {
  ssize_t k = read(fd, buf + *n, 4000 - (size_t)*n); if (k > 0) *n += (int)k;
  char *nl; while ((nl = memchr(buf, '\n', (size_t)*n))) {
    *nl = 0; printf("  [%s] %s\n", tag, buf);
    if (!strcmp(tag, "pi")) { angle_line |= strstr(buf, "angle mode (--no-navigation)") != 0; wait_line |= strstr(buf, "waiting for the position") != 0; }
    int used = (int)(nl - buf) + 1; memmove(buf, nl + 1, (size_t)(*n - used)); *n -= used;
  }
}

int main(void) {
  srand(11);
  uint32_t lq; uint8_t *quad = read_file("../fc/testdata/quadx.dfa", &lq);
  start(quad, lq, 0);
  write_file("/tmp/test_angle.dnc", cfg_blob, sizeof cfg_blob);
  struct termios t; cfmakeraw(&t);
  int lm, ls; char lname[64];
  if (openpty(&lm, &ls, lname, &t, NULL)) { perror("openpty"); return 1; }
  char ports[3][8]; snprintf(ports[0], 8, "%d", PI_CMD_PORT); snprintf(ports[1], 8, "%d", DRONE_PORT); snprintf(ports[2], 8, "%d", GND_CMD_PORT);
  char drone[32]; snprintf(drone, sizeof drone, "127.0.0.1:%d", DRONE_PORT);
  int pi_out, gnd_out;
  char *pa[] = { "dfb_pi", "--link", lname, "--nav", "/tmp/test_angle.dnc", "--no-navigation", "--port", ports[0],
                 "--radio", "wifi,sta", "--radio-port", ports[1], "--bind", "angle test", 0 };
  char *ga[] = { "dfb_ground", "--radio", "wifi", "--drone", drone, "--bind", "angle test", "--port", ports[2], 0 };
  pid_t pi = spawn("../pi/dfb_pi", pa, &pi_out); usleep(300000);
  pid_t gnd = spawn("./dfb_ground", ga, &gnd_out);
  fcntl(lm, F_SETFL, O_NONBLOCK);
  udp = socket(AF_INET, SOCK_DGRAM, 0); to.sin_family = AF_INET; to.sin_port = htons(GND_CMD_PORT); to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  static uint8_t lbuf[8192]; rn_link L; rn_link_init(&L, lbuf, sizeof lbuf);
  double t0 = now_s(), next = t0, armed_at = 0, fwd_at = 0, up_at = 0, stop_at = 0, last_cmd = 0;
  char pbuf[4096], gbuf[4096]; int pn = 0, gn = 0;
  double guided_t = -10; long arm_flaps = 0;                                         /* when a guided command last came: NAV frames go for 1 s after (as flight.c) */
  long n28 = 0, n48 = 0, armed_cmds = 0; float pitch_max = 0, thr_rest = -1, thr_up = 0; int refused_thr = 0;
  while (now_s() - t0 < 22) {
    double el = now_s() - t0;
    lines(pi_out, pbuf, &pn, "pi"); lines(gnd_out, gbuf, &gn, "ground");
    /* the fake ESP32: the Pi's commands */
    uint8_t rx[1024]; ssize_t n = read(lm, rx, sizeof rx);
    for (ssize_t i = 0; i < n; i++) {
      int type = rn_link_feed(&L, rx[i]);
      if (type != RN_LINK_CMD) continue;
      if (L.len == 48) { n48++; guided_t = el; continue; }
      if (L.len != 28) continue;
      float v[7]; memcpy(v, L.buf, 28); n28++; last_cmd = el;
      fc_cmd c = { v[0] > 0.5f, v[1], v[2], v[3], v[4], v[5] >= -0.5f ? (int)(v[5] + 0.5f) : -1, v[6], 0, { 0, 0, 0 }, 0 };
      fc_command(&F, &c);
      { static int was; if (c.arm && !was && armed_cmds) arm_flaps++; was = c.arm; }
      if (c.arm) { armed_cmds++; refused_thr |= strstr(F.why, "throttle stick not at the bottom") != 0; }
      if (armed_at && !fwd_at && el > armed_at + 1.5 && thr_rest < 0) thr_rest = c.throttle;
      if (fwd_at && el < fwd_at + 1.5 && c.pitch > pitch_max) pitch_max = c.pitch;
      if (up_at && el < up_at + 1.5 && c.throttle > thr_up) thr_up = c.throttle;
    }
    while (now_s() < next) usleep(200);
    next += 0.01;
    for (int k = 0; k < 10; k++) step_ms();
    float nb[16] = { (float)F.t, (float)F.state, F.q[0], F.q[1], F.q[2], F.q[3], F.w[0], F.w[1], F.w[2], (float)acc_b[0], (float)acc_b[1], (float)acc_b[2], F.alt_e, (float)F.have_alt, (float)F.att_ok, 0 };
    if (el - guided_t < 1.0) send_frame(lm, RN_LINK_NAV, nb, 64);   /* (as the ESP32: only while the Pi's navigation flies it) */
    /* the pilot's script */
    if (el > 3 && !armed_at) { say("press arm"); armed_at = el; }
    if (armed_at && !fwd_at && el > armed_at + 3) fwd_at = el;
    if (fwd_at && el < fwd_at + 1.5) { static double nf; if (el >= nf) { nf = el + 0.05; say("press fwd"); } }
    if (fwd_at && !up_at && el > fwd_at + 2.5) up_at = el;
    if (up_at && el < up_at + 1.5) { static double nu; if (el >= nu) { nu = el + 0.05; say("press up"); } }
    if (up_at && !stop_at && el > up_at + 3) { stop_at = el; kill(gnd, SIGTERM); waitpid(gnd, 0, 0); printf("  the command module stopped at %.1f s (%s)\n", el, fc_state_name(F.state)); }
    if (stop_at && el > stop_at + 5) break;
  }
  kill(pi, SIGTERM); usleep(300000); lines(pi_out, pbuf, &pn, "pi");
  printf("  commands: %ld angle (%ld armed), %ld guided; throttle at rest %.2f, pressing up %.2f; pitch pressing forward %.2f; last command %.1f s after the stop; ESP32 now %s; it refused to arm on the middle throttle %d; the arm switch flapped %ld times\n",
         n28, armed_cmds, n48, thr_rest, thr_up, pitch_max, last_cmd - stop_at, fc_state_name(F.state), refused_thr, arm_flaps);
  int ok = angle_line && !wait_line && n28 > 300 && n48 == 0 && armed_cmds > 100 && refused_thr && F.state == FC_DISARMED && arm_flaps == 0
        && thr_rest > 0.45f && thr_rest < 0.55f && thr_up > 0.8f && pitch_max > 0.5f
        && last_cmd < stop_at + 1.5;
  printf(ok ? "angle mode on the Pi end-to-end: ok\n" : "angle mode on the Pi end-to-end: FAIL\n");
  return ok ? 0 : 1;
}
