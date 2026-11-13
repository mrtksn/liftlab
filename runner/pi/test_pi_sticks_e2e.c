/* End-to-end test of angle mode's text sticks (dfb_pi --no-navigation; `sticks`, `motor`: what the control panel,
 * liftlab, sends): dfb_pi with a fake ESP32 behind it (the real flight core flying test_nav.c's plant) and no radio
 * pilot. Over dfb_pi's UDP command port it sends a motor test, the bottom sticks, arms, and stops, and checks that
 *   - the ESP32 gets them as the pilot's commands (RN_LINK_CMD of 28 bytes), 50 a second while they're renewed;
 *   - motor 2 at 0.1 runs that motor alone (the flight core's motor test), a motor the airframe hasn't is refused,
 *     and so is a test while it's armed;
 *   - arm on the bottom throttle arms it, and status says the text sticks, the ESP32's state and its telemetry;
 *   - when they stop coming, the commands stop within a second and the ESP32's failsafe disarms it (on the ground).
 *   sh build.sh && cc -O2 -I.. -I../fc -o test_pi_sticks_e2e test_pi_sticks_e2e.c ../fc/nav_core.c ../fc/fc_core.c \
 *     ../fc/tlm_core.c ../fc/tlm_sources.c ../fc/learn_core.c ../fc/super_core.c ../fc/rc_core.c ../fc/pickup_core.c ../fc/crsf.c \
 *     ../rn_host.c ../rn.c ../rn_link.c ../rn_builtin.c -lm -lutil && ./test_pi_sticks_e2e */
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

#define PI_CMD_PORT 14587
#define RADIO_PORT 14588
static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static void write_file(const char *path, const void *p, size_t n) { FILE *f = fopen(path, "wb"); fwrite(p, 1, n, f); fclose(f); }
static void send_frame(int fd, uint8_t type, const void *p, uint32_t n) {
  static uint8_t fr[8192]; uint32_t len = rn_link_frame(fr, sizeof fr, type, (const uint8_t *)p, n); if (len) (void)!write(fd, fr, len);
}
static void step_ms(void) {
  fc_imu m; memset(&m, 0, sizeof m); for (int j = 0; j < 3; j++) { m.gyro[j] = (float)B.w[j]; m.acc[j] = (float)acc_b[j]; }
  if (B.t == 0) m.acc[2] = 9.81f;
  m.have_gyro = 1; m.have_baro = (lround(B.t * 1000) % 40) == 0; m.baro_alt = (float)(B.p[2] + 50);
  rn_host_tick(&HF, 0.001f); fc_step(&F, &m, 0.001f, 0, &O); if (F.state != FC_TESTING) plant_step(&B, &F.A, &O, 0.001);
}
static int udp; static struct sockaddr_in to;
static char last_reply[700];
static void say(const char *s) { sendto(udp, s, strlen(s), 0, (struct sockaddr *)&to, sizeof to); }
static void replies(void) { char b[700]; ssize_t k; while ((k = recv(udp, b, sizeof b - 1, MSG_DONTWAIT)) > 0) { b[k] = 0; memcpy(last_reply, b, (size_t)k + 1); } }
static const char *ask(const char *s) {                    /* a command and its answer (the replies before it read first) */
  replies(); last_reply[0] = 0; say(s);
  for (int i = 0; i < 100 && !last_reply[0]; i++) { usleep(2000); replies(); }
  printf("  > %s\n  < %s\n", s, last_reply); return last_reply;
}
static pid_t spawn(const char *prog, char *const argv[], int *out_fd) {
  int p[2]; if (pipe(p)) return -1;
  pid_t pid = fork();
  if (!pid) { int nul = open("/dev/null", O_RDONLY); dup2(nul, 0); dup2(p[1], 1); dup2(p[1], 2); execv(prog, argv); _exit(1); }
  close(p[1]); fcntl(p[0], F_SETFL, O_NONBLOCK); *out_fd = p[0]; return pid;
}
static void lines(int fd, char *buf, int *n) {
  ssize_t k = read(fd, buf + *n, 4000 - (size_t)*n); if (k > 0) *n += (int)k;
  char *nl; while ((nl = memchr(buf, '\n', (size_t)*n))) { *nl = 0; printf("  [pi] %s\n", buf); int used = (int)(nl - buf) + 1; memmove(buf, nl + 1, (size_t)(*n - used)); *n -= used; }
}

int main(void) {
  srand(7);
  uint32_t lq; uint8_t *quad = read_file("../fc/testdata/quadx.dfa", &lq);
  start(quad, lq, 0);
  write_file("/tmp/test_sticks.dnc", cfg_blob, sizeof cfg_blob);
  write_file("/tmp/test_sticks.dfa", quad, lq);
  struct termios t; cfmakeraw(&t);
  int lm, ls; char lname[64];
  if (openpty(&lm, &ls, lname, &t, NULL)) { perror("openpty"); return 1; }
  char p0[8], p1[8]; snprintf(p0, 8, "%d", PI_CMD_PORT); snprintf(p1, 8, "%d", RADIO_PORT);
  int pi_out;
  char *pa[] = { "dfb_pi", "--link", lname, "--nav", "/tmp/test_sticks.dnc", "--airframe", "/tmp/test_sticks.dfa", "--no-navigation", "--port", p0,
                 "--radio", "wifi,sta", "--radio-port", p1, "--bind", "sticks test", 0 };
  pid_t pi = spawn("./dfb_pi", pa, &pi_out); usleep(400000);
  fcntl(lm, F_SETFL, O_NONBLOCK);
  udp = socket(AF_INET, SOCK_DGRAM, 0); to.sin_family = AF_INET; to.sin_port = htons(PI_CMD_PORT); to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  static uint8_t lbuf[8192]; rn_link L; rn_link_init(&L, lbuf, sizeof lbuf);
  char pbuf[4096]; int pn = 0;
  double t0 = now_s(), next = t0, last_cmd = -1, next_tl = 0, next_say = 0;
  long n28 = 0, n48 = 0, cmds_test = 0; float m2_max = 0, other_max = 0; int tested = 0, armed = 0, idle_ok = 0;
  int got_setting = 0, setting_said = 0;
  int bad_motor = 0, armed_test_refused = 0, status_ok = 0, stop_disarmed = 0; double stop_at = 0;
  const char *phase = "";
  while (now_s() - t0 < 12) {
    double el = now_s() - t0;
    lines(pi_out, pbuf, &pn);
    uint8_t rx[1024]; ssize_t n = read(lm, rx, sizeof rx);
    for (ssize_t i = 0; i < n; i++) {
      int type = rn_link_feed(&L, rx[i]);
      if (type == RN_LINK_SETTING) { char line[128]; int k = L.len < 127 ? (int)L.len : 127; memcpy(line, L.buf, (size_t)k); line[k] = 0;   /* (as flight.c's setting()) */
        got_setting = !strcmp(line, "safety off"); if (got_setting) { fc_safety(&F, 1); send_frame(lm, RN_LINK_REPORT, "safety off until safety on or a reboot", 38); } continue; }
      if (type != RN_LINK_CMD) continue;
      if (L.len == 48) { n48++; continue; }
      if (L.len != 28) continue;
      float v[7]; memcpy(v, L.buf, 28); n28++; last_cmd = el;
      fc_cmd c = { v[0] > 0.5f, v[1], v[2], v[3], v[4], v[5] >= -0.5f ? (int)(v[5] + 0.5f) : -1, v[6], 0, { 0, 0, 0 }, 0 };
      if (c.test_motor >= 0) cmds_test++;
      fc_command(&F, &c);
    }
    while (now_s() < next) usleep(200);
    next += 0.01;
    for (int k = 0; k < 10; k++) step_ms();
    if (F.state == FC_TESTING) { tested = 1; if (O.motor[1] > m2_max) m2_max = O.motor[1]; for (int j = 0; j < 4; j++) if (j != 1 && O.motor[j] > other_max) other_max = O.motor[j]; }
    if (el >= next_tl) {                                    /* the ESP32's telemetry, 20 a second (flight.c's layout) */
      next_tl = el + 0.05; float tl[36] = { 0 }; tl[0] = (float)F.t; tl[1] = (float)F.state; tl[2] = 1.5f; tl[3] = -2.5f; tl[4] = 90; tl[8] = 0.12f; tl[13] = (float)(31 | (F.safety_off ? 32 : 0));
      for (int j = 0; j < 4; j++) tl[16 + j] = O.motor[j];
      send_frame(lm, RN_LINK_TELEM, tl, sizeof tl);
    }
    /* the panel's script: 20 a second, as liftlab sends them */
    const char *now_phase = el < 1 ? "" : el < 2 ? "sticks 0 0" : el < 3.5 ? "motor 2 0.1" : el < 4.5 ? "motor off" : el < 7 ? "sticks 1 0" : "";
    if (strcmp(now_phase, phase)) {
      phase = now_phase;
      if (!strcmp(phase, "sticks 0 0")) { ask("safety off"); bad_motor = strstr(ask("motor 9 0.1"), "isn't one of this airframe's 4 motors") != 0; idle_ok = 1; }
      if (!strcmp(phase, "sticks 1 0")) printf("  motor test: motor 2 at most %.2f, the others %.2f (%ld test commands)\n", m2_max, other_max, cmds_test);
      if (!*phase && el > 6) { stop_at = el; printf("  the sticks stop at %.1f s (%s)\n", el, fc_state_name(F.state)); }
    }
    static int asked_safety; if (el > 1.7 && !asked_safety && got_setting) { asked_safety = 1; const char *st = ask("status"); setting_said = strstr(st, "SAFETY OFF") && strstr(st, "the ESP32 said") && strstr(st, "safety off until"); }
    if (*phase && el >= next_say) { next_say = el + 0.05; say(phase); }
    replies();
    static double armed_t;
    if (!armed_t && F.state == FC_ARMED) armed_t = el;
    if (!armed && armed_t && el > armed_t + 0.3) {           /* (after its telemetry has said so) */
      armed = 1;
      const char *s = ask("status");
      status_ok = strstr(s, "text sticks arm on, throttle 0.00") && strstr(s, "the ESP32 armed") && strstr(s, "attitude roll +1.5 pitch -2.5 yaw 90") && strstr(s, "height 0.12 m");
      armed_test_refused = strstr(ask("motor 1 0.1"), "not while it's armed") != 0;
    }
    if (stop_at && el > stop_at + 3) { stop_disarmed = F.state == FC_DISARMED; break; }
  }
  kill(pi, SIGTERM); usleep(300000); lines(pi_out, pbuf, &pn);
  printf("  commands: %ld angle, %ld guided; the last %.2f s after the stop; ESP32 now %s\n", n28, n48, last_cmd - stop_at, fc_state_name(F.state));
  printf("  safety off: the ESP32 got it %d, status says it %d\n", got_setting, setting_said);
  int ok = got_setting && setting_said && n28 > 250 && n48 == 0 && idle_ok && bad_motor && tested && m2_max > 0.09f && other_max == 0 && armed && status_ok && armed_test_refused
        && stop_at > 0 && last_cmd < stop_at + 0.7 && stop_disarmed;
  printf("  bad motor refused %d, tested %d, armed %d, status %d, test refused armed %d, disarmed after the stop %d\n", bad_motor, tested, armed, status_ok, armed_test_refused, stop_disarmed);
  printf(ok ? "text sticks on the Pi end-to-end: ok\n" : "text sticks on the Pi end-to-end: FAIL\n");
  return ok ? 0 : 1;
}
