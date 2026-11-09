/* The Pi companion (dfb_pi) with its link to the flight controller over MQTT (--link mqtt://HOST:PORT/PREFIX), end to
 * end: the real dfb_pi against the test broker (mqtt_test_broker.h), and a stand-in flight controller, an MQTT client
 * on PREFIX/link/to-fc. It checks that dfb_pi connects, prints the flight controller's events (frames that came as
 * messages on PREFIX/link/from-fc), sends its commands as one frame a message (its "disarmed, I'm here" commands, the
 * flight core's 48-byte RN_LINK_CMD), and reconnects and carries on when the broker drops everyone.
 *   cd runner/pi && sh build.sh && cd ../.. && cc -O2 -Irunner -Irunner/fc -Itools tools/test_pi_mqtt_link.c \
 *     runner/fc/nav_core.c runner/fc/fc_core.c runner/rn_host.c runner/rn.c runner/rn_link.c runner/rn_builtin.c \
 *     runner/fc/mqtt_min.c -lm -o /tmp/test_pi_mqtt && /tmp/test_pi_mqtt runner/pi/dfb_pi */
#define _DEFAULT_SOURCE
#define main main_orig
#include "../runner/fc/test_nav.c"
#undef main
#include "mqtt_min.h"
#include "rn_link.h"
#include "mqtt_test_broker.h"
#include <signal.h>
#include <time.h>
#include <sys/wait.h>

static int cmds, cmd_armed, bad;
static rn_link FL; static uint8_t flbuf[4096];
static void fc_begin(void *c, uint32_t len) { (void)c; (void)len; if (FL.state) rn_link_reset(&FL); }
static void fc_bytes(void *c, const uint8_t *b, int n) {
  (void)c;
  for (int i = 0; i < n; i++) { int t = rn_link_feed(&FL, b[i]); if (t == RN_LINK_CMD) { if (FL.len == 48) { cmds++; cmd_armed |= ((float *)FL.buf)[0] > 0.5f; } else bad++; } }
}
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

int main(int argc, char **argv) {
  if (argc < 2) { fprintf(stderr, "usage: test_pi_mqtt_link PATH/TO/dfb_pi\n"); return 2; }
  uint32_t lq; uint8_t *quad = read_file("runner/fc/testdata/quadx.dfa", &lq); if (!quad) { fprintf(stderr, "run from the repository root\n"); return 2; }
  start(quad, lq, 0);
  char dnc[] = "/tmp/test_pi_mqtt_XXXXXX"; int dfd = mkstemp(dnc); assert(dfd >= 0); assert(write(dfd, cfg_blob, sizeof cfg_blob) == (ssize_t)sizeof cfg_blob); close(dfd);
  broker_open();
  char url[96]; snprintf(url, sizeof url, "mqtt://127.0.0.1:%d/test/pi", BRK.port);
  int out[2]; assert(!pipe(out));
  pid_t pid = fork();
  if (!pid) {
    int nul = open("/dev/null", O_RDONLY); dup2(nul, 0); dup2(out[1], 1); dup2(out[1], 2); close(out[0]);
    execl(argv[1], "dfb_pi", "--link", url, "--nav", dnc, "--no-learning", "--no-supervisor", "--port", "0", (char *)0);
    perror("exec"); _exit(127);
  }
  close(out[1]); fcntl(out[0], F_SETFL, O_NONBLOCK);
  char to[96], from[96]; mq_link_topics("test/pi", to, sizeof to, from, sizeof from);
  static uint8_t tx[8192]; mq_client M; mq_init(&M, "127.0.0.1", BRK.port, "fc", to, tx, sizeof tx); M.on_begin = fc_begin; M.on_bytes = fc_bytes; M.keepalive = 5;
  rn_link_init(&FL, flbuf, sizeof flbuf);
  static char log[65536]; size_t ln = 0; int saw_event = 0, phase = 0, cmds_at_drop = 0; double t0 = now(), next_ev = 0, dropped_at = 0;
  for (double t = now(); t - t0 < 15; t = now()) {
    broker_poll(); mq_poll(&M, t);
    ssize_t r = read(out[0], log + ln, sizeof log - 1 - ln); if (r > 0) { ln += (size_t)r; log[ln] = 0; }
    if (mq_up(&M) && t >= next_ev && !saw_event) {          /* an event from the "flight controller", as a frame in a message */
      static uint8_t fr[128]; const char *ev = "hello over mqtt"; uint32_t k = rn_link_frame(fr, sizeof fr, RN_LINK_EVENT, (const uint8_t *)ev, (uint32_t)strlen(ev));
      mq_publish(&M, from, fr, k, t); next_ev = t + 0.5;
    }
    saw_event = saw_event || strstr(log, "esp32: hello over mqtt");
    if (phase == 0 && saw_event && cmds >= 5) { phase = 1; cmds_at_drop = cmds; broker_drop_all(); dropped_at = t; }   /* the broker drops everyone */
    if (phase == 1 && t - dropped_at > 0.3 && cmds >= cmds_at_drop + 5 && strstr(log, "MQTT: connected")) { phase = 2; break; }
    usleep(2000);
  }
  kill(pid, SIGTERM); waitpid(pid, 0, 0); unlink(dnc);
  if (phase < 2 || !saw_event || bad || cmd_armed) {
    fprintf(stderr, "FAILED: phase %d, event %d, commands %d (bad %d, armed %d)\n--- dfb_pi said ---\n%s\n", phase, saw_event, cmds, bad, cmd_armed, log);
    return 1;
  }
  int connects = 0; for (const char *p = log; (p = strstr(p, "MQTT: connected")); p++) connects++;
  assert(connects >= 2 && strstr(log, "link: MQTT broker 127.0.0.1"));
  printf("Pi MQTT link test passed: dfb_pi connected, printed the flight controller's event, sent %d disarmed commands (48-byte frames, one a message), reconnected after the broker dropped it (%d connections) and carried on.\n", cmds, connects);
  return 0;
}
