/* End-to-end test of the Wi-Fi radio link: the command module and the drone's Pi talking UDP (../pi/radio_udp.c,
 * ../fc/plink.c), on a PC, in real time:
 *   ./dfb_ground --radio wifi --drone 127.0.0.1:14596  ⇄ UDP ⇄  [this test: the air, a proxy]  ⇄ UDP ⇄  ../pi/dfb_pi --radio wifi,ap,6 --radio-port 14595
 * and behind dfb_pi a fake ESP32 (the real flight core flying test_nav.c's plant, its telemetry items when asked) and
 * a fake GPS, as in test_ground_e2e.c (the same flight over ExpressLRS). The proxy forwards the datagrams both ways,
 * and can drop a share of them (loss) or all (the cut).
 * It drives dfb_ground over UDP as a script would ("press arm", "press fly", "goto" through 15% loss, "press fwd",
 * "calibrate", "status"), and checks the drone takes off, goes there, follows the stick, starts the learning's
 * calibration from the radio, that a text command ("drone esp32 show") reaches the ESP32 and its answer comes back down
 * as a message, that the telemetry comes back and the uplink's quality is counted (about 85% through
 * the loss, 100% after), and that when the air is cut the drone flies home and lands by itself and the command module
 * raises its alarm. Before the take-off it puts the design's files on the Pi as LiftLab's drone card does (Upload
 * design: file_rx.h's text commands through 15% loss): an airframe, written in dfb_pi's folder and sent to the ESP32
 * whole (airframe), the same file announced again (taken again: a file ending in its own CRC-32 can't be told
 * "the same" by size and CRC, dfb_pi's CRC32_RESIDUE), and a wiring file whose lines go to the ESP32 but
 * link_uart's, then save (wiring), the ESP32's answers coming down but its "set ..." for each line. Then it starts a command module with another binding phrase: nothing it sends is taken (the
 * drone doesn't arm), nothing of the drone's reaches it, and both ends say the phrase doesn't match. And dfb_pi
 * refuses ESP-NOW (it needs an ESP32), dfb_ground ESP-NOW without the bridge's port.
 *   sh build.sh && sh ../pi/build.sh && cc -O2 -I.. -I../fc -o test_ground_wifi_e2e test_ground_wifi_e2e.c ../fc/nav_core.c ../fc/map_core.c ../fc/fc_core.c \
 *     ../fc/tlm_core.c ../fc/tlm_sources.c ../fc/learn_core.c ../fc/super_core.c ../fc/rc_core.c ../fc/pickup_core.c ../fc/crsf.c ../rn_host.c ../rn.c ../rn_link.c ../rn_builtin.c -lm -lutil && ./test_ground_wifi_e2e */
#define _DEFAULT_SOURCE
#define main main_orig
#include "../fc/test_nav.c"
#undef main
#include "rn_link.h"
#include "tlm_sources.h"
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#ifdef __APPLE__
#include <util.h>
#else
#include <pty.h>
#endif
#include <limits.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

enum { GND_CMD_PORT = 14593, PI_CMD_PORT = 14594, DRONE_PORT = 14595, PROXY_PORT = 14596 };

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
static const char *spawn_dir;   /* (where the next program runs: dfb_pi writes the files it's sent there) */
static pid_t spawn(const char *prog, char *const argv[], int *out_fd) {
  int p[2]; if (pipe(p)) return -1;
  char path[PATH_MAX]; if (!realpath(prog, path)) snprintf(path, sizeof path, "%s", prog);
  pid_t pid = fork();
  if (!pid) { int nul = open("/dev/null", O_RDONLY); dup2(nul, 0); dup2(p[1], 1); dup2(p[1], 2); if (spawn_dir && chdir(spawn_dir)) _exit(1); execv(path, argv); _exit(1); }
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
/* a program that should refuse its arguments: exit code 2 and its message says `want` */
static int refuses(const char *prog, char *const argv[], const char *want) {
  int fd; pid_t pid = spawn(prog, argv, &fd); int st = -1; waitpid(pid, &st, 0);
  char b[2048]; ssize_t n = read(fd, b, sizeof b - 1); b[n > 0 ? n : 0] = 0; close(fd);
  int ok = WIFEXITED(st) && WEXITSTATUS(st) == 2 && strstr(b, want);
  printf("  %s %s %s: %s", prog, argv[1], argv[2] ? argv[2] : "", ok ? "refused: " : "NOT REFUSED AS EXPECTED: "); fputs(b, stdout); if (!b[0] || b[strlen(b) - 1] != '\n') putchar('\n');
  return ok;
}

static int esp_setting, esp_answer_down, pi_text_line;
static int up_saved, up_same, up_starts, w_saved, af_down, af_got, af_said, wiring_said, w_lines, w_link_uart, w_save, w_set_down, saved_down, piece_lines;
static int learn_line, cal_line, lost_line, landed_line, alarm_line, prog_line, pi_peer_line, pi_bad_line, gnd_bad_line, pi_radio_lines;
static void on_pi(const char *s) {
  learn_line |= strstr(s, "learning command 1") != 0; cal_line |= strstr(s, "Calibration stopped: the drone flies home") != 0;   /* (it runs until the link goes, then stops)*/ lost_line |= strstr(s, "radio link lost: flying home") != 0;
  landed_line |= strstr(s, "landed by itself") != 0; pi_peer_line += strstr(s, "the command module is at") != 0;
  pi_bad_line += strstr(s, "fail the binding phrase") != 0; pi_radio_lines += !strncmp(s, "radio: ", 7); pi_text_line |= strstr(s, "radio text: esp32 show") != 0;
  piece_lines += strstr(s, "radio text: fd ") != 0;   /* (a file's pieces aren't each logged) */
}
static void on_gnd(const char *s) {
  up_saved |= strstr(s, "file up.dfa: saved") != 0; up_same |= strstr(s, "file up.dfa: the same is here") != 0; up_starts += strstr(s, "file up.dfa: ") && strstr(s, "pieces, send them"); w_saved |= strstr(s, "file w.txt: saved") != 0;
  af_said |= strstr(s, "airframe up.dfa: 4 motors") != 0; af_down |= strstr(s, "esp32: airframe loaded and saved") != 0;
  wiring_said |= strstr(s, "wiring: 2 settings lines") != 0; w_set_down += strstr(s, "esp32: set ") != 0; saved_down |= strstr(s, "esp32: saved; reboot") != 0;
  esp_answer_down |= strstr(s, "esp32: fake settings for show") != 0; alarm_line |= strstr(s, "ALARM: no telemetry") != 0; prog_line += strstr(s, "drone: ") != 0; gnd_bad_line += strstr(s, "fail the binding phrase") != 0; }

/* ── the air: a UDP proxy between the command module and the drone ── */
static int sg = -1, sd = -1; static struct sockaddr_in g_addr, d_addr; static int have_g;
static int loss_pct, cut; static long up_pk, down_pk, up_dropped, down_dropped;
static void air(void) {
  uint8_t b[2048]; struct sockaddr_in f; socklen_t fl; ssize_t n;
  while (fl = sizeof f, (n = recvfrom(sg, b, sizeof b, 0, (struct sockaddr *)&f, &fl)) > 0) {   /* up */
    g_addr = f; have_g = 1;
    if (cut || rand() % 100 < loss_pct) { up_dropped++; continue; }
    sendto(sd, b, (size_t)n, 0, (struct sockaddr *)&d_addr, sizeof d_addr); up_pk++;
  }
  while (fl = sizeof f, (n = recvfrom(sd, b, sizeof b, 0, (struct sockaddr *)&f, &fl)) > 0) {   /* down */
    if (!have_g || cut || rand() % 100 < loss_pct) { down_dropped++; continue; }
    sendto(sg, b, (size_t)n, 0, (struct sockaddr *)&g_addr, sizeof g_addr); down_pk++;
  }
}

int main(void) {
  srand(7);
  /* ESP-NOW is refused where it can't be */
  char *ra[] = { "dfb_pi", "--nav", "/nonexistent.dnc", "--radio", "espnow,1", 0 };
  char *rg[] = { "dfb_ground", "--radio", "espnow", 0 };
  char *rw[] = { "dfb_ground", "--radio", "wifi", 0 };
  int ok_refuse = refuses("../pi/dfb_pi", ra, "ESP-NOW needs an ESP32") && refuses("./dfb_ground", rg, "--tx") && refuses("./dfb_ground", rw, "--drone");

  uint32_t lq; uint8_t *quad = read_file("../fc/testdata/quadx.dfa", &lq);
  start(quad, lq, 0);
  write_file("/tmp/test_wifi.dnc", cfg_blob, sizeof cfg_blob); write_file("/tmp/test_wifi.dfa", quad, lq); write_pi_config("/tmp/test_wifi.dlc", F.A.n_motors);
  struct termios t; cfmakeraw(&t);
  int lm, ls, gm, gs; char lname[64], gname[64];
  if (openpty(&lm, &ls, lname, &t, NULL) || openpty(&gm, &gs, gname, &t, NULL)) { perror("openpty"); return 1; }
  /* the air first, so nothing the programs send at the start is lost to a port not yet open */
  sg = socket(AF_INET, SOCK_DGRAM, 0); sd = socket(AF_INET, SOCK_DGRAM, 0);
  struct sockaddr_in pa_ = { 0 }; pa_.sin_family = AF_INET; pa_.sin_port = htons(PROXY_PORT); pa_.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(sg, (struct sockaddr *)&pa_, sizeof pa_)) { perror("proxy port"); return 1; }
  fcntl(sg, F_SETFL, O_NONBLOCK); fcntl(sd, F_SETFL, O_NONBLOCK);
  d_addr.sin_family = AF_INET; d_addr.sin_port = htons(DRONE_PORT); d_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  char ports[4][8]; snprintf(ports[0], 8, "%d", PI_CMD_PORT); snprintf(ports[1], 8, "%d", DRONE_PORT); snprintf(ports[2], 8, "%d", GND_CMD_PORT);
  char drone[32]; snprintf(drone, sizeof drone, "127.0.0.1:%d", PROXY_PORT);
  int pi_out, gnd_out;
  char *pa[] = { "dfb_pi", "--link", lname, "--nav", "/tmp/test_wifi.dnc", "--airframe", "/tmp/test_wifi.dfa", "--pi", "/tmp/test_wifi.dlc", "--port", ports[0], "--gps", gname,
                 "--radio", "wifi,ap,6", "--radio-port", ports[1], "--bind", "test phrase",
#ifdef __APPLE__
                 "--baud", "115200",   /* (a macOS pty takes only the standard speeds; a pty's speed is nothing anyway) */
#endif
                 0 };
  char *ga[] = { "dfb_ground", "--radio", "wifi", "--drone", drone, "--bind", "test phrase", "--port", ports[2], "--status", 0 };
  char pi_dir[] = "/tmp/test_wifi_pi.XXXXXX"; if (!mkdtemp(pi_dir)) { perror("mkdtemp"); return 1; }
  spawn_dir = pi_dir; pid_t pi = spawn("../pi/dfb_pi", pa, &pi_out); spawn_dir = 0;
  pid_t gnd = spawn("./dfb_ground", ga, &gnd_out);
  fcntl(lm, F_SETFL, O_NONBLOCK); fcntl(gm, F_SETFL, O_NONBLOCK);   /* (a GPS port nobody reads mustn't stop the test) */
  udp = socket(AF_INET, SOCK_DGRAM, 0); to.sin_family = AF_INET; to.sin_port = htons(GND_CMD_PORT); to.sin_addr.s_addr = htonl(INADDR_LOOPBACK); fcntl(udp, F_SETFL, O_NONBLOCK);
  static uint8_t lbuf[8192]; rn_link L; rn_link_init(&L, lbuf, sizeof lbuf);
  static tlm_store TS; static tlm_watch TW; tlm_init(&TS); tlm_watch_init(&TW);
  double t0 = now_s(), next = t0, want_tlm = -10, next_pack = 0, next_gps = 0;
  char pbuf[8192], gbuf[8192]; int pn = 0, gn = 0;
  double armed_at = 0, goto_at = 0, fwd_at = 0, cal_at = 0, quiet_at = 0, wrong_at = 0; float x0 = 0, x1 = 0;
  char status[600] = "", status_loss[600] = "", status_wrong[600] = "";
  int ok_takeoff = 0, ok_goto = 0, wrong_armed = 0, radio_lines_before = 0; long up_before = 0, down_before = 0;
  while (now_s() - t0 < 160) {
    double el = now_s() - t0;
    lines(pi_out, pbuf, &pn, "pi", on_pi); lines(gnd_out, gbuf, &gn, "ground", on_gnd);
    { char r[600]; struct sockaddr_in f; socklen_t fl = sizeof f; ssize_t n = recvfrom(udp, r, sizeof r - 1, 0, (struct sockaddr *)&f, &fl);
      if (n > 0) { r[n] = 0; if (strstr(r, "uplink")) snprintf(wrong_at ? status_wrong : loss_pct ? status_loss : status, sizeof status, "%s", r); } }
    /* the fake ESP32: commands from the Pi; its telemetry items when asked */
    uint8_t rx[1024]; ssize_t n = read(lm, rx, sizeof rx);
    for (ssize_t i = 0; i < n; i++) {
      int type = rn_link_feed(&L, rx[i]);
      if (type == RN_LINK_CMD && L.len == 48) { float v[12]; memcpy(v, L.buf, 48); fc_cmd c = { v[0] > 0.5f, v[1], v[2], v[3], v[4], -1, v[6], v[7] > 0.5f, { v[8], v[9], v[10] }, v[11] }; fc_command(&F, &c); }
      else if (type == RN_LINK_WANT && L.len == 4) { float w; memcpy(&w, L.buf, 4); if ((int)w & 2) want_tlm = el; }
      else if (type == RN_LINK_EXC) fc_exc(&F, (const float *)L.buf, (int)(L.len / 4));
      else if (type == RN_LINK_MODEL) fc_model(&F, (const float *)L.buf, (int)(L.len / 4));
      else if (type == RN_LINK_SET) fc_set(&F, (const float *)L.buf, (int)(L.len / 4));
      else if (type == RN_LINK_SETTING && L.len == 4 && !memcmp(L.buf, "show", 4)) { esp_setting = 1; send_frame(lm, RN_LINK_REPORT, "fake settings for show", 22); }   /* (as flight.c setting())*/
      else if (type == RN_LINK_SETTING && L.len == 4 && !memcmp(L.buf, "save", 4)) { w_save++; send_frame(lm, RN_LINK_REPORT, "saved; reboot to use them", 25); }
      else if (type == RN_LINK_SETTING) {
        char m[160]; snprintf(m, sizeof m, "set %.*s (save, then reboot, to use it)", (int)L.len, (char *)L.buf); send_frame(lm, RN_LINK_REPORT, m, (uint32_t)strlen(m));
        if (L.len >= 9 && !memcmp(L.buf, "link_uart", 9)) w_link_uart++; else w_lines++;
      } else if (type == RN_LINK_AIRFRAME) { af_got = L.len == lq && !memcmp(L.buf, quad, lq); send_frame(lm, RN_LINK_REPORT, "airframe loaded and saved", 25); }
    }
    while (now_s() < next) { air(); usleep(200); }                     /* (the air passes the datagrams as they come) */
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
    /* the pilot's script, over UDP */
    { static int sent; if (el > 1.5 && !sent) { say("drone esp32 show"); sent = 1; } }   /* a text command up the radio, to the ESP32 */
    /* the design's files up the radio, as the drone card's Upload design (js/drone-card.js), through 15% loss; then the take-off */
    static int up_phase, up_done; static double up_next, up_phase_t; static uint32_t up_off;
    if (el > 2 && !up_done && el >= up_next) {
      static const char wiring[] = "motors=25,26,27,14\nlink_uart=32,35\n\nesc_hz=490\n";
      const uint8_t *fb = up_phase < 4 ? quad : (const uint8_t *)wiring; uint32_t fn = up_phase < 4 ? lq : (uint32_t)strlen(wiring); const char *nm = up_phase < 4 ? "up.dfa" : "w.txt";
      char line[96]; up_next = el + 0.16;
      if (up_phase == 0 || up_phase == 4) { loss_pct = 15; snprintf(line, sizeof line, "drone file %s %u %x", nm, (unsigned)fn, (unsigned)rn_crc32(fb, fn)); say(line); up_off = 0; up_phase++; up_phase_t = el; }
      else if (up_phase == 1 || up_phase == 5) {
        if (up_off < fn) {
          static const char *A = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/"; char d[64]; int k = 0;
          uint32_t m = fn - up_off < 36 ? fn - up_off : 36; const uint8_t *p = fb + up_off;
          for (uint32_t i = 0; i < m; i += 3) { uint32_t v = (uint32_t)p[i] << 16 | (i + 1 < m ? (uint32_t)p[i + 1] << 8 : 0) | (i + 2 < m ? p[i + 2] : 0);
            d[k++] = A[v >> 18 & 63]; d[k++] = A[v >> 12 & 63]; d[k++] = i + 1 < m ? A[v >> 6 & 63] : '='; d[k++] = i + 2 < m ? A[v & 63] : '='; }
          d[k] = 0; snprintf(line, sizeof line, "drone fd %u %s", (unsigned)up_off, d); say(line); up_off += 36;
        } else { say("drone file end"); up_phase++; up_phase_t = el; }
      }
      else if (up_phase == 2) { if (up_saved || el - up_phase_t > 8) { loss_pct = 0; snprintf(line, sizeof line, "drone file up.dfa %u %x", (unsigned)lq, (unsigned)rn_crc32(quad, lq)); say(line); up_phase++; up_phase_t = el; } }
      else if (up_phase == 3) { if (up_starts >= 2 || up_same || el - up_phase_t > 5) { say("drone airframe up.dfa"); up_phase = 31; up_phase_t = el; } }
      else if (up_phase == 31) { if (af_down || el - up_phase_t > 5) up_phase = 4; }
      else if (up_phase == 6) { if (w_saved || el - up_phase_t > 8) { loss_pct = 0; say("drone wiring w.txt"); up_phase++; up_phase_t = el; } }
      else if (up_phase == 7) { if (saved_down || el - up_phase_t > 6) { up_done = 1; printf("  the upload took %.1f s\n", el - 2); } }
    }
    if (up_done && !armed_at) { say("press arm"); armed_at = el; }
    { static int flew; if (armed_at && !flew && el > armed_at + 1) { say("press fly"); flew = 1; } }
    if (armed_at && !goto_at && el > armed_at + 13) {
      ok_takeoff = B.p[2] > 1.2 && F.state == FC_ARMED; printf("  after take-off: z %.2f, %s\n", B.p[2], fc_state_name(F.state));
      loss_pct = 15; printf("  the air loses 15%% of the packets from here\n"); say("goto 2 1 2 0"); goto_at = el;   /* (a command through the loss: sent again until taken) */
    }
    { static int asked; if (goto_at && !asked && el > goto_at + 8) { say("status"); asked = 1; } }
    if (goto_at && !fwd_at && el > goto_at + 10) { ok_goto = hypot(B.p[0] - 2, B.p[1] - 1) < 0.5 && fabs(B.p[2] - 2) < 0.4; printf("  after the go-to: (%.2f %.2f %.2f)\n", B.p[0], B.p[1], B.p[2]); x0 = (float)B.p[0]; fwd_at = el; loss_pct = 0; }
    if (fwd_at && el < fwd_at + 2) { static double nf; if (el >= nf) { nf = el + 0.05; say("press fwd"); } }
    if (fwd_at && !cal_at && el > fwd_at + 5) { x1 = (float)B.p[0]; say("status"); say("calibrate"); cal_at = el; }
    if (cal_at && !quiet_at && el > cal_at + 5) { quiet_at = el; cut = 1; printf("  the air is cut at (%.2f %.2f %.2f)\n", B.p[0], B.p[1], B.p[2]); }
    /* a command module with another binding phrase: nothing gets through */
    if (quiet_at && !wrong_at && el > quiet_at + 22 && (F.state == FC_DISARMED || el > quiet_at + 32)) {   /* (landed and disarmed by itself first: a slow machine takes longer) */
      printf("  after the cut: at (%.2f %.2f %.2f), %s\n", B.p[0], B.p[1], B.p[2], fc_state_name(F.state));
      kill(gnd, SIGTERM); waitpid(gnd, 0, 0); lines(gnd_out, gbuf, &gn, "ground", on_gnd); close(gnd_out); gn = 0;
      air();                                                           /* (what it sent last, still cut: not to arrive late) */
      char *gw[] = { "dfb_ground", "--radio", "wifi", "--drone", drone, "--bind", "another phrase", "--port", ports[2], 0 };
      gnd = spawn("./dfb_ground", gw, &gnd_out); cut = 0; wrong_at = el; up_before = up_pk; down_before = down_pk; radio_lines_before = pi_radio_lines;
      printf("  a command module with another phrase, the air open again\n");
    }
    if (wrong_at && el > wrong_at + 1) { static double na; if (el >= na) { na = el + 0.05; say("press arm"); } }   /* (it would arm it, were it heard) */
    if (wrong_at && F.state != FC_DISARMED) wrong_armed = 1;
    { static int asked; if (wrong_at && !asked && el > wrong_at + 4.5) { say("status"); asked = 1; } }
    if (wrong_at && el > wrong_at + 5) break;
  }
  kill(gnd, SIGTERM); kill(pi, SIGTERM); usleep(300000);
  lines(pi_out, pbuf, &pn, "pi", on_pi); lines(gnd_out, gbuf, &gn, "ground", on_gnd);
  { char r[600]; struct sockaddr_in f; socklen_t fl = sizeof f; ssize_t n = recvfrom(udp, r, sizeof r - 1, 0, (struct sockaddr *)&f, &fl); if (n > 0) { r[n] = 0; if (strstr(r, "uplink")) snprintf(status_wrong, sizeof status_wrong, "%s", r); } }
  int lq_loss = -1; { const char *u = strstr(status_loss, "uplink "); if (u) lq_loss = atoi(u + 7); }
  printf("  status through the loss: %s\n  status after it: %s\n  status with another phrase: %s\n", status_loss, status, status_wrong);
  printf("  packets up %ld (%ld dropped), down %ld (%ld dropped); with another phrase: %ld up, %ld down; the stick moved it %.2f m forward\n", up_pk, up_dropped, down_pk, down_dropped, up_pk - up_before, down_pk - down_before, x1 - x0);
  printf("  a text command: the Pi took it %d, the ESP32 got it %d, its answer came down %d\n", pi_text_line, esp_setting, esp_answer_down);
  char up_path[64]; snprintf(up_path, sizeof up_path, "%s/up.dfa", pi_dir); uint32_t un = 0; uint8_t *ub = read_file(up_path, &un); int up_file = ub && un == lq && !memcmp(ub, quad, lq);
  int ok_upload = up_saved && up_file && !up_same && up_starts >= 2 && af_said && af_got && af_down && w_saved && wiring_said && w_lines == 2 && !w_link_uart && w_save == 1 && saved_down && !w_set_down && !piece_lines;
  printf("  the upload: saved %d, on disk whole %d, announced again: taken %d, called the same %d; airframe said %d, the ESP32 got it whole %d, its answer down %d; wiring file %d, said %d, lines %d (link_uart %d), save %d, saved down %d, \"set\" down %d; pieces logged %d\n",
         up_saved, up_file, up_starts >= 2, up_same, af_said, af_got, af_down, w_saved, wiring_said, w_lines, w_link_uart, w_save, saved_down, w_set_down, piece_lines);
  printf("  Pi: peer %d, lost %d, landed %d, phrase warnings %d, radio lines with another phrase %d; command module: alarm %d, phrase warnings %d; armed with another phrase %d\n",
         pi_peer_line, lost_line, landed_line, pi_bad_line, pi_radio_lines - radio_lines_before, alarm_line, gnd_bad_line, wrong_armed);
  int ok_flight = ok_takeoff && ok_goto && x1 - x0 > 1.5 && learn_line && cal_line && strstr(status, "telemetry |") && strstr(status, "uplink 100%") && strstr(status, "POSHOLD")
        && down_pk > 500 && prog_line > 0 && pi_text_line && esp_setting && esp_answer_down && pi_peer_line >= 1 && lq_loss >= 65 && lq_loss <= 97;
  int ok_cut = lost_line && landed_line && alarm_line && hypot(B.p[0], B.p[1]) < 1.0 && B.p[2] < 0.2;
  int ok_wrong = wrong_at > 0 && !wrong_armed && F.state == FC_DISARMED && up_pk - up_before > 200 && down_pk - down_before > 50 && pi_radio_lines == radio_lines_before
        && pi_bad_line > 0 && gnd_bad_line > 0 && strstr(status_wrong, "NO TELEMETRY") && strstr(status_wrong, "uplink 0%");
  int ok = ok_refuse && ok_flight && ok_cut && ok_wrong && ok_upload;
  printf(ok ? "command module over Wi-Fi end-to-end: ok\n" : "command module over Wi-Fi end-to-end: FAIL (refusals %d, upload %d; take-off %d, goto %d, stick %d, learning %d/%d, status %d, uplink through loss %d%%, telemetry %ld; cut: lost %d, landed %d, alarm %d; another phrase: %d)\n",
         ok_refuse, ok_upload, ok_takeoff, ok_goto, x1 - x0 > 1.5, learn_line, cal_line, strstr(status, "POSHOLD") != 0, lq_loss, down_pk, lost_line, landed_line, alarm_line, ok_wrong);
  return ok ? 0 : 1;
}
