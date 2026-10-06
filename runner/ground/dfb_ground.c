/*
 * The command module on a Mac, a Raspberry Pi or any Linux computer: ground_core.c (the same code the simulator runs
 * on the far side of its radio link, and the ESP32 command module runs) driving an ExpressLRS transmitter module
 * over a serial port. It sends the module CRSF channel frames 250 times a second and your commands, and reads back
 * the drone's telemetry and the module's link statistics. Or, with no module, over Wi-Fi (--radio wifi): the same
 * frames in UDP datagrams to the drone's Pi (dfb_pi --radio wifi,...), the module's part done here (../pi/radio_udp.c,
 * ../fc/plink.h), the link statistics too.
 *
 *   sh runner/ground/build.sh
 *   ./runner/ground/dfb_ground --tx /dev/tty.usbserial-XXXX --keys            (fly from the terminal)
 *   ./runner/ground/dfb_ground --tx /dev/ttyUSB0 --joystick /dev/input/js0    (a gamepad, on Linux)
 *   ./runner/ground/dfb_ground --tx /dev/ttyUSB0                              (your own code over UDP, below)
 *   ./runner/ground/dfb_ground --radio wifi --drone 192.168.4.1 --bind PHRASE --keys   (Wi-Fi: no module; this
 *                                                                             computer on the drone's network)
 *   ./runner/ground/dfb_ground --test                                         (no module: runs 1.5 s and shows the channels)
 *
 * Options:
 *   --radio KIND       the link (../fc/radio_link.h): elrs (the default: an ExpressLRS module on --tx), espnow or wifi
 *   --tx DEV           the transmitter module's serial port (CRSF). --baud N: its speed (400000 by default; set the
 *                      module to the same, or the speed it auto-detects). Wiring below.
 *                      With --radio espnow the module is an ESP32 running the command module firmware in its USB
 *                      bridge mode, which does ESP-NOW: --tx /dev/ttyUSB0 (115200 baud by default), otherwise the same
 *   --drone HOST[:PORT] with --radio wifi: the drone's address (its Pi running dfb_pi --radio wifi,...; port 14570
 *                      by default). This computer joins the drone's network (wifi,ap) or the one it joined (wifi,sta)
 *   --bind PHRASE      with --radio wifi: the binding phrase, the same as the drone's: it signs the packets, and a
 *                      packet signed with another is dropped ("liftlab" if not given, with a warning)
 *   --keys             the terminal's keys are the buttons (below). A terminal reports presses but not releases, so
 *                      a key counts as held until 0.5 s after its last repeat
 *   --joystick DEV     a gamepad (Linux joystick API). --axes R,P,T,Y: which axis is roll, pitch, throttle, yaw (an i
 *                      after it inverts it; default 3,4i,1i,0: Mode 2 on an Xbox-style pad).
 *                      --buttons arm=4,fly=5,hold=0,home=3,cal=2: which button does what (arm and fly latch)
 *   --port N           text commands over UDP (default 14561; 0: off), from this computer only. They are also read
 *                      from the terminal without --keys
 *   --listen-all       take UDP commands from any computer that can reach this one: anyone on the network can
 *                      then arm the drone (there is no password). Only on a network that is yours
 *   --program FILE     a program with edited formulas (the simulator: Computers → On the ground → Download its
 *                      program); it goes through the same checks as on the drone, then takes over
 *   --latch arm,fly    which buttons latch (a press toggles), for push buttons sent as presses
 *   --status           a status line every second
 *
 * Keys (--keys): W/S climb, sink · A/D turn · arrows (or I/K/J/L) forward, back, left, right · Space hold here ·
 *   H home · 1/2/3 gentle, normal, sport · R arm/disarm · T take off/land · C calibrate · G latch 1 (drop, grab) · Q quit
 *
 * Text commands (ground_text.h), one per line, from the terminal or as UDP datagrams (replies go back to the
 * sender): press/release/tap NAME, stick AXIS V, goto X Y Z [HEADING], calibrate, latch N open|close, pickup X Y Z [LATCH],
 * cmd ID V…, status, messages, quit. --hook DX,DY,DZ: where the drone's hook is from its hub, for pickup (0,0,-0.06).
 * Sticks and stick buttons sent this way lapse after a second unless sent again: if a script stops, the sticks centre.
 *
 * Wiring. A module with separate CRSF TX and RX pads (a full UART): a USB-serial adapter at 3.3 V, its TX to the
 * module's RX, its RX to the module's TX, ground to ground; plain (non-inverted) serial. A module in a radio's module
 * bay has one CRSF wire, both ways, run as ExpressLRS's CRSFHandset runs it: half duplex, inverted serial (the line
 * idles low; the module pulls it down while it listens, and drives it only to answer, right after each frame it gets).
 * Use an adapter that inverts TX and RX (an FT232R or FT231X, set so with FT_PROG), its TX through a 1 kΩ resistor to
 * the wire (the module's answer then wins over our idle level), its RX straight to the wire; our own frames echo back
 * and are ignored. (ExpressLRS also tries the plain polarity when it reads nothing, so a plain adapter may do too.)
 * One frame goes per beat, never two back to back (ground_core.h), so the module's answer has the wire to itself.
 *
 * Restarting this program while the drone flies (a crash, Ctrl-C, the computer sleeping) stops the channels: after a
 * second the drone counts the link lost and flies home to land. When it comes back the arm and fly switches start off:
 * a latching one (a gamepad's) forgets it was on, and one still on is held off by the switch warning until it has been
 * seen off. The drone hears arm off and disarms where it is, in the air too. Don't restart it in flight; if it did,
 * leave it off and let the drone land itself.
 */
#include "ground_core.h"
#include "rc_core.h"
#include "ground_text.h"
#include "radio_serial.h"
#include "radio_udp.h"
#include "radio_link.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#ifdef __linux__
#include <linux/joystick.h>
#endif

extern const uint8_t *const rn_builtin_ground_img;
extern const uint32_t rn_builtin_ground_len;


static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
/* ── the inputs: text commands (ground_text.c), the terminal's keys, a gamepad ── */
static gnd_state G; static gnd_text_in IN; static int running = 1, status_line = 0;
/* what goes to the module: a frame the port didn't take whole waits here, so none is cut (cut, it would fail its CRC,
 * and the frame after it too); while it waits, the next beat waits too (no frames pile up to go late, back to back) */
static uint8_t txq[CRSF_MAX_FRAME]; static int txn;
static void radio_close(radio_io *R) { if (radio_udp_is(R)) radio_udp_close(R); else radio_serial_close(R); }
static int tx_flush(radio_io *R) {
  while (txn > 0) {
    int w = R->write(R, txq, txn);
    if (w < 0) return -1;
    if (w == 0) return 0;
    memmove(txq, txq + w, (size_t)(txn - w)); txn -= (int)w;
  }
  return 0;
}
static float js_axis[GND_AXES]; static uint32_t js_has, js_held;
static void input_now(double t, gnd_input *in) {
  memset(in, 0, sizeof *in);
  for (int a = 0; a < GND_AXES; a++) if ((js_has >> a) & 1) { in->axis[a] = js_axis[a]; in->has_axis |= 1u << a; }
  in->held = js_held;
  gnd_text_inputs(&IN, t, in);                                       /* (a text command's stick wins over the pad's) */
}
static void command(char *s, char *reply, size_t rn, double t) {
  char cp[256]; snprintf(cp, sizeof cp, "%s", s);
  int r = gnd_text(&G, &IN, s, t, reply, (int)rn);
  if (r == 2) running = 0;
  else if (!r) snprintf(reply, rn, "unknown: %.40s (press, release, tap, stick, goto, calibrate, latch, pickup, cmd, status, messages, quit)", cp);
}

/* ── the terminal's keys ── */
static struct termios term_saved; static int term_raw;
static void term_restore(void) { if (term_raw) tcsetattr(0, TCSANOW, &term_saved); term_raw = 0; }
static void on_signal(int s) { (void)s; running = 0; }
static void key(int c, double t, int *esc) {
  const double HOLD = 0.5;                   /* a terminal sends repeats while a key is held, but no release */
  int b = -1;
  if (*esc == 2) { *esc = 0; b = c == 'A' ? GB_FWD : c == 'B' ? GB_BACK : c == 'C' ? GB_RIGHT : c == 'D' ? GB_LEFT : -1; }
  else if (*esc == 1) { *esc = c == '[' ? 2 : 0; return; }
  else if (c == 27) { *esc = 1; return; }
  else switch (c | 0x20) {
    case 'w': b = GB_UP; break; case 's': b = GB_DOWN; break; case 'a': b = GB_YAWL; break; case 'd': b = GB_YAWR; break;
    case 'i': b = GB_FWD; break; case 'k': b = GB_BACK; break; case 'j': b = GB_LEFT; break; case 'l': b = GB_RIGHT; break;
    case ' ': IN.until[GB_HOLD] = t + 0.3; return; case 'h': IN.until[GB_HOME] = t + 0.3; return;
    case '1' | 0x20: IN.until[GB_GENTLE] = t + 0.1; return; case '2' | 0x20: IN.until[GB_NORMAL] = t + 0.1; return; case '3' | 0x20: IN.until[GB_SPORT] = t + 0.1; return;
    case 'r': IN.held ^= GB(GB_ARM); printf("\r%s\r\n", IN.held & GB(GB_ARM) ? "arm switch on" : "arm switch off"); return;
    case 't': IN.held ^= GB(GB_FLY); printf("\r%s\r\n", IN.held & GB(GB_FLY) ? "fly switch on: take off" : "fly switch off: land"); return;
    case 'c': { float v = 1; gnd_command(&G, RC_CMD_LEARN, &v, 1); printf("\rasked the learning to calibrate\r\n"); return; }
    case 'g': { float v[2] = { 0, 2 }; gnd_command(&G, RC_CMD_LATCH, v, 2); printf("\rlatch 1: toggle (drop, or grab)\r\n"); return; }
    case 'q': running = 0; return;
  }
  if (b >= 0) IN.until[b] = t + HOLD;
}

int main(int argc, char **argv) {
  const char *tx_dev = 0, *js_dev = 0, *prog = 0, *drone = 0, *bind_phrase = 0; int baud = 0, port = 14561, keys = 0, test = 0, listen_all = 0, kind = RLINK_ELRS;
  int axmap[4] = { 3, 4, 1, 0 }, axinv[4] = { 0, 1, 1, 0 }, bmap[GB_N]; for (int i = 0; i < GB_N; i++) bmap[i] = -1;
  bmap[GB_ARM] = 4; bmap[GB_FLY] = 5; bmap[GB_HOLD] = 0; bmap[GB_HOME] = 3; bmap[GB_CAL] = 2;
  gnd_config cfg; gnd_config_default(&cfg); uint32_t latch = 0; int latch_set = 0;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--tx") && i + 1 < argc) tx_dev = argv[++i];
    else if (!strcmp(argv[i], "--baud") && i + 1 < argc) baud = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--port") && i + 1 < argc) port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--drone") && i + 1 < argc) drone = argv[++i];
    else if (!strcmp(argv[i], "--bind") && i + 1 < argc) bind_phrase = argv[++i];
    else if (!strcmp(argv[i], "--radio") && i + 1 < argc) {             /* the kind; its settings, if given, are the drone's business: checked, not used */
      const char *r = argv[++i]; int k = (int)strcspn(r, ","); kind = -1;
      for (int j = 0; j < RLINK_KINDS; j++) if ((int)strlen(rlink_names[j]) == k && !strncmp(r, rlink_names[j], (size_t)k)) kind = j;
      rlink_cfg L; char err[120];
      if (kind < 0) { fprintf(stderr, "--radio %s: elrs, espnow or wifi\n", r); return 2; }
      if (r[k] && rlink_parse(&L, r, err, sizeof err)) { fprintf(stderr, "--radio %s: %s\n", r, err); return 2; }
    }
    else if (!strcmp(argv[i], "--joystick") && i + 1 < argc) js_dev = argv[++i];
    else if (!strcmp(argv[i], "--axes") && i + 1 < argc) {
      char buf[64]; snprintf(buf, sizeof buf, "%s", argv[++i]); int a = 0;
      for (char *p = strtok(buf, ","); p && a < 4; p = strtok(0, ","), a++) { axmap[a] = atoi(p); axinv[a] = strchr(p, 'i') != 0; }
    }
    else if (!strcmp(argv[i], "--buttons") && i + 1 < argc) {
      char buf[200]; snprintf(buf, sizeof buf, "%s", argv[++i]);
      for (char *p = strtok(buf, ","); p; p = strtok(0, ",")) { char *e = strchr(p, '='); if (!e) continue; *e = 0; int b = gnd_button(p); if (b >= 0) bmap[b] = atoi(e + 1); }
    } else if (!strcmp(argv[i], "--latch") && i + 1 < argc) {
      char buf[200]; snprintf(buf, sizeof buf, "%s", argv[++i]); latch_set = 1;
      for (char *p = strtok(buf, ","); p; p = strtok(0, ",")) { int b = gnd_button(p); if (b >= 0) latch |= GB(b); }
    } else if (!strcmp(argv[i], "--hook") && i + 1 < argc) {
      if (sscanf(argv[++i], "%f,%f,%f", &cfg.hook[0], &cfg.hook[1], &cfg.hook[2]) != 3) { fprintf(stderr, "--hook DX,DY,DZ: where the hook is from the hub [m], body axes\n"); return 2; }
    } else if (!strcmp(argv[i], "--program") && i + 1 < argc) prog = argv[++i];
    else if (!strcmp(argv[i], "--keys")) keys = 1;
    else if (!strcmp(argv[i], "--status")) status_line = 1;
    else if (!strcmp(argv[i], "--test")) test = 1;
    else if (!strcmp(argv[i], "--listen-all")) listen_all = 1;
    else { fprintf(stderr, "usage: %s [--radio elrs] --tx DEV [--baud 400000] | --radio espnow --tx DEV [--baud 115200] | --radio wifi --drone HOST[:14570] [--bind PHRASE]\n"
                           "          [--keys] [--joystick /dev/input/js0 [--axes 3,4i,1i,0] [--buttons arm=4,fly=5,…]]\n"
                           "          [--port 14561 [--listen-all]] [--program FILE.rnp] [--latch arm,fly] [--status] | --test\n"
                           "  elrs: an ExpressLRS transmitter module on the serial port DEV\n"
                           "  espnow: an ESP32 running the command module firmware in its USB bridge mode, on DEV, is the module (ESP-NOW)\n"
                           "  wifi: UDP to the drone's Pi (dfb_pi --radio wifi,...) at HOST, this computer on its network; --bind: the drone's phrase\n"
                           "  UDP commands come from this computer only; --listen-all takes them from the network (anyone on it can arm)\n", argv[0]); return 2; }
  }
  char drone_host[256] = ""; int drone_port = RLINK_UDP_PORT;
  if (kind == RLINK_WIFI) {
    if (tx_dev) { fprintf(stderr, "dfb_ground: --radio wifi has no transmitter module: --drone HOST instead of --tx\n"); return 2; }
    if (!drone && !test) { fprintf(stderr, "dfb_ground: --radio wifi: say where the drone is (--drone HOST[:PORT], its Pi's address on the Wi-Fi network; port %d by default)\n", RLINK_UDP_PORT); return 2; }
    if (drone) {
      snprintf(drone_host, sizeof drone_host, "%s", drone); char *c = strrchr(drone_host, ':');
      if (c && strchr(drone_host, ':') == c) { *c = 0; drone_port = atoi(c + 1); if (drone_port <= 0 || drone_port > 65535) { fprintf(stderr, "--drone %s: no such port\n", drone); return 2; } }
    }
    if (!bind_phrase) { bind_phrase = "liftlab"; fprintf(stderr, "warning: no --bind PHRASE: using the default phrase \"liftlab\": anyone with LiftLab on this network can talk to your drone; give both ends a phrase of their own\n"); }
  } else {
    if (drone) { fprintf(stderr, "dfb_ground: --drone is for --radio wifi\n"); return 2; }
    if (bind_phrase) fprintf(stderr, "dfb_ground: --bind is for --radio wifi: %s keeps its binding phrase in the module\n", kind == RLINK_ESPNOW ? "ESP-NOW's bridge" : "ExpressLRS");
    if (!tx_dev && !test) {
      if (kind == RLINK_ESPNOW) fprintf(stderr, "dfb_ground: --radio espnow: an ESP32 running the command module firmware in its USB bridge mode is the module: --tx /dev/ttyUSB0 (its USB serial port; 115200 baud)\n");
      else fprintf(stderr, "dfb_ground: say where the transmitter module is (--tx DEV), or --test to run without one\n");
      return 2;
    }
  }
  if (!baud) baud = kind == RLINK_ESPNOW ? 115200 : 400000;
  if (js_dev && !latch_set) latch = GB(GB_ARM) | GB(GB_FLY);          /* a gamepad's buttons are push buttons */
  cfg.latch = latch; cfg.seq0 = (uint8_t)(time(0) ^ getpid());   /* (see gnd_config.seq0) */

  /* the step runner with the built-in ground program, then the core */
  static float arenas_[3][16384], pools_[3][4096]; static int32_t codes_[3][16384]; static rn_host H;
  float *ar[3] = { arenas_[0], arenas_[1], arenas_[2] }, *po[3] = { pools_[0], pools_[1], pools_[2] }; int32_t *co[3] = { codes_[0], codes_[1], codes_[2] };
  int e = rn_host_init(&H, rn_builtin_ground_img, rn_builtin_ground_len, ar, 16384, co, 16384, po, 4096);
  if (e) fprintf(stderr, "the built-in ground program didn't load (%d): the sticks go up unshaped\n", e);
  if (gnd_init(&G, e ? 0 : &H, &cfg)) fprintf(stderr, "%s\n", G.why);
  if (prog && !e) {
    static uint8_t img[65536]; FILE *f = fopen(prog, "rb"); uint32_t n = f ? (uint32_t)fread(img, 1, sizeof img, f) : 0; if (f) fclose(f);
    int r = n ? rn_host_stage(&H, img, n) : -1;
    if (r) fprintf(stderr, "%s: rejected (%d): staying on the built-in program\n", prog, r);
    else printf("%s: checked; runs in the background for a second, then takes over\n", prog);
  }

  /* the pilot's radio link (radio_io.h): a module on a serial port, or the Wi-Fi link's end here */
  radio_io *R = 0; char radio_said[300] = "no transmitter module (test)";
  if (tx_dev) { R = radio_serial_open(tx_dev, baud, kind == RLINK_ESPNOW ? "ESP-NOW bridge (serial)" : "ExpressLRS transmitter module (serial)"); if (!R) return 1;
    snprintf(radio_said, sizeof radio_said, "%s on %s", kind == RLINK_ESPNOW ? "ESP-NOW bridge (an ESP32)" : "transmitter module", tx_dev); }
  else if (kind == RLINK_WIFI && drone) { R = radio_udp_open(PLINK_GROUND, drone_host, drone_port, bind_phrase, "Wi-Fi radio"); if (!R) return 1;
    snprintf(radio_said, sizeof radio_said, "Wi-Fi to the drone at %s:%d", drone_host, drone_port); }
  int packets = R && radio_udp_is(R);               /* a packet link: R does the module's part, read() every pass drives it */
  int udp = -1;
  if (port > 0) {
    udp = socket(AF_INET, SOCK_DGRAM, 0); struct sockaddr_in a = { 0 }; a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port); a.sin_addr.s_addr = htonl(listen_all ? INADDR_ANY : INADDR_LOOPBACK);
    if (bind(udp, (struct sockaddr *)&a, sizeof a)) { perror("udp"); close(udp); udp = -1; } else fcntl(udp, F_SETFL, O_NONBLOCK);
    if (udp >= 0 && listen_all) fprintf(stderr, "UDP commands on port %d from any computer on the network: anyone there can arm the drone\n", port);
  }
  int js = -1;
#ifdef __linux__
  if (js_dev) { js = open(js_dev, O_RDONLY | O_NONBLOCK); if (js < 0) perror(js_dev); }
#else
  if (js_dev) fprintf(stderr, "--joystick works on Linux; on a Mac use gamepad.py (UDP)\n");
#endif
  if (keys && isatty(0)) { tcgetattr(0, &term_saved); struct termios t = term_saved; t.c_lflag &= ~(ICANON | ECHO); t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0; tcsetattr(0, TCSANOW, &t); term_raw = 1; atexit(term_restore); }
  signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
  printf("command module: %s, inputs: %s%s%s; %s\r\n", radio_said,
         keys ? "keys" : "text commands", js >= 0 ? ", gamepad" : "", udp >= 0 ? ", UDP" : "", G.why);
  if (keys) printf("W/S climb, sink · A/D turn · arrows move · Space hold · H home · 1/2/3 speed · R arm · T take off/land · C calibrate · G latch 1 · Q quit\r\n");

  const double step = 0.004; double t0 = now_s(), next = t0, next_status = t0 + 1, last = t0;
  uint32_t msgs_seen = 0, dropped_was = 0; int alert_was = -1, why_was = -1, esc = 0, stdin_open = 1; char line[512]; int ln = 0;
  while (running) {
    double t = now_s();
    struct pollfd p[4]; int np = 0, itx = -1, iudp = -1, iin = -1, ijs = -1;
    if (R) { itx = np; p[np].fd = R->fd; p[np++].events = POLLIN | (txn ? POLLOUT : 0); }
    if (udp >= 0) { iudp = np; p[np].fd = udp; p[np++].events = POLLIN; }
    if (stdin_open) { iin = np; p[np].fd = 0; p[np++].events = POLLIN; }
    if (js >= 0) { ijs = np; p[np].fd = js; p[np++].events = POLLIN; }
    int wait = (int)((next - t) * 1000); if (wait < 0) wait = 0;
    poll(p, (nfds_t)np, wait);
    t = now_s();
    if (itx >= 0 && (packets || (p[itx].revents & (POLLIN | POLLERR | POLLHUP | POLLNVAL)))) {   /* (a packet link: every pass, input or not: it sends its packets from there) */
      uint8_t b[512]; int n = R->read(R, b, sizeof b, 0);
      if (n > 0) for (int got = n; got > 0; got = got == (int)sizeof b ? R->read(R, b, sizeof b, 0) : 0) gnd_from_radio(&G, b, got, t - t0);   /* (all that waits) */
      else if ((n == 0 && !packets && (p[itx].revents & (POLLHUP | POLLERR | POLLNVAL))) || n < 0) {   /* unplugged */
        fprintf(stderr, "\rtransmitter module: gone (%s): nothing goes up any more; the drone will count the link lost\r\n", n < 0 ? strerror(errno) : "hung up");
        radio_close(R); R = 0;
      }
    }
    if (R && txn && (p[itx].revents & POLLOUT) && tx_flush(R)) { perror("\rtransmitter module"); radio_close(R); R = 0; }
    if (iudp >= 0 && (p[iudp].revents & POLLIN)) {
      char b[512]; struct sockaddr_in from; socklen_t fl = sizeof from; ssize_t n = recvfrom(udp, b, sizeof b - 1, 0, (struct sockaddr *)&from, &fl);
      if (n > 0) { b[n] = 0; char reply[1200]; for (char *s = strtok(b, "\n"), *nx; s; s = nx) { nx = strtok(0, "\n"); char cp[256]; snprintf(cp, sizeof cp, "%s", s); command(cp, reply, sizeof reply, t - t0); if (reply[0]) sendto(udp, reply, strlen(reply), 0, (struct sockaddr *)&from, fl); } }
    }
    if (iin >= 0 && (p[iin].revents & (POLLIN | POLLHUP))) {
      char b[128]; ssize_t n = read(0, b, sizeof b);
      if (n <= 0) stdin_open = 0;                                       /* (stdin closed: keep running on the other inputs) */
      for (ssize_t i = 0; i < n; i++) {
        if (keys) { key((unsigned char)b[i], t - t0, &esc); continue; }
        if (b[i] != '\n' && ln == (int)sizeof line - 1) { line[ln] = 0; ln = 0; char reply[1200]; command(line, reply, sizeof reply, t - t0); if (reply[0]) printf("%s\n", reply); }   /* (a too-long line: what came so far) */
        if (b[i] == '\n') { line[ln] = 0; ln = 0; char reply[1200]; command(line, reply, sizeof reply, t - t0); if (reply[0]) printf("%s\n", reply); }
        else line[ln++] = b[i];
      }
    }
#ifdef __linux__
    if (ijs >= 0 && (p[ijs].revents & (POLLIN | POLLERR | POLLHUP | POLLNVAL))) {
      struct js_event ev; ssize_t r;
      while ((r = read(js, &ev, sizeof ev)) == (ssize_t)sizeof ev) {
        if ((ev.type & 0x7F) == JS_EVENT_AXIS) for (int a = 0; a < GND_AXES; a++) if (axmap[a] == ev.number) { js_axis[a] = (axinv[a] ? -1.0f : 1.0f) * ev.value / 32767.0f; js_has |= 1u << a; }
        if ((ev.type & 0x7F) == JS_EVENT_BUTTON) for (int b = 0; b < GB_N; b++) if (bmap[b] == ev.number) { if (ev.value) js_held |= GB(b); else js_held &= ~GB(b); }
      }
      if ((r < 0 && errno != EAGAIN && errno != EINTR) || (p[ijs].revents & (POLLERR | POLLHUP | POLLNVAL))) {   /* unplugged: its sticks centre, its buttons let go */
        fprintf(stderr, "\rgamepad: gone: its sticks centre and its buttons are released\r\n");
        close(js); js = -1; js_has = 0; js_held = 0; for (int a = 0; a < GND_AXES; a++) js_axis[a] = 0;
      }
    }
#else
    (void)ijs; (void)axmap; (void)axinv;
#endif
    if (t < next) continue;
    /* a step */
    float dt = (float)(t - last); last = t; next += step; if (next < t) next = t + step;
    rn_host_tick(&H, dt);
    gnd_input in; input_now(t - t0, &in);
    uint8_t out[CRSF_MAX_FRAME]; int n = gnd_step(&G, &in, t - t0, dt, out, txn ? 0 : (int)sizeof out);   /* (cap 0: no frame due) */
    if (n && R) { memcpy(txq, out, (size_t)n); txn = n; if (tx_flush(R)) { perror("\rtransmitter module"); radio_close(R); R = 0; } }
    if (H.last_event) { int ev = H.last_event; H.last_event = 0; printf("\rprogram: %s\r\n", ev == RN_EV_SWAPPED ? "the new program runs now" : ev == RN_EV_REJECTED ? "the new program was rejected" : ev == RN_EV_FELL_BACK ? "the new program stopped: back to the one before" : ev == RN_EV_LOADED ? "checking the new program in the background" : "event"); }
    /* what to tell the pilot */
    if (G.V.nmsg - msgs_seen > GND_MSGS) msgs_seen = G.V.nmsg - GND_MSGS;   /* (more came than the ring keeps: the newest) */
    for (; msgs_seen < G.V.nmsg; msgs_seen++) printf("\rdrone: %s\r\n", G.V.msg[msgs_seen % GND_MSGS].s);
    if (G.dropped != dropped_was) { dropped_was = G.dropped; printf("\rcommand module: %s\r\n", G.why); }
    int why, lvl = gnd_alert(&G, &why);
    if (lvl != alert_was || why != why_was) { if (alert_was >= 0 || lvl) printf("\r%s%s\r\n", lvl == 2 ? "ALARM: " : lvl ? "warning: " : "", lvl ? gnd_why_text[why] : "all fine again"); alert_was = lvl; why_was = why; if (lvl == 2) { putchar('\a'); fflush(stdout); } }
    if (status_line && t >= next_status) {
      next_status = t + 1; char s[600]; gnd_status(&G, t - t0, s, sizeof s);
      if (packets && R) { size_t k = strlen(s); if (k + 4 < sizeof s) { memcpy(s + k, " | ", 3); radio_udp_counts(R, s + k + 3, (int)(sizeof s - k - 3)); } }
      printf("\r%s\r\n", s);
    }
    fflush(stdout);
    if (test && t - t0 > 1.5) break;
  }
  term_restore();
  if (test) { printf("channels: "); for (int i = 0; i < 9; i++) printf("%.2f ", G.ch[i]); printf("\n"); }
  return 0;
}
