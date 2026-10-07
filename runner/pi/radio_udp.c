/* A packet radio link's end over UDP, on Linux or a Mac (radio_io.h): the Wi-Fi link (fc/radio_link.h), the drone's
 * end on the Pi (dfb_pi --radio wifi,ap,CHANNEL or wifi,sta) and the command module's on a laptop or a Pi
 * (dfb_ground --radio wifi --drone HOST). There is no module here: this does its part (fc/plink.h), the same at both
 * ends. The program writes CRSF frames and reads CRSF frames, as it would with an ExpressLRS module on a serial port;
 * between, the frames travel in signed UDP datagrams (port RLINK_UDP_PORT, 14570, unless told otherwise).
 *
 *   - write(): the frames the program wrote go to the packet layer (plink_from_stack), and a packet goes if one is due;
 *   - read(): the datagrams that came go to the packet layer (plink_from_air), a packet goes if one is due
 *     (plink_to_air: the timing of the packets is driven from here, so the program calls read() every few ms, input
 *     or not), and what the program should read comes back (plink_to_stack: the frames that came, and the link
 *     statistics frames the packet layer makes, as a receiver or a transmitter module would).
 * The drone's end listens on 0.0.0.0:port and answers the address of the last packet that passed the packet layer's
 * checks (its binding phrase's signature, not a replay): nothing goes before one came. The ground's end sends to the
 * drone's address. The signal strength is unknown here (0): the link quality is counted from the packet numbers.
 *
 * Wi-Fi itself is the computer's business: on a Pi, hostapd or NetworkManager makes the network (wifi,ap) or joins
 * one (wifi,sta); a laptop joins it as any network. */
#define _DEFAULT_SOURCE            /* (glibc: POSIX and BSD calls also under a strict -std; on a Mac all are there) */
#include "radio_udp.h"
#include "radio_session.h"
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

typedef struct {
  plink P;
  int role;
  struct sockaddr_in peer; int have_peer;            /* where packets go */
  double t_good, t_bad_warned; uint32_t bad_seen;    /* for the binding phrase warning */
  uint32_t send_errors;
  const char *name;
} udp_ctx;

static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }


static void addr_text(const struct sockaddr_in *a, char *out, int n) {
  char ip[INET_ADDRSTRLEN] = "?"; inet_ntop(AF_INET, &a->sin_addr, ip, sizeof ip);
  snprintf(out, (size_t)n, "%s:%d", ip, ntohs(a->sin_port));
}

/* a packet, if one is due */
static void pump(radio_io *R, udp_ctx *C, double t) {
  if (!C->have_peer) return;                         /* (the drone: nobody to talk to yet) */
  uint8_t pkt[PLINK_MTU];                            /* (the ground's packets keep a fixed beat of 1/up_hz in plink itself) */
  int k = plink_to_air(&C->P, t, pkt, sizeof pkt);
  if (k > 0 && sendto(R->fd, pkt, (size_t)k, 0, (const struct sockaddr *)&C->peer, sizeof C->peer) < 0) C->send_errors++;   /* (no network yet, a full buffer: as a lost packet) */
}

static int udp_read(radio_io *R, uint8_t *b, int n, int wait_ms) {
  udp_ctx *C = R->ctx;
  if (wait_ms > 0) { struct pollfd p = { R->fd, POLLIN, 0 }; poll(&p, 1, wait_ms); }
  double t = now_s();
  for (int i = 0; i < 64; i++) {                     /* what came */
    uint8_t pkt[PLINK_MTU + 64]; struct sockaddr_in from; socklen_t fl = sizeof from;
    ssize_t k = recvfrom(R->fd, pkt, sizeof pkt, 0, (struct sockaddr *)&from, &fl);
    if (k < 0) break;                                /* (EAGAIN: no more; anything else, as a lost packet) */
    if (plink_from_air(&C->P, pkt, (int)k, 0, t) == 1) {
      C->t_good = t;
      if (C->role == PLINK_DRONE && fl == sizeof from && from.sin_family == AF_INET &&
          (!C->have_peer || from.sin_addr.s_addr != C->peer.sin_addr.s_addr || from.sin_port != C->peer.sin_port)) {
        char a[48]; addr_text(&from, a, sizeof a);
        fprintf(stderr, "\r%s: the command module is at %s\r\n", C->name, a);
        C->peer = from; C->have_peer = 1;
      }
    }
  }
  if (C->P.N.bad != C->bad_seen) {                   /* packets that don't carry our phrase's signature, and none that do */
    C->bad_seen = C->P.N.bad;
    if (t - C->t_good > 2 && t - C->t_bad_warned > 30) {
      C->t_bad_warned = t;
      fprintf(stderr, "\r%s: packets come that fail the binding phrase's check: is --bind the same at both ends?\r\n", C->name);
    }
  }
  pump(R, C, t);
  return plink_to_stack(&C->P, t, b, n);
}
static int udp_write(radio_io *R, const uint8_t *b, int n) {
  udp_ctx *C = R->ctx; double t = now_s();
  plink_from_stack(&C->P, b, n, t);
  pump(R, C, t);
  return n;
}

static uint32_t udp_peer_ses(radio_io *R) { return ((udp_ctx *)R->ctx)->P.known; }
static void udp_hear(radio_io *R, int lq, int rssi) { plink_hear(&((udp_ctx *)R->ctx)->P, lq, rssi); }
radio_io *radio_udp_open(int role, const char *peer_host, int port, const char *phrase, const char *name) {
  if (port <= 0 || port > 65535) { fprintf(stderr, "%s: no such port %d\n", name, port); return 0; }
  struct sockaddr_in peer; memset(&peer, 0, sizeof peer);
  if (role == PLINK_GROUND) {
    if (!peer_host || !*peer_host) { fprintf(stderr, "%s: the drone's address is needed\n", name); return 0; }
    struct addrinfo hints, *res = 0; memset(&hints, 0, sizeof hints); hints.ai_family = AF_INET; hints.ai_socktype = SOCK_DGRAM;
    char ps[8]; snprintf(ps, sizeof ps, "%d", port);
    int e = getaddrinfo(peer_host, ps, &hints, &res);
    if (e || !res) { fprintf(stderr, "%s: can't find %s: %s (is this computer on the drone's network?)\n", name, peer_host, e ? gai_strerror(e) : "no address"); return 0; }
    memcpy(&peer, res->ai_addr, sizeof peer); freeaddrinfo(res);
  }
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) { perror(name); return 0; }
  if (role == PLINK_DRONE) {
    struct sockaddr_in a; memset(&a, 0, sizeof a); a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port); a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(fd, (struct sockaddr *)&a, sizeof a)) { fprintf(stderr, "%s: UDP port %d: %s\n", name, port, strerror(errno)); close(fd); return 0; }
  }
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK);
  radio_io *R = calloc(1, sizeof *R); udp_ctx *C = calloc(1, sizeof *C);
  if (!R || !C) { free(R); free(C); close(fd); return 0; }
  plink_cfg cfg; plink_cfg_default(&cfg, role); plink_key(phrase ? phrase : "liftlab", &cfg.k0, &cfg.k1);
  plink_init(&C->P, &cfg, radio_session());
  C->role = role; C->name = name; C->t_good = now_s(); C->t_bad_warned = -1e9;
  if (role == PLINK_GROUND) { C->peer = peer; C->have_peer = 1; }
  R->name = name; R->read = udp_read; R->write = udp_write; R->fd = fd; R->ctx = C;
  R->peer = udp_peer_ses; R->hear = udp_hear;
  return R;
}
void radio_udp_close(radio_io *R) { if (!R) return; if (R->fd >= 0) close(R->fd); free(R->ctx); free(R); }
int radio_udp_is(const radio_io *R) { return R && R->read == udp_read; }
const plink *radio_udp_plink(const radio_io *R) { return radio_udp_is(R) ? &((const udp_ctx *)R->ctx)->P : 0; }
void radio_udp_peer(const radio_io *R, char *out, int n) {
  const udp_ctx *C = R->ctx;
  if (C->have_peer) addr_text(&C->peer, out, n); else snprintf(out, (size_t)n, "nobody yet");
}
void radio_udp_counts(const radio_io *R, char *out, int n) {
  const udp_ctx *C = R->ctx; const plink_counts *N = &C->P.N; char a[48]; radio_udp_peer(R, a, sizeof a);
  snprintf(out, (size_t)n, "wifi with %s: packets sent %u (%u failed), got %u (LQ %d%%), bad %u, replays %u; frames resent %u, dropped %u",
           a, N->sent, C->send_errors, N->got, plink_lq(&C->P, now_s()), N->bad, N->replays, N->resent, N->uq_dropped);
}
