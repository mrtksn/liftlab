/* A radio link made of packets (ESP-NOW, Wi-Fi UDP; nRF24L01 and Bluetooth LE to come): the part an ExpressLRS
 * transmitter module and receiver play in hardware, here in our own code, the same at both ends.
 *
 * The stack at each end (the command module's ground_core, the drone's rc_core/tlm_core) writes and reads CRSF frames,
 * exactly as it does with an ExpressLRS module on a UART: plink_from_stack takes what it writes, plink_to_stack gives
 * what it should read. Between, plink_to_air makes the packets to send and plink_from_air takes the ones that came.
 *
 * Packets: a 16-byte header (magic, version and sender's role, packet number, its session, the session it talks to,
 * the reliable frame it last took from that one, the link quality and signal it hears from it), then records, then an 8-byte SipHash-2-4 tag over
 * all of it with the key from the binding phrase. A record is one whole CRSF frame: kind 1 sent once, kind 2 with a
 * reliable number (sent again in every packet until the other end says it took it, delivered once, in order).
 *   - up (ground → drone): a packet every 1/up_hz s (on a fixed beat, whenever it's asked), each with the newest
 *     channel frame (state: the newest wins; none once the stack hasn't written one for PLINK_RC_STALE s, so a command
 *     module that stopped looks like no channels to the drone, and its failsafe acts), and the commands (0x80/0xD1)
 *     not yet taken, reliably;
 *   - down (drone → ground): the telemetry frames the drone writes, in order (state, superseded by the next ones),
 *     its messages (0x80/0xF1) reliably; a packet as soon as there is something, at most down_hz_max a second, and
 *     at least down_hz_min (to report and acknowledge).
 * A packet with a bad tag is dropped; so is one with a number seen already or more than 64 behind (a replay). A new
 * session (the other end restarted) is taken only once the old one has been quiet for half a second.
 *
 * Link statistics: each end counts the other's packets by their numbers (link quality, % of the last 100, falling
 * while nothing comes) and reads the signal from the radio if it can; each tells the other its counts in its packets.
 * Ten times a second each end hands its stack a CRSF link statistics frame, as a receiver (to the drone: while the
 * ground has been heard within a second) or a transmitter module (to the command module: always, LQ 0 while not
 * connected) would. So the drone's failsafe and telemetry budget, and the command module's alerts and held commands,
 * work over any packet link unchanged.
 *
 * No C library: it also builds for the simulator. */
#ifndef PLINK_H
#define PLINK_H
#include <stdint.h>
#include "crsf.h"

#define PLINK_MTU 250          /* the biggest packet (ESP-NOW's); a link with smaller ones sets mtu */
#define PLINK_HDR 16
#define PLINK_TAG 8
#define PLINK_RQ 16            /* reliable frames waiting to be taken */
#define PLINK_UQ 1024          /* telemetry bytes waiting to go */
#define PLINK_OUT 2048         /* bytes for the stack */
#define PLINK_RC_STALE 0.25    /* [s] */
enum { PLINK_GROUND = 0, PLINK_DRONE = 1 };

typedef struct {
  int role;                    /* which end this is */
  int mtu;                     /* the link's biggest packet [bytes] (≤ PLINK_MTU) */
  float up_hz;                 /* packets up a second (both ends know it: the drone counts what it misses by it) */
  float down_hz_min, down_hz_max;
  uint64_t k0, k1;             /* the key (plink_key from the binding phrase) */
} plink_cfg;

typedef struct {
  uint32_t sent, got, bad, replays, stale_sessions, uq_dropped, resent;
} plink_counts;

typedef struct {
  plink_cfg C;
  uint32_t session;            /* ours (random, at start) */
  uint32_t peer;               /* theirs (0: none yet) */
  double t_peer, t_sent, t_stats, t_up;   /* t_up: the ground's next packet, on a fixed beat of 1/up_hz */
  uint16_t seq;                /* our next packet number */
  /* theirs: the newest number, which of the last 128 came (by number), for replays and link quality */
  uint16_t rx_top, rx_first; int rx_any; uint64_t rx_bits[2];
  int lq, rssi;                /* what we hear of them: link quality [%], signal [dBm] (0: unknown) */
  float rate;                  /* their packets a second, as they come (numbers over time) */
  int peer_lq, peer_rssi;      /* what they hear of us, as they last said */
  /* reliable frames out: number, length, frame; the next number; and theirs we took last */
  struct { uint8_t num, n, tries; uint8_t f[CRSF_MAX_FRAME]; } rq[PLINK_RQ];
  int rq_n; uint8_t rq_next, peer_took;
  uint8_t rx_next;             /* the reliable number we take next from them */
  uint8_t rc[CRSF_MAX_FRAME]; int rc_n; double t_rc;    /* the ground: the newest channel frame, and when the stack wrote it */
  uint8_t uq[PLINK_UQ]; int uq_n;                       /* frames to go once */
  uint8_t out[PLINK_OUT]; int out_n;                    /* for the stack */
  crsf_parser P;               /* the stack's bytes, split into frames */
  plink_counts N;
} plink;

/* The key from a binding phrase (both ends the same phrase: the same key). */
void plink_key(const char *phrase, uint64_t *k0, uint64_t *k1);
void plink_cfg_default(plink_cfg *C, int role);        /* ESP-NOW's numbers: up 100 Hz, down 20–100 Hz, mtu 250 */
/* session: a number of this start's own (a random one: the other end tells restarts by it), not 0 */
void plink_init(plink *L, const plink_cfg *C, uint32_t session);
/* What the stack wrote (any split of whole frames: they're put together here). */
void plink_from_stack(plink *L, const uint8_t *b, int n, double t);
/* A packet that came, with the signal it came at [dBm] (0: unknown). 1 taken, 0 dropped (see N). */
int plink_from_air(plink *L, const uint8_t *pkt, int n, int rssi, double t);
/* The packet to send now, if one is due: its length, 0 if none. */
int plink_to_air(plink *L, double t, uint8_t *pkt, int cap);
/* What the stack should read now (frames that came, and the link statistics this end makes): bytes, ≤ cap. */
int plink_to_stack(plink *L, double t, uint8_t *b, int cap);
/* Heard the other end within a second. */
static inline int plink_connected(const plink *L, double t) { return L->peer && t - L->t_peer < 1.0; }
/* The link quality we hear now [%]: the last 100 packets by number, the ones not come yet missing. */
int plink_lq(const plink *L, double t);
#endif
