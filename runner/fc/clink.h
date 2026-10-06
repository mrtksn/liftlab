/* A radio link made of small packets (an nRF24L01's 32 bytes; raw LoRa or CAN later): the part an ExpressLRS
 * transmitter module and receiver play, in our own code, the same at both ends. As plink.h does for ESP-NOW, Wi-Fi and
 * a serial line, but in a packet too small for plink's 24 bytes of header and signature beside a channel frame.
 *
 * The stack at each end writes and reads CRSF frames as with ExpressLRS (clink_from_stack, clink_to_stack); between,
 * clink_to_air makes the packets to send and clink_from_air takes the ones that came.
 *
 * The ground sends a packet every 1/up_hz s (a fixed beat); the drone answers each one it takes, at once (on an
 * nRF24L01 the answer rides back in the radio's own acknowledgement of the ground's next packet). Packets:
 *   - hello: kind, number, the sender's session, the session it has heard from the other end (0: none yet), a tag.
 *     Sent until both ends know each other's session: a new start of either end (its own random session) is a new
 *     pair, and the stream numbering below starts again.
 *   - channels (up): kind, number, the stream acknowledgements, the link quality heard, the 22 bytes of a CRSF channel
 *     frame's payload (the newest the stack wrote; none once it is PLINK_RC_STALE s old), a tag: 32 bytes.
 *   - stream (both ways): kind, number, acknowledgement, link quality, a chunk number and up to 21 bytes of the
 *     sender's stream, a tag. The stream is every other frame the stack writes, byte after byte: the commands up,
 *     the telemetry and messages down. It is reliable and in order (selective repeat: up to 8 chunks in flight; the
 *     receiver keeps the ones that come early and says which, so the sender sends again only the ones lost), so each
 *     frame arrives whole, in order.
 * Every packet but a hello acknowledges the other way's stream: the chunk number up to which all came, and a byte of
 * which of the 8 after it came too.
 *     Up, a stream packet goes in place of a channel packet every other beat while there is something to send.
 * The tag is 4 bytes of SipHash-2-4 with the binding phrase's key over the packet and, not sent, both ends' sessions
 * and the packet number's high bits: a packet from another pair (another phrase, an old session: a replay of a
 * recording) doesn't check. Within a pair, a packet number seen already or 64 behind is dropped.
 *
 * Link statistics as plink.h: each end counts the other's packets by number, and hands its stack a CRSF link
 * statistics frame ten times a second (the drone: while the ground has been heard within a second).
 *
 * Channel hopping (clink_channel): both ends hop over 8 channels picked from the binding phrase, one per packet by
 * its number. The drone follows the ground's numbers; having heard nothing for a while, it waits on the first channel
 * of the 8, where the ground comes every eighth packet.
 *
 * No C library: it also builds for the simulator. */
#ifndef CLINK_H
#define CLINK_H
#include <stdint.h>
#include "crsf.h"
#include "plink.h"

#define CLINK_MTU 32
#define CLINK_TAG 4
#define CLINK_HOPS 8
#define CLINK_WIN 8              /* stream chunks in flight */
#define CLINK_SQ 1024            /* stream bytes waiting (sent, not acknowledged, and not sent) */

typedef struct {
  int role;                      /* PLINK_GROUND or PLINK_DRONE */
  float up_hz;                   /* the ground's packets a second */
  int chunk;                     /* stream bytes a packet (≤ 21) */
  int first_ch, n_ch;            /* the radio's channels to hop over: first_ch … first_ch + n_ch − 1 */
  uint64_t k0, k1;
} clink_cfg;

typedef struct {
  /* sending: bytes [0, sent) are in the chunks in flight, base … hi−1; [sent, n) not yet sent */
  uint8_t b[CLINK_SQ]; int n, sent;
  uint8_t base, hi;
  uint8_t len[CLINK_WIN], sacked[CLINK_WIN];   /* each in flight (by number % CLINK_WIN): its length; the other end has it */
  double t_sent[CLINK_WIN];
  /* receiving: the chunk wanted next; the ones after it that came early (by number % CLINK_WIN) */
  uint8_t expect, early[CLINK_WIN], early_n[CLINK_WIN], early_b[CLINK_WIN][21];
} clink_stream;

typedef struct {
  uint32_t sent, got, bad, replays, hellos, resent, dropped;
} clink_counts;

typedef struct {
  clink_cfg C;
  uint32_t session, peer;        /* ours; theirs (0: not yet) */
  int knows_me;                  /* they have shown they know our session */
  double t_peer, t_stats, t_up, t_rc, t_sent;
  uint32_t seq;                  /* our next packet number */
  uint32_t rx_top; int rx_any; uint16_t rx_first; uint64_t rx_bits[2];
  int polled;                    /* the drone: a packet came it hasn't answered */
  int st_turn;                   /* the ground: the next stream packet's turn */
  int lq, peer_lq;
  float rate;
  uint8_t hop[CLINK_HOPS];
  uint32_t rx_seq_last; double t_rx_last;  /* the drone: the ground's last packet's number and when (hopping) */
  uint8_t rc[22]; int rc_have;
  clink_stream tx, rx;
  uint8_t out[PLINK_OUT]; int out_n;
  crsf_parser P, RP;            /* the stack's frames; the frames in the stream that came (whole ones go to the stack) */
  clink_counts N;
} clink;

void clink_cfg_default(clink_cfg *C, int role);              /* 100 packets a second, 21-byte chunks, channels 2–81 */
static inline void clink_cfg_link(clink_cfg *C, const rlink_cfg *L) { C->up_hz = rlink_compact_hz(L); }   /* (the rate for the link's speed) */
void clink_init(clink *L, const clink_cfg *C, uint32_t session);
void clink_from_stack(clink *L, const uint8_t *b, int n, double t);
int clink_to_air(clink *L, double t, uint8_t *pkt, int cap);   /* the packet due now, or 0 */
int clink_from_air(clink *L, const uint8_t *pkt, int n, int rssi, double t);   /* 1 taken, 0 not */
int clink_to_stack(clink *L, double t, uint8_t *b, int cap);
/* The radio channel to be on now: the ground, for the packet it sends next; the drone, for the one it expects. */
int clink_channel(const clink *L, double t);
/* The radio address (5 bytes) for this binding phrase's key: both ends the same, other pairs (mostly) not. */
void clink_address(const clink_cfg *C, uint8_t addr[5]);
static inline int clink_connected(const clink *L, double t) { return L->peer && L->knows_me && t - L->t_peer < 1.0; }
int clink_lq(const clink *L, double t);
#endif
