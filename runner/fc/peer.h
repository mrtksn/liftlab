/* Drones talking to each other: finding one another, and a session with each one found (ESP-NOW between ESP32s;
 * in the simulator, js/peer-air.js between the fleet's drones). The same C on the ESP32 and in the simulator.
 *
 * Finding: each drone broadcasts a beacon every PEER_BEACON_S (± a third, so two drones don't keep colliding): its
 * node number (the same at every start: from its MAC address), its session (new at each start), its name. Signed
 * with the fleet phrase's key (SipHash, as plink.h), so only drones of the same fleet hear each other.
 *
 * A session with each drone heard: packets addressed to it (by its radio address), about PEER_DATA_HZ a second and
 * sooner while something waits, each with a number of that pair's own, the values this drone publishes
 * (peer_publish: the latest wins), the messages not yet acknowledged (peer_send: delivered once, in order, sent
 * again in every packet until the other drone says it took them), a ping or its answer. Its signature covers both
 * drones' sessions: frames are taken only from packets naming this start of ours, so a recording played back
 * gets nowhere, and a restarted drone is known by its new session (taken once the old one has been quiet for half
 * a second, as plink.h).
 *
 * The link comes and goes as they fly: nothing waits on it. Each drone is, by how lately it was heard:
 *   PEER_CONNECTED  a packet naming us within PEER_FRESH_S (its values are current; its messages flow)
 *   PEER_STALE      it was connected, nothing for PEER_FRESH_S to PEER_LOST_S (its values are old, and say how old)
 *   PEER_HEARD      heard (its beacons), not connected yet: introducing ourselves
 *   PEER_LOST       nothing for PEER_LOST_S; forgotten after PEER_FORGET_S
 * and nothing more is needed to come back: its next packet does. Link quality per drone: the share of its packets
 * to us that came, of the last 32 (falling while nothing comes), and the same as it hears us (it says so).
 *
 * The radio's part (radio_peer.c on an ESP32) only moves packets: peer_to_air gives the next one due and where to
 * (a drone's address, or everyone: PEER_BROADCAST), peer_from_air takes one that came, with its sender's address.
 * No C library: it also builds for the simulator. */
#ifndef PEER_H
#define PEER_H
#include <stdint.h>

#define PEER_MTU 250                /* ESP-NOW's */
#define PEER_MAX 8                  /* drones in the table */
#define PEER_VALS 16                /* values a drone publishes */
#define PEER_MQ 8                   /* messages waiting to be taken, each way */
#define PEER_MSG 64                 /* bytes a message */
#define PEER_NAME 16
#define PEER_BEACON_S 0.15
#define PEER_DATA_HZ 10.0
#define PEER_FRESH_S 1.0
#define PEER_LOST_S 3.0
#define PEER_FORGET_S 10.0
#define PEER_TAG 8
enum { PEER_LOST = 0, PEER_HEARD = 1, PEER_STALE = 2, PEER_CONNECTED = 3 };
extern const uint8_t PEER_BROADCAST[6];

typedef struct { uint32_t sent, beacons, got, bad, replays, stale_sessions, resent, dropped, skipped; } peer_counts;
typedef struct {
  int used;
  uint8_t addr[6];                  /* its radio address */
  uint32_t id, session, known;      /* its node number, its session, and that session once confirmed */
  char name[PEER_NAME];
  double t_heard, t_fresh, t_sent, t_vals, t_ever;   /* heard (any valid packet), a packet naming us, sent to it, its values, first heard */
  uint16_t beacon_seq; int beacon_any;
  uint16_t seq;                     /* our next packet number to it */
  uint16_t rx_top, rx_first; int rx_any; uint32_t rx_bits;     /* its numbers to us: the newest, which of the last 32 came */
  float rate;                       /* its packets to us a second */
  int heard_us;                     /* its link quality of us, as it says [%] */
  int rssi;
  float vals[PEER_VALS]; int nvals;
  struct { uint8_t num, n, tries; uint8_t b[PEER_MSG]; } mq[PEER_MQ]; int mq_n; uint8_t mq_next;   /* our messages to it */
  uint8_t rx_next;                  /* its next message number we take */
  uint32_t ping_out; double t_ping; int ping_new, pong_due; uint32_t pong_stamp;   /* a ping of ours waiting; one of its to answer */
  float rtt;                        /* the last round trip [s], −1 none */
  uint32_t taken, msgs_in;
} peer_t;
typedef struct {
  uint64_t k0, k1;
  uint32_t id, session;
  char name[PEER_NAME];
  int on;
  peer_t P[PEER_MAX];
  float pub[PEER_VALS]; int npub;
  double next_beacon; uint16_t beacon_seq; uint32_t rnd;
  int rr;                           /* round robin over the drones for the next packet */
  /* messages taken, for the program: from whom, the bytes */
  struct { uint32_t from; uint8_t n; uint8_t b[PEER_MSG]; } in[PEER_MQ * 2]; int in_n;
  peer_counts N;
} peer_net;

/* id: this drone's node number (not 0; the same at every start); session: this start's (random, not 0); name: shown
 * to the others (≤ 15 characters); phrase: the fleet phrase (every drone of the fleet the same). */
void peer_init(peer_net *N, uint32_t id, uint32_t session, const char *name, const char *phrase);
/* The values this drone publishes (n ≤ PEER_VALS): the latest go in each packet. */
void peer_publish(peer_net *N, const float *v, int n);
/* A message to a drone (by its node number): queued to go, delivered once and in order. 0, −1: no such drone, −2:
 * too many waiting, −3: too long. */
int peer_send(peer_net *N, uint32_t to, const uint8_t *msg, int n);
/* A ping to a drone: its round trip comes in rtt. 0, −1 no such drone. */
int peer_ping(peer_net *N, uint32_t to, double t);
/* A packet that came, from addr, with its signal [dBm] (0 unknown): 1 taken, 0 dropped (see N). */
int peer_from_air(peer_net *N, const uint8_t addr[6], const uint8_t *p, int n, int rssi, double t);
/* The packet due now, if one is: its length, the address it goes to in addr (PEER_BROADCAST: a beacon); 0 none. */
int peer_to_air(peer_net *N, double t, uint8_t addr[6], uint8_t *p, int cap);
/* The next message taken (into msg, ≤ cap bytes; its sender's node number in from): its length, −1 none. */
int peer_recv(peer_net *N, uint32_t *from, uint8_t *msg, int cap);
/* A drone in the table: its state now (PEER_*), −1 an empty slot; its link quality (ours of it) [%]. */
int peer_state(const peer_net *N, int i, double t);
int peer_lq(const peer_net *N, int i, double t);
int peer_find(const peer_net *N, uint32_t id);       /* its slot, −1 none */
/* A drone's node number from its radio address (an ESP32's MAC: its last four bytes; never 0). */
uint32_t peer_id_of(const uint8_t addr[6]);
#endif
