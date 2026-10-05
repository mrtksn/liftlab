/*
 * LiftLab telemetry: one interface every task publishes to, whatever carries the data to the ground.
 *
 * The tasks (flight core, navigation, learning, supervisor) put their latest values into a store as numbered items
 * (tlm_put) and short text messages (tlm_text): tlm_sources.c does that for each task. The board that runs the
 * telemetry task owns the store; another board packs what its tasks put (tlm_pack) and sends it over the board link
 * (RN_LINK_TLM), where it is unpacked into the store (tlm_unpack). Nothing here knows about radios.
 *
 * What leaves the drone is chosen by tlm_service, within the byte budget of the radio link: messages first, then
 * items that changed state (flight mode, the supervisor's decisions), then the item most overdue for its period. A
 * transport (tlm_transport) turns each into bytes for its radio: tlm_crsf.c makes CRSF frames for an ExpressLRS (or
 * Crossfire) receiver. Another radio (WiFi, a modem, MAVLink) is another transport; the tasks don't change.
 *
 * The same transport reads what the radio brings up (the pilot's sticks, link statistics, ground-station commands)
 * into an rc_input (rc_core.h).
 */
#ifndef TLM_CORE_H
#define TLM_CORE_H
#include <stdint.h>

#define TLM_NV 16                        /* values in an item */
#define TLM_TEXT 56                      /* characters in a message */
#define TLM_QN 12                        /* messages waiting */
#define TLM_PACK_MAX 256                 /* floats in one RN_LINK_TLM pack (the ESP32 takes no more; what doesn't fit goes next time) */

/* The items. Values are floats; the table in tlm_core.c gives each one's period and how it is encoded. */
enum {
  TLM_ATT = 1,       /* roll, pitch, yaw [rad] (flight core) */
  TLM_BATT,          /* voltage [V] (flight core; 0: not measured) */
  TLM_ALT,           /* barometric height [m], vertical speed [m/s] (flight core) */
  TLM_GPS,           /* latitude and longitude ×1e7 as two floats each (high, low 16 bits), altitude [m], ground speed [m/s], course [°], satellites (navigation) */
  TLM_STATE,         /* flight core state (0 disarmed, 1 armed, 2 failsafe, 3 crashed, 4 testing), flags (1 attitude settled, 2 has height, 4 guided, 8 open loop) */
  TLM_MOTORS,        /* motor count, then each motor's throttle 0–1 */
  TLM_POS,           /* position from home x y z [m], velocity [m/s] (navigation's estimate) */
  TLM_NAV,           /* target from home x y z [m], heading [rad], mode bits (1 ready, 2 flying, 4 home set, 8 returning, 16 landing, 32 landed, 64 radio lost), speed level 0–2 */
  TLM_LEARN,         /* calibrating, progress 0–1, on the learned model, learning in flight, throw phase, rotation fit, force fit */
  TLM_SUPER,         /* mode (0 normal, 1 careful, 2 return, 3 land), reason, lift margin, charge 0–1 (−1 unknown), cells, current [A], used [mAh] */
  TLM_SUPER_M,       /* motor count, then each motor's effectiveness (−1: taken out) */
  TLM_LINK,          /* what the drone hears: uplink RSSI [dBm], LQ [%], SNR [dB], radio link lost (0/1) */
  TLM_CARGO,         /* latch count, then each latch's bits: 1 closed, 2 loaded, 4 moving, 8 has a load switch (cargo) */
  TLM_ITEMS
};
typedef struct {
  uint8_t id, nmax; float period;        /* how often it is worth sending [s] */
  uint8_t prio;                          /* 0 most important */
  uint8_t on_change;                     /* a change of any value is sent at once */
  const char *name;
} tlm_def;
extern const tlm_def tlm_defs[TLM_ITEMS];
/* Scale for the 16-bit integers an item travels as in a generic frame (CRSF 0x80/0xD0), per value index. */
float tlm_scale(int id, int k);

typedef struct { float v[TLM_NV], sent[TLM_NV]; int n, has, dirty_link; double t_put, t_sent; } tlm_slot;
typedef struct { int sev; char s[TLM_TEXT + 1]; } tlm_msg;
typedef struct {
  tlm_slot it[TLM_ITEMS];
  tlm_msg q[TLM_QN]; int qh, qn;         /* messages for the radio */
  tlm_msg fq[TLM_QN]; int fh, fn;        /* messages to forward over the board link */
  double tokens, t_service;
  double rx_q, rx_text;                  /* the receiver's queue as it would drain at the budget: bytes in it, and bytes
                                          * up to the end of the last message written (0: that one has gone) */
  uint32_t bytes_sent, frames_sent, dropped;
  char mode[16], sent_mode[16]; double t_mode;   /* the flight mode as last composed, and as last sent */
} tlm_store;

void tlm_init(tlm_store *T);
void tlm_put(tlm_store *T, int id, const float *v, int n, double t);
/* severity as MAVLink/ArduPilot: 2 critical, 3 error, 4 warning, 6 info. Long text is split into several messages. */
void tlm_text(tlm_store *T, int sev, const char *s);
const tlm_slot *tlm_get(const tlm_store *T, int id);
/* The flight mode as a short word ("DISARMED", "POSHOLD", "RTH", …), from the state, navigation and supervisor items. */
const char *tlm_mode(tlm_store *T);

/* Between boards: what this board's tasks put since the last pack (floats: id, n, values…; messages as id 255, sev,
 * length, characters), for RN_LINK_TLM. Returns the floats written. */
int tlm_pack(tlm_store *T, float *out, int cap);
void tlm_unpack(tlm_store *T, const float *in, int n, double t);

/* A radio. */
typedef struct tlm_transport {
  const char *name;
  int max_frame;
  /* encode an item (it may combine others from the store), the flight mode, or a message; 0: not carried */
  int (*item)(const tlm_store *T, int id, uint8_t *out);
  int (*mode)(const char *mode, uint8_t *out);
  int (*text)(int sev, const char *s, uint8_t *out);
} tlm_transport;
extern const tlm_transport tlm_crsf;

/* What to send now: frames into out (cap bytes), within budget [bytes/s] averaged over time. Returns the bytes.
 * A message is written only once the one before it has left the receiver's queue: an ExpressLRS receiver replaces a
 * status text still waiting there by the next one (as it does any standard frame by a newer one of its type), so two
 * written close together would lose the first. The next waits in the store meanwhile. */
int tlm_service(tlm_store *T, const tlm_transport *X, double t, float budget, uint8_t *out, int cap);

#endif
