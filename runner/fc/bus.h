/*
 * The data bus (docs/topic-bus.md): each board's table of named topics that its programs publish and read, and that
 * other boards copy over the board link when they subscribe.
 *
 * A topic: a name (dotted: "fc.attitude", "user.camera.photo"), up to BUS_VALS floats, one writer, a sequence number
 * that goes up at each publish, and the time it was published. A mirror is another board's topic, copied here
 * (read-only here). Everything is fixed size: nothing allocates, nothing blocks, a publish is a copy.
 *
 * The store itself is here, inline, so the flight code needs nothing else to link; copying between boards
 * (subscriptions, the RN_LINK_BUS_SUB and RN_LINK_BUS frames) is in bus.c.
 *
 * Use: bus_init; the board sets its clock each step (bus_clock); a writer registers its topic once
 * (bus_topic) and publishes (bus_pub); a reader finds it (bus_find), reads it (bus_get, bus_age) or watches it
 * (bus_changed: has it been published since I last looked?).
 */
#ifndef BUS_H
#define BUS_H
#include <stdint.h>

#define BUS_TOPICS 48                  /* topics on a board */
#define BUS_VALS 32                    /* floats in a topic */
#define BUS_POOL 1024                  /* floats for all of them */
#define BUS_NAME 24                    /* a name's characters, with its 0 */
#define BUS_SUBS 32                    /* subscriptions a board serves (from all the boards that ask) */
#define BUS_WANTS 32                   /* subscriptions a board asks for */
#define BUS_LAPSE 2.0                  /* a subscription not renewed for this long lapses [s] */
#define BUS_RENEW 0.5                  /* how often a subscriber renews [s] */
#define BUS_BEAT 0.5                   /* on change: also this often while nothing changes, so a mirror's age stays meaningful [s] */
#define BUS_VERSION 1                  /* the frames' layout */

enum { BUS_LOCAL = 0, BUS_MIRROR = 1 };
typedef struct {
  char name[BUS_NAME];
  uint32_t hash;                       /* FNV-1a of the name: how it travels */
  uint16_t n, off;                     /* its floats, and where they are in the pool */
  uint8_t kind, has;                   /* BUS_LOCAL or BUS_MIRROR; published at least once */
  uint32_t seq;                        /* goes up at each publish (a mirror: the writer's, mod 2^24) */
  double t;                            /* when (this board's clock; a mirror: now − its age when it came) */
  uint32_t bad;                        /* a mirror: copies refused (their size didn't match) */
  uint32_t got;                        /* updates here: publishes, or (a mirror) copies taken */
} bus_entry;
typedef struct { uint32_t hash, sum_sent; int topic, peer; float period; double t_sent, t_renew; uint32_t seq_sent; } bus_sub;
typedef struct { int topic; float period; } bus_ask;   /* a mirror this board asks for: every period [s] (0: on change) */
typedef struct {
  bus_entry T[BUS_TOPICS]; int nt;
  float pool[BUS_POOL]; int used;
  double now;                          /* the board's clock [s], set by bus_clock */
  bus_sub S[BUS_SUBS]; int ns;         /* who asked for what (bus.c) */
  bus_ask W[BUS_WANTS]; int nw;       /* what this board asks for (bus.c) */
  uint32_t n_pub, n_sent, n_got, n_bad, n_unknown;   /* publishes; topics sent, taken, refused; asked for but not here */
} bus;

static inline uint32_t bus_hash(const char *s) { uint32_t h = 2166136261u; while (*s) { h ^= (uint8_t)*s++; h *= 16777619u; } return h; }
static inline int bus_name_eq(const char *a, const char *b) { int i = 0; for (; i < BUS_NAME; i++) { if (a[i] != b[i]) return 0; if (!a[i]) return 1; } return 1; }
static inline void bus_init(bus *B) { char *p = (char *)B; for (unsigned i = 0; i < sizeof *B; i++) p[i] = 0; }
static inline void bus_clock(bus *B, double t) { B->now = t; }

/* A topic by name, or −1. */
static inline int bus_find(const bus *B, const char *name) { for (int i = 0; i < B->nt; i++) if (bus_name_eq(B->T[i].name, name)) return i; return -1; }
/* Register a topic (or find it, if it is there with the same size and kind): its index, or −1 (no room, a bad name or
 * size, or there already with another size or kind). */
static inline int bus_topic_kind(bus *B, const char *name, int n, int kind) {
  int len = 0; while (name[len] && len < BUS_NAME) len++;
  if (!len || len >= BUS_NAME || n < 1 || n > BUS_VALS) return -1;
  int i = bus_find(B, name);
  if (i >= 0) return B->T[i].n == n && B->T[i].kind == kind ? i : -1;
  if (B->nt >= BUS_TOPICS || B->used + n > BUS_POOL) return -1;
  bus_entry *T = &B->T[i = B->nt++];
  for (int k = 0; k <= len; k++) T->name[k] = name[k];
  T->hash = bus_hash(name); T->n = (uint16_t)n; T->off = (uint16_t)B->used; T->kind = (uint8_t)kind; B->used += n;
  return i;
}
static inline int bus_topic(bus *B, const char *name, int n) { return bus_topic_kind(B, name, n, BUS_LOCAL); }
/* Publish this board's topic: n values (fewer: the rest 0). 0, or −1 (not a topic of this board's, or too many). */
static inline int bus_pub(bus *B, int id, const float *v, int n) {
  if (!B || id < 0 || id >= B->nt || B->T[id].kind != BUS_LOCAL || n > B->T[id].n) return -1;
  bus_entry *T = &B->T[id]; float *d = B->pool + T->off;
  for (int k = 0; k < T->n; k++) d[k] = k < n ? v[k] : 0;
  T->seq++; T->got++; T->t = B->now; T->has = 1; B->n_pub++;
  return 0;
}
/* Its values (NULL until it has been published), and its sequence number and time if asked. */
static inline const float *bus_get(const bus *B, int id, uint32_t *seq, double *t) {
  if (id < 0 || id >= B->nt || !B->T[id].has) return 0;
  if (seq) *seq = B->T[id].seq;
  if (t) *t = B->T[id].t;
  return B->pool + B->T[id].off;
}
/* How old it is now [s], or −1 if it has never been published. */
static inline double bus_age(const bus *B, int id) { return id >= 0 && id < B->nt && B->T[id].has ? B->now - B->T[id].t : -1; }
/* Watching: 1 if it has been published since *seen (then *seen is now), else 0. Start *seen at 0. */
static inline int bus_changed(const bus *B, int id, uint32_t *seen) {
  if (id < 0 || id >= B->nt || !B->T[id].has || B->T[id].seq == *seen) return 0;
  *seen = B->T[id].seq; return 1;
}

/* ── copying between boards (bus.c) ── */
/* Ask for another board's topic: register its mirror here (its name and size as the writer has it) and want it every
 * period [s], or (0) whenever its values change and at least every BUS_BEAT. Its index, or −1. */
int bus_want_topic(bus *B, const char *name, int n, float period);
/* The subscription frame (RN_LINK_BUS_SUB) for what this board wants, into out: floats written (0: wants nothing). */
int bus_sub_pack(const bus *B, float *out, int cap);
/* A subscription frame from peer (any number the caller gives the board it came from), at this board's now. Topics
 * not here (or another size) are counted and left out. Returns how many it took, or −1 if the frame is malformed. */
int bus_sub_take(bus *B, int peer, const float *in, int n);
/* What is due for peer now (RN_LINK_BUS), into out: floats written, 0 if nothing is due. Lapses old subscriptions. */
int bus_pack(bus *B, int peer, float *out, int cap);
/* A data frame: into the mirrors. Returns how many topics it took, or −1 if malformed. */
int bus_unpack(bus *B, const float *in, int n);

#endif
