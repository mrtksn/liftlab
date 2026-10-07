/* Drones talking to each other: see peer.h. */
#include "peer.h"
#include "plink.h"                  /* (its SipHash and key) */

#define MAGIC 0x50                  /* 'P' */
#define VERSION 1
enum { T_BEACON = 1, T_DATA = 2 };
enum { R_VALS = 1, R_MSG = 2, R_PING = 3, R_PONG = 4 };
#define BEACON_HDR 16               /* magic, version|type, id, session, seq, caps, name length */
#define DATA_HDR 24                 /* magic, version|type, id, session, seq, to id, to session, ack, lq, rssi, flags */
const uint8_t PEER_BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void copy(uint8_t *d, const uint8_t *s, int n) { while (n-- > 0) *d++ = *s++; }
static void zero(void *p, unsigned n) { uint8_t *b = p; while (n--) *b++ = 0; }
static void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }
static uint32_t get16(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8; }
static uint32_t get32(const uint8_t *p) { return get16(p) | get16(p + 2) << 16; }
static void putf(uint8_t *p, float f) { union { float f; uint32_t u; } x; x.f = f; put32(p, x.u); }
static float getf(const uint8_t *p) { union { float f; uint32_t u; } x; x.u = get32(p); return x.f; }

void peer_init(peer_net *N, uint32_t id, uint32_t session, const char *name, const char *phrase) {
  zero(N, sizeof *N);
  N->id = id ? id : 1; N->session = session ? session : 1; N->rnd = N->session ^ 0x9E3779B9u; N->on = 1;
  int k = 0;
  if (name) for (; name[k] && k < PEER_NAME - 1; k++) N->name[k] = name[k];
  N->name[k] = 0;
  plink_key(phrase && phrase[0] ? phrase : "liftlab", &N->k0, &N->k1);
  N->next_beacon = -1;
}
uint32_t peer_id_of(const uint8_t a[6]) { uint32_t v = get32(a + 2); return v ? v : 1; }
void peer_publish(peer_net *N, const float *v, int n) { if (n > PEER_VALS) n = PEER_VALS; if (n < 0) n = 0; for (int i = 0; i < n; i++) N->pub[i] = v[i]; N->npub = n; }
int peer_find(const peer_net *N, uint32_t id) { for (int i = 0; i < PEER_MAX; i++) if (N->P[i].used && N->P[i].id == id) return i; return -1; }

int peer_state(const peer_net *N, int i, double t) {
  const peer_t *P = &N->P[i]; if (!P->used) return -1;
  if (P->known && t - P->t_fresh < PEER_FRESH_S) return PEER_CONNECTED;
  if (t - P->t_heard >= PEER_LOST_S) return PEER_LOST;
  return P->known && t - P->t_fresh < PEER_LOST_S ? PEER_STALE : PEER_HEARD;
}
int peer_lq(const peer_net *N, int i, double t) {
  const peer_t *P = &N->P[i]; if (!P->used || !P->rx_any) return 0;
  float rate = P->rate > PEER_DATA_HZ ? P->rate : (float)PEER_DATA_HZ;
  int w = (uint16_t)(P->rx_top - P->rx_first) + 1; if (w > 32) w = 32;   /* (fewer just after it was first heard) */
  double since = t - P->t_fresh; if (since > 10) return 0;
  int gap = (int)(since * rate) - 1; if (gap < 0) gap = 0; if (gap >= w) return 0;   /* the ones that should have come since */
  int got = 0; for (int k = 0; k < w - gap; k++) got += (int)(P->rx_bits >> k & 1);   /* (bit 0: the newest) */
  return (got * 100 + w / 2) / w;
}
/* a slot for a drone first heard: an empty one, else the one forgotten longest (a lost one) */
static peer_t *slot(peer_net *N, uint32_t id, const uint8_t addr[6], double t) {
  int i = peer_find(N, id); if (i >= 0) return &N->P[i];
  int best = -1;
  for (int k = 0; k < PEER_MAX; k++) {
    if (!N->P[k].used) { best = k; break; }
    if (t - N->P[k].t_heard >= PEER_LOST_S && (best < 0 || N->P[k].t_heard < N->P[best].t_heard)) best = k;
  }
  if (best < 0) return 0;                                            /* (all eight talking: no room) */
  peer_t *P = &N->P[best]; zero(P, sizeof *P);
  P->used = 1; P->id = id; copy(P->addr, addr, 6); P->t_ever = t; P->rtt = -1; P->t_heard = P->t_fresh = P->t_sent = P->t_vals = -1e9;
  return P;
}
/* the other drone started (again): its numbers and our messages to it from the beginning */
static void new_session(peer_t *P, uint32_t ses) {
  P->session = ses; P->known = 0; P->rx_any = 0; P->rx_next = 0; P->beacon_any = 0; P->nvals = 0; P->heard_us = 0; P->ping_out = 0; P->pong_due = 0;
  for (int i = 0; i < P->mq_n; i++) P->mq[i].num = (uint8_t)i;
  P->mq_next = (uint8_t)P->mq_n;
}

int peer_from_air(peer_net *N, const uint8_t addr[6], const uint8_t *p, int n, int rssi, double t) {
  if (!N->on || n < BEACON_HDR + PEER_TAG || n > PEER_MTU || p[0] != MAGIC || (p[1] >> 4) != VERSION) return 0;
  uint64_t tag = 0; for (int k = 7; k >= 0; k--) tag = tag << 8 | p[n - PEER_TAG + k];
  if (tag != plink_siphash(N->k0, N->k1, p, n - PEER_TAG)) { N->N.bad++; return 0; }
  uint32_t id = get32(p + 2), ses = get32(p + 6); uint16_t seq = (uint16_t)get16(p + 10); int type = p[1] & 15;
  if (id == N->id) return 0;                                          /* (our own, heard back) */
  peer_t *P = slot(N, id, addr, t); if (!P) { N->N.dropped++; return 0; }
  if (ses != P->session) {
    if (P->session && t - P->t_heard < 0.5) { N->N.stale_sessions++; return 0; }   /* (not while the one we have still talks: a replay) */
    new_session(P, ses);
  }
  if (type == T_BEACON) {
    if (P->beacon_any && (int16_t)(seq - P->beacon_seq) <= 0) { N->N.replays++; return 0; }
    P->beacon_seq = seq; P->beacon_any = 1;
    int nl = p[15]; if (nl > PEER_NAME - 1) nl = PEER_NAME - 1; if (BEACON_HDR + nl + PEER_TAG > n) nl = n - BEACON_HDR - PEER_TAG;
    for (int k = 0; k < nl; k++) P->name[k] = (char)p[BEACON_HDR + k];
    P->name[nl] = 0;
    copy(P->addr, addr, 6); P->t_heard = t; if (rssi) P->rssi = rssi;
    return 1;
  }
  if (type != T_DATA || n < DATA_HDR + PEER_TAG || get32(p + 12) != N->id) { N->N.bad++; return 0; }
  /* its numbers to us: the newest, which of the last 32 came; replays dropped */
  if (!P->rx_any) { P->rx_top = P->rx_first = seq; P->rx_bits = 1; P->rx_any = 1; }
  else {
    int16_t d = (int16_t)(seq - P->rx_top);
    if (d > 0) {
      double dt = t - P->t_fresh; if (dt > 1e-4 && dt < 2) { float r = (float)(d / dt); P->rate = P->rate > 0 ? P->rate * 0.9f + r * 0.1f : r; }
      P->rx_bits = d >= 32 ? 1 : P->rx_bits << d | 1; P->rx_top = seq;
    } else if (d > -32 && !(P->rx_bits >> -d & 1)) P->rx_bits |= 1u << -d;
    else { N->N.replays++; return 0; }
  }
  copy(P->addr, addr, 6); P->t_heard = t; if (rssi) P->rssi = rssi; N->N.got++;
  /* only a packet naming this start of ours (it can only have heard it from us) goes further */
  if (get32(p + 16) != N->session) return 1;
  P->known = ses; P->t_fresh = t;
  { uint8_t took = p[20];                                            /* our messages it took */
    while (P->mq_n && (uint8_t)(took - P->mq[0].num) < 128) { for (int i = 1; i < P->mq_n; i++) P->mq[i - 1] = P->mq[i]; P->mq_n--; }
    if (p[21] <= 100) P->heard_us = p[21]; }
  int k = DATA_HDR, end = n - PEER_TAG, first = 1;
  while (k < end) {
    int r = p[k++];
    if (r == R_VALS && k < end) {
      int m = p[k++]; if (m > PEER_VALS || k + 4 * m > end) break;
      for (int i = 0; i < m; i++) P->vals[i] = getf(p + k + 4 * i);
      P->nvals = m; P->t_vals = t; k += 4 * m;
    } else if (r == R_MSG && k + 2 <= end) {
      uint8_t num = p[k], len = p[k + 1]; k += 2; if (len > PEER_MSG || k + len > end) break;
      /* the first is the oldest it still has: one beyond the next we want means those between are gone */
      if (first && (uint8_t)(num - P->rx_next) < 128 && num != P->rx_next) { N->N.skipped += (uint8_t)(num - P->rx_next); P->rx_next = num; }
      first = 0;
      if (num == P->rx_next) {
        P->rx_next++; P->msgs_in++;
        if (N->in_n < (int)(sizeof N->in / sizeof N->in[0])) { N->in[N->in_n].from = id; N->in[N->in_n].n = len; copy(N->in[N->in_n].b, p + k, len); N->in_n++; }
        else N->N.dropped++;
      }
      k += len;
    } else if (r == R_PING && k + 4 <= end) { P->pong_stamp = get32(p + k); P->pong_due = 1; k += 4; }
    else if (r == R_PONG && k + 4 <= end) {
      uint32_t s = get32(p + k); k += 4;
      if (P->ping_out && s == P->ping_out) { P->rtt = (float)(t - P->t_ping); P->ping_out = 0; }
    } else break;
  }
  P->taken++;
  return 1;
}

static uint32_t rnd(peer_net *N) { N->rnd ^= N->rnd << 13; N->rnd ^= N->rnd >> 17; N->rnd ^= N->rnd << 5; return N->rnd; }
static int sign(peer_net *N, uint8_t *p, int k) { uint64_t tag = plink_siphash(N->k0, N->k1, p, k); for (int i = 0; i < 8; i++) p[k + i] = (uint8_t)(tag >> (8 * i)); return k + PEER_TAG; }
int peer_to_air(peer_net *N, double t, uint8_t addr[6], uint8_t *p, int cap) {
  if (!N->on || cap < PEER_MTU) return 0;
  if (N->next_beacon < 0) N->next_beacon = t + PEER_BEACON_S * (rnd(N) % 1000) / 1000.0;   /* (the first soon, not all at once) */
  if (t >= N->next_beacon) {                                         /* a beacon, to everyone */
    N->next_beacon = t + PEER_BEACON_S * (0.67 + 0.66 * (rnd(N) % 1000) / 1000.0);
    p[0] = MAGIC; p[1] = VERSION << 4 | T_BEACON; put32(p + 2, N->id); put32(p + 6, N->session); put16(p + 10, N->beacon_seq++);
    put32(p + 12, 0); p[15] = 0;
    int nl = 0; while (N->name[nl]) { p[BEACON_HDR + nl] = (uint8_t)N->name[nl]; nl++; }
    p[15] = (uint8_t)nl;
    copy(addr, PEER_BROADCAST, 6); N->N.beacons++;
    return sign(N, p, BEACON_HDR + nl);
  }
  /* a packet to one drone heard lately: on its beat, or sooner (20 ms after the last) when something waits */
  for (int j = 0; j < PEER_MAX; j++) {
    int i = (N->rr + j) % PEER_MAX; peer_t *P = &N->P[i];
    if (!P->used) continue;
    if (t - P->t_heard >= PEER_FORGET_S) { P->used = 0; continue; }  /* (forgotten) */
    if (t - P->t_heard >= PEER_LOST_S) continue;                       /* (lost: its beacons bring it back) */
    double since = t - P->t_sent;
    int waits = P->pong_due || P->ping_new;                          /* (an answer to give, a ping or a message not sent yet) */
    for (int m = 0; m < P->mq_n && !waits; m++) if (!P->mq[m].tries) waits = 1;
    if (!(since >= 1.0 / PEER_DATA_HZ || (waits && since >= 0.02))) continue;
    N->rr = (i + 1) % PEER_MAX;
    p[0] = MAGIC; p[1] = VERSION << 4 | T_DATA; put32(p + 2, N->id); put32(p + 6, N->session); put16(p + 10, P->seq++);
    put32(p + 12, P->id); put32(p + 16, P->session);                 /* (0 until we've heard its session: it can't take what follows) */
    p[20] = (uint8_t)(P->rx_next - 1); p[21] = (uint8_t)peer_lq(N, i, t); p[22] = (uint8_t)(int8_t)P->rssi; p[23] = 0;
    int k = DATA_HDR, room = PEER_MTU - PEER_TAG;
    if (P->pong_due && k + 5 <= room) { p[k++] = R_PONG; put32(p + k, P->pong_stamp); k += 4; P->pong_due = 0; }
    if (P->ping_out && k + 5 <= room) { p[k++] = R_PING; put32(p + k, P->ping_out); k += 4; P->ping_new = 0; }
    if (N->npub && k + 2 + 4 * N->npub <= room) { p[k++] = R_VALS; p[k++] = (uint8_t)N->npub; for (int v = 0; v < N->npub; v++) { putf(p + k, N->pub[v]); k += 4; } }
    for (int m = 0; m < P->mq_n; m++) {                              /* the messages not taken yet, oldest first, as many as fit */
      int len = P->mq[m].n; if (k + 3 + len > room) break;
      p[k++] = R_MSG; p[k++] = P->mq[m].num; p[k++] = (uint8_t)len; copy(p + k, P->mq[m].b, len); k += len;
      if (P->mq[m].tries++) N->N.resent++;
    }
    copy(addr, P->addr, 6); P->t_sent = t; N->N.sent++;
    return sign(N, p, k);
  }
  return 0;
}
int peer_send(peer_net *N, uint32_t to, const uint8_t *msg, int n) {
  int i = peer_find(N, to); if (i < 0) return -1;
  if (n < 0 || n > PEER_MSG) return -3;
  peer_t *P = &N->P[i]; if (P->mq_n >= PEER_MQ) return -2;
  P->mq[P->mq_n].num = P->mq_next++; P->mq[P->mq_n].n = (uint8_t)n; P->mq[P->mq_n].tries = 0; copy(P->mq[P->mq_n].b, msg, n); P->mq_n++;
  return 0;
}
int peer_ping(peer_net *N, uint32_t to, double t) {
  int i = peer_find(N, to); if (i < 0) return -1;
  peer_t *P = &N->P[i]; P->ping_out = (uint32_t)(t * 1000) | 1u; P->t_ping = t; P->rtt = -1; P->ping_new = 1;
  return 0;
}
int peer_recv(peer_net *N, uint32_t *from, uint8_t *msg, int cap) {
  if (!N->in_n) return -1;
  int n = N->in[0].n < cap ? N->in[0].n : cap; *from = N->in[0].from; copy(msg, N->in[0].b, n);
  for (int i = 1; i < N->in_n; i++) N->in[i - 1] = N->in[i];
  N->in_n--;
  return n;
}
