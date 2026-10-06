/* A radio link made of packets: see plink.h. */
#include "plink.h"

#define MAGIC 0x4C             /* 'L' */
#define VERSION 1
enum { REC_ONCE = 1, REC_RELIABLE = 2 };

static void copy(uint8_t *d, const uint8_t *s, int n) { while (n-- > 0) *d++ = *s++; }
static void put16(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v) { put16(p, v); put16(p + 2, v >> 16); }
static uint32_t get16(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8; }
static uint32_t get32(const uint8_t *p) { return get16(p) | get16(p + 2) << 16; }

/* SipHash-2-4 (Aumasson, Bernstein): a 64-bit tag of a message under a 128-bit key. */
#define ROTL(x, b) (uint64_t)(((x) << (b)) | ((x) >> (64 - (b))))
#define SIPROUND do { v0 += v1; v1 = ROTL(v1, 13); v1 ^= v0; v0 = ROTL(v0, 32); v2 += v3; v3 = ROTL(v3, 16); v3 ^= v2; \
  v0 += v3; v3 = ROTL(v3, 21); v3 ^= v0; v2 += v1; v1 = ROTL(v1, 17); v1 ^= v2; v2 = ROTL(v2, 32); } while (0)
static uint64_t siphash(uint64_t k0, uint64_t k1, const uint8_t *m, int n) {
  uint64_t v0 = 0x736f6d6570736575ULL ^ k0, v1 = 0x646f72616e646f6dULL ^ k1, v2 = 0x6c7967656e657261ULL ^ k0, v3 = 0x7465646279746573ULL ^ k1;
  int end = n - n % 8;
  for (int i = 0; i < end; i += 8) {
    uint64_t w = 0; for (int k = 7; k >= 0; k--) w = w << 8 | m[i + k];
    v3 ^= w; SIPROUND; SIPROUND; v0 ^= w;
  }
  uint64_t b = (uint64_t)n << 56; for (int k = n % 8 - 1; k >= 0; k--) b |= (uint64_t)m[end + k] << (8 * k);
  v3 ^= b; SIPROUND; SIPROUND; v0 ^= b;
  v2 ^= 0xff; SIPROUND; SIPROUND; SIPROUND; SIPROUND;
  return v0 ^ v1 ^ v2 ^ v3;
}
void plink_key(const char *phrase, uint64_t *k0, uint64_t *k1) {
  int n = 0; while (phrase[n]) n++;
  *k0 = siphash(0x4c6966744c616231ULL, 0x6b65792d30303031ULL, (const uint8_t *)phrase, n);   /* (two fixed keys: "LiftLab1", "key-0001") */
  *k1 = siphash(0x6b65792d30303032ULL, 0x4c6966744c616232ULL, (const uint8_t *)phrase, n);
}

void plink_cfg_default(plink_cfg *C, int role) {
  C->role = role; C->mtu = PLINK_MTU; C->up_hz = 100; C->down_hz_min = 20; C->down_hz_max = 100;
  plink_key("liftlab", &C->k0, &C->k1);
}
void plink_init(plink *L, const plink_cfg *C, uint32_t session) {
  uint8_t *p = (uint8_t *)L; for (unsigned i = 0; i < sizeof *L; i++) p[i] = 0;
  L->C = *C; if (L->C.mtu > PLINK_MTU) L->C.mtu = PLINK_MTU;
  L->session = session ? session : 1;
  L->t_peer = L->t_sent = L->t_stats = L->t_up = L->t_rc = -1e9; L->peer_lq = L->peer_rssi = 0; L->peer_took = 255;
}

/* ── what the stack writes ── */
static int reliable(const uint8_t *f) { return f[2] == CRSF_EXT && f[1] >= 3 && (f[3] == CRSF_EXT_TEXT || f[3] == CRSF_EXT_CMD); }
static void queue_once(plink *L, const uint8_t *f, int n) {
  while (L->uq_n + n > PLINK_UQ && L->uq_n) {                       /* full: the oldest go (they're state, the newer say more) */
    int k = L->uq[1] + 2; for (int i = k; i < L->uq_n; i++) L->uq[i - k] = L->uq[i]; L->uq_n -= k; L->N.uq_dropped++;
  }
  copy(L->uq + L->uq_n, f, n); L->uq_n += n;
}
static void queue_reliable(plink *L, const uint8_t *f, int n) {
  if (L->rq_n == PLINK_RQ) { for (int i = 1; i < PLINK_RQ; i++) L->rq[i - 1] = L->rq[i]; L->rq_n--; L->N.uq_dropped++; }   /* (never in practice: 16 waiting) */
  L->rq[L->rq_n].num = L->rq_next++; L->rq[L->rq_n].n = (uint8_t)n; L->rq[L->rq_n].tries = 0; copy(L->rq[L->rq_n].f, f, n); L->rq_n++;
}
void plink_from_stack(plink *L, const uint8_t *b, int n, double t) {
  for (int i = 0; i < n; i++) {
    int len = crsf_feed(&L->P, b[i]); if (len <= 0) continue;
    const uint8_t *f = L->P.buf;
    if (L->C.role == PLINK_GROUND && f[2] == CRSF_RC) { copy(L->rc, f, len); L->rc_n = len; L->t_rc = t; }
    else if (reliable(f)) queue_reliable(L, f, len);
    else queue_once(L, f, len);
  }
}

/* ── their packet numbers: which came ── */
static int bit(const plink *L, uint16_t s) { return (int)(L->rx_bits[(s >> 6) & 1] >> (s & 63) & 1); }
static void set_bit(plink *L, uint16_t s, int v) { uint64_t m = 1ULL << (s & 63); if (v) L->rx_bits[(s >> 6) & 1] |= m; else L->rx_bits[(s >> 6) & 1] &= ~m; }
int plink_lq(const plink *L, double t) {
  if (!L->rx_any) return 0;
  int w = (uint16_t)(L->rx_top - L->rx_first) + 1; if (w > 100) w = 100;   /* the last 100 numbers (fewer just after the start) */
  float rate = L->C.role == PLINK_DRONE ? L->C.up_hz : L->C.down_hz_min;   /* the other end's packets a second: at least what it must send, */
  if (L->rate > rate) rate = L->rate;                                         /* more if it has been sending more */
  int gap = (int)((t - L->t_peer) * rate) - 1; if (gap < 0) gap = 0; if (gap >= w) return 0;   /* the ones that should have come since */
  int got = 0; for (int k = 0; k < w - gap; k++) got += bit(L, (uint16_t)(L->rx_top - k));
  return (got * 100 + w / 2) / w;
}

/* ── a packet that came ── */
static void to_stack(plink *L, const uint8_t *f, int n) { if (L->out_n + n <= PLINK_OUT) { copy(L->out + L->out_n, f, n); L->out_n += n; } }
int plink_from_air(plink *L, const uint8_t *p, int n, int rssi, double t) {
  if (n < PLINK_HDR + PLINK_TAG || n > PLINK_MTU || p[0] != MAGIC || (p[1] >> 4) != VERSION) { L->N.bad++; return 0; }
  uint64_t tag = 0; for (int k = 7; k >= 0; k--) tag = tag << 8 | p[n - PLINK_TAG + k];
  if (tag != siphash(L->C.k0, L->C.k1, p, n - PLINK_TAG) || (int)(p[1] & 15) == L->C.role) { L->N.bad++; return 0; }
  uint16_t seq = (uint16_t)get16(p + 2); uint32_t ses = get32(p + 4);
  if (ses != L->peer) {                                              /* the other end started (again) */
    if (L->peer && t - L->t_peer < 0.5) { L->N.stale_sessions++; return 0; }   /* (not while the one we have still talks: a replay) */
    L->peer = ses; L->rx_any = 0; L->rx_next = 0;
    for (int i = 0; i < L->rq_n; i++) L->rq[i].num = (uint8_t)i;    /* it takes our reliable frames from 0 again */
    L->rq_next = (uint8_t)L->rq_n; L->peer_took = 255;
  }
  if (!L->rx_any) { L->rx_top = L->rx_first = seq; L->rx_bits[0] = L->rx_bits[1] = 0; L->rx_any = 1; set_bit(L, seq, 1); }
  else {
    int16_t d = (int16_t)(seq - L->rx_top);
    if (d > 0) {
      double dt = t - L->t_peer;
      if (dt > 1e-4 && dt < 1.0) { float r = (float)(d / dt); L->rate = L->rate > 0 ? L->rate * 0.9f + r * 0.1f : r; }   /* (numbers a second: lost ones count) */
      for (int k = 1; k <= d && k <= 128; k++) set_bit(L, (uint16_t)(L->rx_top + k), 0);
      L->rx_top = seq; set_bit(L, seq, 1);
    }
    else if (d > -64 && !bit(L, seq)) set_bit(L, seq, 1);           /* late, not seen: fine */
    else { L->N.replays++; return 0; }
  }
  L->t_peer = t; L->N.got++;
  if (rssi) L->rssi = L->rssi ? (L->rssi * 3 + rssi) / 4 : rssi;
  /* what they took of ours, and what they hear of us: only if it's about us (not a session of ours before a restart) */
  if (get32(p + 8) == L->session) {
    uint8_t took = p[12];
    while (L->rq_n && (uint8_t)(took - L->rq[0].num) < 128) { for (int i = 1; i < L->rq_n; i++) L->rq[i - 1] = L->rq[i]; L->rq_n--; }
    L->peer_took = took;
    if (p[13] <= 100) L->peer_lq = p[13];
    L->peer_rssi = (int8_t)p[14];
  } else { L->peer_lq = 0; L->peer_rssi = 0; }
  /* the records */
  int k = PLINK_HDR, end = n - PLINK_TAG;
  while (k + 1 < end) {
    int kind = p[k++], num = -1;
    if (kind == REC_RELIABLE) num = p[k++];
    if (k + 2 > end) break;
    int len = p[k + 1] + 2; if (len < 4 || k + len > end) break;
    if (num < 0) to_stack(L, p + k, len);
    else if ((uint8_t)num == L->rx_next) { to_stack(L, p + k, len); L->rx_next++; }   /* the next one: in order, once */
    k += len;
  }
  return 1;
}

/* ── the packet to send ── */
int plink_to_air(plink *L, double t, uint8_t *p, int cap) {
  int mtu = L->C.mtu < cap ? L->C.mtu : cap; if (mtu < PLINK_HDR + PLINK_TAG + 8) return 0;
  double since = t - L->t_sent;
  if (L->C.role == PLINK_GROUND) {                                   /* on a fixed beat: asked every 4 ms, a 10 ms beat stays 10 */
    double per = 1.0 / L->C.up_hz;
    if (t < L->t_up - 1e-6) return 0;
    L->t_up = t - L->t_up < per ? L->t_up + per : t + per;           /* (late by more than a beat, after a stall: from now) */
  }
  else {
    int has = L->uq_n || L->rq_n;
    if (!(since >= 1.0 / L->C.down_hz_min - 1e-6 || (has && since >= 1.0 / L->C.down_hz_max - 1e-6))) return 0;
  }
  p[0] = MAGIC; p[1] = (uint8_t)(VERSION << 4 | L->C.role); put16(p + 2, L->seq++); put32(p + 4, L->session);
  put32(p + 8, L->peer);                                             /* who we talk to: what follows is about them */
  p[12] = (uint8_t)(L->rx_next - 1);
  p[13] = (uint8_t)plink_lq(L, t); p[14] = (uint8_t)(int8_t)L->rssi; p[15] = 0;
  int k = PLINK_HDR, room = mtu - PLINK_TAG;
  for (int i = 0; i < L->rq_n; i++) {                                /* the reliable ones not yet taken, oldest first */
    int n = L->rq[i].n; if (k + 2 + n > room) break;
    p[k++] = REC_RELIABLE; p[k++] = L->rq[i].num; copy(p + k, L->rq[i].f, n); k += n;
    if (L->rq[i].tries++) L->N.resent++;
  }
  if (L->rc_n && t - L->t_rc <= PLINK_RC_STALE && k + 1 + L->rc_n <= room) { p[k++] = REC_ONCE; copy(p + k, L->rc, L->rc_n); k += L->rc_n; }   /* the channels as they are now */
  int u = 0;                                                         /* then the frames that go once, as many as fit */
  while (u < L->uq_n) { int n = L->uq[u + 1] + 2; if (k + 1 + n > room) break; p[k++] = REC_ONCE; copy(p + k, L->uq + u, n); k += n; u += n; }
  if (u) { for (int i = u; i < L->uq_n; i++) L->uq[i - u] = L->uq[i]; L->uq_n -= u; }
  uint64_t tag = siphash(L->C.k0, L->C.k1, p, k);
  for (int i = 0; i < 8; i++) p[k + i] = (uint8_t)(tag >> (8 * i));
  L->t_sent = t; L->N.sent++;
  return k + PLINK_TAG;
}

/* ── what the stack reads ── */
int plink_to_stack(plink *L, double t, uint8_t *b, int cap) {
  if (t - L->t_stats >= 0.1) {                                       /* link statistics, as the module or the receiver would give */
    L->t_stats = t;
    int conn = plink_connected(L, t), lq = plink_lq(L, t);
    crsf_link S = { 0 }; uint8_t f[CRSF_MAX_FRAME]; int n = 0;
    if (L->C.role == PLINK_DRONE) {                                  /* a receiver: while it hears the ground */
      if (conn) { S.up_rssi = (float)L->rssi; S.up_lq = (float)lq; S.down_rssi = (float)L->peer_rssi; S.down_lq = (float)L->peer_lq; n = crsf_link_stats(f, CRSF_ADDR_FC, &S); }
    } else {                                                         /* a transmitter module: always, LQ 0 while not connected */
      S.up_rssi = conn ? (float)L->peer_rssi : 0; S.up_lq = conn ? (float)L->peer_lq : 0; S.down_rssi = (float)L->rssi; S.down_lq = (float)lq;
      n = crsf_link_stats(f, CRSF_ADDR_HANDSET, &S);
    }
    if (n) to_stack(L, f, n);
  }
  int n = L->out_n < cap ? L->out_n : cap;
  copy(b, L->out, n); for (int i = n; i < L->out_n; i++) L->out[i - n] = L->out[i]; L->out_n -= n;
  return n;
}
