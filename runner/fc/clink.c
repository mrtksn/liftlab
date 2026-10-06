/* A radio link made of small packets: see clink.h. */
#include "clink.h"

enum { K_HELLO = 0, K_CH = 1, K_ST = 2 };
#define HDR 6                    /* kind, number (2), acknowledgement, which came early, link quality */
#define RC_BYTES 22

static void copy(uint8_t *d, const uint8_t *s, int n) { while (n-- > 0) *d++ = *s++; }
static void zero(void *p, unsigned n) { uint8_t *b = p; while (n--) *b++ = 0; }
static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24; }

void clink_cfg_default(clink_cfg *C, int role) {
  C->role = role; C->up_hz = 100; C->chunk = 21; C->first_ch = 2; C->n_ch = 80;
  plink_key("liftlab", &C->k0, &C->k1);
}
/* the 8 channels: from the key, distinct, at least 3 apart (2 MHz wide at 2 Mbit/s, and a little to spare) */
static void make_hops(clink *L) {
  uint64_t x = L->C.k0 ^ (L->C.k1 * 0x9E3779B97F4A7C15ULL) ^ 0x6E7266ULL; if (!x) x = 1;
  int n = 0, tries = 0, span = L->C.n_ch > 1 ? L->C.n_ch : 1;
  while (n < CLINK_HOPS && tries < 1000) {
    x ^= x << 13; x ^= x >> 7; x ^= x << 17; tries++;
    int c = L->C.first_ch + (int)(x % (uint64_t)span), ok = 1;
    for (int i = 0; i < n; i++) { int d = c - L->hop[i]; if (d < 0) d = -d; if (d < 3) ok = 0; }
    if (ok || tries > 900) L->hop[n++] = (uint8_t)c;
  }
}
void clink_address(const clink_cfg *C, uint8_t a[5]) {
  uint8_t m[4] = { 'a', 'd', 'd', 'r' }; uint64_t h = plink_siphash(C->k0, C->k1, m, 4);
  for (int i = 0; i < 5; i++) {
    uint8_t b = (uint8_t)(h >> (8 * i));
    if (b == 0x00 || b == 0xFF || b == 0x55 || b == 0xAA) b ^= 0x3C;   /* (not like the preamble or a level line) */
    a[i] = b;
  }
}
static void stream_reset(clink_stream *S) { S->sent = 0; S->base = S->hi = 0; S->expect = 0; for (int i = 0; i < CLINK_WIN; i++) { S->sacked[i] = 0; S->early[i] = 0; } }
void clink_init(clink *L, const clink_cfg *C, uint32_t session) {
  zero(L, sizeof *L);
  L->C = *C; if (L->C.chunk > 21 || L->C.chunk < 1) L->C.chunk = 21; if (L->C.up_hz <= 0) L->C.up_hz = 100;
  L->session = session ? session : 1;
  L->t_peer = L->t_stats = L->t_up = L->t_rc = L->t_sent = L->t_rx_last = -1e9;
  stream_reset(&L->tx); stream_reset(&L->rx);
  make_hops(L);
}

/* ── the stack's frames ── */
static void stream_add(clink *L, const uint8_t *f, int n) {
  clink_stream *S = &L->tx;
  if (S->n + n > CLINK_SQ) { L->N.dropped++; return; }     /* (full: the telemetry's budget keeps it from that) */
  copy(S->b + S->n, f, n); S->n += n;
}
void clink_from_stack(clink *L, const uint8_t *b, int n, double t) {
  for (int i = 0; i < n; i++) {
    int len = crsf_feed(&L->P, b[i]); if (len <= 0) continue;
    const uint8_t *f = L->P.buf;
    if (L->C.role == PLINK_GROUND && f[2] == CRSF_RC && f[1] == RC_BYTES + 2) { copy(L->rc, f + 3, RC_BYTES); L->rc_have = 1; L->t_rc = t; }
    else stream_add(L, f, len);
  }
}

/* ── the stream: chunks in flight are base … hi−1 ── */
#define SLOT(c) ((c) % CLINK_WIN)
static int in_flight(const clink_stream *S, uint8_t c) { return (uint8_t)(c - S->base) < (uint8_t)(S->hi - S->base); }
static int offset_of(const clink_stream *S, uint8_t k) { int o = 0; for (uint8_t c = S->base; c != k; c++) o += S->len[SLOT(c)]; return o; }
/* the other end's acknowledgement: all up to a came; of the 8 after a+1, the ones whose bit is set came too */
static void stream_ack(clink *L, uint8_t a, uint8_t early) {
  clink_stream *S = &L->tx;
  if (in_flight(S, a)) {
    int k = 0; for (uint8_t c = S->base; c != (uint8_t)(a + 1); c++) k += S->len[SLOT(c)];
    for (int i = k; i < S->n; i++) S->b[i - k] = S->b[i];
    S->n -= k; S->sent -= k; S->base = (uint8_t)(a + 1);
  }
  for (int i = 0; i < 8; i++) { uint8_t c = (uint8_t)(a + 2 + i); if ((early >> i & 1) && in_flight(S, c)) S->sacked[SLOT(c)] = 1; }
}
/* a chunk in flight to send again now: the oldest not come that is lost (a later one came, or it has been too long),
 * and not just sent; −1 none */
static int lost_chunk(const clink *L, double t) {
  const clink_stream *S = &L->tx; double per = 1.0 / L->C.up_hz;
  for (uint8_t c = S->base; in_flight(S, c); c++) {
    if (S->sacked[SLOT(c)]) continue;
    double since = t - S->t_sent[SLOT(c)];
    int after = 0; for (uint8_t d = (uint8_t)(c + 1); in_flight(S, d); d++) if (S->sacked[SLOT(d)]) { after = 1; break; }
    if (since > 2.5 * per && (after || since > 4 * per)) return c;   /* (2.5 beats: an acknowledgement's way back) */
  }
  return -1;
}
/* the chunk to send now, if any: its number and bytes (pointer into the stream), length; −1 none */
static int stream_chunk(clink *L, double t, uint8_t *num, const uint8_t **data) {
  clink_stream *S = &L->tx;
  int c = lost_chunk(L, t);
  if (c >= 0) {                                                 /* sent again: as it was sent */
    *num = (uint8_t)c; *data = S->b + offset_of(S, (uint8_t)c); S->t_sent[SLOT(c)] = t; L->N.resent++;
    return S->len[SLOT(c)];
  }
  if ((uint8_t)(S->hi - S->base) >= CLINK_WIN || S->sent >= S->n) return -1;
  int n = S->n - S->sent; if (n > L->C.chunk) n = L->C.chunk;
  uint8_t h = S->hi;
  S->len[SLOT(h)] = (uint8_t)n; S->sacked[SLOT(h)] = 0; S->t_sent[SLOT(h)] = t;
  *num = h; *data = S->b + S->sent; S->sent += n; S->hi = (uint8_t)(h + 1);
  return n;
}
static int stream_waiting(clink *L, double t) {
  clink_stream *S = &L->tx;
  return lost_chunk(L, t) >= 0 || ((uint8_t)(S->hi - S->base) < CLINK_WIN && S->sent < S->n);
}
/* receiving: a chunk; the ones in order go to the stack (whole frames), the early ones wait */
static void to_stack(clink *L, const uint8_t *f, int n);
static void stream_bytes(clink *L, const uint8_t *b, int n) { for (int i = 0; i < n; i++) { int len = crsf_feed(&L->RP, b[i]); if (len > 0) to_stack(L, L->RP.buf, len); } }
static void stream_take(clink *L, uint8_t c, const uint8_t *b, int n) {
  clink_stream *S = &L->rx; uint8_t d = (uint8_t)(c - S->expect);
  if (d == 0) {
    stream_bytes(L, b, n); S->expect++;
    while (S->early[SLOT(S->expect)]) { int s = SLOT(S->expect); S->early[s] = 0; stream_bytes(L, S->early_b[s], S->early_n[s]); S->expect++; }
  } else if (d < CLINK_WIN && !S->early[SLOT(c)]) {
    int s = SLOT(c); S->early[s] = 1; S->early_n[s] = (uint8_t)n; copy(S->early_b[s], b, n);
  }
}
static uint8_t early_bits(const clink_stream *S) {
  uint8_t m = 0; for (int i = 0; i < 7; i++) { uint8_t c = (uint8_t)(S->expect + 1 + i); if (S->early[SLOT(c)]) m |= (uint8_t)(1 << i); }
  return m;
}

/* ── packet numbers ── */
static int bit(const clink *L, uint32_t s) { return (int)(L->rx_bits[(s >> 6) & 1] >> (s & 63) & 1); }
static void set_bit(clink *L, uint32_t s, int v) { uint64_t m = 1ULL << (s & 63); if (v) L->rx_bits[(s >> 6) & 1] |= m; else L->rx_bits[(s >> 6) & 1] &= ~m; }
static uint32_t extend(const clink *L, uint16_t lo) {
  uint32_t c = (L->rx_top & 0xFFFF0000u) | lo; int32_t d = (int32_t)(c - L->rx_top);
  if (d > 0x8000) c -= 0x10000; else if (d < -0x8000) c += 0x10000;
  return c;
}
/* 1: new (counted), 0: a replay */
static int take_seq(clink *L, uint32_t s, double t) {
  if (!L->rx_any) { L->rx_top = s; L->rx_first = (uint16_t)s; L->rx_bits[0] = L->rx_bits[1] = 0; L->rx_any = 1; set_bit(L, s, 1); return 1; }
  int32_t d = (int32_t)(s - L->rx_top);
  if (d > 0) {
    double dt = t - L->t_peer;
    if (dt > 1e-4 && dt < 1.0) { float r = (float)(d / dt); L->rate = L->rate > 0 ? L->rate * 0.9f + r * 0.1f : r; }
    for (int32_t k = 1; k <= d && k <= 128; k++) set_bit(L, L->rx_top + (uint32_t)k, 0);
    L->rx_top = s; set_bit(L, s, 1); return 1;
  }
  if (d > -64 && !bit(L, s)) { set_bit(L, s, 1); return 1; }
  L->N.replays++; return 0;
}
int clink_lq(const clink *L, double t) {
  if (!L->rx_any) return 0;
  int w = (uint16_t)((uint16_t)L->rx_top - L->rx_first) + 1; if (w > 100) w = 100;
  float rate = L->C.up_hz; if (L->rate > rate) rate = L->rate;
  int gap = (int)((t - L->t_peer) * rate) - 1; if (gap < 0) gap = 0; if (gap >= w) return 0;
  int got = 0; for (int k = 0; k < w - gap; k++) got += bit(L, L->rx_top - (uint32_t)k);
  return (got * 100 + w / 2) / w;
}

/* the tag: over the sessions (sender's, receiver's), the number's high half, and the packet */
static uint32_t tag_of(const clink *L, const uint8_t *p, int n, uint32_t from, uint32_t to, uint32_t seq, int hello) {
  uint8_t m[CLINK_MTU + 10]; int k = 0;
  if (hello) { for (int i = 0; i < 10; i++) m[k++] = 0; }
  else { put32(m, from); put32(m + 4, to); m[8] = (uint8_t)(seq >> 16); m[9] = (uint8_t)(seq >> 24); k = 10; }
  copy(m + k, p, n); k += n;
  return (uint32_t)plink_siphash(L->C.k0, L->C.k1, m, k);
}
static void new_pair(clink *L, uint32_t peer, uint32_t seq, double t) {
  L->peer = peer; L->knows_me = 0; L->rx_any = 0; L->rate = 0; take_seq(L, seq, t);
  stream_reset(&L->tx); stream_reset(&L->rx); L->RP.n = 0;   /* (the bytes not yet acknowledged go again, from chunk 0) */
}
static void to_stack(clink *L, const uint8_t *f, int n) { if (L->out_n + n <= PLINK_OUT) { copy(L->out + L->out_n, f, n); L->out_n += n; } }

int clink_from_air(clink *L, const uint8_t *p, int n, int rssi, double t) {
  (void)rssi;
  if (n < 1 || n > CLINK_MTU || (p[0] & 0xC0) != 0x40 || (p[0] & 1) == L->C.role) { L->N.bad++; return 0; }
  int kind = (p[0] >> 2) & 3;
  uint32_t tag = get32(p + n - CLINK_TAG);
  if (kind == K_HELLO) {
    if (n != 1 + 4 + 4 + 4 + CLINK_TAG || tag != tag_of(L, p, n - CLINK_TAG, 0, 0, 0, 1)) { L->N.bad++; return 0; }
    uint32_t seq = get32(p + 1), own = get32(p + 5), heard = get32(p + 9);
    if (own != L->peer) {
      if (L->peer && t - L->t_peer < 0.5) { L->N.replays++; return 0; }   /* (not while the one we have still talks) */
      new_pair(L, own, seq, t);
    } else if (!take_seq(L, seq, t)) return 0;
    L->knows_me = heard == L->session;                         /* (another of ours, or none: they don't know this start of ours) */
    L->N.hellos++;
  } else {
    if (!L->peer || n < HDR + CLINK_TAG) { L->N.bad++; return 0; }
    uint32_t seq = extend(L, (uint16_t)(p[1] | p[2] << 8));
    if (tag != tag_of(L, p, n - CLINK_TAG, L->peer, L->session, seq, 0)) { L->N.bad++; return 0; }
    if (!take_seq(L, seq, t)) return 0;
    L->knows_me = 1;
    stream_ack(L, p[3], p[4]);
    if (p[5] <= 100) L->peer_lq = p[5];
    const uint8_t *b = p + HDR; int m = n - HDR - CLINK_TAG;
    if (kind == K_CH && m == RC_BYTES && L->C.role == PLINK_DRONE) { uint8_t f[CRSF_MAX_FRAME]; int k = crsf_frame(f, CRSF_ADDR_FC, CRSF_RC, b, RC_BYTES); to_stack(L, f, k); }
    else if (kind == K_ST && m >= 2) stream_take(L, b[0], b + 1, m - 1);   /* (an empty one carries no number) */
  }
  L->t_peer = t; L->N.got++; L->polled = 1;
  L->rx_seq_last = L->rx_top; L->t_rx_last = t;
  return 1;
}

int clink_to_air(clink *L, double t, uint8_t *p, int cap) {
  if (cap < CLINK_MTU) return 0;
  if (L->C.role == PLINK_GROUND) {                             /* on a fixed beat */
    double per = 1.0 / L->C.up_hz;
    if (t < L->t_up - 1e-6) return 0;
    L->t_up = t - L->t_up < per ? L->t_up + per : t + per;
  } else { if (!L->polled) return 0; L->polled = 0; }          /* the drone: an answer to each */
  int k;
  uint32_t seq = L->seq++;
  if (L->knows_me && t - L->t_peer > 0.5) L->knows_me = 0;      /* quiet: hello again (the other end may have started again) */
  if (!(L->peer && L->knows_me)) {                              /* not a pair yet: hello */
    p[0] = (uint8_t)(0x40 | K_HELLO << 2 | L->C.role); put32(p + 1, seq); put32(p + 5, L->session); put32(p + 9, L->peer); k = 13;
    put32(p + k, tag_of(L, p, k, 0, 0, 0, 1));
  } else {
    int rc_fresh = L->rc_have && t - L->t_rc <= PLINK_RC_STALE;
    int kind = K_ST;
    if (L->C.role == PLINK_GROUND && rc_fresh && !(stream_waiting(L, t) && L->st_turn)) kind = K_CH;
    p[0] = (uint8_t)(0x40 | kind << 2 | L->C.role); p[1] = (uint8_t)seq; p[2] = (uint8_t)(seq >> 8);
    p[3] = (uint8_t)(L->rx.expect - 1); p[4] = early_bits(&L->rx); p[5] = (uint8_t)clink_lq(L, t); k = HDR;
    if (kind == K_CH) { copy(p + k, L->rc, RC_BYTES); k += RC_BYTES; L->st_turn = 1; }
    else {
      uint8_t num; const uint8_t *d; int m = stream_chunk(L, t, &num, &d);
      if (m > 0) { p[k++] = num; copy(p + k, d, m); k += m; }
      L->st_turn = 0;
    }
    put32(p + k, tag_of(L, p, k, L->session, L->peer, seq, 0));
  }
  L->t_sent = t; L->N.sent++;
  return k + CLINK_TAG;
}

int clink_to_stack(clink *L, double t, uint8_t *b, int cap) {
  if (t - L->t_stats >= 0.1) {                                  /* link statistics, as a receiver or a transmitter module */
    L->t_stats = t;
    int conn = clink_connected(L, t), lq = clink_lq(L, t);
    crsf_link S = { 0 }; uint8_t f[CRSF_MAX_FRAME]; int n = 0;
    if (L->C.role == PLINK_DRONE) { if (conn) { S.up_lq = (float)lq; S.down_lq = (float)L->peer_lq; n = crsf_link_stats(f, CRSF_ADDR_FC, &S); } }
    else { S.up_lq = conn ? (float)L->peer_lq : 0; S.down_lq = (float)lq; n = crsf_link_stats(f, CRSF_ADDR_HANDSET, &S); }
    if (n) to_stack(L, f, n);
  }
  int n = L->out_n < cap ? L->out_n : cap;
  copy(b, L->out, n); for (int i = n; i < L->out_n; i++) L->out[i - n] = L->out[i]; L->out_n -= n;
  return n;
}

int clink_channel(const clink *L, double t) {
  if (L->C.role == PLINK_GROUND) return L->hop[L->seq % CLINK_HOPS];
  if (t - L->t_rx_last > 0.5) return L->hop[0];                /* lost: wait where the ground comes every 8th packet */
  double k = (t - L->t_rx_last) * L->C.up_hz + 0.5; uint32_t ahead = k < 1 ? 1 : (uint32_t)k;
  return L->hop[(L->rx_seq_last + ahead) % CLINK_HOPS];
}
