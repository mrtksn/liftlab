/* The data bus: copying topics between boards (bus.h, docs/topic-bus.md).
 *
 * Frames (lists of floats, like the board link's others):
 *   RN_LINK_BUS_SUB  version, count, then per topic: hash high 16 bits, low 16 bits, period [ms] (0: on change), floats
 *   RN_LINK_BUS      version, count, then per topic: hash high, low, sequence (mod 2^24), age [ms] when sent, floats n,
 *                    the values
 * A float holds integers exactly up to 2^24: the hash travels as two halves, the sequence wraps there. */
#include "bus.h"

#define SEQ_MOD 16777216u

static float hi16(uint32_t h) { return (float)(h >> 16); }
static float lo16(uint32_t h) { return (float)(h & 0xffffu); }
static int whole(float x, float lo, float hi) { return x == x && x >= lo && x <= hi && (float)(int32_t)x == x; }   /* (NaN fails) */
static uint32_t join(float hi, float lo) { return ((uint32_t)hi << 16) | (uint32_t)lo; }
static uint32_t sum(const float *v, int n) { uint32_t h = 2166136261u; const uint8_t *b = (const uint8_t *)v; for (int i = 0; i < 4 * n; i++) { h ^= b[i]; h *= 16777619u; } return h; }
static int by_hash(const bus *B, uint32_t h, int kind) { for (int i = 0; i < B->nt; i++) if (B->T[i].hash == h && B->T[i].kind == kind) return i; return -1; }

int bus_want_topic(bus *B, const char *name, int n, float period) {
  int id = bus_topic_kind(B, name, n, BUS_MIRROR);
  if (id < 0) return -1;
  if (!(period >= 0)) period = 0;
  for (int k = 0; k < B->nw; k++) if (B->W[k].topic == id) { B->W[k].period = period; return id; }
  if (B->nw >= BUS_WANTS) return -1;
  B->W[B->nw].topic = id; B->W[B->nw].period = period; B->nw++;
  return id;
}

int bus_sub_pack(const bus *B, float *out, int cap) {
  if (!B->nw || cap < 2 + 4 * B->nw) return 0;
  int k = 0; out[k++] = BUS_VERSION; out[k++] = (float)B->nw;
  for (int w = 0; w < B->nw; w++) {
    const bus_entry *T = &B->T[B->W[w].topic];
    out[k++] = hi16(T->hash); out[k++] = lo16(T->hash); out[k++] = (float)(int32_t)(B->W[w].period * 1000 + 0.5f); out[k++] = (float)T->n;
  }
  return k;
}

int bus_sub_take(bus *B, int peer, const float *in, int n) {
  if (n < 2 || in[0] != BUS_VERSION || !whole(in[1], 0, BUS_WANTS) || n != 2 + 4 * (int)in[1]) return -1;
  int took = 0;
  for (int e = 0; e < (int)in[1]; e++) {
    const float *q = in + 2 + 4 * e;
    if (!whole(q[0], 0, 65535) || !whole(q[1], 0, 65535) || !whole(q[2], 0, 3600000) || !whole(q[3], 1, BUS_VALS)) return -1;
    int id = by_hash(B, join(q[0], q[1]), BUS_LOCAL);
    if (id < 0 || B->T[id].n != (int)q[3]) { B->n_unknown++; continue; }
    int s = 0; while (s < B->ns && !(B->S[s].peer == peer && B->S[s].topic == id)) s++;
    if (s == B->ns) { if (B->ns >= BUS_SUBS) { B->n_unknown++; continue; } B->ns++; B->S[s].t_sent = -1e9; B->S[s].seq_sent = 0; }
    bus_sub *S = &B->S[s]; S->peer = peer; S->topic = id; S->hash = B->T[id].hash; S->period = q[2] / 1000; S->t_renew = B->now;
    took++;
  }
  return took;
}

int bus_pack(bus *B, int peer, float *out, int cap) {
  for (int s = 0; s < B->ns;) if (B->now - B->S[s].t_renew > BUS_LAPSE) B->S[s] = B->S[--B->ns]; else s++;   /* lapsed */
  if (cap < 2) return 0;
  int k = 2, count = 0;
  for (int s = 0; s < B->ns; s++) {
    bus_sub *S = &B->S[s]; const bus_entry *T = &B->T[S->topic];
    if (S->peer != peer || !T->has || T->seq == S->seq_sent) continue;
    const float *v = B->pool + T->off; uint32_t h = 0;
    if (S->period > 0) { if (B->now - S->t_sent < S->period - 1e-9) continue; }
    else if ((h = sum(v, T->n)) == S->sum_sent && B->now - S->t_sent < BUS_BEAT - 1e-9) continue;   /* on change, with a heartbeat */
    if (k + 5 + T->n > cap) break;                                   /* (the rest next time) */
    float age = (float)((B->now - T->t) * 1000); if (age < 0) age = 0;
    out[k++] = hi16(T->hash); out[k++] = lo16(T->hash); out[k++] = (float)(T->seq % SEQ_MOD); out[k++] = age; out[k++] = (float)T->n;
    for (int i = 0; i < T->n; i++) out[k++] = v[i];
    S->seq_sent = T->seq; S->sum_sent = h; S->t_sent = B->now; count++; B->n_sent++;
  }
  if (!count) return 0;
  out[0] = BUS_VERSION; out[1] = (float)count;
  return k;
}

int bus_unpack(bus *B, const float *in, int n) {
  if (n < 2 || in[0] != BUS_VERSION || !whole(in[1], 0, BUS_TOPICS)) return -1;
  int k = 2, took = 0;
  for (int e = 0; e < (int)in[1]; e++) {
    if (k + 5 > n) return -1;
    const float *q = in + k;
    if (!whole(q[0], 0, 65535) || !whole(q[1], 0, 65535) || !whole(q[2], 0, SEQ_MOD) || !(q[3] >= 0 && q[3] < 1e9f) || !whole(q[4], 1, BUS_VALS) || k + 5 + (int)q[4] > n) return -1;
    int m = (int)q[4], id = by_hash(B, join(q[0], q[1]), BUS_MIRROR);
    k += 5;
    if (id < 0) { B->n_unknown++; k += m; continue; }
    bus_entry *T = &B->T[id];
    if (T->n != m) { T->bad++; B->n_bad++; k += m; continue; }
    float *d = B->pool + T->off; for (int i = 0; i < m; i++) d[i] = in[k + i];
    k += m; T->seq = (uint32_t)q[2]; T->t = B->now - q[3] / 1000; T->has = 1; T->got++; took++; B->n_got++;
  }
  return k == n ? took : -1;
}
