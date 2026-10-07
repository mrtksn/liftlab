/* The fleet program's link on the peer end's board: see fleet.h. */
#include "fleet.h"

static float lo16(uint32_t v) { return (float)(v & 0xFFFF); }
static float hi16(uint32_t v) { return (float)(v >> 16); }
static uint32_t join16(float lo, float hi) { return (uint32_t)(lo < 0 ? 0 : lo > 65535 ? 65535 : lo) | (uint32_t)(hi < 0 ? 0 : hi > 65535 ? 65535 : hi) << 16; }
static int fin(float x) { return (x - x) == 0; }

void fleet_link_init(fleet_link *K) { char *p = (char *)K; for (unsigned i = 0; i < sizeof *K; i++) p[i] = 0; }

void fleet_link_publish(fleet_link *K, peer_net *N, const float head[FLEET_HEAD], double t) {
  float v[FLEET_PUB]; int n = 0;
  for (int i = 0; i < FLEET_HEAD; i++) v[n++] = head[i];
  if (K->n_ext && t - K->t_ext < FLEET_STALE_S) for (int i = 0; i < K->n_ext; i++) v[n++] = K->ext[i];   /* (gone stale: the head alone) */
  peer_publish(N, v, n);
}

int fleet_link_pack(peer_net *N, double t, const float head[FLEET_HEAD], float *o) {
  int k = 0, ns = 0, nm = 0;
  o[k++] = 1; o[k++] = lo16(N->id); o[k++] = hi16(N->id); o[k++] = head[0]; o[k++] = head[1];
  int at_ns = k++, at_nm = k++;
  for (int i = 0; i < PEER_MAX; i++) {
    const peer_t *P = &N->P[i]; int s = peer_state(N, i, t); if (s < 0) continue;
    o[k++] = lo16(P->id); o[k++] = hi16(P->id); o[k++] = (float)s; o[k++] = (float)peer_lq(N, i, t); o[k++] = (float)P->heard_us;
    o[k++] = (float)(t - P->t_heard); o[k++] = P->nvals ? (float)(t - P->t_vals) : -1; o[k++] = (float)P->nvals;
    for (int j = 0; j < P->nvals; j++) o[k++] = P->vals[j];
    ns++;
  }
  /* the program's messages that came (others aren't the program's: dropped) */
  uint8_t b[PEER_MSG]; uint32_t from; int n;
  while (nm < FLEET_MSG && (n = peer_recv(N, &from, b, sizeof b)) >= 0) {
    if (n < 2 || b[0] != FLEET_MSG_TAG || b[1] > FLEET_VALS || n != 2 + 4 * b[1]) continue;
    o[k++] = lo16(from); o[k++] = hi16(from); o[k++] = b[1];
    for (int j = 0; j < b[1]; j++) { uint32_t u = (uint32_t)b[2 + 4 * j] | (uint32_t)b[3 + 4 * j] << 8 | (uint32_t)b[4 + 4 * j] << 16 | (uint32_t)b[5 + 4 * j] << 24; float f; uint8_t *d = (uint8_t *)&f, *s = (uint8_t *)&u; for (int q = 0; q < 4; q++) d[q] = s[q]; o[k++] = f; }
    nm++;
  }
  o[at_ns] = (float)ns; o[at_nm] = (float)nm;
  return k;
}

int fleet_link_apply(fleet_link *K, peer_net *N, const float *in, int n, double t) {
  if (n < 3 || in[0] != 1) return -1;
  int k = 1, ne = (int)in[k++];
  if (ne < 0 || ne > FLEET_EXT + FLEET_VALS || k + ne + 1 > n) return -1;
  for (int i = 0; i < ne; i++) if (!fin(in[k + i])) return -1;
  for (int i = 0; i < ne; i++) K->ext[i] = in[k++];
  K->n_ext = ne; K->t_ext = t;
  int nm = (int)in[k++], sent = 0;
  if (nm < 0 || nm > 2 * FLEET_MSG) return -1;
  for (int m = 0; m < nm; m++) {
    if (k + 3 > n) return -1;
    uint32_t to = join16(in[k], in[k + 1]); int c = (int)in[k + 2]; k += 3;
    if (c < 0 || c > FLEET_VALS || k + c > n) return -1;
    uint8_t b[2 + 4 * FLEET_VALS]; b[0] = FLEET_MSG_TAG; b[1] = (uint8_t)c;
    for (int j = 0; j < c; j++) { float f = in[k++]; const uint8_t *s = (const uint8_t *)&f; for (int q = 0; q < 4; q++) b[2 + 4 * j + q] = s[q]; }   /* (little-endian: the ESP32, the Pi, WebAssembly) */
    if (to) sent += !peer_send(N, to, b, 2 + 4 * c);
    else for (int i = 0; i < PEER_MAX; i++) if (peer_state(N, i, t) >= PEER_STALE && N->P[i].known) sent += !peer_send(N, N->P[i].id, b, 2 + 4 * c);   /* (to every drone it has a session with) */
  }
  return sent;
}
