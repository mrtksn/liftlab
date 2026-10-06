/* Packets over a byte pipe: see pframe.h. */
#include "pframe.h"

int pframe_encode(const uint8_t *pkt, int n, uint8_t *out, int cap) {
  if (n < 0 || cap < PFRAME_WIRE(n)) return 0;
  int k = 0; out[k++] = 0;
  int code_at = k++; uint8_t code = 1;              /* each block: a code (the bytes to the next zero, +1), then the bytes */
  for (int i = 0; i < n; i++) {
    if (pkt[i]) { out[k++] = pkt[i]; code++; }
    if (!pkt[i] || code == 0xFF) {                  /* a zero (dropped: the code says where it was), or a full block */
      out[code_at] = code; code_at = k++; code = 1;
    }
  }
  out[code_at] = code; out[k++] = 0;
  return k;
}

void pframe_rx_init(pframe_rx *R) { uint8_t *p = (uint8_t *)R; for (unsigned i = 0; i < sizeof *R; i++) p[i] = 0; }

/* a whole frame (without its zeros) back to the packet: its length, or −1 if it isn't one */
static int decode(const uint8_t *in, int n, uint8_t *out, int cap) {
  int k = 0, i = 0;
  while (i < n) {
    int code = in[i++]; if (!code) return -1;
    for (int j = 1; j < code; j++) { if (i >= n || k >= cap) return -1; out[k++] = in[i++]; }
    if (code < 0xFF && i < n) { if (k >= cap) return -1; out[k++] = 0; }   /* (the zero it stood for; none after the last block) */
  }
  return k;
}
int pframe_feed(pframe_rx *R, uint8_t b, uint8_t *pkt, int cap) {
  R->bytes++;
  if (b) {
    if (R->n < (int)sizeof R->buf) R->buf[R->n++] = b; else R->over = 1;
    return 0;
  }
  int n = R->n, over = R->over; R->n = 0; R->over = 0;
  if (!n) return 0;                                 /* (between frames: the zero before the next) */
  int m = over ? -1 : decode(R->buf, n, pkt, cap < PFRAME_MAX ? cap : PFRAME_MAX);
  if (m <= 0) { R->bad++; return 0; }
  R->frames++; return m;
}
