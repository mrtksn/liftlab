/* The USB port's text and CRSF frames, apart: see usb_split.h. */
#include "usb_split.h"

void usb_split_init(usb_split *S) {
  uint8_t *p = (uint8_t *)S; for (unsigned i = 0; i < sizeof *S; i++) p[i] = 0;
  S->t_frame = -1; S->t_byte = -1;
}
static int is_addr(uint8_t b) { return b == CRSF_ADDR_FC || b == CRSF_ADDR_HANDSET || b == CRSF_ADDR_RX || b == CRSF_ADDR_TX; }

static void text(usb_split *S, uint8_t b, const usb_split_out *O) {
  if (b == '\n' || b == '\r') { if (S->ln) { S->line[S->ln] = 0; S->ln = 0; if (O->line) O->line(O->ctx, S->line); } return; }
  if (b < 0x20 && b != '\t') return;                  /* (other control characters, bytes ≥ 0x80: not text) */
  if (b >= 0x7f) return;
  if (S->ln == USB_SPLIT_LINE - 1) { S->line[S->ln] = 0; S->ln = 0; if (O->line) O->line(O->ctx, S->line); }   /* too long: what came runs, this starts the next */
  S->line[S->ln++] = (char)b;
}

/* One byte; q holds the bytes still to look at, after this one: a frame that wasn't one puts its bytes (but the first)
 * back in front of them. */
#define QN (2 * CRSF_MAX_FRAME + 8)
typedef struct { uint8_t b[QN]; int h, n; } queue;
static void unread(usb_split *S, queue *q) {
  int k = S->fn - 1; S->fn = 0; S->bad++;
  if (k <= 0) return;
  for (int i = k - 1; i >= 0; i--) { q->h = (q->h + QN - 1) % QN; q->b[q->h] = S->f[1 + i]; q->n++; }
}
static void one(usb_split *S, uint8_t b, double t, queue *q, const usb_split_out *O) {
  if (S->fn == 0) { if (is_addr(b)) S->f[S->fn++] = b; else text(S, b, O); return; }
  S->f[S->fn++] = b;
  if (S->fn == 2) { if (b < 2 || b > CRSF_MAX_FRAME - 2) unread(S, q); return; }
  if (S->fn < S->f[1] + 2) return;
  if (crsf_crc8(S->f + 2, S->f[1] - 1) != S->f[S->fn - 1]) { unread(S, q); return; }
  int n = S->fn; S->fn = 0; S->frames++; S->t_frame = t;
  S->ln = 0;                                          /* (text half a line before a frame: the bytes of a broken one, likely: dropped) */
  if (O->frame) O->frame(O->ctx, S->f, n);
}
static void run(usb_split *S, queue *q, double t, const usb_split_out *O) {
  while (q->n) { uint8_t b = q->b[q->h]; q->h = (q->h + 1) % QN; q->n--; one(S, b, t, q, O); }
}
void usb_split_feed(usb_split *S, const uint8_t *b, int n, double t, const usb_split_out *O) {
  queue q; q.h = q.n = 0;
  if (S->fn && t - S->t_byte > USB_SPLIT_STALL) { unread(S, &q); run(S, &q, t, O); }   /* a frame that stopped coming: given up */
  for (int i = 0; i < n; i++) {
    q.h = 0; q.n = 1; q.b[0] = b[i];
    run(S, &q, t, O);
  }
  if (n > 0) S->t_byte = t;
  /* bytes put back that, looked at again, began a frame: they stay in f, and wait for the rest (or the stall) */
}
