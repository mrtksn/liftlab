/* The Pi ↔ drone link framing: see rn_link.h. */
#include "rn_link.h"
#include "rn.h"

void rn_link_init(rn_link *L, uint8_t *buf, uint32_t cap) { L->buf = buf; L->cap = cap; L->state = 0; L->got = 0; L->len = 0; L->type = 0; L->limit = 0; }
void rn_link_reset(rn_link *L) { L->state = 0; L->got = 0; }

static uint32_t crc_update(uint32_t c, const uint8_t *p, uint32_t n) {
  for (uint32_t i = 0; i < n; i++) { c ^= p[i]; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); }
  return c;
}

/* states: 0 wait 'D', 1 wait 'F', 2 header (type + length), 3 payload, 4 CRC */
int rn_link_feed(rn_link *L, uint8_t b) {
  switch (L->state) {
    case 0: if (b == 'D') L->state = 1; return 0;
    case 1: L->state = b == 'F' ? 2 : b == 'D' ? 1 : 0; L->got = 0; return 0;
    case 2:
      L->hdr[L->got++] = b;
      if (L->got < 5) return 0;
      L->type = L->hdr[0]; L->len = (uint32_t)L->hdr[1] | (uint32_t)L->hdr[2] << 8 | (uint32_t)L->hdr[3] << 16 | (uint32_t)L->hdr[4] << 24;
      L->got = 0;
      if (L->len > L->cap || (L->limit && L->len > L->limit(L->type))) { L->state = 0; return -1; }
      L->state = L->len ? 3 : 4; return 0;
    case 3:
      L->buf[L->got++] = b;
      if (L->got == L->len) { L->state = 4; L->got = 0; }
      return 0;
    case 4: {
      L->hdr[L->got++] = b;                     /* the CRC's bytes reuse the header's first four places after it is read */
      if (L->got < 4) return 0;
      uint32_t want = (uint32_t)L->hdr[0] | (uint32_t)L->hdr[1] << 8 | (uint32_t)L->hdr[2] << 16 | (uint32_t)L->hdr[3] << 24;
      uint8_t h[5] = { L->type, (uint8_t)L->len, (uint8_t)(L->len >> 8), (uint8_t)(L->len >> 16), (uint8_t)(L->len >> 24) };
      uint32_t c = ~crc_update(crc_update(0xFFFFFFFFu, h, 5), L->buf, L->len);
      L->state = 0; L->got = 0;
      return c == want ? L->type : -1;
    }
  }
  L->state = 0; return 0;
}

uint32_t rn_link_frame(uint8_t *out, uint32_t cap, uint8_t type, const uint8_t *payload, uint32_t len) {
  if (cap < len + 11) return 0;
  out[0] = 'D'; out[1] = 'F'; out[2] = type;
  for (int i = 0; i < 4; i++) out[3 + i] = (uint8_t)(len >> (8 * i));
  for (uint32_t i = 0; i < len; i++) out[7 + i] = payload[i];
  uint32_t c = ~crc_update(0xFFFFFFFFu, out + 2, 5 + len);
  for (int i = 0; i < 4; i++) out[7 + len + i] = (uint8_t)(c >> (8 * i));
  return len + 11;
}
