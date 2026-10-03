/* CRSF frames: see crsf.h. Portable C, no allocation, no C library beyond what the build provides. */
#include "crsf.h"

uint8_t crsf_crc8(const uint8_t *p, int n) {
  uint8_t c = 0;
  for (int i = 0; i < n; i++) { c ^= p[i]; for (int k = 0; k < 8; k++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xD5) : (uint8_t)(c << 1); }
  return c;
}
int crsf_frame(uint8_t *out, uint8_t addr, uint8_t type, const uint8_t *payload, int n) {
  if (n < 0 || n > CRSF_MAX_PAYLOAD) return 0;
  out[0] = addr; out[1] = (uint8_t)(n + 2); out[2] = type;
  for (int i = 0; i < n; i++) out[3 + i] = payload[i];
  out[3 + n] = crsf_crc8(out + 2, n + 1);
  return n + 4;
}

static float fmodf_(float x, float m) { while (x < 0) x += m; while (x >= m) x -= m; return x; }
static int clampi(long v, long lo, long hi) { return (int)(v < lo ? lo : v > hi ? hi : v); }
static long roundl_(double x) { return (long)(x < 0 ? x - 0.5 : x + 0.5); }
static uint8_t *be16(uint8_t *p, int v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; return p + 2; }
static uint8_t *be24(uint8_t *p, long v) { p[0] = (uint8_t)(v >> 16); p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)v; return p + 3; }
static uint8_t *be32(uint8_t *p, long v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; return p + 4; }

int crsf_battery(uint8_t *out, float volts, float amps, float used_mah, int remaining_pct) {
  uint8_t p[8], *q = p;
  q = be16(q, clampi(roundl_(volts * 10), 0, 65535)); q = be16(q, clampi(roundl_(amps * 10), 0, 65535));
  q = be24(q, clampi(roundl_(used_mah), 0, 0xFFFFFF)); *q++ = (uint8_t)clampi(remaining_pct, 0, 100);
  return crsf_frame(out, CRSF_ADDR_FC, CRSF_BATTERY, p, 8);
}
int crsf_gps(uint8_t *out, double lat, double lon, float gs, float course, float alt, int sats) {
  uint8_t p[15], *q = p;
  q = be32(q, roundl_(lat * 1e7)); q = be32(q, roundl_(lon * 1e7));
  q = be16(q, clampi(roundl_(gs * 36), 0, 65535));                    /* km/h × 10 */
  course = fmodf_(course, 360);
  q = be16(q, clampi(roundl_(course * 100), 0, 35999));
  q = be16(q, clampi(roundl_(alt + 1000), 0, 65535)); *q++ = (uint8_t)clampi(sats, 0, 255);
  return crsf_frame(out, CRSF_ADDR_FC, CRSF_GPS, p, 15);
}
int crsf_attitude(uint8_t *out, float roll, float pitch, float yaw) {
  uint8_t p[6], *q = p;
  q = be16(q, clampi(roundl_(pitch * 10000), -32768, 32767)); q = be16(q, clampi(roundl_(roll * 10000), -32768, 32767));
  q = be16(q, clampi(roundl_(yaw * 10000), -32768, 32767));
  return crsf_frame(out, CRSF_ADDR_FC, CRSF_ATTITUDE, p, 6);
}
int crsf_vario(uint8_t *out, float vz) { uint8_t p[2]; be16(p, clampi(roundl_(vz * 100), -32768, 32767)); return crsf_frame(out, CRSF_ADDR_FC, CRSF_VARIO, p, 2); }
int crsf_baro_alt(uint8_t *out, float alt, float vz) {
  uint8_t p[4], *q = p;
  long dm = roundl_(alt * 10) + 10000;
  q = be16(q, dm >= 0 && dm < 0x8000 ? (int)dm : (0x8000 | clampi(roundl_(alt), 0, 0x7FFF)));   /* decimetres + 10000, or metres with the top bit */
  be16(q, clampi(roundl_(vz * 100), -32768, 32767));
  return crsf_frame(out, CRSF_ADDR_FC, CRSF_BARO_ALT, p, 4);
}
int crsf_flight_mode(uint8_t *out, const char *mode) {
  uint8_t p[16]; int n = 0;
  while (mode[n] && n < 15) { p[n] = (uint8_t)mode[n]; n++; }
  p[n++] = 0;
  return crsf_frame(out, CRSF_ADDR_FC, CRSF_FLIGHT_MODE, p, n);
}
int crsf_text(uint8_t *out, int severity, const char *text) {
  uint8_t p[CRSF_MAX_PAYLOAD]; int n = 0;
  p[n++] = CRSF_EXT_TEXT; p[n++] = (uint8_t)clampi(severity, 0, 7);
  for (int i = 0; text[i] && n < CRSF_MAX_PAYLOAD - 1; i++) p[n++] = (uint8_t)text[i];
  p[n++] = 0;
  return crsf_frame(out, CRSF_ADDR_FC, CRSF_EXT, p, n);
}

/* Link statistics: the RSSIs are sent as positive numbers (−dBm), the SNRs signed, LQ in %. */
int crsf_link_stats(uint8_t *out, uint8_t addr, const crsf_link *L) {
  uint8_t p[10];
  p[0] = p[1] = (uint8_t)clampi(roundl_(-L->up_rssi), 0, 255); p[2] = (uint8_t)clampi(roundl_(L->up_lq), 0, 100);
  p[3] = (uint8_t)(int8_t)clampi(roundl_(L->up_snr), -128, 127); p[4] = (uint8_t)L->antenna; p[5] = (uint8_t)L->rf_mode;
  int pw = L->tx_power_mw, code = pw <= 0 ? 0 : pw <= 10 ? 1 : pw <= 25 ? 2 : pw <= 50 ? 8 : pw <= 100 ? 3 : pw <= 250 ? 7 : pw <= 500 ? 4 : 5;   /* the CRSF power table */
  p[6] = (uint8_t)code;
  p[7] = (uint8_t)clampi(roundl_(-L->down_rssi), 0, 255); p[8] = (uint8_t)clampi(roundl_(L->down_lq), 0, 100);
  p[9] = (uint8_t)(int8_t)clampi(roundl_(L->down_snr), -128, 127);
  return crsf_frame(out, addr, CRSF_LINK_STATS, p, 10);
}
void crsf_link_stats_read(const uint8_t *p, int n, crsf_link *L) {
  static const int pw[] = { 0, 10, 25, 100, 500, 1000, 2000, 250, 50 };
  if (n < 10) return;
  L->up_rssi = -(float)(p[0] < p[1] || p[1] == 0 ? p[0] : p[1]); L->up_lq = p[2]; L->up_snr = (int8_t)p[3];
  L->antenna = p[4]; L->rf_mode = p[5]; L->tx_power_mw = p[6] < 9 ? pw[p[6]] : 0;
  L->down_rssi = -(float)p[7]; L->down_lq = p[8]; L->down_snr = (int8_t)p[9];
}

int crsf_rc(uint8_t *out, uint8_t addr, const float ch[16]) {
  uint8_t p[22] = { 0 }; int bit = 0;
  for (int i = 0; i < 16; i++) {
    float c = ch[i] != ch[i] ? 0 : ch[i] < -1 ? -1 : ch[i] > 1 ? 1 : ch[i];   /* (not a number: the centre, not −1) */
    int v = (int)roundl_(CRSF_CH_MID + c * (CRSF_CH_MAX - CRSF_CH_MID));
    for (int b = 0; b < 11; b++, bit++) if (v & (1 << b)) p[bit >> 3] |= (uint8_t)(1 << (bit & 7));
  }
  return crsf_frame(out, addr, CRSF_RC, p, 22);
}
void crsf_rc_read(const uint8_t *p, float ch[16]) {
  int bit = 0;
  for (int i = 0; i < 16; i++) {
    int v = 0; for (int b = 0; b < 11; b++, bit++) if (p[bit >> 3] & (1 << (bit & 7))) v |= 1 << b;
    float c = (float)(v - CRSF_CH_MID) / (CRSF_CH_MAX - CRSF_CH_MID);
    ch[i] = c < -1 ? -1 : c > 1 ? 1 : c;
  }
}

int crsf_feed(crsf_parser *P, uint8_t b) {
  if (P->n == 0) { if (b == CRSF_ADDR_FC || b == CRSF_ADDR_HANDSET || b == CRSF_ADDR_RX || b == CRSF_ADDR_TX) P->buf[P->n++] = b; return 0; }
  if (P->n == 1) { if (b < 2 || b > CRSF_MAX_FRAME - 2) { P->n = 0; return -1; } P->buf[P->n++] = b; return 0; }
  P->buf[P->n++] = b;
  if (P->n < P->buf[1] + 2) return 0;
  int len = P->n; P->n = 0;
  if (crsf_crc8(P->buf + 2, P->buf[1] - 1) != P->buf[len - 1]) return -1;
  return len;
}
