/* The pilot's radio link: see radio_link.h. (No C library: it also builds for the simulator.) */
#include "radio_link.h"

const char *const rlink_names[RLINK_KINDS] = { "elrs", "espnow", "wifi", "serial", "nrf24", "ble" };
const char *const rlink_labels[RLINK_KINDS] = { "ExpressLRS 2.4 GHz", "ESP-NOW (ESP32 to ESP32)", "Wi-Fi (UDP)", "Serial line (laser, fibre, radio modem)", "nRF24L01 2.4 GHz", "Bluetooth LE (ESP32 to ESP32)" };

void rlink_default(rlink_cfg *L) { L->kind = RLINK_ELRS; L->rate_hz = 250; L->ratio = 4; L->channel = 1; L->lr = 0; L->sta = 0; L->baud = 115200; L->half = 0; L->kbps = 1000; L->dir = RLINK_BOTH; }

static int say(char *err, int en, const char *s) { int k = 0; if (en > 0) { while (s[k] && k < en - 1) { err[k] = s[k]; k++; } err[k] = 0; } return -1; }
static int same(const char *a, int n, const char *b) { int k = 0; while (k < n && b[k] && a[k] == b[k]) k++; return k == n && !b[k]; }
/* whole numbers separated by commas, all of s: how many (up to max), or −1 if anything else is there */
static int ints(const char *s, int *v, int max) {
  int n = 0;
  while (*s) {
    int neg = *s == '-'; if (neg) s++;
    if (*s < '0' || *s > '9' || n >= max) return -1;
    long x = 0; while (*s >= '0' && *s <= '9') { x = x * 10 + (*s - '0'); if (x > 100000000) return -1; s++; }
    v[n++] = (int)(neg ? -x : x);
    if (*s == ',') { s++; if (!*s) return -1; } else if (*s) return -1;
  }
  return n;
}

static int elrs_ok(int rate, int ratio) { return (rate == 50 || rate == 150 || rate == 250 || rate == 500) && ratio >= 2 && ratio <= 128; }
int rlink_parse(rlink_cfg *L, const char *s, char *err, int en) {
  int k = 0; while (s[k] && s[k] != ',') k++;
  int kind = -1; for (int i = 0; i < RLINK_KINDS; i++) if (same(s, k, rlink_names[i])) kind = i;
  const char *v = s;
  if (kind >= 0) v = s[k] ? s + k + 1 : s + k;
  else if (s[0] >= '0' && s[0] <= '9') kind = RLINK_ELRS;            /* (just numbers: ExpressLRS's, as before) */
  else return say(err, en, "no such link (elrs, espnow, wifi, serial, nrf24 or ble)");
  int x[2] = { 0, 0 };
  if (kind == RLINK_ELRS && (ints(v, x, 2) != 2 || rlink_make(L, kind, x[0], x[1])))
    return say(err, en, "elrs,rate,ratio as set on the radio: 50, 150, 250 or 500 Hz; telemetry 1:2 to 1:128");
  if (kind == RLINK_ESPNOW) {                                        /* espnow,CHANNEL[,lr] */
    int k2 = 0; while (v[k2] && v[k2] != ',') k2++;
    char num[8]; int m = 0; while (m < k2 && m < 7) { num[m] = v[m]; m++; } num[m] = 0;
    int lr = v[k2] == ',' ? (same(v + k2 + 1, 2, "lr") && !v[k2 + 3] ? 1 : -1) : 0;
    if (k2 == 0 || k2 > 7 || ints(num, x, 1) != 1 || lr < 0 || rlink_make(L, kind, x[0], lr)) return say(err, en, "espnow,channel[,lr]: a Wi-Fi channel 1 to 13, and lr for the long-range mode");
  }
  if (kind == RLINK_WIFI) {                                          /* wifi,ap,CHANNEL or wifi,sta */
    if (same(v, 3, "sta") && !v[3]) { if (rlink_make(L, kind, 1, 1)) return -1; }
    else if (v[0] == 'a' && v[1] == 'p' && v[2] == ',' && ints(v + 3, x, 1) == 1 && !rlink_make(L, kind, 0, x[0])) {}
    else return say(err, en, "wifi,ap,channel (the drone makes the network, channel 1 to 13) or wifi,sta (it joins one)");
  }
  if (kind == RLINK_BLE && (*v || rlink_make(L, kind, 0, 0))) return say(err, en, "ble: no settings (both ends ESP32s with the same binding phrase)");
  if (kind == RLINK_NRF24 && (ints(v, x, 2) != 1 || rlink_make(L, kind, x[0], 0)))   /* nrf24,KBPS */
    return say(err, en, "nrf24,rate: the air data rate in kbit/s, 250, 1000 or 2000 (250 reaches furthest), the same at both ends");
  if (kind == RLINK_SERIAL) {                                        /* serial,BAUD[,half|,up|,down] */
    int k2 = 0; while (v[k2] && v[k2] != ',') k2++;
    char num[12]; int m = 0; while (m < k2 && m < 11) { num[m] = v[m]; m++; } num[m] = 0;
    const char *o = v + k2 + 1;
    int opt = v[k2] != ',' ? 0 : same(o, 4, "half") && !o[4] ? 1 : same(o, 2, "up") && !o[2] ? 2 : same(o, 4, "down") && !o[4] ? 3 : -1;
    if (k2 == 0 || k2 > 11 || ints(num, x, 1) != 1 || opt < 0) return say(err, en, "serial,baud[,half|up|down]: the line's speed, the same at both ends (115200, say); half for a line that goes one way at a time (most radio modems); up or down for one that goes one way only");
    if (rlink_make(L, kind, x[0], opt)) return say(err, en, opt == 1 ? "serial,baud,half: 38400 to 4000000 baud (slower can't carry the channels often enough, answers and all)" : "serial,baud: 19200 to 4000000 baud (slower can't carry the channels often enough)");
  }
  return 0;
}
int rlink_make(rlink_cfg *L, int kind, int a, int b) {
  rlink_cfg c; rlink_default(&c); c.kind = kind;
  if (kind == RLINK_ELRS && elrs_ok(a, b)) { c.rate_hz = a; c.ratio = b; }
  else if (kind == RLINK_ESPNOW && a >= 1 && a <= 13 && (b == 0 || b == 1)) { c.channel = a; c.lr = b; }
  else if (kind == RLINK_WIFI && (a == 0 || a == 1) && b >= 1 && b <= 13) { c.sta = a; c.channel = b; }
  else if (kind == RLINK_BLE && a == 0 && b == 0) {}
  else if (kind == RLINK_NRF24 && (a == 250 || a == 1000 || a == 2000) && b == 0) c.kbps = a;
  else if (kind == RLINK_SERIAL && b >= 0 && b <= 3 && a >= (b == 1 ? RLINK_BAUD_HALF_MIN : RLINK_BAUD_MIN) && a <= RLINK_BAUD_MAX) { c.baud = a; c.half = b == 1; c.dir = b == 2 ? RLINK_UP : b == 3 ? RLINK_DOWN : RLINK_BOTH; }
  else return -1;
  *L = c; return 0;
}
int rlink_pair_ok(const rlink_cfg *a, const rlink_cfg *b, int esp32, char *err, int en) {
  int ka = a->kind, kb = b->kind;
  int wifi_a = ka == RLINK_ESPNOW || ka == RLINK_WIFI, wifi_b = kb == RLINK_ESPNOW || kb == RLINK_WIFI;
  int uart_a = ka == RLINK_ELRS || ka == RLINK_SERIAL, uart_b = kb == RLINK_ELRS || kb == RLINK_SERIAL;
  if (ka == kb && !(ka == RLINK_SERIAL && !esp32)) return say(err, en, "the two links must be different kinds (two serial lines only on a Pi or a computer, on two ports)");
  if (wifi_a && wifi_b) return say(err, en, "ESP-NOW and Wi-Fi share the ESP32's one 2.4 GHz radio: pick one of them");
  if (esp32 && uart_a && uart_b) return say(err, en, "ExpressLRS and a serial line both need the radio's UART (crsf= or tx=): only one of them on an ESP32");
  if (esp32 && ((ka == RLINK_BLE && wifi_b) || (kb == RLINK_BLE && wifi_a))) return say(err, en, "Bluetooth LE beside ESP-NOW or Wi-Fi: not enough memory on the ESP32: pick another pair");
  if (!rlink_up(a) && !rlink_up(b)) return say(err, en, "neither link carries the channels up: one of them must");
  return 0;
}
void rlink_args(const rlink_cfg *L, int *a, int *b) {
  *a = *b = 0;
  if (L->kind == RLINK_ELRS) { *a = L->rate_hz; *b = L->ratio; }
  else if (L->kind == RLINK_ESPNOW) { *a = L->channel; *b = L->lr; }
  else if (L->kind == RLINK_WIFI) { *a = L->sta; *b = L->channel; }
  else if (L->kind == RLINK_NRF24) *a = L->kbps;
  else if (L->kind == RLINK_SERIAL) { *a = L->baud; *b = L->half ? 1 : L->dir == RLINK_UP ? 2 : L->dir == RLINK_DOWN ? 3 : 0; }
}
static int put(char *o, int n, int k, const char *s) { while (*s) { if (k < n - 1) o[k] = *s; k++; s++; } if (n > 0) o[k < n ? k : n - 1] = 0; return k; }
static int put_int(char *o, int n, int k, int x) { char b[12]; int i = 11; b[i] = 0; int neg = x < 0; unsigned u = neg ? 0u - (unsigned)x : (unsigned)x; do { b[--i] = (char)('0' + u % 10); u /= 10; } while (u); if (neg) b[--i] = '-'; return put(o, n, k, b + i); }
int rlink_describe(const rlink_cfg *L, char *out, int n) {
  if (L->kind == RLINK_ESPNOW) { int k = put(out, n, 0, "espnow,"); k = put_int(out, n, k, L->channel); return L->lr ? put(out, n, k, ",lr") : k; }
  if (L->kind == RLINK_WIFI) { if (L->sta) return put(out, n, 0, "wifi,sta"); int k = put(out, n, 0, "wifi,ap,"); return put_int(out, n, k, L->channel); }
  if (L->kind == RLINK_BLE) return put(out, n, 0, "ble");
  if (L->kind == RLINK_NRF24) { int k = put(out, n, 0, "nrf24,"); return put_int(out, n, k, L->kbps); }
  if (L->kind == RLINK_SERIAL) { int k = put(out, n, 0, "serial,"); k = put_int(out, n, k, L->baud); return L->half ? put(out, n, k, ",half") : L->dir == RLINK_UP ? put(out, n, k, ",up") : L->dir == RLINK_DOWN ? put(out, n, k, ",down") : k; }
  if (L->kind != RLINK_ELRS) return put(out, n, 0, "?");
  int k = put(out, n, 0, "elrs,"); k = put_int(out, n, k, L->rate_hz); k = put(out, n, k, ","); return put_int(out, n, k, L->ratio);
}

/* A serial line's packets, from its speed: B = baud / 10 bytes a second each way (8 data bits, a start and a stop).
 * An uplink packet is a 16-byte header, the channel frame (26 bytes and its record's byte) and the 8-byte signature:
 * 51 bytes, 56 on the line with the framing (pframe.h) and a little to spare.
 *   - Both ways at once (a laser and a photodiode each way, two fibres, a wire pair): the channels take at most half
 *     the line up, 100 a second at most (19200 baud: 17); an uplink packet with commands waiting can be bigger, up
 *     to nine tenths of the line at that rate (the channels go first in it). Down, a packet as soon as there is something, at most 50 a
 *     second and fewer on a slow line (so the 30 bytes each costs stay under 15% of it); the telemetry gets what is
 *     left of three quarters of the line, and each packet has room for its share.
 *   - One way at a time (half): the ground sends, the drone answers at once, then the ground's next. A cycle holds an
 *     uplink packet, an answer of up to mtu bytes and the turnarounds (a quarter of it left as margin), 50 a second at
 *     most; the telemetry gets the answers' room. An uplink packet with commands can use some of that margin.
 * At least 15 channel packets a second either way: the drone counts the sticks centred after 0.1 s without channels
 * (rc_core.h), so slower would stutter on every loss. Hence the slowest speeds (RLINK_BAUD_MIN, RLINK_BAUD_HALF_MIN). */
typedef struct { int mtu, mtu_up; float up, dmin, dmax, room; } serial_plan;
#define UP_WIRE 56.0f
#define DOWN_COST 30.0f
static float clampf(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
static void serial_sizing(const rlink_cfg *L, serial_plan *P) {
  float B = (float)L->baud / 10;
  if (!L->half) {
    P->up = clampf(0.5f * B / UP_WIRE, 1, 100);
    P->dmax = clampf(0.15f * B / DOWN_COST, 1, 50); P->dmin = P->dmax < 20 ? P->dmax : 20;
    float room = 0.75f * B - P->dmax * DOWN_COST;
    P->mtu = (int)clampf(room / P->dmax + 40, 96, 250);
    P->room = clampf(room * 0.85f, 0, 6000);
    P->mtu_up = (int)clampf(0.9f * B / P->up - 3, 72, 250);           /* (commands waiting with the channels: still within the line) */
  } else {
    P->mtu = (int)clampf(B * 0.012f, 64, 200);
    P->up = clampf(0.75f * B / (UP_WIRE + P->mtu + 3), 1, 50); P->dmin = P->dmax = P->up;
    P->room = clampf(P->up * (P->mtu - DOWN_COST) * 0.85f, 0, 6000);
    P->mtu_up = (int)clampf(UP_WIRE + 0.2f * (UP_WIRE + P->mtu), 72, 250);   /* (a command or two with the channels: within the cycle's margin) */
  }
}
void rlink_sizing(const rlink_cfg *L, int *mtu_down, int *mtu_up, float *up_hz, float *down_min, float *down_max, int *half) {
  *mtu_down = *mtu_up = 250; *up_hz = 100; *down_min = 20; *down_max = 100; *half = 0;    /* ESP-NOW, Wi-Fi (plink_cfg_default) */
  if (L->kind != RLINK_SERIAL) return;
  serial_plan P; serial_sizing(L, &P);
  *mtu_down = P.mtu; *mtu_up = P.mtu_up; *up_hz = P.up; *down_min = P.dmin; *down_max = P.dmax; *half = L->half;
}

/* ExpressLRS: each telemetry packet carries 5 bytes of a frame (the stubborn sender's chunks); one packet in `ratio`
 * is telemetry. */
float rlink_budget(const rlink_cfg *L) {
  if (!rlink_down(L)) return 0;                                     /* (a link that goes up only carries none) */
  if (L->kind == RLINK_ELRS) return L->ratio > 0 ? (float)L->rate_hz / L->ratio * 5 * 0.9f : 0;
  /* the packet links: up to 100 packets a second down of up to 226 bytes of frames (plink.h), so ~22 KB/s; the
   * telemetry gets a share that leaves room for messages and repeats on a poor link (ESP-NOW's long-range mode, at
   * 0.5 Mbit/s, still carries it) */
  if (L->kind == RLINK_ESPNOW || L->kind == RLINK_WIFI || L->kind == RLINK_BLE) return 6000;
  if (L->kind == RLINK_SERIAL) { serial_plan P; serial_sizing(L, &P); return P.room; }
  /* nRF24L01 (clink.h): an answer to each packet up, up to 21 bytes of the stream each, sent again when lost: the
   * telemetry gets 60% of it (1260 B/s at 100 a second, 630 at 50) */
  if (L->kind == RLINK_NRF24) return rlink_compact_hz(L) * 21 * 0.6f;
  return 0;
}
/* As the link is now: scaled by the telemetry link quality the receiver reports (a lost chunk is sent again, so half
 * the packets through is half the room), and nothing while it reports nothing for a second. A receiver has no flow
 * control: written into a link that can't carry it, telemetry only fills its queue, and it drops frames, the newest
 * messages among them. Held back here, messages wait in the store and go first when the link is back. */
float rlink_budget_now(const rlink_cfg *L, const rc_input *in, double t) {
  if (!(in->t_link > 0 && t - in->t_link < 1.0)) return 0;
  float lq = in->down_lq > 0 || in->down_rssi < 0 ? in->down_lq : in->up_lq;   /* (some receivers leave the downlink figures 0: go by the uplink) */
  float q = lq / 100; q = q < 0.05f ? 0.05f : q > 1 ? 1 : q;
  return rlink_budget(L) * q;
}
