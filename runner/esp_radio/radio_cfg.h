/* The packet links' own settings, as both ESP32 firmwares take them (the flight firmware's config.c, the command
 * module's ground.c): the binding phrase, the Wi-Fi network's name and password, the drone's address. Header only,
 * no ESP-IDF: config.c also builds on a PC for the wiring tests, and test_esp_radio.c tests these.
 *
 *   bind=PHRASE        1–31 characters (printable ASCII, spaces inside allowed). Both ends the same: it signs the
 *                      packets (plink_key). Default "liftlab": anyone who knows the default can fly a drone left on it.
 *   wifi=SSID,PASSWORD the Wi-Fi network: SSID 1–32 characters (no comma), password 8–63 characters or left out
 *                      (wifi=SSID: the default password below). wifi= alone: back to the defaults.
 *   drone=IP           (the command module, Wi-Fi) the drone's address: 192.168.4.1 by default, the address an ESP32
 *                      access point gives itself.
 * The default password (a drone's own access point, and what a command module tries on it): the binding phrase if it
 * has 8 characters or more (WPA2's minimum), else "liftlab1" (weak: everyone knows it). */
#ifndef RADIO_CFG_H
#define RADIO_CFG_H
#include <stdint.h>

#define RCFG_BIND_DEFAULT "liftlab"
#define RCFG_PASS_WEAK "liftlab1"
#define RCFG_DRONE_DEFAULT "192.168.4.1"
#define RCFG_BIND_N 32           /* 31 characters and the 0 */
#define RCFG_SSID_N 33
#define RCFG_PASS_N 64
#define RCFG_IP_N 16

static inline int rcfg_say_(char *err, int en, const char *s) { int k = 0; if (en > 0) { while (s[k] && k < en - 1) { err[k] = s[k]; k++; } err[k] = 0; } return -1; }
static inline int rcfg_len_(const char *s) { int n = 0; while (s[n]) n++; return n; }
static inline void rcfg_copy_(char *d, const char *s, int n) { for (int i = 0; i < n; i++) d[i] = s[i]; d[n] = 0; }
static inline int rcfg_same_(const char *a, const char *b) { while (*a && *a == *b) { a++; b++; } return *a == *b; }

/* bind=: into out (RCFG_BIND_N). 0, or −1 with why (out unchanged). */
static inline int rcfg_bind_parse(char *out, const char *v, char *err, int en) {
  int n = rcfg_len_(v);
  if (n < 1 || n > RCFG_BIND_N - 1) return rcfg_say_(err, en, "bind=PHRASE: 1 to 31 characters, the same at both ends");
  for (int i = 0; i < n; i++) if (v[i] < 0x20 || v[i] > 0x7e) return rcfg_say_(err, en, "bind: plain characters only (ASCII letters, digits, punctuation, spaces)");
  if (v[0] == ' ' || v[n - 1] == ' ') return rcfg_say_(err, en, "bind: no space at the start or the end");
  rcfg_copy_(out, v, n); return 0;
}
static inline int rcfg_bind_default(const char *bind) { return !bind[0] || rcfg_same_(bind, RCFG_BIND_DEFAULT); }
/* The phrase in use (an empty one, from a blob that has none: the default). */
static inline const char *rcfg_bind(const char *bind) { return bind[0] ? bind : RCFG_BIND_DEFAULT; }

/* wifi=SSID[,PASSWORD], or wifi= (both back to their defaults): into ssid (RCFG_SSID_N) and pass (RCFG_PASS_N). */
static inline int rcfg_wifi_parse(char *ssid, char *pass, const char *v, char *err, int en) {
  int n = rcfg_len_(v), c = 0; while (v[c] && v[c] != ',') c++;
  if (!n) { ssid[0] = pass[0] = 0; return 0; }
  int pn = v[c] ? n - c - 1 : 0;
  if (c < 1 || c > RCFG_SSID_N - 1) return rcfg_say_(err, en, "wifi=SSID,PASSWORD: a network name of 1 to 32 characters (no comma), then its password");
  if (v[c] && (pn < 8 || pn > RCFG_PASS_N - 1)) return rcfg_say_(err, en, "wifi: the password has 8 to 63 characters (WPA2); leave it out (wifi=SSID) for the default");
  for (int i = 0; i < n; i++) if (v[i] < 0x20 || v[i] > 0x7e) return rcfg_say_(err, en, "wifi: plain characters only");
  rcfg_copy_(ssid, v, c);
  if (v[c]) rcfg_copy_(pass, v + c + 1, pn); else pass[0] = 0;
  return 0;
}
/* The password in use: the one set, else the binding phrase if it's long enough, else the weak default (*weak = 1). */
static inline const char *rcfg_pass(const char *pass, const char *bind, int *weak) {
  bind = rcfg_bind(bind); int w = 0; const char *p = pass;
  if (!pass[0]) { if (rcfg_len_(bind) >= 8) p = bind; else { p = RCFG_PASS_WEAK; w = 1; } }
  if (weak) *weak = w;
  return p;
}
/* A drone's own network's default name: "LiftLab-" and the last two bytes of its MAC address, in hex (out: RCFG_SSID_N). */
static inline void rcfg_default_ssid(char *out, const uint8_t mac[6]) {
  static const char hx[] = "0123456789ABCDEF"; const char *pre = "LiftLab-"; int k = 0;
  while (pre[k]) { out[k] = pre[k]; k++; }
  for (int i = 4; i < 6; i++) { out[k++] = hx[mac[i] >> 4]; out[k++] = hx[mac[i] & 15]; }
  out[k] = 0;
}

/* drone=A.B.C.D: into out (RCFG_IP_N) and, if ip, as the four numbers (ip[0] = A). 0 or −1. */
static inline int rcfg_ip_parse(char *out, uint8_t *ip, const char *v, char *err, int en) {
  uint8_t q[4]; int k = 0;
  for (int i = 0; i < 4; i++) {
    int x = 0, d = 0; while (v[k] >= '0' && v[k] <= '9' && d < 4) { x = x * 10 + (v[k++] - '0'); d++; }
    if (!d || x > 255 || (i < 3 && v[k++] != '.')) return rcfg_say_(err, en, "drone=IP: the drone's address, four numbers 0 to 255 (192.168.4.1)");
    q[i] = (uint8_t)x;
  }
  if (v[k] || k > RCFG_IP_N - 1) return rcfg_say_(err, en, "drone=IP: the drone's address, four numbers 0 to 255 (192.168.4.1)");
  if (out) rcfg_copy_(out, v, k);
  if (ip) for (int i = 0; i < 4; i++) ip[i] = q[i];
  return 0;
}

/* A stored string field is whole (ends within its room): settings from flash are checked with it. */
static inline int rcfg_terminated(const char *s, int n) { for (int i = 0; i < n; i++) if (!s[i]) return 1; return 0; }
#endif
