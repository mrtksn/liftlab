/* The pilot's radio link: see radio_link.h. (No C library: it also builds for the simulator.) */
#include "radio_link.h"

const char *const rlink_names[RLINK_KINDS] = { "elrs" };
const char *const rlink_labels[RLINK_KINDS] = { "ExpressLRS 2.4 GHz" };

void rlink_default(rlink_cfg *L) { L->kind = RLINK_ELRS; L->rate_hz = 250; L->ratio = 4; }

static int say(char *err, int en, const char *s) { int k = 0; if (en > 0) { while (s[k] && k < en - 1) { err[k] = s[k]; k++; } err[k] = 0; } return -1; }
static int same(const char *a, int n, const char *b) { int k = 0; while (k < n && b[k] && a[k] == b[k]) k++; return k == n && !b[k]; }
/* whole numbers separated by commas, all of s: how many (up to max), or −1 if anything else is there */
static int ints(const char *s, int *v, int max) {
  int n = 0;
  while (*s) {
    int neg = *s == '-'; if (neg) s++;
    if (*s < '0' || *s > '9' || n >= max) return -1;
    long x = 0; while (*s >= '0' && *s <= '9') { x = x * 10 + (*s - '0'); if (x > 100000) return -1; s++; }
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
  else return say(err, en, "no such link (this build has: elrs)");
  int x[2] = { 0, 0 };
  if (kind == RLINK_ELRS && (ints(v, x, 2) != 2 || rlink_make(L, kind, x[0], x[1])))
    return say(err, en, "elrs,rate,ratio as set on the radio: 50, 150, 250 or 500 Hz; telemetry 1:2 to 1:128");
  return 0;
}
int rlink_make(rlink_cfg *L, int kind, int a, int b) {
  if (kind == RLINK_ELRS && elrs_ok(a, b)) { L->kind = kind; L->rate_hz = a; L->ratio = b; return 0; }
  return -1;
}
static int put(char *o, int n, int k, const char *s) { while (*s) { if (k < n - 1) o[k] = *s; k++; s++; } if (n > 0) o[k < n ? k : n - 1] = 0; return k; }
static int put_int(char *o, int n, int k, int x) { char b[12]; int i = 11; b[i] = 0; int neg = x < 0; unsigned u = neg ? 0u - (unsigned)x : (unsigned)x; do { b[--i] = (char)('0' + u % 10); u /= 10; } while (u); if (neg) b[--i] = '-'; return put(o, n, k, b + i); }
int rlink_describe(const rlink_cfg *L, char *out, int n) {
  if (L->kind != RLINK_ELRS) return put(out, n, 0, "?");
  int k = put(out, n, 0, "elrs,"); k = put_int(out, n, k, L->rate_hz); k = put(out, n, k, ","); return put_int(out, n, k, L->ratio);
}

/* ExpressLRS: each telemetry packet carries 5 bytes of a frame (the stubborn sender's chunks); one packet in `ratio`
 * is telemetry. */
float rlink_budget(const rlink_cfg *L) {
  if (L->kind == RLINK_ELRS) return L->ratio > 0 ? (float)L->rate_hz / L->ratio * 5 * 0.9f : 0;
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
