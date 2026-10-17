/* What a captured motor line carries; see sniff_core.h. */
#include "sniff_core.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int cmp_u32(const void *a, const void *b) { uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b; return x < y ? -1 : x > y; }
static uint32_t median(uint32_t *v, int n) { qsort(v, (size_t)n, sizeof *v, cmp_u32); return v[n / 2]; }

/* The line's edges: e[k] is the tick (from t0) of its k-th change; it starts at level L0 and toggles at each. Segment k
 * (1 <= k <= E-1) runs from e[k-1] to e[k] at level L0 ^ (k & 1); the first and last stretches are cut by the window. */
typedef struct { const uint32_t *e; int E, L0; uint32_t end; float tpu; } line_t;

static int level_at(const line_t *L, uint32_t t) {
  int lo = 0, hi = L->E;                         /* edges at or before t */
  while (lo < hi) { int m = (lo + hi) / 2; if (L->e[m] <= t) lo = m + 1; else hi = m; }
  return L->L0 ^ (lo & 1);
}
/* The first segment index whose level is A; pulses of level A are k, k+2, ... while k <= E-1. */
static int first_k(const line_t *L, int A) { return (L->L0 ^ A) ? 1 : 2; }

static const int DSHOT_RATES[] = { 150, 300, 600, 1200 };

/* DShot: frames of 16 pulses of level A, bit period T; a 1 is high for 75% of T, a 0 for 37.5%. Bidirectional DShot is
 * the same inverted (idle high, active low) with the CRC inverted. Returns 1 when it decodes. */
static int try_dshot(const line_t *L, int A, uint32_t *tmp, sniff_res *r) {
  int k1 = first_k(L, A), np = 0;
  for (int k = k1; k + 2 <= L->E - 1; k += 2) tmp[np++] = L->e[k + 1] - L->e[k - 1];
  if (np < 16) return 0;
  uint32_t pmin = 0xffffffffu;
  for (int i = 0; i < np; i++) if (tmp[i] < pmin) pmin = tmp[i];
  int nf = 0;
  for (int i = 0; i < np; i++) if (tmp[i] * 2 <= pmin * 3) tmp[nf++] = tmp[i];
  uint32_t T = median(tmp, nf);
  float Tus = T / L->tpu;
  if (Tus < 0.5f || Tus > 8.f) return 0;
  int frames = 0, good = 0, run = 0, thr_min = 2047, thr_max = 0, last_thr = 0, telem = 0;
  uint32_t v = 0, prev_s = 0, run_s = 0, first_good_s = 0, last_good_s = 0;
  for (int k = k1; ; k += 2) {
    int have = k <= L->E - 1;
    uint32_t s = have ? L->e[k - 1] : 0, w = have ? L->e[k] - L->e[k - 1] : 0;
    int breaks = !have || (run && (s - prev_s) * 2 > T * 3) || w >= T;
    if (breaks && run) {
      if (run == 16) {
        frames++;
        uint32_t v12 = v >> 4, crc = (v12 ^ (v12 >> 4) ^ (v12 >> 8)) & 0xF;
        if (A == 0) crc = ~crc & 0xF;
        if (crc == (v & 0xF)) {
          int thr = (int)(v12 >> 1);
          if (!good) first_good_s = run_s;
          last_good_s = run_s; good++;
          last_thr = thr; telem |= (int)(v12 & 1);
          if (thr < thr_min) thr_min = thr;
          if (thr > thr_max) thr_max = thr;
        }
      }
      run = 0; v = 0;
    }
    if (!have) break;
    if (w >= T) continue;
    if (!run) run_s = s;
    v = (v << 1) | (w * 2 > T); run++; prev_s = s;
  }
  if (good < 2 || good * 2 < frames) return 0;
  r->kind = SN_DSHOT; r->dshot_inverted = A == 0;
  int best = DSHOT_RATES[0]; float bd = 1e9f;
  for (int i = 0; i < 4; i++) { float d = Tus * DSHOT_RATES[i] / 1000.f - 1.f; if (d < 0) d = -d; if (d < bd) { bd = d; best = DSHOT_RATES[i]; } }
  r->dshot_rate = best; r->frames = frames; r->bad_crc = frames - good;
  r->throttle = last_thr; r->thr_min = thr_min; r->thr_max = thr_max; r->telem_req = telem;
  r->frame_hz = last_good_s > first_good_s ? (good - 1) * 1e6f * L->tpu / (float)(last_good_s - first_good_s) : 0;
  return 1;
}

/* UART 8N1, idle high. The shortest stretch may contain multiple bits
 * (e.g. repeated 00 bytes with several stop/idle bits between them). */
static const int BAUDS[] = { 1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 100000, 115200, 230400, 250000, 400000, 460800, 500000, 921600, 1000000, 2000000 };
static int serial_at(const line_t *L, float baud, sniff_res *r) {
  float bit = 1e6f * L->tpu / baud;
  int nb = 0, bad = 0, j = 0;
  while (j < L->E) {
    if ((L->L0 ^ ((j + 1) & 1)) != 0) { j++; continue; }   /* a falling edge: a start bit */
    uint32_t f = L->e[j];
    if (f + (uint32_t)(10.f * bit) > L->end) break;
    if (level_at(L, f + (uint32_t)(0.5f * bit)) != 0) { j++; continue; }  /* a glitch, not a start bit */
    int byte = 0;
    for (int i = 0; i < 8; i++) byte |= level_at(L, f + (uint32_t)((1.5f + i) * bit)) << i;
    int framing_bad = level_at(L, f + (uint32_t)(9.5f * bit)) != 1;
    /* Sampling plausible bits is insufficient: transitions within a UART
     * frame must also lie near integer bit boundaries. */
    for (int p = j + 1; p < L->E && L->e[p] - f < 10.f * bit; p++) {
      float at = (L->e[p] - f) / bit;
      int boundary = (int)(at + 0.5f);
      if (at < boundary - 0.2f || at > boundary + 0.2f) framing_bad = 1;
    }
    if (framing_bad) bad++;
    if (nb < 16) r->bytes[nb] = (uint8_t)byte;
    nb++;
    uint32_t next = f + (uint32_t)(9.5f * bit);
    while (j < L->E && L->e[j] <= next) j++;
  }
  if (nb < 2 || bad * 4 > nb) return 0;
  r->kind = SN_SERIAL; r->baud = (int)(baud + 0.5f); r->nbytes = nb; r->bad_bytes = bad;
  return 1;
}

static int try_serial(const line_t *L, uint32_t *tmp, sniff_res *r) {
  if (L->E < 4) return 0;
  uint32_t smin = 0xffffffffu, longest = 0; int longest_lv = 0, ns = 0;
  for (int k = 1; k <= L->E - 1; k++) { uint32_t d = L->e[k] - L->e[k - 1]; if (d < smin) smin = d; }
  for (int k = 0; k <= L->E; k++) {
    uint32_t a = k ? L->e[k - 1] : 0, b = k < L->E ? L->e[k] : L->end;
    if (b - a > longest) { longest = b - a; longest_lv = L->L0 ^ (k & 1); }
  }
  if (longest_lv != 1) return 0;
  for (int k = 1; k <= L->E - 1; k++) { uint32_t d = L->e[k] - L->e[k - 1]; if (d * 2 <= smin * 3) tmp[ns++] = d; }
  float shortest = (float)median(tmp, ns), inferred = 1e6f * L->tpu / shortest;
  int found = 0;
  sniff_res best = *r;
  /* Try the inferred rate first, then standard rates for which the shortest
   * stretch is approximately 1..9 bit times. Keep the first clean decode:
   * higher harmonics can also parse a repeating waveform as extra bytes. */
  for (int i = -1; i < (int)(sizeof BAUDS / sizeof *BAUDS); i++) {
    float baud = i < 0 ? inferred : (float)BAUDS[i];
    if (baud < 1100.f || baud > 2.2e6f) continue;
    if (i < 0) {
      for (int j = 0; j < (int)(sizeof BAUDS / sizeof *BAUDS); j++)
        if (baud > BAUDS[j] * 0.94f && baud < BAUDS[j] * 1.06f) { baud = (float)BAUDS[j]; break; }
    } else {
      float bits = shortest * baud / (1e6f * L->tpu);
      int count = (int)(bits + 0.5f);
      if (count < 1 || count > 9 || bits < count - 0.15f || bits > count + 0.15f) continue;
    }
    sniff_res candidate = *r;
    if (!serial_at(L, baud, &candidate)) continue;
    if (!candidate.bad_bytes) { *r = candidate; return 1; }
    int cmp = found ? candidate.bad_bytes * best.nbytes - best.bad_bytes * candidate.nbytes : -1;
    if (!found || cmp < 0 || (cmp == 0 && candidate.nbytes > best.nbytes)) { best = candidate; found = 1; }
  }
  if (found) *r = best;
  return found;
}

void sniff_analyze(const sniff_cap *c, int ch, uint32_t *a, uint32_t *b, sniff_res *r) {
  memset(r, 0, sizeof *r);
  line_t L = { a, 0, (c->lv0 >> ch) & 1, c->t_end - c->t0, c->tick_per_us };
  int prev = L.L0;
  for (int i = 0; i < c->n; i++) {
    int x = (c->lv[i] >> ch) & 1;
    if (x != prev) { a[L.E++] = c->t[i] - c->t0; prev = x; }
  }
  float tpu = c->tick_per_us;
  r->level = L.L0; r->edges = L.E; r->window_us = L.end / tpu;
  if (L.E == 0) { r->kind = SN_STEADY; return; }

  uint32_t smin = 0xffffffffu;
  for (int k = 1; k <= L.E - 1; k++) { uint32_t d = a[k] - a[k - 1]; if (d < smin) smin = d; }
  r->shortest_us = L.E > 1 ? smin / tpu : 0;

  /* high pulses: widths, and high-to-high periods */
  int k1 = first_k(&L, 1), np = 0, nper = 0;
  for (int k = k1; k <= L.E - 1; k += 2) b[np++] = a[k] - a[k - 1];
  r->pulses = np;
  if (np) {
    uint32_t mn = 0xffffffffu, mx = 0;
    for (int i = 0; i < np; i++) { if (b[i] < mn) mn = b[i]; if (b[i] > mx) mx = b[i]; }
    r->high_us = median(b, np) / tpu; r->high_min = mn / tpu; r->high_max = mx / tpu;
  }
  for (int k = k1; k + 1 <= L.E - 1; k += 2) b[nper++] = a[k + 1] - a[k - 1];
  int periodic = 0;
  if (nper >= 2) {
    uint32_t P = median(b, nper), mn = b[0], mx = b[nper - 1];
    r->period_us = P / tpu; r->hz = 1e6f / r->period_us; r->duty = r->high_us / r->period_us;
    if (mx * 100 <= P * 115 && mn * 100 >= P * 85) {   /* evenly spaced: a PWM of some kind */
      float H = r->high_us, Pu = r->period_us;
      if (Pu < 1000.f) r->kind = SN_DUTY;
      else if (H >= 700.f && H <= 2300.f) r->kind = SN_SERVO;
      else if (H >= 100.f && H <= 280.f) r->kind = SN_ONESHOT125;
      else if (H >= 30.f && H < 100.f) r->kind = SN_ONESHOT42;
      else if (H < 30.f) r->kind = SN_SHORT;
      else r->kind = SN_PERIODIC;
      periodic = 1;
    }
  }
  /* CRC-checked frames are stronger evidence than periodic timing. Repeating
   * UART bytes can look exactly like PWM, so try both before choosing a label. */
  if (try_dshot(&L, 1, b, r) || try_dshot(&L, 0, b, r)) return;
  sniff_res serial = *r;
  if (try_serial(&L, b, &serial)) {
    if (periodic) { serial.kind = r->kind; serial.pwm_uart_ambiguous = 1; }
    *r = serial;
    return;
  }
  if (periodic) return;
  r->kind = L.E < 6 ? SN_FEW : SN_IRREGULAR;
}

const char *sniff_kind_name(int kind) {
  static const char *N[] = { "steady", "few edges", "servo-style PWM", "OneShot125", "OneShot42", "short pulses", "duty PWM",
                             "periodic pulses", "DShot", "serial", "irregular" };
  return kind >= 0 && kind <= SN_IRREGULAR ? N[kind] : "?";
}

int sniff_line(const sniff_res *r, char *buf, int size) {
  switch (r->kind) {
  case SN_STEADY:
    return snprintf(buf, (size_t)size, "steady %s, no pulses%s", r->level ? "HIGH" : "low",
                    r->level ? " (100% duty? a supply? an enable?)" : " (idle, not armed, or not connected)");
  case SN_FEW:
    return snprintf(buf, (size_t)size, "%d edge%s in %.0f ms: too few to tell (longer window: win <ms>)", r->edges, r->edges == 1 ? "" : "s", r->window_us / 1000.f);
  case SN_SERVO: case SN_ONESHOT125: case SN_ONESHOT42: case SN_SHORT: case SN_DUTY: case SN_PERIODIC: {
    const char *what =
      r->kind == SN_SERVO ? "servo-style PWM ESC signal (LiftLab: PWM ESC)" :
      r->kind == SN_ONESHOT125 ? "OneShot125-like" :
      r->kind == SN_ONESHOT42 ? "OneShot42-like" :
      r->kind == SN_SHORT ? "short pulses: Multishot, or duty PWM" :
      r->kind == SN_DUTY ? (r->high_us >= 120.f && r->high_us <= 260.f ? "duty PWM (or OneShot125 if high stays 125-250 us)" : "duty PWM (LiftLab: brushed MOSFET signal)") :
      "periodic pulses";
    int n = snprintf(buf, (size_t)size, "%8.1f Hz  high %7.1f us (%.1f-%.1f)  duty %5.1f%%  -> %s",
                     r->hz, r->high_us, r->high_min, r->high_max, r->duty * 100.f, what);
    /* a steady pulse train can also be one UART byte sent over and over: say so, but PWM is the likelier reading.
     * Changing the throttle tells them apart: PWM keeps its period, UART keeps its edges on bit boundaries. */
    if (r->pwm_uart_ambiguous && n < size - 4) {
      n += snprintf(buf + n, (size_t)(size - n), "  [also fits UART ~%d baud, repeated", r->baud);
      for (int i = 0; i < r->nbytes && i < 2 && n < size - 4; i++) n += snprintf(buf + n, (size_t)(size - n), " %02x", r->bytes[i]);
      if (n < size - 2) n += snprintf(buf + n, (size_t)(size - n), "]");
    }
    return n;
  }
  case SN_DSHOT:
    return snprintf(buf, (size_t)size, "DShot%d%s  throttle %d (%d-%d of 2047, 0 = stop, 1-47 commands)  %d frames, %d bad CRC, %.0f frames/s%s",
                    r->dshot_rate, r->dshot_inverted ? " bidirectional (inverted)" : "", r->throttle, r->thr_min, r->thr_max,
                    r->frames, r->bad_crc, r->frame_hz, r->telem_req ? ", telemetry requested" : "");
  case SN_SERIAL: {
    int n = snprintf(buf, (size_t)size, "serial 8N1 ~%d baud, idle high, %d bytes (%d framing errors):", r->baud, r->nbytes, r->bad_bytes);
    for (int i = 0; i < r->nbytes && i < 16 && n < size - 4; i++) n += snprintf(buf + n, (size_t)(size - n), " %02x", r->bytes[i]);
    return n;
  }
  default:
    return snprintf(buf, (size_t)size, "%d edges, irregular: shortest stretch %.2f us, median high %.1f us (try: raw <ch>)",
                    r->edges, r->shortest_us, r->high_us);
  }
}
