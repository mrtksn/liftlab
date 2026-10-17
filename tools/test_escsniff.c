/* Host tests for the ESC signal sniffer's analysis (runner/escsniff/main/sniff_core.c): synthetic captures of each
 * signal kind, sampled as the chip would (changes stamped in 240 MHz cycles, with a little sampling jitter).
 *   cc -Wall -Wextra -Werror -fsanitize=undefined -Irunner/escsniff/main tools/test_escsniff.c runner/escsniff/main/sniff_core.c -lm -o /tmp/test-escsniff */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "sniff_core.h"
#include "sniff_console.h"

#define TPU 240.f
#define MAXE 40000
static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* one channel's edges: times in us, the level after each, its starting level */
typedef struct { double t[MAXE]; int n, lv0; } wave;
static wave W[SNIFF_CH];
static uint32_t T[MAXE * 2], A[MAXE * 2 + 2], B[MAXE * 2 + 2];
static uint8_t LV[MAXE * 2];
static double WIN;

static void clear(double win_us) { memset(W, 0, sizeof W); WIN = win_us; }
static void edge(int ch, double t) { if (t < WIN && W[ch].n < MAXE) W[ch].t[W[ch].n++] = t; }
static void pulse(int ch, double at, double width) { edge(ch, at); edge(ch, at + width); }
/* the chip samples about every 20 cycles: an edge lands on the next sample */
static uint32_t stamp(double t_us) { uint32_t c = (uint32_t)(t_us * TPU); return c + (uint32_t)(rand() % 20); }

static sniff_cap merge(uint32_t t0) {
  int idx[SNIFF_CH] = { 0 }, n = 0; uint8_t lv = 0;
  for (int c = 0; c < SNIFF_CH; c++) lv |= (uint8_t)(W[c].lv0 << c);
  sniff_cap cap = { T, LV, 0, lv, t0, t0 + (uint32_t)(WIN * TPU), TPU };
  for (;;) {
    int best = -1;
    for (int c = 0; c < SNIFF_CH; c++) if (idx[c] < W[c].n && (best < 0 || W[c].t[idx[c]] < W[best].t[idx[best]])) best = c;
    if (best < 0) break;
    lv ^= (uint8_t)(1 << best);
    T[n] = t0 + stamp(W[best].t[idx[best]]); LV[n] = lv; n++; idx[best]++;
  }
  cap.n = n;
  return cap;
}

static sniff_res analyze(const sniff_cap *c, int ch) {
  sniff_res r; char line[300];
  sniff_analyze(c, ch, A, B, &r);
  sniff_line(&r, line, sizeof line);
  printf("  CH%d %s\n", ch + 1, line);
  return r;
}

static void dshot_frame(int ch, double at, double T_us, int thr, int telem, int inverted) {
  unsigned v12 = (unsigned)(thr << 1 | telem), crc = (v12 ^ (v12 >> 4) ^ (v12 >> 8)) & 0xF;
  if (inverted) crc = ~crc & 0xF;
  unsigned v = v12 << 4 | crc;
  for (int i = 0; i < 16; i++) pulse(ch, at + i * T_us, T_us * ((v >> (15 - i)) & 1 ? 0.75 : 0.375));
}

static void uart_bytes(int ch, double at, int baud, const uint8_t *b, int n) {
  double bit = 1e6 / baud; int lvl = 1;          /* idle high: the channel starts high */
  for (int k = 0; k < n; k++) {
    int bits[10] = { 0 }; for (int i = 0; i < 8; i++) bits[1 + i] = (b[k] >> i) & 1; bits[9] = 1;
    for (int i = 0; i < 10; i++) { if (bits[i] != lvl) { edge(ch, at + i * bit); lvl = bits[i]; } }
    at += 12 * bit;                               /* a little idle between bytes */
  }
}

static void console_case(const char *input, const char *const *expected, int count) {
  sniff_console console = { 0 }; char out[SNIFF_LINE_SIZE]; int got = 0;
  for (const char *p = input; *p; p++) {
    if (!sniff_console_feed(&console, (unsigned char)*p, out)) continue;
    CHECK(got < count && !strcmp(out, expected[got]), "console line %d: unexpected [%s]", got, out);
    got++;
  }
  CHECK(got == count, "console expected %d lines, got %d", count, got);
}

int main(void) {
  srand(1);
  uint32_t t0 = 0xfffff000u;                     /* the cycle counter wraps inside the window */

  printf("servo PWM 50 Hz 1500 us, 490 Hz 1100 us, steady low, steady high\n");
  clear(60000);
  for (double t = 300; t < WIN; t += 20000) pulse(0, t, 1500);
  for (double t = 100; t < WIN; t += 1e6 / 490) pulse(1, t, 1100);
  W[3].lv0 = 1;
  sniff_cap c = merge(t0);
  sniff_res r = analyze(&c, 0);
  CHECK(r.kind == SN_SERVO && fabsf(r.hz - 50) < 0.5f && fabsf(r.high_us - 1500) < 1, "50 Hz servo: kind %d hz %.2f high %.2f", r.kind, r.hz, r.high_us);
  r = analyze(&c, 1);
  CHECK(r.kind == SN_SERVO && fabsf(r.hz - 490) < 3 && fabsf(r.high_us - 1100) < 1, "490 Hz servo: kind %d hz %.2f", r.kind, r.hz);
  r = analyze(&c, 2); CHECK(r.kind == SN_STEADY && r.level == 0, "steady low: kind %d", r.kind);
  r = analyze(&c, 3); CHECK(r.kind == SN_STEADY && r.level == 1, "steady high: kind %d", r.kind);

  printf("duty PWM 8 kHz 25%%, 32 kHz 60%%, OneShot125 at 2 kHz loop... (shows as duty PWM with a note), OneShot42 at 1 kHz\n");
  clear(20000);
  for (double t = 3; t < WIN; t += 125) pulse(0, t, 31.25);
  W[1].lv0 = 1; for (double t = 0; t < WIN; t += 31.25) { edge(1, t + 18.75); edge(1, t + 31.25); }
  for (double t = 7; t < WIN; t += 500) pulse(2, t, 180);
  for (double t = 7; t < WIN; t += 1000) pulse(3, t, 60);
  c = merge(t0);
  r = analyze(&c, 0); CHECK(r.kind == SN_DUTY && fabsf(r.hz - 8000) < 40 && fabsf(r.duty - 0.25f) < 0.01f, "8 kHz duty: kind %d hz %.1f duty %.3f", r.kind, r.hz, r.duty);
  r = analyze(&c, 1); CHECK(r.kind == SN_DUTY && fabsf(r.hz - 32000) < 300 && fabsf(r.duty - 0.6f) < 0.02f, "32 kHz duty: kind %d hz %.1f duty %.3f", r.kind, r.hz, r.duty);
  /* This waveform also decodes as a repeated UART byte: the reading stays duty PWM, with the LiftLab guidance, and
   * the UART fit is a note after it. */
  CHECK(r.pwm_uart_ambiguous, "32 kHz duty also admits a UART interpretation");
  { char line[300]; sniff_line(&r, line, sizeof line);
    CHECK(strstr(line, "-> duty PWM (LiftLab: brushed MOSFET signal)") && strstr(line, "[also fits UART"), "duty PWM stays the main reading: %s", line); }
  r = analyze(&c, 2); CHECK(r.kind == SN_DUTY && fabsf(r.high_us - 180) < 1, "2 kHz 180 us: kind %d", r.kind);
  r = analyze(&c, 3); CHECK(r.kind == SN_ONESHOT42 && fabsf(r.high_us - 60) < 1, "OneShot42: kind %d", r.kind);

  printf("DShot600, DShot300 (throttle ramp), bidirectional DShot600, a stray glitch line\n");
  clear(30000);
  for (int i = 0; i * 250.0 < WIN - 50; i++) dshot_frame(0, 10 + i * 250.0, 1e3 / 600, 1046, 0, 0);
  for (int i = 0; i * 500.0 < WIN - 100; i++) dshot_frame(1, 20 + i * 500.0, 1e3 / 300, 48 + i, i == 3, 0);
  W[2].lv0 = 1;                                   /* inverted: idle high, the bits are low pulses */
  for (int i = 0; i * 250.0 < WIN - 50; i++) {
    unsigned v12 = 300u << 1, crc = ~(v12 ^ (v12 >> 4) ^ (v12 >> 8)) & 0xF, v = v12 << 4 | crc; double T_us = 1e3 / 600, at = 30 + i * 250.0;
    for (int b = 0; b < 16; b++) pulse(2, at + b * T_us, T_us * ((v >> (15 - b)) & 1 ? 0.75 : 0.375));
  }
  pulse(3, 5000, 3); pulse(3, 17000, 40);
  c = merge(t0);
  r = analyze(&c, 0); CHECK(r.kind == SN_DSHOT && r.dshot_rate == 600 && r.throttle == 1046 && r.bad_crc == 0 && !r.dshot_inverted && fabsf(r.frame_hz - 4000) < 50,
                            "DShot600: kind %d rate %d thr %d bad %d fhz %.0f", r.kind, r.dshot_rate, r.throttle, r.bad_crc, r.frame_hz);
  r = analyze(&c, 1); CHECK(r.kind == SN_DSHOT && r.dshot_rate == 300 && r.thr_min == 48 && r.thr_max > 100 && r.telem_req, "DShot300: kind %d rate %d %d-%d", r.kind, r.dshot_rate, r.thr_min, r.thr_max);
  r = analyze(&c, 2); CHECK(r.kind == SN_DSHOT && r.dshot_inverted && r.throttle == 300, "bidir DShot: kind %d inv %d thr %d", r.kind, r.dshot_inverted, r.throttle);
  r = analyze(&c, 3); CHECK(r.kind == SN_FEW, "glitches: kind %d", r.kind);

  printf("UART 115200 and 19200 (idle high), DShot with a corrupted CRC on every frame\n");
  clear(20000);
  const uint8_t msg[] = { 0x55, 0xa5, 0x00, 0xff, 0x12, 0x80 };
  W[0].lv0 = 1; uart_bytes(0, 200, 115200, msg, 6);
  W[1].lv0 = 1; uart_bytes(1, 300, 19200, msg, 3);
  for (int i = 0; i < 20; i++) {                  /* throttle 500 with the CRC of 501 */
    unsigned v12 = 501u << 1, crc = (v12 ^ (v12 >> 4) ^ (v12 >> 8)) & 0xF, v = (500u << 1) << 4 | crc; double T_us = 1e3 / 600;
    for (int b = 0; b < 16; b++) pulse(2, 10 + i * 500.0 + b * T_us, T_us * ((v >> (15 - b)) & 1 ? 0.75 : 0.375));
  }
  c = merge(t0);
  r = analyze(&c, 0);
  CHECK(r.kind == SN_SERIAL && r.baud == 115200 && r.nbytes == 6 && r.bad_bytes == 0 && !memcmp(r.bytes, msg, 6), "UART 115200: kind %d baud %d n %d bad %d", r.kind, r.baud, r.nbytes, r.bad_bytes);
  r = analyze(&c, 1);
  CHECK(r.kind == SN_SERIAL && r.baud == 19200 && r.nbytes == 3 && !memcmp(r.bytes, msg, 3), "UART 19200: kind %d baud %d n %d", r.kind, r.baud, r.nbytes);
  r = analyze(&c, 2); CHECK(r.kind != SN_DSHOT, "bad-CRC DShot must not decode: kind %d", r.kind);

  printf("a repeated UART byte reads as a pulse train, with the UART fit noted\n");
  const uint8_t repeated[] = { 0x00, 0xff, 0x33 };
  for (unsigned i = 0; i < sizeof repeated; i++) {
    uint8_t bytes[20]; memset(bytes, repeated[i], sizeof bytes);
    clear(10000); W[0].lv0 = 1;
    uart_bytes(0, 200, 115200, bytes, sizeof bytes);
    c = merge(t0); r = analyze(&c, 0);
    CHECK(r.pwm_uart_ambiguous && r.baud == 115200 && r.nbytes == 20 && !r.bad_bytes && !memcmp(r.bytes, bytes, 16),
          "repeated %02x: ambiguity %d baud %d n %d bad %d", repeated[i], r.pwm_uart_ambiguous, r.baud, r.nbytes, r.bad_bytes);
    char line[300]; sniff_line(&r, line, sizeof line);
    CHECK(strstr(line, "Hz  high") && strstr(line, "[also fits UART ~115200 baud"), "PWM reading with a UART note: %s", line);
  }
  uint8_t sync[20]; memset(sync, 0x55, sizeof sync);
  clear(10000); W[0].lv0 = 1;
  uart_bytes(0, 200, 115200, sync, sizeof sync);
  c = merge(t0); r = analyze(&c, 0);
  CHECK(r.kind == SN_SERIAL && r.baud == 115200 && r.nbytes == 20 && !memcmp(r.bytes, sync, 16),
        "UART sync must not pick a faster harmonic: baud %d n %d", r.baud, r.nbytes);

  printf("LF, CR and CRLF Enter events, consecutive Enter, and independent USB readers\n");
  const char *one_enter[] = { "" }, *two_enters[] = { "", "" }, *commands[] = { "w", "", "s" };
  console_case("\n", one_enter, 1);
  console_case("\r", one_enter, 1);
  console_case("\r\n", one_enter, 1);
  console_case("\n\n", two_enters, 2);
  console_case("\r\r", two_enters, 2);
  console_case("w\r\n\r\ns\r\n", commands, 3);
  console_case("w\n\ns\n", commands, 3);
  sniff_console uart = { 0 }, usb = { 0 }; char line[SNIFF_LINE_SIZE];
  CHECK(!sniff_console_feed(&uart, 's', line), "partial UART command");
  CHECK(sniff_console_feed(&usb, '\n', line) && !*line, "native USB blank Enter");
  CHECK(sniff_console_feed(&uart, '\r', line) && !strcmp(line, "s"), "UART command retained");
  CHECK(!sniff_console_feed(&uart, '\n', line), "split CRLF must not emit a second capture");

  if (fails) { printf("%d FAILED\n", fails); return 1; }
  printf("escsniff analysis: all passed\n");
  return 0;
}
