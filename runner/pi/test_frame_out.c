/* Checks dfb_flow's pictures (frame_out.c): raw Bayer frames (10-bit packed and 8-bit, each colour order) to RGB, and
 * a reader on the port gets the header and the pixels; no reader, nothing sent. */
#include "frame_out.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

/* a W×H raw frame whose 2×2 blocks hold red R, greens G1 G2 and blue B in the given order (values up to 1023) */
static void make(uint8_t *raw, int bpl, int W, int H, int raw10, int order, int R, int G1, int G2, int B) {
  static const int RED[4] = { 3, 2, 1, 0 }, BLUE[4] = { 0, 1, 2, 3 };
  for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) {
    int k = (y & 1) * 2 + (x & 1), v = k == RED[order] ? R : k == BLUE[order] ? B : k < 2 ? G1 : G2;   /* (the green in the top row G1) */
    v += x / 2 % 3;                                    /* (a little variation across the picture) */
    uint8_t *row = raw + (size_t)y * (size_t)bpl;
    if (raw10) { row[x / 4 * 5 + x % 4] = (uint8_t)(v >> 2); row[x / 4 * 5 + 4] = (uint8_t)((row[x / 4 * 5 + 4] & ~(3 << (2 * (x % 4)))) | ((v & 3) << (2 * (x % 4)))); }
    else row[x] = (uint8_t)(v >> 2);
  }
}

int main(void) {
  static uint8_t raw[480 * 800], rgb[320 * 240 * 3];
  const int W = 64, H = 48;
  for (int raw10 = 0; raw10 < 2; raw10++) for (int order = 0; order < 4; order++) {
    int bpl = raw10 ? W / 4 * 5 : W; memset(raw, 0, sizeof raw);
    make(raw, bpl, W, H, raw10, order, 800, 400, 440, 120);
    frame_out_rgb(rgb, raw, bpl, W, H, raw10, order);
    int ok = 1;
    for (int y = 0; y < H / 2 && ok; y++) for (int x = 0; x < W / 2 && ok; x++) {
      const uint8_t *p = rgb + ((size_t)y * (W / 2) + (size_t)x) * 3; int d = x % 3;
      if (p[0] != (800 + d) >> 2 || p[1] != (((400 + d) >> 2) + ((440 + d) >> 2)) / 2 || p[2] != (120 + d) >> 2) { ok = 0;
        printf("  at %d,%d: %d %d %d\n", x, y, p[0], p[1], p[2]); }
    }
    CHECK(ok, "raw%d, colour order %d: each 2x2 block one RGB pixel", raw10 ? 10 : 8, order);
  }
  { /* balanced: a reddish dark picture comes out grey and brighter, the same on a second frame (eased toward it) */
    frame_out B; memset(&B, 0, sizeof B); static uint8_t pic[32 * 24 * 3];
    for (int f = 0; f < 12; f++) { for (int i = 0; i < 32 * 24; i++) { pic[3 * i] = 60; pic[3 * i + 1] = 30; pic[3 * i + 2] = 20; } frame_out_balance(&B, pic, 32, 24); }
    CHECK(abs(pic[0] - pic[1]) < 4 && abs(pic[1] - pic[2]) < 4 && pic[1] > 90 && pic[1] < 130, "a dark reddish picture: grey, about middle grey (%d %d %d)", pic[0], pic[1], pic[2]); }
  /* the port: nothing sent without a reader; a reader gets the frame */
  frame_out O; int port = 25000 + (int)(getpid() % 20000);
  CHECK(frame_out_open(&O, port) == 0, "listens on 127.0.0.1:%d", port);
  CHECK(!frame_out_wanted(&O) && frame_out_send(&O, rgb, W / 2, H / 2, 1, 2, 3, 4) == -1, "no reader: not wanted, nothing sent");
  int c = socket(AF_INET, SOCK_STREAM, 0); struct sockaddr_in a; memset(&a, 0, sizeof a); a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port); a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  CHECK(connect(c, (struct sockaddr *)&a, sizeof a) == 0, "a reader connects");
  usleep(20000);
  CHECK(frame_out_wanted(&O), "and is taken");
  CHECK(frame_out_send(&O, rgb, W / 2, H / 2, 123456789012345ull, 77, 8000, 400) == 0, "a frame sent");
  uint8_t got[36 + 32 * 24 * 3]; size_t n = 0; while (n < sizeof got) { ssize_t k = read(c, got + n, sizeof got - n); if (k <= 0) break; n += (size_t)k; }
  uint32_t v[8]; memcpy(v, got + 4, sizeof v);
  CHECK(n == sizeof got && !memcmp(got, "DFF1", 4) && v[0] == 32 && v[1] == 24 && ((uint64_t)v[3] << 32 | v[2]) == 123456789012345ull && v[4] == 77 && v[5] == 8000 && v[6] == 400 && v[7] == 32 * 24 * 3
        && !memcmp(got + 36, rgb, 32 * 24 * 3), "the header and the pixels arrive (%zu bytes)", n);
  close(c); usleep(20000);
  int gone = 0; for (int k = 0; k < 50 && !gone; k++) gone = frame_out_send(&O, rgb, W / 2, H / 2, 1, 2, 3, 4) == -1;
  CHECK(gone && O.fd < 0, "the reader gone: let go");
  frame_out_close(&O);
  if (fails) { printf("frame_out: %d checks failed\n", fails); return 1; }
  printf("frame_out: all checks passed\n");
  return 0;
}
