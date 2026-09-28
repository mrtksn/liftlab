/* Native test of the link framing: frames built here and by pi/send_program.py are parsed; corrupted and
 * oversized frames are dropped; noise between frames is skipped.
 *   cc -O2 -Wall -Wextra -o test_link rn.c rn_link.c test_link.c -lm && ./test_link [frame-from-python.bin] */
#include "rn_link.h"
#include <stdio.h>
#include <string.h>
static int fails;
#define CHECK(c, m) do { printf("  %s %s\n", (c) ? "ok  " : "FAIL", m); if (!(c)) fails++; } while (0)
static int feed_all(rn_link *L, const uint8_t *p, uint32_t n, int *types) { int k = 0; for (uint32_t i = 0; i < n; i++) { int r = rn_link_feed(L, p[i]); if (r) types[k++] = r; } return k; }
int main(int argc, char **argv) {
  static uint8_t buf[4096], fr[8192], stream[20000]; rn_link L; rn_link_init(&L, buf, sizeof buf);
  uint8_t pay[1000]; for (int i = 0; i < 1000; i++) pay[i] = (uint8_t)(i * 7 + 3);
  uint32_t n = 0, m;
  const char noise[] = "DDxF garbage D"; memcpy(stream, noise, sizeof noise - 1); n += sizeof noise - 1;
  m = rn_link_frame(fr, sizeof fr, RN_LINK_PROGRAM, pay, 1000); memcpy(stream + n, fr, m); n += m;
  m = rn_link_frame(fr, sizeof fr, RN_LINK_STATUS, 0, 0); memcpy(stream + n, fr, m); n += m;
  m = rn_link_frame(fr, sizeof fr, RN_LINK_PROGRAM, pay, 1000); fr[500] ^= 1; memcpy(stream + n, fr, m); n += m;   /* corrupted */
  static uint8_t big[5000]; m = rn_link_frame(fr, sizeof fr, RN_LINK_PROGRAM, big, 5000); memcpy(stream + n, fr, m); n += m;   /* too big */
  m = rn_link_frame(fr, sizeof fr, RN_LINK_EVENT, (const uint8_t *)"swapped", 7); memcpy(stream + n, fr, m); n += m;
  int t[16], k = feed_all(&L, stream, n, t);
  CHECK(k == 5 && t[0] == RN_LINK_PROGRAM && t[1] == RN_LINK_STATUS && t[2] == -1 && t[3] == -1 && t[4] == RN_LINK_EVENT, "frames, noise, a corrupted and an oversized frame");
  CHECK(L.len == 7 && !memcmp(buf, "swapped", 7), "payload of the last frame");
  if (argc > 1) {
    FILE *f = fopen(argv[1], "rb"); uint32_t fl = f ? (uint32_t)fread(stream, 1, sizeof stream, f) : 0; if (f) fclose(f);
    rn_link_init(&L, buf, sizeof buf); k = feed_all(&L, stream, fl, t);
    CHECK(k == 1 && t[0] == RN_LINK_PROGRAM && L.len == 1000 && !memcmp(buf, pay, 1000), "a frame built by send_program.py");
  }
  printf("%s\n", fails ? "FAILED" : "all passed"); return fails != 0;
}
