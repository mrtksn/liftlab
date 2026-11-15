/* file_rx.h: a file in pieces of text, as LiftLab's drone card sends it (js/drone-card.js cardUpload): pieces in any
 * order and again, the missing ones listed, a damaged one found, names and sizes refused.
 *   cc -O2 -I.. -o test_file_rx test_file_rx.c file_rx.c fc_core.c ../rn_host.c ../rn.c ../rn_builtin.c -lm && ./test_file_rx */
#include "file_rx.h"
#include "../rn.h"
#include <stdio.h>
#include <string.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void b64(const uint8_t *p, int n, char *o) {
  static const char *A = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  int k = 0;
  for (int i = 0; i < n; i += 3) {
    uint32_t v = (uint32_t)p[i] << 16 | (i + 1 < n ? (uint32_t)p[i + 1] << 8 : 0) | (i + 2 < n ? p[i + 2] : 0);
    o[k++] = A[v >> 18 & 63]; o[k++] = A[v >> 12 & 63]; o[k++] = i + 1 < n ? A[v >> 6 & 63] : '='; o[k++] = i + 2 < n ? A[v & 63] : '=';
  }
  o[k] = 0;
}
static int piece(file_rx *R, const uint8_t *f, uint32_t size, uint32_t off, char *r) {
  char d[80], line[96]; uint32_t n = size - off < FILE_RX_PIECE ? size - off : FILE_RX_PIECE;
  b64(f + off, (int)n, d); snprintf(line, sizeof line, "fd %u %s", (unsigned)off, d);
  CHECK(strlen(line) <= 57, "a piece's line is %d characters", (int)strlen(line));
  return file_rx_line(R, line, r, 200);
}

int main(void) {
  static file_rx R; static uint8_t f[3613]; char r[200], line[96];
  for (unsigned i = 0; i < sizeof f; i++) f[i] = (uint8_t)(i * 37 + (i >> 5));
  uint32_t size = sizeof f, crc = rn_crc32(f, size), np = file_rx_pieces(size);

  CHECK(file_rx_line(&R, "status", r, sizeof r) == 0, "not one of these");
  CHECK(file_rx_line(&R, "fd 0 AAAA", r, sizeof r) == 1 && strstr(r, "no file"), "a piece before a file: %s", r);
  CHECK(file_rx_line(&R, "file ../etc/x 10 1f", r, sizeof r) == 1 && strstr(r, "no folders"), "a folder in the name: %s", r);
  CHECK(file_rx_line(&R, "file .hidden 10 1f", r, sizeof r) == 1 && strstr(r, "no folders"), "a dot first: %s", r);
  CHECK(file_rx_line(&R, "file big 40000 1f", r, sizeof r) == 1 && strstr(r, "1 to"), "too big: %s", r);
  CHECK(file_rx_line(&R, "file x 10", r, sizeof r) == 1 && strstr(r, "file NAME SIZE CRC"), "no CRC: %s", r);

  snprintf(line, sizeof line, "file drone.dfa %u %x", (unsigned)size, (unsigned)crc);
  char nm[40]; uint32_t s2, c2;
  CHECK(file_rx_begins(line, nm, &s2, &c2) && !strcmp(nm, "drone.dfa") && s2 == size && c2 == crc, "the begin line read");
  CHECK(file_rx_line(&R, line, r, sizeof r) == 1 && strstr(r, "send them"), "begin: %s", r);
  /* every piece but three, backwards, some twice */
  for (int k = (int)np - 1; k >= 0; k--) if (k != 3 && k != 50 && k != (int)np - 1) {
    CHECK(piece(&R, f, size, (uint32_t)k * FILE_RX_PIECE, r) == 1 && !r[0], "a piece says nothing: %s", r);
    if (k % 7 == 0) piece(&R, f, size, (uint32_t)k * FILE_RX_PIECE, r);
  }
  CHECK(file_rx_line(&R, "fd 5 AAAA", r, sizeof r) == 1 && strstr(r, "not a piece"), "an offset off the pieces: %s", r);
  CHECK(file_rx_line(&R, "fd 0 AAAA", r, sizeof r) == 1 && strstr(r, "damaged"), "a piece of the wrong length: %s", r);
  CHECK(file_rx_line(&R, "file end", r, sizeof r) == 1, "end with pieces missing");
  char want[80]; snprintf(want, sizeof want, "missing 3 of %u: %d %d %u", (unsigned)np, 3 * FILE_RX_PIECE, 50 * FILE_RX_PIECE, (unsigned)((np - 1) * FILE_RX_PIECE));
  CHECK(strstr(r, want) != 0, "the missing pieces: \"%s\" (want \"%s\")", r, want);
  piece(&R, f, size, 3 * FILE_RX_PIECE, r); piece(&R, f, size, 50 * FILE_RX_PIECE, r); piece(&R, f, size, (np - 1) * FILE_RX_PIECE, r);
  CHECK(file_rx_line(&R, "file", r, sizeof r) == 1 && strstr(r, "pieces"), "where it stands: %s", r);
  CHECK(file_rx_line(&R, "file end", r, sizeof r) == 2 && strstr(r, "whole"), "whole: %s", r);
  CHECK(R.size == size && !memcmp(R.buf, f, size) && !strcmp(R.name, "drone.dfa"), "the file as it was sent");

  /* a damaged piece: the CRC finds it, and it starts again */
  snprintf(line, sizeof line, "file x.txt %u %x", (unsigned)size, (unsigned)crc);
  file_rx_line(&R, line, r, sizeof r);
  f[100] ^= 1; for (uint32_t o = 0; o < size; o += FILE_RX_PIECE) piece(&R, f, size, o, r); f[100] ^= 1;
  CHECK(file_rx_line(&R, "file end", r, sizeof r) == 1 && strstr(r, "CRC"), "damaged: %s", r);
  CHECK(file_rx_line(&R, "file end", r, sizeof r) == 1 && strstr(r, "missing"), "after damage, all missing: %s", r);
  for (uint32_t o = 0; o < size; o += FILE_RX_PIECE) piece(&R, f, size, o, r);
  CHECK(file_rx_line(&R, "file end", r, sizeof r) == 2, "again, whole: %s", r);

  /* a one-byte file, and a long missing list cut to fit */
  file_rx_line(&R, "file one 1 e8b7be43", r, sizeof r);
  CHECK(file_rx_line(&R, "fd 0 YQ==", r, sizeof r) == 1 && !r[0], "one byte: %s", r);
  CHECK(file_rx_line(&R, "file end", r, sizeof r) == 2 && R.buf[0] == 'a', "one byte, whole (crc32 of \"a\"): %s", r);
  snprintf(line, sizeof line, "file x %u %x", (unsigned)size, (unsigned)crc); file_rx_line(&R, line, r, sizeof r);
  char small[60]; file_rx_line(&R, "file end", small, sizeof small);
  CHECK(strlen(small) < sizeof small && strstr(small, "missing"), "the missing list fits its room: %s", small);
  file_rx_line(&R, "file end", r, sizeof r);
  CHECK(strlen(r) <= FILE_RX_SAY && strstr(r, "missing 101 of 101: 0 36"), "the missing list fits one message (%d): %s", (int)strlen(r), r);

  if (fails) printf("file_rx: %d FAILED\n", fails);
  else printf("file_rx: ok (%u pieces of %d bytes)\n", (unsigned)np, FILE_RX_PIECE);
  return fails != 0;
}
