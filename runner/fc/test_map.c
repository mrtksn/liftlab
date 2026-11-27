/* Checks the navigation's map (map_core.c): a file made and read back, what a point is, how far the drone can go
 * along a line (with its radius grown in), the unknown beyond what was seen, and bad files refused with the map kept. */
#include "map_core.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
static map_grid M;
static uint8_t cells[60 * 40 * 20], file[40000];

int main(void) {
  /* a room 6 × 4 × 2 m at 10 cm from (−1, −2, 0): free, a wall at x 3.0–3.2 m, the far end (x > 4.5, 5 cells) never
   * seen */
  const int n[3] = { 60, 40, 20 }; const float o[3] = { -1, -2, 0 }, res = 0.1f;
  for (int z = 0; z < 20; z++) for (int y = 0; y < 40; y++) for (int x = 0; x < 60; x++) {
    float px = o[0] + (x + 0.5f) * res; int s = MAP_FREE;
    if (px > 3.0f && px < 3.2f) s = MAP_OCC; else if (px > 4.5f) s = MAP_UNKNOWN;
    cells[x + 60 * (y + 40 * z)] = (uint8_t)s;
  }
  uint32_t len = map_encode(file, sizeof file, res, o, n, cells); char why[80];
  CHECK(len > 40 && len < 120, "a room this plain is small (its rows and layers repeat): %u bytes", len);
  CHECK(map_load(&M, file, len, 0.3f, why, sizeof why) == 0, "loads: %s", why);
  CHECK(M.n_occ == 2 * 40 * 20 && M.n_unknown == 5 * 40 * 20, "the cells counted: %u occupied, %u unknown", M.n_occ, M.n_unknown);
  float p[3] = { 0, 0, 1 }, wall[3] = { 3.1f, 0, 1 }, near[3] = { 2.85f, 0, 1 }, far[3] = { 5, 0, 1 }, out[3] = { 0, 0, 5 };
  CHECK(map_at(&M, p, 0) == MAP_FREE && map_at(&M, wall, 0) == MAP_OCC && map_at(&M, far, 0) == MAP_UNKNOWN && map_at(&M, out, 0) == MAP_UNKNOWN, "what points are");
  CHECK(map_at(&M, near, 0) == MAP_FREE && map_at(&M, near, 1) == MAP_OCC, "25 cm from the wall: free, but inside the grown wall");
  float d_occ, d_unk, east[3] = { 1, 0, 0 }, west[3] = { -1, 0, 0 };
  map_look(&M, p, east, 10, &d_occ, &d_unk);
  CHECK(fabsf(d_occ - 2.7f) < 0.11f, "toward the wall: it stops 0.3 m short of it at x 3.0 (%.2f m)", (double)d_occ);
  CHECK(fabsf(d_unk - d_occ) < 1e-6f, "and the unknown beyond the wall isn't reached first (%.2f)", (double)d_unk);
  float q[3] = { 4.0f, 0, 1 };
  map_look(&M, q, east, 10, &d_occ, &d_unk);
  CHECK(fabsf(d_unk - 0.5f) < 0.11f && d_occ > 9.9f, "beyond the wall: the unknown 0.5 m on (%.2f), nothing occupied (%.2f)", (double)d_unk, (double)d_occ);
  map_look(&M, p, west, 3, &d_occ, &d_unk);
  CHECK(fabsf(d_unk - 1.0f) < 0.11f && d_occ >= 3, "west: the map ends 1 m away (%.2f)", (double)d_unk);
  map_look(&M, near, east, 3, &d_occ, &d_unk);
  CHECK(d_occ == 0, "from inside the grown wall: 0 (%.2f)", (double)d_occ);
  map_grow(&M, 0.05f); CHECK(map_at(&M, near, 1) == MAP_FREE, "a smaller radius: 25 cm from the wall is free");
  /* bad files: refused, the map kept */
  uint32_t crc = M.crc, mid = len / 2; file[mid] ^= 1;
  CHECK(map_load(&M, file, len, 0.3f, why, sizeof why) == -1 && strstr(why, "CRC") && M.ok && M.crc == crc, "a damaged file: refused, the map kept (%s)", why);
  CHECK(map_at(&M, near, 1) == MAP_FREE, "and its grown copy as it was");
  file[mid] ^= 1; file[0] = 'X';
  CHECK(map_load(&M, file, len, 0.3f, why, sizeof why) == -1, "not a map file: %s", why);
  file[0] = 'D';
  CHECK(map_encode(file, 50, res, o, n, cells) == 0, "a file that doesn't fit: 0");
  { /* an irregular map, every cell read back as it was (runs, row and layer copies) */
    const int m[3] = { 23, 17, 9 }; static uint8_t c2[23 * 17 * 9]; unsigned r = 7;
    for (int i = 0; i < 23 * 17 * 9; i++) { r = r * 1103515245u + 12345u; int x = i % 23, y = (i / 23) % 17;
      c2[i] = (uint8_t)(i >= 23 * 17 && (r >> 16) % 4 ? c2[i - 23 * 17] : y && (r >> 20) % 3 ? c2[i - 23] : x > 5 && x < 9 ? MAP_OCC : (r >> 24) % 3); }
    uint32_t l3 = map_encode(file, sizeof file, 0.2f, o, m, c2); int same = l3 > 0 && map_load(&M, file, l3, 0, why, sizeof why) == 0;
    for (int i = 0; same && i < 23 * 17 * 9; i++) { float q2[3] = { o[0] + (i % 23 + 0.5f) * 0.2f, o[1] + ((i / 23) % 17 + 0.5f) * 0.2f, o[2] + (i / (23 * 17) + 0.5f) * 0.2f }; if (map_at(&M, q2, 0) != c2[i]) same = 0; }
    CHECK(same, "an irregular map reads back cell for cell (%u bytes): %s", l3, why);
  }
  if (fails) { printf("map: %d checks failed\n", fails); return 1; }
  printf("map: all checks passed\n");
  return 0;
}
