/* The navigation's map: see map_core.h. */
#include "map_core.h"
#include "rn.h"

static uint32_t rd32(const uint8_t *b) { return (uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24; }
static float rdf(const uint8_t *b) { union { uint32_t u; float f; } x; x.u = rd32(b); return x.f; }
static void wr32(uint8_t *b, uint32_t v) { b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); b[2] = (uint8_t)(v >> 16); b[3] = (uint8_t)(v >> 24); }
static void wrf(uint8_t *b, float f) { union { uint32_t u; float f; } x; x.f = f; wr32(b, x.u); }
static int get2(const uint8_t *a, uint32_t i) { return (a[i >> 2] >> ((i & 3) * 2)) & 3; }
static void set2(uint8_t *a, uint32_t i, int v) { uint8_t s = (uint8_t)((i & 3) * 2); a[i >> 2] = (uint8_t)((a[i >> 2] & ~(3u << s)) | ((unsigned)v << s)); }
static void say(char *why, int wn, const char *s) { int i = 0; if (!why || wn <= 0) return; for (; s[i] && i < wn - 1; i++) why[i] = s[i]; why[i] = 0; }
static int finite_pos(float x) { return x == x && x > 0 && x < 1e6f; }
static int finite(float x) { return x == x && x > -1e7f && x < 1e7f; }

int map_load(map_grid *M, const uint8_t *b, uint32_t len, float radius, char *why, int wn) {
  if (len < 40 || b[0] != 'D' || b[1] != 'F' || b[2] != 'M' || b[3] != 'P') { say(why, wn, "not a map file (DFMP)"); return -1; }
  if (rd32(b + 4) != MAP_VERSION) { say(why, wn, "a map file of another version"); return -1; }
  if (rn_crc32(b, len - 4) != rd32(b + len - 4)) { say(why, wn, "the map file is damaged (its CRC)"); return -1; }
  float res = rdf(b + 8), o[3] = { rdf(b + 12), rdf(b + 16), rdf(b + 20) }; uint32_t n[3] = { rd32(b + 24), rd32(b + 28), rd32(b + 32) };
  if (!finite_pos(res) || !finite(o[0]) || !finite(o[1]) || !finite(o[2])) { say(why, wn, "the map's resolution or origin is bad"); return -1; }
  if (!n[0] || !n[1] || !n[2] || n[0] > MAP_MAX_CELLS || n[1] > MAP_MAX_CELLS || n[2] > MAP_MAX_CELLS || (uint64_t)n[0] * n[1] * n[2] > MAP_MAX_CELLS) { say(why, wn, "the map has too many cells (MAP_MAX_CELLS)"); return -1; }
  uint32_t total = n[0] * n[1] * n[2], at = 0, k = 36, end = len - 4, cnt[3] = { 0, 0, 0 };
  /* decode into grown first, then copy (a bad file leaves the loaded map as it was: its grown copy made again) */
  int bad = 0;
  while (!bad && k < end && at < total) {
    int st = b[k++]; if (st > 4) { say(why, wn, "a run's kind in the map file is bad"); bad = 1; break; }
    uint32_t run = 0; int sh = 0;
    for (;;) { if (k >= end || sh > 28) { say(why, wn, "the map file ends inside a run"); bad = 1; break; } uint8_t c = b[k++]; run |= (uint32_t)(c & 0x7f) << sh; sh += 7; if (!(c & 0x80)) break; }
    if (bad) break;
    if (!run || run > total - at) { say(why, wn, "a run in the map file is bad"); bad = 1; break; }
    uint32_t back = st == 3 ? n[0] : st == 4 ? n[0] * n[1] : 0;     /* (a copy: from a row, or a layer, back) */
    if (back > at) { say(why, wn, "a copy in the map file reaches before its start"); bad = 1; break; }
    for (uint32_t i = 0; i < run; i++) { int c = back ? get2(M->grown, at + i - back) : st; set2(M->grown, at + i, c); cnt[c]++; }
    at += run;
  }
  if (!bad && (at != total || k != end)) { say(why, wn, "the map file's cells don't add up"); bad = 1; }
  if (bad) { if (M->ok) map_grow(M, M->radius); return -1; }
  for (uint32_t i = 0; i < (total + 3) / 4; i++) M->raw[i] = M->grown[i];
  M->res = res; for (int i = 0; i < 3; i++) { M->origin[i] = o[i]; M->n[i] = (int)n[i]; }
  M->crc = rd32(b + len - 4); M->n_unknown = cnt[0]; M->n_free = cnt[1]; M->n_occ = cnt[2]; M->ok = 1;
  map_grow(M, radius);
  say(why, wn, "");
  return 0;
}

/* grow the occupied cells along one axis by k: cells within k of an occupied one (2) become 3, then all 3s become 2 */
static void grow_axis(map_grid *M, int ax, int k) {
  const int nx = M->n[0], ny = M->n[1], nz = M->n[2];
  const int len = M->n[ax], stride = ax == 0 ? 1 : ax == 1 ? nx : nx * ny;
  const int o1 = ax == 0 ? ny : nx, o2 = ax == 2 ? ny : nz;   /* the line's start runs over the other two axes */
  for (int a = 0; a < o1; a++) for (int c = 0; c < o2; c++) {
    uint32_t base = ax == 0 ? (uint32_t)nx * ((uint32_t)a + (uint32_t)ny * (uint32_t)c)          /* a = y, c = z */
                  : ax == 1 ? (uint32_t)a + (uint32_t)nx * (uint32_t)ny * (uint32_t)c             /* a = x, c = z */
                            : (uint32_t)a + (uint32_t)nx * (uint32_t)c;                           /* a = x, c = y */
    int last = -1000000;
    for (int i = 0; i < len; i++) { uint32_t j = base + (uint32_t)(i * stride); int s = get2(M->grown, j); if (s == MAP_OCC) last = i; else if (i - last <= k) set2(M->grown, j, 3); }
    last = 1000000;
    for (int i = len - 1; i >= 0; i--) { uint32_t j = base + (uint32_t)(i * stride); int s = get2(M->grown, j); if (s == MAP_OCC) last = i; else if (last - i <= k) set2(M->grown, j, 3); }
  }
  uint32_t total = (uint32_t)nx * (uint32_t)ny * (uint32_t)nz;
  for (uint32_t i = 0; i < total; i++) if (get2(M->grown, i) == 3) set2(M->grown, i, MAP_OCC);
}

void map_grow(map_grid *M, float radius) {
  if (!M->ok) return;
  uint32_t total = (uint32_t)M->n[0] * (uint32_t)M->n[1] * (uint32_t)M->n[2];
  for (uint32_t i = 0; i < (total + 3) / 4; i++) M->grown[i] = M->raw[i];
  M->radius = radius > 0 ? radius : 0;
  int k = 0; while ((float)k * M->res < M->radius - 1e-6f) k++;
  if (k > 0) for (int ax = 0; ax < 3; ax++) grow_axis(M, ax, k);
}

static int cell_of(const map_grid *M, const float p[3], uint32_t *idx) {
  int c[3];
  for (int i = 0; i < 3; i++) { float f = (p[i] - M->origin[i]) / M->res; if (!(f >= 0) || f >= (float)M->n[i]) return 0; c[i] = (int)f; }
  *idx = (uint32_t)c[0] + (uint32_t)M->n[0] * ((uint32_t)c[1] + (uint32_t)M->n[1] * (uint32_t)c[2]);
  return 1;
}

int map_at(const map_grid *M, const float p[3], int grown) {
  uint32_t i; if (!M->ok || !cell_of(M, p, &i)) return MAP_UNKNOWN;
  return get2(grown ? M->grown : M->raw, i);
}

void map_look(const map_grid *M, const float p[3], const float d[3], float maxd, int grown, float *d_occ, float *d_unk) {
  *d_occ = maxd; *d_unk = maxd;
  if (!M->ok) { *d_unk = 0; return; }
  float step = M->res * 0.5f; int seen_unk = 0;
  for (float s = 0;; s += step) {
    if (s > maxd) s = maxd;
    float q[3] = { p[0] + d[0] * s, p[1] + d[1] * s, p[2] + d[2] * s };
    int st = map_at(M, q, grown);
    if (st == MAP_OCC) { *d_occ = s; if (!seen_unk) *d_unk = s; return; }
    if (st == MAP_UNKNOWN && !seen_unk) { seen_unk = 1; *d_unk = s; }
    if (s >= maxd) return;
  }
}

static uint32_t leb(uint8_t *o, uint32_t v) { uint32_t k = 0; do { uint8_t c = v & 0x7f; v >>= 7; o[k++] = (uint8_t)(c | (v ? 0x80 : 0)); } while (v); return k; }

uint32_t map_encode(uint8_t *out, uint32_t cap, float res, const float origin[3], const int n[3], const uint8_t *cells) {
  if (cap < 44) return 0;
  out[0] = 'D'; out[1] = 'F'; out[2] = 'M'; out[3] = 'P'; wr32(out + 4, MAP_VERSION); wrf(out + 8, res);
  for (int i = 0; i < 3; i++) { wrf(out + 12 + 4 * i, origin[i]); wr32(out + 24 + 4 * i, (uint32_t)n[i]); }
  uint32_t total = (uint32_t)n[0] * (uint32_t)n[1] * (uint32_t)n[2], k = 36, row = (uint32_t)n[0], lay = (uint32_t)n[0] * (uint32_t)n[1];
  for (uint32_t i = 0; i < total;) {
    /* the longest of: the same state; as a row back; as a layer back */
    uint32_t j = i; while (j < total && cells[j] == cells[i]) j++;
    uint32_t best = j - i; uint8_t kind = cells[i];
    if (i >= row) { uint32_t r = i; while (r < total && cells[r] == cells[r - row]) r++; if (r - i > best) { best = r - i; kind = 3; } }
    if (i >= lay) { uint32_t r = i; while (r < total && cells[r] == cells[r - lay]) r++; if (r - i > best) { best = r - i; kind = 4; } }
    if (k + 6 + 4 > cap) return 0;
    out[k++] = kind; k += leb(out + k, best); i += best;
  }
  wr32(out + k, rn_crc32(out, k)); return k + 4;
}
