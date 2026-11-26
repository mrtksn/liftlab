/* Checks the optical flow core (flow_core.c) on pictures made here: a floor texture (a sum of waves, so it can be
 * drawn at any sub-pixel shift exactly), shifted, noisy, darker, plain; and the Bayer binning from a raw frame. */
#include "flow_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

#define NW 40
static float wa[NW], wkx[NW], wky[NW], wph[NW];
static unsigned rs = 12345;
static float rnd(void) { rs = rs * 1103515245u + 12345u; return ((rs >> 8) & 0xffff) / 65536.0f; }
static float gauss(void) { float u = rnd() + 1e-6f, v = rnd(); return sqrtf(-2 * logf(u)) * cosf(6.2831853f * v); }

static void make_floor(void) {
  for (int i = 0; i < NW; i++) {
    float lam = 4 + 36 * rnd(), ang = 6.2831853f * rnd();       /* wavelengths 4–40 px, all directions */
    wkx[i] = 6.2831853f / lam * cosf(ang); wky[i] = 6.2831853f / lam * sinf(ang); wph[i] = 6.2831853f * rnd(); wa[i] = 0.5f + rnd();
  }
}
/* the floor seen with its texture moved by (sx, sy): what was at (x, y) is at (x + sx, y + sy) */
static void draw(uint8_t *img, int n, float sx, float sy, float gain, float offset, float noise, float contrast) {
  for (int y = 0; y < n; y++) for (int x = 0; x < n; x++) {
    float X = x - sx, Y = y - sy, s = 0;
    for (int i = 0; i < NW; i++) s += wa[i] * sinf(wkx[i] * X + wky[i] * Y + wph[i]);
    float v = offset + gain * (128 + contrast * 12 * s) + noise * gauss();
    img[y * n + x] = (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v + 0.5f);
  }
}

static flow_state F;
static uint8_t a[FLOW_MAX_N * FLOW_MAX_N], b[FLOW_MAX_N * FLOW_MAX_N];

static flow_result pair(int n, float sx, float sy, float gain2, float off2, float noise, float contrast) {
  flow_result r; memset(&r, 0, sizeof r);
  flow_reset(&F);
  draw(a, n, 0, 0, 1, 0, noise, contrast); flow_frame(&F, a, n, &r);
  draw(b, n, sx, sy, gain2, off2, noise, contrast);
  if (!flow_frame(&F, b, n, &r)) { fails++; printf("FAIL: no result on the second frame\n"); }
  return r;
}

int main(void) {
  make_floor();
  int sizes[] = { 64, 128 };
  for (int si = 0; si < 2; si++) {
    int n = sizes[si];
    CHECK(!flow_init(&F, n, 3, 6), "init %d", n);
    /* clean shifts, whole and sub-pixel, up to a quarter of the coarse range */
    float sh[][2] = { { 0, 0 }, { 0.3f, -0.2f }, { 1.5f, 0.7f }, { -2.25f, 3.1f }, { 5.4f, -4.6f }, { -7.8f, -6.3f } };
    for (int k = 0; k < 6; k++) {
      flow_result r = pair(n, sh[k][0], sh[k][1], 1, 0, 0, 1);
      float e = hypotf(r.dx - sh[k][0], r.dy - sh[k][1]);
      CHECK(e < 0.05f && r.q > 0.7f, "n=%d shift (%.2f, %.2f): got (%.3f, %.3f) q %.2f tex %.3f resid %.3f", n, sh[k][0], sh[k][1], r.dx, r.dy, r.q, r.texture, r.resid);
      if (k == 4) printf("n=%d clean (5.4, -4.6): (%.3f, %.3f) q %.2f tex %.3f resid %.4f, %d iterations\n", n, r.dx, r.dy, r.q, r.texture, r.resid, r.iters);
    }
    /* exposure changed between the frames (darker, offset): still the same shift */
    flow_result r = pair(n, 1.2f, -0.8f, 0.6f, 10, 0, 1);
    CHECK(hypotf(r.dx - 1.2f, r.dy + 0.8f) < 0.05f && r.q > 0.7f, "n=%d exposure change: (%.3f, %.3f) q %.2f", n, r.dx, r.dy, r.q);
    /* a noisy dim picture: close, lower quality */
    r = pair(n, 2.0f, 1.0f, 1, 0, 6, 0.4f);
    CHECK(hypotf(r.dx - 2.0f, r.dy - 1.0f) < 0.15f && r.q > 0.2f, "n=%d noisy: (%.3f, %.3f) q %.2f resid %.3f", n, r.dx, r.dy, r.q, r.resid);
    printf("n=%d noisy dim: (%.3f, %.3f) q %.2f tex %.3f resid %.3f\n", n, r.dx, r.dy, r.q, r.texture, r.resid);
    /* a plain floor (only sensor noise): no reading */
    r = pair(n, 1, 1, 1, 0, 3, 0);
    CHECK(r.q < 0.1f, "n=%d plain floor: q %.2f (%.3f, %.3f) tex %.3f resid %.3f", n, r.q, r.dx, r.dy, r.texture, r.resid);
    /* two unrelated pictures: no reading */
    flow_reset(&F); draw(a, n, 0, 0, 1, 0, 0, 1); flow_frame(&F, a, n, &r);
    for (int i = 0; i < n * n; i++) b[i] = (uint8_t)(rnd() * 255);
    flow_frame(&F, b, n, &r);
    CHECK(r.q < 0.1f, "n=%d unrelated pictures: q %.2f resid %.3f", n, r.q, r.resid);
    /* a steady glide: frame after frame, each step's shift starts from the last (constant velocity), faster than the
     * pyramid alone would follow */
    flow_reset(&F); float v[2] = { 9.0f, -6.5f }; int bad = 0;
    for (int f = 0; f < 12; f++) { draw(a, n, v[0] * f, v[1] * f, 1, 0, 0, 1); if (flow_frame(&F, a, n, &r) && (hypotf(r.dx - v[0], r.dy - v[1]) > 0.1f || r.q < 0.5f)) bad++; }
    CHECK(!bad, "n=%d glide at (9, -6.5) px/frame: %d frames off", n, bad);
  }

  /* the Bayer binning: a raw10 frame with known values in each 2×2 cell */
  {
    int W = 64, H = 32, bpl = W / 4 * 5; static uint8_t raw[32 * 80];
    memset(raw, 0, sizeof raw);
    for (int y = 0; y < H; y++) for (int x = 0; x < W; x++) raw[y * bpl + (x >> 2) * 5 + (x & 3)] = (uint8_t)((x / 4) * 10 + (y / 4) + ((x + y) & 1) * 2);
    uint8_t out[16]; int mean = flow_bin_bayer(raw, bpl, 1, 8, 4, 4, 4, out);
    /* patch pixel (px, py) covers raw x 8+4px …, y 4+4py …: (2+px)*10 + (1+py) + 1 (half the cells +2); mean 38.5 → 39 */
    int okb = 1; for (int py = 0; py < 4; py++) for (int px = 0; px < 4; px++) if (out[py * 4 + px] != (2 + px) * 10 + 1 + py + 1) okb = 0;
    CHECK(okb && mean == 39, "raw10 binning: first %d mean %d", out[0], mean);
  }

  /* the speed: frames a second this computer manages (the Pi measures its own with dfb_flow --bench) */
  {
    flow_init(&F, 64, 3, 6); flow_result r; int frames = 2000;
    draw(a, 64, 0, 0, 1, 0, 2, 1); draw(b, 64, 1.3f, -0.6f, 1, 0, 2, 1);
    for (int i = 0; i < frames; i++) flow_frame(&F, (i & 1) ? b : a, 64, &r);
  }
  if (fails) { printf("flow: %d checks failed\n", fails); return 1; }
  printf("flow: all checks passed\n");
  return 0;
}
