/* Optical flow from a downward camera: see flow_core.h. */
#include "flow_core.h"
#include <math.h>
#include <string.h>

int flow_init(flow_state *F, int n, int levels, int iters) {
  if (levels < 1 || levels > FLOW_MAX_LEVELS || n > FLOW_MAX_N || iters < 1) return -1;
  if ((n >> (levels - 1)) < 8 || (n & ((1 << (levels - 1)) - 1))) return -1;   /* the coarsest level: 8 px or more, whole */
  memset(F, 0, sizeof *F);
  F->n = n; F->levels = levels; F->iters = iters; F->min_texture = 0.02f;
  return 0;
}

void flow_reset(flow_state *F) { F->have_prev = 0; F->pred[0] = F->pred[1] = 0; }

/* the picture, zero-mean and unit spread, and its halvings */
static void pyramid(flow_state *F, const uint8_t *img, int stride, float (*P)[FLOW_MAX_N * FLOW_MAX_N]) {
  int n = F->n; double s = 0, ss = 0;
  for (int y = 0; y < n; y++) for (int x = 0; x < n; x++) { double v = img[y * stride + x]; s += v; ss += v * v; }
  double mean = s / (n * n), var = ss / (n * n) - mean * mean;
  float k = var > 1e-6 ? (float)(1 / sqrt(var)) : 0, m = (float)mean;
  float *L0 = P[0];
  for (int y = 0; y < n; y++) for (int x = 0; x < n; x++) L0[y * n + x] = (img[y * stride + x] - m) * k;
  for (int l = 1; l < F->levels; l++) {
    int m2 = n >> l, m1 = m2 * 2; const float *A = P[l - 1]; float *B = P[l];
    for (int y = 0; y < m2; y++) for (int x = 0; x < m2; x++) {
      const float *a = A + 2 * y * m1 + 2 * x;
      B[y * m2 + x] = 0.25f * (a[0] + a[1] + a[m1] + a[m1 + 1]);
    }
  }
}

/* Lucas–Kanade for one shift on one level (T the previous picture, I the current, side m): d moves to where T's
 * patch lies in I. Returns 0, or −1 if it can't (no texture, or the shift leaves too little overlap). */
static int lk(const float *T, const float *I, int m, int iters, float d[2], float *tex, float *resid, int *used) {
  for (int it = 0; it < iters; it++) {
    int ix = (int)floorf(d[0]), iy = (int)floorf(d[1]); float fx = d[0] - ix, fy = d[1] - iy;
    float w00 = (1 - fx) * (1 - fy), w01 = fx * (1 - fy), w10 = (1 - fx) * fy, w11 = fx * fy;
    /* the pixels whose gradient is there (1 … m−2) and whose shifted place is inside the current picture */
    int x0 = ix < 0 ? 1 - ix : 1, x1 = (m - 2 - ix < m - 2 ? m - 2 - ix : m - 2);
    int y0 = iy < 0 ? 1 - iy : 1, y1 = (m - 2 - iy < m - 2 ? m - 2 - iy : m - 2);
    if (x1 - x0 < m / 3 || y1 - y0 < m / 3) return -1;
    float h11 = 0, h12 = 0, h22 = 0, b1 = 0, b2 = 0, ee = 0;
    for (int y = y0; y <= y1; y++) {
      const float *t = T + y * m, *r = I + (y + iy) * m + ix;
      for (int x = x0; x <= x1; x++) {
        float gx = 0.5f * (t[x + 1] - t[x - 1]), gy = 0.5f * (t[x + m] - t[x - m]);
        float e = w00 * r[x] + w01 * r[x + 1] + w10 * r[x + m] + w11 * r[x + m + 1] - t[x];
        h11 += gx * gx; h12 += gx * gy; h22 += gy * gy; b1 += gx * e; b2 += gy * e; ee += e * e;
      }
    }
    float cnt = (float)((x1 - x0 + 1) * (y1 - y0 + 1));
    float det = h11 * h22 - h12 * h12;
    float tr = h11 + h22, lmin = 0.5f * (tr - sqrtf((h11 - h22) * (h11 - h22) + 4 * h12 * h12));
    *tex = lmin / cnt; *resid = ee / cnt; *used += 1;
    if (det <= 1e-9f * tr * tr || lmin <= 0) return -1;
    float u = (h22 * b1 - h12 * b2) / det, v = (h11 * b2 - h12 * b1) / det;   /* inverse compositional: d − H⁻¹b */
    d[0] -= u; d[1] -= v;
    if (u * u + v * v < 1e-4f) break;
  }
  return 0;
}

int flow_frame(flow_state *F, const uint8_t *img, int stride, flow_result *r) {
  int c = F->cur ^ 1; F->cur = c;
  pyramid(F, img, stride, F->img[c]);
  if (!F->have_prev) { F->have_prev = 1; F->pred[0] = F->pred[1] = 0; return 0; }
  memset(r, 0, sizeof *r);
  int p = c ^ 1, L = F->levels - 1;
  float d[2] = { F->pred[0] / (float)(1 << L), F->pred[1] / (float)(1 << L) }, tex = 0, res = 2;
  int ok = 1;
  for (int l = L; l >= 0 && ok; l--) {
    if (l < L) { d[0] *= 2; d[1] *= 2; }
    float t = 0, e = 2;
    if (lk(F->img[p][l], F->img[c][l], F->n >> l, F->iters, d, &t, &e, &r->iters)) ok = 0;
    if (l == 0) { tex = t; res = e; }
  }
  r->texture = tex; r->resid = res;
  float lim = F->n / 4.0f;
  if (!ok || !(fabsf(d[0]) < lim && fabsf(d[1]) < lim)) { F->pred[0] = F->pred[1] = 0; return 1; }   /* lost: no reading */
  r->dx = d[0]; r->dy = d[1];
  float qt = tex < F->min_texture ? 0 : fminf(1, tex / (4 * F->min_texture));
  float qm = fminf(1, fmaxf(0, 1 - res));
  r->q = qt * qm;
  if (r->q > 0) { F->pred[0] = d[0]; F->pred[1] = d[1]; } else F->pred[0] = F->pred[1] = 0;
  return 1;
}

int flow_bin_bayer(const uint8_t *raw, int bpl, int raw10, int x0, int y0, int n, int bin, uint8_t *out) {
  unsigned total = 0, k = (unsigned)(bin * bin);
  if (raw10 && bin == 4 && !(x0 & 3)) {               /* the usual case, quickly: each patch pixel is 4 rows of one 5-byte group's 4 high bytes */
    for (int py = 0; py < n; py++) {
      const uint8_t *r0 = raw + (size_t)(y0 + py * 4) * (size_t)bpl + (x0 >> 2) * 5, *r1 = r0 + bpl, *r2 = r1 + bpl, *r3 = r2 + bpl;
      for (int px = 0; px < n; px++, r0 += 5, r1 += 5, r2 += 5, r3 += 5) {
        unsigned s = (unsigned)r0[0] + r0[1] + r0[2] + r0[3] + r1[0] + r1[1] + r1[2] + r1[3] + r2[0] + r2[1] + r2[2] + r2[3] + r3[0] + r3[1] + r3[2] + r3[3];
        uint8_t v = (uint8_t)((s + 8) >> 4); out[py * n + px] = v; total += v;
      }
    }
    return (int)((total + (unsigned)(n * n) / 2) / (unsigned)(n * n));
  }
  for (int py = 0; py < n; py++) for (int px = 0; px < n; px++) {
    unsigned s = 0;
    for (int j = 0; j < bin; j++) {
      const uint8_t *row = raw + (size_t)(y0 + py * bin + j) * (size_t)bpl;
      for (int i = 0; i < bin; i++) { int x = x0 + px * bin + i; s += raw10 ? row[(x >> 2) * 5 + (x & 3)] : row[x]; }
    }
    uint8_t v = (uint8_t)((s + k / 2) / k); out[py * n + px] = v; total += v;
  }
  return (int)((total + (unsigned)(n * n) / 2) / (unsigned)(n * n));
}
