/*
 * Optical flow from a downward camera: how far the whole picture moved between two frames.
 *
 * Portable C (no camera, no OS): the Pi's camera program (pi/dfb_flow.c) feeds it frames; the tests feed it pictures
 * they made. One shift for the whole patch (the floor under a hovering drone moves as one piece), found by
 * Lucas–Kanade on a small pyramid: each level halves the picture, the coarsest finds big moves, the finer ones refine
 * them to a fraction of a pixel. Each picture is made zero-mean and unit-spread first, so the camera's exposure
 * changing (or a lamp flickering) between frames is not taken for movement.
 *
 * The answer is in pixels of the patch, along the picture's x (right) and y (down): a point of the floor that was at
 * (x, y) is at (x + dx, y + dy) now. Turning that into angular rates in the camera's axes (what the navigation's
 * flowVelocity takes) is the caller's: it knows the lens (radians per pixel) and how long passed.
 *
 * Quality, 0 to 1: how much texture the patch has (a plain floor has none) and how well the two pictures matched
 * after the shift (noise in a dark room, blur, something moving in the picture, a jump too big to follow make it poor).
 */
#ifndef FLOW_CORE_H
#define FLOW_CORE_H
#include <stdint.h>

#define FLOW_MAX_N 128                   /* the largest patch side [px] */
#define FLOW_MAX_LEVELS 4

typedef struct {
  float dx, dy;                          /* the shift [patch px] */
  float q;                               /* quality 0–1 (0: no reading) */
  float texture;                         /* the patch's texture: smallest eigenvalue of its gradient matrix per pixel */
  float resid;                           /* what's left after the shift: mean squared difference (0 perfect, ~2 unrelated) */
  int iters;                             /* iterations used, all levels */
} flow_result;

typedef struct {
  int n, levels, iters;                  /* patch side (a multiple of 2^(levels−1)), pyramid levels, iterations per level */
  float min_texture;                     /* texture below this: quality falls to 0 (default 0.02) */
  float pred[2];                         /* where the search starts [patch px] (the last shift: constant velocity) */
  int have_prev;
  float img[2][FLOW_MAX_LEVELS][FLOW_MAX_N * FLOW_MAX_N];   /* the two pictures' pyramids (normalized), current and previous */
  int cur;
} flow_state;

/* n: patch side; levels 1–4; iters per level. 0, or −1 if the sizes don't fit. */
int flow_init(flow_state *F, int n, int levels, int iters);
/* Forget the previous frame (after a gap: the next frame only starts again). */
void flow_reset(flow_state *F);
/* A new frame: an n×n grey picture (stride bytes per row). Returns 1 with r filled (the shift since the previous
 * frame), or 0 for the first frame (nothing to compare with). */
int flow_frame(flow_state *F, const uint8_t *img, int stride, flow_result *r);
/* Start the next search here instead of at the last shift [patch px] (a prediction from the gyro, say). */
static inline void flow_predict(flow_state *F, float dx, float dy) { F->pred[0] = dx; F->pred[1] = dy; }

/* A grey patch from a camera's raw Bayer frame: the bin×bin raw pixels under each patch pixel averaged (bin even, so
 * each takes whole 2×2 colour cells: the colours don't matter for flow). raw10: the 10-bit packed layout (4 pixels in
 * 5 bytes, their high 8 bits first), else 8 bits a pixel. (x0, y0): the patch's top-left in raw pixels (even).
 * Returns the patch's mean brightness (0–255). */
int flow_bin_bayer(const uint8_t *raw, int bpl, int raw10, int x0, int y0, int n, int bin, uint8_t *out);

#endif
