/* Fixed numerical fixture, independently recomputed by test_hardware_tuning.js.
 */
#include "autotune_core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static at_complex product(at_complex a, at_complex b) {
  return (at_complex){a.r * b.r - a.i * b.i, a.r * b.i + a.i * b.r};
}
static at_complex quotient(at_complex a, at_complex b) {
  double q = b.r * b.r + b.i * b.i;
  return (at_complex){(a.r * b.r + a.i * b.i) / q, (a.i * b.r - a.r * b.i) / q};
}
static at_complex fixture(double hz) {
  double w = 6.283185307179586 * hz;
  return product(quotient((at_complex){-1.05 * cos(w * .012) / (w * w),
                                       1.05 * sin(w * .012) / (w * w)},
                          (at_complex){1, w * .03}),
                 (at_complex){1, w * .02});
}
static at_complex closed(double hz, const double *g, at_complex G) {
  double w = 6.283185307179586 * hz;
  at_complex L = product(G, (at_complex){g[0], w * g[1] - g[2] / w});
  return quotient(product(G, (at_complex){g[0], -g[2] / w}),
                  (at_complex){1 + L.r, L.i});
}
static void print_model(const at_model *m) {
  printf("[%.12g,%.12g,%.12g,%.12g,%.12g]", m->gain, m->lag, m->delay, m->lead,
         m->error);
}
int main(void) {
  double samples[500][3];
  for (int k = 0; k < 500; k++) {
    double t = k * .005 + .0002 * sin(k);
    samples[k][0] = t;
    samples[k][1] = .13 * sin(6.283185307179586 * 1.6 * t) +
                    .07 * cos(6.283185307179586 * 1.6 * t) + .4 + .03 * t +
                    .001 * sin(17 * t);
    samples[k][2] = 0;
  }
  at_bin b = autotune_bin(samples, 500, 1.6, .05, 1, 1), bins[4] = {0},
         pos[3] = {0};
  double hz[] = {.8, 1.6, 3.2, 6.4}, phz[] = {.18, .32, .55};
  double ig[] = {100, 16, 80}, pg[] = {4, 3.6, 1};
  for (int i = 0; i < 4; i++) {
    bins[i].hz = hz[i];
    bins[i].quality = .99;
    bins[i].n = 500;
    bins[i].T = closed(hz[i], ig, fixture(hz[i]));
    bins[i].plant = fixture(hz[i]);
  }
  at_model models[3];
  assert(autotune_fit(bins, 4, models, 0, 0, 0));
  models[1] = models[2] = models[0];
  for (int i = 0; i < 3; i++) {
    pos[i].hz = phz[i];
    pos[i].quality = .99;
    pos[i].n = 500;
    pos[i].T =
        closed(phz[i], pg,
               product(fixture(phz[i]), closed(phz[i], ig, fixture(phz[i]))));
  }
  at_model pm;
  assert(autotune_fit(pos, 3, &pm, ig, models, pg));
  float before[12], att[12], outer[12];
  pid_defaults(before, 0);
  before[9] = 4;
  before[10] = 3.6f;
  before[11] = 1;
  assert(autotune_recommend(before, 0, models, att, 0));
  assert(autotune_recommend(att, 1, &pm, outer, models));
  printf("{\"bin\":[%.12g,%.12g,%.12g],\"attModel\":", b.T.r, b.T.i, b.quality);
  print_model(models);
  printf(",\"posModel\":");
  print_model(&pm);
  printf(",\"att\":[");
  for (int i = 0; i < 12; i++)
    printf("%s%.12g", i ? "," : "", att[i]);
  printf("],\"pos\":[");
  for (int i = 0; i < 12; i++)
    printf("%s%.12g", i ? "," : "", outer[i]);
  puts("]}");
  for (int i = 0; i < 4; i++)
    bins[i].quality = .1;
  assert(!autotune_fit(bins, 4, models, 0, 0, 0));
  return 0;
}
