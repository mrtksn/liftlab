/* Tests for the command module (ground_core.c) with its built-in program: the sticks and switches as they go up,
 * commands, the drone's telemetry as it comes down, and the alerts.
 *   cc -O2 -I.. -I../fc -o test_ground test_ground.c ground_core.c rn_builtin_ground.c ../fc/tlm_core.c ../fc/tlm_crsf.c ../fc/crsf.c ../fc/rc_core.c ../rn_host.c ../rn.c -lm && ./test_ground */
#include "ground_core.h"
#include "rc_core.h"
#include "tlm_crsf.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

extern const uint8_t *const rn_builtin_ground_img;
extern const uint32_t rn_builtin_ground_len;
static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

static float arenas_[3][16384], pools_[3][4096]; static int32_t codes_[3][16384];
static rn_host H;
static void host(void) {
  float *a[3] = { arenas_[0], arenas_[1], arenas_[2] }, *p[3] = { pools_[0], pools_[1], pools_[2] }; int32_t *c[3] = { codes_[0], codes_[1], codes_[2] };
  memset(&H, 0, sizeof H);
  if (rn_host_init(&H, rn_builtin_ground_img, rn_builtin_ground_len, a, 16384, c, 16384, p, 4096)) { printf("host init failed\n"); fails++; }
}
/* the drone's side: what its receiver hands the flight code */
static rc_input RI; static crsf_parser RP;
static void drone_reads(const uint8_t *b, int n, double t) { for (int i = 0; i < n; i++) tlm_crsf_input(&RP, b[i], &RI, t); }

int main(void) {
  gnd_state G; uint8_t out[256]; gnd_input in; double t = 0; const float dt = 0.004f;
  printf("the sticks and switches\n");
  host();
  CHECK(gnd_init(&G, &H, NULL) == 0, "the built-in ground program has its formulas (%s)", G.why);
  memset(&in, 0, sizeof in);
  in.held = GB(GB_FWD);
  float at125 = 0; double reached = -1;
  for (int k = 0; k < 100; k++) { t += dt; int n = gnd_step(&G, &in, t, dt, out, sizeof out); drone_reads(out, n, t); if (k == 30) at125 = G.stick[GND_PITCH]; if (reached < 0 && G.stick[GND_PITCH] >= 1) reached = t; }
  CHECK(at125 > 0.4f && at125 < 0.6f && reached > 0.2 && reached < 0.3, "a held button eases the pitch stick in: %.2f after 0.12 s, full after %.2f s", at125, reached);
  CHECK(fabsf(RI.ch[RC_PITCH] - 1) < 0.002f && RI.frames > 50, "the drone's receiver gets it: pitch channel %.3f (%u frames)", RI.ch[RC_PITCH], RI.frames);
  in.held = 0; double back = -1, t0 = t;
  for (int k = 0; k < 60; k++) { t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); if (back < 0 && G.stick[GND_PITCH] == 0) back = t - t0; }
  CHECK(back > 0.08 && back < 0.13, "released, back to the centre in %.2f s", back);
  in.has_axis = 1u << GND_ROLL; in.axis[GND_ROLL] = 0.02f; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); float d0 = G.stick[GND_ROLL];
  in.axis[GND_ROLL] = 0.5f; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); float d5 = G.stick[GND_ROLL];
  in.axis[GND_ROLL] = -1; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); float d1 = G.stick[GND_ROLL];
  CHECK(d0 == 0 && fabsf(d5 - 0.368f) < 0.01f && d1 == -1, "a real stick: deadband (0.02 → %.2f), expo (0.5 → %.3f), full (−1 → %.2f)", d0, d5, d1);
  in.has_axis = 0;
  gnd_config c; gnd_config_default(&c); c.latch = GB(GB_ARM) | GB(GB_FLY);
  host(); gnd_init(&G, &H, &c);
  float arm[4]; const uint32_t seq[4] = { GB(GB_ARM), 0, GB(GB_ARM), 0 };
  for (int k = 0; k < 4; k++) { in.held = seq[k]; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); arm[k] = G.ch[RC_ARM]; }
  CHECK(arm[0] == 1 && arm[1] == 1 && arm[2] == -1 && arm[3] == -1, "a push button as a latching arm switch: press on, release stays, press off (%.0f %.0f %.0f %.0f)", arm[0], arm[1], arm[2], arm[3]);
  in.held = GB(GB_SPORT); t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); float lv = G.ch[RC_LEVEL]; in.held = 0;
  CHECK(lv == 1, "the sport button sets the speed level channel to +1");

  printf("commands\n");
  memset(&RI, 0, sizeof RI); memset(&RP, 0, sizeof RP);
  gnd_goto(&G, 3, -2, 4.5f, 1); float cal = 1; gnd_command(&G, RC_CMD_LEARN, &cal, 1);
  double t_goto = -1, t_learn = -1; float gx = 0, gh = 0;
  for (int k = 0; k < 100; k++) {
    t += dt; int n = gnd_step(&G, &in, t, dt, out, sizeof out); uint32_t s0 = RI.cmd_seq; drone_reads(out, n, t);
    if (RI.cmd_seq != s0 && RI.cmd == RC_CMD_GOTO) { t_goto = t; gx = RI.cmd_v[0]; gh = RI.cmd_v[3]; }
    if (RI.cmd_seq != s0 && RI.cmd == RC_CMD_LEARN) t_learn = t;
  }
  CHECK(t_goto > 0 && fabsf(gx - 3) < 0.01f && fabsf(gh - 1) < 0.002f, "go to: x %.2f, heading %.3f", gx, gh);
  CHECK(t_learn - t_goto > 0.14 && t_learn - t_goto < 0.16, "the next command waits its turn: %.2f s later", t_learn - t_goto);
  uint32_t s1 = RI.cmd_seq; in.held = GB(GB_CAL);
  for (int k = 0; k < 60; k++) { t += dt; int n = gnd_step(&G, &in, t, dt, out, sizeof out); drone_reads(out, n, t); }
  in.held = 0;
  CHECK(RI.cmd_seq == s1 % 255 + 1 && RI.cmd == RC_CMD_LEARN && RI.cmd_v[0] == 1, "the calibrate button (held 0.24 s) sends one learning command: calibrate");

  printf("the telemetry, as it comes down\n");
  host(); gnd_init(&G, &H, NULL);
  tlm_store T; tlm_init(&T);
  float att[3] = { 0.1f, -0.2f, 1.5f }, batt[1] = { 15.2f }, st[2] = { 1, 1 }, nav[6] = { 1, 2, 3, 0.5f, 2 | 4 | 8, 1 }, sup[7] = { 2, 4, 2.5f, 0.42f, 4, 12.5f, 300 };
  double lat = 41.0123456, lon = 29.0234567, la = lat * 1e7, lo = lon * 1e7; float hl = (float)floor(la / 65536), ho = (float)floor(lo / 65536);
  float gps[8] = { hl, (float)(la - (double)hl * 65536), ho, (float)(lo - (double)ho * 65536), 120, 3, 90, 11 };
  double tt = 0; uint8_t buf[512];
  for (int k = 0; k < 400; k++) {
    tt += 0.01;
    tlm_put(&T, TLM_ATT, att, 3, tt); tlm_put(&T, TLM_BATT, batt, 1, tt); tlm_put(&T, TLM_STATE, st, 2, tt); tlm_put(&T, TLM_NAV, nav, 6, tt);
    tlm_put(&T, TLM_SUPER, sup, 7, tt); tlm_put(&T, TLM_GPS, gps, 8, tt);
    if (k == 100) tlm_text(&T, 4, "hello from the drone");
    int n = tlm_service(&T, &tlm_crsf, tt, 400, buf, sizeof buf);
    gnd_from_radio(&G, buf, n, tt);
    t += 0.01; gnd_step(&G, &in, tt, 0.01f, out, sizeof out);
  }
  gnd_view *V = &G.V;
  CHECK(fabsf(V->roll - 0.1f) < 1e-3f && fabsf(V->yaw - 1.5f) < 1e-3f, "attitude: roll %.3f, yaw %.3f", V->roll, V->yaw);
  CHECK(fabs(V->lat - lat) < 2e-7 && fabs(V->lon - lon) < 2e-7 && V->sats == 11, "GPS to 1e-7°: %.7f %.7f, %d satellites", V->lat, V->lon, V->sats);
  CHECK(fabsf(V->volts - 15.2f) < 0.06f && V->pct == 42 && fabsf(V->amps - 12.5f) < 0.06f, "battery: %.1f V, %.1f A, %d%%", V->volts, V->amps, V->pct);
  CHECK(!strcmp(V->mode, "RTH"), "flight mode: %s", V->mode);
  CHECK(V->item[TLM_NAV].t > 0 && fabsf(V->item[TLM_NAV].v[1] - 2) < 0.01f && (int)V->item[TLM_NAV].v[4] == 14, "the navigation's item, unscaled: target y %.2f, bits %d", V->item[TLM_NAV].v[1], (int)V->item[TLM_NAV].v[4]);
  CHECK(V->nmsg == 1 && !strcmp(V->msg[0].s, "hello from the drone") && V->msg[0].sev == 4, "message: \"%s\" (severity %d)", V->msg[0].s, V->msg[0].sev);
  CHECK(V->bad == 0 && V->frames > 50, "%u frames, none bad", V->frames);
  int why; int lvl = gnd_alert(&G, &why);
  CHECK(lvl == 1 && why == 5, "alert: returning home (level %d, %s)", lvl, gnd_why_text[why]);
  float f[256]; int nf = gnd_view_pack(&G, tt, f, 256);
  CHECK(nf > 60 && f[0] == 1 && fabs(f[26] + f[27] - lat) < 3e-6, "packed for a display: %d floats, latitude %.6f", nf, f[26] + f[27]);
  /* the telemetry stops: an alarm */
  for (int k = 0; k < 30; k++) { tt += 0.1; gnd_step(&G, &in, tt, 0.1f, out, sizeof out); }
  lvl = gnd_alert(&G, &why);
  CHECK(lvl == 2 && why == 1, "nothing comes down for 3 s: alarm (%s)", gnd_why_text[why]);
  /* the same, but the module still hears the drone's telemetry packets well: the link's settings leave little room */
  { uint8_t ls[64]; crsf_link Lk = { -50, 100, 9, -51, 100, 8, 0, 10, 0 };
    for (int k = 0; k < 40; k++) { tt += 0.1; int nl = crsf_link_stats(ls, CRSF_ADDR_HANDSET, &Lk); gnd_from_radio(&G, ls, nl, tt); gnd_step(&G, &in, tt, 0.1f, out, sizeof out); } }
  lvl = gnd_alert(&G, &why);
  CHECK(lvl == 1, "telemetry 7 s apart while its packets come through: a warning, not an alarm (%s, the last report outranking 'telemetry slow')", gnd_why_text[why]);
  { uint8_t ls[64]; crsf_link Lk = { -50, 100, 9, -51, 100, 8, 0, 10, 0 };
    for (int k = 0; k < 100; k++) { tt += 0.1; int nl = crsf_link_stats(ls, CRSF_ADDR_HANDSET, &Lk); gnd_from_radio(&G, ls, nl, tt); gnd_step(&G, &in, tt, 0.1f, out, sizeof out); } }
  lvl = gnd_alert(&G, &why);
  CHECK(lvl == 2 && why == 1, "nothing at all for 17 s: an alarm again (%s)", gnd_why_text[why]);
  /* battery low only: a warning, which outlasts its cause by 2 s */
  host(); gnd_init(&G, &H, NULL); tlm_init(&T); tt = 0;
  float sup2[7] = { 0, 0, 2.5f, 0.2f, 4, 10, 100 }, nav2[6] = { 0, 0, 1.5f, 0, 2 | 4, 1 };
  int warned = 0; double cleared = -1;
  for (int k = 0; k < 800; k++) {
    tt += 0.01; if (k == 300) sup2[3] = 0.5f;
    tlm_put(&T, TLM_SUPER, sup2, 7, tt); tlm_put(&T, TLM_NAV, nav2, 6, tt); tlm_put(&T, TLM_ATT, att, 3, tt);
    int n = tlm_service(&T, &tlm_crsf, tt, 400, buf, sizeof buf); gnd_from_radio(&G, buf, n, tt);
    gnd_step(&G, &in, tt, 0.01f, out, sizeof out);
    lvl = gnd_alert(&G, &why); if (k == 250 && lvl == 1 && why == 3) warned = 1;
    if (k > 300 && cleared < 0 && lvl == 0) cleared = tt - 3.0;
  }
  CHECK(warned && cleared > 1.9 && cleared < 3.3, "battery at 20%%: warning; charged again, it clears %.1f s later", cleared);

  printf("if the formulas can't answer\n");
  gnd_init(&G, NULL, NULL); in.held = GB(GB_RIGHT) | GB(GB_ARM);
  t += dt; gnd_step(&G, &in, t, dt, out, sizeof out);
  CHECK(G.ch[RC_ROLL] == 1 && G.ch[RC_ARM] == 1 && !G.shaped, "the raw sticks go up: roll %.0f, arm %.0f", G.ch[RC_ROLL], G.ch[RC_ARM]);

  printf(fails ? "%d FAILED\n" : "all passed\n", fails);
  return fails != 0;
}
