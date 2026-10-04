/* Tests for the command module (ground_core.c) with its built-in program: the sticks and switches as they go up,
 * commands, the drone's telemetry as it comes down, and the alerts; and the text commands (ground_text.c, included
 * here so the build line stays as it was).
 *   cc -O2 -I.. -I../fc -o test_ground test_ground.c ground_core.c rn_builtin_ground.c ../fc/tlm_core.c ../fc/tlm_crsf.c ../fc/crsf.c ../fc/rc_core.c ../rn_host.c ../rn.c -lm && ./test_ground */
#define _DEFAULT_SOURCE
#include "ground_core.h"
#include "ground_text.c"
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
/* the frames in what one step wrote: channels, commands */
static void frames_in(const uint8_t *b, int n, int *rc, int *cmd) {
  crsf_parser P; memset(&P, 0, sizeof P); *rc = *cmd = 0;
  for (int i = 0; i < n; i++) if (crsf_feed(&P, b[i]) > 0) { if (crsf_type(&P) == CRSF_RC) ++*rc; else if (crsf_type(&P) == CRSF_EXT) ++*cmd; }
}
/* the transmitter module's link statistics, as it reports them */
static void link_stats(gnd_state *G, float lq, double t) {
  uint8_t f[64]; crsf_link Lk = { -50, lq, 9, -51, lq, 8, 0, 10, 0 }; int n = crsf_link_stats(f, CRSF_ADDR_HANDSET, &Lk); gnd_from_radio(G, f, n, t);
}
static int text(gnd_state *G, gnd_text_in *I, const char *line, double t, char *reply) { char b[200]; snprintf(b, sizeof b, "%s", line); return gnd_text(G, I, b, t, reply, 300); }

static void more_tests(void);
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
  { /* every stick button, both ways */
    static const int bt[8] = { GB_RIGHT, GB_LEFT, GB_FWD, GB_BACK, GB_UP, GB_DOWN, GB_YAWR, GB_YAWL };
    static const int ax[8] = { GND_ROLL, GND_ROLL, GND_PITCH, GND_PITCH, GND_THR, GND_THR, GND_YAW, GND_YAW }; float got[8]; int ok = 1;
    for (int b = 0; b < 8; b++) {
      in.held = GB(bt[b]); for (int k = 0; k < 80; k++) { t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); }
      got[b] = G.ch[ax[b]]; ok &= fabsf(got[b] - (b % 2 ? -1.0f : 1.0f)) < 1e-3f;
      in.held = 0; for (int k = 0; k < 40; k++) { t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); }
    }
    CHECK(ok, "each stick button, each way: right %+.0f left %+.0f fwd %+.0f back %+.0f up %+.0f down %+.0f yaw right %+.0f yaw left %+.0f",
          got[0], got[1], got[2], got[3], got[4], got[5], got[6], got[7]);
  }
  in.has_axis = 1u << GND_ROLL; in.axis[GND_ROLL] = 0.02f; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); float d0 = G.stick[GND_ROLL];
  in.axis[GND_ROLL] = 0.5f; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); float d5 = G.stick[GND_ROLL];
  in.axis[GND_ROLL] = -1; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); float d1 = G.stick[GND_ROLL];
  CHECK(d0 == 0 && fabsf(d5 - 0.368f) < 0.01f && d1 == -1, "a real stick: deadband (0.02 → %.2f), expo (0.5 → %.3f), full (−1 → %.2f)", d0, d5, d1);
  in.has_axis = 0;
  gnd_config c; gnd_config_default(&c); c.latch = GB(GB_ARM) | GB(GB_FLY);
  host(); gnd_init(&G, &H, &c);
  in.held = 0; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out);
  float arm[4]; const uint32_t seq[4] = { GB(GB_ARM), 0, GB(GB_ARM), 0 };
  for (int k = 0; k < 4; k++) { in.held = seq[k]; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); arm[k] = G.ch[RC_ARM]; }
  CHECK(arm[0] == 1 && arm[1] == 1 && arm[2] == -1 && arm[3] == -1, "a push button as a latching arm switch: press on, release stays, press off (%.0f %.0f %.0f %.0f)", arm[0], arm[1], arm[2], arm[3]);
  in.held = GB(GB_SPORT); t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); float lv = G.ch[RC_LEVEL]; in.held = 0;
  CHECK(lv == 1, "the sport button sets the speed level channel to +1");
  { float a[3]; in.sw = GB(GB_ARM); t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); a[0] = G.ch[RC_ARM];
    t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); a[1] = G.ch[RC_ARM];
    in.sw = 0; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); a[2] = G.ch[RC_ARM];
    CHECK(a[0] == 1 && a[1] == 1 && a[2] == -1, "a switch's state (a toggle, \"press arm\") doesn't latch: on while on, off when off (%.0f %.0f %.0f)", a[0], a[1], a[2]); }

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

  { int gotos = 0; float lastx = 0; uint32_t s0 = RI.cmd_seq;
    gnd_goto(&G, 1, 0, 2, 0); gnd_goto(&G, 2, 0, 2, 0); gnd_goto(&G, 7, 0, 2, 0);   /* a dragged target: three before a step */
    for (int k = 0; k < 150; k++) { t += dt; int n = gnd_step(&G, &in, t, dt, out, sizeof out); uint32_t sb = RI.cmd_seq; drone_reads(out, n, t); if (RI.cmd_seq != sb && RI.cmd == RC_CMD_GOTO) { gotos++; lastx = RI.cmd_v[0]; } }
    CHECK(gotos == 1 && fabsf(lastx - 7) < 0.01f && RI.cmd_seq != s0, "three go-tos while one waits: one goes, the latest (x %.1f; %d sent)", lastx, gotos); }
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

  printf("what isn't the drone's\n");
  { host(); gnd_init(&G, &H, NULL); uint8_t fr[64], pl[8] = { 0xEA, 0xEE, 0, 0, 0, 0, 0, 0 };
    int n = crsf_frame(fr, CRSF_ADDR_HANDSET, 0x3A, pl, 8); gnd_from_radio(&G, fr, n, 1.0);
    crsf_link Lk = { -50, 100, 9, -51, 100, 8, 0, 10, 0 }; n = crsf_link_stats(fr, CRSF_ADDR_HANDSET, &Lk); gnd_from_radio(&G, fr, n, 1.0);
    float ch[16] = { 0 }; n = crsf_rc(fr, CRSF_ADDR_FC, ch); gnd_from_radio(&G, fr, n, 1.0);
    CHECK(G.V.frames == 0 && G.V.t_any < 0 && G.V.t_link > 0, "the module's own frames (sync, link statistics, our echo): no telemetry counted (%u frames)", G.V.frames); }
  printf("before the module first connects\n");
  { host(); gnd_init(&G, &H, NULL); memset(&RI, 0, sizeof RI); memset(&RP, 0, sizeof RP); memset(&in, 0, sizeof in);
    uint8_t ls[64]; crsf_link L0 = { -130, 0, 0, -130, 0, 0, 0, 10, 0 }; double tt = 0; int sent = 0, weak = 0;
    gnd_goto(&G, 2, 0, 1.5f, 0);
    for (int k = 0; k < 125; k++) { tt += 0.004; if (k % 25 == 0) { int nl = crsf_link_stats(ls, CRSF_ADDR_HANDSET, &L0); gnd_from_radio(&G, ls, nl, tt); }
      uint32_t s0 = RI.cmd_seq; int n = gnd_step(&G, &in, tt, 0.004f, out, sizeof out); drone_reads(out, n, tt); if (RI.cmd_seq != s0) sent++;
      int why; if (gnd_alert(&G, &why) && why == 2) weak = 1; }
    CHECK(!sent && !weak, "the module reports LQ 0 (not connected yet): the go-to waits, and that's no weak link");
    crsf_link L1 = { -60, 100, 9, -61, 100, 8, 0, 10, 0 }; int nl = crsf_link_stats(ls, CRSF_ADDR_HANDSET, &L1); gnd_from_radio(&G, ls, nl, tt);
    for (int k = 0; k < 50; k++) { tt += 0.004; uint32_t s0 = RI.cmd_seq; int n = gnd_step(&G, &in, tt, 0.004f, out, sizeof out); drone_reads(out, n, tt); if (RI.cmd_seq != s0 && RI.cmd == RC_CMD_GOTO) sent++; }
    CHECK(sent == 1, "connected: it goes (%d sent)", sent); }
  printf("if the formulas can't answer\n");
  gnd_init(&G, NULL, NULL); in.held = 0; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); in.held = GB(GB_RIGHT) | GB(GB_ARM);
  t += dt; gnd_step(&G, &in, t, dt, out, sizeof out);
  CHECK(G.ch[RC_ROLL] == 1 && G.ch[RC_ARM] == 1 && !G.shaped, "the raw sticks go up: roll %.0f, arm %.0f", G.ch[RC_ROLL], G.ch[RC_ARM]);

  more_tests();
  printf(fails ? "%d FAILED\n" : "all passed\n", fails);
  return fails != 0;
}

/* ── what the review found: bad numbers, switches at the start, one frame a beat, commands and the link ── */
static void more_tests(void) {
  gnd_state G; gnd_text_in I; uint8_t out[256]; gnd_input in; char r[300]; double t = 100; const float dt = 0.004f; gnd_config c;
#define STEP() do { t += dt; memset(&in, 0, sizeof in); in.held = hw; gnd_text_inputs(&I, t, &in); n = gnd_step(&G, &in, t, dt, out, sizeof out); } while (0)
  uint32_t hw = 0; int n;
  printf("numbers that aren't\n");
  host(); gnd_init(&G, &H, NULL); memset(&I, 0, sizeof I); STEP();
  text(&G, &I, "stick roll nan", t, r); int refused = strstr(r, "-1 to 1") != 0 && !(I.has & 1);
  text(&G, &I, "stick roll inf", t, r); refused &= !(I.has & 1); text(&G, &I, "stick roll 2", t, r); refused &= !(I.has & 1);
  text(&G, &I, "stick roll 1e999", t, r); refused &= !(I.has & 1); text(&G, &I, "stick roll 0.5x", t, r); refused &= !(I.has & 1);
  CHECK(refused, "stick roll nan, inf, 2, 1e999, 0.5x: refused, and said why (%s)", r);
  memset(&in, 0, sizeof in); in.has_axis = 1u << GND_ROLL; in.axis[GND_ROLL] = NAN;
  for (int k = 0; k < 20; k++) { t += dt; gnd_step(&G, &in, t, dt, out, sizeof out); }
  float nan_ch = G.ch[RC_ROLL]; int nan_shaped = G.shaped;
  memset(&RI, 0, sizeof RI); memset(&RP, 0, sizeof RP); t += dt; drone_reads(out, gnd_step(&G, &in, t, dt, out, sizeof out), t);
  in.axis[GND_ROLL] = 0.5f; t += dt; gnd_step(&G, &in, t, dt, out, sizeof out);
  CHECK(nan_ch == 0 && nan_shaped && RI.ch[RC_ROLL] == 0 && fabsf(G.stick[GND_ROLL] - 0.368f) < 0.01f,
        "a stick that reads NaN counts as no stick: roll %.2f (the drone reads %.2f); then 0.5 → %.3f (stickInput's memory unharmed)", nan_ch, RI.ch[RC_ROLL], G.stick[GND_ROLL]);
  { float ch[16]; for (int i = 0; i < 16; i++) ch[i] = NAN; uint8_t f[64]; crsf_rc(f, CRSF_ADDR_TX, ch); float back[16]; crsf_rc_read(f + 3, back);
    CHECK(back[0] == 0 && back[15] == 0, "crsf_rc: NaN goes as the centre (%.2f), not full deflection", back[0]); }
  host(); gnd_init(&G, &H, NULL); memset(&I, 0, sizeof I); STEP();
  text(&G, &I, "goto 500 0 2", t, r); int bad = G.qn == 0 && strstr(r, "not sent");
  text(&G, &I, "goto home 0 0", t, r); bad &= G.qn == 0 && strstr(r, "not sent") != 0;
  text(&G, &I, "goto 1 2 nan", t, r); bad &= G.qn == 0;
  text(&G, &I, "goto 1 2 3 inf", t, r); bad &= G.qn == 0;
  CHECK(bad, "goto 500 0 2, goto home 0 0, NaN, an infinite heading: refused, nothing queued (%s)", r);
  text(&G, &I, "cmd 257 1", t, r); bad = G.qn == 0 && strstr(r, "not sent");
  text(&G, &I, "cmd 0", t, r); bad &= G.qn == 0; text(&G, &I, "cmd 2x 1", t, r); bad &= G.qn == 0;
  text(&G, &I, "cmd 2 1 2 3 4 5 6 7", t, r); bad &= G.qn == 0; text(&G, &I, "cmd 2 40000", t, r); bad &= G.qn == 0;
  text(&G, &I, "cmd 2 1", t, r); bad &= G.qn == 1 && strstr(r, "queued") != 0;
  CHECK(bad, "cmd 257, 0, 2x, seven values, a value its 16 bits can't hold: refused; cmd 2 1: %s", r);
  { float v[4] = { 1, 2, 3, 0 }, big[1] = { 40000 }; int a = gnd_goto(&G, NAN, 0, 2, 0), b = gnd_goto(&G, 400, 0, 2, 0), cc = gnd_goto(&G, 1, 2, 3, INFINITY);
    int d = gnd_command(&G, 300, v, 1), e = gnd_command(&G, RC_CMD_GOTO, (float[]){ 1000, 0, 2, 0 }, 4), f = gnd_command(&G, 2, big, 1), g = gnd_command(&G, 2, v, 7);
    CHECK(a == -2 && b == -2 && cc == -2 && d == -2 && e == -2 && f == -2 && g == -2 && G.qn == 1, "and the core refuses them too (−2), as the simulator calls it directly: %d %d %d %d %d %d %d", a, b, cc, d, e, f, g); }
  {   /* the cargo's latches: latch N|all open|close|toggle, N from 1 (the command carries it from 0, −1 for all) */
    int q0 = G.qn;
    text(&G, &I, "latch 9 open", t, r); int bad2 = G.qn == q0 && strstr(r, "not sent");
    text(&G, &I, "latch 1 wiggle", t, r); bad2 &= G.qn == q0; text(&G, &I, "latch 1.5 open", t, r); bad2 &= G.qn == q0; text(&G, &I, "latch 0 open", t, r); bad2 &= G.qn == q0;
    text(&G, &I, "latch 2 open", t, r); int k = (G.qh + G.qn - 1) % GND_QN;
    int ok = G.qn == q0 + 1 && G.q[k].cmd == RC_CMD_LATCH && G.q[k].n == 2 && G.q[k].v[0] == 1 && G.q[k].v[1] == 0;
    text(&G, &I, "latch all toggle", t, r); k = (G.qh + G.qn - 1) % GND_QN;
    ok &= G.qn == q0 + 2 && G.q[k].v[0] == -1 && G.q[k].v[1] == 2;
    CHECK(bad2 && ok, "latch 9, wiggle, 1.5, 0: refused; latch 2 open, latch all toggle: queued as LATCH 1 0, −1 2 (%s)", r);
  }

  printf("switches at the start\n");
  gnd_config_default(&c); c.latch = GB(GB_ARM) | GB(GB_FLY);
  host(); gnd_init(&G, &H, &c); memset(&I, 0, sizeof I); hw = GB(GB_ARM) | GB(GB_FLY); STEP(); STEP();
  float a0 = G.ch[RC_ARM], f0 = G.ch[RC_FLY]; hw = 0; STEP(); hw = GB(GB_ARM); STEP();
  CHECK(a0 == -1 && f0 == -1 && G.ch[RC_ARM] == 1 && G.ch[RC_FLY] == -1, "latching buttons held at power-on: no press, nothing latched (arm %.0f, fly %.0f); let go and pressed: arm %.0f", a0, f0, G.ch[RC_ARM]);
  host(); gnd_init(&G, &H, NULL); memset(&I, 0, sizeof I); hw = GB(GB_ARM); STEP();
  for (int k = 0; k < 30; k++) STEP();
  int why, lvl = gnd_alert(&G, &why); float a1 = G.ch[RC_ARM];
  hw = 0; STEP(); hw = GB(GB_ARM); for (int k = 0; k < 30; k++) STEP(); int why2; gnd_alert(&G, &why2);
  CHECK(a1 == -1 && lvl >= 1 && why == GND_WHY_SWITCH && G.ch[RC_ARM] == 1 && why2 != GND_WHY_SWITCH,
        "an arm switch on at the start: not sent (arm %.0f), warning \"%s\"; off, then on: armed (%.0f)", a1, gnd_why_text[why], G.ch[RC_ARM]);
  host(); gnd_init(&G, &H, NULL); memset(&I, 0, sizeof I); I.held = GB(GB_FLY); hw = 0; STEP(); STEP(); float fl = G.ch[RC_FLY];
  text(&G, &I, "release fly", t, r); STEP(); text(&G, &I, "press fly", t, r); STEP();
  CHECK(fl == -1 && G.ch[RC_FLY] == 1, "the same for a switch set by text before the start (fly %.0f, then %.0f)", fl, G.ch[RC_FLY]);
  gnd_config_default(&c); c.latch = GB(GB_FLY); c.resume = GB(GB_ARM) | GB(GB_FLY);
  host(); gnd_init(&G, &H, &c); memset(&I, 0, sizeof I); hw = GB(GB_ARM); STEP(); gnd_alert(&G, &why);
  CHECK(G.ch[RC_ARM] == 1 && G.ch[RC_FLY] == 1 && why != GND_WHY_SWITCH, "back from a restart in flight (resume): the toggle still on and the latched fly as they were (arm %.0f, fly %.0f)", G.ch[RC_ARM], G.ch[RC_FLY]);

  printf("one frame a beat\n");
  host(); gnd_init(&G, &H, NULL); memset(&I, 0, sizeof I); hw = 0; memset(&RI, 0, sizeof RI); memset(&RP, 0, sizeof RP);
  { int both = 0, cmds = 0, rcs = 0, empty = 0, rc, cm; float v = 1;
    for (int k = 0; k < 4; k++) gnd_command(&G, RC_CMD_LEARN, &v, 1);
    gnd_goto(&G, 1, 2, 3, 0);
    for (int k = 0; k < 250; k++) { STEP(); drone_reads(out, n, t); frames_in(out, n, &rc, &cm); both += rc && cm; cmds += cm; rcs += rc; empty += !rc && !cm; }
    CHECK(!both && cmds == 5 && rcs == 245 && !empty && RI.cmd == RC_CMD_GOTO, "commands go instead of the channels, never with them: %d commands, %d channel frames in 250 beats, %d with both", cmds, rcs, both); }

  printf("commands and the link\n");
  host(); gnd_init(&G, &H, NULL); memset(&I, 0, sizeof I); hw = 0; memset(&RI, 0, sizeof RI); memset(&RP, 0, sizeof RP);
  { int rc, cm, sent = 0; double tl = t;
    link_stats(&G, 100, t); STEP();
    for (int k = 0; k < 400; k++) { STEP(); frames_in(out, n, &rc, &cm); sent += cm; }   /* 1.6 s with no statistics */
    gnd_goto(&G, 1, 0, 2, 0); text(&G, &I, "goto 2 0 2", t, r);
    for (int k = 0; k < 100; k++) { STEP(); frames_in(out, n, &rc, &cm); sent += cm; }
    int stale = sent; char said[300]; snprintf(said, sizeof said, "%s", r);
    for (int k = 0; k < 100; k++) { if (k % 25 == 0) link_stats(&G, 0, t); STEP(); frames_in(out, n, &rc, &cm); sent += cm; }
    int lq0 = sent - stale; text(&G, &I, "status", t, r);
    CHECK(!stale && !lq0 && G.qn == 1 && strstr(said, "once the link is back") && strstr(r, "1 command waiting for the link"),
          "link statistics stale, then uplink LQ 0: the go-to waits (%d sent, %d queued: \"%s\"; status: …%s)", stale + lq0, G.qn, said, strstr(r, " | 1 command") ? strstr(r, " | 1 command") : r);
    for (int k = 0; k < 100; k++) { if (k % 25 == 0) link_stats(&G, 90, t); STEP(); drone_reads(out, n, t); frames_in(out, n, &rc, &cm); sent += cm; }
    CHECK(sent == 1 && RI.cmd == RC_CMD_GOTO && fabsf(RI.cmd_v[0] - 2) < 0.01f, "the link back: it goes, once, the latest target (x %.1f)", RI.cmd_v[0]);
    for (int k = 0; k < 300; k++) STEP();                                                 /* (the statistics go stale) */
    float v = 1; gnd_command(&G, RC_CMD_LEARN, &v, 1); sent = 0; uint32_t d0 = G.dropped;
    for (int k = 0; k < 2600; k++) { STEP(); frames_in(out, n, &rc, &cm); sent += cm; }   /* 10.4 s: no statistics */
    int qn = G.qn;
    for (int k = 0; k < 100; k++) { if (k % 25 == 0) link_stats(&G, 100, t); STEP(); frames_in(out, n, &rc, &cm); sent += cm; }
    CHECK(!sent && !qn && G.dropped == d0 + 1 && strstr(G.why, "dropped"), "a command that waited over %.0f s for the link is dropped, not sent late (%s)", GND_CMD_WAIT, G.why);
    host(); gnd_init(&G, &H, NULL); memset(&I, 0, sizeof I); STEP(); gnd_goto(&G, 1, 0, 2, 0); sent = 0;
    for (int k = 0; k < 100; k++) { STEP(); frames_in(out, n, &rc, &cm); sent += cm; }
    CHECK(sent == 1 && tl < t, "no statistics ever (no module that sends them): commands go as before"); }

  printf("text commands and latched switches\n");
  gnd_config_default(&c); c.latch = GB(GB_ARM) | GB(GB_FLY);
  host(); gnd_init(&G, &H, &c); memset(&I, 0, sizeof I); hw = 0; STEP(); hw = GB(GB_ARM); STEP(); hw = 0; STEP();
  { float on = G.ch[RC_ARM]; text(&G, &I, "release arm", t, r); STEP(); float off = G.ch[RC_ARM]; char r1[300]; snprintf(r1, sizeof r1, "%s", r);
    text(&G, &I, "press arm", t, r); STEP(); float on2 = G.ch[RC_ARM]; char r2[300]; snprintf(r2, sizeof r2, "%s", r);
    text(&G, &I, "tap arm", t, r); STEP();
    CHECK(on == 1 && off == -1 && on2 == 1 && G.ch[RC_ARM] == -1 && !strcmp(r1, "arm off") && !strcmp(r2, "arm on") && !strcmp(r, "arm off"),
          "a button-latched arm: \"release arm\" disarms (%s), \"press arm\" arms (%s), \"tap arm\" toggles (%s)", r1, r2, r); }
  host(); gnd_init(&G, &H, NULL); memset(&I, 0, sizeof I); hw = 0; STEP(); hw = GB(GB_ARM); STEP();
  text(&G, &I, "release arm", t, r);
  CHECK(strstr(r, "a switch holds it on") != 0, "a hardware toggle that is on: release says so (%s)", r);

  printf("momentary buttons by text\n");
  host(); gnd_init(&G, &H, NULL); memset(&I, 0, sizeof I); hw = 0; STEP(); memset(&RI, 0, sizeof RI); memset(&RP, 0, sizeof RP);
  { int learns = 0, rc, cm; float hold_on = 0, hold_later = 0;
    for (int k = 0; k < 300; k++) {
      if (k == 10 || k == 150) text(&G, &I, "press cal", t, r);
      if (k == 10) text(&G, &I, "press hold", t, r);
      uint32_t s0 = RI.cmd_seq; STEP(); drone_reads(out, n, t); frames_in(out, n, &rc, &cm); if (RI.cmd_seq != s0 && RI.cmd == RC_CMD_LEARN) learns++;
      if (k == 20) hold_on = G.ch[RC_HOLD];
      if (k == 140) hold_later = G.ch[RC_HOLD];
    }
    CHECK(learns == 2 && hold_on == 1 && hold_later == -1, "press cal twice: two calibrations (%d); press hold: a tap (on %.0f, then %.0f)", learns, hold_on, hold_later); }

  printf("the climb\n");
  host(); gnd_init(&G, &H, NULL);
  { uint8_t f[64]; int m = crsf_vario(f, 1.5f); gnd_from_radio(&G, f, m, 5.0); float p[256]; gnd_view_pack(&G, 5.5, p, 256);
    CHECK(G.V.t_vz == 5.0 && G.V.t_baro < 0 && p[34] == -1 && p[36] == 1.5f, "a vario frame: climb %.1f with its own time; the height's age still 'never' (%.0f)", p[36], p[34]);
    uint8_t b[64]; b[0] = 0x27; b[1] = 0x24; m = crsf_frame(f, CRSF_ADDR_FC, CRSF_BARO_ALT, b, 2); gnd_from_radio(&G, f, m, 9.0); gnd_view_pack(&G, 9.5, p, 256);
    float a1 = p[34]; m = crsf_vario(f, -0.5f); gnd_from_radio(&G, f, m, 9.2); gnd_view_pack(&G, 9.5, p, 256);
    CHECK(fabsf(a1 - 4.5f) < 1e-3f && fabsf(p[34] - 0.5f) < 1e-3f && fabsf(p[35] - 2.0f) < 1e-3f && p[36] == -0.5f,
          "heights without a climb, the climb in vario frames: the age is the older one (%.1f s while the vario stopped; %.1f s with both coming)", a1, p[34]); }
#undef STEP
}
