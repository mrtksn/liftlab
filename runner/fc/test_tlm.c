/* Tests for the telemetry interface, the CRSF transport and the pilot's radio (tlm_core, tlm_crsf, crsf, rc_core).
 *   cc -O2 -I.. -o test_tlm test_tlm.c tlm_core.c tlm_crsf.c tlm_sources.c crsf.c rc_core.c -lm && ./test_tlm */
#include "tlm_sources.h"
#include "tlm_crsf.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

/* read back what tlm_service produced, as the ground would */
typedef struct { int frames, att, batt, gps, mode, text, item, bad; float roll, volts, amps; int pct; double lat, lon; char mode_s[20], text_s[64]; int item_id; float item_v[16]; } ground;
static void ground_read(ground *G, const uint8_t *p, int n) {
  crsf_parser P; memset(&P, 0, sizeof P);
  for (int i = 0; i < n; i++) {
    int len = crsf_feed(&P, p[i]); if (len < 0) { G->bad++; continue; } if (!len) continue;
    G->frames++; const uint8_t *q = crsf_payload(&P); int m = crsf_payload_len(&P);
    switch (crsf_type(&P)) {
      case CRSF_ATTITUDE: G->att++; G->roll = (int16_t)((q[2] << 8) | q[3]) / 10000.0f; break;
      case CRSF_BATTERY: G->batt++; G->volts = ((q[0] << 8) | q[1]) / 10.0f; G->amps = ((q[2] << 8) | q[3]) / 10.0f; G->pct = q[7]; break;
      case CRSF_GPS: G->gps++; G->lat = (int32_t)((uint32_t)q[0] << 24 | q[1] << 16 | q[2] << 8 | q[3]) * 1e-7; G->lon = (int32_t)((uint32_t)q[4] << 24 | q[5] << 16 | q[6] << 8 | q[7]) * 1e-7; break;
      case CRSF_FLIGHT_MODE: G->mode++; snprintf(G->mode_s, sizeof G->mode_s, "%s", (const char *)q); break;
      case CRSF_EXT:
        if (q[0] == CRSF_EXT_TEXT) { G->text++; snprintf(G->text_s, sizeof G->text_s, "%s", (const char *)q + 2); }
        else if (q[0] == CRSF_EXT_ITEM) { G->item++; G->item_id = q[1]; for (int k = 0; k < q[2] && k < 16; k++) G->item_v[k] = (int16_t)((q[3 + 2 * k] << 8) | q[4 + 2 * k]) / tlm_scale(q[1], k); }
        break;
    }
    (void)m;
  }
}

int main(void) {
  printf("CRSF frames\n");
  {
    uint8_t f[64]; float ch[16], back[16];
    for (int i = 0; i < 16; i++) ch[i] = -1 + i / 7.5f;
    int n = crsf_rc(f, CRSF_ADDR_FC, ch);
    crsf_parser P; memset(&P, 0, sizeof P); int got = 0;
    for (int i = 0; i < n; i++) got = crsf_feed(&P, f[i]);
    crsf_rc_read(crsf_payload(&P), back); float err = 0; for (int i = 0; i < 16; i++) err = fmaxf(err, fabsf(back[i] - ch[i]));
    CHECK(n == 26 && got == 26 && crsf_type(&P) == CRSF_RC && err < 0.002f, "RC channels: 26-byte frame, 16 channels back within %.4f", err);
    f[10] ^= 0x10; memset(&P, 0, sizeof P); got = 0; for (int i = 0; i < n; i++) { int r = crsf_feed(&P, f[i]); if (r) got = r; }
    CHECK(got == -1, "a flipped bit fails the CRC");
    crsf_link L = { -71, 98, 9, -80, 95, 6, 3, 100, 0 }, L2; n = crsf_link_stats(f, CRSF_ADDR_FC, &L);
    memset(&P, 0, sizeof P); for (int i = 0; i < n; i++) crsf_feed(&P, f[i]); crsf_link_stats_read(crsf_payload(&P), crsf_payload_len(&P), &L2);
    CHECK(L2.up_rssi == -71 && L2.up_lq == 98 && L2.down_rssi == -80 && L2.tx_power_mw == 100, "link statistics round trip (%.0f dBm, %.0f%%, %d mW)", L2.up_rssi, L2.up_lq, L2.tx_power_mw);
  }

  printf("the store and the scheduler\n");
  {
    static tlm_store T; tlm_init(&T); static tlm_watch W; tlm_watch_init(&W);
    float att[3] = { 0.2f, -0.1f, 1.0f }, b[1] = { 15.2f }, st[2] = { 1, 1 | 2 | 4 };
    float sup[7] = { 0, 0, 2.5f, 0.62f, 4, 12.5f, 410 };
    tlm_put(&T, TLM_ATT, att, 3, 0); tlm_put(&T, TLM_BATT, b, 1, 0); tlm_put(&T, TLM_STATE, st, 2, 0); tlm_put(&T, TLM_SUPER, sup, 7, 0);
    tlm_from_gps(&T, 41.0123456, 29.0234567, 120, 3, 90, 12, 0);
    tlm_text(&T, 6, "hello from the drone");
    ground G; memset(&G, 0, sizeof G); uint8_t out[512]; int total = 0;
    for (int k = 0; k <= 100; k++) { double t = k * 0.01; att[0] = 0.2f + 0.001f * k; tlm_put(&T, TLM_ATT, att, 3, t); int n = tlm_service(&T, &tlm_crsf, t, 300, out, sizeof out); total += n; ground_read(&G, out, n); }
    CHECK(G.bad == 0 && total <= 300 * 1.0 + 64 + 75, "1 s at 300 B/s: %d bytes in %d frames", total, G.frames);
    CHECK(G.att >= 5 && G.batt >= 1 && G.gps >= 1 && G.mode >= 1 && G.text == 1, "attitude %d×, battery %d×, GPS %d×, mode %d×, message %d×", G.att, G.batt, G.gps, G.mode, G.text);
    CHECK(fabs(G.lat - 41.0123456) < 2e-7 && fabs(G.lon - 29.0234567) < 2e-7, "GPS to 1e-7°: %.7f %.7f", G.lat, G.lon);
    CHECK(fabsf(G.volts - 15.2f) < 0.06f && fabsf(G.amps - 12.5f) < 0.06f && G.pct == 62, "battery: %.1f V, %.1f A, %d%% (charge and current from the supervisor)", G.volts, G.amps, G.pct);
    CHECK(!strcmp(G.mode_s, "POSHOLD") || !strcmp(G.mode_s, "READY"), "flight mode: %s", G.mode_s);
    CHECK(!strcmp(G.text_s, "hello from the drone"), "message: \"%s\"", G.text_s);
    /* a slow link: nothing over budget, the important things first */
    tlm_init(&T); memset(&G, 0, sizeof G); total = 0;
    float mot[5] = { 4, 0.5f, 0.5f, 0.5f, 0.5f };
    for (int k = 0; k <= 200; k++) { double t = k * 0.01; tlm_put(&T, TLM_ATT, att, 3, t); tlm_put(&T, TLM_BATT, b, 1, t); tlm_put(&T, TLM_STATE, st, 2, t); tlm_put(&T, TLM_MOTORS, mot, 5, t);
      int n = tlm_service(&T, &tlm_crsf, t, 40, out, sizeof out); total += n; ground_read(&G, out, n); }
    CHECK(total <= 40 * 2 + 64 + 10, "2 s at 40 B/s: %d bytes (attitude %d×, battery %d×, motors item %d×)", total, G.att, G.batt, G.item);
    /* between boards */
    static tlm_store A, B; tlm_init(&A); tlm_init(&B);
    float pos[6] = { 1.25f, -2.5f, 3, 0.1f, 0, -0.2f }; tlm_put(&A, TLM_POS, pos, 6, 1); tlm_text(&A, 4, "from the Pi");
    float buf[256]; int n = tlm_pack(&A, buf, 256); tlm_unpack(&B, buf, n, 1.0);
    const tlm_slot *s = tlm_get(&B, TLM_POS);
    CHECK(s && s->n == 6 && s->v[1] == -2.5f && B.qn == 1 && !strcmp(B.q[B.qh].s, "from the Pi") && tlm_pack(&A, buf, 256) == 0, "packed on one board, unpacked on another (%d floats), sent once", n);
  }

  printf("the pilot's radio\n");
  {
    rc_input in; memset(&in, 0, sizeof in); fc_cmd c;
    CHECK(rc_stick_cmd(&in, 0, &c) == -1, "no channels yet: no stick command (the flight core's failsafe stands)");
    uint8_t f[64]; crsf_parser P; memset(&P, 0, sizeof P);
    float ch[16] = { 0.2f, -0.3f, 0.0f, 0.5f, 1, 0, -1, -1, -1 };
    int n = crsf_rc(f, CRSF_ADDR_FC, ch); for (int i = 0; i < n; i++) tlm_crsf_input(&P, f[i], &in, 1.0);
    CHECK(rc_stick_cmd(&in, 1.05, &c) == 0 && c.arm && fabsf(c.roll - 0.2f) < 0.01f && fabsf(c.throttle - 0.5f) < 0.01f && fabsf(c.yaw + 0.5f) < 0.01f, "angle mode: armed, roll %.2f, throttle %.2f, yaw %.2f (right stick turns right)", c.roll, c.throttle, c.yaw);
    CHECK(rc_stick_cmd(&in, 2.1, &c) == -1, "1.1 s without channels: nothing sent");
    float go[4] = { 3, -2, 4.5f, 1.0f }; n = tlm_crsf_cmd(f, RC_CMD_GOTO, 7, go, 4); for (int i = 0; i < n; i++) tlm_crsf_input(&P, f[i], &in, 2.2);
    CHECK(in.cmd == RC_CMD_GOTO && in.cmd_seq == 7 && fabsf(in.cmd_v[2] - 4.5f) < 0.01f && fabsf(in.cmd_v[3] - 1) < 0.002f, "ground-station go-to: (%.2f %.2f %.2f) heading %.3f", in.cmd_v[0], in.cmd_v[1], in.cmd_v[2], in.cmd_v[3]);
    /* the navigation's set point */
    static nav_state N; memset(&N, 0, sizeof N); nav_out o; memset(&o, 0, sizeof o); o.have_home = 1; o.p[2] = 2;
    rc_pilot RP; rc_pilot_init(&RP); nav_sp sp; memset(&sp, 0, sizeof sp);
    float c2[16] = { 0, 1, 0, 0, 1, 0, 1, -1, -1 }; double t = 3;   /* armed, fly, pitch full forward, normal level */
    for (int k = 0; k < 100; k++, t += 0.01) { memcpy(in.ch, c2, sizeof c2); in.t_ch = t; in.frames++; rc_pilot_step(&RP, &in, t, &N, &o, 0.01f, &sp); }
    { float vn = sqrtf(sp.vref[0] * sp.vref[0] + sp.vref[1] * sp.vref[1]), ang = atan2f(sp.vref[1], sp.vref[0]);
      CHECK(RP.arm && sp.fly && fabsf(vn - 3) < 0.05f && fabsf(ang - 1) < 0.02f && sp.target[0] > 3.9f && sp.target[1] > -0.9f, "the go-to, then 1 s of full forward: %.2f m/s along the heading (1.00 rad: %.2f), target (%.2f %.2f)", vn, ang, sp.target[0], sp.target[1]); }
    for (int k = 0; k < 150; k++, t += 0.01) rc_pilot_step(&RP, &in, t, &N, &o, 0.01f, &sp);   /* the channels stop */
    CHECK(RP.lost && N.rc_rth && sp.vref[0] == 0, "channels stop in flight: after 1 s, lost, the navigation flies home (%s)", RP.msg);
    for (int k = 0; k < 5; k++, t += 0.01) { in.t_ch = t; in.frames++; c2[RC_PITCH] = 0; memcpy(in.ch, c2, sizeof c2); rc_pilot_step(&RP, &in, t, &N, &o, 0.01f, &sp); }
    CHECK(!RP.lost && !N.rc_rth && fabsf(sp.target[2] - 2) < 0.01f, "back: holds where it is (%s)", RP.msg);
  }
  printf(fails ? "%d FAILED\n" : "all passed\n", fails);
  return fails != 0;
}
