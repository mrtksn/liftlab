/* Tests for the cargo task (cargo_core.c): commands, the latches' travel, the load switches and what they say, the
 * radio's LATCH command through CRSF (as the ground sends it and the receiver's board reads it), and its telemetry
 * item.
 *   cc -O2 -I.. -o test_cargo test_cargo.c cargo_core.c tlm_core.c tlm_crsf.c tlm_sources.c crsf.c rc_core.c -lm */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "cargo_core.h"
#include "rc_core.h"
#include "tlm_sources.h"
#include "tlm_crsf.h"
#include "crsf.h"
#include "pickup_core.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void run(cargo_state *C, float s) { for (float t = 0; t < s; t += 0.02f) cargo_step(C, 0.02f); }

int main(void) {
  cargo_state C;
  float tr[3] = { 0.2f, 0, 0.5f };
  cargo_init(&C, 3, 0x5, tr);                                     /* latches 1 and 3 closed, 2 open */
  CHECK(cargo_drive(&C) == 0x5, "drive at start %x", cargo_drive(&C));
  CHECK(C.travel[1] == 0.15f, "default travel %g", C.travel[1]);

  /* commands */
  CHECK(cargo_command(&C, 0, CG_OPEN, "text") == 0 && cargo_drive(&C) == 0x4, "open latch 1: %x", cargo_drive(&C));
  CHECK(!strcmp(C.msg, "latch 1 opening (text)"), "message '%s'", C.msg);
  CHECK(cargo_bits(&C, 0) & 4, "latch 1 moving");
  run(&C, 0.3f);
  CHECK(!(cargo_bits(&C, 0) & 4), "latch 1 has got there");
  CHECK(!strcmp(C.msg, "latch 1 open"), "no switch: just where it went: '%s'", C.msg);
  CHECK(cargo_command(&C, 1, CG_TOGGLE, 0) == 0 && cargo_drive(&C) == 0x6, "toggle latch 2: %x", cargo_drive(&C));
  CHECK(!strcmp(C.msg, "latch 2 closing"), "toggle says which way: '%s'", C.msg);
  CHECK(cargo_command(&C, CG_ALL, CG_CLOSE, 0) == 0 && cargo_drive(&C) == 0x7, "close all: %x", cargo_drive(&C));
  CHECK(cargo_command(&C, 3, CG_OPEN, 0) == -1 && cargo_drive(&C) == 0x7, "no latch 4");
  CHECK(cargo_command(&C, -2, CG_OPEN, 0) == -1, "no latch −2");
  CHECK(cargo_command(&C, 0, 7, 0) == -2 && cargo_drive(&C) == 0x7, "no action 7");
  cargo_state E; cargo_init(&E, 0, 0, 0);
  CHECK(cargo_command(&E, CG_ALL, CG_OPEN, 0) == -1, "no latches at all");
  cargo_init(&E, 20, 0xFFFFF, 0); CHECK(E.n == CG_MAX && cargo_drive(&E) == 0xFF, "at most %d latches", CG_MAX);

  /* load switches: a release that drops its load, one that doesn't (stuck), a close on nothing and on a load */
  cargo_init(&C, 2, 0x3, 0);
  cargo_switches(&C, 0x3, 0x3);
  cargo_command(&C, 0, CG_OPEN, "radio"); cargo_switches(&C, 0x2, 0x3); run(&C, 0.3f);
  CHECK(!strcmp(C.msg, "latch 1 open: load released"), "released: '%s'", C.msg);
  cargo_command(&C, 1, CG_OPEN, 0); run(&C, 0.3f);
  CHECK(!strcmp(C.msg, "latch 2 open but still loaded: stuck?"), "stuck: '%s'", C.msg);
  cargo_command(&C, 0, CG_CLOSE, 0); run(&C, 0.3f);
  CHECK(!strcmp(C.msg, "latch 1 closed: nothing in it"), "closed empty: '%s'", C.msg);
  cargo_command(&C, 0, CG_OPEN, 0); run(&C, 0.3f); cargo_command(&C, 0, CG_CLOSE, 0); cargo_switches(&C, 0x3, 0x3); run(&C, 0.3f);
  CHECK(!strcmp(C.msg, "latch 1 closed: holding a load"), "caught: '%s'", C.msg);
  CHECK(cargo_bits(&C, 0) == (1 | 2 | 8), "bits %d", cargo_bits(&C, 0));
  cargo_switches(&C, 0x3, 0x1);
  CHECK(cargo_bits(&C, 1) == 0, "no switch on latch 2: not loaded, open: %d", cargo_bits(&C, 1));

  /* the radio: the ground's LATCH command as CRSF, read by the receiver's board, taken once, and only while fresh */
  rc_input in; memset(&in, 0, sizeof in);
  crsf_parser P; memset(&P, 0, sizeof P);
  uint8_t f[64]; float v[2] = { 1, CG_CLOSE };
  int n = tlm_crsf_cmd(f, RC_CMD_LATCH, 7, v, 2);
  for (int i = 0; i < n; i++) tlm_crsf_input(&P, f[i], &in, 10.0);
  CHECK(in.cmd == RC_CMD_LATCH && in.cmd_seq == 7 && in.cmd_v[0] == 1 && in.cmd_v[1] == CG_CLOSE, "decoded %d #%u %g %g", in.cmd, in.cmd_seq, in.cmd_v[0], in.cmd_v[1]);
  cargo_init(&C, 2, 0x1, 0);
  CHECK(cargo_from_rc(&C, &in, 10.02) == 1 && cargo_drive(&C) == 0x3, "radio closed latch 2: %x", cargo_drive(&C));
  CHECK(!strcmp(C.msg, "latch 2 closing (radio)"), "says the radio: '%s'", C.msg);
  CHECK(cargo_from_rc(&C, &in, 10.04) == 0, "the same command isn't taken twice");
  v[0] = -1; v[1] = CG_OPEN; n = tlm_crsf_cmd(f, RC_CMD_LATCH, 8, v, 2);
  for (int i = 0; i < n; i++) tlm_crsf_input(&P, f[i], &in, 11.0);
  CHECK(cargo_from_rc(&C, &in, 12.5) == 0 && cargo_drive(&C) == 0x3, "a stale command (1.5 s old) isn't acted on");
  n = tlm_crsf_cmd(f, RC_CMD_LATCH, 9, v, 2);
  for (int i = 0; i < n; i++) tlm_crsf_input(&P, f[i], &in, 13.0);
  CHECK(cargo_from_rc(&C, &in, 13.01) == 1 && cargo_drive(&C) == 0, "−1 opens them all: %x", cargo_drive(&C));
  float g[4] = { 1, 2, 3, 0 }; n = tlm_crsf_cmd(f, RC_CMD_GOTO, 10, g, 4);
  for (int i = 0; i < n; i++) tlm_crsf_input(&P, f[i], &in, 14.0);
  CHECK(cargo_from_rc(&C, &in, 14.01) == 0 && cargo_drive(&C) == 0, "a go-to isn't for the cargo");

  /* the same command passed to another board (RN_LINK_RC) */
  rc_input b2; memset(&b2, 0, sizeof b2); float pk[RC_PACK_N];
  v[0] = 0; v[1] = CG_TOGGLE; n = tlm_crsf_cmd(f, RC_CMD_LATCH, 11, v, 2);
  for (int i = 0; i < n; i++) tlm_crsf_input(&P, f[i], &in, 15.0);
  rc_pack(&in, 15.02, pk); rc_unpack(&b2, pk, RC_PACK_N, 15.03);
  CHECK(cargo_from_rc(&C, &b2, 15.03) == 1 && cargo_drive(&C) == 0x1, "over the board link: %x", cargo_drive(&C));

  /* the telemetry item and its message */
  tlm_store T; tlm_init(&T);
  cargo_init(&C, 2, 0x2, 0); cargo_switches(&C, 0x2, 0x2); cargo_command(&C, 1, CG_OPEN, "radio");
  tlm_from_cargo(&T, &C, 1.0);
  const tlm_slot *s = tlm_get(&T, TLM_CARGO);
  CHECK(s && s->n == 3 && s->v[0] == 2 && s->v[1] == 0 && s->v[2] == (2 | 4 | 8), "item %d: %g %g %g", s ? s->n : -1, s ? s->v[0] : 0, s ? s->v[1] : 0, s ? s->v[2] : 0);
  CHECK(T.qn == 1 && !strcmp(T.q[T.qh].s, "latch 2 opening (radio)") && !C.said, "message queued once: %d '%s'", T.qn, T.qn ? T.q[T.qh].s : "");
  tlm_from_cargo(&T, &C, 1.1); CHECK(T.qn == 1, "not again");
  uint8_t out[64]; int fl = tlm_crsf.item(&T, TLM_CARGO, out);
  CHECK(fl == 4 + 3 + 6 && out[2] == CRSF_EXT && out[3] == CRSF_EXT_ITEM && out[4] == TLM_CARGO && out[5] == 3 && out[11] == (2 | 4 | 8), "CRSF item frame (%d bytes)", fl);

  /* the pickup: a drone that follows its target with a lag (and a little sway), over a spot 3 m away */
  {
    pickup_state K; pickup_init(&K); nav_out o; memset(&o, 0, sizeof o); o.have_home = 1; o.p[2] = 1.5f;
    nav_sp sp; float spot[3] = { 3, 1, 0.2f };
    CHECK(pickup_start(&K, spot, 0.5f, 1, &o, 0) == 0 && K.phase == PK_OVER && K.z_over > 1.49f && K.z_over < 1.51f, "starts over it, as high as it is (1.5 m): z %g", K.z_over);
    float minz = 9; int closed_at = -1, saw[5] = { 0 }; double t = 0;
    for (; t < 40 && pickup_active(&K); t += 0.01) {
      pickup_step(&K, &o, t, 0.01f, &sp); saw[K.phase] = 1;
      for (int k = 0; k < 3; k++) { float v = (sp.target[k] - o.p[k]) * 2.5f; v = v > 1.5f ? 1.5f : v < -1.5f ? -1.5f : v; o.v[k] = v; o.p[k] += v * 0.01f; }
      if (o.p[2] < minz) minz = o.p[2];
      if (K.nreq == 1 && closed_at < 0) closed_at = (int)(t * 100);
    }
    CHECK(saw[PK_OVER] && saw[PK_DOWN] && saw[PK_CLOSE] && saw[PK_UP] && !pickup_active(&K) && K.done_ok, "over, down, close, up, done (%.1f s): %s", t, K.msg);
    CHECK(K.nreq == 1 && K.req_latch == 1 && K.req_act == 1 && closed_at > 0, "asked once to close latch 2, at %.1f s", closed_at / 100.0);
    CHECK(minz > 0.2f - PK_TOL && sp.heading == 0.5f, "never below the spot (lowest %.3f) and facing the way it was told", minz);
    CHECK(o.p[2] > 1.3f && fabsf(o.p[0] - 3) < 0.1f, "climbed back over it: %.2f %.2f %.2f", o.p[0], o.p[1], o.p[2]);
    /* a drone that can't hold still over it gives up, without closing */
    pickup_init(&K); memset(&o, 0, sizeof o); o.have_home = 1; o.p[2] = 1;
    pickup_start(&K, spot, 0, 0, &o, 0);
    for (t = 0; t < 60 && pickup_active(&K); t += 0.01) {
      pickup_step(&K, &o, t, 0.01f, &sp);
      for (int k = 0; k < 3; k++) { o.v[k] = (sp.target[k] - o.p[k]) * 2; o.p[k] += o.v[k] * 0.01f; }
      o.p[0] += 0.08f * sinf((float)t * 3) * 0.01f * 3; o.v[0] = 0.24f * cosf((float)t * 3);   /* swaying in the wind */
    }
    CHECK(!K.done_ok && K.nreq == 0 && !strcmp(K.msg, "pickup gave up"), "swaying: gives up after %.0f s, closes nothing: %s", PK_GIVEUP, K.msg);
    CHECK(pickup_start(&K, (float[]){ 40, 0, 1 }, 0, 0, &o, 0) == -1 && !pickup_active(&K), "a spot outside the box is refused: %s", K.msg);
    /* by radio: PICKUP through CRSF, the radio's pilot flies it; the sticks stop it */
    rc_pilot RP; rc_pilot_init(&RP); nav_state N; memset(&N, 0, sizeof N); rc_input ri; memset(&ri, 0, sizeof ri);
    crsf_parser Q; memset(&Q, 0, sizeof Q); uint8_t fb[64];
    float ch[16] = { 0, 0, 0, 0, 1, 0, 1, -1, -1, 0, 0, 0, 0, 0, 0, 0 };
    memset(&o, 0, sizeof o); o.have_home = 1; o.p[2] = 1.5f; o.fly = 1;
    double rt = 1; nav_sp rs; memset(&rs, 0, sizeof rs);
    for (int i = 0; i < 20; i++, rt += 0.02) { int m = crsf_rc(fb, CRSF_ADDR_FC, ch); for (int j = 0; j < m; j++) tlm_crsf_input(&Q, fb[j], &ri, rt); rc_pilot_step(&RP, &ri, rt, &N, &o, 0.02f, &rs); }
    float pv[5] = { 2, -1, 0.1f, 1.0f, 0 }; int m = tlm_crsf_cmd(fb, RC_CMD_PICKUP, 3, pv, 5); for (int j = 0; j < m; j++) tlm_crsf_input(&Q, fb[j], &ri, rt);
    m = crsf_rc(fb, CRSF_ADDR_FC, ch); for (int j = 0; j < m; j++) tlm_crsf_input(&Q, fb[j], &ri, rt);
    rc_pilot_step(&RP, &ri, rt, &N, &o, 0.02f, &rs);
    CHECK(pickup_active(&RP.pk) && fabsf(rs.target[0] - 2) < 0.01f && fabsf(rs.target[1] + 1) < 0.01f && fabsf(rs.heading - 1) < 0.01f, "the radio's PICKUP: over the spot, facing 1 rad (%g %g, %g)", rs.target[0], rs.target[1], rs.heading);
    ch[RC_PITCH] = 0.6f; rt += 0.02; m = crsf_rc(fb, CRSF_ADDR_FC, ch); for (int j = 0; j < m; j++) tlm_crsf_input(&Q, fb[j], &ri, rt);
    rc_pilot_step(&RP, &ri, rt, &N, &o, 0.02f, &rs);
    CHECK(!pickup_active(&RP.pk) && strstr(RP.msg, "sticks"), "the stick stops it: %s", RP.msg);
  }

  printf(fails ? "cargo: %d FAILED\n" : "cargo: ok\n", fails);
  return fails != 0;
}
