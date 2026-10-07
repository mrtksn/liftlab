/* Tests for the fleet program (fleet.h): drones, each with its peer end (peer.c, fleet_link.c) and its fleet program
 * (fleet_core.c, the default js/laws.js fleetProgram in the built-in program), over a simulated air. The navigation
 * is stood in for by a drone that flies straight to its target; the leader is flown round a circle by its pilot.
 *   cc -O2 -I.. -I. -o test_fleet test_fleet.c fleet_core.c fleet_link.c peer.c plink.c rc_core.c pickup_core.c nav_core.c
 *      radio_link.c crsf.c tlm_crsf.c tlm_core.c ../rn_host.c ../rn.c ../rn_builtin.c -lm && ./test_fleet */
#include "fleet.h"
#include "rc_core.h"
#include "rn_host.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern const uint8_t *const rn_builtin_img;
extern const uint32_t rn_builtin_len;
static int fails = 0;
#define CHECK(c, ...) do { if (c) printf("  ok   "); else { printf("  FAIL "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

#define ND 4
typedef struct {
  rn_host H; peer_net P; fleet_link K; fleet_state F; nav_state N; nav_out o;
  uint8_t addr[6]; float home[3], p[3], v[3], hd; int on, gps;
} drone;
static drone D[ND];
static float arenas_[ND][3][65536], pools_[ND][3][8192]; static int32_t codes_[ND][3][32768];
typedef struct { uint8_t b[PEER_MTU]; int n, from, to; double at; } flying;
static flying air[4096]; static int air_n; static int cut[ND][ND];
static double T;

static void start(int i, float hx, float hy, int gps) {
  drone *d = &D[i]; memset(&d->P, 0, sizeof *d - offsetof(drone, P));
  float *ar[3] = { arenas_[i][0], arenas_[i][1], arenas_[i][2] }, *po[3] = { pools_[i][0], pools_[i][1], pools_[i][2] }; int32_t *co[3] = { codes_[i][0], codes_[i][1], codes_[i][2] };
  memset(&d->H, 0, sizeof d->H);
  if (rn_host_init(&d->H, rn_builtin_img, rn_builtin_len, ar, 65536, co, 32768, po, 8192)) { printf("host init failed\n"); exit(1); }
  d->addr[0] = 0x24; d->addr[1] = 0x6F; d->addr[2] = 0x28; d->addr[3] = 0x10; d->addr[4] = 0; d->addr[5] = (uint8_t)(i + 1);
  char name[16]; snprintf(name, sizeof name, "drone %c", 'A' + i);
  peer_init(&d->P, peer_id_of(d->addr), 100 + (uint32_t)i, name, "fleet");
  fleet_link_init(&d->K);
  if (fleet_init(&d->F, &d->H)) { printf("fleet_init: %s\n", d->F.msg); exit(1); }
  d->home[0] = hx; d->home[1] = hy; d->home[2] = 0; d->gps = gps; d->on = 1;
  d->N.have_home = 1; d->N.have_pa = 1; d->N.seen = gps ? 3 : 1; memcpy(d->N.home, d->home, sizeof d->home);
  d->p[2] = 1.5f; d->o.fly = 1; d->o.have_home = 1;
}
static void air_step(void) {
  for (int i = 0; i < ND; i++) {
    if (!D[i].on) continue;
    uint8_t b[PEER_MTU], a[6]; int n;
    while ((n = peer_to_air(&D[i].P, T, a, b, sizeof b)) > 0 && air_n < 4096) {
      int to = -1; if (memcmp(a, PEER_BROADCAST, 6)) for (int j = 0; j < ND; j++) if (!memcmp(a, D[j].addr, 6)) to = j;
      flying *f = &air[air_n++]; memcpy(f->b, b, (size_t)n); f->n = n; f->from = i; f->to = to; f->at = T + 0.002;
    }
  }
  for (int k = 0; k < air_n;) {
    flying *f = &air[k]; if (f->at > T) { k++; continue; }
    for (int j = 0; j < ND; j++) if (j != f->from && D[j].on && (f->to < 0 || f->to == j) && !cut[f->from][j]) peer_from_air(&D[j].P, D[f->from].addr, f->b, f->n, -60, T);
    air[k] = air[--air_n];
  }
}
/* the leader: round a circle of 6 m, 1 m/s, flown by its pilot */
static void leader_fly(drone *d, float dt) {
  float w = 1.0f / 6, a = (float)T * w;
  float c[2] = { 2, 8 };                                        /* (the circle's centre, in its frame) */
  d->p[0] = c[0] + 6 * cosf(a); d->p[1] = c[1] + 6 * sinf(a); d->p[2] = 2;
  d->v[0] = -sinf(a); d->v[1] = cosf(a); d->v[2] = 0; d->hd = atan2f(d->v[1], d->v[0]); (void)dt;
}
/* a follower: the navigation, roughly: toward its target, at most 3 m/s, feeding its velocity forward */
static void follow_fly(drone *d, float dt) {
  float tgt[3], vf[3]; for (int i = 0; i < 3; i++) { tgt[i] = d->o.fly ? d->p[i] : 0; vf[i] = 0; }
  if (d->F.engaged) for (int i = 0; i < 3; i++) { tgt[i] = d->F.go_p[i]; vf[i] = d->F.go_v[i]; }
  for (int i = 0; i < 3; i++) { float u = 1.5f * (tgt[i] - d->p[i]) + vf[i]; if (u > 3) u = 3; if (u < -3) u = -3; d->v[i] += (u - d->v[i]) * fminf(1, 4 * dt); d->p[i] += d->v[i] * dt; }
  if (d->F.engaged) d->hd = d->F.go_h;
}
static void step(int lead) {
  air_step();
  if (((int)lround(T * 1000)) % 10) return;                   /* the navigation: 100 Hz */
  for (int i = 0; i < ND; i++) {
    drone *d = &D[i]; if (!d->on) continue;
    if (i == lead) leader_fly(d, 0.01f); else follow_fly(d, 0.01f);
    for (int k = 0; k < 3; k++) { d->o.p[k] = d->p[k]; d->o.v[k] = d->v[k]; d->N.pa[k] = d->home[k] + d->p[k]; }
    d->o.heading = d->hd;
    if (((int)lround(T * 1000)) % 100 == 0) {                   /* the peer end and the program: 10 Hz, on one board */
      float head[3] = { 1, 80 - (float)i, d->p[2] }, pk[FLEET_PACK_MAX], out[FLEET_OUT_MAX];
      fleet_link_publish(&d->K, &d->P, head, T);
      int n = fleet_link_pack(&d->P, T, head, pk); fleet_peers(&d->F, pk, n, T);
      fleet_step(&d->F, &d->N, &d->o, T);
      if ((n = fleet_out(&d->F, out))) fleet_link_apply(&d->K, &d->P, out, n, T);
    }
  }
}
static void run(double s, int lead) { int n = (int)lround(s * 1000); for (int k = 0; k < n; k++) { T += 0.001; step(lead); } }
static float dist2(const drone *a, const drone *b) { return hypotf(a->home[0] + a->p[0] - b->home[0] - b->p[0], a->home[1] + a->p[1] - b->home[1] - b->p[1]); }
/* where place k should be, in the world: behind the leader and to the side */
static void place_of(const drone *L, int k, float *x, float *y) {
  float row = (float)(k / 2 + 1), side = k % 2 ? -1.0f : 1.0f, bx = -2 * row, by = 1.5f * row * side, c = cosf(L->hd), s = sinf(L->hd);
  *x = L->home[0] + L->p[0] + c * bx - s * by; *y = L->home[1] + L->p[1] + s * bx + c * by;
}
static int vals_of(int i, int j, float *v) { int s = peer_find(&D[i].P, D[j].P.id); if (s < 0) return 0; memcpy(v, D[i].P.P[s].vals, sizeof D[i].P.P[s].vals); return D[i].P.P[s].nvals; }

int main(void) {
  printf("the program in the built-in program\n");
  start(0, 0, 0, 1); start(1, 3, 0, 1); start(2, 6, 0, 1); start(3, 9, 0, 1);
  D[3].on = 0;
  CHECK(D[0].F.ok && D[1].F.ok, "fleetProgram found, its inputs and outputs as this code passes them");
  { nav_out g = D[1].o; g.fly = 0; CHECK(fleet_engage(&D[1].F, 1, &D[1].N, &g, 0) == -1 && strstr(D[1].F.msg, "take off"), "engaging on the ground: refused (%s)", D[1].F.msg); }
  run(2.0, 0);
  float v[PEER_VALS]; int n = vals_of(1, 0, v);
  CHECK(n == FLEET_PUB - FLEET_VALS + 3 && v[0] == 1 && fabsf(v[FLEET_HEAD + 0] - (D[0].home[0] + D[0].p[0])) < 0.5f && v[FLEET_HEAD + 7] == (FLEET_F_SHARED | FLEET_F_FLYING),
        "B hears A's state, battery, height, its position in the fleet's frame, flags, and what its program publishes (%d values)", n);
  CHECK(D[1].F.calls >= 15 && !D[1].F.fails, "each program runs 10 times a second (%u calls)", (unsigned)D[1].F.calls);

  printf("a formation behind the leader\n");
  fleet_engage(&D[1].F, 1, &D[1].N, &D[1].o, 0); fleet_engage(&D[2].F, 1, &D[2].N, &D[2].o, 0);
  CHECK(D[1].F.engaged && D[2].F.engaged, "B and C engaged (%s)", D[2].F.msg);
  float closest = 1e9, worst = 0;
  for (int k = 0; k < 400; k++) {
    run(0.1, 0);
    for (int a = 0; a < 3; a++) for (int b = a + 1; b < 3; b++) closest = fminf(closest, dist2(&D[a], &D[b]));
    if (T > 25) for (int f = 1; f < 3; f++) { float x, y; place_of(&D[0], f - 1, &x, &y); worst = fmaxf(worst, hypotf(D[f].home[0] + D[f].p[0] - x, D[f].home[1] + D[f].p[1] - y)); }
  }
  CHECK(worst < 1.2f, "B and C keep their places behind A round the circle, within %.2f m", worst);
  CHECK(closest > 1.0f, "never nearer each other than %.2f m", closest);
  { float va[PEER_VALS], vb[PEER_VALS], vc[PEER_VALS]; vals_of(1, 0, va); vals_of(0, 1, vb); vals_of(0, 2, vc);
    float *A = va + FLEET_HEAD + FLEET_EXT, *B = vb + FLEET_HEAD + FLEET_EXT, *C = vc + FLEET_HEAD + FLEET_EXT;
    CHECK(A[0] == 1 && A[2] == 2 && B[0] == 2 && B[1] == 0 && C[0] == 2 && C[1] == 1 && B[2] == (float)(D[0].P.id >> 8),
          "published: A leads (2 joined, by their messages), B follows in place 0, C in place 1, both naming A");
    CHECK(vb[FLEET_HEAD + 7] == (FLEET_F_SHARED | FLEET_F_ENGAGED | FLEET_F_FLYING), "B says it's engaged"); }

  printf("the leader out of range, a drone without GPS, the sticks\n");
  for (int f = 1; f < 3; f++) cut[0][f] = cut[f][0] = 1;
  run(4.0, 0); float x1 = D[1].p[0]; run(2.0, 0);
  CHECK(D[1].F.engaged && fabsf(D[1].p[0] - x1) < 0.2f, "A lost: B holds its last place, still engaged (moved %.2f m in 2 s)", fabsf(D[1].p[0] - x1));
  for (int f = 1; f < 3; f++) cut[0][f] = cut[f][0] = 0;
  start(3, 9, 0, 0); run(3.0, 0); fleet_engage(&D[3].F, 1, &D[3].N, &D[3].o, 0); float x3 = D[3].p[0]; run(3.0, 0);
  CHECK(D[3].F.engaged && fabsf(D[3].p[0] - x3) < 0.05f, "D without GPS (no shared frame): engaged, it holds (it can't place the others)");
  { float vd[PEER_VALS]; vals_of(1, 3, vd); CHECK(vd[FLEET_HEAD + 7] == (FLEET_F_ENGAGED | FLEET_F_FLYING), "and says it has no shared frame"); }
  /* the pilot's sticks end it (rc_core.c), and the caller tells the fleet */
  { rc_pilot RP; rc_pilot_init(&RP); rc_input in; memset(&in, 0, sizeof in); nav_state Ns = D[1].N; nav_out o = D[1].o; nav_sp sp; memset(&sp, 0, sizeof sp);
    for (int i = 0; i < 16; i++) in.ch[i] = -1;
    in.ch[RC_ARM] = 1; in.ch[RC_FLY] = 1; in.ch[RC_THR] = 0; in.ch[RC_ROLL] = in.ch[RC_PITCH] = in.ch[RC_YAW] = 0; in.frames = 1; in.t_ch = T;
    RP.arm = 1; RP.fly = 1; RP.have_target = 1;
    RP.mis_on = 1; memcpy(RP.mis_t, D[1].F.go_p, sizeof RP.mis_t); RP.mis_h = 0.5f;
    rc_pilot_step(&RP, &in, T, &Ns, &o, 0.01f, &sp);
    int flew = RP.mis_on && fabsf(sp.target[0] - D[1].F.go_p[0]) < 1e-3f && sp.heading == 0.5f;
    in.ch[RC_PITCH] = 0.5f; in.t_ch = T + 0.01;
    rc_pilot_step(&RP, &in, T + 0.01, &Ns, &o, 0.01f, &sp);
    CHECK(flew && !RP.mis_on && RP.mis_why && strstr(RP.mis_why, "sticks"), "the radio's pilot flies the program's target; the sticks end it (%s)", RP.mis_why ? RP.mis_why : "-");
    fleet_engage(&D[1].F, 0, &D[1].N, &D[1].o, RP.mis_why);
    CHECK(!D[1].F.engaged && strstr(D[1].F.msg, "sticks"), "%s", D[1].F.msg);
    /* a FLEET command */
    in.cmd = RC_CMD_FLEET; in.cmd_v[0] = 1; in.cmd_seq = 9; in.t_cmd = T;
    rc_pilot_step(&RP, &in, T + 0.02, &Ns, &o, 0.01f, &sp);
    CHECK(RP.fleet_req == 1, "a FLEET 1 command asks the caller to engage it"); }
  { nav_out o = D[2].o; o.fly = 0; fleet_step(&D[2].F, &D[2].N, &o, T + 1);
    CHECK(!D[2].F.engaged && strstr(D[2].F.msg, "not flying"), "landed: it lets go (%s)", D[2].F.msg); }
  printf("%s\n", fails ? "FAILED" : "all passed");
  return fails ? 1 : 0;
}
