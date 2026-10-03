/* The command module: see ground_core.h. */
#include "ground_core.h"
#include "rc_core.h"
#include "tlm_crsf.h"

const char *const gnd_why_text[GND_WHY_N] = { "", "no telemetry", "weak link", "battery low", "battery very low", "returning home", "landing",
  "failsafe", "crashed", "the drone hears no radio" };

static float clampf_(float x, float a, float b) { return x < a ? a : x > b ? b : x; }
static int fin(float x) { return x == x && x < 3e38f && x > -3e38f; }
static void say(gnd_state *G, const char *s) { int i = 0; for (; s[i] && i < (int)sizeof G->why - 1; i++) G->why[i] = s[i]; G->why[i] = 0; }
static void zero(void *p, unsigned n) { char *c = (char *)p; while (n--) *c++ = 0; }

void gnd_config_default(gnd_config *c) { c->latch = 0; c->rc_period = 0.004f; c->cmd_gap = 0.15f; }

int gnd_init(gnd_state *G, rn_host *H, const gnd_config *c) {
  zero(G, sizeof *G);
  G->H = H; if (c) G->C = *c; else gnd_config_default(&G->C);
  G->level = 1; G->t_cmd = G->t_rc = -1e9;
  gnd_view *V = &G->V;
  V->t_link = V->t_att = V->t_batt = V->t_gps = V->t_baro = V->t_mode = V->t_any = -1;
  for (int i = 0; i < TLM_ITEMS; i++) V->item[i].t = -1;
  for (int i = 0; i < 16; i++) G->ch[i] = -1;
  if (!H) { say(G, "no step runner: the sticks go up unshaped"); return -1; }
  /* what this code passes and expects back, in floats (js/rn-sigs.js) */
  G->f_stick = rn_host_find(H, "stickInput"); G->f_alert = rn_host_find(H, "groundAlerts");
  if (G->f_stick < 0 || rn_host_in_size(H, G->f_stick) != 5 || rn_host_out_size(H, G->f_stick) != 1 ||
      G->f_alert < 0 || rn_host_in_size(H, G->f_alert) != 13 || rn_host_out_size(H, G->f_alert) != 2) { say(G, "the ground program's formulas aren't what this code expects: the sticks go up unshaped"); return -1; }
  if (rn_host_instances(H, "stickInput", GND_AXES)) { say(G, "can't give stickInput a memory per stick"); return -1; }
  G->ok = 1; say(G, "command module ready");
  return 0;
}

int gnd_command(gnd_state *G, int cmd, const float *v, int n) {
  if (G->qn >= GND_QN) return -1;
  int k = (G->qh + G->qn) % GND_QN; G->q[k].cmd = cmd; G->q[k].n = n > 6 ? 6 : n;
  for (int i = 0; i < G->q[k].n; i++) G->q[k].v[i] = v[i];
  G->qn++;
  return 0;
}
int gnd_goto(gnd_state *G, float x, float y, float z, float heading) { float v[4] = { x, y, z, heading }; return gnd_command(G, RC_CMD_GOTO, v, 4); }

/* What the drone reports, for the alerts: the flight core's state, the navigation's mode bits, the supervisor. */
static const float *item(const gnd_view *V, int id, int nmin) { return V->item[id].t >= 0 && V->item[id].n >= nmin ? V->item[id].v : 0; }
static void alerts(gnd_state *G, double t, float dt) {
  const gnd_view *V = &G->V;
  const float *st = item(V, TLM_STATE, 1), *nav = item(V, TLM_NAV, 5), *sup = item(V, TLM_SUPER, 5), *lk = item(V, TLM_LINK, 4);
  int navb = nav ? (int)nav[4] : 0, sm = sup ? (int)sup[0] : 0;
  float in[13]; int k = 0;
  in[k++] = V->t_any >= 0 ? (float)(t - V->t_any) : 1e3f;
  int lq = V->t_link >= 0 && t - V->t_link < 2; in[k++] = lq ? 1.0f : 0.0f; in[k++] = lq ? V->link.up_lq : 0;
  int soc = sup && sup[3] >= 0; in[k++] = soc ? 1.0f : 0.0f; in[k++] = soc ? sup[3] : 0;
  int vc = sup && sup[4] > 0 && V->t_batt >= 0 && V->volts > 0; in[k++] = vc ? 1.0f : 0.0f; in[k++] = vc ? V->volts / sup[4] : 0;
  in[k++] = st && (int)st[0] == 2 ? 1.0f : 0.0f;                                     /* failsafe */
  in[k++] = st && (int)st[0] == 3 ? 1.0f : 0.0f;                                     /* crashed */
  in[k++] = (navb & 8) || sm == 2 ? 1.0f : 0.0f;                                     /* returning */
  in[k++] = (navb & 16) || sm == 3 ? 1.0f : 0.0f;                                    /* landing */
  in[k++] = (navb & 64) || (lk && lk[3] > 0.5f) ? 1.0f : 0.0f;                       /* the drone hears no radio */
  in[k++] = dt;
  float out[2];
  if (G->ok && !rn_host_call(G->H, G->f_alert, 0, in, out) && fin(out[0]) && fin(out[1])) {
    G->alert = (int)clampf_(out[0], 0, 2); int w = (int)out[1]; G->alert_why = w >= 0 && w < GND_WHY_N ? w : 0;
  }
}

int gnd_step(gnd_state *G, const gnd_input *in, double t, float dt, uint8_t *out, int cap) {
  /* buttons: latching ones toggle on each press */
  uint32_t rise = in->held & ~G->held_was; G->held_was = in->held;
  G->latched ^= rise & G->C.latch;
  uint32_t on = (in->held & ~G->C.latch) | (G->latched & G->C.latch);
  if (on & GB(GB_GENTLE)) G->level = 0; else if (on & GB(GB_NORMAL)) G->level = 1; else if (on & GB(GB_SPORT)) G->level = 2;
  if (rise & GB(GB_CAL)) { float c = 1; gnd_command(G, RC_CMD_LEARN, &c, 1); }
  /* the sticks, through stickInput (raw if it can't answer) */
  static const int plus[GND_AXES] = { GB_RIGHT, GB_FWD, GB_UP, GB_YAWR }, minus[GND_AXES] = { GB_LEFT, GB_BACK, GB_DOWN, GB_YAWL };
  int shaped = 1;
  for (int a = 0; a < GND_AXES; a++) {
    int analog = (in->has_axis >> a) & 1;
    float dig = (float)(((on >> plus[a]) & 1) - (int)((on >> minus[a]) & 1)), raw = analog ? clampf_(in->axis[a], -1, 1) : dig;
    float x[5] = { (float)a, analog ? 1.0f : 0.0f, analog ? in->axis[a] : 0, dig, dt }, y = raw;
    if (!(G->ok && !rn_host_call(G->H, G->f_stick, a, x, &y) && fin(y))) { y = raw; shaped = 0; }
    G->stick[a] = clampf_(y, -1, 1);
  }
  G->shaped = shaped;
  for (int i = 0; i < 16; i++) G->ch[i] = -1;
  for (int a = 0; a < GND_AXES; a++) G->ch[a] = G->stick[a];
  G->ch[RC_ARM] = on & GB(GB_ARM) ? 1 : -1;
  G->ch[RC_LEVEL] = (float)(G->level - 1);
  G->ch[RC_FLY] = on & GB(GB_FLY) ? 1 : -1;
  G->ch[RC_HOLD] = on & GB(GB_HOLD) ? 1 : -1;
  G->ch[RC_HOME] = on & GB(GB_HOME) ? 1 : -1;
  /* alerts, ten times a second */
  if (t - G->t_alert_step >= 0.1) { float adt = G->t_alert_step > 0 ? (float)(t - G->t_alert_step) : 0.1f; G->t_alert_step = t; alerts(G, t, adt); }
  /* what goes to the transmitter module now */
  int n = 0;
  if (t - G->t_rc >= G->C.rc_period - 1e-6 && cap - n >= 26) { n += crsf_rc(out + n, CRSF_ADDR_TX, G->ch); G->t_rc = t; }
  if (G->qn && t - G->t_cmd >= G->C.cmd_gap && cap - n >= CRSF_MAX_FRAME) {
    G->seq = G->seq % 255 + 1;
    n += tlm_crsf_cmd(out + n, G->q[G->qh].cmd, G->seq, G->q[G->qh].v, G->q[G->qh].n);
    G->qh = (G->qh + 1) % GND_QN; G->qn--; G->t_cmd = t;
  }
  return n;
}

/* ── what comes down ── */
static int be16u(const uint8_t *p) { return (p[0] << 8) | p[1]; }
static int be16s(const uint8_t *p) { int v = be16u(p); return v & 0x8000 ? v - 0x10000 : v; }
static long be32s(const uint8_t *p) { uint32_t v = ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]; return (long)(int32_t)v; }
static void copy_str(char *d, const uint8_t *s, int n, int cap) { int i = 0; for (; i < n && i < cap - 1 && s[i]; i++) d[i] = (char)s[i]; d[i] = 0; }

static int frame(gnd_state *G, double t) {   /* 1: one of the drone's */
  gnd_view *V = &G->V; const uint8_t *p = crsf_payload(&G->P); int n = crsf_payload_len(&G->P);
  switch (crsf_type(&G->P)) {
    case CRSF_LINK_STATS: crsf_link_stats_read(p, n, &V->link); V->t_link = t; return 0;   /* (the transmitter module's own) */
    case CRSF_RC: return 0;                                            /* our own frames, echoed on a one-wire port */
    case CRSF_ATTITUDE: if (n >= 6) { V->pitch = be16s(p) / 1e4f; V->roll = be16s(p + 2) / 1e4f; V->yaw = be16s(p + 4) / 1e4f; V->t_att = t; } break;
    case CRSF_BATTERY: if (n >= 8) { V->volts = be16u(p) / 10.0f; V->amps = be16u(p + 2) / 10.0f; V->mah = (float)((p[4] << 16) | (p[5] << 8) | p[6]); V->pct = p[7]; V->t_batt = t; } break;
    case CRSF_GPS: if (n >= 15) { V->lat = be32s(p) * 1e-7; V->lon = be32s(p + 4) * 1e-7; V->speed = be16u(p + 8) / 36.0f; V->course = be16u(p + 10) / 100.0f;
      V->alt = (float)(be16u(p + 12) - 1000); V->sats = p[14]; V->t_gps = t; } break;
    case CRSF_BARO_ALT: if (n >= 2) { int a = be16u(p); V->baro_alt = a & 0x8000 ? (float)(a & 0x7FFF) : (a - 10000) / 10.0f; if (n >= 4) V->vz = be16s(p + 2) / 100.0f; V->t_baro = t; } break;
    case CRSF_VARIO: if (n >= 2) { V->vz = be16s(p) / 100.0f; } break;
    case CRSF_FLIGHT_MODE: copy_str(V->mode, p, n, (int)sizeof V->mode); V->t_mode = t; break;
    case CRSF_EXT:
      if (n >= 2 && p[0] == CRSF_EXT_TEXT) {
        int k = (int)(V->nmsg % GND_MSGS); V->msg[k].sev = p[1]; copy_str(V->msg[k].s, p + 2, n - 2, (int)sizeof V->msg[k].s); V->msg[k].t = t; V->nmsg++;
      } else if (n >= 3 && p[0] == CRSF_EXT_ITEM) {
        int id = p[1], m = p[2];
        if (id <= 0 || id >= TLM_ITEMS || m > TLM_NV || 3 + 2 * m > n) break;
        for (int k = 0; k < m; k++) V->item[id].v[k] = be16s(p + 3 + 2 * k) / tlm_scale(id, k);
        V->item[id].n = m; V->item[id].t = t;
      } else if (n >= 1 && p[0] == CRSF_EXT_CMD) return 0;            /* (our own command, echoed) */
      break;
    default: break;
  }
  V->frames++; V->t_any = t;
  return 1;
}
void gnd_from_radio(gnd_state *G, const uint8_t *b, int n, double t) {
  for (int i = 0; i < n; i++) {
    int len = crsf_feed(&G->P, b[i]);
    if (len < 0) G->V.bad++; else if (len > 0 && frame(G, t)) G->V.bytes += (uint32_t)len;   /* (the drone's telemetry only) */
  }
}

/* The view as floats. Times are ages [s] (−1: never).
 *   0 alert, why, speed level, bytes, frames, bad frames, messages so far, age of the last frame
 *   8 link: age, uplink RSSI, LQ, SNR, downlink RSSI, LQ, SNR, power [mW]
 *  16 attitude: age, roll, pitch, yaw     20 battery: age, volts, amps, mAh, %
 *  25 GPS: age, latitude (whole degrees, then the rest), longitude (the same), speed, course, altitude, satellites
 *  34 barometer: age, height, climb       37 flight mode: age (the text: gnd_mode)      38 the 16 channels sent
 *  54 shaped (1: the sticks went through stickInput), then the drone's items: id, age, n, values…, ending with 0 */
int gnd_view_pack(const gnd_state *G, double t, float *o, int cap) {
  const gnd_view *V = &G->V; int k = 0;
  if (cap < 64) return 0;
#define AGE(x) ((x) >= 0 ? (float)(t - (x)) : -1.0f)
  o[k++] = (float)G->alert; o[k++] = (float)G->alert_why; o[k++] = (float)G->level; o[k++] = (float)V->bytes; o[k++] = (float)V->frames; o[k++] = (float)V->bad;
  o[k++] = (float)V->nmsg; o[k++] = AGE(V->t_any);
  o[k++] = AGE(V->t_link); o[k++] = V->link.up_rssi; o[k++] = V->link.up_lq; o[k++] = V->link.up_snr; o[k++] = V->link.down_rssi; o[k++] = V->link.down_lq;
  o[k++] = V->link.down_snr; o[k++] = (float)V->link.tx_power_mw;
  o[k++] = AGE(V->t_att); o[k++] = V->roll; o[k++] = V->pitch; o[k++] = V->yaw;
  o[k++] = AGE(V->t_batt); o[k++] = V->volts; o[k++] = V->amps; o[k++] = V->mah; o[k++] = (float)V->pct;
  double la = V->lat, lo = V->lon, lai = (double)(long)la, loi = (double)(long)lo;
  o[k++] = AGE(V->t_gps); o[k++] = (float)lai; o[k++] = (float)(la - lai); o[k++] = (float)loi; o[k++] = (float)(lo - loi);
  o[k++] = V->speed; o[k++] = V->course; o[k++] = V->alt; o[k++] = (float)V->sats;
  o[k++] = AGE(V->t_baro); o[k++] = V->baro_alt; o[k++] = V->vz;
  o[k++] = AGE(V->t_mode);
  for (int i = 0; i < 16; i++) o[k++] = G->ch[i];
  o[k++] = (float)G->shaped;
  for (int id = 1; id < TLM_ITEMS; id++) {
    if (V->item[id].t < 0) continue;
    if (k + 3 + V->item[id].n + 1 > cap) break;
    o[k++] = (float)id; o[k++] = AGE(V->item[id].t); o[k++] = (float)V->item[id].n;
    for (int j = 0; j < V->item[id].n; j++) o[k++] = V->item[id].v[j];
  }
  o[k++] = 0;
#undef AGE
  return k;
}
