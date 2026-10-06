/*
 * The CRSF transport (ExpressLRS, Crossfire): telemetry items as CRSF frames for the receiver's UART, and what the
 * receiver sends the drone (channels, link statistics, ground-station commands) read into an rc_input.
 *
 * Items with a standard CRSF frame go as that frame, so any radio shows them: attitude, battery (voltage from the
 * flight core, current, used capacity and charge from the supervisor when there is one), barometric height and
 * vertical speed, GPS. The flight mode goes as the flight-mode frame, messages as ArduPilot status text. The rest go
 * as 0x80 frames, subtype 0xD0: item id, value count, then each value × its scale (tlm_scale) as a signed 16-bit
 * big-endian integer.
 */
#include "tlm_core.h"
#include "rc_core.h"
#include "crsf.h"
#include "tlm_crsf.h"

static int item(const tlm_store *T, int id, uint8_t *out) {
  const tlm_slot *s = tlm_get(T, id); if (!s) return 0;
  const float *v = s->v;
  switch (id) {
    case TLM_ATT: return crsf_attitude(out, v[0], v[1], v[2]);
    case TLM_BATT: {
      const tlm_slot *sp = tlm_get(T, TLM_SUPER);
      float amps = sp && sp->n > 5 && sp->v[5] >= 0 ? sp->v[5] : 0, mah = sp && sp->n > 6 ? sp->v[6] : 0;
      int pct = sp && sp->n > 3 && sp->v[3] >= 0 ? (int)(sp->v[3] * 100 + 0.5f) : 0;
      return crsf_battery(out, v[0], amps, mah, pct);
    }
    case TLM_ALT: return crsf_baro_alt(out, v[0], s->n > 1 ? v[1] : 0);
    case TLM_GPS: {
      double lat = ((double)v[0] * 65536 + v[1]) * 1e-7, lon = ((double)v[2] * 65536 + v[3]) * 1e-7;
      return crsf_gps(out, lat, lon, v[5], v[6], v[4], (int)v[7]);
    }
    case TLM_LINK: return 0;                                     /* the transmitter makes its own link statistics */
    default: {
      uint8_t p[CRSF_MAX_PAYLOAD]; int n = 0, m = s->n;
      if (3 + 2 * m > CRSF_MAX_PAYLOAD) m = (CRSF_MAX_PAYLOAD - 3) / 2;
      p[n++] = CRSF_EXT_ITEM; p[n++] = (uint8_t)id; p[n++] = (uint8_t)m;
      for (int k = 0; k < m; k++) {
        float x = v[k] * tlm_scale(id, k); long q = (long)(x < 0 ? x - 0.5f : x + 0.5f);
        q = q > 32767 ? 32767 : q < -32768 ? -32768 : q;
        p[n++] = (uint8_t)((q >> 8) & 0xFF); p[n++] = (uint8_t)(q & 0xFF);
      }
      return crsf_frame(out, CRSF_ADDR_FC, CRSF_EXT, p, n);
    }
  }
}
static int mode(const char *m, uint8_t *out) { return crsf_flight_mode(out, m); }
static int text(int sev, const char *s, uint8_t *out) { return crsf_text(out, sev, s); }
const tlm_transport tlm_crsf = { "CRSF (ExpressLRS)", CRSF_MAX_FRAME, item, mode, text };

/* What comes from the receiver. Returns the frame type taken (0: none yet). */
int tlm_crsf_input(crsf_parser *P, uint8_t b, rc_input *in, double t) {
  int len = crsf_feed(P, b); if (len <= 0) return 0;
  const uint8_t *p = crsf_payload(P); int n = crsf_payload_len(P);
  switch (crsf_type(P)) {
    case CRSF_RC:
      if (n >= 22) {
        if (in->frames && t - in->t_ch > RC_LOST_S) { in->cmd_seq = 0; in->cmd = 0; }   /* back after a loss: the ground may have restarted its numbering */
        crsf_rc_read(p, in->ch); in->t_ch = t; in->frames++;
      }
      return CRSF_RC;
    case CRSF_LINK_STATS: {
      if (n < 10) return 0;                                       /* (short: not link statistics) */
      crsf_link L; crsf_link_stats_read(p, n, &L);
      in->up_rssi = L.up_rssi; in->up_lq = L.up_lq; in->up_snr = L.up_snr; in->down_rssi = L.down_rssi; in->down_lq = L.down_lq;
      in->rf_mode = L.rf_mode; in->tx_power = L.tx_power_mw; in->t_link = t;
      return CRSF_LINK_STATS;
    }
    case CRSF_EXT:
      if (n >= 3 && p[0] == CRSF_EXT_CMD) {                       /* a ground-station command: cmd, seq, values (16-bit, /100 or /1000) */
        int cmd = p[1], seq = p[2], m = (n - 3) / 2;
        if ((uint32_t)seq == in->cmd_seq) return CRSF_EXT;          /* (a repeat: the ground numbers them 1–255) */
        in->cmd = cmd; in->cmd_seq = (uint32_t)seq; in->t_cmd = t;
        for (int k = 0; k < 6; k++) {
          int16_t q = k < m ? (int16_t)((p[3 + 2 * k] << 8) | p[4 + 2 * k]) : 0;
          in->cmd_v[k] = q / rc_cmd_scale(cmd, k);
        }
      }
      return CRSF_EXT;
  }
  return 0;
}
/* A ground-station command frame (what the ground sends up): rc_core.h's commands, values scaled by rc_cmd_scale. */
int tlm_crsf_cmd(uint8_t *out, int cmd, int seq, const float *v, int nv) {
  uint8_t p[3 + 12]; int n = 0;
  p[n++] = CRSF_EXT_CMD; p[n++] = (uint8_t)cmd; p[n++] = (uint8_t)seq;
  for (int k = 0; k < nv && k < 6; k++) {
    float x = v[k] * rc_cmd_scale(cmd, k); long q = (long)(x < 0 ? x - 0.5f : x + 0.5f);
    q = q > 32767 ? 32767 : q < -32768 ? -32768 : q;
    p[n++] = (uint8_t)((q >> 8) & 0xFF); p[n++] = (uint8_t)(q & 0xFF);
  }
  return crsf_frame(out, CRSF_ADDR_FC, CRSF_EXT, p, n);
}
