/* The telemetry store and its scheduler: see tlm_core.h. */
#include "tlm_core.h"

const tlm_def tlm_defs[TLM_ITEMS] = {
  [TLM_ATT] = { TLM_ATT, 3, 0.1f, 1, 0, "attitude" },
  [TLM_BATT] = { TLM_BATT, 1, 0.5f, 1, 0, "battery" },
  [TLM_ALT] = { TLM_ALT, 2, 0.2f, 1, 0, "height" },
  [TLM_GPS] = { TLM_GPS, 8, 0.5f, 1, 0, "GPS" },
  [TLM_STATE] = { TLM_STATE, 2, 1.0f, 0, 1, "state" },
  [TLM_MOTORS] = { TLM_MOTORS, 13, 0.5f, 3, 0, "motors" },
  [TLM_POS] = { TLM_POS, 6, 0.25f, 1, 0, "position" },
  [TLM_NAV] = { TLM_NAV, 6, 0.5f, 2, 0, "navigation" },
  [TLM_LEARN] = { TLM_LEARN, 7, 1.0f, 3, 0, "learning" },
  [TLM_SUPER] = { TLM_SUPER, 8, 1.0f, 2, 0, "supervisor" },
  [TLM_SUPER_M] = { TLM_SUPER_M, 13, 2.0f, 3, 0, "parts" },
  [TLM_LINK] = { TLM_LINK, 4, 1.0f, 3, 0, "link" },
  [TLM_CARGO] = { TLM_CARGO, 9, 2.0f, 1, 1, "cargo" },
};
float tlm_scale(int id, int k) {
  switch (id) {
    case TLM_MOTORS: case TLM_SUPER_M: return k == 0 ? 1 : 1000;
    case TLM_POS: return 100;                                          /* cm, cm/s */
    case TLM_NAV: return k < 3 ? 100 : k == 3 ? 1000 : 1;
    case TLM_LEARN: return k == 1 || k >= 5 ? 1000 : 1;
    case TLM_SUPER: return k == 2 ? 100 : k == 3 ? 1000 : k == 5 ? 10 : 1;
    case TLM_LINK: case TLM_CARGO: return 1;
    default: return 100;
  }
}

static void copy_text(char *d, const char *s, int n) { int i = 0; for (; i < n && s[i]; i++) d[i] = s[i]; d[i] = 0; }
void tlm_init(tlm_store *T) {
  char *p = (char *)T; for (unsigned i = 0; i < sizeof *T; i++) p[i] = 0;
  for (int i = 0; i < TLM_ITEMS; i++) T->it[i].t_sent = -1e9;
}
void tlm_put(tlm_store *T, int id, const float *v, int n, double t) {
  if (id <= 0 || id >= TLM_ITEMS) return;
  tlm_slot *s = &T->it[id]; if (n > TLM_NV) n = TLM_NV; if (n > tlm_defs[id].nmax) n = tlm_defs[id].nmax;
  for (int k = 0; k < n; k++) s->v[k] = v[k] == v[k] ? v[k] : 0;   /* (a NaN goes out as 0) */
  s->n = n; s->has = 1; s->t_put = t; s->dirty_link = 1;
}
const tlm_slot *tlm_get(const tlm_store *T, int id) { return id > 0 && id < TLM_ITEMS && T->it[id].has ? &T->it[id] : 0; }

static void push(tlm_msg *q, int *h, int *n, int sev, const char *s) {
  if (*n == TLM_QN) { *h = (*h + 1) % TLM_QN; (*n)--; }            /* full: the oldest goes */
  tlm_msg *m = &q[(*h + *n) % TLM_QN]; m->sev = sev; copy_text(m->s, s, TLM_TEXT); (*n)++;
}
/* Long text: split at spaces into pieces that fit a message. */
static void split(tlm_store *T, int sev, const char *s, int fwd) {
  int len = 0; while (s[len]) len++;
  int at = 0, pieces = 0;
  while (at < len && pieces < 6) {
    int n = len - at;
    if (n > TLM_TEXT) { n = TLM_TEXT; for (int k = TLM_TEXT; k > TLM_TEXT / 2; k--) if (s[at + k] == ' ') { n = k; break; } }
    char piece[TLM_TEXT + 1]; copy_text(piece, s + at, n);
    if (fwd) push(T->fq, &T->fh, &T->fn, sev, piece); else push(T->q, &T->qh, &T->qn, sev, piece);
    at += n; while (s[at] == ' ') at++; pieces++;
  }
}
void tlm_text(tlm_store *T, int sev, const char *s) { split(T, sev, s, 0); split(T, sev, s, 1); }

int tlm_pack(tlm_store *T, float *out, int cap) {
  int k = 0;
  for (int id = 1; id < TLM_ITEMS; id++) {
    tlm_slot *s = &T->it[id];
    if (!s->dirty_link || k + 2 + s->n > cap) continue;
    out[k++] = (float)id; out[k++] = (float)s->n; for (int j = 0; j < s->n; j++) out[k++] = s->v[j];
    s->dirty_link = 0;
  }
  while (T->fn) {
    const tlm_msg *m = &T->fq[T->fh]; int n = 0; while (m->s[n]) n++;
    if (k + 3 + n > cap) break;
    out[k++] = 255; out[k++] = (float)m->sev; out[k++] = (float)n; for (int j = 0; j < n; j++) out[k++] = (float)(unsigned char)m->s[j];
    T->fh = (T->fh + 1) % TLM_QN; T->fn--;
  }
  return k;
}
void tlm_unpack(tlm_store *T, const float *in, int n, double t) {
  int k = 0;
  while (k + 2 <= n) {
    int id = (int)in[k];
    if (id == 255) {
      if (k + 3 > n) return;
      int sev = (int)in[k + 1], len = (int)in[k + 2]; if (len < 0 || len > TLM_TEXT || k + 3 + len > n) return;
      char s[TLM_TEXT + 1]; for (int j = 0; j < len; j++) s[j] = (char)(int)in[k + 3 + j]; s[len] = 0;
      push(T->q, &T->qh, &T->qn, sev, s); k += 3 + len; continue;
    }
    int m = (int)in[k + 1]; if (id <= 0 || id >= TLM_ITEMS || m < 0 || m > TLM_NV || k + 2 + m > n) return;
    tlm_put(T, id, in + k + 2, m, t); T->it[id].dirty_link = 0; k += 2 + m;
  }
}

const char *tlm_mode(tlm_store *T) {
  const tlm_slot *st = tlm_get(T, TLM_STATE), *nv = tlm_get(T, TLM_NAV), *sp = tlm_get(T, TLM_SUPER), *ln = tlm_get(T, TLM_LEARN);
  const char *m = "WAIT";
  if (st) {
    int s = (int)st->v[0], flags = st->n > 1 ? (int)st->v[1] : 0, bits = nv && nv->n > 4 ? (int)nv->v[4] : 0;
    if (s == 0) m = bits & 32 ? "LANDED" : "DISARMED";
    else if (s == 3) m = "CRASHED";
    else if (s == 2) m = "!FS!";
    else if (s == 4) m = "MOTOR TEST";
    else if (flags & 8) m = "THROWN";
    else if (bits & 64) m = "RTH RC LOST";
    else if (bits & 16) m = "LAND";
    else if (bits & 8) m = "RTH";
    else if (ln && ln->v[0] > 0.5f) m = "CALIBRATE";
    else if (flags & 4) m = bits & 2 ? (sp && sp->v[0] >= 1 ? "POSHOLD*" : "POSHOLD") : "READY";
    else m = "ANGLE";
  }
  copy_text(T->mode, m, 15);
  return T->mode;
}

/* The scheduler. */
static int prio_due(const tlm_store *T, int id, double t, double *score) {
  const tlm_slot *s = &T->it[id]; const tlm_def *d = &tlm_defs[id];
  if (!s->has || s->t_put <= s->t_sent) return 0;                   /* nothing new since it went */
  double age = t - s->t_sent;
  if (age < d->period) return 0;
  *score = age / d->period / (1 + d->prio);
  return 1;
}
static int changed(const tlm_slot *s) { for (int k = 0; k < s->n; k++) if (s->v[k] != s->sent[k]) return 1; return 0; }
int tlm_service(tlm_store *T, const tlm_transport *X, double t, float budget, uint8_t *out, int cap) {
  double dt = T->t_service > 0 ? t - T->t_service : 0; T->t_service = t;
  if (dt > 1) T->rx_q = T->rx_text = 0;                      /* (a second and more: the receiver's queue has emptied) */
  if (dt < 0 || dt > 1) dt = 0;
  if (!(budget > 0 && budget < 1e6f)) budget = 0;            /* (NaN, negative: nothing) */
  double drain = dt * budget;                                 /* the receiver's queue, sent on at the budget */
  T->rx_q = T->rx_q > drain ? T->rx_q - drain : 0; T->rx_text = T->rx_text > drain ? T->rx_text - drain : 0;
  if (!(T->tokens == T->tokens)) T->tokens = 0;
  T->tokens += dt * budget; if (T->tokens > budget * 0.25 + X->max_frame) T->tokens = budget * 0.25 + X->max_frame;   /* a quarter second's worth, at most */
  int k = 0;
  while (T->tokens > 0 && k + X->max_frame <= cap) {
    int n = 0;
    /* the flight mode, when it changes and once a second */
    const char *m = tlm_mode(T); int same = 1; for (int i = 0; i < 16; i++) { if (m[i] != T->sent_mode[i]) { same = 0; break; } if (!m[i]) break; }
    if (!same || t - T->t_mode > 1.0) { n = X->mode(m, out + k); copy_text(T->sent_mode, m, 15); T->t_mode = t; }
    /* messages (one at a time through the receiver's queue: see tlm_core.h) */
    int text = 0;
    if (!n && T->qn && !(T->rx_text > 0)) { const tlm_msg *q = &T->q[T->qh]; n = X->text(q->sev, q->s, out + k); T->qh = (T->qh + 1) % TLM_QN; T->qn--; text = n > 0; }
    /* a state change */
    if (!n) for (int id = 1; id < TLM_ITEMS && !n; id++) {
      tlm_slot *s = &T->it[id];
      if (s->has && tlm_defs[id].on_change && s->t_put > s->t_sent && changed(s)) {
        n = X->item(T, id, out + k); s->t_sent = t; for (int j = 0; j < s->n; j++) s->sent[j] = s->v[j];
      }
    }
    /* the most overdue */
    if (!n) {
      int best = 0; double bs = 0, sc;
      for (int id = 1; id < TLM_ITEMS; id++) if (prio_due(T, id, t, &sc) && sc > bs) { bs = sc; best = id; }
      if (!best) break;
      tlm_slot *s = &T->it[best]; n = X->item(T, best, out + k); s->t_sent = t; for (int j = 0; j < s->n; j++) s->sent[j] = s->v[j];
      if (!n) continue;                                              /* this radio doesn't carry it */
    }
    k += n; T->tokens -= n; T->bytes_sent += (uint32_t)n; T->frames_sent++;
    T->rx_q += n; if (text) T->rx_text = T->rx_q;
  }
  return k;
}
