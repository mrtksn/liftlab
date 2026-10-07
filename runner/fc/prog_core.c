/* Programs on the data bus: see prog_core.h. */
#include "prog_core.h"

static void say(prog_state *G, const char *a, const char *b) {
  int k = 0;
  for (const char *s = a; *s && k < (int)sizeof G->why - 1; s++) G->why[k++] = *s;
  for (const char *s = b; s && *s && k < (int)sizeof G->why - 1; s++) G->why[k++] = *s;
  G->why[k] = 0;
}
static int starts(const char *s, const char *p) { for (int i = 0; p[i]; i++) if (s[i] != p[i]) return 0; return 1; }
static int fin(float x) { return (x - x) == 0; }

void prog_init(prog_state *G, rn_host *H, bus *B) {
  char *p = (char *)G; for (unsigned i = 0; i < sizeof *G; i++) p[i] = 0;
  G->H = H; G->B = B;
}

int prog_add(prog_state *G, const char *name, const char *out, int out_n, const char *out_layout, float period) {
  if (G->n >= PROG_MAX) { say(G, "too many programs on this board", 0); return -1; }
  int len = 0; while (name[len]) len++;
  if (!len || len >= (int)sizeof G->P[0].name) { say(G, "a program needs a name of up to 31 characters", 0); return -1; }
  if (!starts(out, "user.")) { say(G, "a program writes a topic under user.: ", out); return -1; }
  int fn = G->H ? rn_host_find(G->H, name) : -1;
  if (fn < 0) { say(G, "no formula in the board's program called ", name); return -1; }
  int o = bus_topic(G->B, out, out_n, out_layout);
  if (o < 0) { say(G, "its topic can't go on the bus (taken by another, or no room): ", out); return -1; }
  for (int i = 0; i < G->n; i++) if (G->P[i].out == o) { say(G, "another program already writes ", out); return -1; }
  prog_t *P = &G->P[G->n]; char *z = (char *)P; for (unsigned i = 0; i < sizeof *P; i++) z[i] = 0;
  for (int k = 0; k <= len; k++) P->name[k] = name[k];
  P->fn = fn; P->out = o; P->period = period > 0 ? period : 0; P->next = -1e9; P->last = -1;
  P->trig = P->period > 0 ? -1 : -2;                                  /* −2: on a change, of the read prog_trigger names */
  return G->n++;
}

int prog_read(prog_state *G, int i, const char *topic) {
  if (i < 0 || i >= G->n) return -1;
  prog_t *P = &G->P[i];
  if (P->nreads >= PROG_READS) { say(G, "a program reads at most 8 topics", 0); return -1; }
  int t = bus_find(G->B, topic); if (t < 0) { say(G, "not on this board's bus: ", topic); return -1; }
  if (t == P->out) { say(G, "a program can't read its own topic: ", topic); return -1; }
  P->reads[P->nreads++] = t;
  return 0;
}

int prog_check(prog_state *G, int i) {
  if (i < 0 || i >= G->n) return -1;
  prog_t *P = &G->P[i]; P->ok = 0;
  int need = 1; for (int k = 0; k < P->nreads; k++) need += G->B->T[P->reads[k]].n;
  if (rn_host_in_size(G->H, P->fn) != need) { say(G, P->name, ": its inputs aren't what its header reads"); return -1; }
  if (rn_host_out_size(G->H, P->fn) != G->B->T[P->out].n) { say(G, P->name, ": what it returns isn't its topic's layout"); return -1; }
  if (P->trig == -2) { say(G, P->name, ": it runs on a change, but not of a topic it reads"); return -1; }
  P->ok = 1; return 0;
}
/* the trigger, by name, among what it reads: set once the reads are in (prog_add can't know mirrors yet) */
int prog_trigger(prog_state *G, int i, const char *trig) {
  if (i < 0 || i >= G->n) return -1;
  int t = bus_find(G->B, trig); prog_t *P = &G->P[i];
  for (int k = 0; k < P->nreads; k++) if (P->reads[k] == t) { P->trig = t; return 0; }
  P->trig = -2; say(G, P->name, ": the topic it runs on isn't one it reads"); return -1;
}

void prog_step(prog_state *G) {
  bus *B = G->B; double now = B->now;
  for (int i = 0; i < G->n; i++) {
    prog_t *P = &G->P[i]; if (!P->ok) continue;
    if (P->trig >= 0) { if (!bus_changed(B, P->trig, &P->seen)) continue; }
    else { if (now < P->next - 1e-9) continue; P->next = P->next + P->period > now ? P->next + P->period : now + P->period; }
    int k = 0, ready = 1;
    for (int r = 0; r < P->nreads && ready; r++) {
      const float *v = bus_get(B, P->reads[r], 0, 0); if (!v) { ready = 0; break; }
      for (int j = 0; j < B->T[P->reads[r]].n; j++) G->in[k++] = v[j];
    }
    if (!ready) { P->waits++; continue; }
    float dt = P->last >= 0 ? (float)(now - P->last) : P->period > 0 ? P->period : 0.01f;
    G->in[k++] = dt; P->last = now;
    int e = rn_host_call(G->H, P->fn, 0, G->in, G->out), n = B->T[P->out].n;
    if (!e) for (int j = 0; j < n; j++) if (!fin(G->out[j])) { e = -100; break; }
    if (e) { P->fails++; P->err = e; continue; }
    bus_pub(B, P->out, G->out, n); P->runs++;
  }
}
