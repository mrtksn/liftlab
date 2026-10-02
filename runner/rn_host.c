/* The flight controller's side of loading programs: see rn_host.h. */
#include "rn_host.h"
#if defined(__wasm__)
typedef __SIZE_TYPE__ size_t;
void *memcpy(void *d, const void *s, size_t n); void *memset(void *d, int c, size_t n);
int strncmp(const char *a, const char *b, size_t n);
#else
#include <string.h>
#endif

static int8_t fmap[3][RN_FN_MAX];            /* slot s: the index of built-in formula i in that slot's table */

static void event(rn_host *H, int code, const char *what) {
  H->last_event = code;
  if (what) { size_t i = 0; for (; i < RN_NAME - 1 && what[i]; i++) H->last_fn[i] = what[i]; H->last_fn[i] = 0; }
  if (H->event) H->event(H->event_ctx, code, what);
}

static int finite_all(const float *v, int n) { for (int i = 0; i < n; i++) if (!(v[i] - v[i] == 0.0f)) return 0; return 1; }

/* Floats one instance's memory takes in the pool: every field's flag and data. */
static int32_t inst_size(const rn_fn *f) { int32_t n = 0; for (int32_t k = 0; k < f->n_state; k++) n += 1 + f->state[k].size; return n; }

/* Where field k of formula j's instance `inst` lives in a slot: its flag and its data. */
static void field_ptrs(rn_slot *S, int j, int inst, int k, float **flag, float **data) {
  const rn_fn *f = &S->P.fn[j]; const rn_field *F = &f->state[k];
  if (inst == 0) { *flag = S->arena + F->flag; *data = F->size ? S->arena + F->addr : 0; return; }
  float *base = S->pool + S->inst_at[j] + (inst - 1) * inst_size(f);
  for (int32_t m = 0; m < k; m++) base += 1 + f->state[m].size;
  *flag = base; *data = base + 1;
}

/* Carry the memory of every formula and instance from slot b to slot a, field by field, by name and size. */
static void transfer(rn_host *H, int a, int b) {
  rn_slot *D = &H->slot[a], *S = &H->slot[b];
  for (int i = 0; i < RN_FN_MAX; i++) {
    int dj = fmap[a][i], sj = fmap[b][i]; if (dj < 0 || sj < 0) continue;
    const rn_fn *df = &D->P.fn[dj], *sf = &S->P.fn[sj];
    int n = H->n_inst[i] > 0 ? H->n_inst[i] : 1;
    for (int inst = 0; inst < n; inst++) for (int32_t k = 0; k < df->n_state; k++) {
      float *dflag, *ddata; field_ptrs(D, dj, inst, k, &dflag, &ddata); *dflag = 0;
      for (int32_t m = 0; m < sf->n_state; m++) {
        if (strncmp(df->state[k].name, sf->state[m].name, RN_SNAME) || df->state[k].size != sf->state[m].size) continue;
        float *sflag, *sdata; field_ptrs(S, sj, inst, m, &sflag, &sdata);
        *dflag = *sflag; if (df->state[k].size) memcpy(ddata, sdata, (size_t)df->state[k].size * sizeof(float));
        break;
      }
    }
  }
}

/* Map the built-in formulas into slot s, check they take and return the same, and lay out the instance pool. */
static int bind_slot(rn_host *H, int s) {
  rn_slot *S = &H->slot[s]; const rn_prog *B = &H->slot[0].P;
  uint32_t at = 0;
  for (int i = 0; i < RN_FN_MAX; i++) fmap[s][i] = -1;
  for (int j = 0; j < RN_FN_MAX; j++) S->inst_at[j] = -1;
  for (int32_t i = 0; i < B->n_fn; i++) {
    int j = rn_find(&S->P, B->fn[i].name); if (j < 0) return RN_E_SIGNATURE;
    const rn_fn *bf = &B->fn[i], *f = &S->P.fn[j];
    if (f->n_args != bf->n_args || f->ret_size != bf->ret_size) return RN_E_SIGNATURE;
    for (int32_t k = 0; k < f->n_args; k++) if (f->arg_size[k] != bf->arg_size[k] || (f->arg_addr[k] < 0) != (bf->arg_addr[k] < 0)) return RN_E_SIGNATURE;
    fmap[s][i] = (int8_t)j;
    if (H->n_inst[i] > 1) {
      uint32_t need = (uint32_t)((H->n_inst[i] - 1) * inst_size(f));
      if (at + need > S->pool_cap) return RN_E_INSTANCES;
      S->inst_at[j] = (int32_t)at; memset(S->pool + at, 0, need * sizeof(float)); at += need;
    }
  }
  return RN_OK;
}

int rn_host_init(rn_host *H, const uint8_t *builtin, uint32_t len,
                 float *arenas[3], uint32_t arena_cap, int32_t *codes[3], uint32_t code_cap, float *pools[3], uint32_t pool_cap) {
  void (*ev)(void *, int, const char *) = H->event, (*lk)(void *, int) = H->lock; void *ctx = H->event_ctx, *lctx = H->lock_ctx;
  memset(H, 0, sizeof *H); H->event = ev; H->event_ctx = ctx; H->lock = lk; H->lock_ctx = lctx;
  for (int s = 0; s < 3; s++) { rn_slot *S = &H->slot[s]; S->arena = arenas[s];   /* arenas[2] may be NULL: two slots */ S->code = codes[s]; S->arena_cap = arena_cap; S->code_cap = code_cap; S->pool = pools[s]; S->pool_cap = pool_cap; }
  H->act = 0; H->cand = -1; H->prev = -1; H->shadow_s = 1.0f; H->blend_s = 0.3f; H->selftest_tol = 1e-2f;
  rn_slot *B = &H->slot[0];
  int e = rn_load(&B->P, builtin, len, B->arena, arena_cap, B->code, code_cap); if (e) return e;
  float worst; e = rn_selftest(&B->P, builtin, len, H->selftest_tol, &worst); if (e) return e;
  B->loaded = 1;
  return bind_slot(H, 0);
}

int rn_host_find(const rn_host *H, const char *name) { return rn_find(&H->slot[0].P, name); }
int rn_host_in_size(const rn_host *H, int fn) { const rn_fn *f = &H->slot[0].P.fn[fn]; int n = 0; for (int32_t k = 0; k < f->n_args; k++) n += f->arg_size[k]; return n; }
int rn_host_out_size(const rn_host *H, int fn) { return H->slot[0].P.fn[fn].ret_size; }

int rn_host_instances(rn_host *H, const char *fn, int n) {
  int i = rn_host_find(H, fn); if (i < 0 || n < 1 || n > RN_HOST_INST_MAX) return RN_E_INSTANCES;
  if (H->act != 0 || H->cand >= 0) return RN_E_BUSY;            /* set up before any program is loaded */
  H->n_inst[i] = n;
  return bind_slot(H, 0);
}

static void lock(rn_host *H, int on) { if (H->lock) H->lock(H->lock_ctx, on); }

/* With two slots (no memory for a third), a loaded program that is flying hands over to the built-in one while the
 * next program loads into its slot. With a lock (two cores) the flight loop's rn_host_tick() does the handover. */
static void to_builtin(rn_host *H) {
  int from = H->act; if (from == 0) return;
  transfer(H, 0, from);
  lock(H, 1); H->act = 0; H->prev = -1; H->to_builtin = 0; lock(H, 0);
}
int rn_host_prepare(rn_host *H, const uint8_t *img, uint32_t len) {
  lock(H, 1);                                                    /* take a slot out of use */
  if (H->cand >= 0) { H->slot[H->cand].loaded = 0; H->cand = -1; H->phase = RN_PH_FLYING; }   /* a newer one replaces it */
  H->pending = 0;
  int two = !H->slot[2].arena;
  lock(H, 0);
  if (two && H->act == 1) {
    if (H->lock) { H->to_builtin = 1; while (H->act != 0) { } }   /* the flight loop hands over at its next tick */
    else to_builtin(H);
  }
  lock(H, 1);
  int s = two ? 1 : H->act == 1 ? 2 : 1;
  if (H->prev == s) H->prev = -1;
  rn_slot *S = &H->slot[s]; S->loaded = 0;
  lock(H, 0);
  while (H->lock && H->in_call) { }                              /* a call still running on that slot finishes first */
  int e = rn_load(&S->P, img, len, S->arena, S->arena_cap, S->code, S->code_cap);
  float worst = 0;
  if (!e) e = rn_selftest(&S->P, img, len, H->selftest_tol, &worst);
  if (!e) e = bind_slot(H, s);
  if (e) { event(H, RN_EV_REJECTED, rn_error_text(e)); return e; }
  lock(H, 1); S->loaded = 1; H->pending = s; lock(H, 0);
  return RN_OK;
}
/* On the flight loop's core: give the prepared program the flying one's memory and start its background run.
 * The copy runs outside the lock: the prepared slot is nobody else's, and the flying one is this core's. */
static void start_pending(rn_host *H) {
  lock(H, 1); int s = H->pending; H->pending = 0; int ok = s > 0 && H->slot[s].loaded && s != H->act; lock(H, 0);
  if (!ok) return;
  transfer(H, s, H->act);
  lock(H, 1); H->cand = s; H->phase = RN_PH_SHADOW; H->t = 0; H->max_diff = 0; H->max_diff_fn = -1; lock(H, 0);
  event(H, RN_EV_LOADED, 0);
}
int rn_host_stage(rn_host *H, const uint8_t *img, uint32_t len) {
  int e = rn_host_prepare(H, img, len); if (e) return e;
  start_pending(H);
  return RN_OK;
}

void rn_host_tick(rn_host *H, float dt) {
  if (H->to_builtin) to_builtin(H);
  if (H->pending) start_pending(H);
  if (H->cand < 0 || H->phase == RN_PH_FLYING) return;
  H->t += dt;
  if (H->phase == RN_PH_SHADOW && H->t >= H->shadow_s) H->phase = RN_PH_BLEND;
  if (H->phase == RN_PH_BLEND && H->t >= H->shadow_s + H->blend_s) {
    lock(H, 1); int ok = H->cand >= 0; if (ok) { H->prev = H->act; H->act = H->cand; H->cand = -1; H->phase = RN_PH_FLYING; } lock(H, 0);
    if (ok) event(H, RN_EV_SWAPPED, 0);
  }
}

/* One call on one slot: the instance's memory in, inputs in, run, result out, memory back. */
static float swap_buf[512];
static int run_on(rn_host *H, int s, int fn, int inst, const float *in, float *out) {
  rn_slot *S = &H->slot[s]; int j = fmap[s][fn]; if (j < 0) return RN_T_NO_FN;
  const rn_fn *f = &S->P.fn[j];
  int32_t isz = inst_size(f);
  if (inst > 0) {                                                /* park instance 0, bring this one in */
    if (inst >= (H->n_inst[fn] > 0 ? H->n_inst[fn] : 1) || isz > (int32_t)(sizeof swap_buf / sizeof *swap_buf)) return RN_T_NO_FN;
    float *p = swap_buf;
    for (int32_t k = 0; k < f->n_state; k++) { float *fl, *d, *fl2, *d2; field_ptrs(S, j, 0, k, &fl, &d); field_ptrs(S, j, inst, k, &fl2, &d2);
      *p++ = *fl; if (f->state[k].size) { memcpy(p, d, (size_t)f->state[k].size * 4); p += f->state[k].size; }
      *fl = *fl2; if (f->state[k].size) memcpy(d, d2, (size_t)f->state[k].size * 4); }
  }
  for (int32_t k = 0, at = 0; k < f->n_args; k++) if (f->arg_size[k]) { memcpy(S->arena + f->arg_addr[k], in + at, (size_t)f->arg_size[k] * 4); at += f->arg_size[k]; }
  int e = rn_run(&S->P, j);
  if (!e) memcpy(out, S->arena + f->ret_addr, (size_t)f->ret_size * 4);
  if (inst > 0) {                                                /* this instance's memory back, instance 0 back in */
    const float *p = swap_buf;
    for (int32_t k = 0; k < f->n_state; k++) { float *fl, *d, *fl2, *d2; field_ptrs(S, j, 0, k, &fl, &d); field_ptrs(S, j, inst, k, &fl2, &d2);
      *fl2 = *fl; if (f->state[k].size) memcpy(d2, d, (size_t)f->state[k].size * 4);
      *fl = *p++; if (f->state[k].size) { memcpy(d, p, (size_t)f->state[k].size * 4); p += f->state[k].size; } }
  }
  return e;
}

static int call_(rn_host *H, int fn, int inst, const float *in, float *out);
int rn_host_call(rn_host *H, int fn, int inst, const float *in, float *out) {
  lock(H, 1); H->in_call++; lock(H, 0);          /* a count: the flight loop and the learning may call at once */
  int e = call_(H, fn, inst, in, out);
  lock(H, 1); H->in_call--; lock(H, 0);
  return e;
}
static int call_(rn_host *H, int fn, int inst, const float *in, float *out) {
  int e = run_on(H, H->act, fn, inst, in, out);
  while (e) {                                                    /* the flying program trapped: fall back */
    const char *name = H->slot[0].P.fn[fn].name;
    lock(H, 1);
    int from = H->act, to = H->prev >= 0 ? H->prev : from != 0 ? 0 : -1;
    if (to >= 0) {
      if (H->cand >= 0) { H->slot[H->cand].loaded = 0; H->cand = -1; H->phase = RN_PH_FLYING; }
      H->act = to; H->prev = -1;
    }
    lock(H, 0);
    if (to < 0) { event(H, RN_EV_BUILTIN_FAILED, name); return e; }
    transfer(H, to, from);                                       /* rn_host_prepare waits for this call before reusing `from` */
    H->slot[from].loaded = 0;
    event(H, RN_EV_FELL_BACK, name);
    e = run_on(H, H->act, fn, inst, in, out);
  }
  int cand = H->cand;
  if (cand >= 0 && H->phase != RN_PH_FLYING) {
    int n = H->slot[0].P.fn[fn].ret_size;
    float shadow_out[n > 0 ? n : 1];                              /* on this caller's stack: two cores may shadow at once */
    int ec = run_on(H, cand, fn, inst, in, shadow_out);
    if (ec || !finite_all(shadow_out, n)) {
      lock(H, 1); if (H->cand == cand) { H->slot[cand].loaded = 0; H->cand = -1; H->phase = RN_PH_FLYING; } lock(H, 0);
      event(H, RN_EV_REJECTED, H->slot[0].P.fn[fn].name);
      return RN_OK;
    }
    float w = H->phase == RN_PH_BLEND ? (H->t - H->shadow_s) / H->blend_s : 0; if (w > 1) w = 1;
    for (int i = 0; i < n; i++) {
      float d = shadow_out[i] - out[i], a = d < 0 ? -d : d;
      if (a > H->max_diff) { H->max_diff = a; H->max_diff_fn = fn; }
      out[i] += d * w;
    }
  }
  return RN_OK;
}

float *rn_host_field(rn_host *H, int fn, const char *field, int32_t *size) {
  rn_slot *S = &H->slot[H->act]; int j = fmap[H->act][fn]; if (j < 0) return 0;
  const rn_fn *f = &S->P.fn[j];
  for (int32_t k = 0; k < f->n_state; k++) if (!strncmp(f->state[k].name, field, RN_SNAME)) {
    if (!f->state[k].size || !S->arena[f->state[k].flag]) return 0;
    *size = f->state[k].size; return S->arena + f->state[k].addr;
  }
  return 0;
}

void rn_host_forget(rn_host *H, int fn) {
  if (fn < 0 || fn >= RN_FN_MAX) return;
  for (int s = 0; s < 3; s++) {
    rn_slot *S = &H->slot[s]; int j = fmap[s][fn]; if (!S->loaded || j < 0) continue;
    const rn_fn *f = &S->P.fn[j]; int n = H->n_inst[fn] > 0 ? H->n_inst[fn] : 1;
    for (int inst = 0; inst < n; inst++) for (int32_t k = 0; k < f->n_state; k++) { float *fl, *d; field_ptrs(S, j, inst, k, &fl, &d); *fl = 0; }
  }
}
