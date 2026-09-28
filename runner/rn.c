/* Drone Force Bench step runner: loader, checker, step loop and the built-in kernels. See rn.h. */
#include "rn.h"
#if defined(__wasm__)
typedef __SIZE_TYPE__ size_t;
void *memcpy(void *d, const void *s, size_t n); void *memmove(void *d, const void *s, size_t n); void *memset(void *d, int c, size_t n);
size_t strlen(const char *s); int strncmp(const char *a, const char *b, size_t n);
#else
#include <string.h>
#endif

/* ── math: the platform's float functions; in WebAssembly, the page's Math (imported) ── */
#if defined(__wasm__)
#define RN_IMPORT(n) __attribute__((import_module("env"), import_name(n)))
RN_IMPORT("sin") double rn_js_sin(double); RN_IMPORT("cos") double rn_js_cos(double); RN_IMPORT("tan") double rn_js_tan(double);
RN_IMPORT("asin") double rn_js_asin(double); RN_IMPORT("acos") double rn_js_acos(double); RN_IMPORT("atan") double rn_js_atan(double);
RN_IMPORT("atan2") double rn_js_atan2(double, double); RN_IMPORT("exp") double rn_js_exp(double); RN_IMPORT("log") double rn_js_log(double);
RN_IMPORT("pow") double rn_js_pow(double, double);
#define SINF(x) ((float)rn_js_sin(x))
#define COSF(x) ((float)rn_js_cos(x))
#define TANF(x) ((float)rn_js_tan(x))
#define ASINF(x) ((float)rn_js_asin(x))
#define ACOSF(x) ((float)rn_js_acos(x))
#define ATANF(x) ((float)rn_js_atan(x))
#define ATAN2F(y, x) ((float)rn_js_atan2(y, x))
#define EXPF(x) ((float)rn_js_exp(x))
#define LOGF(x) ((float)rn_js_log(x))
#define POWF(x, y) ((float)rn_js_pow(x, y))
#define SQRTF(x) __builtin_sqrtf(x)
#define FABSF(x) __builtin_fabsf(x)
#define FLOORF(x) __builtin_floorf(x)
#define CEILF(x) __builtin_ceilf(x)
static float FMODF(float x, float y) { double q = (double)x / (double)y; return (float)((double)x - __builtin_trunc(q) * (double)y); }
#else
#include <math.h>
#define SINF sinf
#define COSF cosf
#define TANF tanf
#define ASINF asinf
#define ACOSF acosf
#define ATANF atanf
#define ATAN2F atan2f
#define EXPF expf
#define LOGF logf
#define POWF powf
#define SQRTF sqrtf
#define FABSF fabsf
#define FLOORF floorf
#define CEILF ceilf
#define FMODF fmodf
#endif

#define TRUTHY(x) ((x) != 0.0f && (x) == (x))
#define ISNAN(x) ((x) != (x))

static float js_min(float a, float b) { return ISNAN(a) || ISNAN(b) ? (a + b) : (a < b ? a : b); }
static float js_max(float a, float b) { return ISNAN(a) || ISNAN(b) ? (a + b) : (a > b ? a : b); }
static float js_sign(float x) { return x > 0 ? 1.0f : x < 0 ? -1.0f : x; }
static float hypot3(float x, float y, float z) { return SQRTF(x * x + y * y + z * z); }

/* ── image reading ── */
typedef struct { const uint8_t *p; uint32_t n, at; int bad; } rd;
static uint32_t rd_u32(rd *r) { if (r->at + 4 > r->n) { r->bad = 1; return 0; } uint32_t v; memcpy(&v, r->p + r->at, 4); r->at += 4; return v; }

uint32_t rn_crc32(const uint8_t *p, uint32_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < n; i++) { c ^= p[i]; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u))); }
  return ~c;
}

static int region_ok(int32_t a, int32_t n, int32_t lo, int32_t hi) { return n >= 0 && a >= lo && a <= hi && n <= hi - a; }

static int block_size(const int32_t *ops, const rn_blk *b, int32_t *size) {
  switch (b->kind) {
    case 'n': *size = b->x; return 1;
    case '$': *size = ops[b->x]; return 1;
    case 'N': *size = 1 + ops[b->x]; return 1;
    case 'L': *size = 1 + ops[b->y] * ops[b->x]; return 1;
    case 'R': *size = 2 + ops[b->y] * ops[b->x]; return 1;
  }
  return 0;
}

/* Check one formula's steps. */
static int verify_fn(rn_prog *P, const rn_fn *f) {
  const int32_t *c = P->code, n = P->arena_size, ce = P->const_end;
  /* pass 1: where steps start */
  static uint32_t start_bits[RN_CODE_MAX / 32];          /* where steps start, one bit per code word */
#define START(pc) (start_bits[(pc) >> 5] >> ((pc) & 31) & 1u)
  if (f->end > RN_CODE_MAX) return RN_E_TOO_BIG;
  for (int32_t pc = f->entry; pc < f->end;) {
    int32_t op = c[pc];
    if (op < 0 || op >= RN_NOPS) { P->trap_pc = pc; return RN_E_BAD_OP; }
    start_bits[pc >> 5] |= 1u << (pc & 31); pc += 1 + (int32_t)strlen(rn_op_spec[op]);
    if (pc > f->end) { P->trap_pc = pc; return RN_E_BAD_OP; }
  }
  int err = RN_OK;
  for (int32_t pc = f->entry; pc < f->end && !err;) {
    int32_t op = c[pc]; const char *s = rn_op_spec[op]; const int32_t *a = c + pc + 1; int32_t k = (int32_t)strlen(s);
    for (int32_t i = 0; i < k && !err; i++) {
      int32_t v = a[i];
      switch (s[i]) {
        case 'd': case 'r': if (v < ce || v >= n) err = v >= 0 && v < ce ? RN_E_WRITES_CONST : RN_E_BAD_ADDR; break;
        case 'a': case 'R': if (v < 0 || v >= n) err = RN_E_BAD_ADDR; break;
        case 'o': case 'g': if (v != -1 && (v < 0 || v >= n)) err = RN_E_BAD_ADDR; break;
        case 't': if (v < f->entry || v > f->end || (v < f->end && !START(v))) err = RN_E_BAD_JUMP; break;
      }
    }
    for (int b = 0; b < RN_NBLK && !err; b++) {
      const rn_blk *B = &rn_op_blk[b]; if (B->op != op) continue;
      int32_t base = a[(int)B->operand], size;
      if (base == -1 && s[(int)B->operand] == 'o') continue;
      if (!block_size(a, B, &size) || size < 0 || base < 0 || base + size > n) err = RN_E_BAD_BLOCK;
      else if (B->write && base < ce) err = RN_E_WRITES_CONST;
    }
    if (op == RN_BLS && (a[9] <= 0 || a[9] > 16 || a[10] > RN_BLS_MAX)) err = RN_E_BAD_BLOCK;
    for (int v = 0; v < RN_NVIEW && !err; v++) {             /* fused list steps: views without a register */
      const rn_view *V = &rn_op_view[v]; if (V->op != op || a[(int)V->reg] != -1) continue;
      int32_t cap = a[k - 1], off = a[(int)V->off], st = a[(int)V->stride];
      int64_t last = (int64_t)off + (int64_t)st * (cap > 0 ? cap - 1 : 0);
      if (st < 0 || off < 0 || last >= n) err = RN_E_BAD_BLOCK; else if (V->write && off < ce) err = RN_E_WRITES_CONST;
    }
    if (err) P->trap_pc = pc;
    pc += 1 + k;
  }
  for (int32_t pc = f->entry; pc < f->end; pc++) start_bits[pc >> 5] &= ~(1u << (pc & 31));
  return err;
}

int rn_load(rn_prog *P, const uint8_t *img, uint32_t len, float *arena, uint32_t arena_cap, int32_t *code, uint32_t code_cap) {
  memset(P, 0, sizeof(*P));
  if (len < 32) return RN_E_SIZE;
  uint32_t crc; memcpy(&crc, img + len - 4, 4);
  if (rn_crc32(img, len - 4) != crc) return RN_E_CRC;
  rd r = { img, len - 4, 0, 0 };
  if (rd_u32(&r) != RN_MAGIC) return RN_E_MAGIC;
  if (rd_u32(&r) != RN_VERSION) return RN_E_VERSION;
  uint32_t asz = rd_u32(&r), ce = rd_u32(&r), clen = rd_u32(&r), nfn = rd_u32(&r), ntest = rd_u32(&r);
  if (r.bad || asz > arena_cap || clen > code_cap || ce > asz || nfn > RN_FN_MAX) return RN_E_TOO_BIG;
  if (r.at + 4 * (ce + clen) > r.n) return RN_E_SIZE;
  if (clen > RN_CODE_MAX) return RN_E_TOO_BIG;
  memset(arena, 0, asz * sizeof(float));
  memcpy(arena, img + r.at, ce * 4); r.at += ce * 4;
  if (code) memcpy(code, img + r.at, clen * 4);
  else if (((uintptr_t)(img + r.at) & 3) == 0) code = (int32_t *)(uintptr_t)(img + r.at);   /* run the steps where they are (flash) */
  else return RN_E_SIZE;
  r.at += clen * 4;
  P->arena = arena; P->arena_size = (int32_t)asz; P->const_end = (int32_t)ce; P->code = code; P->code_len = (int32_t)clen;
  for (uint32_t i = 0; i < nfn; i++) {
    rn_fn *f = &P->fn[i];
    if (r.at + RN_NAME > r.n) return RN_E_SIZE;
    memcpy(f->name, img + r.at, RN_NAME); f->name[RN_NAME - 1] = 0; r.at += RN_NAME;
    f->entry = (int32_t)rd_u32(&r); f->end = (int32_t)rd_u32(&r); f->max_steps = (int32_t)rd_u32(&r);
    if (r.bad || f->entry < 0 || f->end < f->entry || f->end > (int32_t)clen || f->max_steps <= 0) return RN_E_TABLE;
    f->n_args = (int32_t)rd_u32(&r); if (r.bad || f->n_args < 0 || f->n_args > RN_ARGS_MAX) return RN_E_TABLE;
    for (int32_t k = 0; k < f->n_args; k++) {
      f->arg_addr[k] = (int32_t)rd_u32(&r); f->arg_size[k] = (int32_t)rd_u32(&r);
      if (!(f->arg_addr[k] == -1 && f->arg_size[k] == 0) && !region_ok(f->arg_addr[k], f->arg_size[k], (int32_t)ce, (int32_t)asz)) return RN_E_TABLE;
    }
    f->ret_addr = (int32_t)rd_u32(&r); f->ret_size = (int32_t)rd_u32(&r);
    if (!region_ok(f->ret_addr, f->ret_size, (int32_t)ce, (int32_t)asz)) return RN_E_TABLE;
    f->n_state = (int32_t)rd_u32(&r); if (r.bad || f->n_state < 0 || f->n_state > RN_STATE_MAX) return RN_E_TABLE;
    for (int32_t k = 0; k < f->n_state; k++) {
      rn_field *F = &f->state[k];
      F->flag = (int32_t)rd_u32(&r); F->addr = (int32_t)rd_u32(&r); F->size = (int32_t)rd_u32(&r);
      if (r.at + RN_SNAME > r.n) return RN_E_SIZE;
      memcpy(F->name, img + r.at, RN_SNAME); F->name[RN_SNAME - 1] = 0; r.at += RN_SNAME;
      if (!region_ok(F->flag, 1, (int32_t)ce, (int32_t)asz)) return RN_E_TABLE;
      if (!(F->addr == -1 && F->size == 0) && !region_ok(F->addr, F->size, (int32_t)ce, (int32_t)asz)) return RN_E_TABLE;
    }
  }
  /* self-tests: (formula, input regions, output regions); a region list ends with (0, 0) */
  P->tests_at = r.at; P->n_tests = (int32_t)ntest;
  for (uint32_t t = 0; t < ntest; t++) {
    uint32_t fi = rd_u32(&r); if (r.bad || fi >= nfn) return RN_E_TESTS;
    for (int part = 0; part < 2; part++) for (;;) {
      int32_t a = (int32_t)rd_u32(&r), n = (int32_t)rd_u32(&r);
      if (r.bad) return RN_E_TESTS;
      if (!n) break;
      if (!region_ok(a, n, part ? 0 : (int32_t)ce, (int32_t)asz) || r.at + 4u * (uint32_t)n > r.n) return RN_E_TESTS;
      r.at += 4u * (uint32_t)n;
    }
  }
  if (r.bad || r.at != r.n) return RN_E_SIZE;
  P->n_fn = (int32_t)nfn;
  for (int32_t i = 0; i < P->n_fn; i++) { int e = verify_fn(P, &P->fn[i]); if (e) return e; }
  return RN_OK;
}

void rn_clear(rn_prog *P) { memset(P->arena + P->const_end, 0, (size_t)(P->arena_size - P->const_end) * sizeof(float)); }

int rn_selftest(rn_prog *P, const uint8_t *img, uint32_t len, float tol, float *worst) {
  rd r = { img, len - 4, P->tests_at, 0 };
  *worst = 0;
  int err = RN_OK;
  for (int32_t t = 0; t < P->n_tests && !err; t++) {
    int32_t fi = (int32_t)rd_u32(&r);
    rn_clear(P);
    for (;;) { int32_t a = (int32_t)rd_u32(&r), n = (int32_t)rd_u32(&r); if (!n) break; memcpy(P->arena + a, img + r.at, (size_t)n * 4); r.at += 4u * (uint32_t)n; }
    int e = rn_run(P, fi);
    float w = 0;
    for (;;) {
      int32_t a = (int32_t)rd_u32(&r), n = (int32_t)rd_u32(&r); if (!n) break;
      const uint8_t *exp = img + r.at; float scale = 0;
      for (int32_t i = 0; i < n; i++) { float y; memcpy(&y, exp + 4 * i, 4); if (FABSF(y) > scale) scale = FABSF(y); }
      for (int32_t i = 0; i < n && !e; i++) {
        float y, x = P->arena[a + i]; memcpy(&y, exp + 4 * i, 4);
        if (ISNAN(x) && ISNAN(y)) continue;
        float d = FABSF(x - y) / (FABSF(y) + 0.01f * scale + 1e-6f);
        if (!(d <= w)) w = d;
      }
      r.at += 4u * (uint32_t)n;
    }
    if (e) { err = e; P->trap_pc = t; }
    else { if (w > *worst) *worst = w; if (!(w <= tol)) { err = RN_E_SELFTEST; P->trap_pc = t; } }
  }
  rn_clear(P);
  return err;
}

int rn_transfer(rn_prog *dst, const rn_prog *src) {
  int copied = 0;
  for (int32_t i = 0; i < dst->n_fn; i++) {
    const rn_fn *df = &dst->fn[i]; int j = rn_find(src, df->name); if (j < 0) continue;
    const rn_fn *sf = &src->fn[j];
    for (int32_t k = 0; k < df->n_state; k++) {
      const rn_field *D = &df->state[k];
      dst->arena[D->flag] = 0;
      for (int32_t m = 0; m < sf->n_state; m++) {
        const rn_field *S = &sf->state[m];
        if (strncmp(D->name, S->name, RN_SNAME) || D->size != S->size) continue;
        dst->arena[D->flag] = src->arena[S->flag];
        if (D->size > 0) memcpy(dst->arena + D->addr, src->arena + S->addr, (size_t)D->size * sizeof(float));
        copied++; break;
      }
    }
  }
  return copied;
}

int rn_find(const rn_prog *P, const char *name) {
  for (int32_t i = 0; i < P->n_fn; i++) if (!strncmp(P->fn[i].name, name, RN_NAME)) return i;
  return -1;
}

const char *rn_error_text(int e) {
  switch (e) {
    case RN_OK: return "ok";
    case RN_E_MAGIC: return "not a step program"; case RN_E_VERSION: return "made for another runner version";
    case RN_E_SIZE: return "truncated or wrong size"; case RN_E_CRC: return "checksum mismatch";
    case RN_E_TOO_BIG: return "too big for this runner"; case RN_E_BAD_OP: return "unknown step";
    case RN_E_BAD_ADDR: return "address outside the arena"; case RN_E_WRITES_CONST: return "writes a constant";
    case RN_E_BAD_JUMP: return "jump outside the formula"; case RN_E_BAD_BLOCK: return "block outside the arena";
    case RN_E_TABLE: return "bad formula table"; case RN_E_TESTS: return "bad self-test section";
    case RN_E_SELFTEST: return "a self-test gave a different result";
    case RN_E_SIGNATURE: return "a formula takes or returns something else than the built-in one"; case RN_E_BUSY: return "busy";
    case RN_E_INSTANCES: return "not enough room for the formulas' instances";
    case RN_T_STEPS: return "took more steps than its limit"; case RN_T_INDEX: return "index outside the list";
    case RN_T_ADDR: return "computed address outside the arena"; case RN_T_LIST_FULL: return "list full";
    case RN_T_LIST_LEN: return "list length beyond its capacity"; case RN_T_FORMULA: return "stopped by the formula";
    case RN_T_BAD_OP: return "unknown step"; case RN_T_NO_FN: return "no such formula"; case RN_T_KERNEL: return "kernel limits";
  }
  return "?";
}

/* ── kernels ── */
static void k_m3m(float *d, const float *a, const float *b) {
  float t[9];
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) t[3 * i + j] = a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j];
  memcpy(d, t, sizeof t);
}
static void k_qnorm(float *d, const float *q) {
  float m = SQRTF(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  float t0 = q[0] / m, t1 = q[1] / m, t2 = q[2] / m, t3 = q[3] / m; d[0] = t0; d[1] = t1; d[2] = t2; d[3] = t3;
}
static void k_m2q(float *d, const float *M) {
  float tr = M[0] + M[4] + M[8], w, x, y, z;
  if (tr > 0) { float s = SQRTF(tr + 1) * 2; w = 0.25f * s; x = (M[7] - M[5]) / s; y = (M[2] - M[6]) / s; z = (M[3] - M[1]) / s; }
  else if (M[0] > M[4] && M[0] > M[8]) { float s = SQRTF(1 + M[0] - M[4] - M[8]) * 2; w = (M[7] - M[5]) / s; x = 0.25f * s; y = (M[1] + M[3]) / s; z = (M[2] + M[6]) / s; }
  else if (M[4] > M[8]) { float s = SQRTF(1 + M[4] - M[0] - M[8]) * 2; w = (M[2] - M[6]) / s; x = (M[1] + M[3]) / s; y = 0.25f * s; z = (M[5] + M[7]) / s; }
  else { float s = SQRTF(1 + M[8] - M[0] - M[4]) * 2; w = (M[3] - M[1]) / s; x = (M[2] + M[6]) / s; y = (M[5] + M[7]) / s; z = 0.25f * s; }
  float q[4] = { w, x, y, z }; k_qnorm(d, q);
}

/* Bounded weighted least squares (math.js bls): the inputs x within [lo, hi] whose Σ cols·x best matches w,
 * weighted by W, plus a tiny regularization and optional pulls toward preferred values. Active set: the worst
 * violator is fixed at its bound and the rest solved again.
 *
 * math.js solves the normal equations (CᵀWC + Λ) x = CᵀW r in 64-bit floats. In 32-bit floats that fails: the
 * weights span 1e7 (lift and tilt held with ×1e3 while yaw gets ×1e-4) and squaring the conditioning loses the
 * yaw direction entirely, and the 1e-8 regularization vanishes below float precision. So this solves the same
 * minimization,  min ‖√W (C x − r)‖² + Σ (λⱼ + qⱼ)(xⱼ − qⱼ rⱼ / (λⱼ + qⱼ))²,  by Householder QR on the stacked
 * system [√W C; √(Λ+Q)], which never squares it. Same answer in exact arithmetic; holds up in floats. */
#define RN_BLS_ROWS (16 + RN_BLS_MAX)
static float bls_M[RN_BLS_ROWS * (RN_BLS_MAX + 1)];
static void qr_solve(float *M, int rows, int m, float *x) {
  const int W = m + 1;
  for (int c = 0; c < m; c++) {
    float nrm = 0; for (int r = c; r < rows; r++) nrm += M[r * W + c] * M[r * W + c];
    nrm = SQRTF(nrm);
    if (nrm < 1e-30f) continue;
    float alpha = M[c * W + c] > 0 ? -nrm : nrm;           /* reflect onto −sign·‖v‖ e */
    float v0 = M[c * W + c] - alpha;
    float vnorm2 = v0 * v0; for (int r = c + 1; r < rows; r++) vnorm2 += M[r * W + c] * M[r * W + c];
    if (vnorm2 < 1e-30f) { M[c * W + c] = alpha; continue; }
    for (int k = c + 1; k < W; k++) {                       /* apply I − 2vvᵀ/vᵀv to the remaining columns and the right side */
      float dot = v0 * M[c * W + k]; for (int r = c + 1; r < rows; r++) dot += M[r * W + c] * M[r * W + k];
      float f = 2 * dot / vnorm2;
      M[c * W + k] -= f * v0; for (int r = c + 1; r < rows; r++) M[r * W + k] -= f * M[r * W + c];
    }
    M[c * W + c] = alpha; for (int r = c + 1; r < rows; r++) M[r * W + c] = 0;
  }
  for (int r = m - 1; r >= 0; r--) {
    float s = M[r * W + m];
    for (int k = r + 1; k < m; k++) s -= M[r * W + k] * x[k];
    x[r] = FABSF(M[r * W + r]) < 1e-30f ? 0 : s / M[r * W + r];
  }
}
static uint32_t bls_work;
static int k_bls(float *A, int32_t d, int32_t cl, int32_t lo, int32_t hi, int32_t w, int32_t Wt, int32_t pq, int32_t pr, int32_t rel, int K, int cap) {
  int n = (int)A[cl];
  if (n < 0 || n > cap || n > RN_BLS_MAX || K > 16) return RN_T_KERNEL;
  static float x[RN_BLS_MAX], y[RN_BLS_MAX], r[16], sw[16];
  static int F[RN_BLS_MAX]; static uint8_t fixed[RN_BLS_MAX];
  const float *C = A + cl + 1, *L = A + lo + 1, *U = A + hi + 1, *wv = A + w, *Wv = A + Wt;
  float relv = A[rel];
  int nq = pq >= 0 ? (int)A[pq] : 0, nr = pr >= 0 ? (int)A[pr] : 0;
  for (int k = 0; k < K; k++) sw[k] = SQRTF(Wv[k] > 0 ? Wv[k] : 0);
  for (int i = 0; i < n; i++) { x[i] = 0; fixed[i] = 0; }
  for (int iter = 0; iter <= n; iter++) {
    int m = 0; for (int i = 0; i < n; i++) if (!fixed[i]) F[m++] = i;
    if (!m) break;
    for (int k = 0; k < K; k++) r[k] = wv[k];
    for (int i = 0; i < n; i++) if (fixed[i]) for (int k = 0; k < K; k++) r[k] -= C[i * K + k] * x[i];
    const int WW = m + 1, rows = K + m;
    float *M = bls_M;
    float meanEff = 0;
    for (int a = 0; a < m; a++) {
      const float *ca = C + F[a] * K; float h = 0;
      for (int k = 0; k < K; k++) { float v = sw[k] * ca[k]; M[k * WW + a] = v; h += v * v; }
      float sp = U[F[a]] - L[F[a]]; if (sp == 0) sp = 1; meanEff += h * sp * sp;
    }
    for (int k = 0; k < K; k++) M[k * WW + m] = sw[k] * r[k];
    meanEff /= (float)m;
    float lam = 1e-8f * meanEff + 1e-12f;
    for (int a = 0; a < m; a++) {
      int j = F[a]; float sp = U[j] - L[j]; if (sp == 0) sp = 1;
      float q = 0, rj = 0;
      if (pq >= 0) {
        float qj = j < nq ? A[pq + 1 + j] : 0; if (!TRUTHY(qj)) qj = 0;
        rj = j < nr ? A[pr + 1 + j] : 0; if (!TRUTHY(rj)) rj = 0;
        q = qj * relv * meanEff / (sp * sp);
      }
      float dg = lam / (sp * sp) + q, sd = SQRTF(dg);
      float *row = M + (K + a) * WW;
      for (int b = 0; b < m; b++) row[b] = 0;
      row[a] = sd; row[m] = sd > 0 ? q * rj / sd : 0;
    }
    qr_solve(M, rows, m, y);
    bls_work += (uint32_t)(K * m + 2 * rows * m * (m + 1) + m * m);
    int worst = -1; float wv_ = 1e-9f;
    for (int a = 0; a < m; a++) {
      int j = F[a]; float sp = U[j] - L[j]; if (sp == 0) sp = 1;
      float viol = js_max(L[j] - y[a], y[a] - U[j]) / sp;
      if (viol > wv_) { wv_ = viol; worst = a; }
    }
    if (worst < 0 || iter == n) { for (int a = 0; a < m; a++) { int j = F[a]; float v = y[a]; x[j] = v < L[j] ? L[j] : v > U[j] ? U[j] : v; } break; }
    int j = F[worst]; x[j] = y[worst] < L[j] ? L[j] : U[j]; fixed[j] = 1;
  }
  A[d] = (float)n; for (int i = 0; i < n; i++) A[d + 1 + i] = x[i];
  return RN_OK;
}

/* ── the step loop ── */
#define IS_INT(v) ((float)(int32_t)(v) == (v))
int rn_run(rn_prog *P, int32_t fi) {
  if (fi < 0 || fi >= P->n_fn) return RN_T_NO_FN;
  const rn_fn *f = &P->fn[fi];
  float *A = P->arena; const int32_t *c = P->code, n = P->arena_size, ce = P->const_end;
  int32_t pc = f->entry, steps = 0; const int32_t end = f->end, max = f->max_steps;
  bls_work = 0;
  int err = RN_OK;
#define TRAP(e) do { err = (e); goto trap; } while (0)
#define CHK(p) do { if ((p) < 0 || (p) >= n) TRAP(RN_T_ADDR); } while (0)
#define WCHK(p) do { if ((p) < ce || (p) >= n) TRAP(RN_T_ADDR); } while (0)
  while (pc < end) {
    if (++steps > max) TRAP(RN_T_STEPS);
    const int32_t *o = c + pc + 1;
    switch (c[pc]) {
      case RN_NOP: pc += 1; break;
      case RN_MOV: A[o[0]] = A[o[1]]; pc += 3; break;
      case RN_ADD: A[o[0]] = A[o[1]] + A[o[2]]; pc += 4; break;
      case RN_SUB: A[o[0]] = A[o[1]] - A[o[2]]; pc += 4; break;
      case RN_MUL: A[o[0]] = A[o[1]] * A[o[2]]; pc += 4; break;
      case RN_DIV: A[o[0]] = A[o[1]] / A[o[2]]; pc += 4; break;
      case RN_MOD: A[o[0]] = FMODF(A[o[1]], A[o[2]]); pc += 4; break;
      case RN_POW: A[o[0]] = POWF(A[o[1]], A[o[2]]); pc += 4; break;
      case RN_MIN: A[o[0]] = js_min(A[o[1]], A[o[2]]); pc += 4; break;
      case RN_MAX: A[o[0]] = js_max(A[o[1]], A[o[2]]); pc += 4; break;
      case RN_ATAN2: A[o[0]] = ATAN2F(A[o[1]], A[o[2]]); pc += 4; break;
      case RN_LT: A[o[0]] = A[o[1]] < A[o[2]] ? 1.0f : 0.0f; pc += 4; break;
      case RN_LE: A[o[0]] = A[o[1]] <= A[o[2]] ? 1.0f : 0.0f; pc += 4; break;
      case RN_EQ: A[o[0]] = A[o[1]] == A[o[2]] ? 1.0f : 0.0f; pc += 4; break;
      case RN_NE: A[o[0]] = A[o[1]] != A[o[2]] ? 1.0f : 0.0f; pc += 4; break;
      case RN_NEG: A[o[0]] = -A[o[1]]; pc += 3; break;
      case RN_ABS: A[o[0]] = FABSF(A[o[1]]); pc += 3; break;
      case RN_SQRT: A[o[0]] = SQRTF(A[o[1]]); pc += 3; break;
      case RN_SIN: A[o[0]] = SINF(A[o[1]]); pc += 3; break;
      case RN_COS: A[o[0]] = COSF(A[o[1]]); pc += 3; break;
      case RN_TAN: A[o[0]] = TANF(A[o[1]]); pc += 3; break;
      case RN_ASIN: A[o[0]] = ASINF(A[o[1]]); pc += 3; break;
      case RN_ACOS: A[o[0]] = ACOSF(A[o[1]]); pc += 3; break;
      case RN_ATAN: A[o[0]] = ATANF(A[o[1]]); pc += 3; break;
      case RN_EXP: A[o[0]] = EXPF(A[o[1]]); pc += 3; break;
      case RN_LOG: A[o[0]] = LOGF(A[o[1]]); pc += 3; break;
      case RN_FLOOR: A[o[0]] = FLOORF(A[o[1]]); pc += 3; break;
      case RN_CEIL: A[o[0]] = CEILF(A[o[1]]); pc += 3; break;
      case RN_ROUND: A[o[0]] = FLOORF(A[o[1]] + 0.5f); pc += 3; break;
      case RN_SIGN: A[o[0]] = js_sign(A[o[1]]); pc += 3; break;
      case RN_NOT: A[o[0]] = TRUTHY(A[o[1]]) ? 0.0f : 1.0f; pc += 3; break;
      case RN_TRUTH: A[o[0]] = TRUTHY(A[o[1]]) ? 1.0f : 0.0f; pc += 3; break;
      case RN_SEL: A[o[0]] = TRUTHY(A[o[1]]) ? A[o[2]] : A[o[3]]; pc += 5; break;
      case RN_CLAMP: { float x = A[o[1]], lo = A[o[2]], hi = A[o[3]]; A[o[0]] = x < lo ? lo : x > hi ? hi : x; pc += 5; break; }
      case RN_FMA: A[o[0]] = A[o[1]] * A[o[2]] + A[o[3]]; pc += 5; break;
      case RN_JMP: pc = o[0]; break;
      case RN_JZ: pc = TRUTHY(A[o[0]]) ? pc + 3 : o[1]; break;
      case RN_JNZ: pc = TRUTHY(A[o[0]]) ? o[1] : pc + 3; break;
      case RN_TRAP: TRAP(RN_T_FORMULA);
      case RN_CPY: memmove(A + o[0], A + o[1], (size_t)o[2] * 4); pc += 4; break;
      case RN_FILL: { float v = A[o[1]]; for (int32_t k = 0; k < o[2]; k++) A[o[0] + k] = v; pc += 4; break; }
      case RN_LLEN: { float len = A[o[1]]; if (!(len >= 0 && len <= (float)o[2] && IS_INT(len))) TRAP(RN_T_LIST_LEN); A[o[0]] = len; pc += 5; break; }
      case RN_LFILL: { int32_t len = (int32_t)A[o[0]]; if (len < 0 || len > o[3]) TRAP(RN_T_LIST_LEN); float v = A[o[1]]; for (int32_t k = 0; k < len * o[2]; k++) A[o[0] + 1 + k] = v; pc += 5; break; }
      case RN_CPYL: { float len = A[o[1]]; if (!(len >= 0 && len <= (float)o[3])) TRAP(RN_T_LIST_LEN); memmove(A + o[0] + 1, A + o[1] + 1, (size_t)((int32_t)len * o[2]) * 4); A[o[0]] = len; pc += 5; break; }
      case RN_PUSH: { int32_t len = (int32_t)A[o[0]]; if (len < 0 || len >= o[3]) TRAP(RN_T_LIST_FULL); memmove(A + o[0] + 1 + len * o[2], A + o[1], (size_t)o[2] * 4); A[o[0]] = (float)(len + 1); pc += 5; break; }
      case RN_RPUSH: { int32_t len = (int32_t)A[o[0]], head = (int32_t)A[o[0] + 1]; if (len < 0 || len >= o[3] || head < 0 || head >= o[3]) TRAP(RN_T_LIST_FULL); int32_t at = (head + len) % o[3]; memmove(A + o[0] + 2 + at * o[2], A + o[1], (size_t)o[2] * 4); A[o[0]] = (float)(len + 1); pc += 5; break; }
      case RN_RSHIFT: { int32_t len = (int32_t)A[o[0]]; if (len > 0) { A[o[0]] = (float)(len - 1); A[o[0] + 1] = (float)(((int32_t)A[o[0] + 1] + 1) % o[2]); } pc += 4; break; }
      case RN_RCLR: A[o[0]] = 0; A[o[0] + 1] = 0; pc += 4; break;
      case RN_IDX: { float i = A[o[2]], b = A[o[3]]; if (!(i >= 0 && i < b && IS_INT(i))) TRAP(RN_T_INDEX); A[o[0]] = (float)(o[1] + (int32_t)i * o[4]); pc += 6; break; }
      case RN_IDXI: { float i = A[o[3]], b = A[o[4]]; if (!(i >= 0 && i < b && IS_INT(i))) TRAP(RN_T_INDEX); A[o[0]] = (float)((int32_t)A[o[1]] + o[2] + (int32_t)i * o[5]); pc += 7; break; }
      case RN_RIDX: { float i = A[o[2]], len = A[o[1]]; if (!(i >= 0 && i < len && IS_INT(i))) TRAP(RN_T_INDEX); int32_t head = (int32_t)A[o[1] + 1]; A[o[0]] = (float)(o[1] + 2 + ((head + (int32_t)i) % o[4]) * o[3]); pc += 6; break; }
      case RN_LDI: { int32_t p = (int32_t)A[o[1]] + o[2]; CHK(p); A[o[0]] = A[p]; pc += 4; break; }
      case RN_STI: { int32_t p = (int32_t)A[o[0]] + o[1]; WCHK(p); A[p] = A[o[2]]; pc += 4; break; }
      case RN_CPI: { int32_t p = (int32_t)A[o[1]] + o[2]; CHK(p); CHK(p + o[3] - 1); memmove(A + o[0], A + p, (size_t)o[3] * 4); pc += 5; break; }
      case RN_CPO: { int32_t p = (int32_t)A[o[0]] + o[1]; WCHK(p); WCHK(p + o[3] - 1); memmove(A + p, A + o[2], (size_t)o[3] * 4); pc += 5; break; }
      case RN_AR: A[o[0]] = (float)o[1]; pc += 3; break;
      case RN_M3V: { const float *M = A + o[1], *v = A + o[2]; float x = v[0], y = v[1], z = v[2]; float *d = A + o[0];
        float r0 = M[0] * x + M[1] * y + M[2] * z, r1 = M[3] * x + M[4] * y + M[5] * z, r2 = M[6] * x + M[7] * y + M[8] * z; d[0] = r0; d[1] = r1; d[2] = r2; pc += 4; break; }
      case RN_M3M: k_m3m(A + o[0], A + o[1], A + o[2]); pc += 4; break;
      case RN_M3T: { float t[9]; const float *m = A + o[1]; for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) t[3 * i + j] = m[3 * j + i]; memcpy(A + o[0], t, sizeof t); pc += 3; break; }
      case RN_CRS: { const float *a = A + o[1], *b = A + o[2]; float x = a[1] * b[2] - a[2] * b[1], y = a[2] * b[0] - a[0] * b[2], z = a[0] * b[1] - a[1] * b[0]; float *d = A + o[0]; d[0] = x; d[1] = y; d[2] = z; pc += 4; break; }
      case RN_QMUL: { const float *a = A + o[1], *b = A + o[2]; float t[4] = { a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2], a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0] }; memcpy(A + o[0], t, sizeof t); pc += 4; break; }
      case RN_QMAT: { const float *q = A + o[1]; float w = q[0], x = q[1], y = q[2], z = q[3];
        float t[9] = { 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x), 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y) };
        memcpy(A + o[0], t, sizeof t); pc += 3; break; }
      case RN_QNORM: k_qnorm(A + o[0], A + o[1]); pc += 3; break;
      case RN_M2Q: k_m2q(A + o[0], A + o[1]); pc += 3; break;
      case RN_UNIT3: { const float *a = A + o[1]; float x = a[0], y = a[1], z = a[2], m = hypot3(x, y, z); float *d = A + o[0];
        if (m > 1e-12f) { d[0] = x / m; d[1] = y / m; d[2] = z / m; } else { d[0] = 0; d[1] = 0; d[2] = 1; } pc += 3; break; }
      case RN_NRM3: { const float *a = A + o[1]; A[o[0]] = hypot3(a[0], a[1], a[2]); pc += 3; break; }
      case RN_DOT3: { const float *a = A + o[1], *b = A + o[2]; A[o[0]] = a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; pc += 4; break; }
      case RN_ADD3: { const float *a = A + o[1], *b = A + o[2]; float x = a[0] + b[0], y = a[1] + b[1], z = a[2] + b[2]; float *d = A + o[0]; d[0] = x; d[1] = y; d[2] = z; pc += 4; break; }
      case RN_SUB3: { const float *a = A + o[1], *b = A + o[2]; float x = a[0] - b[0], y = a[1] - b[1], z = a[2] - b[2]; float *d = A + o[0]; d[0] = x; d[1] = y; d[2] = z; pc += 4; break; }
      case RN_SCL3: { const float *a = A + o[1]; float s = A[o[2]]; float x = a[0] * s, y = a[1] * s, z = a[2] * s; float *d = A + o[0]; d[0] = x; d[1] = y; d[2] = z; pc += 4; break; }
      case RN_BLS: { int e = k_bls(A, o[0], o[1], o[2], o[3], o[4], o[5], o[6], o[7], o[8], o[9], o[10]); if (e) TRAP(e); pc += 12; break; }
      case RN_VV: case RN_VS: case RN_VDOT: {                 /* fused list steps */
        const int op = c[pc];
        const int ci = op == RN_VV ? 10 : op == RN_VS ? 8 : 7;   /* operand index of the count */
        float cf = A[o[ci]]; int32_t cap = o[ci + 1];
        if (!(cf >= 0 && cf <= (float)cap)) TRAP(RN_T_LIST_LEN);
        int32_t cnt = (int32_t)cf, base[3], st[3]; int nv = op == RN_VV ? 3 : 2;
        for (int v = 0; v < nv; v++) {
          int32_t r = o[v * 3 + 1], off = o[v * 3 + 2], s_ = o[v * 3 + 3];
          int32_t b = (r >= 0 ? (int32_t)A[r] : 0) + off;
          if (r >= 0 && cnt > 0) { int32_t last = b + s_ * (cnt - 1); int w = op != RN_VDOT && v == 0; if (b < (w ? ce : 0) || last >= n) TRAP(RN_T_ADDR); }
          base[v] = b; st[v] = s_;
        }
        if (op == RN_VDOT) {
          float acc = 0; const float *x = A + base[0], *y = A + base[1];
          for (int32_t k = 0; k < cnt; k++) acc += x[k * st[0]] * y[k * st[1]];
          A[o[0]] = acc; pc += 10; break;
        }
        int kind = o[0]; float *d = A + base[0]; const float *x = A + base[1];
        if (op == RN_VV) {
          const float *y = A + base[2];
          switch (kind) {
            case RN_V_ADD: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = x[k * st[1]] + y[k * st[2]]; break;
            case RN_V_SUB: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = x[k * st[1]] - y[k * st[2]]; break;
            case RN_V_MUL: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = x[k * st[1]] * y[k * st[2]]; break;
            case RN_V_DIV: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = x[k * st[1]] / y[k * st[2]]; break;
            case RN_V_RSUB: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = y[k * st[2]] - x[k * st[1]]; break;
            case RN_V_RDIV: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = y[k * st[2]] / x[k * st[1]]; break;
            default: TRAP(RN_T_BAD_OP);
          }
          pc += 13; break;
        }
        float s = A[o[7]];
        switch (kind) {
          case RN_V_ADD: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = x[k * st[1]] + s; break;
          case RN_V_SUB: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = x[k * st[1]] - s; break;
          case RN_V_MUL: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = x[k * st[1]] * s; break;
          case RN_V_DIV: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = x[k * st[1]] / s; break;
          case RN_V_RSUB: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = s - x[k * st[1]]; break;
          case RN_V_RDIV: for (int32_t k = 0; k < cnt; k++) d[k * st[0]] = s / x[k * st[1]]; break;
          default: TRAP(RN_T_BAD_OP);
        }
        pc += 11; break;
      }
      default: TRAP(RN_T_BAD_OP);
    }
  }
  P->steps = steps; P->trap_code = 0; P->work = bls_work;
  return RN_OK;
trap:
  P->steps = steps; P->work = bls_work; P->trap_pc = pc - f->entry; P->trap_code = err;
  return err;
}
