/* WebAssembly entry points for the simulator: one program, static buffers, the page writes inputs straight
 * into the arena. Built by build_wasm.sh. */
#include "rn.h"
#define ARENA_CAP 131072
#define CODE_CAP 131072
#define IMG_CAP (2u << 20)
static float arena[ARENA_CAP];
static int32_t code[CODE_CAP];
static uint8_t img[IMG_CAP];
static rn_prog P;

#define EXPORT(n) __attribute__((export_name(n)))
EXPORT("img_ptr") uint8_t *img_ptr(void) { return img; }
EXPORT("img_cap") uint32_t img_cap(void) { return IMG_CAP; }
EXPORT("arena_ptr") float *arena_ptr(void) { return arena; }
EXPORT("load") int load(uint32_t len) { return rn_load(&P, img, len, arena, ARENA_CAP, code, CODE_CAP); }
EXPORT("run") int run(int32_t i) { return rn_run(&P, i); }
static float st_worst;
EXPORT("selftest") int selftest(uint32_t len, float tol) { return rn_selftest(&P, img, len, tol, &st_worst); }
EXPORT("selftest_worst") float selftest_worst(void) { return st_worst; }
EXPORT("steps") int32_t steps(void) { return P.steps; }
EXPORT("work") uint32_t work(void) { return P.work; }
EXPORT("trap_pc") int32_t trap_pc(void) { return P.trap_pc; }

/* The few C library functions the runner uses (there is no C library in this build). */
typedef __SIZE_TYPE__ size_t;
void *memcpy(void *d, const void *s, size_t n) { uint8_t *a = d; const uint8_t *b = s; while (n--) *a++ = *b++; return d; }
void *memmove(void *d, const void *s, size_t n) {
  uint8_t *a = d; const uint8_t *b = s;
  if (a == b || !n) return d;
  if (a < b) { if (!(((uintptr_t)a | (uintptr_t)b | n) & 3)) { uint32_t *x = (uint32_t *)a; const uint32_t *y = (const uint32_t *)b; n >>= 2; while (n--) *x++ = *y++; } else while (n--) *a++ = *b++; }
  else { if (!(((uintptr_t)a | (uintptr_t)b | n) & 3)) { uint32_t *x = (uint32_t *)(a + n); const uint32_t *y = (const uint32_t *)(b + n); n >>= 2; while (n--) *--x = *--y; } else { a += n; b += n; while (n--) *--a = *--b; } }
  return d;
}
void *memset(void *d, int c, size_t n) { uint8_t *a = d; while (n--) *a++ = (uint8_t)c; return d; }
size_t strlen(const char *s) { size_t n = 0; while (s[n]) n++; return n; }
int strncmp(const char *a, const char *b, size_t n) { for (; n; n--, a++, b++) { if (*a != *b) return (unsigned char)*a - (unsigned char)*b; if (!*a) return 0; } return 0; }
