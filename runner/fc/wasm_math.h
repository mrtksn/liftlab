/* The few math functions rc_core.c and tlm_sources.c use: the C library's, or in the simulator's WebAssembly build
 * (no C library) the page's Math. */
#ifndef WASM_MATH_H
#define WASM_MATH_H
#if defined(__wasm__)
#define WM_IMPORT(n) __attribute__((import_module("env"), import_name(n)))
WM_IMPORT("sin") double wm_sin(double); WM_IMPORT("cos") double wm_cos(double); WM_IMPORT("atan2") double wm_atan2(double, double);
#define sinf(x) ((float)wm_sin(x))
#define cosf(x) ((float)wm_cos(x))
#define atan2f(y, x) ((float)wm_atan2(y, x))
#define sqrtf(x) __builtin_sqrtf(x)
#define fabsf(x) __builtin_fabsf(x)
static inline double wm_floor(double x) { double f = (double)(long long)x; return f > x ? f - 1 : f; }
#define floor(x) wm_floor(x)
#else
#include <math.h>
#endif
#endif
