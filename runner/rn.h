/*
 * Drone Force Bench step runner.
 *
 * Runs the flight formulas compiled by the simulator's step compiler (js/rn-compile.js). The same C builds for
 * the ESP32 (ESP-IDF), for a PC (the test harness) and for WebAssembly (the simulator flies on it).
 *
 * A program image holds constants, steps and a table of formulas. rn_load() checks every step before accepting
 * it: every fixed address inside the arena, no writes to the constants, jumps only to the start of a step inside
 * the same formula, and every block (a 3×3 matrix, a list) inside the arena. At run time, steps that go through
 * a computed address check it, list lengths are checked against their capacity, and a formula that runs longer
 * than its step limit is stopped. So a bad or corrupted image can at worst produce wrong numbers in its own
 * arena: it can't write anywhere else or hang the flight loop.
 *
 * All arithmetic is in 32-bit floats, as on the ESP32.
 */
#ifndef RN_H
#define RN_H
#include <stdint.h>
#include "rn_ops.h"

#define RN_FN_MAX 32
#define RN_NAME 32
#define RN_ARGS_MAX 12           /* inputs per formula */
#define RN_STATE_MAX 16          /* fields in a formula's memory */
#define RN_SNAME 16              /* a memory field's name, zero-padded */
#define RN_BLS_MAX 48            /* most inputs the least-squares kernel handles */
#define RN_CODE_MAX 65536        /* most code words in a program */

/* A formula's memory field: its present flag, its place and size in the arena (address −1, size 0 while it
 * has no type yet), and its name, so the memory can move to another program by name. */
typedef struct { int32_t flag, addr, size; char name[RN_SNAME]; } rn_field;

/* A formula: where its steps are, and where the firmware's glue puts its inputs and reads its result. An input
 * that is the formula's memory has address −1 and size 0. */
typedef struct {
  char name[RN_NAME];
  int32_t entry, end, max_steps;
  int32_t n_args, arg_addr[RN_ARGS_MAX], arg_size[RN_ARGS_MAX];
  int32_t ret_addr, ret_size;
  int32_t n_state; rn_field state[RN_STATE_MAX];
} rn_fn;

typedef struct {
  float *arena; int32_t arena_size, const_end;
  int32_t *code; int32_t code_len;
  rn_fn fn[RN_FN_MAX]; int32_t n_fn;
  uint32_t tests_at; int32_t n_tests;     /* the image's self-test section: byte offset, count */
  /* after a call */
  int32_t steps, trap_pc, trap_code; float trap_value;
  uint32_t work;                          /* kernel arithmetic in the last call (multiply-adds), for the time budget */
} rn_prog;

enum {
  RN_OK = 0,
  /* loading */
  RN_E_MAGIC = 1, RN_E_VERSION, RN_E_SIZE, RN_E_CRC, RN_E_TOO_BIG, RN_E_BAD_OP, RN_E_BAD_ADDR, RN_E_WRITES_CONST, RN_E_BAD_JUMP, RN_E_BAD_BLOCK, RN_E_TABLE, RN_E_TESTS, RN_E_SELFTEST,
  /* running */
  /* loading on the drone (rn_host.c) */
  RN_E_SIGNATURE = 64, RN_E_BUSY, RN_E_INSTANCES,
  RN_T_STEPS = 32, RN_T_INDEX, RN_T_ADDR, RN_T_LIST_FULL, RN_T_LIST_LEN, RN_T_FORMULA, RN_T_BAD_OP, RN_T_NO_FN, RN_T_KERNEL,
};

/* Load an image into the given buffers. Returns RN_OK or an RN_E_ code (P->trap_pc holds the offending step).
 * With code = NULL the steps run from the image itself (a built-in program in flash): the image must stay put
 * and be 4-byte aligned. */
int rn_load(rn_prog *P, const uint8_t *img, uint32_t len, float *arena, uint32_t arena_cap, int32_t *code, uint32_t code_cap);
/* Run formula i. Returns RN_OK or an RN_T_ code; P->steps counts the steps taken. */
int rn_run(rn_prog *P, int32_t i);
/* Run the image's self-tests: each sets a formula's inputs and memory, runs it and compares the result and the
 * memory with what the simulator's runner got, within tol·(|expected| + the block's scale). The image must be the
 * one loaded. Clears the working memory afterwards. Returns RN_OK, RN_E_SELFTEST (P->trap_pc = failing test,
 * *worst = its error) or a run-time trap code. */
int rn_selftest(rn_prog *P, const uint8_t *img, uint32_t len, float tol, float *worst);
/* Carry formula memory from one program to another: every field with the same formula name, field name and
 * size is copied (flag and data); others start empty. Returns how many were copied. */
int rn_transfer(rn_prog *dst, const rn_prog *src);
/* Empty all working memory and every formula's memory (as after loading). */
void rn_clear(rn_prog *P);
int rn_find(const rn_prog *P, const char *name);
const char *rn_error_text(int code);
uint32_t rn_crc32(const uint8_t *p, uint32_t n);

#endif
