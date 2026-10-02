/*
 * Drone Force Bench step runner: the flight controller's side of loading programs.
 *
 * The firmware keeps three program slots:
 *   - the built-in program, compiled into the firmware (rn_builtin.c). It is always there to fall back to;
 *   - the flying program;
 *   - a candidate: a program the companion computer (the Raspberry Pi) just sent.
 * A new program goes through the same steps as in the simulator (js/rn-bridge.js):
 *   rn_host_stage()  loads it into a free slot: the loader checks every step (rn_load), the image's self-tests
 *                    run (rn_selftest), every formula must take and return exactly what the built-in one does,
 *                    and it gets a copy of the flying program's memory, by name (rn_transfer);
 *   shadow           for shadow_s seconds rn_host_call() runs it beside the flying program on the same inputs.
 *                    Its answers aren't used; a trap, a step limit or a number that isn't finite rejects it;
 *   blend            for blend_s seconds the answers move from the flying program's to the candidate's;
 *   swap             it flies, with the memory it built up in the background. The old one is kept as `prev`.
 * If the flying program traps, rn_host_call() switches to `prev` (or the built-in program), carries the memory
 * across by name and answers the same call from there, so the control step still gets its numbers.
 *
 * Inputs and results are passed flat: a formula's inputs in the order of its signature (a formula's memory is
 * not an input here: the host keeps it), each as the floats the simulator's compiler lays it out in (a number is
 * 1 float, a 3-vector 3, a list is its length then its capacity's elements, an optional value its present flag
 * then the value). The simulator's rn-sigs.js lists the signatures; rn_host_arg_size() gives the sizes.
 *
 * A formula called for several things (servoPredictor, once per servo) keeps a memory per instance: call it
 * with inst = 0, 1, …; instance 0's memory lives in the arena, the others in the slot's instance pool.
 */
#ifndef RN_HOST_H
#define RN_HOST_H
#include "rn.h"

#define RN_HOST_INST_MAX 8          /* instances per formula */

typedef struct {
  rn_prog P;
  float *arena; int32_t *code; uint32_t arena_cap, code_cap;
  float *pool; uint32_t pool_cap;   /* memories of instances 1… */
  int32_t inst_at[RN_FN_MAX];       /* where formula i's instances start in the pool (−1: none) */
  uint8_t loaded;
} rn_slot;

enum { RN_PH_FLYING = 0, RN_PH_SHADOW, RN_PH_BLEND };

typedef struct {
  rn_slot slot[3];                  /* 0: built-in, 1 and 2: loaded programs */
  volatile int act, cand, prev;     /* slot indices, −1 when none (volatile: the two cores read them) */
  volatile int phase; float t, shadow_s, blend_s;
  float max_diff; int max_diff_fn;  /* in the background run: the largest difference from the flying answers */
  int n_inst[RN_FN_MAX];            /* instances wanted per formula of the built-in program (by index) */
  float selftest_tol;
  int last_event; char last_fn[RN_NAME];
  int pending;                      /* a slot prepared on the link core, started by the next rn_host_tick */
  volatile int to_builtin;          /* two slots: the flight loop hands over to the built-in program */
  volatile int in_call;             /* formula calls running (rn_host_prepare waits for none before reusing a slot) */
  /* Formulas may be called from two cores at once only if their working space is separate: the in-flight
   * learning has its own (ownPool in js/rn-sigs.js); call the rest from one core. */
  void (*event)(void *ctx, int code, const char *what);   /* optional: tells the firmware what happened */
  void *event_ctx;
  void (*lock)(void *ctx, int on);  /* optional: a lock around slot bookkeeping, when two cores share the host */
  void *lock_ctx;
} rn_host;

/* What happened (also passed to the event callback). */
enum {
  RN_EV_LOADED = 1,      /* a candidate passed the loader and its self-tests and is flying in the background */
  RN_EV_REJECTED,        /* a candidate was rejected (loader, self-test, signature or the background run) */
  RN_EV_SWAPPED,         /* the candidate is flying now */
  RN_EV_FELL_BACK,       /* the flying program trapped; the previous (or built-in) one took over */
  RN_EV_BUILTIN_FAILED,  /* even the built-in program trapped: the firmware must go to its failsafe */
};

/* Set up the slots (the caller gives the memory: each slot needs an arena, a code buffer and an instance pool)
 * and load the built-in program, which must pass its own self-tests. The built-in program's code buffer may be
 * NULL (its steps run from the image, in flash). Where RAM is short, arenas[2] may be NULL: two slots. Then a new
 * program loads while the built-in one flies, and there is no older loaded program to fall back to. */
int rn_host_init(rn_host *H, const uint8_t *builtin, uint32_t len,
                 float *arenas[3], uint32_t arena_cap, int32_t *codes[3], uint32_t code_cap, float *pools[3], uint32_t pool_cap);
/* How many instances a formula needs (before staging; the built-in program's slot is sized at once). */
int rn_host_instances(rn_host *H, const char *fn, int n);
/* Stage a program the companion computer sent: rn_host_prepare() does the heavy part (load, self-tests,
 * signatures) and may run on the other core while the flight loop runs; the next rn_host_tick() gives it a copy
 * of the flying program's memory and starts the background run. rn_host_stage() does both at once, for a
 * single loop. Return RN_OK or why it was rejected. The image buffer is only needed during the call. */
int rn_host_prepare(rn_host *H, const uint8_t *img, uint32_t len);
int rn_host_stage(rn_host *H, const uint8_t *img, uint32_t len);
/* Advance the loading steps; call once per control step with its length. */
void rn_host_tick(rn_host *H, float dt);
/* Run formula `fn` (index in the built-in program's table; see rn_host_find) for instance `inst`. Returns
 * RN_OK, or a trap code if no program could answer. */
int rn_host_call(rn_host *H, int fn, int inst, const float *in, float *out);
int rn_host_find(const rn_host *H, const char *name);
int rn_host_in_size(const rn_host *H, int fn);      /* floats of flat input */
int rn_host_out_size(const rn_host *H, int fn);     /* floats of result */
/* The flying program's memory field of formula fn, instance 0 (the firmware's glue may read or scale it, as the
 * supervisor scales the learned table). Returns NULL if the field has no value yet; *size gets its floats. */
float *rn_host_field(rn_host *H, int fn, const char *field, int32_t *size);
/* Forget formula fn's memory (every instance, every program slot): its next call starts afresh, as `st = {}`. */
void rn_host_forget(rn_host *H, int fn);

#endif
