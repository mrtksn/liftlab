/*
 * Programs (docs/topic-bus.md): your own formulas on a board, run on the data bus.
 *
 * A program is a formula in the board's flight program (compiled by the simulator like the others, run on the step
 * runner) with a header: the topics it reads, the topic it writes (user.…, with its layout), and when it runs:
 * whenever a topic it reads changes, or every period. Its signature is (st, inp, dt): its own memory, a record of the
 * topics it reads (their values laid out as their layouts say, one after another, in the order given), and the time
 * since its last run; it returns the record its topic is laid out as, which is published.
 *
 * It waits until every topic it reads has a value. A run that fails (a trap, a number that isn't finite) publishes
 * nothing and is counted; the step runner's own fallback applies as for any formula.
 *
 * Use: prog_init; for each program prog_add (registers its topic), then (once the topics it reads are on the bus,
 * mirrors included) prog_read for each, prog_trigger if it runs on a change, and prog_check; then prog_step at the
 * board's step, after the bus clock is set.
 */
#ifndef PROG_CORE_H
#define PROG_CORE_H
#include "rn_host.h"
#include "bus.h"

#define PROG_MAX 16                    /* programs on a board */
#define PROG_READS 8                   /* topics a program reads */
#define PROG_IN (PROG_READS * BUS_VALS + 1)

typedef struct {
  char name[32];
  int fn, out;                         /* its formula in the host; the topic it writes */
  int reads[PROG_READS], nreads;
  int trig; float period;              /* the topic whose change runs it (−1: every period [s]) */
  uint32_t seen; double next, last;    /* the trigger's sequence seen; when it next runs (by period); when it last ran */
  int ok;                              /* checked: its formula takes and returns what its header says */
  uint32_t runs, fails, waits; int err;   /* runs, failed runs, runs skipped waiting for its inputs; the last error */
} prog_t;
typedef struct {
  rn_host *H; bus *B;
  prog_t P[PROG_MAX]; int n;
  char why[96];                        /* why the last prog_add, prog_read or prog_check refused */
  float in[PROG_IN], out[BUS_VALS];
} prog_state;

void prog_init(prog_state *G, rn_host *H, bus *B);
/* A program: its formula's name, the topic it writes (a name under "user.", its floats and layout), and when it runs:
 * every period [s], or (period 0) when a topic it reads changes (prog_trigger says which). Its index, or −1 (G->why
 * says why). */
int prog_add(prog_state *G, const char *name, const char *out, int out_n, const char *out_layout, float period);
/* A topic program i reads (on this board's bus: its own, or a mirror). 0, or −1. */
int prog_read(prog_state *G, int i, const char *topic);
/* A program that runs on a change: the read whose change runs it. 0, or −1. */
int prog_trigger(prog_state *G, int i, const char *topic);
/* Program i's formula takes its reads and dt, and returns its topic's floats; its trigger is known. 0, or −1. */
int prog_check(prog_state *G, int i);
/* Run what is due, at the bus's now. */
void prog_step(prog_state *G);

#endif
