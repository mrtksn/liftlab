/* What each task puts into the telemetry store: tlm_sources.c. */
#ifndef TLM_SOURCES_H
#define TLM_SOURCES_H
#include "tlm_core.h"
#include "rc_core.h"
#include "fc_core.h"
#include "nav_core.h"
#include "learn_core.h"
#include "super_core.h"

typedef struct {
  int fc_state; char fc_why[64], nav_why[64]; char learn_msg[480];
  uint32_t super_seq; float mah; double t_super;
} tlm_watch;
void tlm_watch_init(tlm_watch *W);
void tlm_from_core(tlm_store *T, tlm_watch *W, const fc_state *F, double t);
void tlm_from_nav(tlm_store *T, tlm_watch *W, const nav_state *N, const nav_out *o, const nav_sp *sp, int level, double t);
void tlm_from_gps(tlm_store *T, double lat, double lon, float alt, float speed, float course, int sats, double t);
void tlm_from_learn(tlm_store *T, tlm_watch *W, const learn_state *L, double t);
void tlm_from_super(tlm_store *T, tlm_watch *W, const super_state *S, double t);
void tlm_from_link(tlm_store *T, const rc_input *in, double t);
#endif
