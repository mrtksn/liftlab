/* The CRSF transport (tlm_crsf.c): telemetry out, the pilot's radio in. */
#ifndef TLM_CRSF_H
#define TLM_CRSF_H
#include "tlm_core.h"
#include "rc_core.h"
#include "crsf.h"
int tlm_crsf_input(crsf_parser *P, uint8_t b, rc_input *in, double t);
int tlm_crsf_cmd(uint8_t *out, int cmd, int seq, const float *v, int nv);
float tlm_crsf_budget(int rate_hz, int ratio);
#endif
