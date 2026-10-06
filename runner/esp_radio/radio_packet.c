/* A packet link's radio_io: plink between the program's CRSF frames and the transport's packets. See esp_radio.h. */
#include "radio_packet.h"
#include "radio_cfg.h"
#include "esp_timer.h"
#include "esp_random.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* The command module stopped writing channels (its program stalled, or the computer it bridges for went quiet):
 * after this long its last channels no longer go up, so the drone counts the link lost (rc_core.h, RC_LOST_S) and
 * its failsafe flies, as it would with an ExpressLRS transmitter module whose handset stopped. */

double pk_now(void) { return esp_timer_get_time() * 1e-6; }
void pk_say(pk_link *K, const char *fmt, ...) {
  if (!K->say) return;
  char s[200]; va_list a; va_start(a, fmt); vsnprintf(s, sizeof s, fmt, a); va_end(a);
  /* in lines of up to 78 characters, broken at a space (the flight firmware's events hold 79) */
  char *p = s; int n = (int)strlen(p);
  while (n > 78) {
    int k = 78; while (k > 20 && p[k] != ' ') k--;
    if (p[k] != ' ') k = 78;
    char c = p[k]; p[k] = 0; K->say(p); p[k] = c;
    if (p[k] == ' ') k++;
    p += k; n -= k;
  }
  if (n) K->say(p);
}

/* send the packet due now, if one is */
static void pump_out(pk_link *K, double t) {
  uint8_t p[PLINK_MTU]; int n = plink_to_air(&K->L, t, p, sizeof p);
  if (n && K->send(K, p, n)) K->send_fail++;
}
static int pk_read(radio_io *R, uint8_t *b, int n, int wait_ms) {
  pk_link *K = (pk_link *)R;
  uint8_t p[PLINK_MTU]; int rssi = 0;
  int wait = K->L.out_n ? 0 : wait_ms;                 /* (something for the program already: don't wait) */
  for (int k = 0; k < 32; k++) {                        /* what came (a bounded number: the program keeps its pace) */
    int m = K->recv(K, p, sizeof p, &rssi, k ? 0 : wait);
    if (m <= 0) break;
    if (plink_from_air(&K->L, p, m, rssi, pk_now()) && K->taken) K->taken(K);
  }
  double t = pk_now();
  pump_out(K, t);
  return plink_to_stack(&K->L, t, b, n);
}
static int pk_write(radio_io *R, const uint8_t *b, int n) {
  pk_link *K = (pk_link *)R; double t = pk_now();
  plink_from_stack(&K->L, b, n, t);
  pump_out(K, t);                                       /* (the channels up at once, if a packet is due) */
  return n;
}

void pk_init(pk_link *K, const char *name, int role, const char *bind, esp_radio_say say) {
  plink_cfg C; plink_cfg_default(&C, role);
  plink_key(rcfg_bind(bind), &C.k0, &C.k1);
  uint32_t ses = 0; while (!ses) ses = esp_random();   /* (a new one each start: the other end tells restarts by it) */
  plink_init(&K->L, &C, ses);
  K->io.name = name; K->io.read = pk_read; K->io.write = pk_write; K->io.fd = -1; K->io.ctx = K;
  K->say = say;
}

void esp_radio_status(radio_io *R, char *out, int n) {
  pk_link *K = R ? (pk_link *)R->ctx : 0;
  if (!K || K->io.read != pk_read) { snprintf(out, (size_t)n, "%s", R ? R->name : "no radio"); return; }
  double t = pk_now(); const plink_counts *N = &K->L.N;
  snprintf(out, (size_t)n, "%s: %s; hears it at LQ %d%% %d dBm, heard at LQ %d%% %d dBm; packets sent %lu (failed %lu), taken %lu, bad %lu, replays %lu, resent %lu, dropped %lu",
           K->io.name, plink_connected(&K->L, t) ? "connected" : "not connected", plink_lq(&K->L, t), K->L.rssi, K->L.peer_lq, K->L.peer_rssi,
           (unsigned long)N->sent, (unsigned long)K->send_fail, (unsigned long)N->got, (unsigned long)N->bad, (unsigned long)N->replays, (unsigned long)N->resent, (unsigned long)N->uq_dropped);
}
