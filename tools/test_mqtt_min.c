/* The MQTT client (runner/fc/mqtt_min.c) against a small broker in this test, over real TCP on 127.0.0.1: connect and
 * subscribe, link frames both ways (one frame a message, through rn_link), a 40 KB message handed on in pieces, a
 * message split into single bytes, QoS 1 deliveries, keep-alive pings, a refused CONNECT, a broker that goes away and
 * comes back (reconnect, nothing stale replayed), a full queue, and URL parsing.
 *   cc -Wall -Wextra -Werror -Irunner -Irunner/fc tools/test_mqtt_min.c runner/fc/mqtt_min.c runner/rn_link.c -o /tmp/test-mqtt && /tmp/test-mqtt */
#include "mqtt_min.h"
#include "rn_link.h"
#include <arpa/inet.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "mqtt_test_broker.h"

/* ── a link end: an MQTT client feeding rn_link, as the Pi and the flight controller do ── */
typedef struct { rn_link L; uint8_t buf[70000]; int frames, last_type; uint32_t last_len, resets; mq_client M; uint8_t tx[70000]; } end_t;
static void on_begin(void *ctx, uint32_t len) { end_t *E = ctx; (void)len; if (E->L.state) { rn_link_reset(&E->L); E->resets++; } }
static void on_bytes(void *ctx, const uint8_t *b, int n) {
  end_t *E = ctx; for (int i = 0; i < n; i++) { int t = rn_link_feed(&E->L, b[i]); if (t > 0) { E->frames++; E->last_type = t; E->last_len = E->L.len; } }
}
static void end_init(end_t *E, const char *id, const char *sub) {
  memset(E, 0, sizeof *E); rn_link_init(&E->L, E->buf, sizeof E->buf);
  mq_init(&E->M, "127.0.0.1", BRK.port, id, sub, E->tx, sizeof E->tx); E->M.on_begin = on_begin; E->M.on_bytes = on_bytes; E->M.ctx = E; E->M.keepalive = 2;
}
static double T;
static void run(end_t *a, end_t *b, double secs) { for (double t0 = T; T < t0 + secs; T += 0.005) { broker_poll(); mq_poll(&a->M, T); if (b) mq_poll(&b->M, T); usleep(5000); } }   /* (real time: a new connection can take ~100 ms where a firewall vets it) */
static int send_frame(end_t *E, const char *topic, uint8_t type, const uint8_t *p, uint32_t n) {
  static uint8_t fr[70000]; uint32_t k = rn_link_frame(fr, sizeof fr, type, p, n); assert(k); return mq_publish(&E->M, topic, fr, k, T);
}

int main(void) {
  broker_open();
  char to[96], from[96]; mq_link_topics("liftlab/test", to, sizeof to, from, sizeof from);
  assert(!strcmp(to, "liftlab/test/link/to-fc") && !strcmp(from, "liftlab/test/link/from-fc"));
  static end_t FC, PI; end_init(&FC, "fc", to); end_init(&PI, "pi", from);
  assert(send_frame(&PI, to, RN_LINK_CMD, (const uint8_t *)"x", 1) == -1 && PI.M.dropped == 1);   /* not connected: dropped */
  run(&FC, &PI, 1); assert(mq_up(&FC.M) && mq_up(&PI.M));

  /* frames both ways, one a message */
  float cmd[12] = { 1, 0, 0, 0, .5f }; for (int i = 0; i < 5; i++) assert(!send_frame(&PI, to, RN_LINK_CMD, (uint8_t *)cmd, sizeof cmd));
  float tel[36] = { 0 }; assert(!send_frame(&FC, from, RN_LINK_TELEM, (uint8_t *)tel, sizeof tel));
  run(&FC, &PI, 0.1); assert(FC.frames == 5 && FC.last_type == RN_LINK_CMD && FC.last_len == sizeof cmd); assert(PI.frames == 1 && PI.last_type == RN_LINK_TELEM);

  /* a 40 KB message, handed on in pieces */
  static uint8_t big[40000]; for (size_t i = 0; i < sizeof big; i++) big[i] = (uint8_t)(i * 7);
  assert(!send_frame(&PI, to, RN_LINK_PROGRAM, big, sizeof big)); run(&FC, &PI, 0.3);
  assert(FC.frames == 6 && FC.last_type == RN_LINK_PROGRAM && FC.last_len == sizeof big && !memcmp(FC.buf, big, sizeof big));

  /* a message arriving a byte at a time, and QoS 1 deliveries (a packet id before the payload) */
  BRK.byte_by_byte = 1; for (int i = 0; i < BC; i++) BRK.c[i].qos1 = 1;
  assert(!send_frame(&FC, from, RN_LINK_NAV, (uint8_t *)tel, 64)); run(&FC, &PI, 0.2);
  assert(PI.frames == 2 && PI.last_type == RN_LINK_NAV && PI.last_len == 64);
  BRK.byte_by_byte = 0; for (int i = 0; i < BC; i++) BRK.c[i].qos1 = 0;

  /* keep-alive: pings while idle, and still up */
  int p0 = BRK.pings; run(&FC, &PI, 3); assert(BRK.pings > p0 && mq_up(&FC.M) && mq_up(&PI.M));

  /* a full queue drops whole messages, never part of one */
  { mq_client Q; uint8_t small[64]; mq_init(&Q, "127.0.0.1", BRK.port, "q", "", small, sizeof small); Q.state = MQ_UP; Q.fd = -1;
    uint8_t m[40] = { 0 }; assert(!mq_publish(&Q, "t", m, sizeof m, T)); assert(mq_publish(&Q, "t", m, sizeof m, T) == -1 && Q.dropped == 1 && Q.txlen == 1 + 1 + 2 + 1 + 40); }

  /* the broker goes away: both notice, reconnect, and what was queued then isn't replayed */
  broker_drop_all(); run(&FC, &PI, 0.2); assert(strstr(FC.M.why, "closed") || strstr(FC.M.why, "broke"));
  run(&FC, &PI, 3); assert(mq_up(&FC.M) && mq_up(&PI.M) && FC.M.reconnects >= 1);
  int before = FC.frames; assert(!send_frame(&PI, to, RN_LINK_CMD, (uint8_t *)cmd, sizeof cmd)); run(&FC, &PI, 0.1); assert(FC.frames == before + 1);

  /* a refused CONNECT: closed, and tried again later */
  { BRK.refuse = 5; static end_t R; end_init(&R, "r", "x"); run(&R, 0, 0.5); assert(!mq_up(&R.M) && strstr(R.M.why, "refused")); BRK.refuse = 0; run(&R, 0, 2); assert(mq_up(&R.M)); }

  /* URLs */
  char h[64], pre[64]; int port;
  assert(!mq_parse_url("mqtt://192.168.1.100:1884/liftlab/a13", h, sizeof h, &port, pre, sizeof pre) && !strcmp(h, "192.168.1.100") && port == 1884 && !strcmp(pre, "liftlab/a13"));
  assert(!mq_parse_url("broker.local", h, sizeof h, &port, pre, sizeof pre) && !strcmp(h, "broker.local") && port == 1883 && !strcmp(pre, "liftlab"));
  const char *bad[] = { "mqtt://", ":1883", "h:0", "h:70000", "h/", "h/a/", "h/a/#", "h/+", "h:12x" };
  for (unsigned i = 0; i < sizeof bad / sizeof *bad; i++) assert(mq_parse_url(bad[i], h, sizeof h, &port, pre, sizeof pre));
  puts("MQTT client tests passed: connect/subscribe, link frames both ways, a 40 KB message in pieces, byte-at-a-time and QoS 1 deliveries, keep-alive, whole-message queue drops, reconnect without replay, refused CONNECT, URLs.");
  return 0;
}
