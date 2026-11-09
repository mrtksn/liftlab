/*
 * A small MQTT 3.1.1 client over TCP, for the link between the flight controller and a companion computer (and for
 * anything else that wants a broker): QoS 0 only, one subscription, nonblocking, no allocation, BSD sockets (Linux,
 * macOS and the ESP32's lwIP alike).
 *
 * What it does: connects (clean session, keep-alive), subscribes to one topic, publishes whole messages, pings while
 * idle, and reconnects with a growing pause (0.5 s up to 5 s) when the broker goes away or stops answering. A message on
 * the subscribed topic is handed on as it arrives, in pieces (on_begin, then on_bytes), so a large one needs no buffer
 * of its size: the link's frame parser (rn_link) takes bytes anyway.
 *
 * Over the link, one MQTT message carries one whole link frame ('D' 'F' …, rn_link.h). QoS 0 over TCP keeps a sender's
 * messages in order; a message can still be lost when the broker drops a slow client's queue or the connection breaks,
 * and the frames' own CRC and the flight core's failsafes cover that, as they do on a serial line.
 */
#ifndef MQTT_MIN_H
#define MQTT_MIN_H
#include <stdint.h>

enum { MQ_IDLE = 0, MQ_CONNECTING = 1, MQ_HELLO = 2, MQ_UP = 3 };

typedef struct mq_client {
  /* settings (mq_init) */
  char host[64], id[24], sub[96];
  int port, keepalive;                         /* seconds */
  void (*on_begin)(void *ctx, uint32_t len);   /* a message on the subscribed topic starts: len bytes follow */
  void (*on_bytes)(void *ctx, const uint8_t *b, int n);
  void (*on_say)(void *ctx, const char *text); /* what happened to the connection, for the log (may be 0) */
  void *ctx;
  /* state */
  int fd, state;
  double t_retry, backoff, t_rx, t_tx, t_stall, t_state;
  uint8_t *tx; uint32_t txcap, txlen;          /* the outgoing queue (whole messages) */
  /* the incoming packet, parsed as it comes */
  int st; uint8_t hdr; uint32_t rem, mul, tlen, got; int match; uint8_t small[8]; char topic[128];
  /* counts */
  uint32_t sent, recvd, dropped, reconnects;
  char why[80];
} mq_client;

/* Settings: the broker, this client's id (unique on the broker), the topic to receive, the outgoing queue (it holds
 * whole messages: a message that doesn't fit is dropped). */
void mq_init(mq_client *M, const char *host, int port, const char *id, const char *sub, uint8_t *txbuf, uint32_t txcap);
/* Every pass (a few ms apart): connects or reconnects when it's time, reads what came, sends what's queued, pings.
 * now: seconds, any monotonic clock. Returns the socket (−1 while there's none), for poll()/select(). */
int mq_poll(mq_client *M, double now);
/* Queue a message (QoS 0). 0: queued; −1: not connected, or no room (dropped and counted). */
int mq_publish(mq_client *M, const char *topic, const uint8_t *p, uint32_t n, double now);
/* Start from a socket that is already connected (tests, or a transport made elsewhere): sends CONNECT and SUBSCRIBE. */
void mq_attach(mq_client *M, int fd, double now);
void mq_close(mq_client *M, const char *why, double now);
static inline int mq_up(const mq_client *M) { return M->state == MQ_UP; }

/* Parse "mqtt://HOST[:PORT][/PREFIX]" (or "HOST[:PORT][/PREFIX]"). 0 ok. */
int mq_parse_url(const char *url, char *host, int hostn, int *port, char *prefix, int prefixn);
/* The link's two topics under a prefix: PREFIX/link/to-fc (frames for the flight controller) and PREFIX/link/from-fc. */
void mq_link_topics(const char *prefix, char *to_fc, int to_n, char *from_fc, int from_n);
#endif
