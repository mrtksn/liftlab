/* A small MQTT 3.1.1 client: see mqtt_min.h. */
#include "mqtt_min.h"
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#ifdef MSG_NOSIGNAL
#define SEND_FLAGS MSG_NOSIGNAL                  /* (a broker gone: an error, not SIGPIPE) */
#else
#define SEND_FLAGS 0
#endif
enum { P_CONNECT = 1, P_CONNACK = 2, P_PUBLISH = 3, P_SUBSCRIBE = 8, P_SUBACK = 9, P_PINGREQ = 12, P_PINGRESP = 13 };

static void say(mq_client *M, const char *fmt, ...) {
  va_list a; va_start(a, fmt); vsnprintf(M->why, sizeof M->why, fmt, a); va_end(a);
  if (M->on_say) M->on_say(M->ctx, M->why);
}
static void copy(char *d, int n, const char *s) { snprintf(d, (size_t)n, "%s", s ? s : ""); }

void mq_init(mq_client *M, const char *host, int port, const char *id, const char *sub, uint8_t *txbuf, uint32_t txcap) {
  memset(M, 0, sizeof *M);
  copy(M->host, sizeof M->host, host); copy(M->id, sizeof M->id, id); copy(M->sub, sizeof M->sub, sub);
  M->port = port > 0 ? port : 1883; M->keepalive = 10; M->fd = -1; M->tx = txbuf; M->txcap = txcap; M->backoff = 0.5;
}

static int put_len(uint8_t *b, uint32_t n) { int k = 0; do { uint8_t d = n % 128; n /= 128; if (n) d |= 128; b[k++] = d; } while (n && k < 4); return k; }
static int put_str(uint8_t *b, const char *s) { size_t n = strlen(s); b[0] = (uint8_t)(n >> 8); b[1] = (uint8_t)n; memcpy(b + 2, s, n); return 2 + (int)n; }

/* A whole message into the queue, or nothing. */
static int queue(mq_client *M, const uint8_t *h, uint32_t hn, const uint8_t *p, uint32_t pn, double now) {
  if (!M->tx || M->txlen + hn + pn > M->txcap) { M->dropped++; return -1; }
  if (!M->txlen) M->t_stall = now;
  memcpy(M->tx + M->txlen, h, hn); M->txlen += hn;
  if (pn) { memcpy(M->tx + M->txlen, p, pn); M->txlen += pn; }
  return 0;
}
static int flush(mq_client *M, double now) {
  while (M->txlen && M->fd >= 0) {
    ssize_t n = send(M->fd, M->tx, M->txlen, SEND_FLAGS);
    if (n > 0) { memmove(M->tx, M->tx + n, M->txlen - (uint32_t)n); M->txlen -= (uint32_t)n; M->t_tx = now; M->t_stall = now; continue; }
    if (n < 0 && errno == EINTR) continue;
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
    return -1;
  }
  if (!M->txlen) M->t_stall = 0;
  return 0;
}

void mq_close(mq_client *M, const char *why, double now) {
  if (M->fd >= 0) close(M->fd);
  int was = M->state;
  M->fd = -1; M->state = MQ_IDLE; M->txlen = 0; M->st = 0;     /* (what was queued for that connection is stale: commands aren't replayed) */
  M->t_retry = now + M->backoff; M->backoff = M->backoff * 2 > 5 ? 5 : M->backoff * 2;
  if (was == MQ_UP) M->reconnects++;
  say(M, "MQTT: %s; trying %s:%d again in %.1f s", why, M->host, M->port, M->t_retry - now);
}

static void hello(mq_client *M, double now) {             /* connected: CONNECT (clean session, keep-alive) */
  uint8_t b[160]; int k = 0; uint32_t rem = 10 + 2 + (uint32_t)strlen(M->id);
  b[k++] = P_CONNECT << 4; k += put_len(b + k, rem);
  const uint8_t vh[10] = { 0, 4, 'M', 'Q', 'T', 'T', 4, 0x02, (uint8_t)(M->keepalive >> 8), (uint8_t)M->keepalive };
  memcpy(b + k, vh, sizeof vh); k += sizeof vh; k += put_str(b + k, M->id);
  M->txlen = 0; M->st = 0; queue(M, b, (uint32_t)k, 0, 0, now);
  M->state = MQ_HELLO; M->t_state = now; M->t_rx = now;
}
static void up(mq_client *M, double now) {                /* the broker took us: SUBSCRIBE (QoS 0) */
  M->state = MQ_UP; M->backoff = 0.5;
  if (M->sub[0]) {
    uint8_t b[128]; int k = 0; uint32_t rem = 2 + 2 + (uint32_t)strlen(M->sub) + 1;
    b[k++] = P_SUBSCRIBE << 4 | 2; k += put_len(b + k, rem); b[k++] = 0; b[k++] = 1; k += put_str(b + k, M->sub); b[k++] = 0;
    queue(M, b, (uint32_t)k, 0, 0, now);
  }
  say(M, "MQTT: connected to %s:%d as %s%s%s", M->host, M->port, M->id, M->sub[0] ? ", receiving " : "", M->sub);
}

static void start(mq_client *M, double now) {
  struct addrinfo hints, *ai = 0; memset(&hints, 0, sizeof hints); hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM;
  char port[8]; snprintf(port, sizeof port, "%d", M->port);
  M->t_retry = now + M->backoff;
  if (getaddrinfo(M->host, port, &hints, &ai) || !ai) { if (ai) freeaddrinfo(ai); mq_close(M, "can't find the broker's address", now); return; }
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) { freeaddrinfo(ai); mq_close(M, "no socket", now); return; }
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
  int one = 1; setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);   /* (small frames, sent at once) */
#ifdef SO_NOSIGPIPE
  setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
  int r = connect(fd, ai->ai_addr, ai->ai_addrlen); freeaddrinfo(ai);
  if (r && errno != EINPROGRESS) { close(fd); mq_close(M, "can't reach the broker", now); return; }
  M->fd = fd; M->state = MQ_CONNECTING; M->t_state = now;
  if (!r) hello(M, now);
}
void mq_attach(mq_client *M, int fd, double now) {
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL, 0) | O_NONBLOCK);
  M->fd = fd; hello(M, now); flush(M, now);
}

/* A whole packet other than PUBLISH has come (its first bytes in small). 0: go on; −1: the connection was closed. */
static int done(mq_client *M, double now) {
  int type = M->hdr >> 4;
  if (type == P_CONNACK) {
    if (M->got < 2 || M->small[1]) { char w[48]; snprintf(w, sizeof w, "the broker refused us (code %d)", M->got < 2 ? -1 : M->small[1]); mq_close(M, w, now); return -1; }
    if (M->state == MQ_HELLO) up(M, now);
  } else if (type == P_SUBACK && M->got >= 3 && M->small[2] == 0x80) say(M, "MQTT: the broker refused the subscription to %s", M->sub);
  return 0;
}
/* What came, as it comes. 0: go on; −1: closed. */
static int feed(mq_client *M, const uint8_t *b, int n, double now) {
  int i = 0;
  while (i < n) {
    if (M->st == 0) { M->hdr = b[i++]; M->rem = 0; M->mul = 1; M->got = 0; M->st = 1; continue; }
    if (M->st == 1) {
      uint8_t d = b[i++]; M->rem += (uint32_t)(d & 127) * M->mul;
      if (d & 128) { M->mul *= 128; if (M->mul > 128u * 128 * 128) { mq_close(M, "a damaged packet", now); return -1; } continue; }
      M->tlen = 0; M->match = 0;
      if ((M->hdr >> 4) == P_PUBLISH) M->st = M->rem >= 2 ? 2 : 6; else M->st = 6;
      if (!M->rem) { M->st = 0; if (done(M, now)) return -1; }
      continue;
    }
    if (M->st == 2) {                                   /* the topic's length */
      M->tlen = M->tlen << 8 | b[i++]; if (++M->got == 2) { M->st = M->tlen ? 3 : 5; M->topic[0] = 0; }
      if (M->st == 5) goto payload_start;
      continue;
    }
    if (M->st == 3) {                                   /* the topic */
      uint32_t at = M->got - 2; if (at < sizeof M->topic - 1) { M->topic[at] = (char)b[i]; M->topic[at + 1] = 0; }
      i++; M->got++;
      if (M->got - 2 == M->tlen) { M->st = ((M->hdr >> 1) & 3) ? 4 : 5; if (M->st == 5) goto payload_start; }
      continue;
    }
    if (M->st == 4) { i++; if (++M->got == 2 + M->tlen + 2) { M->st = 5; goto payload_start; } continue; }   /* (a packet id: QoS > 0) */
    if (M->st == 5) {                                   /* the payload, passed on in pieces */
      uint32_t k = (uint32_t)(n - i); if (k > M->rem - M->got) k = M->rem - M->got;
      if (M->match && k && M->on_bytes) M->on_bytes(M->ctx, b + i, (int)k);
      i += (int)k; M->got += k;
      if (M->got >= M->rem) { if (M->match) M->recvd++; M->st = 0; }
      continue;
    }
    if (M->st == 6) {                                   /* any other packet: its first bytes kept */
      if (M->got < sizeof M->small) M->small[M->got] = b[i];
      i++; if (++M->got >= M->rem) { M->st = 0; if (done(M, now)) return -1; }
      continue;
    }
    continue;
  payload_start:
    M->match = M->sub[0] && M->tlen == strlen(M->sub) && !strcmp(M->topic, M->sub);
    if (M->match && M->on_begin) M->on_begin(M->ctx, M->rem - M->got);
    if (M->got >= M->rem) { if (M->match) M->recvd++; M->st = 0; }
  }
  return 0;
}

int mq_poll(mq_client *M, double now) {
  if (M->state == MQ_IDLE) { if (M->host[0] && now >= M->t_retry) start(M, now); if (M->fd < 0) return -1; }
  if (M->state == MQ_CONNECTING) {
    fd_set w; FD_ZERO(&w); FD_SET(M->fd, &w); struct timeval tv = { 0, 0 };
    if (select(M->fd + 1, 0, &w, 0, &tv) > 0) {
      int err = 0; socklen_t l = sizeof err; getsockopt(M->fd, SOL_SOCKET, SO_ERROR, &err, &l);
      if (err) { mq_close(M, "can't reach the broker", now); return -1; }
      hello(M, now);
    } else if (now - M->t_state > 3) { mq_close(M, "the broker didn't answer", now); return -1; }
    else return M->fd;
  }
  uint8_t buf[512];
  for (int k = 0; k < 64; k++) {
    ssize_t n = recv(M->fd, buf, sizeof buf, 0);
    if (n > 0) { M->t_rx = now; if (feed(M, buf, (int)n, now)) return -1; continue; }
    if (n == 0) { mq_close(M, "the broker closed the connection", now); return -1; }
    if (errno == EINTR) continue;
    if (errno == EAGAIN || errno == EWOULDBLOCK) break;
    mq_close(M, "the connection broke", now); return -1;
  }
  if (M->state == MQ_HELLO && now - M->t_state > 3) { mq_close(M, "no answer to CONNECT", now); return -1; }
  if (M->state == MQ_UP) {
    if (now - M->t_rx > 1.5 * M->keepalive) { mq_close(M, "the broker went quiet", now); return -1; }
    if (!M->txlen && now - M->t_tx >= 0.5 * M->keepalive) { const uint8_t ping[2] = { P_PINGREQ << 4, 0 }; queue(M, ping, 2, 0, 0, now); }
  }
  if (flush(M, now)) { mq_close(M, "the connection broke", now); return -1; }
  if (M->txlen && M->t_stall && now - M->t_stall > 2) { mq_close(M, "the broker isn't taking messages", now); return -1; }
  return M->fd;
}

int mq_publish(mq_client *M, const char *topic, const uint8_t *p, uint32_t n, double now) {
  if (M->state != MQ_UP) { M->dropped++; return -1; }
  size_t tl = strlen(topic); if (tl > 120) { M->dropped++; return -1; }
  uint8_t h[136]; int k = 0; h[k++] = P_PUBLISH << 4; k += put_len(h + k, (uint32_t)(2 + tl) + n); k += put_str(h + k, topic);
  if (queue(M, h, (uint32_t)k, p, n, now)) return -1;
  M->sent++;
  if (flush(M, now)) { mq_close(M, "the connection broke", now); return -1; }
  return 0;
}

int mq_parse_url(const char *url, char *host, int hostn, int *port, char *prefix, int prefixn) {
  if (!url) return -1;
  if (!strncmp(url, "mqtt://", 7)) url += 7;
  const char *e = url; while (*e && *e != ':' && *e != '/') e++;
  if (e == url || e - url >= hostn) return -1;
  memcpy(host, url, (size_t)(e - url)); host[e - url] = 0;
  *port = 1883;
  if (*e == ':') { char *q; long p = strtol(e + 1, &q, 10); if (q == e + 1 || p < 1 || p > 65535) return -1; *port = (int)p; e = q; }
  const char *pre = *e == '/' ? e + 1 : "liftlab";
  if (*e && *e != '/') return -1;
  size_t pl = strlen(pre); if (!pl || (int)pl >= prefixn || pre[pl - 1] == '/' || strpbrk(pre, "+#")) return -1;
  memcpy(prefix, pre, pl + 1);
  return 0;
}
void mq_link_topics(const char *prefix, char *to_fc, int to_n, char *from_fc, int from_n) {
  snprintf(to_fc, (size_t)to_n, "%s/link/to-fc", prefix); snprintf(from_fc, (size_t)from_n, "%s/link/from-fc", prefix);
}
