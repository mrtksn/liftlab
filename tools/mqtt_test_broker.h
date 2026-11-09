/* A small MQTT broker for the tests (tools/test_mqtt_min.c, tools/test_pi_mqtt_link.c): exact topics, QoS 0 (or 1 on
 * request), CONNACK code settable, on 127.0.0.1 at a free port; polled from the test's own loop. */
#ifndef MQTT_TEST_BROKER_H
#define MQTT_TEST_BROKER_H
#include <arpa/inet.h>
#include <assert.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#define BC 4
typedef struct { int fd; uint8_t in[70000]; uint32_t n; char sub[96]; int qos1; } bclient;
static struct { int ls, port, refuse, byte_by_byte, pings; bclient c[BC]; } BRK;
static void broker_open(void) {
  BRK.ls = socket(AF_INET, SOCK_STREAM, 0); int one = 1; setsockopt(BRK.ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  struct sockaddr_in a = { 0 }; a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  assert(!bind(BRK.ls, (struct sockaddr *)&a, sizeof a)); assert(!listen(BRK.ls, 4));
  socklen_t l = sizeof a; getsockname(BRK.ls, (struct sockaddr *)&a, &l); BRK.port = ntohs(a.sin_port);
  fcntl(BRK.ls, F_SETFL, O_NONBLOCK); for (int i = 0; i < BC; i++) BRK.c[i].fd = -1;
}
static void broker_drop_all(void) { for (int i = 0; i < BC; i++) if (BRK.c[i].fd >= 0) { close(BRK.c[i].fd); BRK.c[i].fd = -1; BRK.c[i].n = 0; BRK.c[i].sub[0] = 0; } }
static void wr(int fd, const uint8_t *p, size_t n) {
  if (BRK.byte_by_byte) { for (size_t i = 0; i < n; i++) { while (write(fd, p + i, 1) != 1) {} } return; }
  while (n) { ssize_t k = write(fd, p, n); if (k > 0) { p += k; n -= (size_t)k; } }
}
static int enc_len(uint8_t *b, uint32_t n) { int k = 0; do { uint8_t d = n % 128; n /= 128; if (n) d |= 128; b[k++] = d; } while (n); return k; }
static void route(const char *topic, const uint8_t *p, uint32_t n) {
  for (int i = 0; i < BC; i++) if (BRK.c[i].fd >= 0 && !strcmp(BRK.c[i].sub, topic)) {
    uint8_t h[200]; int k = 0, q = BRK.c[i].qos1; size_t tl = strlen(topic);
    h[k++] = (uint8_t)(0x30 | (q ? 2 : 0)); k += enc_len(h + k, (uint32_t)(2 + tl + (q ? 2 : 0)) + n);
    h[k++] = (uint8_t)(tl >> 8); h[k++] = (uint8_t)tl; memcpy(h + k, topic, tl); k += (int)tl; if (q) { h[k++] = 0; h[k++] = 7; }
    wr(BRK.c[i].fd, h, (size_t)k); wr(BRK.c[i].fd, p, n);
  }
}
static void broker_poll(void) {
  int fd = accept(BRK.ls, 0, 0);
  if (fd >= 0) for (int i = 0; i < BC; i++) if (BRK.c[i].fd < 0) { BRK.c[i].fd = fd; BRK.c[i].n = 0; BRK.c[i].sub[0] = 0; fcntl(fd, F_SETFL, O_NONBLOCK); break; }
  for (int i = 0; i < BC; i++) {
    bclient *C = &BRK.c[i]; if (C->fd < 0) continue;
    ssize_t r = read(C->fd, C->in + C->n, sizeof C->in - C->n);
    if (r == 0) { close(C->fd); C->fd = -1; continue; }
    if (r > 0) C->n += (uint32_t)r;
    for (;;) {                                    /* whole packets */
      if (C->n < 2) break;
      uint32_t rem = 0, mul = 1, k = 1; for (;; k++) { if (k >= C->n) goto more; rem += (C->in[k] & 127u) * mul; mul *= 128; if (!(C->in[k] & 128)) break; }
      k++; if (C->n < k + rem) break;
      uint8_t t = C->in[0] >> 4, *p = C->in + k;
      if (t == 1) { uint8_t a[4] = { 0x20, 2, 0, (uint8_t)BRK.refuse }; wr(C->fd, a, 4); }
      else if (t == 8) { uint32_t tl = (uint32_t)p[2] << 8 | p[3]; memcpy(C->sub, p + 4, tl); C->sub[tl] = 0; uint8_t a[5] = { 0x90, 3, p[0], p[1], 0 }; wr(C->fd, a, 5); }
      else if (t == 12) { uint8_t a[2] = { 0xd0, 0 }; wr(C->fd, a, 2); BRK.pings++; }
      else if (t == 3) { uint32_t tl = (uint32_t)p[0] << 8 | p[1]; char top[128]; memcpy(top, p + 2, tl); top[tl] = 0; route(top, p + 2 + tl, rem - 2 - tl); }
      memmove(C->in, C->in + k + rem, C->n - k - rem); C->n -= k + rem;
    }
    more:;
  }
}

#endif
