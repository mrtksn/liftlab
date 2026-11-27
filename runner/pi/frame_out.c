/* dfb_flow's pictures for whoever wants them: see frame_out.h. */
#define _DEFAULT_SOURCE
#include "frame_out.h"
#include <arpa/inet.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

int frame_out_open(frame_out *O, int port) {
  O->fd = -1; O->sent = O->dropped = 0; O->have_gain = 0;
  O->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (O->listen_fd < 0) return -1;
  int one = 1; setsockopt(O->listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  struct sockaddr_in a; memset(&a, 0, sizeof a); a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port); a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (bind(O->listen_fd, (struct sockaddr *)&a, sizeof a) < 0 || listen(O->listen_fd, 1) < 0) { close(O->listen_fd); O->listen_fd = -1; return -1; }
  fcntl(O->listen_fd, F_SETFL, fcntl(O->listen_fd, F_GETFL) | O_NONBLOCK);
  return 0;
}

int frame_out_wanted(frame_out *O) {
  if (O->fd >= 0) return 1;
  if (O->listen_fd < 0) return 0;
  int c = accept(O->listen_fd, NULL, NULL);
  if (c < 0) return 0;
  struct timeval tv = { 0, 50000 };                       /* (a send that takes over 50 ms: the reader has fallen behind) */
  setsockopt(c, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
  int buf = 512 * 1024; setsockopt(c, SOL_SOCKET, SO_SNDBUF, &buf, sizeof buf);
#ifdef SO_NOSIGPIPE
  int one = 1; setsockopt(c, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
  O->fd = c; return 1;
}

/* a raw pixel's top 8 bits */
static inline uint8_t px(const uint8_t *row, int x, int raw10) { return raw10 ? row[x / 4 * 5 + x % 4] : row[x]; }

void frame_out_rgb(uint8_t *rgb, const uint8_t *raw, int bpl, int W, int H, int raw10, int order) {
  /* where red and blue sit in the 2×2 block (0 top left, 1 top right, 2 bottom left, 3 bottom right); green the others */
  static const int RED[4] = { 3, 2, 1, 0 }, BLUE[4] = { 0, 1, 2, 3 };
  const int ri = RED[order & 3], bi = BLUE[order & 3];
  for (int y = 0; y + 1 < H; y += 2) {
    const uint8_t *r0 = raw + (size_t)y * (size_t)bpl, *r1 = r0 + bpl;
    uint8_t *o = rgb + (size_t)(y / 2) * (size_t)(W / 2) * 3;
    for (int x = 0; x + 1 < W; x += 2) {
      int q[4] = { px(r0, x, raw10), px(r0, x + 1, raw10), px(r1, x, raw10), px(r1, x + 1, raw10) }, g = 0;
      for (int k = 0; k < 4; k++) if (k != ri && k != bi) g += q[k];
      *o++ = (uint8_t)q[ri]; *o++ = (uint8_t)(g / 2); *o++ = (uint8_t)q[bi];
    }
  }
}

void frame_out_balance(frame_out *O, uint8_t *rgb, int w, int h) {
  double sum[3] = { 0, 0, 0 }; long n = 0;
  for (int y = 0; y < h; y += 4) for (int x = 0; x < w; x += 4) { const uint8_t *p = rgb + ((size_t)y * (size_t)w + (size_t)x) * 3; sum[0] += p[0]; sum[1] += p[1]; sum[2] += p[2]; n++; }
  double m[3], grey = 0; for (int c = 0; c < 3; c++) { m[c] = sum[c] / (n ? n : 1) + 1; grey += m[c] / 3; }
  double lift = 46 / grey; if (lift > 4) lift = 4; if (lift < 0.5) lift = 0.5;              /* (toward middle grey: 18 % linear, ~117 after the gamma) */
  for (int c = 0; c < 3; c++) {
    float want = (float)(grey / m[c]);
    O->gain[c] = O->have_gain ? 0.8f * O->gain[c] + 0.2f * want : want;                     /* (eased: no flicker) */
    double k = O->gain[c] * lift / 255;
    for (int v = 0; v < 256; v++) { double x = v * k; if (x > 1) x = 1; O->lut[c][v] = (uint8_t)(pow(x, 1 / 2.2) * 255 + 0.5); }
  }
  O->have_gain = 1;
  for (size_t i = 0, N = (size_t)w * (size_t)h * 3; i < N; i += 3) { rgb[i] = O->lut[0][rgb[i]]; rgb[i + 1] = O->lut[1][rgb[i + 1]]; rgb[i + 2] = O->lut[2][rgb[i + 2]]; }
}

static int send_all(int fd, const void *p, size_t n) {
  const uint8_t *b = p;
  while (n) { ssize_t k = send(fd, b, n, MSG_NOSIGNAL); if (k <= 0) { if (k < 0 && errno == EINTR) continue; return -1; } b += k; n -= (size_t)k; }
  return 0;
}
static void put32(uint8_t *b, uint32_t v) { for (int i = 0; i < 4; i++) b[i] = (uint8_t)(v >> (8 * i)); }

int frame_out_send(frame_out *O, const uint8_t *rgb, int w, int h, uint64_t t_ns, uint32_t seq, uint32_t exp_us, uint32_t gain_x100) {
  if (O->fd < 0) return -1;
  uint8_t hd[40]; uint32_t bytes = (uint32_t)(w * h * 3);
  memcpy(hd, "DFF1", 4); put32(hd + 4, (uint32_t)w); put32(hd + 8, (uint32_t)h); put32(hd + 12, (uint32_t)t_ns); put32(hd + 16, (uint32_t)(t_ns >> 32));
  put32(hd + 20, seq); put32(hd + 24, exp_us); put32(hd + 28, gain_x100); put32(hd + 32, bytes);
  if (send_all(O->fd, hd, 36) || send_all(O->fd, rgb, bytes)) { close(O->fd); O->fd = -1; O->dropped++; return -1; }   /* (gone, or too slow: let go) */
  O->sent++; return 0;
}

void frame_out_close(frame_out *O) { if (O->fd >= 0) close(O->fd); if (O->listen_fd >= 0) close(O->listen_fd); O->fd = O->listen_fd = -1; }
