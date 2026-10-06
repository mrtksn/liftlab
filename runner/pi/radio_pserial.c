/* A packet link's end over a serial line, on Linux or a Mac (radio_io.h): the serial link (fc/radio_link.h:
 * serial,BAUD[,half]), the drone's end on the Pi (dfb_pi --radio serial,115200 --crsf /dev/ttyAMA1) and the command
 * module's on a computer or a Pi (dfb_ground --radio serial,115200 --tx /dev/ttyUSB0). Whatever is on the port
 * carries its bytes to the other end: a laser or LED and a photodiode, fibre transceivers, an infrared pair, a radio
 * modem in transparent mode, a wire. There is no module that makes packets: this does its part (fc/plink.h, the same
 * at both ends), and the packets are marked out in the byte stream (fc/pframe.h: 0, COBS, 0).
 *
 * As radio_udp.c: write() gives the program's CRSF frames to the packet layer; read() takes what came, sends a packet
 * if one is due (the packets' timing is driven from here: the program calls read() every few ms) and hands back the
 * frames that came and the link statistics. The signal strength is unknown (0): the link quality is counted from the
 * packet numbers. The port is never given more than the line can carry: a packet that finds the last one still
 * waiting to go out is dropped (counted: "line busy"). */
#include "radio_pserial.h"
#include "radio_serial.h"
#include "pframe.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

typedef struct {
  plink P; pframe_rx F;
  radio_io *port;
  rlink_cfg L;
  uint8_t out[2 * PFRAME_WIRE(PLINK_MTU)]; int out_n;   /* bytes still to go to the port */
  uint32_t busy, write_errors;
  double t_good, t_bad_warned; uint32_t bad_seen;
  const char *name;
} ps_ctx;

static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static uint32_t random_session(void) {
  uint32_t s = 0;
  int fd = open("/dev/urandom", O_RDONLY);
  if (fd >= 0) { if (read(fd, &s, sizeof s) != (ssize_t)sizeof s) s = 0; close(fd); }
  if (!s) { struct timespec t; clock_gettime(CLOCK_REALTIME, &t); s = (uint32_t)t.tv_nsec ^ (uint32_t)t.tv_sec * 2654435761u ^ (uint32_t)getpid() << 16; }
  return s ? s : 1;
}

static void flush_out(ps_ctx *C) {
  if (!C->out_n) return;
  ssize_t k = write(C->port->fd, C->out, (size_t)C->out_n);
  if (k < 0) { if (errno != EAGAIN && errno != EINTR) { C->write_errors++; C->out_n = 0; } return; }
  memmove(C->out, C->out + k, (size_t)(C->out_n - k)); C->out_n -= (int)k;
}
static void pump(ps_ctx *C, double t) {
  flush_out(C);
  uint8_t pkt[PLINK_MTU]; int n = plink_to_air(&C->P, t, pkt, sizeof pkt);
  if (!n) return;
  if (C->out_n > 0) { C->busy++; return; }           /* (the last one still going: the line is full, this one is late anyway) */
  C->out_n = pframe_encode(pkt, n, C->out, sizeof C->out);
  flush_out(C);
}
static int ps_read(radio_io *R, uint8_t *b, int n, int wait_ms) {
  ps_ctx *C = R->ctx; (void)wait_ms;                 /* (the program polls R->fd: this never waits) */
  double t = now_s();
  uint8_t in[512], pkt[PFRAME_MAX];
  for (int i = 0; i < 16; i++) {
    ssize_t k = read(R->fd, in, sizeof in);
    if (k <= 0) break;
    for (ssize_t j = 0; j < k; j++) {
      int m = pframe_feed(&C->F, in[j], pkt, sizeof pkt);
      if (m && plink_from_air(&C->P, pkt, m, 0, t) == 1) C->t_good = t;
    }
  }
  if (C->P.N.bad != C->bad_seen) {
    C->bad_seen = C->P.N.bad;
    if (t - C->t_good > 2 && t - C->t_bad_warned > 30) {
      C->t_bad_warned = t;
      fprintf(stderr, "\r%s: packets come that fail the binding phrase's check: is --bind the same at both ends? (or the line is that noisy)\r\n", C->name);
    }
  }
  pump(C, t);
  return plink_to_stack(&C->P, t, b, n);
}
static int ps_write(radio_io *R, const uint8_t *b, int n) {
  ps_ctx *C = R->ctx; double t = now_s();
  plink_from_stack(&C->P, b, n, t);
  pump(C, t);
  return n;
}

radio_io *radio_pserial_open(int role, const char *dev, const rlink_cfg *L, const char *phrase, const char *name) {
  if (L->kind != RLINK_SERIAL) return 0;
  radio_io *port = radio_serial_open(dev, L->baud, name); if (!port) return 0;
  radio_io *R = calloc(1, sizeof *R); ps_ctx *C = calloc(1, sizeof *C);
  if (!R || !C) { free(R); free(C); radio_serial_close(port); return 0; }
  plink_cfg cfg; plink_cfg_default(&cfg, role); plink_cfg_link(&cfg, L); plink_key(phrase ? phrase : "liftlab", &cfg.k0, &cfg.k1);
  plink_init(&C->P, &cfg, random_session()); pframe_rx_init(&C->F);
  C->port = port; C->L = *L; C->name = name; C->t_good = now_s(); C->t_bad_warned = -1e9;
  R->name = name; R->read = ps_read; R->write = ps_write; R->fd = port->fd; R->ctx = C;
  return R;
}
void radio_pserial_close(radio_io *R) { if (!R) return; ps_ctx *C = R->ctx; radio_serial_close(C->port); free(C); free(R); }
int radio_pserial_is(const radio_io *R) { return R && R->read == ps_read; }
const plink *radio_pserial_plink(const radio_io *R) { return radio_pserial_is(R) ? &((const ps_ctx *)R->ctx)->P : 0; }
void radio_pserial_counts(const radio_io *R, char *out, int n) {
  const ps_ctx *C = R->ctx; const plink_counts *N = &C->P.N; char d[32]; rlink_describe(&C->L, d, sizeof d);
  snprintf(out, (size_t)n, "%s: packets sent %u (line busy %u), got %u (LQ %d%%), bad %u, frames that didn't decode %u, replays %u; frames resent %u, dropped %u",
           d, N->sent, C->busy, N->got, plink_lq(&C->P, now_s()), N->bad, C->F.bad, N->replays, N->resent, N->uq_dropped);
}
