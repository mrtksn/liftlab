/* Production saved-gain store and flight-UART backpressure, with actual
 * files/pipes. */
#define _DEFAULT_SOURCE
#include <assert.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
static double clock_s = 1;
static double now_s(void) { return clock_s; }
#include "../runner/pi/serial_tx.h"
#include "../runner/pi/tuning_store.h"

static void transactions(void) {
  pid_tuning t = {0};
  pid_defaults(t.accepted, 0);
  float p[PID_FRAME] = {PID_VERSION, 3, 16777215};
  pid_defaults(p + 3, 0);
  p[3] = 110;
  assert(pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0));
  assert(!pid_frame(&t, p, PID_FRAME, 0, 1, 0, 1));
  p[1] = 1;
  p[2] = 2;
  p[3] = 115;
  assert(pid_frame(&t, p, PID_FRAME, 0, 1, 0, 1));
  assert(!pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0));
  p[3] = 116;
  assert(pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0));
  p[3] = 115;
  p[1] = 0;
  assert(!pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0));
  assert(!t.pending && t.accepted[0] == 110);
  p[1] = 1;
  p[2] = 1;
  assert(pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0)); /* older session */
  p[2] = 3;
  assert(!pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0));
  pid_tick(&t, .3f, 1, 1, 0);
  assert(!t.pending);
  assert(pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0)); /* expired refresh */
  p[2] = 4;
  assert(!pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0));
  p[1] = 2;
  assert(!pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0));
  assert(t.enabled && !t.pending && t.accepted[0] == 115);
  assert(!pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0)); /* idempotent acceptance */
  p[2] = 5;
  assert(pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0));
  p[1] = 1;
  assert(!pid_frame(&t, p, PID_FRAME, 0, 1, 1, 0));
  pid_tick(&t, NAN, 1, 1, 0);
  assert(!t.pending && t.accepted[0] == 115);
  pid_tick(&t, .001f, 0, 1, 0);
  assert(!t.enabled && t.accepted[0] == 100);
  pid_defaults(p + 3, 1);
  p[1] = 3;
  p[2] = 6;
  assert(!pid_frame(&t, p, PID_FRAME, 1, 1, 0, 1));
  p[4] = 5;
  assert(pid_frame(&t, p, PID_FRAME, 1, 1, 0, 1));
  puts("Disarmed restoration, transaction ordering, exact refresh, expiry, "
       "idempotent acceptance, invalid clock and program fallback passed");
}

int main(void) {
  transactions();
  char dir[] = "/tmp/liftlab-pi-tuning.XXXXXX";
  assert(mkdtemp(dir));
  char file[256];
  snprintf(file, sizeof file, "%s/drone.dft", dir);
  float g[12], readback[12];
  pid_defaults(g, 0);
  g[9] = 4;
  g[10] = 3.6f;
  g[11] = 1;
  assert(!tuning_file(file, g, 123, 456, 1));
  assert(!tuning_file(file, readback, 123, 456, 0));
  assert(!memcmp(g, readback, sizeof g));
  assert(tuning_file(file, readback, 124, 456, 0));
  assert(tuning_file(file, readback, 123, 457, 0));
  g[0] = NAN;
  assert(tuning_file(file, g, 123, 456, 1));
  assert(!tuning_file(file, readback, 123, 456, 0));
  g[0] = 100;
  FILE *f = fopen(file, "r+b");
  assert(f);
  assert(!fseek(f, 20, SEEK_SET));
  int c = fgetc(f);
  assert(!fseek(f, 20, SEEK_SET));
  fputc(c ^ 1, f);
  fclose(f);
  assert(tuning_file(file, readback, 123, 456, 0));
  assert(!tuning_file(file, g, 123, 456, 1));
  f = fopen(file, "ab");
  assert(f);
  fputc(0, f);
  fclose(f);
  assert(tuning_file(file, readback, 123, 456, 0));
  assert(!tuning_file(file, g, 123, 456, 1));
  g[9] = 0;
  assert(tuning_file(file, g, 123, 456, 1));
  assert(!unlink(file));
  assert(!rmdir(dir));

  int fd[2];
  assert(!pipe(fd));
  assert(fcntl(fd[0], F_SETFL, O_NONBLOCK) >= 0);
  assert(fcntl(fd[1], F_SETFL, O_NONBLOCK) >= 0);
  uint8_t fill[4096] = {0}, rx[16384];
  ssize_t n;
  while (write(fd[1], fill, sizeof fill) > 0) {
  }
  assert(errno == EAGAIN || errno == EWOULDBLOCK);
  float p[PID_FRAME] = {PID_VERSION, 1, 42};
  pid_defaults(p + 3, 0);
  send_frame(fd[1], RN_LINK_TUNE, p, sizeof p);
  assert(tx_len > 0 && !tx_failed);
  while (read(fd[0], rx, sizeof rx) > 0) {
  }
  assert(errno == EAGAIN || errno == EWOULDBLOCK);
  flush_link(fd[1]);
  assert(!tx_len && !tx_failed);
  n = read(fd[0], rx, sizeof rx);
  assert(n > 0);
  rn_link L;
  uint8_t payload[64];
  rn_link_init(&L, payload, sizeof payload);
  int got = 0;
  for (ssize_t i = 0; i < n; i++)
    got = rn_link_feed(&L, rx[i]);
  assert(got == RN_LINK_TUNE && L.len == sizeof p &&
         !memcmp(L.buf, p, sizeof p));
  while (write(fd[1], fill, sizeof fill) > 0) {
  }
  send_frame(fd[1], RN_LINK_TUNE, p, sizeof p);
  assert(tx_len);
  clock_s += .041;
  flush_link(fd[1]);
  assert(tx_failed);
  close(fd[0]);
  close(fd[1]);
  puts("Pi saved gains roundtrip, design/CRC/length/bounds rejection, retained "
       "old file and UART backlog expiry passed");
  return 0;
}
