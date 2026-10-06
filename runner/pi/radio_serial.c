/* A radio link's end on a serial port, on Linux or a Mac (radio_io.h): an ExpressLRS receiver (the drone's Pi,
 * dfb_pi --crsf) or transmitter module (the command module, dfb_ground --tx), CRSF both ways. The module does the
 * radio's part itself: this only moves its bytes. */
#include "radio_serial.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

int serial_custom_baud(int fd, int baud);

static int serial_read(radio_io *R, uint8_t *b, int n, int wait_ms) {
  (void)wait_ms;                                   /* (poll() R->fd for input: this never waits) */
  ssize_t k = read(R->fd, b, (size_t)n);
  if (k < 0) return errno == EAGAIN || errno == EINTR ? 0 : -1;
  return (int)k;
}
static int serial_write(radio_io *R, const uint8_t *b, int n) {
  ssize_t k = write(R->fd, b, (size_t)n);
  if (k < 0) return errno == EAGAIN || errno == EINTR ? 0 : -1;
  return (int)k;
}

radio_io *radio_serial_open(const char *dev, int baud, const char *name) {
  int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) { perror(dev); return 0; }
  struct termios t; tcgetattr(fd, &t); cfmakeraw(&t); t.c_cflag &= ~(tcflag_t)CRTSCTS;   /* (no flow control: the module has no RTS/CTS) */
  speed_t sp = B115200; int standard = baud == 115200;
#ifdef B230400
  if (baud == 230400) { sp = B230400; standard = 1; }
#endif
#ifdef B460800
  if (baud == 460800) { sp = B460800; standard = 1; }
#endif
#ifdef B921600
  if (baud == 921600) { sp = B921600; standard = 1; }
#endif
  cfsetispeed(&t, sp); cfsetospeed(&t, sp); t.c_cflag |= CLOCAL | CREAD; t.c_cc[VMIN] = 0; t.c_cc[VTIME] = 0;
  tcsetattr(fd, TCSANOW, &t);
  if (!standard && serial_custom_baud(fd, baud)) { fprintf(stderr, "%s: can't set %d baud\n", dev, baud); close(fd); return 0; }
  tcflush(fd, TCIOFLUSH);
  radio_io *R = calloc(1, sizeof *R); if (!R) { close(fd); return 0; }
  R->name = name; R->read = serial_read; R->write = serial_write; R->fd = fd; R->ctx = 0;
  return R;
}
void radio_serial_close(radio_io *R) { if (!R) return; if (R->fd >= 0) close(R->fd); free(R); }
