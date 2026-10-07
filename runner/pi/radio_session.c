/* See radio_session.h. */
#define _DEFAULT_SOURCE
#include "radio_session.h"
#include <fcntl.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

uint32_t radio_session(void) {
  static uint32_t s;
  if (s) return s;
#if defined(__APPLE__) || defined(__FreeBSD__) || defined(__OpenBSD__) || defined(__NetBSD__)
  arc4random_buf(&s, sizeof s);
#else
  int fd = open("/dev/urandom", O_RDONLY);
  if (fd >= 0) { if (read(fd, &s, sizeof s) != (ssize_t)sizeof s) s = 0; close(fd); }
#endif
  if (!s) { struct timespec t; clock_gettime(CLOCK_REALTIME, &t); s = (uint32_t)t.tv_nsec ^ (uint32_t)t.tv_sec * 2654435761u ^ (uint32_t)getpid() << 16; }
  if (!s) s = 1;
  return s;
}
