/* A serial port at a speed termios has no constant for (CRSF's 420000 baud): on Linux termios2 with BOTHER, on macOS
 * IOSSIOSPEED.
 * Kept apart because <asm/termbits.h> and <termios.h> can't be included together. */
#if defined(__linux__)
#include <asm/termbits.h>
#include <sys/ioctl.h>
int serial_custom_baud(int fd, int baud) {
  struct termios2 t;
  if (ioctl(fd, TCGETS2, &t)) return -1;
  t.c_cflag &= ~CBAUD; t.c_cflag |= BOTHER; t.c_ispeed = t.c_ospeed = (speed_t)baud;
  return ioctl(fd, TCSETS2, &t);
}
#elif defined(__APPLE__)
/* macOS: any speed the adapter supports, with IOSSIOSPEED (after the port is set up with a standard one). */
#include <IOKit/serial/ioss.h>
#include <sys/ioctl.h>
int serial_custom_baud(int fd, int baud) { speed_t s = (speed_t)baud; return ioctl(fd, IOSSIOSPEED, &s); }
#else
int serial_custom_baud(int fd, int baud) { (void)fd; (void)baud; return -1; }
#endif
