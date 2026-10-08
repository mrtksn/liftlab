/* The Pi companion has one flight UART. Nonblocking writes retain frame tails.
 */
#ifndef PI_SERIAL_TX_H
#define PI_SERIAL_TX_H
#include "fc_core.h"
#include "rn_link.h"
#include <errno.h>
#include <string.h>
#include <unistd.h>
/* Include after the monotonic now_s() clock helper. */
/* Preserve frame boundaries over nonblocking short writes. A stalled UART must
 * not replay old flight commands: quit and let the core's independent watchdog
 * act. */
static uint8_t tx_queue[8192];
static size_t tx_len;
static double tx_since;
static int tx_failed;
static void flush_link(int fd) {
  if (tx_failed)
    return;
  if (tx_len && now_s() - tx_since > .04) {
    tx_failed = 1;
    return;
  }
  while (tx_len) {
    ssize_t n = write(fd, tx_queue, tx_len);
    if (n > 0) {
      memmove(tx_queue, tx_queue + n, tx_len - (size_t)n);
      tx_len -= (size_t)n;
    } else if (n < 0 && errno == EINTR)
      continue;
    else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      break;
    else {
      tx_failed = 1;
      break;
    }
  }
  if (!tx_len)
    tx_since = 0;
  else if (now_s() - tx_since > .04)
    tx_failed = 1;
}
static void send_frame(int fd, uint8_t type, const void *p, uint32_t n) {
  uint8_t fr[FC_MODEL_MAX * 4 + 32];
  uint32_t len = rn_link_frame(fr, sizeof fr, type, (const uint8_t *)p, n);
  flush_link(fd);
  if (!len || tx_failed)
    return;
  if (tx_len + len > sizeof tx_queue) {
    tx_failed = 1;
    return;
  }
  if (!tx_len)
    tx_since = now_s();
  memcpy(tx_queue + tx_len, fr, len);
  tx_len += len;
  flush_link(fd);
}

#endif
