/* A radio link's bytes, as a program moves them on its hardware: the platform's half of a link (fc/radio_link.h
 * says which link it is and what room it has). Both directions carry CRSF frames, whatever the link: what the
 * receiver or transmitter module hands over, and what goes to it.
 *
 * Today: ExpressLRS, a receiver or a transmitter module on a UART (fc/esp32/main/radio_elrs.c on the drone's ESP32,
 * ground/esp32/main/radio_module.c on the command module's, pi/radio_serial.c on a Pi or a PC). A link that has no
 * module of its own (ESP-NOW, UDP, an nRF24L01) makes the module's part here: it packs the frames into its packets,
 * and hands back link statistics frames of its own, as a receiver or transmitter module would. */
#ifndef RADIO_IO_H
#define RADIO_IO_H
#include <stdint.h>

typedef struct radio_io {
  const char *name;
  /* Up to n bytes that came in, waiting at most wait_ms for the first (0: don't wait). Returns the bytes, 0 for none,
   * −1 if the link's hardware failed. */
  int (*read)(struct radio_io *R, uint8_t *b, int n, int wait_ms);
  /* Sends n bytes (whole frames). Returns the bytes taken (fewer: try the rest later), −1 if it failed. */
  int (*write)(struct radio_io *R, const uint8_t *b, int n);
  int fd;                      /* on Linux: a descriptor to poll() for input, −1 if none */
  void *ctx;                   /* the link's own state */
  /* (optional: packet links, for two links at once, fc/radio_mux.h) The other end's session as this link knows it
   * (0: none yet); a one-way link beside a two-way one: listen only to that sender (fc/plink.h plink_tie); and what
   * this end hears of the other over both links, to say in its packets (plink_hear). */
  uint32_t (*peer)(struct radio_io *R);
  void (*tie)(struct radio_io *R, uint32_t peer);
  void (*hear)(struct radio_io *R, int lq, int rssi);
} radio_io;
#endif
