/*
 * The link between the companion computer (Raspberry Pi) and the flight controller, over a UART.
 *
 * A frame: 'D' 'F', type (1 byte), payload length (4 bytes, little-endian), payload, CRC-32 of the type, the
 * length and the payload (4 bytes, little-endian). The CRC is the same as the program image's (rn_crc32).
 *   Pi → drone:  RN_LINK_PROGRAM  a program image (rn_host_prepare it)
 *                RN_LINK_STATUS   ask what's flying
 *   drone → Pi:  RN_LINK_EVENT    one line of text: "loaded", "rejected: …", "swapped", "fell back: …", …
 *                RN_LINK_REPORT   one line of text: the answer to RN_LINK_STATUS
 * pi/send_program.py is the Pi's side.
 */
#ifndef RN_LINK_H
#define RN_LINK_H
#include <stdint.h>

enum { RN_LINK_PROGRAM = 1, RN_LINK_STATUS = 2, RN_LINK_EVENT = 0x81, RN_LINK_REPORT = 0x82 };

typedef struct {
  uint8_t *buf; uint32_t cap;       /* where payloads are collected */
  int state; uint8_t hdr[5]; uint32_t got, len; uint8_t type;
} rn_link;

void rn_link_init(rn_link *L, uint8_t *buf, uint32_t cap);
/* Feed one received byte. Returns the frame's type when a whole frame with a good CRC has arrived (the payload is
 * in L->buf, L->len bytes), −1 when a frame was dropped (bad CRC or too long for the buffer), 0 otherwise. */
int rn_link_feed(rn_link *L, uint8_t byte);
/* Build a frame into out. Returns its length, or 0 if it doesn't fit. */
uint32_t rn_link_frame(uint8_t *out, uint32_t cap, uint8_t type, const uint8_t *payload, uint32_t len);

#endif
