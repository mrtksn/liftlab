/* Packets over a byte pipe: a link that is only a serial line (a laser or LED and a photodiode, a fibre's
 * transceivers, an infrared pair, a radio modem in transparent mode such as an HC-12 or a SiK radio, a wire). The
 * line carries bytes with no packets of its own, and on a noisy medium some bytes come wrong or not at all; the packet
 * layer (plink.h) needs whole packets. So each packet goes as
 *
 *     0x00, COBS(packet), 0x00
 *
 * Consistent Overhead Byte Stuffing (Cheshire, Baker) rewrites the packet with no zero byte in it, at a cost of one
 * byte in 254, so a zero only ever marks where a packet starts or ends: after any damage, the receiver is back in
 * step at the next zero. (The zero before as well as after: noise on an idle line ends up a short frame of its own,
 * not stuck to the front of the next packet.) A damaged packet is dropped by the packet layer's signature check (8
 * bytes, the binding phrase's key): one wrong bit anywhere is caught, so no CRC is needed here. A frame that doesn't
 * decode (a lost or extra byte where a COBS code was, a frame too long) is counted here.
 *
 * No C library: it also builds for the simulator. */
#ifndef PFRAME_H
#define PFRAME_H
#include <stdint.h>

#define PFRAME_MAX 256                 /* the biggest packet [bytes] (plink.h PLINK_MTU and some) */
#define PFRAME_WIRE(n) ((n) + (n) / 254 + 3)   /* a packet of n bytes on the line, at most */

/* The bytes to send for a packet: 0, COBS, 0. Their count, or 0 if out has no room (cap ≥ PFRAME_WIRE(n)). */
int pframe_encode(const uint8_t *pkt, int n, uint8_t *out, int cap);

typedef struct {
  uint8_t buf[PFRAME_WIRE(PFRAME_MAX)]; int n;   /* the frame so far (as it came, still stuffed) */
  int over;                                      /* longer than any packet: skip to the next zero */
  uint32_t frames, bad, bytes;                   /* packets out; frames that didn't decode or were too long; bytes in */
} pframe_rx;

void pframe_rx_init(pframe_rx *R);
/* One byte from the line. A packet ended by it: its length (copied to pkt, cap ≥ PFRAME_MAX); else 0. */
int pframe_feed(pframe_rx *R, uint8_t b, uint8_t *pkt, int cap);
#endif
