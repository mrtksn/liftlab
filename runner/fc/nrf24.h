/* The nRF24L01(+) radio, the link's part that is the same on every board: its registers set up for the link
 * (Enhanced ShockBurst, 2-byte CRC, dynamic payloads, ACK payloads, 3 retries), and the packet layer (clink.h) driven
 * through it. A board gives it an SPI transfer and the CE pin (nrf24_hal); everything else is here.
 *
 * The ground's module is the primary transmitter: on its beat it goes to the channel clink says, sends the packet
 * clink made, and the radio waits for the acknowledgement, sending again up to 3 times; the acknowledgement brings
 * the drone's answer, which goes to clink. The drone's module listens, on the channel clink says (it follows the
 * ground's hops); each packet it takes goes to clink, and clink's answer is loaded as the payload of the next
 * acknowledgement. So the drone never transmits on its own, and the two never talk at once.
 *
 * Wiring (both ends): VCC 3.3 V (never 5 V) with a 10 µF capacitor or more across VCC and GND at the module (its
 * current comes in bursts the wires can't carry: without it most of these modules drop packets), GND, SCK, MOSI, MISO,
 * CSN, CE; IRQ not used.
 *
 * No C library: it also builds for the simulator. */
#ifndef NRF24_H
#define NRF24_H
#include <stdint.h>
#include "clink.h"

typedef struct {
  /* one SPI transaction: n bytes out and in at once, CSN low around it. 0, or −1 */
  int (*xfer)(void *ctx, const uint8_t *tx, uint8_t *rx, int n);
  void (*ce)(void *ctx, int level);
  void (*delay_us)(void *ctx, int us);
  void *ctx;
} nrf24_hal;

typedef struct {
  nrf24_hal H;
  clink L;
  int role, kbps, ch;             /* ch: the channel the radio is on (−1: none yet) */
  int busy; double t_busy;        /* the ground: a packet on its way, since */
  uint32_t lost, timeouts, too_long;
  uint8_t addr[5];
} nrf24_link;

/* The module set up for the link (role PLINK_GROUND or PLINK_DRONE; kbps 250, 1000 or 2000; L's key and session
 * already in C, session). 0, or −1 if no nRF24L01 answers on the SPI bus (why in err). */
int nrf24_start(nrf24_link *N, const nrf24_hal *H, const clink_cfg *C, uint32_t session, int role, int kbps, char *err, int en);
/* Every 1–4 ms: the radio's events (a packet taken, an acknowledgement come or not), and what's due (the ground's
 * next packet, the drone's next answer loaded, a hop). */
void nrf24_poll(nrf24_link *N, double t);
/* Is a module there: the register test nrf24_start does (also for a status line). */
int nrf24_present(const nrf24_hal *H);
#endif
