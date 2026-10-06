/* The nRF24L01 link on a Pi or a Linux computer: see radio_nrf24.c. */
#ifndef RADIO_NRF24_H
#define RADIO_NRF24_H
#include "radio_io.h"
#include "nrf24.h"
/* role: PLINK_DRONE (dfb_pi) or PLINK_GROUND (dfb_ground); L: nrf24,KBPS; spidev: /dev/spidev0.0; ce_line: the GPIO
 * for CE. 0 if it can't be opened or no module answers (said on stderr). */
radio_io *radio_nrf24_open(int role, const rlink_cfg *L, const char *spidev, int ce_line, const char *phrase, const char *name);
void radio_nrf24_close(radio_io *R);
int radio_nrf24_is(const radio_io *R);
void radio_nrf24_counts(const radio_io *R, char *out, int n);
#endif
