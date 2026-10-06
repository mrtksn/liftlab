/* A packet radio link's end over UDP, on Linux or a Mac: Wi-Fi (fc/radio_link.h: wifi,ap,CHANNEL or wifi,sta). See
 * radio_udp.c. */
#ifndef RADIO_UDP_H
#define RADIO_UDP_H
#include "radio_io.h"
#include "plink.h"
/* role: PLINK_DRONE (dfb_pi: listens on port, peer_host 0) or PLINK_GROUND (dfb_ground: sends to peer_host:port, the
 * drone's address). phrase: the binding phrase (both ends the same). 0 if it can't be opened (said on stderr). */
radio_io *radio_udp_open(int role, const char *peer_host, int port, const char *phrase, const char *name);
void radio_udp_close(radio_io *R);
/* Is R one of these (and not, say, a serial port)? */
int radio_udp_is(const radio_io *R);
/* The packet layer, for its counts (N) and link quality. */
const plink *radio_udp_plink(const radio_io *R);
/* Who it talks to now ("192.168.4.2:50123"; "nobody yet" before the first packet on the drone's end). */
void radio_udp_peer(const radio_io *R, char *out, int n);
/* One line of counts, for a status line: "wifi: sent 1200, got 1180, bad 0, …". */
void radio_udp_counts(const radio_io *R, char *out, int n);
#endif
