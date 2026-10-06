/* The pilot's radio link: HOW the frames travel between the command module and the drone.
 *
 * What travels is the same over every link: CRSF frames (crsf.c), the channels and the ground station's commands up,
 * the telemetry (tlm_core.c through tlm_crsf.c) and link statistics down, made and read by the same code at both ends
 * (rc_core.c, ground_core.c). A link only carries those bytes. What differs from link to link is said here: its name,
 * its settings, and how much room it leaves the telemetry; and in radio_io.h (../radio_io.h), how a program moves its
 * bytes on the hardware. A new link is a kind here plus a radio_io for each platform it runs on.
 *
 * Kinds:
 *   - ExpressLRS: a transmitter module and a receiver on UARTs; the modules do the radio's part themselves.
 *     Settings: elrs,RATE,RATIO (50/150/250/500 Hz; telemetry one packet in 2…128).
 *   - ESP-NOW: an ESP32 at each end, talking directly (no network). Settings: espnow,CHANNEL[,lr] (Wi-Fi channel 1–13;
 *     lr: Espressif's long-range mode, slower and further).
 *   - Wi-Fi: UDP over a Wi-Fi network (port RLINK_UDP_PORT). Settings: wifi,ap,CHANNEL (the drone makes the network,
 *     the ground joins it: a laptop, say) or wifi,sta (the drone joins a network; its name and password are a
 *     setting of the board's own).
 * The packet links (ESP-NOW, Wi-Fi) do the modules' part in our own code, the same at both ends: plink.h. Both ends
 * need the same binding phrase (a setting of each program): it signs the packets.
 * To come: nRF24L01, Bluetooth LE. */
#ifndef RADIO_LINK_H
#define RADIO_LINK_H
#include "rc_core.h"

enum { RLINK_ELRS = 0, RLINK_ESPNOW, RLINK_WIFI, RLINK_KINDS };
#define RLINK_UDP_PORT 14570
typedef struct {
  int kind;
  int rate_hz, ratio;          /* ExpressLRS: the packet rate [Hz] and the telemetry ratio (one packet in `ratio`) */
  int channel;                 /* ESP-NOW, Wi-Fi (access point): the Wi-Fi channel, 1–13 */
  int lr;                      /* ESP-NOW: long range */
  int sta;                     /* Wi-Fi: join a network (1) or make one (0) */
} rlink_cfg;

extern const char *const rlink_names[RLINK_KINDS];    /* as settings write it: "elrs" */
extern const char *const rlink_labels[RLINK_KINDS];   /* for people: "ExpressLRS 2.4 GHz" */
void rlink_default(rlink_cfg *L);                     /* ExpressLRS at 250 Hz, telemetry 1:4 */
/* "elrs,250,4", or "250,4" (ExpressLRS, as the setting was written before there were other links). 0, or −1 with
 * why in err (then L is unchanged). */
int rlink_parse(rlink_cfg *L, const char *s, char *err, int en);
int rlink_describe(const rlink_cfg *L, char *out, int n);    /* back as rlink_parse takes it */
/* The same from numbers: kind, then its settings in order (ExpressLRS: rate, ratio; ESP-NOW: channel, long range;
 * Wi-Fi: joins a network, channel). 0, or −1 (L unchanged). */
int rlink_make(rlink_cfg *L, int kind, int a, int b);
/* The telemetry's room [bytes/s]: on a good link, and as the link is now (from the link statistics the drone's end
 * reports into rc_input; nothing while it reports nothing for a second). */
float rlink_budget(const rlink_cfg *L);
float rlink_budget_now(const rlink_cfg *L, const rc_input *in, double t);
/* A packet link (plink.h does the modules' part): 1; ExpressLRS: 0. */
static inline int rlink_packets(const rlink_cfg *L) { return L->kind == RLINK_ESPNOW || L->kind == RLINK_WIFI; }
#endif
