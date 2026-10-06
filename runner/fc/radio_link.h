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
 *   - A serial line (a byte pipe): whatever carries a UART's bytes from one end to the other: a laser or an LED and
 *     a photodiode, a fibre's transceivers, an infrared pair, a radio modem in transparent mode (HC-12, SiK, LoRa
 *     serial modules), a wire. The packets are marked out in the byte stream (pframe.h). Settings: serial,BAUD[,half]
 *     (the line's speed, 19200 to 4000000 baud, the same at both ends; half: one way at a time, as most radio
 *     modems are: the drone then answers each packet from the ground). The packets' sizes and rates follow from the
 *     speed (rlink_sizing).
 * The packet links (ESP-NOW, Wi-Fi, serial) do the modules' part in our own code, the same at both ends: plink.h.
 * Both ends need the same binding phrase (a setting of each program): it signs the packets.
 * To come: nRF24L01, Bluetooth LE. */
#ifndef RADIO_LINK_H
#define RADIO_LINK_H
#include "rc_core.h"

enum { RLINK_ELRS = 0, RLINK_ESPNOW, RLINK_WIFI, RLINK_SERIAL, RLINK_KINDS };
#define RLINK_UDP_PORT 14570
typedef struct {
  int kind;
  int rate_hz, ratio;          /* ExpressLRS: the packet rate [Hz] and the telemetry ratio (one packet in `ratio`) */
  int channel;                 /* ESP-NOW, Wi-Fi (access point): the Wi-Fi channel, 1–13 */
  int lr;                      /* ESP-NOW: long range */
  int sta;                     /* Wi-Fi: join a network (1) or make one (0) */
  int baud, half;              /* a serial line: its speed [baud], and one way at a time (1) or both at once (0) */
} rlink_cfg;
#define RLINK_BAUD_MIN 19200
#define RLINK_BAUD_HALF_MIN 38400
#define RLINK_BAUD_MAX 4000000

extern const char *const rlink_names[RLINK_KINDS];    /* as settings write it: "elrs" */
extern const char *const rlink_labels[RLINK_KINDS];   /* for people: "ExpressLRS 2.4 GHz" */
void rlink_default(rlink_cfg *L);                     /* ExpressLRS at 250 Hz, telemetry 1:4 */
/* "elrs,250,4", or "250,4" (ExpressLRS, as the setting was written before there were other links). 0, or −1 with
 * why in err (then L is unchanged). */
int rlink_parse(rlink_cfg *L, const char *s, char *err, int en);
int rlink_describe(const rlink_cfg *L, char *out, int n);    /* back as rlink_parse takes it */
/* The same from numbers: kind, then its settings in order (ExpressLRS: rate, ratio; ESP-NOW: channel, long range;
 * Wi-Fi: joins a network, channel; serial: baud, half duplex). 0, or −1 (L unchanged). */
int rlink_make(rlink_cfg *L, int kind, int a, int b);
/* The telemetry's room [bytes/s]: on a good link, and as the link is now (from the link statistics the drone's end
 * reports into rc_input; nothing while it reports nothing for a second). */
float rlink_budget(const rlink_cfg *L);
float rlink_budget_now(const rlink_cfg *L, const rc_input *in, double t);
/* A packet link (plink.h does the modules' part): 1; ExpressLRS: 0. */
static inline int rlink_packets(const rlink_cfg *L) { return L->kind == RLINK_ESPNOW || L->kind == RLINK_WIFI || L->kind == RLINK_SERIAL; }
/* A packet link's packets (plink_cfg: plink.h plink_cfg_link): the biggest down and up [bytes], how many a second
 * up, down at least and at most, and one way at a time. ESP-NOW and Wi-Fi: 250 and 250, 100, 20–100. A serial line:
 * from its speed, so the line never has more to carry than it can and the telemetry has what the channels leave
 * (see radio_link.c). */
void rlink_sizing(const rlink_cfg *L, int *mtu_down, int *mtu_up, float *up_hz, float *down_min, float *down_max, int *half);
#endif
