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
/* The flight firmware's (fc/esp32) frames:
 *   Pi → drone:  RN_LINK_CMD       the pilot's command: 7 little-endian floats (arm, roll, pitch, yaw, throttle,
 *                                  test motor or −1, test throttle); send it steadily (50 Hz): the drone goes to its
 *                                  failsafe when they stop for 0.5 s
 *                RN_LINK_AIRFRAME  an airframe file from the simulator (.dfa); kept in flash (disarmed only)
 *                RN_LINK_SETTING   one line of text: "key=value", "show", "save", "reboot", "gyro"
 *                                  Or 12 floats: those 7, then a guided command from the navigation (fc_cmd.guided):
 *                                  guided (1), acceleration x, y, z [m/s²], heading [rad] (fc/nav_core.h)
 *   drone → Pi:  RN_LINK_TELEM     36 little-endian floats, 20 times a second (see fc/esp32/main/flight.c)
 *                RN_LINK_NAV       what the navigation flies on, 100 times a second while guided commands come:
 *                                  16 floats: t, state, attitude q (4), body rates (3), specific force, body (3),
 *                                  height (barometer, or 0), have height (0/1), attitude settled (0/1) */
enum { RN_LINK_CMD = 3, RN_LINK_AIRFRAME = 4, RN_LINK_SETTING = 5, RN_LINK_TELEM = 0x83, RN_LINK_NAV = 0x84 };

typedef struct {
  uint8_t *buf; uint32_t cap;       /* where payloads are collected */
  int state; uint8_t hdr[5]; uint32_t got, len; uint8_t type;
  uint32_t (*limit)(uint8_t type);  /* optional: the longest payload each type may have (0: not accepted) */
} rn_link;

void rn_link_init(rn_link *L, uint8_t *buf, uint32_t cap);
/* Forget a frame in progress (call when its bytes stopped coming: the next frame then isn't taken as its payload). */
void rn_link_reset(rn_link *L);
/* Feed one received byte. Returns the frame's type when a whole frame with a good CRC has arrived (the payload is
 * in L->buf, L->len bytes), −1 when a frame was dropped (bad CRC, too long for the buffer, or over its type's limit), 0 otherwise. */
int rn_link_feed(rn_link *L, uint8_t byte);
/* Build a frame into out. Returns its length, or 0 if it doesn't fit. */
uint32_t rn_link_frame(uint8_t *out, uint32_t cap, uint8_t type, const uint8_t *payload, uint32_t len);

#endif
