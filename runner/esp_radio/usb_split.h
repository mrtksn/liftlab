/* The command module's USB port carries two things: text (settings and commands, typed or from a script, one per
 * line) and, while a computer uses the ESP32 as its transmitter module (dfb_ground --tx PORT --baud 115200), CRSF
 * frames. This splits the bytes that come into the two.
 *
 * A CRSF frame starts with an address byte (0xC8, 0xEA, 0xEC, 0xEE): never in text, which is ASCII (bytes below
 * 0x80). From one, the bytes are a frame if its length is one (2…62) and its CRC is right; if not, the address byte
 * is dropped and the bytes after it are looked at again, from the start (text, or another frame). A frame whose
 * bytes stop coming for USB_SPLIT_STALL s is given up the same way, so text typed after a stray byte isn't held.
 * Other bytes at 0x80 and above (a terminal's UTF-8) are dropped; control characters other than CR and LF too.
 * A good frame drops the text line it interrupts (the bytes of a broken frame, most likely: a computer that sends
 * frames sends no text).
 *
 * Portable C (no ESP-IDF): tested on a PC by test_esp_radio.c. */
#ifndef USB_SPLIT_H
#define USB_SPLIT_H
#include <stdint.h>
#include "crsf.h"

#define USB_SPLIT_LINE 200       /* a text line's room (longer: the line is cut there, the rest starts the next one) */
#define USB_SPLIT_STALL 0.1      /* [s] a frame whose bytes stopped this long is given up */

typedef struct {
  uint8_t f[CRSF_MAX_FRAME]; int fn;     /* the frame coming in */
  char line[USB_SPLIT_LINE]; int ln;     /* the text line coming in */
  double t_byte;                         /* when the last byte came */
  double t_frame;                        /* when the last good frame came (−1: never) */
  uint32_t frames, bad;                  /* good frames; things that started like one and weren't */
} usb_split;

typedef struct {
  void (*frame)(void *ctx, const uint8_t *f, int n);   /* a whole frame with a good CRC */
  void (*line)(void *ctx, char *line);                  /* a line of text, without its end (never empty) */
  void *ctx;
} usb_split_out;

void usb_split_init(usb_split *S);
/* Bytes that came at t (n may be 0: then only the stall check runs; call it each step). */
void usb_split_feed(usb_split *S, const uint8_t *b, int n, double t, const usb_split_out *O);
/* A computer sends frames: one came within the last second. */
static inline int usb_split_frames_now(const usb_split *S, double t) { return S->frames && t - S->t_frame < 1.0; }
#endif
