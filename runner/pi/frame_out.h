/*
 * dfb_flow's pictures for whoever wants them (Room Survey's recorder, survey/pi_drone_camera.py): every Nth frame of
 * the downward camera as a small colour picture, on a TCP port on this computer. Nothing is made or sent while no one
 * is connected; a reader that falls behind is let go (the flow comes first).
 *
 * The picture: each 2×2 block of the sensor's raw Bayer pixels as one RGB pixel (red, the two greens averaged, blue;
 * the top 8 bits of each): a 640×480 frame as 320×240. Then balanced, as a camera's own processing would: each colour
 * by the grey world (the picture's mean grey, eased over frames), brought up toward a middle grey, and a gamma (2.2).
 * One table a colour, so it costs a Pi Zero a few milliseconds.
 *
 * Each frame, little-endian:
 *   "DFF1"  u32 width  u32 height  u64 time [ns, CLOCK_MONOTONIC: the frame's start, the camera driver's stamp]
 *   u32 frame number  u32 exposure [µs]  u32 gain [×100]  u32 bytes (width × height × 3)  then the RGB pixels
 */
#ifndef FRAME_OUT_H
#define FRAME_OUT_H
#include <stdint.h>

enum { FO_BGGR, FO_GBRG, FO_GRBG, FO_RGGB };   /* the colour of the 2×2 block's pixels, top left first, row by row */

typedef struct { int listen_fd, fd; long sent, dropped; float gain[3]; int have_gain; uint8_t lut[3][256]; } frame_out;

/* Listen on 127.0.0.1:port. 0, or −1 (errno says why). */
int frame_out_open(frame_out *O, int port);
/* Take a waiting reader if there's none: 1 if a reader is connected. */
int frame_out_wanted(frame_out *O);
/* A W×H raw frame (rows bpl bytes apart, raw10 packed or 8-bit) to W/2 × H/2 RGB, as it is (linear). */
void frame_out_rgb(uint8_t *rgb, const uint8_t *raw, int bpl, int W, int H, int raw10, int order);
/* Balance a picture in place (grey world eased over the frames, a lift toward middle grey, the gamma). */
void frame_out_balance(frame_out *O, uint8_t *rgb, int w, int h);
/* Send one picture; a reader that can't take it in time is let go. 0 sent, −1 not. */
int frame_out_send(frame_out *O, const uint8_t *rgb, int w, int h, uint64_t t_ns, uint32_t seq, uint32_t exp_us, uint32_t gain_x100);
void frame_out_close(frame_out *O);

#endif
