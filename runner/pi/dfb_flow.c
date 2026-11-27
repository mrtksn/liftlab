/*
 * dfb_flow: optical flow from the drone's downward camera, for the navigation (dfb_pi takes it as its optical-flow
 * reference, beside a rangefinder or the barometer for the height).
 *
 * The camera is read raw (V4L2, the sensor straight into memory: no ISP, no libcamera), which costs a Pi Zero almost
 * nothing; only a small patch from the middle of each frame is used (the floor under the drone; a fisheye's edges, its
 * distortion and the props stay out of it). The patch is binned to grey, fed to the flow core (fc/flow_core.c: one
 * shift for the whole patch, to a fraction of a pixel), and turned into angular rates with the lens's radians per
 * pixel. Its own auto-exposure keeps the patch bright enough with a short shutter (blur would smear the texture).
 *
 * Each frame (with --send) it sends one text datagram:
 *   flow T DT FX FY Q DX DY EXP GAIN MEAN TEX RESID PROC SEQ RADPX MIRROR
 * T: the middle of the patch's exposure [s, CLOCK_MONOTONIC: the same clock dfb_pi keeps]; DT: since the frame before
 * [s]; FX FY: the floor's angular rate across the picture, camera axes (x right, y up in the picture; the camera looks
 * along its −Z) [rad/s]: the navigation's flow input; Q: quality 0–1; DX DY: the shift [raw px]; EXP [ms], GAIN [×],
 * MEAN [0–255]: the exposure; TEX, RESID: the flow core's texture and residual; PROC: time spent on this frame [ms];
 * SEQ: the camera's frame number (a jump is a dropped frame); RADPX, MIRROR: the lens settings it used.
 *
 *   dfb_flow [--dev /dev/video0] [--subdev /dev/v4l-subdev0] [--size 640x480]
 *            [--fps 30] [--crop 256] [--center X,Y] [--patch 64] [--levels 3] [--iters 6]
 *            [--rad-px R] [--mirror] [--exposure-max MS] [--gain-max X] [--bright B]
 *            [--send HOST:PORT] [--print [EVERY]] [--bench SECONDS] [--save DIR [FRAMES]] [--frames PORT [--frames-every N]]
 *   --size      the sensor mode [raw px] (the ov5647's 640x480 is binned from the whole sensor at up to ~59 fps)
 *   --fps       frames a second (the sensor's own frame time: fewer cost less, and leave room for a longer exposure)
 *   --crop      the patch's side on the sensor [raw px]; --patch its side after binning (crop/patch an even number)
 *   --center    the patch's middle [raw px] (default the frame's middle; a fisheye's optical centre may be off it)
 *   --rad-px    the lens: radians per raw pixel near the middle (dfb_pi's flowcal measures it against the gyro)
 *   --mirror    the picture comes mirrored (left–right): flips FX
 *   --bench     measure for a while, then print the frame rate and the time per frame
 *   --save      write the next FRAMES patches as PGM pictures into DIR (to look at what it sees)
 *   --frames PORT   every Nth whole frame as a small colour picture, to a reader on 127.0.0.1:PORT (frame_out.h: Room
 *                   Survey records the floor through it, survey/pi_drone_camera.py); --frames-every N (10)
 *
 * Build: sh runner/pi/build.sh (with dfb_pi), or zig through tools/build_pi_cross.sh.
 */
#define _DEFAULT_SOURCE
#include "flow_core.h"
#include "frame_out.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>
#include <math.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static volatile sig_atomic_t stop;
static void on_signal(int s) { (void)s; stop = 1; }
static double now(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
static double cpu_now(void) { struct timespec ts; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }   /* (the stages' costs: CPU time, not the time other programs had the CPU) */

static int xioctl(int fd, unsigned long req, void *arg) { int r; do r = ioctl(fd, req, arg); while (r < 0 && errno == EINTR); return r; }

static int get_ctrl(int fd, unsigned id, long long *v) {
  struct v4l2_ext_control c = { .id = id }; struct v4l2_ext_controls cs = { .which = V4L2_CTRL_WHICH_CUR_VAL, .count = 1, .controls = &c };
  if (xioctl(fd, VIDIOC_G_EXT_CTRLS, &cs) < 0) return -1;
  *v = (id == V4L2_CID_PIXEL_RATE) ? c.value64 : c.value; return 0;
}
static int set_ctrl(int fd, unsigned id, int v) {
  struct v4l2_ext_control c = { .id = id, .value = v }; struct v4l2_ext_controls cs = { .which = V4L2_CTRL_WHICH_CUR_VAL, .count = 1, .controls = &c };
  return xioctl(fd, VIDIOC_S_EXT_CTRLS, &cs);
}
static int ctrl_range(int fd, unsigned id, int *lo, int *hi) {
  struct v4l2_query_ext_ctrl q = { .id = id };
  if (xioctl(fd, VIDIOC_QUERY_EXT_CTRL, &q) < 0) return -1;
  *lo = (int)q.minimum; *hi = (int)q.maximum; return 0;
}

/* the sensor's Bayer bus code → the matching V4L2 pixel format (10-bit packed, or 8-bit) */
static unsigned pixfmt_for(unsigned code, int *raw10) {
  switch (code) {
  case MEDIA_BUS_FMT_SBGGR10_1X10: *raw10 = 1; return V4L2_PIX_FMT_SBGGR10P;
  case MEDIA_BUS_FMT_SGBRG10_1X10: *raw10 = 1; return V4L2_PIX_FMT_SGBRG10P;
  case MEDIA_BUS_FMT_SGRBG10_1X10: *raw10 = 1; return V4L2_PIX_FMT_SGRBG10P;
  case MEDIA_BUS_FMT_SRGGB10_1X10: *raw10 = 1; return V4L2_PIX_FMT_SRGGB10P;
  case MEDIA_BUS_FMT_SBGGR8_1X8: *raw10 = 0; return V4L2_PIX_FMT_SBGGR8;
  case MEDIA_BUS_FMT_SGBRG8_1X8: *raw10 = 0; return V4L2_PIX_FMT_SGBRG8;
  case MEDIA_BUS_FMT_SGRBG8_1X8: *raw10 = 0; return V4L2_PIX_FMT_SGRBG8;
  case MEDIA_BUS_FMT_SRGGB8_1X8: *raw10 = 0; return V4L2_PIX_FMT_SRGGB8;
  default: return 0;
  }
}

static void usage(void) {
  fprintf(stderr, "usage: dfb_flow [--dev /dev/video0] [--subdev /dev/v4l-subdev0] [--size 640x480] [--fps 30] [--crop 256] [--center X,Y]\n"
                  "                [--patch 64] [--levels 3] [--iters 6] [--rad-px R] [--mirror] [--exposure-max MS] [--gain-max X]\n"
                  "                [--bright B] [--send HOST:PORT] [--print [EVERY]] [--bench SECONDS] [--save DIR [FRAMES]] [--frames PORT [--frames-every N]]\n");
}

#define NBUF 4
static flow_state F;

int main(int argc, char **argv) {
  const char *dev = "/dev/video0", *subdev = "/dev/v4l-subdev0", *send_to = NULL, *save_dir = NULL;
  int frames_port = 0, frames_every = 10;
  int W = 640, H = 480, crop = 256, cx = -1, cy = -1, patch = 64, levels = 3, iters = 6, mirror = 0, every = 0, save_n = 0;
  double rad_px = 0.0040, exp_max_ms = 8, gain_max = 16, bright = 90, bench = 0, fps = 30;
  for (int i = 1; i < argc; i++) {
    const char *a = argv[i], *v = i + 1 < argc ? argv[i + 1] : NULL;
    #define NEXT() (v ? (i++, v) : (usage(), exit(2), ""))
    if (!strcmp(a, "--dev")) dev = NEXT();
    else if (!strcmp(a, "--subdev")) subdev = NEXT();
    else if (!strcmp(a, "--size")) { if (sscanf(NEXT(), "%dx%d", &W, &H) != 2) { usage(); return 2; } }
    else if (!strcmp(a, "--fps")) fps = atof(NEXT());
    else if (!strcmp(a, "--crop")) crop = atoi(NEXT());
    else if (!strcmp(a, "--center")) { if (sscanf(NEXT(), "%d,%d", &cx, &cy) != 2) { usage(); return 2; } }
    else if (!strcmp(a, "--patch")) patch = atoi(NEXT());
    else if (!strcmp(a, "--levels")) levels = atoi(NEXT());
    else if (!strcmp(a, "--iters")) iters = atoi(NEXT());
    else if (!strcmp(a, "--rad-px")) rad_px = atof(NEXT());
    else if (!strcmp(a, "--mirror")) mirror = 1;
    else if (!strcmp(a, "--exposure-max")) exp_max_ms = atof(NEXT());
    else if (!strcmp(a, "--gain-max")) gain_max = atof(NEXT());
    else if (!strcmp(a, "--bright")) bright = atof(NEXT());
    else if (!strcmp(a, "--send")) send_to = NEXT();
    else if (!strcmp(a, "--frames")) frames_port = atoi(NEXT());
    else if (!strcmp(a, "--frames-every")) frames_every = atoi(NEXT());
    else if (!strcmp(a, "--print")) { every = 1; if (v && v[0] >= '0' && v[0] <= '9') every = atoi(NEXT()); }
    else if (!strcmp(a, "--bench")) bench = atof(NEXT());
    else if (!strcmp(a, "--save")) { save_dir = NEXT(); save_n = 30; if (i + 1 < argc && argv[i + 1][0] >= '0' && argv[i + 1][0] <= '9') save_n = atoi(argv[++i]); }
    else { usage(); return 2; }
  }
  if (cx < 0) { cx = W / 2; cy = H / 2; }
  int bin = patch > 0 ? crop / patch : 0;
  if (bin < 2 || bin & 1 || bin * patch != crop || crop & 3) { fprintf(stderr, "dfb_flow: --crop %d / --patch %d must be an even whole number (2, 4, ...)\n", crop, patch); return 2; }
  int x0 = (cx - crop / 2) & ~3, y0 = (cy - crop / 2) & ~1;   /* (x on a whole 5-byte group of the packed format) */
  if (x0 < 0 || y0 < 0 || x0 + crop > W || y0 + crop > H) { fprintf(stderr, "dfb_flow: the patch (%d px around %d,%d) leaves the %dx%d frame\n", crop, cx, cy, W, H); return 2; }
  if (flow_init(&F, patch, levels, iters)) { fprintf(stderr, "dfb_flow: --patch %d with --levels %d doesn't fit (patch up to %d, its coarsest level 8 px or more)\n", patch, levels, FLOW_MAX_N); return 2; }

  /* the sensor: its mode, and the controls the exposure needs */
  int sd = open(subdev, O_RDWR);
  if (sd < 0) { fprintf(stderr, "dfb_flow: %s: %s (the camera's sensor; is the camera enabled?)\n", subdev, strerror(errno)); return 1; }
  struct v4l2_subdev_format sf = { .which = V4L2_SUBDEV_FORMAT_ACTIVE, .pad = 0 };
  if (xioctl(sd, VIDIOC_SUBDEV_G_FMT, &sf) < 0) { fprintf(stderr, "dfb_flow: %s: no format: %s\n", subdev, strerror(errno)); return 1; }
  sf.format.width = (unsigned)W; sf.format.height = (unsigned)H;
  if (xioctl(sd, VIDIOC_SUBDEV_S_FMT, &sf) < 0 || (int)sf.format.width != W || (int)sf.format.height != H) {
    fprintf(stderr, "dfb_flow: the sensor has no %dx%d mode (it offered %ux%u)\n", W, H, sf.format.width, sf.format.height); return 1; }
  int raw10 = 0; unsigned pf = pixfmt_for(sf.format.code, &raw10);
  if (!pf) { fprintf(stderr, "dfb_flow: the sensor sends bus code 0x%x: not a Bayer format this program reads\n", sf.format.code); return 1; }
  int vb_lo, vb_hi, ex_lo, ex_hi, g_lo, g_hi; long long hblank = 0, vblank = 0, prate = 0;
  get_ctrl(sd, V4L2_CID_HBLANK, &hblank); get_ctrl(sd, V4L2_CID_PIXEL_RATE, &prate);
  if (ctrl_range(sd, V4L2_CID_VBLANK, &vb_lo, &vb_hi) == 0) {   /* the frame rate: the blank lines after each frame */
    int vb = prate > 0 && fps > 0 ? (int)((double)prate / ((double)(W + hblank) * fps)) - H : vb_lo;
    set_ctrl(sd, V4L2_CID_VBLANK, vb < vb_lo ? vb_lo : vb > vb_hi ? vb_hi : vb);
  }
  if (ctrl_range(sd, V4L2_CID_EXPOSURE, &ex_lo, &ex_hi) || ctrl_range(sd, V4L2_CID_ANALOGUE_GAIN, &g_lo, &g_hi)) {
    fprintf(stderr, "dfb_flow: the sensor has no exposure or gain control\n"); return 1; }
  get_ctrl(sd, V4L2_CID_VBLANK, &vblank);
  double line_s = prate > 0 ? (double)(W + hblank) / (double)prate : 1.0 / (60.0 * (H + 24));
  int frame_lines = H + (int)vblank;
  double fps_sensor = 1.0 / (line_s * frame_lines);
  int gain_unit = 16;                                        /* analogue gain code for ×1 (the ov5647 and most Omnivision sensors) */
  int exp_cap = (int)(exp_max_ms * 1e-3 / line_s); if (exp_cap > frame_lines - 4) exp_cap = frame_lines - 4; if (exp_cap > ex_hi) exp_cap = ex_hi; if (exp_cap < ex_lo) exp_cap = ex_lo;
  int gain_cap = (int)(gain_max * gain_unit); if (gain_cap > g_hi) gain_cap = g_hi; if (gain_cap < g_lo) gain_cap = g_lo;
  int exp_l = exp_cap, gain = gain_unit * 4 < gain_cap ? gain_unit * 4 : gain_cap;
  set_ctrl(sd, V4L2_CID_EXPOSURE, exp_l); set_ctrl(sd, V4L2_CID_ANALOGUE_GAIN, gain);

  /* the frames: the sensor's raw Bayer, straight into memory */
  int fd = open(dev, O_RDWR | O_NONBLOCK);
  if (fd < 0) { fprintf(stderr, "dfb_flow: %s: %s\n", dev, strerror(errno)); return 1; }
  struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
  fmt.fmt.pix.width = (unsigned)W; fmt.fmt.pix.height = (unsigned)H; fmt.fmt.pix.pixelformat = pf; fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0 || fmt.fmt.pix.pixelformat != pf) { fprintf(stderr, "dfb_flow: %s won't take the sensor's format: %s\n", dev, strerror(errno)); return 1; }
  int bpl = (int)fmt.fmt.pix.bytesperline;
  struct v4l2_requestbuffers rb = { .count = NBUF, .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
  if (xioctl(fd, VIDIOC_REQBUFS, &rb) < 0 || rb.count < 2) { fprintf(stderr, "dfb_flow: no frame buffers: %s (is another program using the camera?)\n", strerror(errno)); return 1; }
  void *mem[NBUF]; size_t mlen[NBUF];
  for (unsigned i = 0; i < rb.count; i++) {
    struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP, .index = i };
    if (xioctl(fd, VIDIOC_QUERYBUF, &b) < 0) { perror("dfb_flow: QUERYBUF"); return 1; }
    mlen[i] = b.length; mem[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
    if (mem[i] == MAP_FAILED) { perror("dfb_flow: mmap"); return 1; }
    if (xioctl(fd, VIDIOC_QBUF, &b) < 0) { perror("dfb_flow: QBUF"); return 1; }
  }
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) { fprintf(stderr, "dfb_flow: the camera won't start: %s\n", strerror(errno)); return 1; }

  int us = -1; struct sockaddr_in to; memset(&to, 0, sizeof to);
  if (send_to) {
    char host[64]; int port = 0;
    if (sscanf(send_to, "%63[^:]:%d", host, &port) != 2 || inet_pton(AF_INET, host, &to.sin_addr) != 1) { fprintf(stderr, "dfb_flow: --send HOST:PORT (an IPv4 address)\n"); return 2; }
    to.sin_family = AF_INET; to.sin_port = htons((uint16_t)port); us = socket(AF_INET, SOCK_DGRAM, 0);
  }
  signal(SIGINT, on_signal); signal(SIGTERM, on_signal);
  fprintf(stderr, "dfb_flow: %s %dx%d %s at %.1f fps; patch %d px around (%d,%d) binned %d → %d px (%.1f° wide at %.4f rad/px); exposure up to %.1f ms, gain up to ×%.0f%s\n",
          dev, W, H, raw10 ? "raw10" : "raw8", fps_sensor, crop, cx, cy, bin, patch, crop * rad_px * 57.2958, rad_px, exp_cap * line_s * 1e3, (double)gain_cap / gain_unit, send_to ? "" : " (not sending: no --send)");

  static uint8_t img[FLOW_MAX_N * FLOW_MAX_N];
  /* the patch's rows are copied out of the frame first, whole: the frame buffers are uncached (the camera writes
   * them), where reading byte by byte is many times slower than one sequential copy */
  int cbpl = raw10 ? crop / 4 * 5 : crop, xoff = raw10 ? x0 / 4 * 5 : x0;
  uint8_t *rows = malloc((size_t)cbpl * (size_t)crop);
  if (!rows) { fprintf(stderr, "dfb_flow: out of memory\n"); return 1; }
  /* the pictures for a reader (--frames): the whole frame copied out, then each 2x2 block one RGB pixel */
  frame_out FO; memset(&FO, 0, sizeof FO); FO.fd = FO.listen_fd = -1; uint8_t *whole = NULL, *pic = NULL; int order = 0;
  if (frames_port > 0) {
    if (frames_every < 1) frames_every = 1;
    order = pf == V4L2_PIX_FMT_SBGGR10P || pf == V4L2_PIX_FMT_SBGGR8 ? FO_BGGR : pf == V4L2_PIX_FMT_SGBRG10P || pf == V4L2_PIX_FMT_SGBRG8 ? FO_GBRG
          : pf == V4L2_PIX_FMT_SGRBG10P || pf == V4L2_PIX_FMT_SGRBG8 ? FO_GRBG : FO_RGGB;
    whole = malloc((size_t)bpl * (size_t)H); pic = malloc((size_t)(W / 2) * (size_t)(H / 2) * 3);
    if (!whole || !pic || frame_out_open(&FO, frames_port)) { fprintf(stderr, "dfb_flow: --frames %d: %s\n", frames_port, whole && pic ? strerror(errno) : "out of memory"); return 1; }
    fprintf(stderr, "dfb_flow: pictures (%dx%d, every %d frames) to a reader on 127.0.0.1:%d\n", W / 2, H / 2, frames_every, frames_port);
  }
  double copy_sum = 0, bin_sum = 0, flow_sum = 0;
  double t_prev = 0, t0 = now(), proc_sum = 0, proc_max = 0, bench_end = bench > 0 ? t0 + bench : 0; long frames = 0, drops = 0, good = 0;
  unsigned seq_prev = 0; int have_seq = 0, ae_wait = 0, saved = 0;
  struct rusage ru0; getrusage(RUSAGE_SELF, &ru0);
  while (!stop) {
    struct pollfd p = { fd, POLLIN, 0 };
    int pr = poll(&p, 1, 1000);
    if (pr == 0) { fprintf(stderr, "dfb_flow: no frame for a second\n"); continue; }
    if (pr < 0) { if (errno == EINTR) continue; perror("dfb_flow: poll"); break; }
    struct v4l2_buffer b = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE, .memory = V4L2_MEMORY_MMAP };
    if (xioctl(fd, VIDIOC_DQBUF, &b) < 0) { if (errno == EAGAIN) continue; perror("dfb_flow: DQBUF"); break; }
    double tp = now(), c0 = cpu_now();
    /* the patch's exposure middle: the frame starts at its timestamp, the patch's middle row is read out (its exposure
     * ending) row × line time later */
    double ts = b.timestamp.tv_sec + b.timestamp.tv_usec * 1e-6;
    double t_mid = ts + cy * line_s - exp_l * line_s * 0.5;
    if (have_seq && b.sequence != seq_prev + 1) { drops += (long)(b.sequence - seq_prev - 1); if (b.sequence - seq_prev > 3) flow_reset(&F); }
    seq_prev = b.sequence; have_seq = 1;
    for (int j = 0; j < crop; j++) memcpy(rows + (size_t)j * cbpl, (const uint8_t *)mem[b.index] + (size_t)(y0 + j) * bpl + xoff, (size_t)cbpl);
    unsigned seq = b.sequence; int exp_used = exp_l, gain_used = gain;
    int picture = whole && seq % (unsigned)frames_every == 0 && frame_out_wanted(&FO);   /* (only while a reader is there) */
    if (picture) memcpy(whole, mem[b.index], (size_t)bpl * (size_t)H);
    uint64_t t_ns = (uint64_t)b.timestamp.tv_sec * 1000000000ull + (uint64_t)b.timestamp.tv_usec * 1000ull;
    xioctl(fd, VIDIOC_QBUF, &b);
    if (picture) { frame_out_rgb(pic, whole, bpl, W, H, raw10, order); frame_out_balance(&FO, pic, W / 2, H / 2);
      frame_out_send(&FO, pic, W / 2, H / 2, t_ns, seq, (uint32_t)(exp_used * line_s * 1e6), (uint32_t)(100.0 * gain_used / gain_unit)); }
    double tc = cpu_now();
    int mean = flow_bin_bayer(rows, cbpl, raw10, 0, 0, patch, bin, img);
    double tb = cpu_now();
    flow_result r; int have = flow_frame(&F, img, patch, &r);
    double tf = cpu_now(); copy_sum += tc - c0; bin_sum += tb - tc; flow_sum += tf - tb;
    double dt = t_prev > 0 ? t_mid - t_prev : 0; t_prev = t_mid;
    /* auto-exposure: toward the brightness wanted, the shutter first (up to its cap), then the gain; a change shows two
     * frames later, so it waits for it */
    if (ae_wait > 0) ae_wait--;
    else if (mean < bright * 0.85 || mean > bright * 1.15) {
      double ratio = bright / (mean < 2 ? 2 : mean); if (ratio > 3) ratio = 3; if (ratio < 0.33) ratio = 0.33;
      double total = (double)exp_l * gain * sqrt(ratio);
      int e2 = (int)(total / gain_unit); if (e2 > exp_cap) e2 = exp_cap; if (e2 < ex_lo) e2 = ex_lo;
      int g2 = (int)(total / e2 + 0.5); if (g2 > gain_cap) g2 = gain_cap; if (g2 < g_lo) g2 = g_lo; if (g2 < gain_unit && e2 < exp_cap) g2 = gain_unit;
      if (e2 != exp_l) { set_ctrl(sd, V4L2_CID_EXPOSURE, e2); exp_l = e2; ae_wait = 3; }
      if (g2 != gain) { set_ctrl(sd, V4L2_CID_ANALOGUE_GAIN, g2); gain = g2; ae_wait = 3; }
    }
    if (save_dir && saved < save_n) {
      char path[512]; snprintf(path, sizeof path, "%s/flow%04d.pgm", save_dir, saved++);
      FILE *f = fopen(path, "wb"); if (f) { fprintf(f, "P5\n%d %d\n255\n", patch, patch); fwrite(img, 1, (size_t)(patch * patch), f); fclose(f); }
      if (saved == save_n) fprintf(stderr, "dfb_flow: %d pictures in %s\n", saved, save_dir);
    }
    double proc = (now() - tp) * 1e3; proc_sum += proc; if (proc > proc_max) proc_max = proc; frames++;
    if (have && dt > 0) {
      double k = bin * rad_px / dt;
      double fx = r.dx * k * (mirror ? -1 : 1), fy = -r.dy * k;
      if (r.q > 0) good++;
      char line[256];
      int n = snprintf(line, sizeof line, "flow %.4f %.4f %.4f %.4f %.2f %.3f %.3f %.2f %.1f %d %.3f %.3f %.2f %u %.6f %d",
                       t_mid, dt, fx, fy, r.q, r.dx * bin, r.dy * bin, exp_used * line_s * 1e3, (double)gain_used / gain_unit, mean, r.texture, r.resid, proc, seq, rad_px, mirror);
      if (us >= 0) sendto(us, line, (size_t)n, 0, (struct sockaddr *)&to, sizeof to);
      if (every && frames % every == 0) { puts(line); fflush(stdout); }
    }
    if (bench_end && now() > bench_end) break;
  }
  double el = now() - t0; struct rusage ru; getrusage(RUSAGE_SELF, &ru);
  double cpu = (ru.ru_utime.tv_sec - ru0.ru_utime.tv_sec) + (ru.ru_stime.tv_sec - ru0.ru_stime.tv_sec) + 1e-6 * ((ru.ru_utime.tv_usec - ru0.ru_utime.tv_usec) + (ru.ru_stime.tv_usec - ru0.ru_stime.tv_usec));
  double fn = frames ? (double)frames : 1;
  fprintf(stderr, "dfb_flow: %ld frames in %.1f s (%.1f fps, %ld dropped), %.0f%% with a reading; %.2f ms a frame (most %.2f: CPU: copy %.2f, bin %.2f, flow %.2f), %.0f%% of the CPU\n",
          frames, el, frames / el, drops, 100.0 * good / fn, proc_sum / fn, proc_max, copy_sum * 1e3 / fn, bin_sum * 1e3 / fn, flow_sum * 1e3 / fn, 100 * cpu / el);
  xioctl(fd, VIDIOC_STREAMOFF, &type);
  return 0;
}
