/* The flight controller firmware's portable part (fc_core.c, with rn_host.c and rn.c) built to WebAssembly, so
 * the simulator can fly it: "firmware in the loop" (js/fc-fil.js). The page puts the flight program (the image
 * the simulator compiled) and the airframe (js/fc-export.js) in the buffers below, then calls setup(), and each
 * control step writes the IMU sample, calls tick() and reads the throttles and servo angles back.
 * Built by fc/build_wasm.sh. */
#include "fc_core.h"

#define ARENA_CAP 65536
#define CODE_CAP 32768
#define POOL_CAP 8192
#define IMG_CAP (256u << 10)
#define BLOB_CAP 8192
static float arenas_[3][ARENA_CAP], pools_[3][POOL_CAP];
static int32_t codes_[3][CODE_CAP];
static uint8_t img[IMG_CAP], blob[BLOB_CAP];
static rn_host H;
static fc_state F;
/* io: in gyro[3] acc[3] have_gyro dt vbatt baro_alt have_baro | out motor[12] servo[8] | state q[4] w[3] trap rho att_ok yaw_sp az_f iAz vz_i */
static float io[11 + FC_MAX_MOTORS + FC_MAX_JOINTS + 16];
static float cmd[7];                 /* arm roll pitch yaw throttle test_motor test_throttle */

#define EXPORT(n) __attribute__((export_name(n)))
EXPORT("img_ptr") uint8_t *img_ptr(void) { return img; }
EXPORT("img_cap") uint32_t img_cap(void) { return IMG_CAP; }
EXPORT("blob_ptr") uint8_t *blob_ptr(void) { return blob; }
EXPORT("blob_cap") uint32_t blob_cap(void) { return BLOB_CAP; }
EXPORT("io_ptr") float *io_ptr(void) { return io; }
EXPORT("cmd_ptr") float *cmd_ptr(void) { return cmd; }
EXPORT("why_ptr") char *why_ptr(void) { return F.why; }
EXPORT("state") int state(void) { return F.state; }

static void set_why(const char *s) { int i = 0; for (; s[i] && i < 63; i++) F.why[i] = s[i]; F.why[i] = 0; }

/* Set up the host with the given program as the built-in one, then the flight code and the airframe. */
EXPORT("setup") int setup(uint32_t img_len, uint32_t blob_len) {
  static fc_state zero; F = zero;
  float *arenas[3] = { arenas_[0], arenas_[1], arenas_[2] }, *pools[3] = { pools_[0], pools_[1], pools_[2] };
  int32_t *codes[3] = { codes_[0], codes_[1], codes_[2] };
  int e = rn_host_init(&H, img, img_len, arenas, ARENA_CAP, codes, CODE_CAP, pools, POOL_CAP);
  if (e) { set_why("the flight program didn't load"); return 100 + e; }
  if (fc_init(&F, &H)) return 1;
  if (fc_airframe_load(&F, blob, blob_len)) return 2;
  return 0;
}
/* A new program while flying: through the host's loading steps, as on the drone. */
EXPORT("stage") int stage(uint32_t img_len) { return rn_host_stage(&H, img, img_len); }
EXPORT("host_event") int host_event(void) { int e = H.last_event; H.last_event = 0; return e; }
EXPORT("host_phase") int host_phase(void) { return H.phase + 10 * H.act; }

EXPORT("command") void command(void) {
  fc_cmd c = { (int)cmd[0], cmd[1], cmd[2], cmd[3], cmd[4], (int)cmd[5], cmd[6] };
  fc_command(&F, &c);
}
EXPORT("tick") void tick(void) {
  fc_imu m = { { io[0], io[1], io[2] }, { io[3], io[4], io[5] }, io[6] > 0.5f, io[9], io[10] > 0.5f };
  float dt = io[7];
  fc_out o;
  rn_host_tick(&H, dt);
  fc_step(&F, &m, dt, io[8], &o);
  float *p = io + 11;
  for (int i = 0; i < FC_MAX_MOTORS; i++) *p++ = o.motor[i];
  for (int j = 0; j < FC_MAX_JOINTS; j++) *p++ = o.servo[j];
  for (int k = 0; k < 4; k++) *p++ = F.q[k];
  for (int k = 0; k < 3; k++) *p++ = F.w[k];
  *p++ = (float)F.trap; *p++ = F.rho; *p++ = (float)F.att_ok; *p++ = F.yaw_sp; *p++ = F.az_f; *p++ = F.iAz; *p++ = F.have_alt ? F.vz_e : F.vz_i; *p++ = F.alt_e; *p++ = (float)F.have_alt;
}

/* The few C library functions the code uses (there is no C library in this build). */
typedef __SIZE_TYPE__ size_t;
void *memcpy(void *d, const void *s, size_t n) { uint8_t *a = d; const uint8_t *b = s; while (n--) *a++ = *b++; return d; }
void *memmove(void *d, const void *s, size_t n) {
  uint8_t *a = d; const uint8_t *b = s;
  if (a == b || !n) return d;
  if (a < b) while (n--) *a++ = *b++; else { a += n; b += n; while (n--) *--a = *--b; }
  return d;
}
void *memset(void *d, int c, size_t n) { uint8_t *a = d; while (n--) *a++ = (uint8_t)c; return d; }
size_t strlen(const char *s) { size_t n = 0; while (s[n]) n++; return n; }
int strncmp(const char *a, const char *b, size_t n) { for (; n; n--, a++, b++) { if (*a != *b) return (unsigned char)*a - (unsigned char)*b; if (!*a) return 0; } return 0; }
