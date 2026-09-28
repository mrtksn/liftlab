/*
 * Example flight controller firmware around the step runner (ESP-IDF). Not a complete flight controller: it
 * shows where the runner sits.
 *
 *  - Core 1: the flight task, every 1 ms. It reads the sensors, calls the flight formulas through rn_host_call()
 *    in the order the simulator's control step does, and drives the motors and servos. The runner answers from
 *    whichever program is flying: the built-in one, or one the Raspberry Pi sent.
 *  - Core 0: the link task. It receives programs from the Pi over UART1 (rn_link.h framing), prepares them
 *    (loader checks, self-tests, signatures: a few milliseconds, away from the flight loop) and reports what
 *    happened. The flight task's rn_host_tick() starts the background run, the blend and the swap.
 *
 * What's still yours to write is the glue in flight_step(): the sensor drivers, the motor outputs, and the order
 * and wiring of the calls. The simulator's js/sensors.js, js/sim.js and js/learn.js are the reference for that
 * wiring; the formulas themselves come from the program.
 *
 * Memory: each program slot needs its working memory (the arena: 65 KB for the default program with room for 24
 * actuator inputs) and its steps (29 KB). The built-in program's steps run from flash. Three slots take about
 * 250 KB: comfortable on an ESP32-S3, tight on a plain ESP32 without PSRAM (there, set RN_SLOTS_IN_PSRAM, or build
 * the program for fewer inputs: RN_IN in js/rn-sigs.js).
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "rn_host.h"
#include "rn_link.h"

extern const uint8_t *const rn_builtin_img;
extern const uint32_t rn_builtin_len;

#define ARENA_CAP 18000                 /* floats per slot */
#define CODE_CAP 8192                   /* code words per loaded slot */
#define POOL_CAP 1024                   /* floats of instance memory per slot */
#define IMG_CAP (256 * 1024)            /* largest program the Pi may send */
#define LINK_UART UART_NUM_1
#define LINK_TX 17
#define LINK_RX 18
#define LINK_BAUD 921600
#define N_SERVOS 4

#ifdef RN_SLOTS_IN_PSRAM
#define SLOT_MEM EXT_RAM_BSS_ATTR
#else
#define SLOT_MEM
#endif
static float arena0[ARENA_CAP];                              /* the built-in program flies from internal RAM */
static SLOT_MEM float arena1[ARENA_CAP], arena2[ARENA_CAP];
static SLOT_MEM int32_t code1[CODE_CAP], code2[CODE_CAP];
static float pool[3][POOL_CAP];
static SLOT_MEM uint8_t img_buf[IMG_CAP];

static rn_host H;
static portMUX_TYPE host_mux = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t events;                                 /* lines of text for the Pi */
typedef struct { char text[64]; } event_line;

static void host_lock(void *ctx, int on) { if (on) portENTER_CRITICAL(&host_mux); else portEXIT_CRITICAL(&host_mux); }
static void host_event(void *ctx, int code, const char *what) {
  static const char *names[] = { "", "loaded", "rejected", "swapped", "fell back", "built-in failed" };
  event_line l; snprintf(l.text, sizeof l.text, "%s%s%s", names[code], what ? ": " : "", what ? what : "");
  xQueueSend(events, &l, 0);                                 /* never blocks the flight task; called outside the host's lock */
  if (code == RN_EV_BUILTIN_FAILED) { /* failsafe: e.g. cut the motors or hold a fixed hover throttle */ }
}

/* Formula indices, looked up once. */
static int f_att, f_err, f_ctl, f_srv;

/* One control step. The inputs are flat floats in each formula's signature order (js/rn-sigs.js): a number is
 * 1 float, a 3-vector 3, a 3×3 matrix 9, an optional value its present flag then the value. */
static void flight_step(float dt) {
  float gyro[3] = { 0 }, accel[3] = { 0, 0, 9.81f }, mag[3] = { 1, 0, 0 };   /* your IMU and compass drivers */
  int have_mag = 1;

  /* attitudeEstimator(st, gyro, accel, mag?, dt) → { q[4], w[3] } */
  float in[64], att[7];
  memcpy(in, gyro, 12); memcpy(in + 3, accel, 12); in[6] = have_mag; memcpy(in + 7, mag, 12); in[10] = dt;
  if (rn_host_call(&H, f_att, 0, in, att)) return;          /* no program could answer: failsafe (host_event) */

  /* attitudeError(R, Rd) → e[3], attitudeControl(eR, w, ia, J) → torque[3]: R from att's quaternion, Rd from the
   * position controller; the simulator's control step (js/sim.js) shows the whole chain, including
   * positionEstimator, positionControl, thrustAxisTarget, forceDemand, allocationPreferences, allocation,
   * thrustLinearization and voltageCompensation, and the learning (identifyEffectiveness) in between. */

  /* servoPredictor(st, cmd, rate, lag, dt) → angle, one memory per servo */
  for (int s = 0; s < N_SERVOS; s++) {
    float sin_[4] = { 0.0f /* commanded angle */, 5.0f /* rate, rad/s */, 0.02f /* lag, s */, dt }, th;
    rn_host_call(&H, f_srv, s, sin_, &th);
  }

  rn_host_tick(&H, dt);                                      /* the loading steps advance with flight time */
}

static void flight_task(void *arg) {
  TickType_t last = xTaskGetTickCount();
  int64_t t0 = esp_timer_get_time();
  for (;;) {
    vTaskDelayUntil(&last, 1);                               /* 1 ms with CONFIG_FREERTOS_HZ=1000 */
    int64_t t = esp_timer_get_time(); float dt = (float)(t - t0) * 1e-6f; t0 = t;
    flight_step(dt);
  }
}

static void link_send(uint8_t type, const char *text) {
  static uint8_t fr[128]; uint32_t n = rn_link_frame(fr, sizeof fr, type, (const uint8_t *)text, (uint32_t)strlen(text));
  if (n) uart_write_bytes(LINK_UART, fr, n);
}
static void link_task(void *arg) {
  static rn_link L; rn_link_init(&L, img_buf, IMG_CAP);
  static uint8_t rx[512];
  for (;;) {
    event_line l;
    while (xQueueReceive(events, &l, 0)) link_send(RN_LINK_EVENT, l.text);
    int n = uart_read_bytes(LINK_UART, rx, sizeof rx, pdMS_TO_TICKS(10));
    for (int i = 0; i < n; i++) {
      int type = rn_link_feed(&L, rx[i]);
      if (type == RN_LINK_PROGRAM) {
        int e = rn_host_prepare(&H, img_buf, L.len);         /* checks and self-tests here, on core 0 */
        if (!e) link_send(RN_LINK_EVENT, "prepared: flying it in the background from the next control step");
      } else if (type == RN_LINK_STATUS) {
        char s[96]; snprintf(s, sizeof s, "flying slot %d (0 = built-in), candidate %d, phase %d, previous %d", H.act, H.cand, H.phase, H.prev);
        link_send(RN_LINK_REPORT, s);
      } else if (type < 0) link_send(RN_LINK_EVENT, "dropped a damaged frame");
    }
  }
}

void app_main(void) {
  events = xQueueCreate(16, sizeof(event_line));
  float *arenas[3] = { arena0, arena1, arena2 }, *pools[3] = { pool[0], pool[1], pool[2] };
  int32_t *codes[3] = { NULL, code1, code2 };                /* NULL: the built-in program's steps run from flash */
  H.event = host_event; H.lock = host_lock;
  int e = rn_host_init(&H, rn_builtin_img, rn_builtin_len, arenas, ARENA_CAP, codes, CODE_CAP, pools, POOL_CAP);
  if (e) { printf("built-in program: %s\n", rn_error_text(e)); return; }
  rn_host_instances(&H, "servoPredictor", N_SERVOS);
  f_att = rn_host_find(&H, "attitudeEstimator"); f_err = rn_host_find(&H, "attitudeError");
  f_ctl = rn_host_find(&H, "attitudeControl"); f_srv = rn_host_find(&H, "servoPredictor");

  uart_config_t cfg = { .baud_rate = LINK_BAUD, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE,
                        .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT };
  uart_driver_install(LINK_UART, 8192, 1024, 0, NULL, 0);
  uart_param_config(LINK_UART, &cfg);
  uart_set_pin(LINK_UART, LINK_TX, LINK_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  xTaskCreatePinnedToCore(link_task, "link", 6144, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(flight_task, "flight", 8192, NULL, configMAX_PRIORITIES - 1, NULL, 1);
}
