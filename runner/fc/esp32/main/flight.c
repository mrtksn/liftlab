/*
 * Drone Force Bench flight firmware for the ESP32.
 *
 * Core 1: the control loop (1 kHz by default): the newest IMU sample → fc_step (fc_core.c, the flight formulas
 *   through the program slots) → ESC and servo pulses.
 * Core 0: the sensor task (reads the IMU at the loop's rate and the barometer at 25 Hz, then wakes the control
 *   loop), and the link task (the Pi on the USB serial port, 115200 baud: commands, programs, the airframe,
 *   settings, telemetry and events; see rn_link.h).
 *
 * Safety as the firmware sees it (the rest is in fc_core.h):
 *   - from power-on every ESC gets its minimum pulse (standard PWM ESCs arm on it and stay still);
 *   - it won't arm without a gyro, an airframe, or with fewer outputs wired than the airframe has motors/servos;
 *   - the airframe, the wiring and reboots are only accepted disarmed;
 *   - commands must keep coming (fly.py sends them 50 times a second): 0.5 s without one while flying → failsafe.
 * Take the props off for anything but flying: the motor test spins motors.
 *
 * While the Pi's navigation sends guided commands (12-float RN_LINK_CMD), it gets RN_LINK_NAV 100 times a second.
 * Telemetry (RN_LINK_TELEM), 36 floats: t, state, roll, pitch, yaw [deg], body rates [deg/s] ×3, height [m],
 * vertical speed [m/s], battery [V], loop [µs], longest loop [µs], flags (1 gyro, 2 barometer, 4 attitude
 * settled, 8 holding height, 16 airframe loaded), flying program slot, last formula error, 12 throttles, 8 servo
 * angles [deg].
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "rn_host.h"
#include "rn_link.h"
#include "fc_core.h"
#include "hw.h"

extern const uint8_t *const rn_builtin_img;
extern const uint32_t rn_builtin_len;

#define CODE_CAP 8192
#define POOL_CAP 512
#define IMG_CAP (40 * 1024)
#define AIRFRAME_CAP 4096       /* the largest airframe (12 motors on 2 joints each, 8 servos) is 3.6 KB */
#define LINK UART_NUM_0

static hw_config HW, HW_next;            /* the wiring in use, and as it will be after a reboot */
static hw_sensors SENS;
static rn_host H;
static fc_state F;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static void host_lock(void *c, int on) { if (on) portENTER_CRITICAL(&mux); else portEXIT_CRITICAL(&mux); }
static TaskHandle_t flight_h;
static int outputs_ok; static char outputs_why[64] = "outputs not wired";   /* every motor and servo of the airframe has a working output */

/* ── events: from either core to the link task ── */
#define NEV 16
static char ev_text[NEV][80]; static volatile uint32_t ev_w, ev_r;
static portMUX_TYPE ev_mux = portMUX_INITIALIZER_UNLOCKED;
static void post(const char *s) {
  portENTER_CRITICAL(&ev_mux);
  uint32_t i = ev_w % NEV; size_t n = strlen(s); if (n > 79) n = 79; memcpy(ev_text[i], s, n); ev_text[i][n] = 0; ev_w++;
  portEXIT_CRITICAL(&ev_mux);
}
static int take_event(char *out) {   /* the oldest event, copied under the lock; 0 if none */
  int got = 0; portENTER_CRITICAL(&ev_mux);
  if (ev_w - ev_r > NEV) ev_r = ev_w - NEV;                  /* overrun: skip what was overwritten */
  if (ev_r != ev_w) { memcpy(out, ev_text[ev_r % NEV], 80); ev_r++; got = 1; }
  portEXIT_CRITICAL(&ev_mux); return got;
}
static const char *EVN[] = { "", "program loaded, flying in the background", "program rejected", "program swapped in", "program fell back to the previous one", "the built-in program failed" };
static void host_event(void *ctx, int code, const char *what) {
  char s[80]; snprintf(s, sizeof s, "%s%s%s", EVN[code], what ? ": " : "", what ? what : ""); post(s);
}

/* ── sensor task → control loop ── */
static fc_imu imu_now; static int64_t imu_us; static uint32_t imu_seq; static float baro_alt; static int baro_new;   /* under imu_mux */
static portMUX_TYPE imu_mux = portMUX_INITIALIZER_UNLOCKED;
static void sensor_task(void *arg) {
  int period = 1000 / HW.rate_hz; if (period < 1) period = 1;
  /* The gyro's offset: averaged over a second while the drone stands still (skipped if it's moving). */
  if (SENS.imu == 1) {
    for (int tries = 0; tries < 5; tries++) {
      double s[3] = { 0 }, s2[3] = { 0 }; int n = 0; fc_imu m;
      for (int k = 0; k < 500; k++) { if (!hw_imu_read(&m)) { for (int i = 0; i < 3; i++) { s[i] += m.gyro[i]; s2[i] += m.gyro[i] * m.gyro[i]; } n++; } vTaskDelay(pdMS_TO_TICKS(2)); }
      if (n < 400) break;
      double var = 0; float b[3]; for (int i = 0; i < 3; i++) { b[i] = (float)(s[i] / n); var += s2[i] / n - (double)b[i] * b[i]; }
      char t[80];
      if (var < 3e-4) { hw_gyro_calibrate(b); snprintf(t, sizeof t, "gyro offset measured: %.2f %.2f %.2f °/s", b[0] * 57.3, b[1] * 57.3, b[2] * 57.3); post(t); break; }
      post(tries < 4 ? "the drone is moving: measuring the gyro offset again" : "the drone kept moving: flying without a measured gyro offset (the estimator learns it)");
    }
  }
  TickType_t last = xTaskGetTickCount(); int tb = 0;
  for (;;) {
    vTaskDelayUntil(&last, period);
    fc_imu m; memset(&m, 0, sizeof m);
    int64_t us = esp_timer_get_time();
    if (hw_imu_read(&m)) memset(&m, 0, sizeof m);          /* a failed read: no gyro this sample */
    float a; int nb = 0; if ((tb += period) >= 40) { tb = 0; nb = hw_baro_read(&a); }
    portENTER_CRITICAL(&imu_mux);
    imu_now = m; imu_us = us; imu_seq++; if (nb) { baro_alt = a; baro_new = 1; }
    portEXIT_CRITICAL(&imu_mux);
    xTaskNotifyGive(flight_h);
  }
}

/* ── link task → control loop: the newest command, an airframe to load ── */
static fc_cmd cmd_box; static volatile int cmd_new, keep_new;
static volatile int64_t guided_us = -10000000;   /* when the last guided command came (the Pi's navigation is flying) */
/* what the navigation flies on (RN_LINK_NAV), from the control loop at 100 Hz */
static float nav_box[16]; static volatile int nav_new;
static uint8_t af_box[AIRFRAME_CAP]; static volatile uint32_t af_len; static volatile int af_new, af_result;
static volatile float vbatt;

/* ── the control loop ── */
static volatile int64_t loop_us, loop_max; static volatile int loop_late;
static fc_out OUT;
static void flight_task(void *arg) {
  const float dt0 = 1.0f / HW.rate_hz;
  int last_state = -1; char last_why[64] = ""; uint32_t last_seq = 0; int64_t last_us = esp_timer_get_time();
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));      /* the sensor task wakes us with a fresh sample */
    int64_t t0 = esp_timer_get_time();
    fc_imu m; int64_t us; uint32_t seq; int nb;
    portENTER_CRITICAL(&imu_mux); m = imu_now; us = imu_us; seq = imu_seq; nb = baro_new; baro_new = 0; m.baro_alt = baro_alt; portEXIT_CRITICAL(&imu_mux);
    if (seq == last_seq) { m.have_gyro = 0; us = t0; }  /* no new sample: none this step (5 ms of these in flight stops it) */
    m.have_baro = nb;                                  /* only a new barometer reading counts */
    m.have_mag = 0;                                    /* no compass driver yet */
    /* the time since the last step, as measured (a late step integrates the time that really passed) */
    float dt = (float)(us - last_us) * 1e-6f; last_us = us; last_seq = seq;
    dt = dt < 0.5f * dt0 ? 0.5f * dt0 : dt > 3 * dt0 ? 3 * dt0 : dt;
    if (keep_new) { keep_new = 0; fc_keepalive(&F); }
    if (cmd_new) { fc_cmd c; portENTER_CRITICAL(&mux); c = cmd_box; cmd_new = 0; portEXIT_CRITICAL(&mux);
      int refuse = !outputs_ok && (c.arm || c.test_motor >= 0);
      if (refuse) { c.arm = 0; c.test_motor = -1; }
      fc_command(&F, &c);
      if (refuse) { const char *w = !F.have_airframe ? "no airframe loaded" : outputs_why; int k = snprintf(F.why, sizeof F.why, "won't arm: ");
        for (int q = 0; w[q] && k < (int)sizeof F.why - 1; q++) F.why[k++] = w[q];
        F.why[k] = 0; } }
    if (af_new) {                                     /* a new airframe: only while disarmed */
      if (F.state != FC_DISARMED) { af_result = -2; }
      else {
        fc_state G = F; int e = fc_airframe_load(&G, af_box, af_len);
        if (!e) {
          char why[64];
          if (!hw_outputs_ok(G.A.n_motors, G.A.n_joints, why, sizeof why)) { snprintf(F.why, sizeof F.why, "%s", why); e = -3; }
          else { F.A = G.A; F.have_airframe = 1; memcpy(F.th_cmd, G.th_cmd, sizeof F.th_cmd); memcpy(F.th_hat, G.th_hat, sizeof F.th_hat); strcpy(F.why, G.why); outputs_ok = 1; }
        } else strcpy(F.why, G.why);
        af_result = e;
      }
      af_new = 0;
    }
    rn_host_tick(&H, dt);
    fc_step(&F, &m, dt, vbatt, &OUT);
    static int nav_n = 0;
    if (++nav_n >= HW.rate_hz / 100 && m.have_gyro) {   /* 100 Hz: attitude, rates, specific force (body), height */
      nav_n = 0; float nb[16]; const float *Ri = F.A.imu_R;
      nb[0] = (float)F.t; nb[1] = (float)F.state; for (int k = 0; k < 4; k++) nb[2 + k] = F.q[k]; for (int k = 0; k < 3; k++) nb[6 + k] = F.w[k];
      for (int k = 0; k < 3; k++) nb[9 + k] = F.have_airframe ? Ri[3 * k] * m.acc[0] + Ri[3 * k + 1] * m.acc[1] + Ri[3 * k + 2] * m.acc[2] : m.acc[k];
      nb[12] = F.have_alt ? F.alt_e : 0; nb[13] = (float)F.have_alt; nb[14] = (float)F.att_ok; nb[15] = 0;
      portENTER_CRITICAL(&mux); memcpy(nav_box, nb, sizeof nb); nav_new = 1; portEXIT_CRITICAL(&mux);
    }
    if (F.have_airframe) hw_outputs_set(&OUT, F.A.n_motors, F.A.n_joints); else hw_outputs_safe();
    if (F.state != last_state || strcmp(F.why, last_why)) {
      char s[80]; snprintf(s, sizeof s, "%s: %s", fc_state_name(F.state), F.why); post(s);
      last_state = F.state; strcpy(last_why, F.why);
    }
    int64_t took = esp_timer_get_time() - t0;
    loop_us = took; if (took > loop_max) loop_max = took; if (took > 1000000 / HW.rate_hz) loop_late++;
  }
}

/* ── the link ── */
static uint8_t *img_buf;
static void link_send(uint8_t type, const void *p, uint32_t n) {
  static uint8_t fr[IMG_CAP > 512 ? 512 : IMG_CAP]; uint32_t k = rn_link_frame(fr, sizeof fr, type, p, n);
  if (k) uart_write_bytes(LINK, fr, k);
}
static void say(const char *text) { link_send(RN_LINK_EVENT, text, (uint32_t)strlen(text)); printf("%s\n", text); }   /* frame first: fly.py then skips the text copy */
static void report(const char *text) { link_send(RN_LINK_REPORT, text, (uint32_t)strlen(text)); }
static void telemetry(void) {
  float t[36] = { 0 }; const float *R = F.R;
  t[0] = F.t; t[1] = (float)F.state;
  t[2] = atan2f(R[7], R[8]) * 57.29578f; t[3] = -asinf(R[6] < -1 ? -1 : R[6] > 1 ? 1 : R[6]) * 57.29578f; t[4] = atan2f(R[3], R[0]) * 57.29578f;
  for (int k = 0; k < 3; k++) t[5 + k] = F.w[k] * 57.29578f;
  t[8] = F.have_alt ? F.alt_e : 0; t[9] = F.have_alt ? F.vz_e : F.vz_i; t[10] = vbatt;
  t[11] = (float)loop_us; t[12] = (float)loop_max; loop_max = 0;
  t[13] = (float)((SENS.imu == 1 ? 1 : 0) | (SENS.baro ? 2 : 0) | (F.att_ok ? 4 : 0) | (F.holding ? 8 : 0) | (F.have_airframe ? 16 : 0));
  t[14] = (float)H.act; t[15] = (float)F.trap;
  for (int i = 0; i < FC_MAX_MOTORS; i++) t[16 + i] = OUT.motor[i];
  for (int j = 0; j < FC_MAX_JOINTS; j++) t[28 + j] = OUT.servo[j] * 57.29578f;
  link_send(RN_LINK_TELEM, t, sizeof t);
}
static void setting(const char *line) {
  char s[400];
  if (!strcmp(line, "show")) {
    hw_describe(&HW, s, sizeof s); report(s);
    if (memcmp(&HW, &HW_next, sizeof HW)) { strcpy(s, "after a reboot: "); hw_describe(&HW_next, s + 16, sizeof s - 16); report(s); }
    snprintf(s, sizeof s, "IMU: %s; barometer: %s; %s; flying program slot %d", SENS.imu ? SENS.imu_name : "none", SENS.baro ? SENS.baro_name : "none", F.have_airframe ? F.why : "no airframe", H.act);
    report(s); return;
  }
  if (F.state != FC_DISARMED) { report("disarm first"); return; }
  if (!strcmp(line, "save")) { report(hw_save(&HW_next) ? "couldn't save the settings" : "saved; reboot to use them"); return; }
  if (!strcmp(line, "reboot")) { report("rebooting"); vTaskDelay(pdMS_TO_TICKS(100)); hw_outputs_safe(); esp_restart(); }
  if (!strcmp(line, "defaults")) { hw_defaults(&HW_next); report("default wiring; save and reboot to use it"); return; }
  char err[96];
  if (hw_set(&HW_next, line, err, sizeof err)) { report(err); return; }
  snprintf(s, sizeof s, "set %s (save, then reboot, to use it)", line); report(s);
}
/* The longest payload each frame type may have: a damaged header can't swallow the frames after it. */
static uint32_t frame_limit(uint8_t type) {
  switch (type) { case RN_LINK_CMD: return 48; case RN_LINK_STATUS: return 0; case RN_LINK_SETTING: return 127;
    case RN_LINK_AIRFRAME: return AIRFRAME_CAP; case RN_LINK_PROGRAM: return IMG_CAP; }
  return 0;
}
static void link_task(void *arg) {
  static rn_link L; rn_link_init(&L, img_buf, IMG_CAP); L.limit = frame_limit;
  static uint8_t rx[256];
  int64_t next_t = 0, next_b = 0, telem_us = HW.telem_hz ? 1000000 / HW.telem_hz : 0, last_rx = 0, keep = 0, frame_t0 = 0;
  int prev_state = 0;
  for (;;) {
    { char ev[80]; while (take_event(ev)) say(ev); }
    int n = uart_read_bytes(LINK, rx, sizeof rx, pdMS_TO_TICKS(5));
    int64_t now = esp_timer_get_time();
    if (n > 0) last_rx = now;
    /* a frame whose bytes stopped coming is dropped, so the next frame isn't taken as its payload */
    if (L.state != 0 && now - last_rx > 50000) { rn_link_reset(&L); prev_state = 0; say("dropped a frame that stopped halfway"); }
    /* a frame running well past the time its length takes at this speed is damaged (commands swallowed as payload) */
    if (L.state == 3 && now - frame_t0 > (int64_t)L.len * 10 * 1000000 / 115200 + 1000000) { rn_link_reset(&L); prev_state = 0; say("dropped a frame that ran over its time"); }
    /* A program takes seconds to arrive (40 KB at 115200 baud), and no commands can come meanwhile: keep flying on
     * the last one while its bytes keep flowing, for as long as a frame that size takes at this speed. A link that
     * stops, or a frame that runs over, still ends in the failsafe; and this never takes it out of the failsafe. */
    if (L.state == 3 && L.type == RN_LINK_PROGRAM && now - last_rx < 50000 && now - frame_t0 < (int64_t)L.len * 10 * 1000000 / 115200 + 500000 && now - keep > 100000) {
      keep = now; keep_new = 1;
    }
    for (int i = 0; i < n; i++) {
      int type = rn_link_feed(&L, rx[i]);
      if (L.state != 0 && prev_state == 0) frame_t0 = esp_timer_get_time();
      prev_state = L.state;
      if (type == RN_LINK_CMD && (L.len == 28 || L.len == 48)) {   /* the pilot's sticks, or a guided command from the Pi */
        float v[12] = { 0 }; memcpy(v, L.buf, L.len);
        fc_cmd c = { v[0] > 0.5f, v[1], v[2], v[3], v[4], v[5] >= -0.5f && v[5] < FC_MAX_MOTORS - 0.5f ? (int)(v[5] + 0.5f) : -1, v[6],
                     v[7] > 0.5f, { v[8], v[9], v[10] }, v[11] };
        if (c.guided || L.len == 48) guided_us = now;
        portENTER_CRITICAL(&mux); cmd_box = c; cmd_new = 1; portEXIT_CRITICAL(&mux);
      } else if (type == RN_LINK_PROGRAM) {
        char s[96]; snprintf(s, sizeof s, "received a program: %u bytes; checking it", (unsigned)L.len); say(s);
        int64_t t0 = esp_timer_get_time();
        int e = rn_host_prepare(&H, img_buf, L.len);
        if (!e) { snprintf(s, sizeof s, "checked and self-tested in %lld ms", (long long)((esp_timer_get_time() - t0) / 1000)); say(s); }
      } else if (type == RN_LINK_AIRFRAME) {
        memcpy(af_box, L.buf, L.len); af_len = L.len; af_result = 1; af_new = 1;
        while (af_new) vTaskDelay(1);
        if (af_result == -2) say("airframe: disarm first");
        else if (af_result) { char s[96]; snprintf(s, sizeof s, "airframe refused: %s", F.why); say(s); }
        else say(hw_airframe_save(af_box, af_len) ? "airframe loaded, but it couldn't be saved to flash" : "airframe loaded and saved");
      } else if (type == RN_LINK_SETTING) {
        char line[128]; uint32_t k = L.len < sizeof line - 1 ? L.len : sizeof line - 1; memcpy(line, L.buf, k); line[k] = 0; setting(line);
      } else if (type == RN_LINK_STATUS) {
        char s[160]; snprintf(s, sizeof s, "%s: %s; flying slot %d (0 = built-in), candidate %d, phase %d; loop %lld us, over time %d; free heap %u",
                              fc_state_name(F.state), F.why, H.act, H.cand, H.phase, (long long)loop_us, loop_late, (unsigned)esp_get_free_heap_size());
        report(s);
      } else if (type < 0) say("dropped a damaged or oversized frame");
    }
    now = esp_timer_get_time();
    if (now >= next_b) { next_b = now + 50000; vbatt = hw_battery_read(); }
    int guided = now - guided_us < 1000000;
    if (guided && nav_new) { float nb[16]; portENTER_CRITICAL(&mux); memcpy(nb, nav_box, sizeof nb); nav_new = 0; portEXIT_CRITICAL(&mux); link_send(RN_LINK_NAV, nb, sizeof nb); }
    /* the full telemetry: at its rate, or twice a second while the Pi navigates (the link's room goes to RN_LINK_NAV) */
    if (telem_us && now >= next_t) { next_t = now + (guided ? 500000 : telem_us); telemetry(); }
  }
}

void app_main(void) {
  /* Outputs first: every ESC at its minimum pulse from the start. */
  if (nvs_flash_init() != ESP_OK) { nvs_flash_erase(); nvs_flash_init(); }
  hw_load(&HW); HW_next = HW;
  char log[200];
  int oe = hw_outputs_init(&HW, log, sizeof log);
  printf("\n\nDrone Force Bench flight controller\n%s\n", log);
  if (oe) printf("OUTPUTS DIDN'T START: motors stay off\n");

  int se = hw_sensors_init(&HW, &SENS, log, sizeof log);
  printf("IMU: %s\nbarometer: %s\n%s%s", SENS.imu ? SENS.imu_name : "none", SENS.baro ? SENS.baro_name : "none (the throttle stick sets vertical acceleration; the failsafe descent is rough)", log, log[0] ? "\n" : "");
  if (SENS.imu == 2) printf("The LIS3DH has no gyro: this board can't fly (it won't arm). Motor tests and telemetry work. Add an MPU-6050 (GY-521) for flying.\n");
  if (se) printf("no IMU: it won't arm\n");
  if (HW.batt_pin >= 0 && hw_battery_init(&HW)) printf("battery: GPIO %d isn't an ADC1 pin\n", HW.batt_pin);

  /* The program slots: the built-in program's steps in IRAM, one slot for programs from the Pi (two if there's room). */
  int32_t asz; memcpy(&asz, rn_builtin_img + 8, 4); uint32_t acap = (uint32_t)asz + 512;
  int32_t cwords; memcpy(&cwords, rn_builtin_img + 16, 4);
  float *a0 = heap_caps_malloc(acap * 4, MALLOC_CAP_8BIT);
  int32_t *c0 = heap_caps_malloc((size_t)cwords * 4, MALLOC_CAP_EXEC | MALLOC_CAP_32BIT);
  img_buf = heap_caps_malloc(IMG_CAP, MALLOC_CAP_8BIT);
  float *a1 = img_buf ? heap_caps_malloc(acap * 4, MALLOC_CAP_8BIT) : NULL;
  int32_t *c1 = a1 ? heap_caps_malloc(CODE_CAP * 4, MALLOC_CAP_EXEC | MALLOC_CAP_32BIT) : NULL;
  if (!c1) { free(a1); a1 = NULL; }
  float *a2 = c1 ? heap_caps_malloc(acap * 4, MALLOC_CAP_8BIT) : NULL;
  int32_t *c2 = a2 ? heap_caps_malloc(CODE_CAP * 4, MALLOC_CAP_EXEC | MALLOC_CAP_32BIT) : NULL;
  if (a2 && !c2) { free(a2); a2 = NULL; }
  float *pools[3] = { calloc(POOL_CAP, 4), calloc(POOL_CAP, 4), calloc(POOL_CAP, 4) };
  float *arenas[3] = { a0, a1, a2 }; int32_t *codes[3] = { c0, c1, c2 };
  H.event = host_event; H.lock = host_lock;
  int e = a0 ? rn_host_init(&H, rn_builtin_img, rn_builtin_len, arenas, acap, codes, CODE_CAP, pools, POOL_CAP) : RN_E_TOO_BIG;
  printf("flight program: %s (%s)\n", rn_error_text(e), a2 ? "two slots for programs from the Pi" : a1 ? "one slot for programs from the Pi" : "built-in only");
  if (e) { printf("THE FLIGHT PROGRAM DIDN'T LOAD: motors stay off\n"); return; }
  if (fc_init(&F, &H)) { printf("flight code: %s\n", F.why); return; }
  F.vref = HW.vref; F.batt_wired = HW.batt_pin >= 0;

  /* The airframe from flash. */
  uint32_t afl = 0;   /* read into the program receive buffer, which isn't in use yet */
  if (img_buf && !hw_airframe_load(img_buf, AIRFRAME_CAP, &afl) && !fc_airframe_load(&F, img_buf, afl)) {
    outputs_ok = hw_outputs_ok(F.A.n_motors, F.A.n_joints, outputs_why, sizeof outputs_why);
    printf("%s\n%s\n", F.why, outputs_ok ? "every motor and servo has an output" : outputs_why);
  } else printf("no airframe yet: send one from the simulator (fly.py airframe FILE.dfa)\n");
  if (!img_buf) { printf("no memory for the link: this build can't run here\n"); return; }
  printf("control loop %d Hz on core 1; telemetry %d Hz; free heap %u bytes\n\n", HW.rate_hz, HW.telem_hz, (unsigned)esp_get_free_heap_size());

  uart_driver_install(LINK, 4096, 4096, 0, NULL, 0);
  uart_vfs_dev_use_driver(LINK);
  xTaskCreatePinnedToCore(flight_task, "flight", 16384, NULL, configMAX_PRIORITIES - 1, &flight_h, 1);
  xTaskCreatePinnedToCore(sensor_task, "sensors", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 0);
  xTaskCreatePinnedToCore(link_task, "link", 8192, NULL, 5, NULL, 0);
}
