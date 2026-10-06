/*
 * LiftLab flight firmware for the ESP32.
 *
 * Core 1: the control loop (1 kHz by default): the newest IMU sample → fc_step (fc_core.c, the flight formulas
 *   through the program slots) → ESC and servo pulses.
 * Core 0: the sensor task (reads the IMU at the loop's rate and the barometer at 25 Hz, then wakes the control
 *   loop), and the link task (the Pi on the USB serial port, 921600 baud by default (setting baud): commands,
 *   programs, the airframe, settings, telemetry and events; the learning's and the supervisor's frames; see rn_link.h).
 *
 * Safety as the firmware sees it (the rest is in fc_core.h):
 *   - from power-on every ESC gets its minimum pulse (standard PWM ESCs arm on it and stay still);
 *   - it won't arm without a gyro, an airframe, or with fewer outputs wired than the airframe has motors/servos;
 *   - the airframe, the wiring and reboots are only accepted disarmed;
 *   - commands must keep coming (fly.py sends them 50 times a second): 0.5 s without one while flying → failsafe.
 * Take the props off for anything but flying: the motor test spins motors.
 *
 * While the Pi's navigation sends guided commands (12-float RN_LINK_CMD), it gets RN_LINK_NAV 100 times a second.
 * While the Pi runs the learning or the health supervisor (it sends RN_LINK_WANT), it gets RN_LINK_LTEL 200 times a
 * second (fewer at slower links), and takes their RN_LINK_EXC, RN_LINK_MODEL and RN_LINK_SET (fc_core.h).
 * The pilot's radio (radio_link.h): its channels fly the drone (rc_core.h): without the Pi's navigation, as the stick
 * command; with it, they go to the Pi (RN_LINK_RC). The telemetry task (tlm_core.h) sends what the flight core and the
 * Pi's tasks publish back down the radio as CRSF frames, within the link's budget. Without a radio, if the Pi runs the
 * telemetry task (it sends RN_LINK_WANT bit 2), the flight core's items go to the Pi instead (RN_LINK_TLM).
 * Which radio (settings, as all the others: over the link, save, reboot; "show" lists them, the secrets masked):
 *   radio=elrs,250,4      an ExpressLRS (or Crossfire) receiver on a second UART, crsf=RX,TX (CRSF at 420000 baud);
 *                         the packet rate and telemetry ratio as set on the radio (the default)
 *   radio=espnow,6[,lr]   ESP-NOW, ESP32 to ESP32 on Wi-Fi channel 6 (1–13), no network, no receiver: the command
 *                         module is an ESP32 with the ground firmware set the same (radio=espnow,6). lr: Espressif's
 *                         long-range mode, slower and further: both ends lr, or neither
 *   radio=wifi,ap,6       Wi-Fi: the drone makes a network on channel 6 (LiftLab-XXXX, XXXX from its MAC address, as
 *                         it says at power-on; 192.168.4.1) and takes UDP on port 14570
 *   radio=wifi,sta        Wi-Fi: the drone joins wifi= (it says its address once joined)
 *   radio=serial,115200   a serial line on crsf=RX,TX (set those first) at that speed (19200 to 4000000 baud, the same
 *                         at both ends): whatever carries the UART's bytes to the command module, a laser or LED and a
 *                         photodiode, fibre transceivers, an infrared pair, a radio modem in transparent mode, a wire.
 *                         serial,57600,half: a line that goes one way at a time (most radio modems; 38400 and up):
 *                         the drone answers each packet. The packets are framed in the byte stream (pframe.h)
 *   radio=nrf24,1000      an nRF24L01 on SPI, nrf24=SCK,MOSI,MISO,CSN,CE (set those first; 3.3 V and a 10 µF capacitor
 *                         at the module): 250, 1000 or 2000 kbit/s (250 reaches furthest), the command module the same.
 *                         It hops over 8 channels from the binding phrase; the drone answers in the acknowledgements
 *   radio=ble             Bluetooth LE: the drone advertises LiftLab's service with the binding phrase's mark; a
 *                         command module ESP32 set radio=ble (or one on a laptop's USB) connects to it
 *   bind=PHRASE           1–31 characters, the same at both ends: it signs the packets, so nothing else flies the
 *                         drone. The default (liftlab) is everyone's: a warning says so at power-on. Set your own
 *   wifi=SSID,PASSWORD    the network: to join (sta), or the one it makes (ap; optional: LiftLab-XXXX by default,
 *                         its password then the binding phrase if it has 8+ characters, else liftlab1, with a warning).
 *                         wifi=SSID alone: that name, the default password. wifi= alone: the defaults
 * ESP-NOW and Wi-Fi need no crsf= wiring (crsf=-1 is fine): the ESP32's own radio is the receiver. The telemetry and
 * link statistics work as with ExpressLRS (plink.h does the receiver's part), the failsafe too, on a serial line as well.
 * Flying from a laptop over Wi-Fi: set radio=wifi,ap,6 and bind=YOUR PHRASE (8+ characters: it's also the network's
 * password), save, reboot. Join the laptop to LiftLab-XXXX with that password, then
 *   dfb_ground --radio wifi --drone 192.168.4.1 --bind "YOUR PHRASE" --keys
 * With two ESP32s over ESP-NOW: this one radio=espnow,6 bind=…; the command module (runner/ground/esp32) set radio=
 * espnow,6 and the same bind=. From a laptop over ESP-NOW: that command module on its USB port is the laptop's
 * transmitter module (dfb_ground --tx /dev/ttyUSB0 --baud 115200; see ground.c).
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
#include "tlm_sources.h"
#include "tlm_crsf.h"
#include "radio_link.h"
#include "radio_elrs.h"
#include "esp_radio.h"
#include "radio_cfg.h"
#include "hw.h"
#include "esp_board.h"

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
static void host_lock(void *c, int on) { if (on) { portENTER_CRITICAL(&mux); } else { portEXIT_CRITICAL(&mux); } }
static TaskHandle_t flight_h;
static radio_io *RADIO_IO;                         /* the pilot's radio link's bytes (radio_io.h), 0: no radio */
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
  if (SENS.imu == 1 || SENS.imu == 3) {
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
  TickType_t last = xTaskGetTickCount(); int tb = 0,tm=0; float mag[3]={0};int64_t mag_us=-1000000;
  for (;;) {
    vTaskDelayUntil(&last, period);
    fc_imu m; memset(&m, 0, sizeof m);
    int64_t us = esp_timer_get_time();
    if (hw_imu_read(&m)) memset(&m, 0, sizeof m);          /* a failed read: no gyro this sample */
    float a; int nb = 0;
    if(SENS.baro==2 || SENS.baro==3) nb=hw_baro_read(&a); else if ((tb += period) >= 40) { tb = 0; nb = hw_baro_read(&a); }
    if((tm+=period)>=10) { tm=0;if(hw_mag_read(mag)) mag_us=us; }
    m.have_mag=us-mag_us<100000;memcpy(m.mag,mag,sizeof mag);
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

/* the learning's and the supervisor's frames (link task → control loop), and LTEL (control loop → link task) */
static float exc_box[8 + FC_MAX_MOTORS + FC_MAX_JOINTS], set_box[6 + 3 * FC_MAX_MOTORS + 2 * FC_MAX_JOINTS], model_box[FC_MODEL_MAX];
static volatile int exc_n, set_n, model_n;
static float ltel_box[FC_LTEL_MAX]; static volatile int ltel_n; static volatile int64_t want_us = -10000000;
/* the telemetry task's store (the radio task and the link task), and the radio's input */
static tlm_store TS; static tlm_watch TW; static rc_input RCI; static volatile int64_t tlm_want_us = -10000000;
static portMUX_TYPE tlm_mux = portMUX_INITIALIZER_UNLOCKED;
/* What the telemetry reads of the flight state, copied out by the flight task: the radio task runs on the other core,
 * and reading F there while fc_step writes it tears the values (a quaternion half old, a "why" half written). */
static fc_state F_tlm; static volatile int F_tlm_new; static portMUX_TYPE snap_mux = portMUX_INITIALIZER_UNLOCKED;
static void snap_fields(fc_state *d, const fc_state *f) {   /* (just those fields: the whole state is kilobytes) */
  memcpy(d->q, f->q, sizeof d->q); d->vbatt = f->vbatt; d->have_alt = f->have_alt; d->alt_e = f->alt_e; d->vz_e = f->vz_e;
  d->state = f->state; d->att_ok = f->att_ok; d->cmd.guided = f->cmd.guided; d->open_loop = f->open_loop;
  d->A.n_motors = f->A.n_motors; memcpy(d->last_out.motor, f->last_out.motor, sizeof d->last_out.motor); memcpy(d->why, f->why, sizeof d->why);
}
static void snap_put(const fc_state *f) { portENTER_CRITICAL(&snap_mux); snap_fields(&F_tlm, f); F_tlm_new = 1; portEXIT_CRITICAL(&snap_mux); }
static float tlm_in[TLM_PACK_MAX]; static volatile int tlm_in_n;

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
    if(seq==last_seq && !m.have_gyro) m.have_mag=0;
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
        /* loaded into a copy, which then is the state: just what fc_airframe_load makes of it, as in the simulator
         * (the old airframe's learned model and the supervisor's settings go with it); refused, nothing changes */
        static fc_state G; G = F; int e = fc_airframe_load(&G, af_box, af_len);
        if (!e) {
          char why[64];
          if (!hw_outputs_ok(G.A.n_motors, G.A.n_joints, why, sizeof why)) { snprintf(F.why, sizeof F.why, "%s", why); e = -3; }
          else { F = G; outputs_ok = 1; }
        } else strcpy(F.why, G.why);
        af_result = e;
      }
      af_new = 0;
    }
    if (exc_n || set_n || model_n) {                  /* the Pi's learning and supervisor */
      static float b[FC_MODEL_MAX]; int n;
      if ((n = exc_n)) { portENTER_CRITICAL(&mux); memcpy(b, exc_box, (size_t)n * 4); exc_n = 0; portEXIT_CRITICAL(&mux); fc_exc(&F, b, n); }
      if ((n = model_n)) { portENTER_CRITICAL(&mux); memcpy(b, model_box, (size_t)n * 4); model_n = 0; portEXIT_CRITICAL(&mux); if (fc_model(&F, b, n)) post("the learning's model doesn't fit this airframe"); }
      if ((n = set_n)) { portENTER_CRITICAL(&mux); memcpy(b, set_box, (size_t)n * 4); set_n = 0; portEXIT_CRITICAL(&mux); fc_set(&F, b, n); }
    }
    rn_host_tick(&H, dt);
    fc_step(&F, &m, dt, vbatt, &OUT);
    static int lt_k = 0;                               /* LTEL: 200 Hz at 921600 baud, 100 at 460800, 50 slower */
    lt_k += HW.link_baud >= 921600 ? 200 : HW.link_baud >= 460800 ? 100 : 50;
    int lt_due = lt_k >= HW.rate_hz; if (lt_due) lt_k -= HW.rate_hz;
    if (lt_due && F.have_airframe && esp_timer_get_time() - want_us < 1000000) { static float lt[FC_LTEL_MAX]; int n = fc_ltel(&F, lt);
      portENTER_CRITICAL(&mux); memcpy(ltel_box, lt, (size_t)n * 4); ltel_n = n; portEXIT_CRITICAL(&mux);
    }
    static int nav_n = 0; nav_n += 100;
    int nav_due = nav_n >= HW.rate_hz; if (nav_due) nav_n -= HW.rate_hz;
    if (nav_due && m.have_gyro) {   /* 100 Hz: attitude, rates, specific force (body), height */
      float nb[16]; const float *Ri = F.A.imu_R;
      nb[0] = (float)F.t; nb[1] = (float)F.state; for (int k = 0; k < 4; k++) nb[2 + k] = F.q[k]; for (int k = 0; k < 3; k++) nb[6 + k] = F.w[k];
      for (int k = 0; k < 3; k++) nb[9 + k] = F.have_airframe ? Ri[3 * k] * m.acc[0] + Ri[3 * k + 1] * m.acc[1] + Ri[3 * k + 2] * m.acc[2] : m.acc[k];
      nb[12] = F.have_alt ? F.alt_e : 0; nb[13] = (float)F.have_alt; nb[14] = (float)F.att_ok; nb[15] = 0;
      portENTER_CRITICAL(&mux); memcpy(nav_box, nb, sizeof nb); nav_new = 1; portEXIT_CRITICAL(&mux);
    }
    if (F.have_airframe) hw_outputs_set(&OUT, F.A.n_motors, F.A.n_joints); else hw_outputs_safe();
    if(F.have_airframe && outputs_ok && !hw_outputs_ok(F.A.n_motors,F.A.n_joints,outputs_why,sizeof outputs_why)) {
      outputs_ok=0;F.state=FC_CRASHED;snprintf(F.why,sizeof F.why,"%s",outputs_why);memset(OUT.motor,0,sizeof OUT.motor);
    }
    if (F.state != last_state || strcmp(F.why, last_why)) {
      char s[80]; snprintf(s, sizeof s, "%s: %s", fc_state_name(F.state), F.why); post(s);
      last_state = F.state; strcpy(last_why, F.why);
    }
    if (!F_tlm_new) snap_put(&F);                     /* (taken by the radio task, at 100 Hz) */
    int64_t took = esp_timer_get_time() - t0;
    loop_us = took; if (took > loop_max) loop_max = took; if (took > 1000000 / HW.rate_hz) loop_late++;
  }
}

/* ── the link ── */
static uint8_t *img_buf;
static void link_send(uint8_t type, const void *p, uint32_t n) {
  static uint8_t fr[FC_MODEL_MAX * 4 + 16]; uint32_t k = rn_link_frame(fr, sizeof fr, type, p, n);
  if (k) uart_write_bytes(LINK, fr, k);
}
static void link_send2(uint8_t type, const void *p, uint32_t n) {   /* (the radio task's own buffer) */
  static uint8_t fr[1100]; uint32_t k = rn_link_frame(fr, sizeof fr, type, p, n);
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
  char s[800];
  if (!strcmp(line, "show")) {
    hw_describe(&HW, s, sizeof s); report(s);
    snprintf(s,sizeof s,"sensor profiles: imu=%d,%u baro=%d,%u mag=%d,%u",HW.imu_driver,HW.imu_addr,HW.baro_driver,HW.baro_addr,HW.mag_driver,HW.mag_addr);report(s);
    if (memcmp(&HW, &HW_next, sizeof HW)) { strcpy(s, "after a reboot: "); hw_describe(&HW_next, s + 16, sizeof s - 16); report(s); }
    strcpy(s, "radio: "); esp_radio_status(RADIO_IO, s + 7, sizeof s - 7); report(s);   /* (the packet link's counts: read as they are, a count may be a step behind) */
    snprintf(s, sizeof s, "IMU: %s; barometer: %s; compass: %s; %s; flying program slot %d", SENS.imu ? SENS.imu_name : "none", SENS.baro ? SENS.baro_name : "none", SENS.mag ? SENS.mag_name : "none", F.have_airframe ? F.why : "no airframe", H.act);
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
    case RN_LINK_AIRFRAME: return AIRFRAME_CAP; case RN_LINK_PROGRAM: return IMG_CAP;
    case RN_LINK_EXC: return sizeof exc_box; case RN_LINK_SET: return sizeof set_box; case RN_LINK_MODEL: return sizeof model_box; case RN_LINK_WANT: return 4;
    case RN_LINK_TLM: return sizeof tlm_in; }
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
    if (L.state == 3 && now - frame_t0 > (int64_t)L.len * 10 * 1000000 / HW.link_baud + 1000000) { rn_link_reset(&L); prev_state = 0; say("dropped a frame that ran over its time"); }
    /* A program takes a while to arrive (40 KB: 3.5 s at 115200 baud), and no commands can come meanwhile: keep flying on
     * the last one while its bytes keep flowing, for as long as a frame that size takes at this speed. A link that
     * stops, or a frame that runs over, still ends in the failsafe; and this never takes it out of the failsafe. */
    if (L.state == 3 && L.type == RN_LINK_PROGRAM && now - last_rx < 50000 && now - frame_t0 < (int64_t)L.len * 10 * 1000000 / HW.link_baud + 500000 && now - keep > 100000) {
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
      } else if (type == RN_LINK_EXC || type == RN_LINK_SET || type == RN_LINK_MODEL) {   /* the learning and the supervisor: to the control loop */
        float *box = type == RN_LINK_EXC ? exc_box : type == RN_LINK_SET ? set_box : model_box; volatile int *cnt = type == RN_LINK_EXC ? &exc_n : type == RN_LINK_SET ? &set_n : &model_n;
        portENTER_CRITICAL(&mux); memcpy(box, L.buf, L.len & ~3u); *cnt = (int)(L.len / 4); portEXIT_CRITICAL(&mux);
      } else if (type == RN_LINK_WANT) {
        float w = 1; if (L.len == 4) memcpy(&w, L.buf, 4);
        if ((int)w & 1) want_us = now;
        if ((int)w & 2) tlm_want_us = now;
      } else if (type == RN_LINK_TLM) {                   /* the Pi's tasks' telemetry, for the radio */
        portENTER_CRITICAL(&tlm_mux); memcpy(tlm_in, L.buf, L.len & ~3u); tlm_in_n = (int)(L.len / 4); portEXIT_CRITICAL(&tlm_mux);
      }
      else if (type < 0) say("dropped a damaged or oversized frame");
    }
    now = esp_timer_get_time();
    if (now >= next_b) { next_b = now + 50000; vbatt = hw_battery_read(); }
    int guided = now - guided_us < 1000000;
    if (guided && nav_new) { float nb[16]; portENTER_CRITICAL(&mux); memcpy(nb, nav_box, sizeof nb); nav_new = 0; portEXIT_CRITICAL(&mux); link_send(RN_LINK_NAV, nb, sizeof nb); }
    if (ltel_n) { static float lt[FC_LTEL_MAX]; int k; portENTER_CRITICAL(&mux); k = ltel_n; memcpy(lt, ltel_box, (size_t)k * 4); ltel_n = 0; portEXIT_CRITICAL(&mux); link_send(RN_LINK_LTEL, lt, (uint32_t)k * 4); }
    /* the full telemetry: at its rate, or twice a second while the Pi navigates (the link's room goes to RN_LINK_NAV) */
    if (telem_us && now >= next_t) { next_t = now + (guided ? 500000 : telem_us); telemetry(); }
  }
}

/* ── the pilot's radio and the telemetry task ── */
static void radio_task(void *arg) {
  radio_io *R = RADIO_IO; int radio = R != 0;
  rlink_cfg RL; hw_radio(&HW, &RL);
  static crsf_parser P; static uint8_t rx[128], out[256]; static float pk[TLM_PACK_MAX];
  fc_state *Fs = calloc(1, sizeof *Fs);               /* (the telemetry's copy of the flight state, on the heap: static DRAM is short with Wi-Fi) */
  if (!Fs) { post("no memory for the telemetry: no radio"); vTaskDelete(NULL); }
  tlm_init(&TS); tlm_watch_init(&TW);
  int64_t next_pub = 0, next_rc = 0, next_want = 0, next_pack = 0;
  for (;;) {
    int n = radio ? R->read(R, rx, sizeof rx, 2) : (vTaskDelay(pdMS_TO_TICKS(5)), 0); if (n < 0) n = 0;
    int64_t now = esp_timer_get_time(); double t = now * 1e-6;
    for (int i = 0; i < n; i++) tlm_crsf_input(&P, rx[i], &RCI, t);
    int guided = now - guided_us < 1000000;                  /* the Pi's navigation flies it */
    if (radio && !guided) {                                  /* angle mode: the sticks, while the channels come */
      fc_cmd c; if (!rc_stick_cmd(&RCI, t, &c)) { portENTER_CRITICAL(&mux); cmd_box = c; cmd_new = 1; portEXIT_CRITICAL(&mux); }
    }
    if (radio && guided && now >= next_rc) { next_rc = now + 20000; float r[RC_PACK_N]; rc_pack(&RCI, t, r); link_send2(RN_LINK_RC, r, sizeof r); }
    if (tlm_in_n) { int k; portENTER_CRITICAL(&tlm_mux); k = tlm_in_n; memcpy(pk, tlm_in, (size_t)k * 4); tlm_in_n = 0; portEXIT_CRITICAL(&tlm_mux); tlm_unpack(&TS, pk, k, t); }
    if (now >= next_pub) { next_pub = now + 10000; if (F_tlm_new) { portENTER_CRITICAL(&snap_mux); snap_fields(Fs, &F_tlm); F_tlm_new = 0; portEXIT_CRITICAL(&snap_mux); } tlm_from_core(&TS, &TW, Fs, t); if (radio) tlm_from_link(&TS, &RCI, t); }
    if (radio) {
      int m = tlm_service(&TS, &tlm_crsf, t, rlink_budget_now(&RL, &RCI, t), out, sizeof out); if (m) R->write(R, out, m);
      if (now >= next_want) { next_want = now + 500000; float w = 2; link_send2(RN_LINK_WANT, &w, 4); }   /* the Pi's items, please */
    } else if (now - tlm_want_us < 1000000 && now >= next_pack) {   /* the Pi runs the telemetry: our items go there */
      next_pack = now + 50000; int k = tlm_pack(&TS, pk, TLM_PACK_MAX); if (k) link_send2(RN_LINK_TLM, pk, (uint32_t)k * 4);
    }
  }
}

/* The radio the settings ask for (radio_io.h), started; 0: none. */
static radio_io *radio_start(void) {
  rlink_cfg L; hw_radio(&HW, &L);
  if (L.kind != RLINK_BLE) radio_ble_release();                  /* (Bluetooth's memory back to the heap: not this time) */
  if (L.kind == RLINK_ELRS) return radio_elrs_start(&HW);
  if (rcfg_bind_default(HW.bind)) printf("WARNING: the binding phrase is the default (liftlab): anyone who knows it can fly this drone. set bind=YOUR PHRASE (the same on the command module)\n");
  if (HW.crsf_rx >= 0 && L.kind != RLINK_SERIAL) printf("(crsf=%d,%d is set, but this radio is the ESP32's own: those pins stay free)\n", HW.crsf_rx, HW.crsf_tx);
  radio_io *R = L.kind == RLINK_ESPNOW ? radio_espnow_start(&L, PLINK_DRONE, HW.bind, post)
              : L.kind == RLINK_SERIAL ? radio_uart_start(&L, PLINK_DRONE, HW.bind, LB_RADIO_UART, HW.crsf_tx, HW.crsf_rx, post)   /* (the receiver's pins: the line's) */
              : L.kind == RLINK_NRF24 ? radio_nrf24_start(&L, PLINK_DRONE, HW.bind, HW.nrf_pin, post)
              : L.kind == RLINK_BLE ? radio_ble_start(&L, PLINK_DRONE, HW.bind, post)
              : radio_wifi_start(&L, PLINK_DRONE, HW.bind, HW.wifi_ssid, HW.wifi_pass, 0, post);
  printf("radio: %s; free heap %u bytes\n", R ? R->name : "DIDN'T START (see the next messages): no pilot's radio", (unsigned)esp_get_free_heap_size());
  return R;
}

void app_main(void) {
  /* Outputs first: every ESC at its minimum pulse from the start. */
  if (nvs_flash_init() != ESP_OK) { nvs_flash_erase(); nvs_flash_init(); }
  hw_load(&HW); HW_next = HW;
  char log[200];
  int oe = hw_outputs_init(&HW, log, sizeof log);
  printf("\n\nLiftLab flight controller\n%s\n", log);
  if (oe) printf("OUTPUTS DIDN'T START: motors stay off\n");

  int se = hw_sensors_init(&HW, &SENS, log, sizeof log);
  printf("IMU: %s\nbarometer: %s\n%s%s", SENS.imu ? SENS.imu_name : "none", SENS.baro ? SENS.baro_name : "none (the throttle stick sets vertical acceleration; the failsafe descent is rough)", log, log[0] ? "\n" : "");
  printf("compass: %s\n",SENS.mag ? SENS.mag_name : "none");
  if (SENS.imu == 2) printf("The LIS3DH has no gyro: this board can't fly (it won't arm). Motor tests and telemetry work. Add an MPU-6050 (GY-521) for flying.\n");
  if (se) printf("no IMU: it won't arm\n");
  if (HW.batt_pin >= 0 && hw_battery_init(&HW)) printf("battery: GPIO %d isn't an ADC1 pin\n", HW.batt_pin);

  /* The pilot's radio before the program slots: Wi-Fi takes its memory first (the slots fit in what's left; one
   * slot fewer for programs from the Pi, maybe, but the drone can still be flown). Its news comes after the link starts. */
  RADIO_IO = radio_start();

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
  printf("control loop %d Hz on core %d; telemetry %d Hz; free heap %u bytes\n\n", HW.rate_hz, LB_FLIGHT_CPU, HW.telem_hz, (unsigned)esp_get_free_heap_size());

  printf("link: %ld baud from here on\n", (long)HW.link_baud); fflush(stdout); vTaskDelay(pdMS_TO_TICKS(20));
  uart_driver_install(LINK, 8192, 8192, 0, NULL, 0);
  uart_set_baudrate(LINK, (uint32_t)HW.link_baud);
  uart_vfs_dev_use_driver(LINK);
  xTaskCreatePinnedToCore(flight_task, "flight", 16384, NULL, configMAX_PRIORITIES - 1, &flight_h, LB_FLIGHT_CPU);
  xTaskCreatePinnedToCore(sensor_task, "sensors", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 0);
  xTaskCreatePinnedToCore(link_task, "link", 8192, NULL, 5, NULL, 0);
  xTaskCreatePinnedToCore(radio_task, "radio", 6144, NULL, 4, NULL, 0);
}
