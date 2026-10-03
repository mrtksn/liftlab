/*
 * The command module on an ESP32: ground_core.c (the same code the simulator runs on the far side of its radio link,
 * and dfb_ground runs on a Mac or a Pi) with buttons, sticks, a buzzer and an LED wired to the ESP32, driving an
 * ExpressLRS transmitter module over CRSF.
 *
 * Wiring (set over USB, kept in flash; type "show" for what is set now):
 *   set tx=17,16          the transmitter module: the ESP32's TX pin to the module's CRSF input, then the pin its
 *                         replies come in on. One pin for both (set tx=17,17) for a module bay's single CRSF wire:
 *                         it is driven open-drain, with a pull-up (add 1–4.7 kΩ to 3.3 V if the line has none)
 *   set baud=400000       the CRSF speed the module expects
 *   set arm=25            a button (to ground; GPIO 34–39 need an external pull-up): right left fwd back up down
 *                         yawr yawl arm fly hold home gentle normal sport cal. set arm=-1 removes it
 *   set latch=arm,fly     the buttons that toggle on each press (push buttons as switches)
 *   set roll=34           an analog stick on an ADC pin (32–39), centred at power-on; 34i inverts it.
 *                         Also pitch, throttle, yaw. set span=1800: the reading from the centre to full deflection
 *   set buzzer=26         beeps the command module's alerts (groundAlerts): an alarm fast, a warning every 2 s
 *   set led=2             lit while all is fine, blinking for an alert
 *   save, reboot, show
 * And the text commands (ground_text.h) from a script or a terminal on the USB port: press/release/tap NAME,
 * stick AXIS V, goto X Y Z [HEADING], calibrate, cmd ID V…, status, messages. A program with edited formulas is
 * not loaded here yet: the ESP32 runs its built-in one (dfb_ground --program takes one).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "ground_core.h"
#include "ground_text.h"

extern const uint8_t *const rn_builtin_ground_img;
extern const uint32_t rn_builtin_ground_len;
#define CONSOLE UART_NUM_0
#define TXU UART_NUM_2

/* ── the wiring ── */
#define CFG_VERSION 1
typedef struct {
  uint32_t version;
  int8_t tx, rx; int32_t baud;
  int8_t btn[GB_N];
  int8_t ax[GND_AXES]; uint8_t ax_inv[GND_AXES]; int16_t span;
  int8_t buzzer, led; uint32_t latch;
} gcfg;
static gcfg C, N;                  /* C: the wiring running now; N: as set since (saved, it runs after a reboot) */
static void defaults(gcfg *c) {
  memset(c, 0, sizeof *c); c->version = CFG_VERSION; c->tx = 17; c->rx = 16; c->baud = 400000;
  for (int i = 0; i < GB_N; i++) c->btn[i] = -1;
  for (int a = 0; a < GND_AXES; a++) c->ax[a] = -1;
  c->span = 1800; c->buzzer = -1; c->led = 2; c->latch = GB(GB_ARM) | GB(GB_FLY);
}
static void cfg_load(void) {
  defaults(&C); nvs_handle_t h;
  if (nvs_open("dfbg", NVS_READONLY, &h) == ESP_OK) {
    gcfg t; size_t n = sizeof t; if (nvs_get_blob(h, "cfg", &t, &n) == ESP_OK && n == sizeof t && t.version == CFG_VERSION) C = t;
    nvs_close(h);
  }
  N = C;
}
static int cfg_save(void) {
  nvs_handle_t h; if (nvs_open("dfbg", NVS_READWRITE, &h) != ESP_OK) return -1;
  esp_err_t e = nvs_set_blob(h, "cfg", &N, sizeof N); if (e == ESP_OK) e = nvs_commit(h); nvs_close(h); return e == ESP_OK ? 0 : -1;
}
static int pin_in(int p) { return p == -1 || (p >= 0 && p <= 39 && !(p >= 6 && p <= 11) && p != 20 && p != 24 && !(p >= 28 && p <= 31)); }
static int pin_out(int p) { return p == -1 || (pin_in(p) && p < 34); }
/* The wiring as set (N); what runs (C) changes only at the next power-on, so a half-done rewiring never flies. */
static void show(char *o, int n) {
  int k = memcmp(&N, &C, sizeof N) ? snprintf(o, (size_t)n, "(changed since power-on: runs after save and reboot) ") : 0;
  k += snprintf(o + k, (size_t)(n - k), "tx=%d,%d baud=%ld span=%d buzzer=%d led=%d latch=", N.tx, N.rx, (long)N.baud, N.span, N.buzzer, N.led);
  for (int b = 0; b < GB_N && k < n - 20; b++) if (N.latch & GB(b)) k += snprintf(o + k, (size_t)(n - k), "%s,", gnd_button_names[b]);
  for (int b = 0; b < GB_N && k < n - 20; b++) if (N.btn[b] >= 0) k += snprintf(o + k, (size_t)(n - k), " %s=%d", gnd_button_names[b], N.btn[b]);
  static const char *const ax[4] = { "roll", "pitch", "throttle", "yaw" };
  for (int a = 0; a < GND_AXES && k < n - 20; a++) if (N.ax[a] >= 0) k += snprintf(o + k, (size_t)(n - k), " %s=%d%s", ax[a], N.ax[a], N.ax_inv[a] ? "i" : "");
}
/* "set key=value" into N: 0, or −1 with why in err */
static int setting(char *kv, char *err, int en) {
  char *e = strchr(kv, '='); if (!e) { snprintf(err, (size_t)en, "set key=value"); return -1; }
  *e = 0; const char *k = kv, *v = e + 1; int x = atoi(v);
  static const char *const ax[4] = { "roll", "pitch", "throttle", "yaw" };
  if (!strcmp(k, "tx")) { int a = -1, b = -1; if (sscanf(v, "%d,%d", &a, &b) != 2 || !pin_out(a) || !pin_in(b) || a < 0 || b < 0) { snprintf(err, (size_t)en, "tx=OUT,IN (one pin for both is fine: tx=17,17)"); return -1; } N.tx = (int8_t)a; N.rx = (int8_t)b; }
  else if (!strcmp(k, "baud")) { if (x < 9600 || x > 5250000) { snprintf(err, (size_t)en, "baud: 9600 to 5250000"); return -1; } N.baud = x; }
  else if (!strcmp(k, "span")) { if (x < 100 || x > 2047) { snprintf(err, (size_t)en, "span: 100 to 2047"); return -1; } N.span = (int16_t)x; }
  else if (!strcmp(k, "buzzer") || !strcmp(k, "led")) { if (!pin_out(x)) { snprintf(err, (size_t)en, "%.20s: an output pin (below 34), or -1", k); return -1; } if (k[0] == 'b') N.buzzer = (int8_t)x; else N.led = (int8_t)x; }
  else if (!strcmp(k, "latch")) { N.latch = 0; char buf[128]; snprintf(buf, sizeof buf, "%s", v); char *sv; for (char *p = strtok_r(buf, ",", &sv); p; p = strtok_r(0, ",", &sv)) { int b = gnd_button(p); if (b >= 0) N.latch |= GB(b); } }
  else {
    for (int a = 0; a < GND_AXES; a++) if (!strcmp(k, ax[a])) {
      if (x != -1 && (x < 32 || x > 39)) { snprintf(err, (size_t)en, "%.20s: an ADC pin, 32–39 (or -1)", k); return -1; }
      N.ax[a] = (int8_t)x; N.ax_inv[a] = strchr(v, 'i') != 0; return 0;
    }
    int b = gnd_button(k); if (b < 0) { snprintf(err, (size_t)en, "no setting %.20s", k); return -1; }
    if (!pin_in(x)) { snprintf(err, (size_t)en, "%.20s: not a usable pin", k); return -1; }
    N.btn[b] = (int8_t)x;
  }
  return 0;
}

/* ── the hardware ── */
static adc_oneshot_unit_handle_t adc; static adc_channel_t ax_ch[GND_AXES]; static int ax_ok[GND_AXES]; static float ax_mid[GND_AXES];
static void hw_init(void) {
  for (int b = 0; b < GB_N; b++) if (C.btn[b] >= 0) {
    gpio_config_t g = { .pin_bit_mask = 1ULL << C.btn[b], .mode = GPIO_MODE_INPUT, .pull_up_en = C.btn[b] < 34 ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE };
    gpio_config(&g);
  }
  for (int p = 0; p < 2; p++) { int pin = p ? C.led : C.buzzer; if (pin >= 0) { gpio_reset_pin(pin); gpio_set_direction(pin, GPIO_MODE_OUTPUT); gpio_set_level(pin, 0); } }
  int any = 0; for (int a = 0; a < GND_AXES; a++) any |= C.ax[a] >= 0;
  if (any) {
    adc_oneshot_unit_init_cfg_t u = { .unit_id = ADC_UNIT_1 }; adc_oneshot_new_unit(&u, &adc);
    for (int a = 0; a < GND_AXES; a++) if (C.ax[a] >= 0) {
      adc_unit_t un; if (adc_oneshot_io_to_channel(C.ax[a], &un, &ax_ch[a]) != ESP_OK || un != ADC_UNIT_1) continue;
      adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 }; adc_oneshot_config_channel(adc, ax_ch[a], &cc);
      int sum = 0; for (int k = 0; k < 16; k++) { int r = 0; adc_oneshot_read(adc, ax_ch[a], &r); sum += r; }
      ax_mid[a] = sum / 16.0f; ax_ok[a] = 1;   /* centred where it rests at power-on */
    }
  }
  uart_config_t uc = { .baud_rate = C.baud, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT };
  uart_driver_install(TXU, 1024, 1024, 0, NULL, 0); uart_param_config(TXU, &uc);
  uart_set_pin(TXU, C.tx, C.rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (C.tx == C.rx) { gpio_set_direction(C.tx, GPIO_MODE_INPUT_OUTPUT_OD); gpio_set_pull_mode(C.tx, GPIO_PULLUP_ONLY); }   /* one wire, both ways: open drain */
}
static void read_inputs(gnd_input *in) {
  memset(in, 0, sizeof *in);
  for (int b = 0; b < GB_N; b++) if (C.btn[b] >= 0 && gpio_get_level(C.btn[b]) == 0) in->held |= GB(b);
  for (int a = 0; a < GND_AXES; a++) if (ax_ok[a]) {
    int r = 0; if (adc_oneshot_read(adc, ax_ch[a], &r) != ESP_OK) continue;
    float x = (r - ax_mid[a]) / C.span; x = x > 1 ? 1 : x < -1 ? -1 : x;
    in->axis[a] = C.ax_inv[a] ? -x : x; in->has_axis |= 1u << a;
  }
}

/* ── the step runner and the core ── */
static float arenas_[3][2048], pools_[3][1024]; static int32_t codes_[3][2048]; static rn_host H;
static gnd_state G; static gnd_text_in TI;

void app_main(void) {
  if (nvs_flash_init() != ESP_OK) { nvs_flash_erase(); nvs_flash_init(); }
  cfg_load();
  printf("\nDrone Force Bench command module (ESP32)\n");
  float *ar[3] = { arenas_[0], arenas_[1], arenas_[2] }, *po[3] = { pools_[0], pools_[1], pools_[2] }; int32_t *co[3] = { codes_[0], codes_[1], codes_[2] };
  int e = rn_host_init(&H, rn_builtin_ground_img, rn_builtin_ground_len, ar, 2048, co, 2048, po, 1024);
  gnd_config gc; gnd_config_default(&gc); gc.latch = C.latch; gc.seq0 = (uint8_t)esp_random();   /* (gnd_config.seq0) */
  gnd_init(&G, e ? 0 : &H, &gc);
  printf("program: %s; %s\n", rn_error_text(e), G.why);
  hw_init();
  { char s[400]; show(s, sizeof s); printf("wiring: %s\n", s); }
  printf("type \"show\", \"set key=value\", \"save\", \"reboot\", or a command (status, goto X Y Z, calibrate…)\n");
  uart_driver_install(CONSOLE, 1024, 1024, 0, NULL, 0); uart_vfs_dev_use_driver(CONSOLE);

  TickType_t wake = xTaskGetTickCount(); int64_t t0 = esp_timer_get_time(), last = t0;
  char line[200]; int ln = 0; uint32_t msgs = 0; int alert_was = 0, why_was = 0; double next_status = 1;
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(4));
    int64_t now = esp_timer_get_time(); double t = (now - t0) * 1e-6; float dt = (float)((now - last) * 1e-6); last = now;
    /* the USB port: settings and text commands */
    uint8_t b[64]; int n = uart_read_bytes(CONSOLE, b, sizeof b, 0);
    for (int i = 0; i < n; i++) {
      int eol = b[i] == '\n' || b[i] == '\r';
      if (!eol && ln < (int)sizeof line - 1) { line[ln++] = (char)b[i]; continue; }
      if (!eol) i--;                                                 /* (a too-long line: run what came, then this character starts the next) */
      if (!ln) continue;
      line[ln] = 0; ln = 0; char reply[700] = "";
      if (!strncmp(line, "set ", 4)) { char err[100]; if (setting(line + 4, err, sizeof err)) snprintf(reply, sizeof reply, "%s", err); else snprintf(reply, sizeof reply, "ok (save, then reboot)"); }
      else if (!strcmp(line, "show")) show(reply, sizeof reply);
      else if (!strcmp(line, "save")) snprintf(reply, sizeof reply, cfg_save() ? "couldn't save" : "saved");
      else if (!strcmp(line, "reboot")) { fflush(stdout); esp_restart(); }
      else if (!gnd_text(&G, &TI, line, t, reply, sizeof reply)) snprintf(reply, sizeof reply, "unknown (show, set, save, reboot, press, release, tap, stick, goto, calibrate, cmd, status, messages)");
      if (reply[0]) printf("%s\n", reply);
    }
    /* what the module hands back */
    n = uart_read_bytes(TXU, b, sizeof b, 0);
    while (n > 0) { gnd_from_radio(&G, b, n, t); n = uart_read_bytes(TXU, b, sizeof b, 0); }
    /* a step */
    rn_host_tick(&H, dt);
    gnd_input in; read_inputs(&in); gnd_text_inputs(&TI, t, &in);
    uint8_t out[128]; int m = gnd_step(&G, &in, t, dt, out, sizeof out);
    if (m) uart_write_bytes(TXU, out, (size_t)m);
    /* the pilot: the drone's messages, the alerts on the buzzer and the LED */
    for (; msgs < G.V.nmsg; msgs++) printf("drone: %s\n", G.V.msg[msgs % GND_MSGS].s);
    int why, lvl = gnd_alert(&G, &why);
    if (lvl != alert_was || why != why_was) { printf("%s%s\n", lvl == 2 ? "ALARM: " : lvl ? "warning: " : "", lvl ? gnd_why_text[why] : "all fine again"); alert_was = lvl; why_was = why; }
    int ph = (int)(t * 1000);
    int beep = lvl == 2 ? (ph % 400) < 200 : lvl == 1 ? (ph % 2000) < 100 : 0;
    if (C.buzzer >= 0) gpio_set_level(C.buzzer, beep);
    if (C.led >= 0) gpio_set_level(C.led, lvl == 2 ? (ph % 200) < 100 : lvl == 1 ? (ph % 1000) < 500 : 1);
    if (t >= next_status && G.V.t_any < 0) { next_status = t + 5; printf("waiting for the drone's telemetry (type status for more)\n"); }
  }
}
