/*
 * The command module on an ESP32: ground_core.c (the same code the simulator runs on the far side of its radio link,
 * and dfb_ground runs on a Mac or a Pi) with buttons, sticks, a buzzer and an LED wired to the ESP32, driving an
 * ExpressLRS transmitter module over CRSF.
 *
 * Wiring (set over USB, kept in flash; type "show" for what is set now):
 *   set tx=17,16          the transmitter module: the ESP32's TX pin to the module's CRSF input, then the pin its
 *                         replies come in on (plain serial, idling high). One pin for both (set tx=17,17) for a module
 *                         bay's single CRSF wire, run as ExpressLRS runs it there (CRSFHandset): inverted serial,
 *                         idling low (the module pulls it down while it listens), driven only while a frame goes out
 *                         and let go as soon as it has, as the module answers each frame right after it. Not 12
 *   set baud=400000       the CRSF speed the module expects
 *   set arm=25            a button (to ground; GPIO 34–39 need an external pull-up): right left fwd back up down
 *                         yawr yawl arm fly hold home gentle normal sport cal. set arm=-1 removes it. A button counts
 *                         as changed once it has read the same for 20 ms (contacts bounce)
 *   set latch=arm,fly     the buttons that toggle on each press (push buttons as switches)
 *   set roll=34           an analog stick on an ADC pin (32–39), centred at power-on; 34i inverts it.
 *                         Also pitch, throttle, yaw. set span=1800: the reading from the centre to full deflection
 *   set buzzer=26         beeps the command module's alerts (groundAlerts): an alarm fast, a warning every 2 s
 *   set led=2             lit while all is fine, blinking for an alert
 *   save, reboot, show    reboot is refused while the telemetry says the drone is armed or flying (reboot force)
 * Pins: not 1 and 3 (the USB port's serial: this console), 6–11 (the flash); 34–39 are inputs only. 0, 2, 5, 12 and
 * 15 are read at power-on to choose how the ESP32 boots: taken with a warning (nothing may pull them then; a button
 * on 0 held at power-on starts the bootloader). On a WROVER board (with PSRAM) 16 and 17 are the PSRAM's, the
 * default tx=17,16 among them: pick other pins there.
 * A restart that isn't a power-on (a crash, the watchdog, reboot force) keeps the switches as they were, in memory
 * that survives it, so the switch warning (ground_core.h) doesn't turn arm and fly off in flight.
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
#include "esp_attr.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"
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
static void cfg_fix(gcfg *c);
static void cfg_load(void) {
  defaults(&C); nvs_handle_t h;
  if (nvs_open("dfbg", NVS_READONLY, &h) == ESP_OK) {
    gcfg t; size_t n = sizeof t; if (nvs_get_blob(h, "cfg", &t, &n) == ESP_OK && n == sizeof t && t.version == CFG_VERSION) C = t;
    nvs_close(h);
  }
  cfg_fix(&C); N = C;
}
static int cfg_save(void) {
  nvs_handle_t h; if (nvs_open("dfbg", NVS_READWRITE, &h) != ESP_OK) return -1;
  esp_err_t e = nvs_set_blob(h, "cfg", &N, sizeof N); if (e == ESP_OK) e = nvs_commit(h); nvs_close(h); return e == ESP_OK ? 0 : -1;
}
/* pins: 1 and 3 are this console's; 6–11 the flash's; 20, 24, 28–31 aren't there; 34–39 only read */
static int pin_ok(int p) { return p >= 0 && p <= 39 && p != 1 && p != 3 && !(p >= 6 && p <= 11) && p != 20 && p != 24 && !(p >= 28 && p <= 31); }
static int pin_in(int p) { return p == -1 || pin_ok(p); }
static int pin_out(int p) { return p == -1 || (pin_ok(p) && p < 34); }
/* the module's line idles high on a full UART: on 12 at power-on that picks 1.8 V flash, and the ESP32 doesn't boot */
static int pin_crsf(int p) { return p != 12; }
static const char *pin_note(int p) {
  return p == 0 || p == 2 || p == 5 || p == 12 || p == 15 ? "a strapping pin (read at power-on to choose how the ESP32 boots): nothing may pull it then"
       : p == 16 || p == 17 ? "on a WROVER board (with PSRAM) the PSRAM's: not there" : 0;
}
/* a wiring saved by an older build that is refused now: that part back to its default, said */
static void cfg_fix(gcfg *c) {
  gcfg d; defaults(&d); int bad = 0;
  if (!pin_out(c->tx) || !pin_in(c->rx) || c->tx < 0 || c->rx < 0 || !pin_crsf(c->tx) || !pin_crsf(c->rx) || (c->tx == c->rx && !pin_out(c->rx))) { c->tx = d.tx; c->rx = d.rx; bad = 1; }
  for (int b = 0; b < GB_N; b++) if (!pin_in(c->btn[b])) { c->btn[b] = -1; bad = 1; }
  if (!pin_out(c->buzzer)) { c->buzzer = -1; bad = 1; }
  if (!pin_out(c->led)) { c->led = -1; bad = 1; }
  if (bad) printf("the saved wiring had pins not taken now (1, 3 or 12 for the module): those are back to their defaults\n");
}
/* The wiring as set (N); what runs (C) changes only at the next power-on, so a half-done rewiring never flies. */
static void show(char *o, int n) {
  int k = memcmp(&N, &C, sizeof N) ? snprintf(o, (size_t)n, "(changed since power-on: runs after save and reboot) ") : 0;
  k += snprintf(o + k, (size_t)(n - k), "tx=%d,%d baud=%ld span=%d buzzer=%d led=%d latch=", N.tx, N.rx, (long)N.baud, N.span, N.buzzer, N.led);
  for (int b = 0; b < GB_N && k < n - 20; b++) if (N.latch & GB(b)) k += snprintf(o + k, (size_t)(n - k), "%s,", gnd_button_names[b]);
  for (int b = 0; b < GB_N && k < n - 20; b++) if (N.btn[b] >= 0) k += snprintf(o + k, (size_t)(n - k), " %s=%d", gnd_button_names[b], N.btn[b]);
  static const char *const ax[4] = { "roll", "pitch", "throttle", "yaw" };
  for (int a = 0; a < GND_AXES && k < n - 20; a++) if (N.ax[a] >= 0) k += snprintf(o + k, (size_t)(n - k), " %s=%d%s", ax[a], N.ax[a], N.ax_inv[a] ? "i" : "");
}
/* "set key=value" into N: 0 (err: a warning, or empty), or −1 with why in err */
static int setting(char *kv, char *err, int en) {
  char *e = strchr(kv, '='); err[0] = 0; if (!e) { snprintf(err, (size_t)en, "set key=value"); return -1; }
  *e = 0; const char *k = kv, *v = e + 1; int x = atoi(v);
  static const char *const ax[4] = { "roll", "pitch", "throttle", "yaw" };
  if (!strcmp(k, "tx")) {
    int a = -1, b = -1; if (sscanf(v, "%d,%d", &a, &b) != 2 || !pin_out(a) || !pin_in(b) || a < 0 || b < 0 || !pin_crsf(a) || !pin_crsf(b)) { snprintf(err, (size_t)en, "tx=OUT,IN: output-capable pins, not 1, 3 or 12 (one pin for both is fine: tx=17,17)"); return -1; }
    N.tx = (int8_t)a; N.rx = (int8_t)b; x = pin_note(a) ? a : b;
  }
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
    if (!pin_in(x)) { snprintf(err, (size_t)en, "%.20s: not a usable pin (not 1 or 3: the USB port's; not 6–11: the flash's)", k); return -1; }
    N.btn[b] = (int8_t)x;
  }
  if (strcmp(k, "latch") && strcmp(k, "baud") && strcmp(k, "span") && x >= 0 && pin_note(x)) snprintf(err, (size_t)en, "GPIO %d is %s", x, pin_note(x));
  return 0;
}

/* ── the hardware ── */
/* One wire both ways (tx = rx, a module bay's CRSF pin), as ExpressLRS's CRSFHandset runs it from the module's side:
 * inverted serial (the UART inverts both ways), so the line idles low; listening, the pin is an input pulled down
 * (as the module's is); talking, it drives the line, with our receiver held at idle so we don't hear ourselves. */
static int one_wire;
static void wire_listen(void) {
  gpio_set_direction(C.tx, GPIO_MODE_INPUT); gpio_set_pull_mode(C.tx, GPIO_PULLDOWN_ONLY);
  esp_rom_gpio_connect_in_signal(C.tx, U2RXD_IN_IDX, false);
}
static void wire_talk(void) {
  gpio_set_pull_mode(C.tx, GPIO_FLOATING);
  esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, U2RXD_IN_IDX, false);   /* (0, inverted: idle) */
  gpio_set_level(C.tx, 0); gpio_set_direction(C.tx, GPIO_MODE_OUTPUT);                  /* (idle low, then the UART's) */
  esp_rom_gpio_connect_out_signal(C.tx, U2TXD_OUT_IDX, false, false);
}
static void to_module(const uint8_t *b, int n) {
  if (!one_wire) { uart_write_bytes(TXU, b, (size_t)n); return; }
  wire_talk(); uart_write_bytes(TXU, b, (size_t)n);
  uart_wait_tx_done(TXU, pdMS_TO_TICKS(10)); wire_listen();     /* the last bit out: let go at once, the module answers now */
}
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
  one_wire = C.tx == C.rx;
  if (one_wire) { uart_set_line_inverse(TXU, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV); wire_listen(); }
}
/* buttons, debounced: one changes once it has read the other way DEBOUNCE steps running; the first read is taken as
 * it is (held at power-on: held, not pressed) */
#define DEBOUNCE 5                 /* steps of 4 ms: 20 ms */
static uint32_t btn_state; static uint8_t btn_n[GB_N]; static int btn_read;
static void read_inputs(gnd_input *in) {
  memset(in, 0, sizeof *in);
  uint32_t raw = 0; for (int b = 0; b < GB_N; b++) if (C.btn[b] >= 0 && gpio_get_level(C.btn[b]) == 0) raw |= GB(b);
  if (!btn_read) { btn_read = 1; btn_state = raw; }
  for (int b = 0; b < GB_N; b++) {
    if (!((raw ^ btn_state) & GB(b))) btn_n[b] = 0;
    else if (++btn_n[b] >= DEBOUNCE) { btn_state ^= GB(b); btn_n[b] = 0; }
  }
  in->held = btn_state;
  for (int a = 0; a < GND_AXES; a++) if (ax_ok[a]) {
    int r = 0; if (adc_oneshot_read(adc, ax_ch[a], &r) != ESP_OK) continue;
    float x = (r - ax_mid[a]) / C.span; x = x > 1 ? 1 : x < -1 ? -1 : x;
    in->axis[a] = C.ax_inv[a] ? -x : x; in->has_axis |= 1u << a;
  }
}

/* ── the step runner and the core ── */
static float arenas_[3][2048], pools_[3][1024]; static int32_t codes_[3][2048]; static rn_host H;
static gnd_state G; static gnd_text_in TI;
/* the switches as sent, kept where a restart that isn't a power-on doesn't clear them (check: ~on, so half-written or
 * power-on noise doesn't count) */
#define KEEP_MAGIC 0x6466626Bu
static RTC_NOINIT_ATTR struct { uint32_t magic, on, check; } keep;
/* the telemetry's last word on the drone: armed (or in its failsafe), or flying */
static int drone_up(void) {
  const gnd_view *V = &G.V; int st = V->item[TLM_STATE].t >= 0 && V->item[TLM_STATE].n >= 1 ? (int)V->item[TLM_STATE].v[0] : 0;
  int nav = V->item[TLM_NAV].t >= 0 && V->item[TLM_NAV].n >= 5 ? (int)V->item[TLM_NAV].v[4] : 0;
  return st == 1 || st == 2 || (nav & 2);
}

void app_main(void) {
  if (nvs_flash_init() != ESP_OK) { nvs_flash_erase(); nvs_flash_init(); }
  cfg_load();
  printf("\nDrone Force Bench command module (ESP32)\n");
  float *ar[3] = { arenas_[0], arenas_[1], arenas_[2] }, *po[3] = { pools_[0], pools_[1], pools_[2] }; int32_t *co[3] = { codes_[0], codes_[1], codes_[2] };
  int e = rn_host_init(&H, rn_builtin_ground_img, rn_builtin_ground_len, ar, 2048, co, 2048, po, 1024);
  gnd_config gc; gnd_config_default(&gc); gc.latch = C.latch; gc.seq0 = (uint8_t)esp_random();   /* (gnd_config.seq0) */
  esp_reset_reason_t rr = esp_reset_reason();
  if (rr != ESP_RST_POWERON && keep.magic == KEEP_MAGIC && keep.check == ~keep.on) gc.resume = keep.on;   /* (a restart, maybe in flight) */
  keep.magic = 0;
  gnd_init(&G, e ? 0 : &H, &gc);
  /* a switch only text set (no button, not latching) is held by nothing else: set again */
  for (int b = GB_ARM; b <= GB_FLY; b++) if ((gc.resume & GB(b)) && !(C.latch & GB(b)) && C.btn[b] < 0) TI.held |= GB(b);
  if (gc.resume & (GB(GB_ARM) | GB(GB_FLY))) printf("restarted (reason %d) with%s%s on: kept as it was\n", (int)rr, gc.resume & GB(GB_ARM) ? " arm" : "", gc.resume & GB(GB_FLY) ? " fly" : "");
  printf("program: %s; %s\n", rn_error_text(e), G.why);
  hw_init();
  { char s[400]; show(s, sizeof s); printf("wiring: %s\n", s); }
  if (C.tx == 16 || C.tx == 17 || C.rx == 16 || C.rx == 17) printf("(on a WROVER board, with PSRAM, GPIO 16 and 17 are the PSRAM's: there, set tx= to other pins)\n");
  printf("type \"show\", \"set key=value\", \"save\", \"reboot\", or a command (status, goto X Y Z, calibrate…)\n");
  if (one_wire) printf("one wire to the module (GPIO %d): inverted, half duplex, as a module bay's\n", C.tx);
  uart_driver_install(CONSOLE, 1024, 1024, 0, NULL, 0); uart_vfs_dev_use_driver(CONSOLE);

  TickType_t wake = xTaskGetTickCount(); int64_t t0 = esp_timer_get_time(), last = t0;
  char line[200]; int ln = 0; uint32_t msgs = 0, dropped_was = 0; int alert_was = 0, why_was = 0; double next_status = 1;
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
      if (!strncmp(line, "set ", 4)) { char err[160]; int r = setting(line + 4, err, sizeof err); snprintf(reply, sizeof reply, r ? "%s" : err[0] ? "ok (save, then reboot); but %s" : "ok (save, then reboot)%s", err); }
      else if (!strcmp(line, "show")) show(reply, sizeof reply);
      else if (!strcmp(line, "save")) snprintf(reply, sizeof reply, cfg_save() ? "couldn't save" : "saved");
      else if (!strcmp(line, "reboot") && drone_up()) snprintf(reply, sizeof reply, "the telemetry says the drone is armed or flying: the channels would stop for a second or two (it would fly home). reboot force to do it anyway");
      else if (!strcmp(line, "reboot") || !strcmp(line, "reboot force")) {
        if (line[6]) printf("rebooting; the switches stay as they are\n"); else keep.magic = 0;   /* (a plain reboot starts afresh: the switch warning) */
        fflush(stdout); esp_restart();
      }
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
    if (m) to_module(out, m);
    keep.on = G.on; keep.check = ~G.on; keep.magic = KEEP_MAGIC;
    /* the pilot: the drone's messages, the alerts on the buzzer and the LED */
    if (G.V.nmsg - msgs > GND_MSGS) msgs = G.V.nmsg - GND_MSGS;      /* (more came than the ring keeps: the newest) */
    for (; msgs < G.V.nmsg; msgs++) printf("drone: %s\n", G.V.msg[msgs % GND_MSGS].s);
    if (G.dropped != dropped_was) { dropped_was = G.dropped; printf("command module: %s\n", G.why); }
    int why, lvl = gnd_alert(&G, &why);
    if (lvl != alert_was || why != why_was) { printf("%s%s\n", lvl == 2 ? "ALARM: " : lvl ? "warning: " : "", lvl ? gnd_why_text[why] : "all fine again"); alert_was = lvl; why_was = why; }
    int ph = (int)(t * 1000);
    int beep = lvl == 2 ? (ph % 400) < 200 : lvl == 1 ? (ph % 2000) < 100 : 0;
    if (C.buzzer >= 0) gpio_set_level(C.buzzer, beep);
    if (C.led >= 0) gpio_set_level(C.led, lvl == 2 ? (ph % 200) < 100 : lvl == 1 ? (ph % 1000) < 500 : 1);
    if (t >= next_status && G.V.t_any < 0) { next_status = t + 5; printf("waiting for the drone's telemetry (type status for more)\n"); }
  }
}
