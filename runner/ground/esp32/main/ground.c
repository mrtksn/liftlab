/*
 * The command module on an ESP32: ground_core.c (the same code the simulator runs on the far side of its radio link,
 * and dfb_ground runs on a Mac or a Pi) with buttons, sticks, a buzzer and an LED wired to the ESP32, driving the
 * pilot's radio link (radio_link.h): an ExpressLRS transmitter module over CRSF, or the ESP32's own radio (ESP-NOW,
 * Wi-Fi), where it does the module's part itself (plink.h).
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
 * The radio link (show lists it, the secrets masked; save and reboot to use it):
 *   set radio=elrs,250,4  an ExpressLRS transmitter module on tx= (the default; its rate and ratio are the module's
 *                         own, set on it: this only says what it is)
 *   set radio=espnow,6    ESP-NOW on Wi-Fi channel 6 (1–13), ESP32 to ESP32: the drone set radio=espnow,6 too.
 *                         espnow,6,lr: Espressif's long-range mode (slower, further), at both ends or neither
 *   set radio=wifi,ap,6   Wi-Fi, UDP to the drone's port 14570. The command module always joins a network: wifi= (the
 *   set radio=wifi,sta    drone's own, radio=wifi,ap,CH on the drone, or one both join); the channel is the network's
 *   set bind=PHRASE       1–31 characters, as on the drone: it signs the packets. The default (liftlab) is everyone's
 *   set wifi=SSID,PASS    the network to join: the drone's LiftLab-XXXX (as it says at power-on). set wifi=SSID: the
 *                         drone's default password (the binding phrase if it has 8+ characters, else liftlab1)
 *   set drone=IP          the drone's address on it: 192.168.4.1 (the default) on the drone's own network
 *   set radio=serial,115200  a serial line on tx= (two pins: to the line's input, from its output) at that speed, the
 *                         drone's the same: a laser or LED and a photodiode, fibre transceivers, an infrared pair, a
 *                         radio modem in transparent mode, a wire. serial,57600,half: one way at a time (radio modems)
 *   radio                 the link now: connected, link quality, signal, packets (for ExpressLRS: just its name)
 * ESP-NOW with two ESP32s: the drone radio=espnow,6 and bind=YOUR PHRASE, this one the same; save, reboot both.
 *
 * A computer's transmitter module (a "dongle"): plugged into a computer's USB, this ESP32 is the transmitter module
 * for dfb_ground (or anything that speaks CRSF to one):
 *   dfb_ground --tx /dev/ttyUSB0 --baud 115200 --keys          (macOS: /dev/cu.usbserial-… or /dev/cu.SLAB_USBtoUART)
 * Set the link here first (radio=, bind=, wifi=, drone=; save, reboot), then start dfb_ground. Once CRSF frames (with
 * a good CRC) come in on the USB port, they go into the link (ESP-NOW, Wi-Fi, or the ExpressLRS module on tx=: passed
 * through, one frame a beat), and what the link hands back (the drone's telemetry, the link statistics) goes out on the
 * USB port as CRSF. While a computer drives it (a frame within the last second) the console prints nothing (its text
 * would land among the frames; the ESP-IDF log too), and this command module's own buttons and sticks don't go up. When
 * the computer stops, nothing goes up (after 0.25 s not even its last channels: the drone's failsafe takes over, as
 * with a module whose handset stopped), the console talks again, and its own buttons stay off the air until you type
 * "local" (refused while the telemetry says the drone flies: "local force"), which starts them as at power-on: arm and
 * fly off, held by the switch warning until seen off. Text commands work whenever no frames come; a restart that isn't
 * a power-on keeps it a dongle. ESP-NOW from a laptop: this ESP32 radio=espnow,6 as the dongle, the drone the same.
 * (Flying from a laptop over Wi-Fi needs no ESP32 here: dfb_ground --radio wifi, see flight.c.)
 * Pins: not 1 and 3 (the USB port's serial: this console), 6–11 (the flash); 34–39 are inputs only. 0, 2, 5, 12 and
 * 15 are read at power-on to choose how the ESP32 boots: taken with a warning (nothing may pull them then; a button
 * on 0 held at power-on starts the bootloader). On a WROVER board (with PSRAM) 16 and 17 are the PSRAM's, the
 * default tx=17,16 among them: pick other pins there.
 * A restart that isn't a power-on (a crash, the watchdog, reboot force) keeps the switches as they were, in memory
 * that survives it, so the switch warning (ground_core.h) doesn't turn arm and fly off in flight (a dongle stays one).
 * And the text commands (ground_text.h) from a script or a terminal on the USB port: press/release/tap NAME,
 * stick AXIS V, goto X Y Z [HEADING], calibrate, latch N open|close, cmd ID V…, status, messages. A program with edited formulas is
 * not loaded here yet: the ESP32 runs its built-in one (dfb_ground --program takes one).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stddef.h>
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
#include "esp_log.h"
#include "ground_core.h"
#include "ground_text.h"
#include "esp_board.h"
#include "radio_module.h"
#include "esp_radio.h"
#include "radio_cfg.h"
#include "usb_split.h"

extern const uint8_t *const rn_builtin_ground_img;
extern const uint32_t rn_builtin_ground_len;
#define CONSOLE UART_NUM_0

/* The console: quiet while a computer drives this as its transmitter module (its frames on the USB port); printf's
 * and the ESP-IDF log's text would land among them. */
static volatile int quiet;
static void con(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void con(const char *fmt, ...) { if (quiet) return; va_list a; va_start(a, fmt); vprintf(fmt, a); va_end(a); }
static int log_out(const char *fmt, va_list a) { return quiet ? 0 : vprintf(fmt, a); }
static void radio_say(const char *s) { con("%s\n", s); }

/* ── the wiring ── */
#define CFG_VERSION 3
typedef struct {
  uint32_t version;
  int8_t tx, rx; int32_t baud;
  int8_t btn[GB_N];
  int8_t ax[GND_AXES]; uint8_t ax_inv[GND_AXES]; int16_t span;
  int8_t buzzer, led; uint32_t latch;
  /* v2 (a v1 blob is the part above; the rest takes its defaults): the radio link and the packet links' settings */
  int8_t radio_kind, radio_channel, radio_opt, radio_pad;   /* (as the flight firmware's hw_config) */
  int16_t elrs_rate, elrs_ratio;
  char bind[RCFG_BIND_N], wifi_ssid[RCFG_SSID_N], wifi_pass[RCFG_PASS_N], drone[RCFG_IP_N];
  /* v3 (a v2 blob is the part above): a serial line's speed (radio=serial,BAUD; radio_opt: half) */
  int32_t radio_baud;
} gcfg;
static gcfg C, N;                  /* C: the wiring running now; N: as set since (saved, it runs after a reboot) */
static void defaults(gcfg *c) {
  memset(c, 0, sizeof *c); c->version = CFG_VERSION; c->tx = LB_TX; c->rx = LB_RX; c->baud = 400000;
  for (int i = 0; i < GB_N; i++) c->btn[i] = -1;
  for (int a = 0; a < GND_AXES; a++) c->ax[a] = -1;
  c->span = 1800; c->buzzer = -1; c->led = LB_LED; c->latch = GB(GB_ARM) | GB(GB_FLY);
  c->radio_kind = RLINK_ELRS; c->radio_channel = 1; c->elrs_rate = 250; c->elrs_ratio = 4; c->radio_baud = 115200;
  strcpy(c->bind, RCFG_BIND_DEFAULT); strcpy(c->drone, RCFG_DRONE_DEFAULT);
}
static int cfg_radio(const gcfg *c, rlink_cfg *L) {   /* the link as set: 0, or −1 (L: the default) */
  int e = c->radio_kind == RLINK_ESPNOW ? rlink_make(L, RLINK_ESPNOW, c->radio_channel, c->radio_opt)
        : c->radio_kind == RLINK_WIFI ? rlink_make(L, RLINK_WIFI, c->radio_opt, c->radio_channel)
        : c->radio_kind == RLINK_SERIAL ? rlink_make(L, RLINK_SERIAL, (int)c->radio_baud, c->radio_opt)
        : c->radio_kind == RLINK_ELRS ? rlink_make(L, RLINK_ELRS, c->elrs_rate, c->elrs_ratio) : -1;
  if (e) rlink_default(L);
  return e;
}
static void cfg_fix(gcfg *c);
static void cfg_load(void) {
  defaults(&C); nvs_handle_t h;
  if (nvs_open("dfbg", NVS_READONLY, &h) == ESP_OK) {
    gcfg t; size_t n = sizeof t;
    if (nvs_get_blob(h, "cfg", &t, &n) == ESP_OK) {
      if (n == sizeof t && t.version == CFG_VERSION) C = t;
      else if (t.version == 2 && n == offsetof(gcfg, radio_baud)) { memcpy(&C, &t, n); C.version = CFG_VERSION; }   /* (v2: a serial line's speed takes its default) */
      else if (t.version == 1 && n == offsetof(gcfg, radio_kind)) { memcpy(&C, &t, n); C.version = CFG_VERSION; }   /* (v1: the radio's settings take their defaults) */
    }
    nvs_close(h);
  }
  cfg_fix(&C); N = C;
}
static int cfg_save(void) {
  nvs_handle_t h; if (nvs_open("dfbg", NVS_READWRITE, &h) != ESP_OK) return -1;
  esp_err_t e = nvs_set_blob(h, "cfg", &N, sizeof N); if (e == ESP_OK) e = nvs_commit(h); nvs_close(h); return e == ESP_OK ? 0 : -1;
}
/* pins: 1 and 3 are this console's; 6–11 the flash's; 20, 24, 28–31 aren't there; 34–39 only read */
static int pin_in(int p) {
#if CONFIG_IDF_TARGET_ESP32
  return p == -1 || (p >= 0 && p <= 39 && p != 1 && p != 3 && !(p >= 6 && p <= 11) && p != 20 && p != 24 && !(p >= 28 && p <= 31));
#else
  return p == -1 || lb_input_pin(p);
#endif
}
static int pin_out(int p) { return pin_in(p) && (p == -1 || GPIO_IS_VALID_OUTPUT_GPIO(p)); }
static int pin_crsf(int p) {
#if CONFIG_IDF_TARGET_ESP32
  return pin_out(p) && p >= 0 && p != 12;
#else
  return lb_output_pin(p);
#endif
}
static const char *pin_note(int p) {
#if CONFIG_IDF_TARGET_ESP32
  return p == 0 || p == 2 || p == 5 || p == 12 || p == 15 ? "a strapping pin: do not pull it during boot"
       : p == 16 || p == 17 ? "on a WROVER board these pins belong to PSRAM" : 0;
#else
  (void)p; return 0;
#endif
}
/* a wiring saved by an older build that is refused now: that part back to its default, said */
static void cfg_fix(gcfg *c) {
  gcfg d; defaults(&d); int bad = 0;
  if (!pin_out(c->tx) || !pin_in(c->rx) || c->tx < 0 || c->rx < 0 || !pin_crsf(c->tx) || !pin_crsf(c->rx) || (c->tx == c->rx && !pin_out(c->rx))) { c->tx = d.tx; c->rx = d.rx; bad = 1; }
  for (int a = 0; a < GND_AXES; a++) if (c->ax[a] >= 0 && !lb_adc_pin(c->ax[a])) { c->ax[a] = -1; bad = 1; }
  for (int b = 0; b < GB_N; b++) if (!pin_in(c->btn[b])) { c->btn[b] = -1; bad = 1; }
  if (!pin_out(c->buzzer)) { c->buzzer = -1; bad = 1; }
  if (!pin_out(c->led)) { c->led = -1; bad = 1; }
  rlink_cfg L; char t[RCFG_BIND_N];
  if (cfg_radio(c, &L)) { c->radio_kind = d.radio_kind; c->radio_channel = d.radio_channel; c->radio_opt = d.radio_opt; c->elrs_rate = d.elrs_rate; c->elrs_ratio = d.elrs_ratio; c->radio_baud = d.radio_baud; bad = 1; }
  if (L.kind == RLINK_SERIAL && c->tx == c->rx) { c->radio_kind = d.radio_kind; bad = 1; }   /* (a serial line needs two pins) */
  if (!rcfg_terminated(c->bind, sizeof c->bind) || rcfg_bind_parse(t, c->bind, 0, 0)) { strcpy(c->bind, d.bind); bad = 1; }
  if (!rcfg_terminated(c->wifi_ssid, sizeof c->wifi_ssid) || !rcfg_terminated(c->wifi_pass, sizeof c->wifi_pass)) { c->wifi_ssid[0] = c->wifi_pass[0] = 0; bad = 1; }
  if (!rcfg_terminated(c->drone, sizeof c->drone) || rcfg_ip_parse(0, 0, c->drone, 0, 0)) { strcpy(c->drone, d.drone); bad = 1; }
  if (bad) printf("the saved wiring had reserved or unavailable pins (or settings): those are back to their defaults\n");
}
/* The wiring as set (N); what runs (C) changes only at the next power-on, so a half-done rewiring never flies. */
static void show(char *o, int n) {
  int k = memcmp(&N, &C, sizeof N) ? snprintf(o, (size_t)n, "(changed since power-on: runs after save and reboot) ") : 0;
  k += snprintf(o + k, (size_t)(n - k), "tx=%d,%d baud=%ld span=%d buzzer=%d led=%d latch=", N.tx, N.rx, (long)N.baud, N.span, N.buzzer, N.led);
  for (int b = 0; b < GB_N && k < n - 20; b++) if (N.latch & GB(b)) k += snprintf(o + k, (size_t)(n - k), "%s,", gnd_button_names[b]);
  for (int b = 0; b < GB_N && k < n - 20; b++) if (N.btn[b] >= 0) k += snprintf(o + k, (size_t)(n - k), " %s=%d", gnd_button_names[b], N.btn[b]);
  static const char *const ax[4] = { "roll", "pitch", "throttle", "yaw" };
  for (int a = 0; a < GND_AXES && k < n - 20; a++) if (N.ax[a] >= 0) k += snprintf(o + k, (size_t)(n - k), " %s=%d%s", ax[a], N.ax[a], N.ax_inv[a] ? "i" : "");
  rlink_cfg L; cfg_radio(&N, &L); char rl[32]; rlink_describe(&L, rl, sizeof rl);   /* the radio; the secrets masked */
  if (k < n - 1) k += snprintf(o + k, (size_t)(n - k), " radio=%s", rl);
  if (k < n - 1) k += rcfg_bind_default(N.bind) ? snprintf(o + k, (size_t)(n - k), " bind=%s(the default: set your own)", RCFG_BIND_DEFAULT)
                                                : snprintf(o + k, (size_t)(n - k), " bind=(set, %d characters)", (int)strlen(N.bind));
  if (k < n - 1) k += N.wifi_ssid[0] ? snprintf(o + k, (size_t)(n - k), " wifi=%s,%s", N.wifi_ssid, N.wifi_pass[0] ? "********" : "(default password)")
                                     : snprintf(o + k, (size_t)(n - k), " wifi=(none)");
  if (k < n - 1) snprintf(o + k, (size_t)(n - k), " drone=%s", N.drone);
}
/* "set key=value" into N: 0 (err: a warning, or empty), or −1 with why in err */
static int setting(char *kv, char *err, int en) {
  char *e = strchr(kv, '='); err[0] = 0; if (!e) { snprintf(err, (size_t)en, "set key=value"); return -1; }
  *e = 0; const char *k = kv, *v = e + 1; int x = atoi(v);
  static const char *const ax[4] = { "roll", "pitch", "throttle", "yaw" };
  if (!strcmp(k, "tx")) {
    int a = -1, b = -1; if (sscanf(v, "%d,%d", &a, &b) != 2 || !pin_out(a) || !pin_in(b) || a < 0 || b < 0 || !pin_crsf(a) || !pin_crsf(b)) { snprintf(err, (size_t)en, "tx=OUT,IN: available output pins (one pin for both is fine)"); return -1; }
    N.tx = (int8_t)a; N.rx = (int8_t)b; x = pin_note(a) ? a : b;
  }
  else if (!strcmp(k, "baud")) { if (x < 9600 || x > 5250000) { snprintf(err, (size_t)en, "baud: 9600 to 5250000"); return -1; } N.baud = x; }
  else if (!strcmp(k, "span")) { if (x < 100 || x > 2047) { snprintf(err, (size_t)en, "span: 100 to 2047"); return -1; } N.span = (int16_t)x; }
  else if (!strcmp(k, "buzzer") || !strcmp(k, "led")) { if (!pin_out(x)) { snprintf(err, (size_t)en, "%.20s: an available output pin, or -1", k); return -1; } if (k[0] == 'b') N.buzzer = (int8_t)x; else N.led = (int8_t)x; }
  else if (!strcmp(k, "radio")) {                     /* the radio link: elrs,250,4 | espnow,CH[,lr] | wifi,ap,CH | wifi,sta | serial,BAUD[,half] */
    rlink_cfg L; if (rlink_parse(&L, v, err, en)) return -1;
    if (L.kind == RLINK_SERIAL && N.tx == N.rx) { snprintf(err, (size_t)en, "radio=serial: a serial line needs two pins: set tx=OUT,IN first (to the line's input, from its output)"); return -1; }
    N.radio_kind = (int8_t)L.kind;
    if (L.kind == RLINK_ELRS) { N.elrs_rate = (int16_t)L.rate_hz; N.elrs_ratio = (int16_t)L.ratio; }
    else if (L.kind == RLINK_ESPNOW) { N.radio_channel = (int8_t)L.channel; N.radio_opt = (int8_t)L.lr; }
    else if (L.kind == RLINK_SERIAL) { N.radio_baud = L.baud; N.radio_opt = (int8_t)L.half; }
    else { N.radio_opt = (int8_t)L.sta; if (!L.sta) N.radio_channel = (int8_t)L.channel; if (!N.wifi_ssid[0]) snprintf(err, (size_t)en, "set wifi=SSID[,PASSWORD] too: the network to join (the drone's: LiftLab-XXXX)"); }
    return 0;
  }
  else if (!strcmp(k, "bind")) return rcfg_bind_parse(N.bind, v, err, en);
  else if (!strcmp(k, "wifi")) return rcfg_wifi_parse(N.wifi_ssid, N.wifi_pass, v, err, en);
  else if (!strcmp(k, "drone")) return rcfg_ip_parse(N.drone, 0, v, err, en);
  else if (!strcmp(k, "latch")) { N.latch = 0; char buf[128]; snprintf(buf, sizeof buf, "%s", v); char *sv; for (char *p = strtok_r(buf, ",", &sv); p; p = strtok_r(0, ",", &sv)) { int b = gnd_button(p); if (b >= 0) N.latch |= GB(b); } }
  else {
    for (int a = 0; a < GND_AXES; a++) if (!strcmp(k, ax[a])) {
      if (x != -1 && !lb_adc_pin(x)) { snprintf(err, (size_t)en, "%.20s: an available ADC1 pin (or -1)", k); return -1; }
      N.ax[a] = (int8_t)x; N.ax_inv[a] = strchr(v, 'i') != 0; return 0;
    }
    int b = gnd_button(k); if (b < 0) { snprintf(err, (size_t)en, "no setting %.20s", k); return -1; }
    if (!pin_in(x)) { snprintf(err, (size_t)en, "%.20s: reserved or unavailable on this chip", k); return -1; }
    N.btn[b] = (int8_t)x;
  }
  if (strcmp(k, "latch") && strcmp(k, "baud") && strcmp(k, "span") && x >= 0 && pin_note(x)) snprintf(err, (size_t)en, "GPIO %d is %s", x, pin_note(x));
  return 0;
}

/* ── the hardware ── */
static radio_io *RADIO;
/* no radio (one that didn't start): nothing comes, what's written goes nowhere; the console still works */
static int none_read(radio_io *R, uint8_t *b, int n, int w) { (void)R; (void)b; (void)n; (void)w; return 0; }
static int none_write(radio_io *R, const uint8_t *b, int n) { (void)R; (void)b; return n; }
static radio_io no_radio = { "no radio (it didn't start: see above)", none_read, none_write, -1, 0 };
static adc_oneshot_unit_handle_t adc; static adc_channel_t ax_ch[GND_AXES]; static int ax_ok[GND_AXES]; static float ax_mid[GND_AXES];
static void hw_init(void) {
  for (int b = 0; b < GB_N; b++) if (C.btn[b] >= 0) {
    gpio_config_t g = { .pin_bit_mask = 1ULL << C.btn[b], .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_IS_VALID_OUTPUT_GPIO(C.btn[b]) ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE };
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
  rlink_cfg L; cfg_radio(&C, &L);                       /* the pilot's radio link (radio_io.h) */
  if (L.kind == RLINK_ELRS) RADIO = radio_module_start(C.tx, C.rx, (int)C.baud);
  else {
    if (rcfg_bind_default(C.bind)) printf("WARNING: the binding phrase is the default (liftlab): anyone who knows it can fly your drone. set bind=YOUR PHRASE (the same on the drone)\n");
    RADIO = L.kind == RLINK_ESPNOW ? radio_espnow_start(&L, PLINK_GROUND, C.bind, radio_say)
          : L.kind == RLINK_SERIAL ? radio_uart_start(&L, PLINK_GROUND, C.bind, LB_RADIO_UART, C.tx, C.rx, radio_say)   /* (the module's pins: the line's) */
          : radio_wifi_start(&L, PLINK_GROUND, C.bind, C.wifi_ssid, C.wifi_pass, C.drone, radio_say);
  }
  if (!RADIO) RADIO = &no_radio;
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
static float arenas_[3][2048], pools_[3][1024]; static rn_host H;   /* (the program slots' steps on the heap: static DRAM is short with Wi-Fi) */
static gnd_state G; static gnd_text_in TI;
/* the switches as sent, kept where a restart that isn't a power-on doesn't clear them (check: ~(on ^ dongle), so
 * half-written or power-on noise doesn't count); and whether a computer was driving it (it stays a dongle) */
#define KEEP_MAGIC 0x6466626Cu
static RTC_NOINIT_ATTR struct { uint32_t magic, on, dongle, check; } keep;
/* the telemetry's last word on the drone: armed (or in its failsafe), or flying */
static int drone_up(void) {
  const gnd_view *V = &G.V; int st = V->item[TLM_STATE].t >= 0 && V->item[TLM_STATE].n >= 1 ? (int)V->item[TLM_STATE].v[0] : 0;
  int nav = V->item[TLM_NAV].t >= 0 && V->item[TLM_NAV].n >= 5 ? (int)V->item[TLM_NAV].v[4] : 0;
  return st == 1 || st == 2 || (nav & 2);
}

/* ── the USB port: text, and a computer's CRSF frames (usb_split.h) ── */
static usb_split US;
static int dongle;                 /* a computer has driven it: its own buttons stay off the air until "local" */
static gnd_config GC;              /* (the core's settings: "local" starts it again with them) */
/* the computer's frames, to go up one a beat: the newest channels, and the others (commands) in order, first */
#define BQ 8
static uint8_t bq[BQ][CRSF_MAX_FRAME]; static int bq_len[BQ], bq_h, bq_n;
static uint8_t brc[CRSF_MAX_FRAME]; static int brc_len;
static double t_line;              /* (the time, for the text commands) */
static void usb_frame(void *ctx, const uint8_t *f, int n) {
  (void)ctx;
  if (f[2] == CRSF_RC) { memcpy(brc, f, (size_t)n); brc_len = n; }
  else if (bq_n < BQ) { int i = (bq_h + bq_n) % BQ; memcpy(bq[i], f, (size_t)n); bq_len[i] = n; bq_n++; }
}
static void usb_line(void *ctx, char *line) {
  (void)ctx; double t = t_line; char reply[700] = "";
  if (!strncmp(line, "set ", 4)) { char err[160]; int r = setting(line + 4, err, sizeof err); snprintf(reply, sizeof reply, r ? "%s" : err[0] ? "ok (save, then reboot); but %s" : "ok (save, then reboot)%s", err); }
  else if (!strcmp(line, "show")) show(reply, sizeof reply);
  else if (!strcmp(line, "save")) snprintf(reply, sizeof reply, cfg_save() ? "couldn't save" : "saved");
  else if (!strcmp(line, "radio")) esp_radio_status(RADIO, reply, sizeof reply);
  else if (!strcmp(line, "reboot") && drone_up()) snprintf(reply, sizeof reply, "the telemetry says the drone is armed or flying: the channels would stop for a second or two (it would fly home). reboot force to do it anyway");
  else if (!strcmp(line, "reboot") || !strcmp(line, "reboot force")) {
    if (line[6]) con("rebooting; the switches stay as they are\n"); else keep.magic = 0;   /* (a plain reboot starts afresh: the switch warning) */
    fflush(stdout); esp_restart();
  }
  else if (!strcmp(line, "local") || !strcmp(line, "local force")) {
    if (!dongle) snprintf(reply, sizeof reply, "this command module's own buttons fly already");
    else if (usb_split_frames_now(&US, t)) snprintf(reply, sizeof reply, "a computer is sending frames: stop it first");
    else if (!line[5] && drone_up()) snprintf(reply, sizeof reply, "the telemetry says the drone is armed or flying: the buttons start as at power-on (arm off: it would disarm). local force to do it anyway");
    else {
      gnd_config c = GC; c.resume = 0; c.seq0 = (uint8_t)esp_random(); gnd_init(&G, G.H, &c);   /* (as at power-on: the switch warning) */
      memset(&TI, 0, sizeof TI); dongle = 0; bq_n = 0; brc_len = 0;
      snprintf(reply, sizeof reply, "this command module's own buttons and sticks fly now (arm and fly start off)");
    }
  }
  else if (!gnd_text(&G, &TI, line, t, reply, sizeof reply)) snprintf(reply, sizeof reply, "unknown (show, set, save, reboot, radio, local, press, release, tap, stick, goto, calibrate, latch, cmd, status, messages)");
  if (reply[0]) con("%s\n", reply);
}

void app_main(void) {
  esp_log_set_vprintf(log_out);                                      /* (the log too is quiet while a computer drives it) */
  if (nvs_flash_init() != ESP_OK) { nvs_flash_erase(); nvs_flash_init(); }
  cfg_load();
  printf("\nLiftLab command module (ESP32)\n");
  float *ar[3] = { arenas_[0], arenas_[1], arenas_[2] }, *po[3] = { pools_[0], pools_[1], pools_[2] }; int32_t *co[3]; for (int i = 0; i < 3; i++) co[i] = calloc(2048, sizeof(int32_t));
  int e = co[0] && co[1] && co[2] ? rn_host_init(&H, rn_builtin_ground_img, rn_builtin_ground_len, ar, 2048, co, 2048, po, 1024) : RN_E_TOO_BIG;
  gnd_config gc; gnd_config_default(&gc); gc.latch = C.latch; GC = gc; gc.seq0 = (uint8_t)esp_random();   /* (gnd_config.seq0) */
  esp_reset_reason_t rr = esp_reset_reason();
  if (rr != ESP_RST_POWERON && keep.magic == KEEP_MAGIC && keep.check == ~(keep.on ^ keep.dongle)) {   /* (a restart, maybe in flight) */
    gc.resume = keep.on; dongle = keep.dongle == 1;
  }
  keep.magic = 0;
  gnd_init(&G, e ? 0 : &H, &gc);
  /* a switch only text set (no button, not latching) is held by nothing else: set again */
  for (int b = GB_ARM; b <= GB_FLY; b++) if ((gc.resume & GB(b)) && !(C.latch & GB(b)) && C.btn[b] < 0) TI.held |= GB(b);
  if (gc.resume & (GB(GB_ARM) | GB(GB_FLY))) printf("restarted (reason %d) with%s%s on: kept as it was\n", (int)rr, gc.resume & GB(GB_ARM) ? " arm" : "", gc.resume & GB(GB_FLY) ? " fly" : "");
  if (dongle) printf("restarted (reason %d) while a computer drove it: its own buttons stay off the air (type local)\n", (int)rr);
  printf("program: %s; %s\n", rn_error_text(e), G.why);
  hw_init();
  { char s[700]; show(s, sizeof s); printf("wiring: %s\n", s); }
  printf("radio: %s\n", RADIO->name);
  if (pin_note(C.tx) || pin_note(C.rx)) printf("(on a WROVER board, with PSRAM, GPIO 16 and 17 are the PSRAM's: there, set tx= to other pins)\n");
  printf("type \"show\", \"set key=value\", \"save\", \"reboot\", \"radio\", or a command (status, goto X Y Z, calibrate…)\n");
  printf("a computer's transmitter module: dfb_ground --tx THIS_PORT --baud 115200 (see ground.c)\n");
  if (C.tx == C.rx && RADIO != &no_radio && !strncmp(RADIO->name, "ExpressLRS", 10)) printf("one wire to the module (GPIO %d): inverted, half duplex, as a module bay's\n", C.tx);
  uart_driver_install(CONSOLE, 1024, 4096, 0, NULL, 0); uart_vfs_dev_use_driver(CONSOLE);
  usb_split_init(&US); const usb_split_out uo = { usb_frame, usb_line, 0 };

  TickType_t wake = xTaskGetTickCount(); int64_t t0 = esp_timer_get_time(), last = t0;
  uint32_t msgs = 0, dropped_was = 0; int alert_was = 0, why_was = 0, bridged = 0; double next_status = 1;
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(4));
    int64_t now = esp_timer_get_time(); double t = (now - t0) * 1e-6; float dt = (float)((now - last) * 1e-6); last = now;
    /* the USB port: settings and text commands, or a computer's frames */
    uint8_t b[256]; int n; t_line = t;
    while ((n = uart_read_bytes(CONSOLE, b, sizeof b, 0)) > 0) usb_split_feed(&US, b, n, t, &uo);
    usb_split_feed(&US, b, 0, t, &uo);                               /* (a frame that stopped coming: given up) */
    int bridging = usb_split_frames_now(&US, t);
    if (bridging) dongle = 1;
    quiet = bridging;
    if (bridged && !bridging) con("the computer stopped sending frames: nothing goes up now (the drone's failsafe flies it). Type local for this command module's own buttons\n");
    bridged = bridging;
    /* what the link hands back: to the core (the alerts), and to the computer while it drives this */
    n = RADIO->read(RADIO, b, sizeof b, 0);
    while (n > 0) {
      gnd_from_radio(&G, b, n, t);
      if (bridging) uart_write_bytes(CONSOLE, b, (size_t)n);
      n = RADIO->read(RADIO, b, sizeof b, 0);
    }
    /* a step */
    rn_host_tick(&H, dt);
    gnd_input in; read_inputs(&in); gnd_text_inputs(&TI, t, &in);
    uint8_t out[128]; int m = gnd_step(&G, &in, t, dt, out, sizeof out);
    if (dongle) {                                                     /* the computer's frames, one a beat (a module bay's one wire answers each) */
      if (bq_n) { RADIO->write(RADIO, bq[bq_h], bq_len[bq_h]); bq_h = (bq_h + 1) % BQ; bq_n--; }
      else if (brc_len) { RADIO->write(RADIO, brc, brc_len); brc_len = 0; }
    } else if (m) RADIO->write(RADIO, out, m);
    keep.on = G.on; keep.dongle = (uint32_t)dongle; keep.check = ~(G.on ^ keep.dongle); keep.magic = KEEP_MAGIC;
    /* the pilot: the drone's messages, the alerts on the buzzer and the LED (the console's part not while bridging) */
    if (G.V.nmsg - msgs > GND_MSGS || quiet) msgs = quiet ? G.V.nmsg : G.V.nmsg - GND_MSGS;   /* (more came than the ring keeps: the newest) */
    for (; msgs < G.V.nmsg; msgs++) con("drone: %s\n", G.V.msg[msgs % GND_MSGS].s);
    if (G.dropped != dropped_was) { dropped_was = G.dropped; if (!dongle) con("command module: %s\n", G.why); }
    int why, lvl = gnd_alert(&G, &why);
    if (lvl != alert_was || why != why_was) { con("%s%s\n", lvl == 2 ? "ALARM: " : lvl ? "warning: " : "", lvl ? gnd_why_text[why] : "all fine again"); alert_was = lvl; why_was = why; }
    int ph = (int)(t * 1000);
    int beep = lvl == 2 ? (ph % 400) < 200 : lvl == 1 ? (ph % 2000) < 100 : 0;
    if (C.buzzer >= 0) gpio_set_level(C.buzzer, beep);
    if (C.led >= 0) gpio_set_level(C.led, lvl == 2 ? (ph % 200) < 100 : lvl == 1 ? (ph % 1000) < 500 : 1);
    if (t >= next_status && G.V.t_any < 0) { next_status = t + 5; con("waiting for the drone's telemetry (type status, or radio, for more)\n"); }
  }
}
