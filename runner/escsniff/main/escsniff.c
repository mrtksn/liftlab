/*
 * LiftLab: ESC signal sniffer for an ESP32-S3. It only listens: four inputs, nothing drives a motor.
 *
 * Tap a drone's motor-signal lines (e.g. the toy flight board's wire to each arm ESC) and it tells you what they
 * carry: servo-style PWM, OneShot, duty PWM, DShot (decoded) or UART serial (decoded), with the numbers.
 *
 *   CH1 = GPIO4   CH2 = GPIO5   CH3 = GPIO6   CH4 = GPIO7    protected 3.3 V inputs; GND to the drone's GND
 *   GEN = GPIO8   a test signal of your choosing (gen), to wire to a CH input and check the sniffer itself
 *
 * A series resistor alone does NOT make a 4.2 V signal safe. Use the 3.3 V-powered, overvoltage-tolerant input
 * buffer described in docs/esc-sniffer.md. Never connect battery (+) to a GPIO. Power the S3 from USB only.
 *
 * Commands (serial, 115200 baud, either USB port; end each with Enter):
 *   Enter (or s)       one capture of all four lines, described
 *   w                  watch: a capture every 300 ms until you press Enter
 *   raw <1-4> [n]      the first n (default 48) high/low stretches on a line, in microseconds
 *   csv [n]            every change (up to n, default 2000) as time_us,ch4ch3ch2ch1 to paste somewhere
 *   win <10-200>       capture window in ms (default 60; servo PWM at 50 Hz needs at least 45)
 *   gen <hz> <high_us> test signal on GPIO8, e.g. gen 50 1500 (servo), gen 8000 30 (duty PWM); gen off
 *
 * How: a task pinned to core 1 polls the four inputs through the S3's dedicated-GPIO port with interrupts masked and
 * stamps each change with the CPU cycle counter (about 20 cycles a sample at 240 MHz: ~0.1 us resolution, enough for
 * DShot600). The analysis (sniff_core.c) runs afterwards.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/dedic_gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_cpu.h"
#include "esp_err.h"
#include "sniff_core.h"
#include "sniff_console.h"

/* replies go to both USB ports (the UART bridge and the native USB), whichever one you're on */
static int out_ready;
static void say(const char *fmt, ...) {
  char b[400]; va_list ap; va_start(ap, fmt); int n = vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof b - 1) n = sizeof b - 1;
  if (!out_ready) { fwrite(b, 1, (size_t)n, stdout); return; }
  char c[820]; int k = 0;                       /* terminals want \r\n */
  for (int i = 0; i < n && k < (int)sizeof c - 2; i++) { if (b[i] == '\n') c[k++] = '\r'; c[k++] = b[i]; }
  uart_write_bytes(UART_NUM_0, c, (size_t)k);
  usb_serial_jtag_write_bytes(c, (size_t)k, pdMS_TO_TICKS(100));
}

static const int PINS[SNIFF_CH] = { 4, 5, 6, 7 };
#define GEN_PIN 8
#define CAP 12000
#define CPU_MHZ CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ

static uint32_t T[CAP], SA[CAP + 2], SB[CAP + 2];
static uint8_t LV[CAP];
static sniff_cap cap = { T, LV, 0, 0, 0, 0, (float)CPU_MHZ };
static uint32_t loops;
static int win_ms = 60, full;
static TaskHandle_t cap_h;
static SemaphoreHandle_t cap_done;

static void capture(dedic_gpio_bundle_handle_t bundle) {
  uint32_t win = (uint32_t)win_ms * 1000u * CPU_MHZ, n = 0, k = 0;
  UBaseType_t s = portSET_INTERRUPT_MASK_FROM_ISR();
  uint32_t v = dedic_gpio_bundle_read_in(bundle), v0 = v, t0 = esp_cpu_get_cycle_count(), now = t0;
  while (now - t0 < win) {
    uint32_t x = dedic_gpio_bundle_read_in(bundle);
    now = esp_cpu_get_cycle_count(); k++;
    if (x != v) { T[n] = now; LV[n] = (uint8_t)x; v = x; if (++n >= CAP) break; }
  }
  portCLEAR_INTERRUPT_MASK_FROM_ISR(s);
  cap.lv0 = (uint8_t)v0; cap.n = (int)n; cap.t0 = t0; cap.t_end = now; loops = k; full = n >= CAP;
}

static void cap_task(void *arg) {
  gpio_config_t io = { .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_ENABLE, .intr_type = GPIO_INTR_DISABLE };
  for (int i = 0; i < SNIFF_CH; i++) io.pin_bit_mask |= 1ULL << PINS[i];
  ESP_ERROR_CHECK(gpio_config(&io));
  dedic_gpio_bundle_handle_t bundle = NULL;
  dedic_gpio_bundle_config_t bc = { .gpio_array = PINS, .array_size = SNIFF_CH, .flags = { .in_en = 1 } };
  ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bc, &bundle));   /* the port belongs to this core: read it only here */
  for (int i = 0; i < SNIFF_CH; i++) gpio_pulldown_en(PINS[i]);
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    capture(bundle);
    xSemaphoreGive(cap_done);
  }
}

static void grab(void) { xTaskNotifyGive(cap_h); xSemaphoreTake(cap_done, portMAX_DELAY); }

static void report(int compact) {
  if (!compact)
    say("capture: %.0f ms, %d changes%s, a sample every %.0f ns\n", (cap.t_end - cap.t0) / (float)CPU_MHZ / 1000.f, cap.n,
        full ? " (buffer FULL: the window was cut short)" : "", loops ? (cap.t_end - cap.t0) * 1000.f / CPU_MHZ / (float)loops : 0.f);
  for (int ch = 0; ch < SNIFF_CH; ch++) {
    sniff_res r; char line[300];
    sniff_analyze(&cap, ch, SA, SB, &r);
    sniff_line(&r, line, sizeof line);
    say("CH%d GPIO%d  %s\n", ch + 1, PINS[ch], line);
  }
  if (compact) say("\n");
}

static void raw(int ch, int max) {
  float tpu = (float)CPU_MHZ; int lvl = (cap.lv0 >> ch) & 1, shown = 0; uint32_t at = cap.t0;
  say("CH%d starts %s; stretches in us (H high, L low), the first one cut by the window:\n", ch + 1, lvl ? "HIGH" : "low");
  char b[400]; int n = 0;
  for (int i = 0; i < cap.n && shown < max; i++) {
    int x = (cap.lv[i] >> ch) & 1;
    if (x == lvl) continue;
    n += snprintf(b + n, sizeof b - (size_t)n, "%c%.2f ", lvl ? 'H' : 'L', (cap.t[i] - at) / tpu);
    at = cap.t[i]; lvl = x; shown++;
    if (n > 300) { say("%s\n", b); n = 0; }
  }
  if (n) say("%s\n", b);
  if (!shown) say("(no changes)\n");
}

static void csv(int max) {
  say("time_us,ch4ch3ch2ch1\n0.000,%d%d%d%d\n", (cap.lv0 >> 3) & 1, (cap.lv0 >> 2) & 1, (cap.lv0 >> 1) & 1, cap.lv0 & 1);
  for (int i = 0; i < cap.n && i < max; i++) {
    uint8_t v = cap.lv[i];
    say("%.3f,%d%d%d%d\n", (cap.t[i] - cap.t0) / (float)CPU_MHZ, (v >> 3) & 1, (v >> 2) & 1, (v >> 1) & 1, v & 1);
  }
  if (cap.n > max) say("(%d more; csv <n> for more)\n", cap.n - max);
}

static int gen_on;
static int gen(int hz, float high_us) {
  int bits = 14;
  while (bits > 1 && 80e6 / ((double)hz * (1 << bits)) < 1.0) bits--;
  ledc_timer_config_t t = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = (ledc_timer_bit_t)bits, .timer_num = LEDC_TIMER_0,
                            .freq_hz = (uint32_t)hz, .clk_cfg = LEDC_AUTO_CLK };
  if (ledc_timer_config(&t) != ESP_OK) return -1;
  double d = high_us * 1e-6 * hz * (1 << bits);
  uint32_t duty = d < 0 ? 0 : d > (1 << bits) ? (1u << bits) : (uint32_t)(d + 0.5);
  ledc_channel_config_t c = { .gpio_num = GEN_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
                              .timer_sel = LEDC_TIMER_0, .duty = duty, .hpoint = 0 };
  if (ledc_channel_config(&c) != ESP_OK) return -1;
  gen_on = 1;
  return bits;
}
static void gen_off(void) {
  if (gen_on) ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
  gen_on = 0;
}

static int watching;
static void command(char *s) {
  while (*s && isspace((unsigned char)*s)) s++;
  char *e = s + strlen(s); while (e > s && isspace((unsigned char)e[-1])) *--e = 0;
  int a = 0, b = 0; float f = 0;
  if (watching) { watching = 0; say("watch stopped\n"); if (!*s) return; }
  if (!*s || !strcmp(s, "s")) { grab(); report(0); }
  else if (!strcmp(s, "w")) { watching = 1; say("watching every 300 ms; Enter stops\n"); }
  else if (sscanf(s, "raw %d %d", &a, &b) >= 1) {
    if (a < 1 || a > SNIFF_CH) { say("raw <1-4> [n]\n"); return; }
    raw(a - 1, b > 0 ? b : 48);
  } else if (!strncmp(s, "csv", 3)) { int m = 2000; sscanf(s, "csv %d", &m); csv(m > 0 ? m : 2000); }
  else if (sscanf(s, "win %d", &a) == 1) {
    if (a < 10 || a > 200) { say("win <10-200> (ms)\n"); return; }
    win_ms = a; say("window %d ms\n", win_ms);
  } else if (!strcmp(s, "gen off")) { gen_off(); say("test signal off\n"); }
  else if (sscanf(s, "gen %d %f", &a, &f) == 2) {
    if (a < 1 || a > 1000000 || f < 0) { say("gen <hz> <high_us>, or gen off\n"); return; }
    int bits = gen(a, f);
    if (bits < 0) say("that signal isn't possible\n");
    else say("GPIO8: %d Hz, high %.2f us (%d-bit steps of %.3f us); wire it to a CH input\n", a, f, bits, 1e6 / a / (1 << bits));
  } else if (!strcmp(s, "?") || !strcmp(s, "help") || !strcmp(s, "h")) {
    say("Enter: capture | w: watch | raw <ch> [n] | csv [n] | win <ms> (now %d) | gen <hz> <high_us> | gen off\n", win_ms);
  } else say("unknown: %s  (type ? for help)\n", s);
}

/* lines from either USB port: the UART bridge ("UART"/"COM" port) and the native USB ("USB" port) */
static QueueHandle_t lines;
typedef struct { char t[SNIFF_LINE_SIZE]; } line_t;
static void feed(sniff_console *console, const uint8_t *in, int k) {
  for (int i = 0; i < k; i++) {
    line_t l;
    if (sniff_console_feed(console, in[i], l.t)) xQueueSend(lines, &l, 0);
  }
}
static void uart_task(void *arg) {
  sniff_console console = { 0 }; uint8_t in[64];
  for (;;) { int k = uart_read_bytes(UART_NUM_0, in, sizeof in, pdMS_TO_TICKS(50)); if (k > 0) feed(&console, in, k); }
}
static void usb_task(void *arg) {
  sniff_console console = { 0 }; uint8_t in[64];
  for (;;) { int k = usb_serial_jtag_read_bytes(in, sizeof in, pdMS_TO_TICKS(50)); if (k > 0) feed(&console, in, k); }
}

void app_main(void) {
  uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
  usb_serial_jtag_driver_config_t uc = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  usb_serial_jtag_driver_install(&uc);
  out_ready = 1;
  lines = xQueueCreate(8, sizeof(line_t));
  cap_done = xSemaphoreCreateBinary();
  xTaskCreate(uart_task, "uart", 3072, NULL, 5, NULL);
  xTaskCreate(usb_task, "usb", 3072, NULL, 5, NULL);
  xTaskCreatePinnedToCore(cap_task, "capture", 4096, NULL, 10, &cap_h, 1);
  say("\n\nLiftLab ESC signal sniffer (ESP32-S3) - listens only, drives nothing but the GPIO8 test signal\n"
      "CH1 GPIO4, CH2 GPIO5, CH3 GPIO6, CH4 GPIO7: protected 3.3 V inputs; see docs/esc-sniffer.md.\n"
      "A series resistor alone is NOT 4.2 V protection. Drone GND to S3 GND; never battery + to a GPIO.\n"
      "Enter: capture | w: watch | raw <ch> | csv | win <ms> | gen <hz> <high_us> | ? help\n");
  for (;;) {
    line_t l;
    if (xQueueReceive(lines, &l, pdMS_TO_TICKS(watching ? 300 : 1000))) { say("> %s\n", l.t); command(l.t); }
    else if (watching) { grab(); report(1); }
  }
}
