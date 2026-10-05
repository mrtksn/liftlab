/*
 * LiftLab: brushed motor test on H-bridge boards (L9110S, L293D) with an ESP32-S3.
 *
 * Four motors, each on two driver inputs: PWM on one input and the other held low spins it one way; swapped,
 * the other way; both low lets it coast. Nothing spins until you type a command.
 *
 *   M1  L9110S "Motor A"   A-1A = GPIO4   A-1B = GPIO5
 *   M2  L9110S "Motor B"   B-1A = GPIO6   B-1B = GPIO7
 *   M3  L293D  "A"         IN1  = GPIO15  IN2  = GPIO16
 *   M4  L293D  "B"         IN3  = GPIO17  IN4  = GPIO18
 *
 * Commands (serial, 115200 baud, either USB port; end each with Enter):
 *   m <1-4> <0-100>   motor n at that % (capped by max)       e.g.  m 1 30
 *   a <0-100>         all four at that %
 *   r <1-4>           reverse motor n's direction
 *   seq               each motor in turn at 30% for 1.5 s
 *   max <0-100>       the highest % allowed (default 60; protects 3.7 V motors on a 5 V supply)
 *   f <hz>            PWM frequency (default 1000; the L293D is slow, keep it ≤ 5000 with it)
 *   s  (or Enter)     stop all
 * Safety: if no command arrives for 10 s while a motor runs, all stop.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "driver/usb_serial_jtag.h"
#include "esp_timer.h"
#include <stdarg.h>

/* replies go to both USB ports (the UART bridge and the native USB), whichever one you're on */
static int out_ready;
static void say(const char *fmt, ...) {
  char b[256]; va_list ap; va_start(ap, fmt); int n = vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof b - 1) n = sizeof b - 1;
  if (!out_ready) { fwrite(b, 1, (size_t)n, stdout); return; }
  char c[300]; int k = 0;                       /* terminals want \r\n */
  for (int i = 0; i < n && k < (int)sizeof c - 2; i++) { if (b[i] == '\n') c[k++] = '\r'; c[k++] = b[i]; }
  uart_write_bytes(UART_NUM_0, c, (size_t)k);
  usb_serial_jtag_write_bytes(c, (size_t)k, 0);
}

#define N 4
static const int PIN_A[N] = { 4, 6, 15, 17 }, PIN_B[N] = { 5, 7, 16, 18 };
static const char *NAME[N] = { "M1 (L9110S A)", "M2 (L9110S B)", "M3 (L293D A)", "M4 (L293D B)" };
static int duty[N], rev[N], max_pct = 60, freq = 1000;
static int64_t last_cmd;
#define RES LEDC_TIMER_13_BIT
#define FULL ((1 << 13) - 1)

static void out(int m) {
  int d = duty[m] * FULL / 100;
  int a = rev[m] ? 0 : d, b = rev[m] ? d : 0;
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(2 * m), (uint32_t)a); ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(2 * m));
  ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(2 * m + 1), (uint32_t)b); ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(2 * m + 1));
}
static void stop_all(void) { for (int m = 0; m < N; m++) { duty[m] = 0; out(m); } }
static int set_timer(int hz) {
  ledc_timer_config_t t = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = RES, .timer_num = LEDC_TIMER_0, .freq_hz = (uint32_t)hz, .clk_cfg = LEDC_AUTO_CLK };
  return ledc_timer_config(&t) == ESP_OK ? 0 : -1;
}
static void outputs_init(void) {
  set_timer(freq);
  for (int m = 0; m < N; m++) for (int k = 0; k < 2; k++) {
    ledc_channel_config_t c = { .gpio_num = k ? PIN_B[m] : PIN_A[m], .speed_mode = LEDC_LOW_SPEED_MODE, .channel = (ledc_channel_t)(2 * m + k),
                                .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 };
    ledc_channel_config(&c);
  }
}
static void status(void) {
  say("  ");
  for (int m = 0; m < N; m++) say("%s %3d%%%s   ", NAME[m], duty[m], rev[m] ? " (rev)" : "");
  say("| max %d%%, %d Hz\n", max_pct, freq);
}

static void command(char *s) {
  while (*s && isspace((unsigned char)*s)) s++;
  char *e = s + strlen(s); while (e > s && isspace((unsigned char)e[-1])) *--e = 0;
  last_cmd = esp_timer_get_time();
  int a = -1, b = -1;
  if (!*s || !strcmp(s, "s") || !strcmp(s, "stop")) { stop_all(); say("all stopped\n"); }
  else if (sscanf(s, "m %d %d", &a, &b) == 2) {
    if (a < 1 || a > N || b < 0 || b > 100) { say("m <1-4> <0-100>\n"); return; }
    if (b > max_pct) { say("capped at max %d%% (type: max <n> to change)\n", max_pct); b = max_pct; }
    duty[a - 1] = b; out(a - 1); status();
  } else if (sscanf(s, "a %d", &b) == 1) {
    if (b < 0 || b > 100) { say("a <0-100>\n"); return; }
    if (b > max_pct) { say("capped at max %d%%\n", max_pct); b = max_pct; }
    for (int m = 0; m < N; m++) { duty[m] = b; out(m); }
    status();
  } else if (sscanf(s, "r %d", &a) == 1) {
    if (a < 1 || a > N) { say("r <1-4>\n"); return; }
    rev[a - 1] = !rev[a - 1]; out(a - 1); status();
  } else if (!strcmp(s, "seq")) {
    stop_all();
    for (int m = 0; m < N; m++) {
      int d = 30 < max_pct ? 30 : max_pct;
      say("%s at %d%%...\n", NAME[m], d);
      duty[m] = d; out(m); vTaskDelay(pdMS_TO_TICKS(1500)); duty[m] = 0; out(m); vTaskDelay(pdMS_TO_TICKS(500));
    }
    say("sequence done\n");
  } else if (sscanf(s, "max %d", &b) == 1) {
    if (b < 0 || b > 100) { say("max <0-100>\n"); return; }
    max_pct = b;
    for (int m = 0; m < N; m++) if (duty[m] > max_pct) { duty[m] = max_pct; out(m); }
    status();
  } else if (sscanf(s, "f %d", &b) == 1) {
    if (b < 100 || b > 40000) { say("f <100-40000>\n"); return; }
    stop_all(); freq = b;
    if (set_timer(freq)) say("that frequency isn't possible\n"); else say("PWM at %d Hz (motors stopped)\n", freq);
  } else if (!strcmp(s, "?") || !strcmp(s, "help") || !strcmp(s, "h")) {
    say("m <1-4> <0-100> | a <0-100> | r <1-4> | seq | max <0-100> | f <hz> | s (stop)\n"); status();
  } else say("unknown: %s  (type ? for help)\n", s);
}

/* lines from either USB port: the UART bridge ("UART"/"COM" port) and the native USB ("USB" port) */
static QueueHandle_t lines;
typedef struct { char t[64]; } line_t;
static void feed(char *buf, int *n, const uint8_t *in, int k) {
  for (int i = 0; i < k; i++) {
    char c = (char)in[i];
    if (c == '\r' || c == '\n') { if (*n > 0 || c == '\r') { line_t l; buf[*n] = 0; strncpy(l.t, buf, sizeof l.t - 1); l.t[sizeof l.t - 1] = 0; xQueueSend(lines, &l, 0); } *n = 0; }
    else if (*n < 62) buf[(*n)++] = c;
  }
}
static void uart_task(void *arg) {
  char buf[64]; int n = 0; uint8_t in[64];
  for (;;) { int k = uart_read_bytes(UART_NUM_0, in, sizeof in, pdMS_TO_TICKS(50)); if (k > 0) feed(buf, &n, in, k); }
}
static void usb_task(void *arg) {
  char buf[64]; int n = 0; uint8_t in[64];
  for (;;) { int k = usb_serial_jtag_read_bytes(in, sizeof in, pdMS_TO_TICKS(50)); if (k > 0) feed(buf, &n, in, k); }
}

void app_main(void) {
  outputs_init();   /* every input low first: nothing spins */
  stop_all();
  uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0);
  usb_serial_jtag_driver_config_t uc = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  usb_serial_jtag_driver_install(&uc);
  out_ready = 1;
  lines = xQueueCreate(8, sizeof(line_t));
  xTaskCreate(uart_task, "uart", 3072, NULL, 5, NULL);
  xTaskCreate(usb_task, "usb", 3072, NULL, 5, NULL);
  say("\n\nLiftLab motor test (ESP32-S3)\n"
         "M1 GPIO4/5 (L9110S A), M2 GPIO6/7 (L9110S B), M3 GPIO15/16 (L293D A), M4 GPIO17/18 (L293D B)\n"
         "PROPELLERS OFF for the first tests. Type ? for help; Enter alone stops everything.\n");
  status();
  last_cmd = esp_timer_get_time();
  for (;;) {
    line_t l;
    if (xQueueReceive(lines, &l, pdMS_TO_TICKS(200))) { say("> %s\n", l.t); command(l.t); }
    int running = 0; for (int m = 0; m < N; m++) running |= duty[m] > 0;
    if (running && esp_timer_get_time() - last_cmd > 10000000) { stop_all(); say("no command for 10 s: all stopped\n"); }
  }
}
