/* The flight controller's hardware: see hw.h. */
#include "hw.h"
#include "esp_board.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define G 9.80665f

/* ───────── the wiring ───────── */
static int hw_check(const hw_config *c, char *err, int errn);
/* Default pins for an ESP32 DevKit (ESP32-WROOM-32): outputs that are free at boot (no strapping pins). */
void hw_defaults(hw_config *c) {
  memset(c, 0, sizeof *c);
  c->version = HW_VERSION;
  memset(c->motor_pin, -1, sizeof c->motor_pin); memset(c->servo_pin, -1, sizeof c->servo_pin);
  static const int8_t mp[] = { LB_MOTOR_PINS }, sp[] = { LB_SERVO_PINS };
  memcpy(c->motor_pin, mp, sizeof mp); memcpy(c->servo_pin, sp, sizeof sp);
  c->esc_hz = 400; c->esc_min_us = 1000; c->esc_max_us = 2000;
  for (int j = 0; j < FC_MAX_JOINTS; j++) { c->servo_center_us[j] = 1500; c->servo_us_per_rad[j] = 500.0f / (float)(M_PI / 4); }   /* ±500 µs = ±45° */
  c->sda = LB_SDA; c->scl = LB_SCL;
  c->batt_pin = -1; c->batt_divider = 11.0f;
  c->rate_hz = LB_RATE; c->telem_hz = 20; c->vref = 16.0f; c->link_baud = 921600;
  c->crsf_rx = c->crsf_tx = -1; c->elrs_rate = 250; c->elrs_ratio = 4;
}
int hw_load(hw_config *c) {
  hw_defaults(c);
  nvs_handle_t h; if (nvs_open("dfb", NVS_READONLY, &h) != ESP_OK) return 0;
  hw_config t; size_t n = sizeof t;
  if (nvs_get_blob(h, "hw", &t, &n) == ESP_OK && n == sizeof t && t.version == HW_VERSION) *c = t;
  else if (n == offsetof(hw_config, link_baud) && t.version == 2) { memcpy(c, &t, n); c->version = HW_VERSION; c->link_baud = 115200; }   /* saved by v2: its link speed */
  else if (n == offsetof(hw_config, crsf_rx) && t.version == 3) { memcpy(c, &t, n); c->version = HW_VERSION; }                            /* saved by v3: no receiver yet */
  nvs_close(h);
  char why[96];
  if (hw_check(c, why, sizeof why)) { printf("saved wiring refused: %s; using chip defaults\n", why); hw_defaults(c); }
  return 0;
}
int hw_save(const hw_config *c) {
  nvs_handle_t h; if (nvs_open("dfb", NVS_READWRITE, &h) != ESP_OK) return -1;
  esp_err_t e = nvs_set_blob(h, "hw", c, sizeof *c); if (e == ESP_OK) e = nvs_commit(h);
  nvs_close(h); return e == ESP_OK ? 0 : -1;
}
int hw_airframe_load(uint8_t *buf, uint32_t cap, uint32_t *len) {
  nvs_handle_t h; if (nvs_open("dfb", NVS_READONLY, &h) != ESP_OK) return -1;
  size_t n = cap; esp_err_t e = nvs_get_blob(h, "airframe", buf, &n);
  nvs_close(h); *len = (uint32_t)n; return e == ESP_OK ? 0 : -1;
}
int hw_airframe_save(const uint8_t *buf, uint32_t len) {
  nvs_handle_t h; if (nvs_open("dfb", NVS_READWRITE, &h) != ESP_OK) return -1;
  esp_err_t e = nvs_set_blob(h, "airframe", buf, len); if (e == ESP_OK) e = nvs_commit(h);
  nvs_close(h); return e == ESP_OK ? 0 : -1;
}

static int parse_list(const char *s, float *v, int max) {
  int n = 0; char *end;
  while (*s && n < max) { v[n] = strtof(s, &end); if (end == s) return -1; n++; s = end; while (*s == ',' || *s == ' ') s++; }
  return n;
}
/* GPIOs an ESP32 (WROOM) can drive an output on: not the flash pins (6–11), not UART0 (1, 3), not input-only (34–39),
 * and not the boot-strapping pins (0, 2, 5, 12, 15): something wired there can stop it booting, and some of them
 * toggle during boot, which an ESC could take as a pulse. */
static int pin_ok(int p) { return p == -1 || lb_output_pin(p); }
/* The wiring as a whole: no pin used twice, the ESC's longest pulse fits its period. */
static int hw_check(const hw_config *c, char *err, int errn) {
  for (int i = 0; i < FC_MAX_MOTORS; i++) if (!pin_ok(c->motor_pin[i])) { snprintf(err, errn, "reserved motor pin"); return -1; }
  for (int i = 0; i < FC_MAX_JOINTS; i++) if (!pin_ok(c->servo_pin[i])) { snprintf(err, errn, "reserved servo pin"); return -1; }
  if (!lb_output_pin(c->sda) || !lb_output_pin(c->scl) || (c->batt_pin >= 0 && !lb_adc_pin(c->batt_pin)) ||
      (c->crsf_rx >= 0 && !lb_input_pin(c->crsf_rx)) || (c->crsf_tx >= 0 && !lb_output_pin(c->crsf_tx))) {
    snprintf(err, errn, "reserved sensor or radio pin"); return -1;
  }
  int used[SOC_GPIO_PIN_COUNT] = { 0 };
  #define USE(p, what) do { int p_ = (p); if (p_ >= 0 && p_ < SOC_GPIO_PIN_COUNT) { if (used[p_]) { snprintf(err, errn, "GPIO %d is used twice (%s)", p_, what); return -1; } used[p_] = 1; } } while (0)
  for (int i = 0; i < FC_MAX_MOTORS; i++) USE(c->motor_pin[i], "motor");
  for (int j = 0; j < FC_MAX_JOINTS; j++) USE(c->servo_pin[j], "servo");
  USE(c->sda, "I2C"); USE(c->scl, "I2C"); USE(c->batt_pin, "battery"); USE(c->crsf_rx, "radio receiver"); USE(c->crsf_tx, "radio receiver");
  #undef USE
  int nm = 0, ns = 0; for (int i = 0; i < FC_MAX_MOTORS; i++) nm += c->motor_pin[i] >= 0; for (int j = 0; j < FC_MAX_JOINTS; j++) ns += c->servo_pin[j] >= 0;
  if (nm + ns > LB_PWM_OUTPUTS) { snprintf(err, errn, "%d motors and %d servos: this chip has %d PWM outputs", nm, ns, LB_PWM_OUTPUTS); return -1; }
  if (c->esc_max_us > 1000000 / c->esc_hz - 100) { snprintf(err, errn, "at %d Hz an ESC pulse can be at most %d µs: lower esc_hz or esc_us", c->esc_hz, 1000000 / c->esc_hz - 100); return -1; }
  return 0;
}
static int hw_set1(hw_config *c, const char *line, char *err, int errn);
int hw_set(hw_config *c, const char *line, char *err, int errn) {
  hw_config t = *c;
  if (hw_set1(&t, line, err, errn) || hw_check(&t, err, errn)) return -1;
  *c = t; return 0;
}
static int hw_set1(hw_config *c, const char *line, char *err, int errn) {
  char key[24]; const char *eq = strchr(line, '='); float v[FC_MAX_MOTORS]; int n;
  if (!eq || eq - line >= (int)sizeof key) { snprintf(err, errn, "expected key=value"); return -1; }
  memcpy(key, line, (size_t)(eq - line)); key[eq - line] = 0;
  n = parse_list(eq + 1, v, FC_MAX_MOTORS);
  if (n < 0) { snprintf(err, errn, "%s: not a list of numbers", key); return -1; }
  if (!strcmp(key, "motors") || !strcmp(key, "servos")) {
    int m = !strcmp(key, "motors"), max = m ? FC_MAX_MOTORS : FC_MAX_JOINTS; int8_t *pins = m ? c->motor_pin : c->servo_pin;
    if (n > max) { snprintf(err, errn, "at most %d %s", max, key); return -1; }
    for (int i = 0; i < n; i++) if (v[i] < 0 || !pin_ok((int)v[i])) { snprintf(err, errn, "GPIO %d is reserved or cannot drive an output on this chip", (int)v[i]); return -1; }
    for (int i = 0; i < max; i++) pins[i] = i < n ? (int8_t)v[i] : -1;
  } else if (!strcmp(key, "esc_hz")) {
    if (n != 1 || v[0] < 50 || v[0] > 490) { snprintf(err, errn, "esc_hz: 50 to 490 for standard PWM ESCs"); return -1; }
    c->esc_hz = (int16_t)v[0];
  } else if (!strcmp(key, "esc_us")) {
    if (n != 2 || v[0] < 800 || v[1] > 2200 || v[1] - v[0] < 500) { snprintf(err, errn, "esc_us=min,max, e.g. 1000,2000"); return -1; }
    c->esc_min_us = (int16_t)v[0]; c->esc_max_us = (int16_t)v[1];
  } else if (!strcmp(key, "servo_center")) {
    for (int i = 0; i < n && i < FC_MAX_JOINTS; i++) { if (v[i] < 800 || v[i] > 2200) { snprintf(err, errn, "servo_center: 800 to 2200 µs"); return -1; } c->servo_center_us[i] = (int16_t)v[i]; }
  } else if (!strcmp(key, "servo_us_per_rad")) {
    for (int i = 0; i < n && i < FC_MAX_JOINTS; i++) { if (fabsf(v[i]) < 100 || fabsf(v[i]) > 2000) { snprintf(err, errn, "servo_us_per_rad: 100 to 2000 (negative to reverse)"); return -1; } c->servo_us_per_rad[i] = v[i]; }
  } else if (!strcmp(key, "i2c")) {
    if (n != 2 || !pin_ok((int)v[0]) || !pin_ok((int)v[1]) || v[0] < 0 || v[1] < 0) { snprintf(err, errn, "i2c=sda,scl"); return -1; }
    c->sda = (int8_t)v[0]; c->scl = (int8_t)v[1];
  } else if (!strcmp(key, "battery")) {
    if (n == 1 && v[0] == -1) c->batt_pin = -1;
    else if (n != 2 || !lb_adc_pin((int)v[0]) || v[1] < 1 || v[1] > 30) { snprintf(err, errn, "battery=pin,divider with a free ADC1 pin (or battery=-1)"); return -1; }
    else { c->batt_pin = (int8_t)v[0]; c->batt_divider = v[1]; }
  } else if (!strcmp(key, "rate")) {
    if (n != 1 || (v[0] != 250 && v[0] != 500 && v[0] != 1000)) { snprintf(err, errn, "rate: 250, 500 or 1000 Hz"); return -1; }
    c->rate_hz = (int16_t)v[0];
  } else if (!strcmp(key, "vref")) {
    if (n != 1 || v[0] < 3 || v[0] > 60) { snprintf(err, errn, "vref: the pack voltage the airframe's thrust is for, e.g. 16 for 4S"); return -1; }
    c->vref = v[0];
  } else if (!strcmp(key, "baud")) {
    if (n != 1 || (v[0] != 115200 && v[0] != 230400 && v[0] != 460800 && v[0] != 921600)) { snprintf(err, errn, "baud: 115200, 230400, 460800 or 921600 (the learning wants 921600)"); return -1; }
    c->link_baud = (int32_t)v[0];
  } else if (!strcmp(key, "crsf")) {
    if (n == 1 && v[0] == -1) c->crsf_rx = c->crsf_tx = -1;
    else if (n != 2 || !lb_input_pin((int)v[0]) || !pin_ok((int)v[1]) || v[1] < 0) { snprintf(err, errn, "crsf=rx,tx: free input and output pins (or crsf=-1)"); return -1; }
    else { c->crsf_rx = (int8_t)v[0]; c->crsf_tx = (int8_t)v[1]; }
  } else if (!strcmp(key, "elrs")) {
    if (n != 2 || (v[0] != 50 && v[0] != 150 && v[0] != 250 && v[0] != 500) || v[1] < 2 || v[1] > 128) { snprintf(err, errn, "elrs=rate,ratio as set on the radio: 50, 150, 250 or 500 Hz; telemetry 1:2 to 1:128"); return -1; }
    c->elrs_rate = (int16_t)v[0]; c->elrs_ratio = (int16_t)v[1];
  } else if (!strcmp(key, "telemetry")) {
    if (n != 1 || v[0] < 0 || v[0] > 50) { snprintf(err, errn, "telemetry: 0 to 50 Hz"); return -1; }
    c->telem_hz = (int16_t)v[0];
  } else { snprintf(err, errn, "unknown setting %s", key); return -1; }
  return 0;
}
void hw_describe(const hw_config *c, char *out, int n) {
  int k = snprintf(out, n, "motors=");
  for (int i = 0; i < FC_MAX_MOTORS && c->motor_pin[i] >= 0; i++) k += snprintf(out + k, n - k, "%s%d", i ? "," : "", c->motor_pin[i]);
  k += snprintf(out + k, n - k, " servos=");
  for (int i = 0; i < FC_MAX_JOINTS && c->servo_pin[i] >= 0; i++) k += snprintf(out + k, n - k, "%s%d", i ? "," : "", c->servo_pin[i]);
  k += snprintf(out + k, n - k, " esc_hz=%d esc_us=%d,%d i2c=%d,%d battery=%d,%.1f vref=%.1f rate=%d telemetry=%d baud=%ld crsf=%d,%d elrs=%d,%d servo_center=",
                c->esc_hz, c->esc_min_us, c->esc_max_us, c->sda, c->scl, c->batt_pin, (double)c->batt_divider, (double)c->vref, c->rate_hz, c->telem_hz, (long)c->link_baud, c->crsf_rx, c->crsf_tx, c->elrs_rate, c->elrs_ratio);
  for (int i = 0; i < FC_MAX_JOINTS && c->servo_pin[i] >= 0; i++) k += snprintf(out + k, n - k, "%s%d", i ? "," : "", c->servo_center_us[i]);
  k += snprintf(out + k, n - k, " servo_us_per_rad=");
  for (int i = 0; i < FC_MAX_JOINTS && c->servo_pin[i] >= 0; i++) k += snprintf(out + k, n - k, "%s%.0f", i ? "," : "", (double)c->servo_us_per_rad[i]);
}

/* ───────── sensors ───────── */
static i2c_master_bus_handle_t bus;
static i2c_master_dev_handle_t imu_dev, baro_dev;
static int imu_kind, baro_kind;
static float gyro_bias[3];
static int wr(i2c_master_dev_handle_t d, uint8_t reg, uint8_t v) { uint8_t b[2] = { reg, v }; return i2c_master_transmit(d, b, 2, 5) == ESP_OK ? 0 : -1; }
static int rd(i2c_master_dev_handle_t d, uint8_t reg, uint8_t *buf, int n) { return i2c_master_transmit_receive(d, &reg, 1, buf, (size_t)n, 5) == ESP_OK ? 0 : -1; }
static i2c_master_dev_handle_t add(uint8_t addr) {
  i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = 400000 };
  i2c_master_dev_handle_t d = NULL; if (i2c_master_bus_add_device(bus, &dc, &d) != ESP_OK) return NULL; return d;
}

/* BMP280 calibration and state */
static struct { uint16_t T1, P1; int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9; } bc;
static float p0;

int hw_sensors_init(const hw_config *c, hw_sensors *s, char *log, int logn) {
  memset(s, 0, sizeof *s); int k = 0;
  i2c_master_bus_config_t bcfg = { .i2c_port = I2C_NUM_0, .sda_io_num = c->sda, .scl_io_num = c->scl, .clk_source = I2C_CLK_SRC_DEFAULT,
                                   .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true };
  if (i2c_new_master_bus(&bcfg, &bus) != ESP_OK) { snprintf(log, logn, "I2C bus on SDA %d, SCL %d didn't start", c->sda, c->scl); return -1; }
  /* The IMU: an MPU-6050 family chip at 0x68/0x69 (MPU-6050, MPU-6500, MPU-9250), else a LIS3DH at 0x18/0x19. */
  for (uint8_t a = 0x68; a <= 0x69 && !imu_kind; a++) {
    if (i2c_master_probe(bus, a, 20) != ESP_OK) continue;
    i2c_master_dev_handle_t d = add(a); uint8_t who = 0;
    if (!d || rd(d, 0x75, &who, 1)) continue;
    const char *nm = who == 0x68 ? "MPU-6050" : who == 0x70 ? "MPU-6500" : who == 0x71 ? "MPU-9250" : who == 0x73 ? "MPU-9255" : who == 0x72 ? "MPU-6052" : NULL;
    if (!nm) { k += snprintf(log + k, logn - k, "0x%02x answers with id 0x%02x, not a known IMU; ", a, who); continue; }
    wr(d, 0x6B, 0x80); vTaskDelay(pdMS_TO_TICKS(100));       /* reset */
    /* clock from the gyro's PLL; low-pass ~98 Hz (gyro; the MPU-6050's accelerometer too); 1 kHz; ±2000 °/s; ±8 g;
     * the MPU-6500/9250's accelerometer low-pass (its own register) ~99 Hz. Each is read back: a setting that didn't
     * take would scale every reading wrong. */
    static const uint8_t set[][2] = { { 0x6B, 0x01 }, { 0x1A, 0x02 }, { 0x19, 0x00 }, { 0x1B, 0x18 }, { 0x1C, 0x10 }, { 0x1D, 0x02 } };
    int bad = 0;
    for (unsigned q = 0; q < sizeof set / sizeof set[0]; q++) {
      if (set[q][0] == 0x1D && who == 0x68) continue;
      uint8_t back = 0xFF; wr(d, set[q][0], set[q][1]);
      if (rd(d, set[q][0], &back, 1) || (back & (set[q][0] == 0x1B || set[q][0] == 0x1C ? 0x18 : 0xFF)) != set[q][1]) bad = 1;
    }
    if (bad) { k += snprintf(log + k, logn - k, "%s at 0x%02x didn't take its settings; ", nm, a); continue; }
    imu_dev = d; imu_kind = 1; s->imu = 1; snprintf(s->imu_name, sizeof s->imu_name, "%s at 0x%02x", nm, a);
  }
  for (uint8_t a = 0x18; a <= 0x19 && !imu_kind; a++) {
    if (i2c_master_probe(bus, a, 20) != ESP_OK) continue;
    i2c_master_dev_handle_t d = add(a); uint8_t who = 0;
    if (!d || rd(d, 0x0F, &who, 1) || who != 0x33) continue;
    wr(d, 0x20, 0x97);                                        /* 1.344 kHz, x y z on */
    wr(d, 0x23, 0x28);                                        /* ±8 g, high resolution */
    uint8_t r1 = 0, r4 = 0; if (rd(d, 0x20, &r1, 1) || rd(d, 0x23, &r4, 1) || r1 != 0x97 || (r4 & 0x38) != 0x28) { k += snprintf(log + k, logn - k, "LIS3DH didn't take its settings; "); continue; }
    imu_dev = d; imu_kind = 2; s->imu = 2; snprintf(s->imu_name, sizeof s->imu_name, "LIS3DH at 0x%02x (no gyro)", a);
  }
  if (!imu_kind) k += snprintf(log + k, logn - k, "no IMU found on I2C (SDA %d, SCL %d); ", c->sda, c->scl);
  /* The barometer: BMP280 or BME280 at 0x76/0x77. */
  for (uint8_t a = 0x76; a <= 0x77 && !baro_kind; a++) {
    if (i2c_master_probe(bus, a, 20) != ESP_OK) continue;
    i2c_master_dev_handle_t d = add(a); uint8_t id = 0, cal[24];
    if (!d || rd(d, 0xD0, &id, 1) || (id != 0x58 && id != 0x60) || rd(d, 0x88, cal, 24)) continue;
    #define U16(i) (uint16_t)(cal[i] | cal[i + 1] << 8)
    bc.T1 = U16(0); bc.T2 = (int16_t)U16(2); bc.T3 = (int16_t)U16(4); bc.P1 = U16(6); bc.P2 = (int16_t)U16(8); bc.P3 = (int16_t)U16(10);
    bc.P4 = (int16_t)U16(12); bc.P5 = (int16_t)U16(14); bc.P6 = (int16_t)U16(16); bc.P7 = (int16_t)U16(18); bc.P8 = (int16_t)U16(20); bc.P9 = (int16_t)U16(22);
    #undef U16
    wr(d, 0xF5, 0x08);                                        /* filter ×4, no standby */
    wr(d, 0xF4, 0x57);                                        /* temperature ×2, pressure ×16, running */
    baro_dev = d; baro_kind = 1; s->baro = 1; snprintf(s->baro_name, sizeof s->baro_name, "%s at 0x%02x", id == 0x60 ? "BME280" : "BMP280", a);
  }
  if (k == 0 && log) log[0] = 0;
  return imu_kind ? 0 : -1;
}

int hw_imu_read(fc_imu *m) {
  uint8_t b[14];
  m->have_gyro = 0;
  if (imu_kind == 1) {
    if (rd(imu_dev, 0x3B, b, 14)) return -1;
    const float ka = 8 * G / 32768.0f, kg = 2000.0f * (float)M_PI / 180.0f / 32768.0f;
    for (int i = 0; i < 3; i++) {
      m->acc[i] = (float)(int16_t)(b[2 * i] << 8 | b[2 * i + 1]) * ka;
      m->gyro[i] = (float)(int16_t)(b[8 + 2 * i] << 8 | b[9 + 2 * i]) * kg - gyro_bias[i];
    }
    m->have_gyro = 1; return 0;
  }
  if (imu_kind == 2) {
    if (rd(imu_dev, 0x28 | 0x80, b, 6)) return -1;
    for (int i = 0; i < 3; i++) { m->acc[i] = (float)((int16_t)(b[2 * i] | b[2 * i + 1] << 8) >> 4) * 0.004f * G; m->gyro[i] = 0; }
    return 0;
  }
  return -1;
}
void hw_gyro_calibrate(const float bias[3]) { for (int i = 0; i < 3; i++) gyro_bias[i] += bias[i]; }

int hw_baro_read(float *alt) {
  if (!baro_kind) return 0;
  uint8_t b[6]; if (rd(baro_dev, 0xF7, b, 6)) return 0;
  int32_t adc_P = (int32_t)(b[0] << 12 | b[1] << 4 | b[2] >> 4), adc_T = (int32_t)(b[3] << 12 | b[4] << 4 | b[5] >> 4);
  /* the datasheet's compensation (32-bit temperature, 64-bit pressure) */
  int32_t v1 = ((((adc_T >> 3) - ((int32_t)bc.T1 << 1))) * (int32_t)bc.T2) >> 11;
  int32_t v2 = (((((adc_T >> 4) - (int32_t)bc.T1) * ((adc_T >> 4) - (int32_t)bc.T1)) >> 12) * (int32_t)bc.T3) >> 14;
  int32_t t_fine = v1 + v2;
  int64_t p1 = (int64_t)t_fine - 128000, p2 = p1 * p1 * (int64_t)bc.P6;
  p2 += (p1 * (int64_t)bc.P5) << 17; p2 += ((int64_t)bc.P4) << 35;
  p1 = ((p1 * p1 * (int64_t)bc.P3) >> 8) + ((p1 * (int64_t)bc.P2) << 12);
  p1 = ((((int64_t)1) << 47) + p1) * (int64_t)bc.P1 >> 33;
  if (!p1) return 0;
  int64_t p = 1048576 - adc_P; p = (((p << 31) - p2) * 3125) / p1;
  p1 = ((int64_t)bc.P9 * (p >> 13) * (p >> 13)) >> 25; p2 = ((int64_t)bc.P8 * p) >> 19;
  p = ((p + p1 + p2) >> 8) + ((int64_t)bc.P7 << 4);
  float pa = (float)p / 256.0f;
  if (pa < 30000 || pa > 110000) return 0;
  if (p0 == 0) p0 = pa;
  *alt = 44330.0f * (1.0f - powf(pa / p0, 0.190295f));
  return 1;
}

/* ───────── outputs ───────── */
/* ESCs 1–8 on the high-speed channels (timer 0 at esc_hz), servos on the low-speed ones (timer 1 at 50 Hz), and
 * ESCs 9–12, if any, on the low-speed channels left (timer 2 at esc_hz). */
static int8_t esc_ch[FC_MAX_MOTORS], esc_mode[FC_MAX_MOTORS], srv_ch[FC_MAX_JOINTS];
static hw_config oc; static int outputs_up;
static uint32_t esc_duty(float us) { return (uint32_t)(us * oc.esc_hz * 16384.0f / 1e6f); }
static uint32_t srv_duty(float us) { return (uint32_t)(us * 50 * 16384.0f / 1e6f); }
int hw_outputs_init(const hw_config *c, char *log, int logn) {
  oc = *c; int n_hs = 0, n_ls = 0, n_esc = 0, n_srv = 0;
  memset(esc_ch, -1, sizeof esc_ch); memset(srv_ch, -1, sizeof srv_ch);
#if SOC_LEDC_SUPPORT_HS_MODE
  const ledc_mode_t motor_mode = LEDC_HIGH_SPEED_MODE;
#else
  const ledc_mode_t motor_mode = LEDC_LOW_SPEED_MODE;
#endif
  ledc_timer_config_t te = { .speed_mode = motor_mode, .duty_resolution = LEDC_TIMER_14_BIT, .timer_num = LEDC_TIMER_0, .freq_hz = (uint32_t)c->esc_hz, .clk_cfg = LEDC_AUTO_CLK };
  ledc_timer_config_t ts = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_14_BIT, .timer_num = LEDC_TIMER_1, .freq_hz = 50, .clk_cfg = LEDC_AUTO_CLK };
  ledc_timer_config_t tl = { .speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_14_BIT, .timer_num = LEDC_TIMER_2, .freq_hz = (uint32_t)c->esc_hz, .clk_cfg = LEDC_AUTO_CLK };
  if (ledc_timer_config(&te) != ESP_OK || ledc_timer_config(&ts) != ESP_OK || ledc_timer_config(&tl) != ESP_OK) { snprintf(log, logn, "PWM timers didn't start"); return -1; }
  for (int i = 0; i < FC_MAX_MOTORS; i++) {
    if (c->motor_pin[i] < 0) continue;
    #if SOC_LEDC_SUPPORT_HS_MODE
    int hs = n_hs < SOC_LEDC_CHANNEL_NUM;
#else
    int hs = 0;
#endif
    int ch = hs ? n_hs : n_ls; if (!hs && n_ls >= SOC_LEDC_CHANNEL_NUM) break;
    ledc_channel_config_t cc = { .gpio_num = c->motor_pin[i], .speed_mode = hs ? motor_mode : LEDC_LOW_SPEED_MODE, .channel = (ledc_channel_t)ch,
                                 .timer_sel = hs ? LEDC_TIMER_0 : LEDC_TIMER_2, .duty = esc_duty(c->esc_min_us) };
    if (ledc_channel_config(&cc) != ESP_OK) continue;
    esc_ch[i] = (int8_t)ch; esc_mode[i] = (int8_t)(hs ? motor_mode : LEDC_LOW_SPEED_MODE); n_esc++;
    if (hs) n_hs++; else n_ls++;
  }
  for (int j = 0; j < FC_MAX_JOINTS; j++) {
    if (c->servo_pin[j] < 0 || n_ls >= SOC_LEDC_CHANNEL_NUM) continue;
    ledc_channel_config_t cc = { .gpio_num = c->servo_pin[j], .speed_mode = LEDC_LOW_SPEED_MODE, .channel = (ledc_channel_t)n_ls, .timer_sel = LEDC_TIMER_1, .duty = srv_duty(c->servo_center_us[j]) };
    if (ledc_channel_config(&cc) == ESP_OK) { srv_ch[j] = (int8_t)n_ls++; n_srv++; }
  }
  outputs_up = 1;
  snprintf(log, logn, "%d ESC outputs at %d Hz (%d–%d µs), %d servo outputs at 50 Hz", n_esc, c->esc_hz, c->esc_min_us, c->esc_max_us, n_srv);
  return 0;
}
/* Every motor and servo of an airframe has a working output (motor i on motor_pin[i], servo j on servo_pin[j]). */
int hw_outputs_ok(int n_motors, int n_servos, char *why, int whyn) {
  if (!outputs_up) { snprintf(why, whyn, "the PWM outputs didn't start"); return 0; }
  for (int i = 0; i < n_motors; i++) if (esc_ch[i] < 0) { snprintf(why, whyn, "motor %d has no output (motors=… in the wiring)", i + 1); return 0; }
  for (int j = 0; j < n_servos; j++) if (srv_ch[j] < 0) { snprintf(why, whyn, "servo %d has no output (servos=… in the wiring)", j + 1); return 0; }
  return 1;
}
void hw_outputs_set(const fc_out *o, int n_motors, int n_servos) {
  for (int i = 0; i < n_motors && i < FC_MAX_MOTORS; i++) if (esc_ch[i] >= 0) {
    float t = o->motor[i]; t = t > 0 ? (t < 1 ? t : 1) : 0;          /* NaN → 0 */
    ledc_set_duty((ledc_mode_t)esc_mode[i], (ledc_channel_t)esc_ch[i], esc_duty(oc.esc_min_us + t * (oc.esc_max_us - oc.esc_min_us)));
    ledc_update_duty((ledc_mode_t)esc_mode[i], (ledc_channel_t)esc_ch[i]);
  }
  for (int j = 0; j < n_servos && j < FC_MAX_JOINTS; j++) if (srv_ch[j] >= 0) {
    float us = oc.servo_center_us[j] + o->servo[j] * oc.servo_us_per_rad[j];
    if (!(us == us)) us = oc.servo_center_us[j]; else us = us < 500 ? 500 : us > 2500 ? 2500 : us;   /* NaN → centre */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)srv_ch[j], srv_duty(us));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)srv_ch[j]);
  }
}
void hw_outputs_safe(void) {
  for (int i = 0; i < FC_MAX_MOTORS; i++) if (esc_ch[i] >= 0) {
    ledc_set_duty((ledc_mode_t)esc_mode[i], (ledc_channel_t)esc_ch[i], esc_duty(oc.esc_min_us));
    ledc_update_duty((ledc_mode_t)esc_mode[i], (ledc_channel_t)esc_ch[i]);
  }
}

/* ───────── battery ───────── */
static adc_oneshot_unit_handle_t adc; static adc_cali_handle_t cali; static adc_channel_t batt_ch; static float divider, vf;
int hw_battery_init(const hw_config *c) {
  if (c->batt_pin < 0) return 0;
  adc_unit_t unit;
  if (adc_oneshot_io_to_channel(c->batt_pin, &unit, &batt_ch) != ESP_OK || unit != ADC_UNIT_1) return -1;
  adc_oneshot_unit_init_cfg_t uc = { .unit_id = ADC_UNIT_1 };
  if (adc_oneshot_new_unit(&uc, &adc) != ESP_OK) return -1;
  adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
  adc_oneshot_config_channel(adc, batt_ch, &cc);
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  adc_cali_curve_fitting_config_t ccali = { .unit_id = ADC_UNIT_1, .chan = batt_ch, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
  if (adc_cali_create_scheme_curve_fitting(&ccali, &cali) != ESP_OK) cali = NULL;
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
  adc_cali_line_fitting_config_t lc = { .unit_id = ADC_UNIT_1, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
  if (adc_cali_create_scheme_line_fitting(&lc, &cali) != ESP_OK) cali = NULL;
#endif
  divider = c->batt_divider; return 0;
}
float hw_battery_read(void) {
  if (!adc) return 0;
  int mv = 0, raw = 0;
  if (cali) { if (adc_oneshot_get_calibrated_result(adc, cali, batt_ch, &mv) != ESP_OK) return vf; }
  else { if (adc_oneshot_read(adc, batt_ch, &raw) != ESP_OK) return vf; mv = raw * 3300 / 4095; }
  float v = mv / 1000.0f * divider;
  vf = vf == 0 ? v : vf + (v - vf) * 0.2f;   /* called at 20 Hz: ~0.25 s smoothing */
  return vf;
}
