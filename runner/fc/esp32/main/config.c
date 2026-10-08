/* Portable wiring defaults, settings validation and diagnostics. */
#include "hw.h"
#include "esp_board.h"
#include "radio_link.h"
#include "../../../esp_radio/radio_cfg.h"   /* (by its path: the wiring tests build this file on a PC) */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The defaults: no motors and no servos. A fresh board (or one whose saved wiring was refused) drives no pin until the
 * wiring says which: an ESC's idle pulse (1000 µs at 400 Hz: 40% duty) on a pin that turns out to be an H-bridge or
 * MOSFET input would spin that motor. The chip's usual pins (LB_MOTOR_PINS, LB_SERVO_PINS) are only suggested. */
void hw_defaults(hw_config *c) {
  memset(c, 0, sizeof *c);
  c->version = HW_VERSION;
  memset(c->motor_pin, -1, sizeof c->motor_pin); memset(c->servo_pin, -1, sizeof c->servo_pin);
  memset(c->motor_max_pct,100,sizeof c->motor_max_pct);c->brushed_hz=20000;
  c->esc_hz = 400; c->esc_min_us = 1000; c->esc_max_us = 2000;
  for (int j = 0; j < FC_MAX_JOINTS; j++) { c->servo_center_us[j] = 1500; c->servo_us_per_rad[j] = 500.0f / (float)(M_PI / 4); }   /* ±500 µs = ±45° */
  c->sda = LB_SDA; c->scl = LB_SCL;
  c->batt_pin = -1; c->batt_divider = 11.0f;
  c->rate_hz = LB_RATE; c->telem_hz = 20; c->vref = 16.0f; c->link_baud = 921600;
  c->mag_driver = -1; for (int i=0;i<3;i++) { c->mag_matrix[4*i]=1; c->mag_scale[i]=1; }
  c->crsf_rx = c->crsf_tx = -1; c->elrs_rate = 250; c->elrs_ratio = 4;
  c->radio_kind = RLINK_ELRS; c->radio_channel = 1; c->radio_opt = 0; strcpy(c->bind, RCFG_BIND_DEFAULT); c->radio_baud = 115200;
  for (int i = 0; i < 5; i++) c->nrf_pin[i] = -1;
  c->radio_kbps = 1000;
  c->radio2_kind = -1; c->radio2_a = c->radio2_b = 0;
  c->peer_channel = 0; strcpy(c->fleet, RCFG_BIND_DEFAULT);
}
int hw_peers(const hw_config *c) { return c->peer_channel; }
int hw_outputs_wired(const hw_config *c) {
  int n = 0; for (int i = 0; i < FC_MAX_MOTORS; i++) n += c->motor_pin[i] >= 0; for (int j = 0; j < FC_MAX_JOINTS; j++) n += c->servo_pin[j] >= 0;
  return n;
}
/* What usually answers at an I2C address: a guess from the address alone ("show" says what was identified). */
const char *hw_i2c_name(int a) {
  switch (a) {
    case 0x0C: return "AK8963 compass (an MPU-9250's): no driver";
    case 0x0D: return "QMC5883L compass: no driver yet";
    case 0x1E: return "HMC5883L compass";
    case 0x18: case 0x19: return "LIS3DH accelerometer";
    case 0x29: return "VL53L0X rangefinder: no driver";
    case 0x3C: case 0x3D: return "OLED display";
    case 0x40: return "INA219 or PCA9685";
    case 0x48: case 0x49: case 0x4A: case 0x4B: return "ADS1115 or a temperature sensor";
    case 0x53: return "ADXL345 accelerometer: no driver";
    case 0x68: case 0x69: return "MPU-6050 family IMU (or a clock chip)";
    case 0x76: return "BMP280/BME280 barometer";
    case 0x77: return "BMP180, BMP280 or BME280 barometer";
    default: return "unknown";
  }
}
int hw_radio2(const hw_config *c, rlink_cfg *L) { return c->radio2_kind < 0 ? -1 : rlink_make(L, c->radio2_kind, (int)c->radio2_a, (int)c->radio2_b); }
void hw_radio(const hw_config *c, rlink_cfg *L) {
  int e = c->radio_kind == RLINK_ESPNOW ? rlink_make(L, RLINK_ESPNOW, c->radio_channel, c->radio_opt)
        : c->radio_kind == RLINK_WIFI ? rlink_make(L, RLINK_WIFI, c->radio_opt, c->radio_channel)
        : c->radio_kind == RLINK_SERIAL ? rlink_make(L, RLINK_SERIAL, (int)c->radio_baud, c->radio_opt)
        : c->radio_kind == RLINK_NRF24 ? rlink_make(L, RLINK_NRF24, c->radio_kbps, 0)
        : c->radio_kind == RLINK_BLE ? rlink_make(L, RLINK_BLE, 0, 0)
        : rlink_make(L, RLINK_ELRS, c->elrs_rate, c->elrs_ratio);
  if (e) rlink_default(L);
}
static void radio_put(hw_config *c, const rlink_cfg *L) {
  c->radio_kind = (int8_t)L->kind;
  if (L->kind == RLINK_ELRS) { c->elrs_rate = (int16_t)L->rate_hz; c->elrs_ratio = (int16_t)L->ratio; }
  else if (L->kind == RLINK_ESPNOW) { c->radio_channel = (int8_t)L->channel; c->radio_opt = (int8_t)L->lr; }
  else if (L->kind == RLINK_SERIAL) { int a, b; rlink_args(L, &a, &b); c->radio_baud = L->baud; c->radio_opt = (int8_t)b; }   /* (b: half, up or down) */
  else if (L->kind == RLINK_NRF24) c->radio_kbps = (int16_t)L->kbps;
  else if (L->kind == RLINK_BLE) {}
  else { c->radio_opt = (int8_t)L->sta; if (!L->sta) c->radio_channel = (int8_t)L->channel; }   /* (wifi,sta keeps the channel an access point had) */
}
static int parse_list(const char *s, float *v, int max) {
  int n = 0; char *end;
  while (*s && n < max) { v[n] = strtof(s, &end); if (end == s) return -1; n++; s = end; while (*s == ',' || *s == ' ') s++; }
  return *s ? -1 : n;
}
/* GPIOs an ESP32 (WROOM) can drive an output on: not the flash pins (6–11), not UART0 (1, 3), not input-only (34–39),
 * and not the boot-strapping pins (0, 2, 5, 12, 15): something wired there can stop it booting, and some of them
 * toggle during boot, which an ESC could take as a pulse. */
static int pin_ok(int p) { return p == -1 || lb_output_pin(p); }
/* The wiring as a whole: no pin used twice, the ESC's longest pulse fits its period. */
int hw_check(const hw_config *c, char *err, int errn) {
  for (int i = 0; i < FC_MAX_MOTORS; i++) if (!pin_ok(c->motor_pin[i])) { snprintf(err, errn, "reserved motor pin"); return -1; }
  for (int i = 0; i < FC_MAX_JOINTS; i++) if (!pin_ok(c->servo_pin[i])) { snprintf(err, errn, "reserved servo pin"); return -1; }
  if (!lb_output_pin(c->sda) || !lb_output_pin(c->scl) || (c->batt_pin >= 0 && !lb_adc_pin(c->batt_pin)) ||
      (c->crsf_rx >= 0 && !lb_input_pin(c->crsf_rx)) || (c->crsf_tx >= 0 && !lb_output_pin(c->crsf_tx))) {
    snprintf(err, errn, "reserved sensor or radio pin"); return -1;
  }
  const int ds[3]={c->imu_driver,c->baro_driver,c->mag_driver}; const int as[3]={c->imu_addr,c->baro_addr,c->mag_addr};
  if(c->mag_driver==2) {snprintf(err,errn,"unknown compass driver");return -1;}
  for(int i=0;i<3;i++) { if(ds[i]<-1 || ds[i]>3 || (as[i] && (as[i]<8 || as[i]>119))) { snprintf(err,errn,"invalid sensor driver/address"); return -1; }
    for(int j=0;j<i;j++) if(ds[i]>=0 && ds[j]>=0 && as[i] && as[i]==as[j]) { snprintf(err,errn,"I2C address used twice"); return -1; } }
  for(int i=0;i<3;i++) if(!isfinite(c->mag_bias[i]) || !isfinite(c->mag_scale[i]) || c->mag_scale[i]<0.01f || c->mag_scale[i]>100) { snprintf(err,errn,"invalid compass calibration"); return -1; }
  for(int i=0;i<9;i++) if(!isfinite(c->mag_matrix[i]) || fabsf(c->mag_matrix[i])>1.001f) { snprintf(err,errn,"invalid compass rotation");return -1; }
  for(int i=0;i<3;i++) for(int j=0;j<3;j++) { float dot=0;for(int k=0;k<3;k++) dot+=c->mag_matrix[3*i+k]*c->mag_matrix[3*j+k]; if(fabsf(dot-(i==j?1:0))>0.01f) { snprintf(err,errn,"compass matrix must be orthonormal");return -1; } }
  { rlink_cfg L; int k = c->radio_kind;
    if (k == RLINK_ELRS ? rlink_make(&L, k, c->elrs_rate, c->elrs_ratio) : k == RLINK_ESPNOW ? rlink_make(&L, k, c->radio_channel, c->radio_opt)
        : k == RLINK_WIFI ? rlink_make(&L, k, c->radio_opt, c->radio_channel) : k == RLINK_SERIAL ? rlink_make(&L, k, (int)c->radio_baud, c->radio_opt) : k == RLINK_NRF24 ? rlink_make(&L, k, c->radio_kbps, 0) : k == RLINK_BLE ? rlink_make(&L, k, 0, 0) : -1) { snprintf(err, errn, "invalid radio link"); return -1; }
    char t[RCFG_BIND_N];
    if (!rcfg_terminated(c->bind, sizeof c->bind) || (c->bind[0] && rcfg_bind_parse(t, c->bind, err, errn)))   /* (empty: the default) */
      { snprintf(err, errn, "invalid binding phrase"); return -1; }
    if (!rcfg_terminated(c->wifi_ssid, sizeof c->wifi_ssid) || !rcfg_terminated(c->wifi_pass, sizeof c->wifi_pass) || (c->wifi_pass[0] && !c->wifi_ssid[0])) { snprintf(err, errn, "invalid Wi-Fi network"); return -1; }
    if (c->radio2_kind >= 0) {                                       /* a second link: valid, and possible beside the first */
      rlink_cfg L2; char why[120];
      if (hw_radio2(c, &L2)) { snprintf(err, errn, "invalid second radio link"); return -1; }
      if (rlink_pair_ok(&L, &L2, 1, why, sizeof why)) { snprintf(err, errn, "radio2: %s", why); return -1; }
      if (L2.kind == RLINK_SERIAL && (c->crsf_rx < 0 || c->crsf_tx < 0 || c->crsf_rx == c->crsf_tx)) { snprintf(err, errn, "radio2=serial: set crsf=RX,TX first: the line's pins"); return -1; }
      if (L2.kind == RLINK_NRF24 && c->nrf_pin[0] < 0) { snprintf(err, errn, "radio2=nrf24: set nrf24=SCK,MOSI,MISO,CSN,CE first"); return -1; }
      if (L2.kind == RLINK_ELRS && c->crsf_rx < 0) { snprintf(err, errn, "radio2=elrs: set crsf=RX,TX first: the receiver's pins"); return -1; }
    } else if (!rlink_up(&L)) { snprintf(err, errn, "radio=%s carries nothing up: the drone would get no channels (as a second link, radio2=, it can)", "serial,...,down"); return -1; }
  }
  if (c->peer_channel) {                                            /* the other drones: on ESP-NOW, beside the links */
    rlink_cfg L[2]; int nl = 0; hw_radio(c, &L[nl++]); if (!hw_radio2(c, &L[nl])) nl++;
    if (c->peer_channel < 1 || c->peer_channel > 13) { snprintf(err, errn, "peers: a Wi-Fi channel, 1 to 13, or peers=off"); return -1; }
    for (int i = 0; i < nl; i++) {
      if (L[i].kind == RLINK_ESPNOW && L[i].channel != c->peer_channel) { snprintf(err, errn, "peers=%d: the ESP-NOW link is on channel %d: the same, please (they share the radio)", c->peer_channel, L[i].channel); return -1; }
      if (L[i].kind == RLINK_WIFI || L[i].kind == RLINK_BLE) { snprintf(err, errn, "peers: not beside a %s link (yet): ESP-NOW, ExpressLRS, nRF24 or a serial line", L[i].kind == RLINK_WIFI ? "Wi-Fi" : "Bluetooth"); return -1; }
    }
  }
  { char t[RCFG_BIND_N]; if (!rcfg_terminated(c->fleet, sizeof c->fleet) || (c->fleet[0] && rcfg_bind_parse(t, c->fleet, err, errn))) { snprintf(err, errn, "invalid fleet phrase"); return -1; } }
  if(c->esc_hz<50 || c->esc_hz>490 || c->esc_min_us<800 || c->esc_max_us>2200 || c->esc_max_us-c->esc_min_us<500) {snprintf(err,errn,"invalid ESC pulse timing");return -1;}
  if(c->brushed_hz<1000 || c->brushed_hz>30000) {snprintf(err,errn,"brushed_hz: 1000 to 30000 Hz");return -1;}
  for(int i=0;i<FC_MAX_MOTORS;i++)if(c->motor_driver[i]>1 || c->motor_max_pct[i]<1 || c->motor_max_pct[i]>100) {snprintf(err,errn,"invalid motor driver/duty ceiling");return -1;}
  int used[SOC_GPIO_PIN_COUNT] = { 0 };
  #define USE(p, what) do { int p_ = (p); if (p_ >= 0 && p_ < SOC_GPIO_PIN_COUNT) { if (used[p_]) { snprintf(err, errn, "GPIO %d is used twice (%s)", p_, what); return -1; } used[p_] = 1; } } while (0)
  for (int i = 0; i < FC_MAX_MOTORS; i++) USE(c->motor_pin[i], "motor");
  for (int j = 0; j < FC_MAX_JOINTS; j++) USE(c->servo_pin[j], "servo");
  USE(c->sda, "I2C"); USE(c->scl, "I2C"); USE(c->batt_pin, "battery"); USE(c->crsf_rx, "radio receiver"); USE(c->crsf_tx, "radio receiver");
  for (int i = 0; i < 5; i++) USE(c->nrf_pin[i], "nRF24L01");
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
  if (!strcmp(key, "radio") || !strcmp(key, "elrs")) {   /* the radio link: radio=elrs,250,4 | espnow,CH[,lr] | wifi,ap,CH | wifi,sta | serial,BAUD[,half] (elrs=250,4 as before) */
    rlink_cfg L; if (rlink_parse(&L, eq + 1, err, errn)) return -1;
    if (key[0] == 'e' && L.kind != RLINK_ELRS) { snprintf(err, errn, "elrs=rate,ratio; for the other links radio="); return -1; }
    if (L.kind == RLINK_SERIAL && (c->crsf_rx < 0 || c->crsf_tx < 0 || c->crsf_rx == c->crsf_tx)) { snprintf(err, errn, "radio=serial: set crsf=RX,TX first: the pins from the line's output and to its input"); return -1; }
    if (L.kind == RLINK_NRF24 && c->nrf_pin[0] < 0) { snprintf(err, errn, "radio=nrf24: set nrf24=SCK,MOSI,MISO,CSN,CE first: the module's pins"); return -1; }
    radio_put(c, &L); return 0;
  }
  if (!strcmp(key, "radio2")) {                                      /* a second link at once: radio2=KIND,... as radio=, or radio2=none */
    if (!strcmp(eq + 1, "none") || !strcmp(eq + 1, "-1") || !eq[1]) { c->radio2_kind = -1; return 0; }
    rlink_cfg L; if (rlink_parse(&L, eq + 1, err, errn)) return -1;
    int a, b; rlink_args(&L, &a, &b); c->radio2_kind = (int8_t)L.kind; c->radio2_a = a; c->radio2_b = b; return 0;
  }
  if (!strcmp(key, "bind")) return rcfg_bind_parse(c->bind, eq + 1, err, errn);   /* the binding phrase */
  if (!strcmp(key, "fleet")) {                                       /* the fleet phrase: the other drones' */
    if (rcfg_bind_parse(c->fleet, eq + 1, err, errn)) { snprintf(err, errn, "fleet=PHRASE: 1 to 31 characters, the same on each drone of the fleet"); return -1; }
    return 0;
  }
  if (!strcmp(key, "peers")) {                                       /* the other drones: peers=CHANNEL, or peers=off */
    if (!strcmp(eq + 1, "off") || !strcmp(eq + 1, "0") || !eq[1]) { c->peer_channel = 0; return 0; }
    char *end; long ch = strtol(eq + 1, &end, 10);
    if (*end || ch < 1 || ch > 13) { snprintf(err, errn, "peers=CHANNEL (1 to 13: an ESP-NOW link's own), or peers=off"); return -1; }
    c->peer_channel = (int8_t)ch; return 0;
  }
  if (!strcmp(key, "wifi")) return rcfg_wifi_parse(c->wifi_ssid, c->wifi_pass, eq + 1, err, errn);   /* wifi=SSID,PASSWORD */
  n = parse_list(eq + 1, v, FC_MAX_MOTORS);
  for(int i=0;i<n;i++)if(!isfinite(v[i]))n=-1;
  if (n < 0) { snprintf(err, errn, "%s: not a list of numbers", key); return -1; }
  if (!strcmp(key,"imu") || !strcmp(key,"baro") || !strcmp(key,"mag")) {
    if(n!=2 || v[0]!=(int)v[0] || v[0]<-1 || v[0]>3 || v[1]!=(int)v[1] || (v[1]!=0 && (v[1]<8 || v[1]>119))) { snprintf(err,errn,"sensor=driver,address (-1 off,0 auto,1/2 named,3 custom)");return -1; }
    int8_t *d=!strcmp(key,"imu")?&c->imu_driver:!strcmp(key,"baro")?&c->baro_driver:&c->mag_driver;
    uint8_t *a=!strcmp(key,"imu")?&c->imu_addr:!strcmp(key,"baro")?&c->baro_addr:&c->mag_addr; *d=(int8_t)v[0];*a=(uint8_t)v[1];
  } else if (!strcmp(key,"mag_matrix") || !strcmp(key,"mag_bias") || !strcmp(key,"mag_scale")) {
    int want=!strcmp(key,"mag_matrix")?9:3; if(n!=want) { snprintf(err,errn,"expected %d compass values",want);return -1; }
    float *out=want==9?c->mag_matrix:!strcmp(key,"mag_bias")?c->mag_bias:c->mag_scale;memcpy(out,v,(size_t)want*sizeof(float));
  } else if (!strcmp(key, "motors") || !strcmp(key, "servos")) {
    int m = !strcmp(key, "motors"), max = m ? FC_MAX_MOTORS : FC_MAX_JOINTS; int8_t *pins = m ? c->motor_pin : c->servo_pin;
    if (n > max) { snprintf(err, errn, "at most %d %s", max, key); return -1; }
    for (int i = 0; i < n; i++) if (v[i] < 0 || !pin_ok((int)v[i])) { snprintf(err, errn, "GPIO %d is reserved or cannot drive an output on this chip", (int)v[i]); return -1; }
    for (int i = 0; i < max; i++) pins[i] = i < n ? (int8_t)v[i] : -1;
  } else if (!strcmp(key,"motor_driver") || !strcmp(key,"motor_max")) {
    int driver=!strcmp(key,"motor_driver");uint8_t *dst=driver?c->motor_driver:c->motor_max_pct;
    for(int i=0;i<n;i++)if(v[i]<(driver?0:1) || v[i]>(driver?1:100) || v[i]!=(int)v[i]) {snprintf(err,errn,driver?"motor_driver: 0 ESC, 1 MOSFET":"motor_max: integer duty percentages 1–100");return -1;}
    for(int i=0;i<FC_MAX_MOTORS;i++)dst[i]=i<n?(uint8_t)v[i]:(driver?0:100);
  } else if (!strcmp(key,"brushed_hz")) {
    if(n!=1 || v[0]<1000 || v[0]>30000 || v[0]!=(int)v[0]) {snprintf(err,errn,"brushed_hz: integer 1000 to 30000 Hz");return -1;}
    c->brushed_hz=(int32_t)v[0];
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

  } else if (!strcmp(key, "nrf24")) {                 /* an nRF24L01's pins: nrf24=SCK,MOSI,MISO,CSN,CE, or nrf24=-1 */
    if (n == 1 && v[0] == -1) for (int i = 0; i < 5; i++) c->nrf_pin[i] = -1;
    else {
      int ok = n == 5; for (int i = 0; i < 5 && ok; i++) ok = v[i] >= 0 && (i == 2 ? lb_input_pin((int)v[i]) : pin_ok((int)v[i]));
      if (!ok) { snprintf(err, errn, "nrf24=SCK,MOSI,MISO,CSN,CE: free pins (MISO an input; the others outputs), or nrf24=-1"); return -1; }
      for (int i = 0; i < 5; i++) c->nrf_pin[i] = (int8_t)v[i];
    }

  } else if (!strcmp(key, "telemetry")) {
    if (n != 1 || v[0] < 0 || v[0] > 50) { snprintf(err, errn, "telemetry: 0 to 50 Hz"); return -1; }
    c->telem_hz = (int16_t)v[0];
  } else { snprintf(err, errn, "unknown setting %s", key); return -1; }
  return 0;
}
void hw_describe(const hw_config *c, char *out, int n) {
  int k=0;
  #define APP(...) do {if(k<n){int w=snprintf(out+k,(size_t)(n-k),__VA_ARGS__);if(w>0)k+=w<n-k?w:n-k;}}while(0)
  APP("motors=");for(int i=0;i<FC_MAX_MOTORS && c->motor_pin[i]>=0;i++)APP("%s%d",i?",":"",c->motor_pin[i]);
  APP(" motor_driver=");for(int i=0;i<FC_MAX_MOTORS && c->motor_pin[i]>=0;i++)APP("%s%u",i?",":"",c->motor_driver[i]);
  APP(" motor_max=");for(int i=0;i<FC_MAX_MOTORS && c->motor_pin[i]>=0;i++)APP("%s%u",i?",":"",c->motor_max_pct[i]);
  APP(" brushed_hz=%ld servos=",(long)c->brushed_hz);for(int i=0;i<FC_MAX_JOINTS && c->servo_pin[i]>=0;i++)APP("%s%d",i?",":"",c->servo_pin[i]);
  char rl[32]; rlink_cfg L; hw_radio(c,&L); rlink_describe(&L,rl,sizeof rl);
  APP(" esc_hz=%d esc_us=%d,%d i2c=%d,%d battery=%d,%.1f vref=%.1f rate=%d telemetry=%d baud=%ld crsf=%d,%d radio=%s",
      c->esc_hz,c->esc_min_us,c->esc_max_us,c->sda,c->scl,c->batt_pin,(double)c->batt_divider,(double)c->vref,c->rate_hz,c->telem_hz,(long)c->link_baud,c->crsf_rx,c->crsf_tx,rl);
  { rlink_cfg L2; if (!hw_radio2(c,&L2)) { rlink_describe(&L2,rl,sizeof rl); APP(" radio2=%s",rl); } else APP(" radio2=none"); }
  if (c->nrf_pin[0] >= 0) APP(" nrf24=%d,%d,%d,%d,%d",c->nrf_pin[0],c->nrf_pin[1],c->nrf_pin[2],c->nrf_pin[3],c->nrf_pin[4]); else APP(" nrf24=-1");
  /* the packet links' settings: the binding phrase and the password masked (show goes wherever the link goes) */
  if (rcfg_bind_default(c->bind)) APP(" bind=%s(the default: set your own)",RCFG_BIND_DEFAULT); else APP(" bind=(set, %d characters)",(int)strlen(c->bind));
  if (c->peer_channel) APP(" peers=%d",c->peer_channel); else APP(" peers=off");
  if (rcfg_bind_default(c->fleet)) APP(" fleet=%s(the default)",RCFG_BIND_DEFAULT); else APP(" fleet=(set, %d characters)",(int)strlen(c->fleet));
  if (c->wifi_ssid[0]) APP(" wifi=%s,%s",c->wifi_ssid,c->wifi_pass[0]?"********":"(default password)"); else APP(" wifi=(default)");
  APP(" servo_center=");
  for(int i=0;i<FC_MAX_JOINTS && c->servo_pin[i]>=0;i++)APP("%s%d",i?",":"",c->servo_center_us[i]);
  APP(" servo_us_per_rad=");for(int i=0;i<FC_MAX_JOINTS && c->servo_pin[i]>=0;i++)APP("%s%.0f",i?",":"",(double)c->servo_us_per_rad[i]);
  #undef APP
}

