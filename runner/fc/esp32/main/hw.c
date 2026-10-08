/* The flight controller's hardware: see hw.h. */
#include "hw.h"
#include "hw_migration.h"
#include "esp_board.h"
#include "bmp180.h"
#include "esp_timer.h"
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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define G 9.80665f

/* ───────── the wiring ───────── */
int hw_load(hw_config *c) {
  hw_defaults(c);
  nvs_handle_t h; if (nvs_open("dfb", NVS_READONLY, &h) != ESP_OK) return 0;
  hw_config t = {0}; size_t n = sizeof t;
  if(nvs_get_blob(h,"hw",&t,&n)==ESP_OK)lb_hw_restore(c,&t,n);
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

/* ───────── sensors ───────── */
static i2c_master_bus_handle_t imu_i2c_bus;
static i2c_master_dev_handle_t imu_dev, baro_dev, mag_dev;
static hw_config sensor_config;
static int custom_imu,custom_baro,custom_mag;
static int imu_kind, baro_kind;
static float gyro_bias[3];
static int wr(i2c_master_dev_handle_t d, uint8_t reg, uint8_t v) { uint8_t b[2] = { reg, v }; return i2c_master_transmit(d, b, 2, 5) == ESP_OK ? 0 : -1; }
static int rd(i2c_master_dev_handle_t d, uint8_t reg, uint8_t *buf, int n) { return i2c_master_transmit_receive(d, &reg, 1, buf, (size_t)n, 5) == ESP_OK ? 0 : -1; }
static i2c_master_dev_handle_t add(uint8_t addr) {
  i2c_device_config_t dc = { .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = 400000 };
  i2c_master_dev_handle_t d = NULL; if (i2c_master_bus_add_device(imu_i2c_bus, &dc, &d) != ESP_OK) return NULL; return d;
}

/* BMP280 calibration and state */
static struct { uint16_t T1, P1; int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9; } bc;
static float p0;
static lb_bmp180_cal b180;
static int b180_phase; static int32_t b180_ut; static int64_t b180_due;
static i2c_master_dev_handle_t custom_devices[128];
static int custom_read(int addr,int reg,uint8_t *buf,int n) {
  if(addr<8 || addr>119 || reg<0 || reg>255 || n<1 || n>64) return -1;
  if(!custom_devices[addr]) custom_devices[addr]=add((uint8_t)addr);
  return custom_devices[addr]?rd(custom_devices[addr],(uint8_t)reg,buf,n):-1;
}
static int custom_write(int addr,int reg,int value) {
  if(addr<8 || addr>119 || reg<0 || reg>255 || value<0 || value>255) return -1;
  if(!custom_devices[addr]) custom_devices[addr]=add((uint8_t)addr);
  return custom_devices[addr]?wr(custom_devices[addr],(uint8_t)reg,(uint8_t)value):-1;
}
#include "custom_sensors.h"

int hw_i2c_scan(char *out, int n) {
  int k = 0, found = 0, mpu = 0, mag = 0;
  #define APP(...) do { if (k < n) { int w = snprintf(out + k, (size_t)(n - k), __VA_ARGS__); if (w > 0) k += w < n - k ? w : n - k; } } while (0)
  APP("I2C on SDA %d, SCL %d:", sensor_config.sda, sensor_config.scl);
  if (!imu_i2c_bus) { APP(" the bus didn't start"); return 0; }
  for (int a = 0x08; a <= 0x77; a++) {
    if (i2c_master_probe(imu_i2c_bus, (uint16_t)a, 10) == ESP_OK) {
      found++; APP(" 0x%02x %s;", a, hw_i2c_name(a));
      mpu |= a == 0x68 || a == 0x69; mag |= a == 0x0C || a == 0x0D || a == 0x1E;
    }
    vTaskDelay(1);                                           /* (the sensor task's reads come in between) */
  }
  if (!found) APP(" nothing answers: check SDA/SCL, 3.3 V, ground and the pull-ups");
  else if (mpu && !mag && imu_kind != 1) APP(" (a GY-87's compass sits behind the MPU: it answers once the IMU driver is on, imu=0,0)");
  #undef APP
  return found;
}

int hw_sensors_init(const hw_config *c, hw_sensors *s, char *log, int logn) {
  memset(s, 0, sizeof *s); int k = 0; sensor_config=*c;
  i2c_master_bus_config_t bcfg = { .i2c_port = I2C_NUM_0, .sda_io_num = c->sda, .scl_io_num = c->scl, .clk_source = I2C_CLK_SRC_DEFAULT,
                                   .glitch_ignore_cnt = 7, .flags.enable_internal_pullup = true };
  if (i2c_new_master_bus(&bcfg, &imu_i2c_bus) != ESP_OK) { snprintf(log, logn, "I2C bus on SDA %d, SCL %d didn't start", c->sda, c->scl); return -1; }
  if(c->imu_driver==3 && !custom_imu_init(c->imu_addr)) { custom_imu=1;imu_kind=3;s->imu=3;snprintf(s->imu_name,sizeof s->imu_name,"custom C at 0x%02x",c->imu_addr); }
  if(c->baro_driver==3 && !custom_baro_init(c->baro_addr)) { custom_baro=1;baro_kind=3;s->baro=3;snprintf(s->baro_name,sizeof s->baro_name,"custom C"); }
  if(c->mag_driver==3 && !custom_mag_init(c->mag_addr)) { custom_mag=1;s->mag=3;snprintf(s->mag_name,sizeof s->mag_name,"custom C"); }
  /* The IMU: an MPU-6050 family chip at 0x68/0x69 (MPU-6050, MPU-6500, MPU-9250), else a LIS3DH at 0x18/0x19. */
  for (uint8_t a = 0x68; a <= 0x69 && !imu_kind && (c->imu_driver==0 || c->imu_driver==1); a++) {
    if(c->imu_addr && c->imu_addr!=a) continue;
    if (i2c_master_probe(imu_i2c_bus, a, 20) != ESP_OK) continue;
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
    /* GY-87-style modules put the compass on the MPU's auxiliary bus. */
    if(wr(d,0x6A,0x00) || wr(d,0x37,0x02)) { k+=snprintf(log+k,logn-k,"MPU bypass failed; "); }
    imu_dev = d; imu_kind = 1; s->imu = 1; snprintf(s->imu_name, sizeof s->imu_name, "%s at 0x%02x", nm, a);
  }
  for (uint8_t a = 0x18; a <= 0x19 && !imu_kind && (c->imu_driver==0 || c->imu_driver==2); a++) {
    if(c->imu_addr && c->imu_addr!=a) continue;
    if (i2c_master_probe(imu_i2c_bus, a, 20) != ESP_OK) continue;
    i2c_master_dev_handle_t d = add(a); uint8_t who = 0;
    if (!d || rd(d, 0x0F, &who, 1) || who != 0x33) continue;
    wr(d, 0x20, 0x97);                                        /* 1.344 kHz, x y z on */
    wr(d, 0x23, 0x28);                                        /* ±8 g, high resolution */
    uint8_t r1 = 0, r4 = 0; if (rd(d, 0x20, &r1, 1) || rd(d, 0x23, &r4, 1) || r1 != 0x97 || (r4 & 0x38) != 0x28) { k += snprintf(log + k, logn - k, "LIS3DH didn't take its settings; "); continue; }
    imu_dev = d; imu_kind = 2; s->imu = 2; snprintf(s->imu_name, sizeof s->imu_name, "LIS3DH at 0x%02x (no gyro)", a);
  }
  if (!imu_kind) k += snprintf(log + k, logn - k, "no IMU found on I2C (SDA %d, SCL %d); ", c->sda, c->scl);
  /* The barometer: BMP280 or BME280 at 0x76/0x77. */
  for (uint8_t a = 0x76; a <= 0x77 && !baro_kind && (c->baro_driver==0 || c->baro_driver==1); a++) {
    if (i2c_master_probe(imu_i2c_bus, a, 20) != ESP_OK) continue;
    if(c->baro_addr && c->baro_addr!=a) continue;
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
  if(!baro_kind && (c->baro_driver==0 || c->baro_driver==2) && (!c->baro_addr || c->baro_addr==0x77)) {
    i2c_master_dev_handle_t d=add(0x77);uint8_t id=0,cal[22];
    if(d && !rd(d,0xD0,&id,1) && id==0x55 && !rd(d,0xAA,cal,22)) {
      int valid=1;for(int i=0;i<22;i+=2) if((cal[i]==0 && cal[i+1]==0) || (cal[i]==255 && cal[i+1]==255)) valid=0;
      if(valid) {
        #define B16(i) ((uint16_t)(cal[i]<<8|cal[i+1]))
        b180=(lb_bmp180_cal){.ac1=(int16_t)B16(0),.ac2=(int16_t)B16(2),.ac3=(int16_t)B16(4),.ac4=B16(6),.ac5=B16(8),.ac6=B16(10),.b1=(int16_t)B16(12),.b2=(int16_t)B16(14),.mb=(int16_t)B16(16),.mc=(int16_t)B16(18),.md=(int16_t)B16(20)};
        #undef B16
        baro_dev=d;baro_kind=2;s->baro=2;snprintf(s->baro_name,sizeof s->baro_name,"BMP180 at 0x77");
      }
    }
  }
  if(c->mag_driver==0 || c->mag_driver==1) {
    int addr=c->mag_addr?c->mag_addr:0x1e; i2c_master_dev_handle_t d=add((uint8_t)addr);uint8_t id[3];
    if(d && !rd(d,0x0A,id,3) && id[0]=='H' && id[1]=='4' && id[2]=='3' && !wr(d,0,0x78) && !wr(d,1,0x20) && !wr(d,2,0)) { mag_dev=d;s->mag=1;snprintf(s->mag_name,sizeof s->mag_name,"HMC5883L at 0x%02x",addr); }
    else k+=snprintf(log+k,logn-k,"HMC5883L not found; ");
  }
    if (k == 0 && log) log[0] = 0;
  return imu_kind ? 0 : -1;
}

int hw_imu_read(fc_imu *m) {
  uint8_t b[14];
  m->have_gyro = 0;
  if(custom_imu) { int e=custom_imu_read(m);if(!e && m->have_gyro) for(int i=0;i<3;i++) m->gyro[i]-=gyro_bias[i];return e; }
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
  if(custom_baro) return custom_baro_read(alt);
  if (!baro_kind) return 0;
  if(baro_kind==2) {
    int64_t now=esp_timer_get_time();uint8_t b[3];
    if(b180_phase && now<b180_due) return 0;
    if(!b180_phase) { if(wr(baro_dev,0xF4,0x2e)) return 0; b180_phase=1;b180_due=now+5000;return 0; }
    if(b180_phase==1) { if(rd(baro_dev,0xF6,b,2) || wr(baro_dev,0xF4,0x34)) {b180_phase=0;return 0;} b180_ut=b[0]<<8|b[1];b180_phase=2;b180_due=now+5000;return 0; }
    b180_phase=0;if(rd(baro_dev,0xF6,b,3)) return 0;int32_t pa;
    if(lb_bmp180_pressure(&b180,b180_ut,(b[0]<<16|b[1]<<8|b[2])>>8,0,&pa)) return 0;
    if(p0==0) { p0=(float)pa; }
    *alt=44330.0f*(1-powf((float)pa/p0,0.190295f));return 1;
  }
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

int hw_mag_read(float mag[3]) {
  float raw[3];
  if(custom_mag) { if(!custom_mag_read(raw)) return 0; }
  else { if(!mag_dev) return 0;uint8_t status,b[6];if(rd(mag_dev,9,&status,1) || !(status&1) || rd(mag_dev,3,b,6)) return 0;
    int16_t x=(int16_t)(b[0]<<8|b[1]),z=(int16_t)(b[2]<<8|b[3]),y=(int16_t)(b[4]<<8|b[5]);if(x==-4096 || y==-4096 || z==-4096) return 0;
    raw[0]=x*(100.0f/1090);raw[1]=y*(100.0f/1090);raw[2]=z*(100.0f/1090); }
  for(int i=0;i<3;i++) { if(!isfinite(raw[i])) return 0;raw[i]=(raw[i]-sensor_config.mag_bias[i])*sensor_config.mag_scale[i]; }
  for(int i=0;i<3;i++) { mag[i]=0;for(int j=0;j<3;j++) mag[i]+=sensor_config.mag_matrix[3*i+j]*raw[j]; }
  return 1;
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
