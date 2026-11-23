/* Register-bus regression tests: run without an ESP or physical sensors. */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../runner/fc/esp32/main/bmp180.h"
#include "../runner/fc/esp32/main/hw.h"
#include "../runner/fc/esp32/main/tof250.h"
#include "../runner/fc/esp32/main/ina.h"
static uint8_t regs[128][256];static int fail;static int64_t now;
static int custom_read(int a,int r,uint8_t *b,int n) { if(fail)return -1;memcpy(b,regs[a]+r,(size_t)n);return 0; }
static int custom_write(int a,int r,int v) { if(fail)return -1;regs[a][r]=(uint8_t)v;return 0; }
static int64_t esp_timer_get_time(void){return now;}
#define pdMS_TO_TICKS(x) (x)
static void vTaskDelay(int ms){now+=ms*1000;}
#ifndef CUSTOM_SENSOR_HEADER
#define CUSTOM_SENSOR_HEADER "../runner/fc/esp32/main/custom_sensors.h"
#endif
#include CUSTOM_SENSOR_HEADER
static void set16(int a,int r,int v){regs[a][r]=(uint8_t)(v>>8);regs[a][r+1]=(uint8_t)v;}
int main(void) {
  (void)esp_timer_get_time();vTaskDelay(0);
  lb_bmp180_cal cal={408,-72,-14383,6190,4,-32768,-8711,2868,32741,32757,23153};int32_t pa;
  assert(!lb_bmp180_pressure(&cal,27898,23843,0,&pa));assert(pa==69964);
  lb_bmp180_cal bad=cal;bad.ac4=0;assert(lb_bmp180_pressure(&bad,27898,23843,0,&pa));
  assert(lb_bmp180_pressure(&cal,27898,23843,4,&pa));
  /* oversampling: the commands and waits of datasheet table 3; UP from the three bytes, shifted by 8 - oss */
  assert(lb_bmp180_cmd(0)==0x34 && lb_bmp180_cmd(3)==0xF4 && lb_bmp180_wait_us(0)==5000 && lb_bmp180_wait_us(3)==26000);
  {uint8_t r0[3]={0x5D,0x23,0x00}; assert(lb_bmp180_up(r0,0)==23843);
   uint8_t r3[3]={(uint8_t)(190744>>11),(uint8_t)(190744>>3),(uint8_t)(190744<<5)}; assert(lb_bmp180_up(r3,3)==190744);
   uint8_t hi[3]={0xFF,0xFF,0xE0}; assert(lb_bmp180_up(hi,3)==524287);}
  /* the same pressure at oss 3 (UP eight times as fine, b3 and b7 scaled to match): within a pascal of oss 0's */
  {int32_t p3; assert(!lb_bmp180_pressure(&cal,27898,23843<<3,3,&p3)); assert(p3>=pa-1 && p3<=pa+1);}
  regs[0x68][0x75]=0x68;assert(!custom_imu_init(0x68));
  assert(regs[0x68][0x37]==2 && regs[0x68][0x6a]==0); /* auxiliary compass bypass */
  set16(0x68,0x3b,4096);set16(0x68,0x43,16384);fc_imu m={0};assert(!custom_imu_read(&m));
  assert(m.have_gyro && fabsf(m.acc[0]-9.80665f)<0.001f && fabsf(m.gyro[0]-17.45329252f)<0.001f);
  fail=1;assert(custom_imu_read(&m) && !m.have_gyro);fail=0;
  regs[0x1e][0xa]='H';regs[0x1e][0xb]='4';regs[0x1e][0xc]='3';assert(!custom_mag_init(0x1e));
  regs[0x1e][9]=1;set16(0x1e,3,1090);set16(0x1e,5,-1090);set16(0x1e,7,545);float mag[3];assert(custom_mag_read(mag));
  assert(fabsf(mag[0]-100)<0.001f && fabsf(mag[1]-50)<0.001f && fabsf(mag[2]+100)<0.001f);
  set16(0x1e,3,-4096);assert(!custom_mag_read(mag));regs[0x1e][9]=0;assert(!custom_mag_read(mag));
  regs[0x77][0xd0]=0x55;
  int cs[]={408,-72,-14383,32741,32757,23153,6190,4,-32768,-8711,2868};for(int i=0;i<11;i++)set16(0x77,0xaa+2*i,cs[i]);
  assert(!custom_baro_init(0x77));float alt=999;
  assert(!custom_baro_read(&alt) && regs[0x77][0xf4]==0x2e);assert(!custom_baro_read(&alt));
  now+=5000;set16(0x77,0xf6,27898);assert(!custom_baro_read(&alt) && regs[0x77][0xf4]==0x34);
  now+=5000;set16(0x77,0xf6,23843);regs[0x77][0xf8]=0;assert(custom_baro_read(&alt) && fabsf(alt)<0.001f);
  assert(fabsf(lb_tof250_metres((uint8_t[]){0, 121}) - 1.21f) < 0.001f);
  assert(lb_tof250_metres((uint8_t[]){0, 5}) == 0.05f);
  assert(lb_tof250_metres((uint8_t[]){0, 250}) == 2.5f);
  assert(lb_tof250_metres((uint8_t[]){0x22, 0xb8}) < 0);  /* 8888, no return */
  assert(lb_tof250_metres((uint8_t[]){1, 0}) < 0);        /* 256 cm, outside the supported range */
  assert(lb_tof250_metres((uint8_t[]){0, 0}) < 0);
  /* power monitors (ina.h): the INA3221's shunt in 40 µV and bus in 8 mV steps (bits 15..3), the INA226's 2.5 µV and
   * 1.25 mV, the INA219's 10 µV and 4 mV (bus bits 15..3); each channel's registers; the configurations */
  assert(lb_ina_channels(LB_INA3221)==3 && lb_ina_channels(LB_INA219)==1 && lb_ina_channels(LB_INA226)==1 && !lb_ina_channels(220));
  assert(lb_ina_reg(LB_INA3221,0,0)==1 && lb_ina_reg(LB_INA3221,0,1)==2 && lb_ina_reg(LB_INA3221,2,0)==5 && lb_ina_reg(LB_INA3221,2,1)==6 && lb_ina_reg(LB_INA219,0,1)==2);
  { uint8_t sh[2]={0x03,0xE8};   /* 1000 → 125 steps of 40 µV = 5 mV: 1 A through 5 mΩ */
    assert(fabsf(lb_ina_volts(LB_INA3221,0,sh)-0.005f)<1e-7f);
    uint8_t neg[2]={0xFC,0x18};    /* −1000 → −5 mV (current the other way) */
    assert(fabsf(lb_ina_volts(LB_INA3221,0,neg)+0.005f)<1e-7f);
    uint8_t bus[2]={0x1F,0x40};    /* 8000 → 1000 × 8 mV = 8.0 V */
    assert(fabsf(lb_ina_volts(LB_INA3221,1,bus)-8.0f)<1e-5f);
    uint8_t b226[2]={0x0C,0x80};   /* 3200 × 1.25 mV = 4.0 V */
    assert(fabsf(lb_ina_volts(LB_INA226,1,b226)-4.0f)<1e-5f && fabsf(lb_ina_volts(LB_INA226,0,sh)-0.0025f)<1e-7f);
    uint8_t b219[2]={0x1F,0x40};   /* 8000 >> 3 = 1000 × 4 mV = 4.0 V */
    assert(fabsf(lb_ina_volts(LB_INA219,1,b219)-4.0f)<1e-5f && fabsf(lb_ina_volts(LB_INA219,0,sh)-0.01f)<1e-7f); }
  assert(lb_ina_config(LB_INA3221,3)==0x6327 && lb_ina_config(LB_INA3221,7)==0x7327 && lb_ina_config(LB_INA219,1)==0x39DF && lb_ina_config(LB_INA226,1)==0x4327);
  puts("Sensor driver tests passed: BMP180 datasheet vector, nonblocking conversion, MPU units/bypass, HMC axes/status/overflow, I2C failures and TOF250 register decoding.");
}
