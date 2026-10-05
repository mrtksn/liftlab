/* Register-bus regression tests: run without an ESP or physical sensors. */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../runner/fc/esp32/main/bmp180.h"
#include "../runner/fc/esp32/main/hw.h"
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
  puts("Sensor driver tests passed: BMP180 datasheet vector, nonblocking conversion, MPU units/bypass, HMC axes/status/overflow, I2C failures.");
}
