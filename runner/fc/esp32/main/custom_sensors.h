/* LiftLab custom sensor drivers. Replace this file with the editor's export and
 * rebuild firmware. Enable driver=3 for each custom sensor. Inputs: 7-bit I2C
 * address; outputs: gyro rad/s, acceleration m/s², altitude m, compass microtesla.
 * custom_read(addr,reg,buffer,length), custom_write(addr,reg,value) return 0 on
 * success. esp_timer_get_time() returns microseconds. Reads must not block/delay:
 * use a state machine for conversion waits; return 0 when no new baro/mag sample.
 * C drivers are trusted native firmware and require a full rebuild, not RNP upload.
 */
#ifndef LIFTLAB_CUSTOM_SENSORS_H
#define LIFTLAB_CUSTOM_SENSORS_H
// BEGIN IMU
static int ci_addr;
static int custom_imu_init(int addr) {
  ci_addr=addr?addr:0x68;uint8_t who;
  if(custom_read(ci_addr,0x75,&who,1) || who!=0x68) return -1;
  if(custom_write(ci_addr,0x6B,0x80)) return -1;
  vTaskDelay(pdMS_TO_TICKS(100)); /* init only, before the flight task starts */
  const uint8_t settings[][2]={{0x6B,1},{0x1A,2},{0x19,0},{0x1B,0x18},{0x1C,0x10},{0x6A,0},{0x37,2}};
  for(unsigned i=0;i<sizeof settings/sizeof settings[0];i++) {
    uint8_t back;if(custom_write(ci_addr,settings[i][0],settings[i][1]) || custom_read(ci_addr,settings[i][0],&back,1) || back!=settings[i][1]) return -1;
  }
  return 0;
}
static int custom_imu_read(fc_imu *m) {
  uint8_t b[14];m->have_gyro=0;if(custom_read(ci_addr,0x3B,b,14)) return -1;
  for(int i=0;i<3;i++) { m->acc[i]=(int16_t)(b[2*i]<<8|b[2*i+1])*(8*9.80665f/32768);m->gyro[i]=(int16_t)(b[8+2*i]<<8|b[9+2*i])*(2000*0.01745329252f/32768); }
  m->have_gyro=1;return 0;
}
// END IMU
// BEGIN BAROMETER
static int cb_addr,cb_phase;static int64_t cb_due;static int32_t cb_ut;static float cb_p0;static lb_bmp180_cal cb_cal;
static int custom_baro_init(int addr) {
  cb_addr=addr?addr:0x77;uint8_t id,b[22];
  if(custom_read(cb_addr,0xD0,&id,1) || id!=0x55 || custom_read(cb_addr,0xAA,b,22)) return -1;
  for(int i=0;i<22;i+=2) if((b[i]==0 && b[i+1]==0) || (b[i]==255 && b[i+1]==255)) return -1;
  #define CB16(i) ((uint16_t)(b[i]<<8|b[i+1]))
  cb_cal=(lb_bmp180_cal){.ac1=(int16_t)CB16(0),.ac2=(int16_t)CB16(2),.ac3=(int16_t)CB16(4),.ac4=CB16(6),.ac5=CB16(8),.ac6=CB16(10),.b1=(int16_t)CB16(12),.b2=(int16_t)CB16(14),.mb=(int16_t)CB16(16),.mc=(int16_t)CB16(18),.md=(int16_t)CB16(20)};
  #undef CB16
  cb_phase=0;cb_p0=0;return 0;
}
static int custom_baro_read(float *alt) {
  int64_t now=esp_timer_get_time();uint8_t b[3];
  if(cb_phase && now<cb_due) return 0;
  if(!cb_phase) { if(custom_write(cb_addr,0xF4,0x2e)) return 0;cb_phase=1;cb_due=now+5000;return 0; }
  if(cb_phase==1) { if(custom_read(cb_addr,0xF6,b,2) || custom_write(cb_addr,0xF4,0x34)) {cb_phase=0;return 0;}cb_ut=b[0]<<8|b[1];cb_phase=2;cb_due=now+5000;return 0; }
  cb_phase=0;if(custom_read(cb_addr,0xF6,b,3)) return 0;int32_t pa;
  if(lb_bmp180_pressure(&cb_cal,cb_ut,(b[0]<<16|b[1]<<8|b[2])>>8,0,&pa)) return 0;
  if(cb_p0==0) { cb_p0=(float)pa; }
  *alt=44330*(1-powf((float)pa/cb_p0,0.190295f));return isfinite(*alt)?1:0;
}
// END BAROMETER
// BEGIN COMPASS
static int cm_addr;
static int custom_mag_init(int addr) {
  cm_addr=addr?addr:0x1e;uint8_t id[3];
  if(custom_read(cm_addr,0x0A,id,3) || id[0]!='H' || id[1]!='4' || id[2]!='3') return -1;
  return custom_write(cm_addr,0,0x78) || custom_write(cm_addr,1,0x20) || custom_write(cm_addr,2,0) ? -1:0;
}
static int custom_mag_read(float out[3]) {
  uint8_t st,b[6];if(custom_read(cm_addr,9,&st,1) || !(st&1) || custom_read(cm_addr,3,b,6)) return 0;
  int16_t x=(int16_t)(b[0]<<8|b[1]),z=(int16_t)(b[2]<<8|b[3]),y=(int16_t)(b[4]<<8|b[5]);
  if(x==-4096 || y==-4096 || z==-4096) return 0;
  out[0]=x*(100.0f/1090);out[1]=y*(100.0f/1090);out[2]=z*(100.0f/1090);return 1;
}
// END COMPASS
#endif
