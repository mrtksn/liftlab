/* Compile the production output adapter against recorded GPIO/LEDC operations, for each chip. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../runner/fc/esp32/main/outputs.c"
#include "../runner/fc/esp32/main/config.c"
#include "../runner/fc/esp32/main/hw_migration.h"
static ledc_timer_config_t timer_cfg[2][4];
static ledc_channel_config_t channels[2][8];
static uint32_t duty[2][8];static int stopped[2][8],gpio_low[64],gpio_pulled[64],calls,fail_at;
static int fail(void){return ++calls==fail_at;}
int gpio_set_level(int pin,int value){assert(!value);gpio_low[pin]=1;return fail()?-1:0;}
int gpio_set_pull_mode(int pin,int pull){assert(pull==GPIO_PULLDOWN_ONLY);gpio_pulled[pin]=1;return fail()?-1:0;}
int gpio_set_direction(int pin,int mode){assert(mode==GPIO_MODE_OUTPUT && gpio_low[pin] && gpio_pulled[pin]);return fail()?-1:0;}
int ledc_timer_config(const ledc_timer_config_t *c){if(fail())return -1;timer_cfg[c->speed_mode][c->timer_num]=*c;return 0;}
int ledc_channel_config(const ledc_channel_config_t *c){if(fail())return -1;channels[c->speed_mode][c->channel]=*c;duty[c->speed_mode][c->channel]=c->duty;stopped[c->speed_mode][c->channel]=0;return 0;}
int ledc_set_duty(int m,int ch,uint32_t d){if(fail())return -1;duty[m][ch]=d;return 0;}
int ledc_update_duty(int m,int ch){(void)m;(void)ch;return fail()?-1:0;}
int ledc_stop(int m,int ch,uint32_t idle){assert(!idle);stopped[m][ch]=1;duty[m][ch]=0;return 0;}
static uint32_t motor_actual(int i){return duty[motor_mode[i]][motor_ch[i]];}
static hw_config config(void){hw_config c={0};c.version=HW_VERSION;memset(c.motor_pin,-1,sizeof c.motor_pin);memset(c.servo_pin,-1,sizeof c.servo_pin);memset(c.motor_max_pct,100,sizeof c.motor_max_pct);c.brushed_hz=20000;c.esc_hz=400;c.esc_min_us=1000;c.esc_max_us=2000;c.servo_center_us[0]=1500;c.servo_us_per_rad[0]=636;return c;}
int main(void){
  char log[160];hw_config c=config();c.motor_pin[0]=4;c.motor_pin[1]=5;c.motor_pin[2]=6;c.servo_pin[0]=10;c.motor_driver[0]=c.motor_driver[2]=1;c.motor_max_pct[0]=60;
  assert(!hw_outputs_init(&c,log,sizeof log));assert(hw_outputs_ok(3,1,log,sizeof log));
  assert(motor_actual(0)==0 && motor_actual(2)==0 && motor_actual(1)==6553);
  for(int i=0;i<3;i++){int m=motor_mode[i],ch=motor_ch[i];ledc_timer_config_t *tc=&timer_cfg[m][channels[m][ch].timer_sel];assert(tc->freq_hz==(c.motor_driver[i]?20000u:400u));assert(tc->duty_resolution==(c.motor_driver[i]?10:14));}
  assert(timer_cfg[0][1].freq_hz==50);
  fc_out o={0};o.motor[0]=1;o.motor[1]=1;o.motor[2]=0.5f;hw_outputs_set(&o,3,1);
  assert(motor_actual(0)==613 && motor_actual(1)==13107 && motor_actual(2)==511);
  o.motor[0]=NAN;o.motor[1]=NAN;o.motor[2]=-1;hw_outputs_set(&o,3,1);assert(motor_actual(0)==0 && motor_actual(1)==6553 && motor_actual(2)==0);
  o.motor[0]=2;o.motor[2]=2;hw_outputs_set(&o,3,1);assert(motor_actual(0)==613 && motor_actual(2)==1023);
  hw_outputs_safe();assert(motor_actual(0)==0 && motor_actual(2)==0 && motor_actual(1)==6553);
  hw_outputs_set(&o,1,0);assert(motor_actual(2)==0); /* no leftover duty on unused outputs */
  fail_at=calls+1;hw_outputs_set(&o,3,1);assert(!hw_outputs_ok(3,1,log,sizeof log));for(int i=0;i<3;i++)assert(stopped[motor_mode[i]][motor_ch[i]]);
  fail_at=0;calls=0;assert(!hw_outputs_init(&c,log,sizeof log));int setup_calls=calls;
  for(int f=1;f<=setup_calls;f++){calls=0;fail_at=f;assert(hw_outputs_init(&c,log,sizeof log));assert(!hw_outputs_ok(3,1,log,sizeof log));}
  fail_at=0;calls=0;
  c=config();for(int i=0;i<FC_MAX_MOTORS && i<LB_PWM_OUTPUTS-1;i++){c.motor_pin[i]=i+1;c.motor_driver[i]=i%2;}c.servo_pin[0]=40;
  assert(!hw_outputs_init(&c,log,sizeof log));assert(hw_outputs_ok(LB_PWM_OUTPUTS-1<FC_MAX_MOTORS?LB_PWM_OUTPUTS-1:FC_MAX_MOTORS,1,log,sizeof log));
  hw_config old=config(),restored=config();old.version=5;old.motor_pin[0]=27;old.imu_driver=3;old.mag_scale[0]=1.5f;
  assert(lb_hw_restore(&restored,&old,offsetof(hw_config,motor_driver)));assert(restored.motor_pin[0]==27 && restored.imu_driver==3 && restored.mag_scale[0]==1.5f);assert(restored.motor_driver[0]==0 && restored.motor_max_pct[0]==100 && restored.brushed_hz==20000);
  old.version=4;restored=config();restored.imu_driver=0;assert(lb_hw_restore(&restored,&old,(offsetof(hw_config,imu_driver)+3u)&~3u));assert(restored.motor_pin[0]==27 && restored.imu_driver==0);
  old.version=6;restored=config();assert(lb_hw_restore(&restored,&old,offsetof(hw_config,radio_kind)));
  old.version=3;restored=config();assert(lb_hw_restore(&restored,&old,offsetof(hw_config,crsf_rx)));
  old.version=2;restored=config();assert(lb_hw_restore(&restored,&old,offsetof(hw_config,link_baud)));assert(restored.link_baud==115200);
  old.version=HW_VERSION;old.motor_driver[0]=1;old.motor_max_pct[0]=70;assert(lb_hw_restore(&restored,&old,sizeof old));assert(restored.motor_driver[0]==1 && restored.motor_max_pct[0]==70);
  assert(!lb_hw_restore(&restored,&old,sizeof old-1));assert(!lb_hw_restore(&restored,&old,1));old.version=99;assert(!lb_hw_restore(&restored,&old,sizeof old));
  hw_config settings;hw_defaults(&settings);assert(settings.version==HW_VERSION && settings.brushed_hz==20000 && settings.motor_max_pct[0]==100 && settings.motor_driver[0]==0);
  assert(!hw_check(&settings,log,sizeof log));assert(!hw_set(&settings,"motor_driver=1,0,1,0",log,sizeof log));assert(!hw_set(&settings,"motor_max=60,100,70,100",log,sizeof log));assert(!hw_set(&settings,"brushed_hz=30000",log,sizeof log));
  const char *bad[]={"motor_driver=2","motor_driver=0.5","motor_driver=NaN","motor_max=0","motor_max=101","motor_max=10.5","brushed_hz=999","brushed_hz=30001","brushed_hz=20000.5","motor_driver=1,0,1,0,1,0,1,0,1,0,1,0,1","brushed_hz=20000junk"};
  for(unsigned i=0;i<sizeof bad/sizeof *bad;i++){hw_config before=settings;assert(hw_set(&settings,bad[i],log,sizeof log));assert(!memcmp(&settings,&before,sizeof settings));}
  assert(!hw_set(&settings,"motor_driver=",log,sizeof log));assert(settings.motor_driver[0]==0 && settings.motor_driver[11]==0);assert(!hw_set(&settings,"motor_max=",log,sizeof log));assert(settings.motor_max_pct[0]==100);
  char text[800];hw_describe(&settings,text,sizeof text);assert(strstr(text,"motor_driver=") && strstr(text,"motor_max=") && strstr(text,"brushed_hz=30000"));
  struct {char text[8];char canary[8];} small;memset(&small,42,sizeof small);hw_describe(&settings,small.text,sizeof small.text);assert(small.text[7]==0);for(int i=0;i<8;i++)assert(small.canary[i]==42);
  puts("Motor output tests passed: mixed timers, zero startup/stop, caps, invalid throttle, setup/write failures, channel limits, v2–v6 migration.");
}
