/* Motor output adapters: ESC pulse PWM, active-high brushed MOSFET duty PWM, and servo PWM.
 * Each mode has dedicated timers; channels retain airframe order, including mixed motor drivers. */
#include "hw.h"
#include "esp_board.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>

static int8_t motor_ch[FC_MAX_MOTORS], motor_mode[FC_MAX_MOTORS], servo_ch[FC_MAX_JOINTS];
static hw_config oc;
static int outputs_up;
static uint32_t motor_duty(int i,float t) {
  t=t>0?(t<1?t:1):0; /* NaN/negative -> off. */
  if(oc.motor_driver[i]) {
    float max=oc.motor_max_pct[i]*0.01f;if(t>max)t=max;
    return (uint32_t)(t*1023.0f); /* 10 bits; never use the overflowing 2^resolution endpoint. */
  }
  return (uint32_t)((oc.esc_min_us+t*(oc.esc_max_us-oc.esc_min_us))*oc.esc_hz*16384.0f/1e6f);
}
static uint32_t servo_duty(float us) {return (uint32_t)(us*50*16384.0f/1e6f);}
static void stop_outputs(void) {
  outputs_up=0;
  for(int i=0;i<FC_MAX_MOTORS;i++)if(motor_ch[i]>=0)ledc_stop((ledc_mode_t)motor_mode[i],(ledc_channel_t)motor_ch[i],0);
  for(int j=0;j<FC_MAX_JOINTS;j++)if(servo_ch[j]>=0)ledc_stop(LEDC_LOW_SPEED_MODE,(ledc_channel_t)servo_ch[j],0);
}
int hw_outputs_init(const hw_config *c,char *log,int logn) {
  if(outputs_up)stop_outputs();
  outputs_up=0;oc=*c;
  memset(motor_ch,-1,sizeof motor_ch);memset(servo_ch,-1,sizeof servo_ch);
  /* A MOSFET must be low before routing PWM, even if a later timer/channel fails. */
  for(int i=0;i<FC_MAX_MOTORS;i++)if(c->motor_pin[i]>=0 && c->motor_driver[i]) {
    if(gpio_set_level(c->motor_pin[i],0)!=ESP_OK || gpio_set_pull_mode(c->motor_pin[i],GPIO_PULLDOWN_ONLY)!=ESP_OK || gpio_set_direction(c->motor_pin[i],GPIO_MODE_OUTPUT)!=ESP_OK)goto failed;
  }
  int nh=0,nl=0,ne=0,nb=0,ns=0,timers[2][4]={{0}};
  for(int i=0;i<FC_MAX_MOTORS;i++) {
    if(c->motor_pin[i]<0)continue;
    int hs=0;
    ledc_mode_t mode=LEDC_LOW_SPEED_MODE;
#if SOC_LEDC_SUPPORT_HS_MODE
    hs=nh<SOC_LEDC_CHANNEL_NUM;
    if(hs)mode=LEDC_HIGH_SPEED_MODE;
#endif
    int ch=hs?nh:nl;if(ch>=SOC_LEDC_CHANNEL_NUM)goto failed;
    int brushed=c->motor_driver[i],timer=brushed?3:hs?0:2;
    if(!timers[hs][timer]) {
      ledc_timer_config_t tc={.speed_mode=mode,.timer_num=(ledc_timer_t)timer,.duty_resolution=brushed?LEDC_TIMER_10_BIT:LEDC_TIMER_14_BIT,.freq_hz=(uint32_t)(brushed?c->brushed_hz:c->esc_hz),.clk_cfg=LEDC_USE_APB_CLK};
      if(ledc_timer_config(&tc)!=ESP_OK)goto failed;
      timers[hs][timer]=1;
    }
    ledc_channel_config_t cc={.gpio_num=c->motor_pin[i],.speed_mode=mode,.channel=(ledc_channel_t)ch,.timer_sel=(ledc_timer_t)timer,.duty=motor_duty(i,0)};
    if(ledc_channel_config(&cc)!=ESP_OK)goto failed;
    motor_ch[i]=(int8_t)ch;motor_mode[i]=(int8_t)mode;
    if(hs)nh++;else nl++;if(brushed)nb++;else ne++;
  }
  for(int j=0;j<FC_MAX_JOINTS;j++) {
    if(c->servo_pin[j]<0)continue;
    if(nl>=SOC_LEDC_CHANNEL_NUM)goto failed;
    if(!timers[0][1]) {
      ledc_timer_config_t tc={.speed_mode=LEDC_LOW_SPEED_MODE,.timer_num=LEDC_TIMER_1,.duty_resolution=LEDC_TIMER_14_BIT,.freq_hz=50,.clk_cfg=LEDC_USE_APB_CLK};
      if(ledc_timer_config(&tc)!=ESP_OK)goto failed;
      timers[0][1]=1;
    }
    ledc_channel_config_t cc={.gpio_num=c->servo_pin[j],.speed_mode=LEDC_LOW_SPEED_MODE,.channel=(ledc_channel_t)nl,.timer_sel=LEDC_TIMER_1,.duty=servo_duty(c->servo_center_us[j])};
    if(ledc_channel_config(&cc)!=ESP_OK)goto failed;
    servo_ch[j]=(int8_t)nl++;ns++;
  }
  outputs_up=1;
  snprintf(log,logn,"%d ESCs at %d Hz, %d MOSFETs at %ld Hz (zero when stopped), %d servos",ne,c->esc_hz,nb,(long)c->brushed_hz,ns);return 0;
failed:
  stop_outputs();snprintf(log,logn,"PWM outputs refused: timer/channel/GPIO setup failed; outputs stopped");return -1;
}
int hw_outputs_ok(int nm,int ns,char *why,int n) {
  if(!outputs_up){snprintf(why,n,"PWM outputs did not start or were stopped after an error");return 0;}
  if(nm<0 || nm>FC_MAX_MOTORS || ns<0 || ns>FC_MAX_JOINTS)return 0;
  for(int i=0;i<nm;i++)if(motor_ch[i]<0){snprintf(why,n,"motor %d has no output",i+1);return 0;}
  for(int j=0;j<ns;j++)if(servo_ch[j]<0){snprintf(why,n,"servo %d has no output",j+1);return 0;}
  return 1;
}
static int write_motor(int i,float t) {
  return ledc_set_duty((ledc_mode_t)motor_mode[i],(ledc_channel_t)motor_ch[i],motor_duty(i,t))==ESP_OK && ledc_update_duty((ledc_mode_t)motor_mode[i],(ledc_channel_t)motor_ch[i])==ESP_OK;
}
void hw_outputs_set(const fc_out *o,int nm,int ns) {
  if(!outputs_up)return;
  for(int i=0;i<FC_MAX_MOTORS;i++)if(motor_ch[i]>=0 && !write_motor(i,i<nm?o->motor[i]:0)){stop_outputs();return;}
  for(int j=0;j<ns && j<FC_MAX_JOINTS;j++)if(servo_ch[j]>=0) {
    float us=oc.servo_center_us[j]+o->servo[j]*oc.servo_us_per_rad[j];if(!(us==us))us=oc.servo_center_us[j];else us=us<500?500:us>2500?2500:us;
    if(ledc_set_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)servo_ch[j],servo_duty(us))!=ESP_OK || ledc_update_duty(LEDC_LOW_SPEED_MODE,(ledc_channel_t)servo_ch[j])!=ESP_OK){stop_outputs();return;}
  }
}
void hw_outputs_safe(void) {
  if(!outputs_up)return;
  for(int i=0;i<FC_MAX_MOTORS;i++)if(motor_ch[i]>=0 && !write_motor(i,0)){stop_outputs();return;}
}
