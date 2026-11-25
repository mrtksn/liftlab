/* Dedicated ESP32-WROOM digital signal workbench. JSON-lines over UART0.
 * Outputs start high-impedance. A host lease expires after 900 ms, on core 0.
 * Native GPIO capture and analysis run on core 1; LEDC/RMT keep timing in hardware.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "driver/uart.h"
#include "soc/gpio_struct.h"
#include "hal/gpio_ll.h"
#include "esp_cpu.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "cJSON.h"
#include "sniff_core.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define CHANNELS 8
#define CAPACITY 12000
#define ANALOG_POINTS 1024
#define LEASE_MS 900
#define COMMAND_BYTES 8192
#define MHZ 240
static const int out_pins[] = {4,13,14,16,17,18,19,21,22,23,25,26,27,32,33};
static const int input_only[] = {34,35,36,39};
static SemaphoreHandle_t io_lock, tx_lock, captured;
static QueueHandle_t commands;
static TaskHandle_t capture_handle;
static volatile bool armed;
static volatile int64_t last_ping;
static volatile uint32_t cancel_epoch;
typedef struct { cJSON *json; uint32_t epoch; } queued_command_t;
static int configured;
static cJSON *configuration;
typedef struct { int pin; bool output, analog, ledc, rmt, driven; int adc_channel, adc_unit, attenuation; cJSON *idle; rmt_item32_t wave[64]; } channel_t;
static channel_t channels[CHANNELS];
static uint32_t times[CAPACITY], scratch_b[CAPACITY+2];
/* Digital analysis and analog acquisition are serialized by the command task. */
static union {uint32_t words[CAPACITY+2];uint16_t adc[ANALOG_POINTS][CHANNELS];} scratch_storage;
#define scratch_a scratch_storage.words
#define analog_raw scratch_storage.adc
#define analog_times scratch_b
static uint8_t levels[CAPACITY];
static sniff_cap capture = {times,levels,0,0,0,0,MHZ};
static uint32_t sample_loops;
static int capture_ms, trigger_ch, trigger_edge;
static bool trigger_ok;
static bool capture_outputs;

static const int analog_pins[]={32,33,34,35,36,39,4,13,14,25,26,27};
static adc_oneshot_unit_handle_t adc_units[2];
static adc_cali_handle_t adc_calibration[2][4];
static bool adc_efuse;
static volatile uint32_t analog_cancel_epoch;

static const char *str(cJSON *o,const char *k,const char *fallback) { cJSON *v=cJSON_GetObjectItem(o,k); return cJSON_IsString(v)?v->valuestring:fallback; }
static double num(cJSON *o,const char *k,double fallback) { cJSON *v=cJSON_GetObjectItem(o,k); return cJSON_IsNumber(v)&&isfinite(v->valuedouble)?v->valuedouble:fallback; }
static bool has_pin(const int *p,size_t n,int pin) { for(size_t i=0;i<n;i++) if(p[i]==pin)return true; return false; }
static bool output_pin(int p) { return has_pin(out_pins,sizeof out_pins/sizeof *out_pins,p); }
static bool input_pin(int p) { return output_pin(p)||has_pin(input_only,sizeof input_only/sizeof *input_only,p); }
static int64_t now_ms(void) { return esp_timer_get_time()/1000; }
static void emit(cJSON *o) {
  char *s=cJSON_PrintUnformatted(o); if(s) { xSemaphoreTake(tx_lock,portMAX_DELAY); uart_write_bytes(UART_NUM_0,"@SB ",4); uart_write_bytes(UART_NUM_0,s,strlen(s)); uart_write_bytes(UART_NUM_0,"\n",1); xSemaphoreGive(tx_lock); free(s); } cJSON_Delete(o);
}
static cJSON *event(const char *type) { cJSON *o=cJSON_CreateObject(); cJSON_AddStringToObject(o,"type",type); return o; }
static void reply(cJSON *cmd,const char *error) {
  cJSON *o=event(error?"error":"ack"); cJSON_AddNumberToObject(o,"id",num(cmd,"id",0));
  cJSON_AddBoolToObject(o,"armed",armed); if(error)cJSON_AddStringToObject(o,"message",error); emit(o);
}
static ledc_mode_t speed(int i) { return i<4?LEDC_HIGH_SPEED_MODE:LEDC_LOW_SPEED_MODE; }
static void release_output(int i) {
  channel_t *c=&channels[i];c->driven=false;
  if(c->ledc) { ledc_stop(speed(i),i%4,0); c->ledc=false; }
  if(c->rmt) { rmt_tx_stop(i); rmt_driver_uninstall(i); c->rmt=false; }
  if(c->pin>=0) { gpio_reset_pin(c->pin); gpio_set_direction(c->pin,GPIO_MODE_INPUT); }
}
static const char *apply_signal(int i,cJSON *s,bool continuous,int repeat_ms);
static void idle_output(int i) {
  if(channels[i].output && channels[i].idle) {
    if(!apply_signal(i,channels[i].idle,true,100))return;
  }
  release_output(i);
}
static void stop_all(void) { cancel_epoch++; for(int i=0;i<configured;i++)if(channels[i].output&&channels[i].driven)idle_output(i); armed=false; }

/* Pack arbitrary segments into the RMT channel's 64 words, keeping a terminator. */
static bool segment(channel_t *c,int *half,int level,uint32_t ticks) {
  while(ticks) {
    if(*half>=126)return false;
    uint32_t n=ticks>32767?32767:ticks; int index=*half/2;
    if(*half%2) { c->wave[index].level1=level; c->wave[index].duration1=n; }
    else { c->wave[index].level0=level; c->wave[index].duration0=n; }
    (*half)++; ticks-=n;
  }
  return true;
}
static const char *apply_signal(int i,cJSON *s,bool continuous,int repeat_ms) {
  channel_t *c=&channels[i]; const char *kind=str(s,"kind","");
  double value=num(s,"value",0), hz=num(s,"hz",490);
  if(!strcmp(kind,"idle")) { idle_output(i); return NULL; }
  if(!strcmp(kind,"highz")) { release_output(i); return NULL; }
  bool pwm=!strcmp(kind,"pwm")||!strcmp(kind,"duty");
  if(pwm && (hz<1||hz>1000000||hz!=floor(hz)))return "Frequency must be an integer from 1 to 1000000 Hz";
  double width=!strcmp(kind,"duty")?value*10000.0/hz:value;
  if(pwm && (width<0||width*hz>1000000))return "Pulse width exceeds the period";
  if(!strcmp(kind,"digital")&&(value!=0&&value!=1))return "Digital value must be 0 or 1";
  if(pwm&&continuous) {
    /* Keep a running timer/channel when only duty changes, without a low glitch. */
    int bits=20; while(bits>1 && 80000000.0/(hz*(1u<<bits))<1)bits--;
    uint32_t duty=(uint32_t)llround(width*hz*(1u<<bits)/1000000.0);
    if(duty>=(1u<<bits))duty=(1u<<bits)-1;
    if(c->rmt)release_output(i);
    ledc_timer_config_t timer={.speed_mode=speed(i),.timer_num=i%4,.duty_resolution=bits,.freq_hz=hz,.clk_cfg=LEDC_USE_APB_CLK};
    if(ledc_timer_config(&timer)!=ESP_OK)return "PWM frequency is outside the hardware timer range";
    if(!c->ledc) {
      ledc_channel_config_t ch={.gpio_num=c->pin,.speed_mode=speed(i),.channel=i%4,.timer_sel=i%4,.duty=duty,.hpoint=0};
      if(ledc_channel_config(&ch)!=ESP_OK)return "Cannot allocate PWM output";
      c->ledc=true;
    } else { if(ledc_set_duty(speed(i),i%4,duty)!=ESP_OK||ledc_update_duty(speed(i),i%4)!=ESP_OK)return "Cannot update PWM output"; }
    c->driven=true;gpio_ll_input_enable(&GPIO,c->pin); /* Preserve the peripheral output-matrix route. */
    return NULL;
  }
  if(!strcmp(kind,"digital")&&continuous) {
    release_output(i); gpio_set_level(c->pin,(int)value); gpio_set_direction(c->pin,GPIO_MODE_INPUT_OUTPUT);c->driven=true; return NULL;
  }
  release_output(i); memset(c->wave,0,sizeof c->wave); int half=0,div=80,idle=0; double duration_us=0;
#define SEG(level,ticks) do { if(!segment(c,&half,(level),(uint32_t)(ticks)))return "Waveform exceeds 126 RMT segments; shorten the data or repeat interval"; } while(0)
  if(pwm) { SEG(1,llround(width)); SEG(0,llround(1000000.0/hz-width)); }
  else if(!strcmp(kind,"digital")) { double us=num(s,"duration",1000); if(us<1||us>1000000)return "Pulse duration must be 1 to 1000000 us"; SEG((int)value,llround(us)); SEG(1-(int)value,1); idle=1-(int)value; }
  else if(!strcmp(kind,"pulse")) {
    cJSON *pairs=cJSON_GetObjectItem(s,"pulses"); int n=cJSON_GetArraySize(pairs);
    if(n<1||n>126)return "Provide 1 to 126 [level, duration_us] segments";
    for(int k=0;k<n;k++) { cJSON *p=cJSON_GetArrayItem(pairs,k),*l=cJSON_GetArrayItem(p,0),*d=cJSON_GetArrayItem(p,1);
      if(!cJSON_IsArray(p)||cJSON_GetArraySize(p)!=2||!cJSON_IsNumber(l)||!cJSON_IsNumber(d)||!isfinite(d->valuedouble)||(l->valuedouble!=0&&l->valuedouble!=1)||d->valuedouble<1||d->valuedouble>1000000)return "Each segment needs level 0/1 and duration 1..1000000 us";
      SEG(l->valueint,llround(d->valuedouble)); }
    idle=(int)num(s,"idleLevel",0); if(idle!=0&&idle!=1)return "Idle level must be 0 or 1";
  } else if(!strcmp(kind,"uart")) {
    idle=1; double baud=num(s,"baud",115200); const char *hex=str(s,"data",""); size_t n=strlen(hex);
    if(baud<300||baud>1000000||n<2||n>24||n%2)return "UART needs 1..12 hex bytes and baud 300..1000000";
    div=baud>200000?2:baud>38400?8:80;
    uint32_t bit=llround(80000000.0/div/baud);
    for(size_t k=0;k<n;k+=2) { char b[3]={hex[k],hex[k+1],0},*end; unsigned long byte=strtoul(b,&end,16); if(*end)return "UART data must be hexadecimal";
      SEG(0,bit); for(int j=0;j<8;j++)SEG((byte>>j)&1,bit); SEG(1,bit); }
  } else if(!strcmp(kind,"dshot")) {
    div=2; int rate=(int)num(s,"rate",150); if(rate!=150&&rate!=300&&rate!=600)return "DShot rate must be 150, 300 or 600";
    if(value<0||value>2047||value!=floor(value))return "DShot value must be 0..2047";
    uint16_t payload=((uint16_t)value<<1)|(num(s,"telemetry",0)?1:0);
    uint16_t frame=(payload<<4)|((payload^(payload>>4)^(payload>>8))&15);
    uint32_t period=llround(40000.0/rate);
    for(int j=15;j>=0;j--) { uint32_t high=llround(period*((frame&(1<<j))?.75:.375)); SEG(1,high); SEG(0,period-high); }
    SEG(0,2000); /* at least 50 us between frames */
  } else return "Unknown signal kind";
  if(!half)return "Waveform has no duration";
  for(int k=0;k<(half+1)/2;k++)duration_us+=(c->wave[k].duration0+c->wave[k].duration1)*div/80.0;
  if(continuous && !pwm && strcmp(kind,"dshot")) {
    double gap=repeat_ms*1000.0-duration_us;
    if(gap<0)return "Repeat interval is shorter than the waveform";
    SEG(idle,llround(gap*80.0/div));
  }
  if(half%2)SEG(idle,1);
  rmt_config_t r={.rmt_mode=RMT_MODE_TX,.channel=i,.gpio_num=c->pin,.clk_div=div,.mem_block_num=1,
    .tx_config={.loop_en=continuous,.carrier_en=false,.idle_output_en=true,.idle_level=idle}};
  if(rmt_config(&r)!=ESP_OK||rmt_driver_install(i,0,0)!=ESP_OK)return "Cannot allocate waveform output";
  c->rmt=true;
  gpio_ll_input_enable(&GPIO,c->pin);
  if(rmt_write_items(i,c->wave,(half+1)/2,false)!=ESP_OK) { release_output(i); return "Waveform transmission failed"; }
  c->driven=true;return NULL;
#undef SEG
}

/* Capture raw GPIO-bank snapshots into the analysis scratch arrays. Packing
 * channels afterwards keeps the hot loop independent of the channel count. */
static uint8_t pack_levels(uint32_t low,uint32_t high) {
  uint8_t result=0;
  for(int i=0;i<configured;i++)if(!channels[i].analog&&(capture_outputs||!channels[i].output)) {
    int p=channels[i].pin;result|=((p<32?low>>p:high>>(p-32))&1)<<i;
  }
  return result;
}
static void capture_task(void *unused) {
  for(;;) {
    ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
    uint32_t mask_low=0,mask_high=0;
    for(int i=0;i<configured;i++)if(!channels[i].analog&&(capture_outputs||!channels[i].output)) {
      int p=channels[i].pin;if(p<32)mask_low|=1u<<p;else mask_high|=1u<<(p-32);
    }
    int tp=trigger_ch>=0?channels[trigger_ch].pin:0;
    uint32_t n=0,loops=0;trigger_ok=trigger_ch<0;
    UBaseType_t mask=portSET_INTERRUPT_MASK_FROM_ISR();
    uint32_t low=GPIO.in&mask_low,high=GPIO.in1.val&mask_high;
    uint32_t begin=esp_cpu_get_cycle_count(),now=begin;
    if(trigger_ch>=0) {
      while(now-begin<100000u*MHZ) {
        uint32_t next_low=GPIO.in&mask_low,next_high=GPIO.in1.val&mask_high;
        now=esp_cpu_get_cycle_count();
        int old=(tp<32?low>>tp:high>>(tp-32))&1,new=(tp<32?next_low>>tp:next_high>>(tp-32))&1;
        low=next_low;high=next_high;
        if(old!=new&&(trigger_edge==2||new==trigger_edge)){trigger_ok=true;break;}
      }
    }
    uint32_t initial_low=low,initial_high=high;
    begin=esp_cpu_get_cycle_count();now=begin;
    if(trigger_ok)while(now-begin<(uint32_t)capture_ms*1000u*MHZ) {
      uint32_t next_low=GPIO.in&mask_low,next_high=GPIO.in1.val&mask_high;
      now=esp_cpu_get_cycle_count();loops++;
      if(next_low!=low||next_high!=high) {
        times[n]=now;scratch_a[n]=next_low;scratch_b[n]=next_high;
        low=next_low;high=next_high;if(++n==CAPACITY)break;
      }
    }
    else now=begin;
    portCLEAR_INTERRUPT_MASK_FROM_ISR(mask);
    capture.lv0=pack_levels(initial_low,initial_high);
    for(uint32_t i=0;i<n;i++)levels[i]=pack_levels(scratch_a[i],scratch_b[i]);
    capture.n=n;capture.t0=begin;capture.t_end=now;sample_loops=loops;
    xSemaphoreGive(captured);
  }
}
static void send_capture(cJSON *cmd) {
  capture_outputs=cJSON_IsTrue(cJSON_GetObjectItem(cmd,"includeOutputs"));
  capture_ms=(int)num(cmd,"ms",60); trigger_ch=(int)num(cmd,"triggerCh",-1); trigger_edge=(int)num(cmd,"triggerEdge",1);
  if(capture_ms<1||capture_ms>100||trigger_ch < -1||trigger_ch>=configured||trigger_edge<0||trigger_edge>2||(trigger_ch>=0&&channels[trigger_ch].analog)||(trigger_ch>=0&&channels[trigger_ch].output&&!capture_outputs)) { reply(cmd,"Capture window 1..100 ms; trigger must be a captured channel");return; }
  bool any=false;for(int i=0;i<configured;i++)any|=!channels[i].analog&&(capture_outputs||!channels[i].output);
  if(!any) { reply(cmd,"Configure at least one input channel");return; }
  xTaskNotifyGive(capture_handle);xSemaphoreTake(captured,portMAX_DELAY);
  cJSON *o=event("capture");cJSON_AddNumberToObject(o,"id",num(cmd,"id",0));cJSON_AddNumberToObject(o,"initial",capture.lv0);
  cJSON_AddNumberToObject(o,"duration",(capture.t_end-capture.t0)/(double)MHZ);cJSON_AddNumberToObject(o,"count",capture.n);
  cJSON_AddNumberToObject(o,"sampleUs",sample_loops?(capture.t_end-capture.t0)/(double)MHZ/sample_loops:0);
  cJSON_AddBoolToObject(o,"full",capture.n==CAPACITY);cJSON_AddBoolToObject(o,"triggered",trigger_ok);
  cJSON_AddBoolToObject(o,"includeOutputs",capture_outputs);
  cJSON *a=cJSON_AddArrayToObject(o,"analysis");
  for(int i=0;i<configured;i++)if(!channels[i].analog&&(capture_outputs||!channels[i].output)) {
    sniff_res r;char line[320];sniff_analyze(&capture,i,scratch_a,scratch_b,&r);sniff_line(&r,line,sizeof line);
    cJSON *s=cJSON_CreateObject();cJSON_AddNumberToObject(s,"ch",i);cJSON_AddStringToObject(s,"kind",sniff_kind_name(r.kind));cJSON_AddStringToObject(s,"description",line);
    cJSON_AddNumberToObject(s,"hz",r.hz);cJSON_AddNumberToObject(s,"high",r.high_us);cJSON_AddNumberToObject(s,"duty",r.duty*100.0);cJSON_AddNumberToObject(s,"edges",r.edges);
    cJSON_AddItemToArray(a,s);
  }
  emit(o);
  for(int k=0;k<capture.n;k+=128) {
    o=event("edges");cJSON_AddNumberToObject(o,"id",num(cmd,"id",0));a=cJSON_AddArrayToObject(o,"edges");
    for(int j=k;j<capture.n&&j<k+128;j++) { cJSON *p=cJSON_CreateArray();cJSON_AddItemToArray(p,cJSON_CreateNumber((times[j]-capture.t0)/(double)MHZ));cJSON_AddItemToArray(p,cJSON_CreateNumber(levels[j]));cJSON_AddItemToArray(a,p); } emit(o);
  }
  o=event("captureEnd");cJSON_AddNumberToObject(o,"id",num(cmd,"id",0));emit(o);
}
static int attenuation_index(double value) {return value==0?0:value==2.5?1:value==6?2:value==12?3:-1;}
static const char *setup_adc(int i) {
  channel_t *c=&channels[i];adc_unit_t unit;adc_channel_t channel;
  if(adc_oneshot_io_to_channel(c->pin,&unit,&channel)!=ESP_OK)return "GPIO has no ADC channel";
  c->adc_unit=unit;c->adc_channel=channel;
  if(!adc_units[unit]) {adc_oneshot_unit_init_cfg_t cfg={.unit_id=unit,.ulp_mode=ADC_ULP_MODE_DISABLE};if(adc_oneshot_new_unit(&cfg,&adc_units[unit])!=ESP_OK)return "Cannot allocate ADC unit";}
  adc_oneshot_chan_cfg_t cfg={.bitwidth=ADC_BITWIDTH_12,.atten=c->attenuation};
  if(adc_oneshot_config_channel(adc_units[unit],channel,&cfg)!=ESP_OK)return "Cannot configure ADC channel";
  if(!adc_calibration[unit][c->attenuation]) {
    adc_cali_line_fitting_efuse_val_t source;
    adc_efuse=adc_cali_scheme_line_fitting_check_efuse(&source)==ESP_OK&&source!=ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF;
    adc_cali_line_fitting_config_t calibration={.unit_id=unit,.atten=c->attenuation,.bitwidth=ADC_BITWIDTH_12,.default_vref=1100};
    if(adc_cali_create_scheme_line_fitting(&calibration,&adc_calibration[unit][c->attenuation])!=ESP_OK)return "Cannot initialize ADC calibration";
  }
  return NULL;
}
static void send_analog(cJSON *cmd,uint32_t epoch) {
  int rate=(int)num(cmd,"hz",100),ms=(int)num(cmd,"ms",1000),n=0,index[CHANNELS];
  int count=(int)ceil(ms*rate/1000.0);
  if(rate<1||rate>1000||ms<10||ms>10000||count<1||count>ANALOG_POINTS){reply(cmd,"ADC: 1..1000 samples/s, 10..10000 ms, at most 1024 points per channel");return;}
  for(int i=0;i<configured;i++)if(channels[i].analog)index[n++]=i;
  if(!n){reply(cmd,"Configure at least one analog input channel");return;}
  uint32_t analog_epoch=analog_cancel_epoch;
  int64_t start=esp_timer_get_time();int actual=0;const char *error=NULL;
  for(int row=0;row<count;row++) {
    if(epoch!=cancel_epoch||analog_epoch!=analog_cancel_epoch)break;
    int64_t target=start+(int64_t)row*1000000/rate;
    while(esp_timer_get_time()<target) {vTaskDelay(pdMS_TO_TICKS(1));if(epoch!=cancel_epoch||analog_epoch!=analog_cancel_epoch)break;}
    if(epoch!=cancel_epoch||analog_epoch!=analog_cancel_epoch)break;
    analog_times[row]=esp_timer_get_time()-start;
    for(int j=0;j<n;j++){channel_t *c=&channels[index[j]];int raw;if(adc_oneshot_read(adc_units[c->adc_unit],c->adc_channel,&raw)!=ESP_OK){error="ADC conversion failed; no fabricated sample was returned";break;}analog_raw[row][j]=raw;}
    if(error)break;
    actual++;
  }
  if(error){reply(cmd,error);return;}
  cJSON *o=event("analog");cJSON_AddNumberToObject(o,"id",num(cmd,"id",0));cJSON_AddNumberToObject(o,"count",actual);cJSON_AddNumberToObject(o,"requestedHz",rate);cJSON_AddNumberToObject(o,"duration",esp_timer_get_time()-start);
  cJSON_AddBoolToObject(o,"cancelled",epoch!=cancel_epoch||analog_epoch!=analog_cancel_epoch);cJSON_AddStringToObject(o,"calibration",adc_efuse?"eFuse line fitting":"nominal 1100 mV reference estimate");
  cJSON *list=cJSON_AddArrayToObject(o,"channels");
  for(int j=0;j<n;j++){channel_t *c=&channels[index[j]];cJSON *entry=cJSON_CreateObject();cJSON_AddNumberToObject(entry,"ch",index[j]);cJSON_AddNumberToObject(entry,"pin",c->pin);cJSON_AddNumberToObject(entry,"unit",c->adc_unit+1);cJSON_AddNumberToObject(entry,"attenuation",(double[]){0,2.5,6,12}[c->attenuation]);cJSON_AddItemToArray(list,entry);}
  emit(o);
  for(int row=0;row<actual;row+=16){o=event("analogPoints");cJSON_AddNumberToObject(o,"id",num(cmd,"id",0));list=cJSON_AddArrayToObject(o,"points");
    for(int k=row;k<actual&&k<row+16;k++){cJSON *point=cJSON_CreateArray();cJSON_AddItemToArray(point,cJSON_CreateNumber(analog_times[k]));for(int j=0;j<n;j++){channel_t *c=&channels[index[j]];int mv=0;if(adc_cali_raw_to_voltage(adc_calibration[c->adc_unit][c->attenuation],analog_raw[k][j],&mv)!=ESP_OK)mv=-1;cJSON_AddItemToArray(point,cJSON_CreateNumber(analog_raw[k][j]));cJSON_AddItemToArray(point,cJSON_CreateNumber(mv));}cJSON_AddItemToArray(list,point);}emit(o);
  }
  o=event("analogEnd");cJSON_AddNumberToObject(o,"id",num(cmd,"id",0));emit(o);
}
static const char *configure(cJSON *cmd) {
  cJSON *a=cJSON_GetObjectItem(cmd,"channels");int n=cJSON_GetArraySize(a); uint64_t used=0;
  if(!cJSON_IsArray(a)||n<1||n>CHANNELS)return "Configure 1..8 channels";
  for(int i=0;i<n;i++) {
    cJSON *c=cJSON_GetArrayItem(a,i);int p=(int)num(c,"pin",-1);const char *d=str(c,"direction",""); bool out=!strcmp(d,"out");
    bool analog=!strcmp(d,"analog");
    if(strcmp(d,"out")&&strcmp(d,"in")&&!analog)return "Channel direction must be in, out or analog";
    if(analog&&(!has_pin(analog_pins,sizeof analog_pins/sizeof *analog_pins,p)||attenuation_index(num(c,"attenuation",12))<0))return "Choose an ADC-capable GPIO and attenuation 0, 2.5, 6 or 12 dB";
    if(!input_pin(p)||(out&&!output_pin(p)))return "Reserved, invalid, or input-only GPIO";
    if(used&(1ULL<<p))return "A GPIO can only belong to one channel";
    used|=1ULL<<p;
    cJSON *idle=cJSON_GetObjectItem(c,"idle");const char *kind=str(idle,"kind","highz");
    if(strcmp(kind,"highz")&&strcmp(kind,"digital")&&strcmp(kind,"pwm")&&strcmp(kind,"duty"))return "Idle state must be highz, digital, PWM or duty PWM";
    if(!strcmp(kind,"digital")&&num(idle,"value",0)!=0&&num(idle,"value",0)!=1)return "Idle level must be 0 or 1";
    if(!strcmp(kind,"pwm")||!strcmp(kind,"duty")) { double hz=num(idle,"hz",490),value=num(idle,"value",0); if(hz<1||hz>1000000||hz!=floor(hz)||value<0||(!strcmp(kind,"pwm")?value*hz>1000000:value>100))return "Invalid idle PWM signal"; }
  }
  /* Validation above is transactional; no pin changes before the whole map is valid. */
  for(int i=0;i<configured;i++)release_output(i);
  cJSON_Delete(configuration);configuration=cJSON_Duplicate(a,true);configured=n;armed=false;
  for(int i=0;i<n;i++) {
    cJSON *c=cJSON_GetArrayItem(configuration,i);channels[i].pin=num(c,"pin",-1);channels[i].output=!strcmp(str(c,"direction","in"),"out");channels[i].idle=cJSON_GetObjectItem(c,"idle");channels[i].analog=!strcmp(str(c,"direction","in"),"analog");channels[i].attenuation=attenuation_index(num(c,"attenuation",12));
    gpio_reset_pin(channels[i].pin);gpio_set_direction(channels[i].pin,GPIO_MODE_INPUT);
    if(channels[i].analog){const char *error=setup_adc(i);if(error){configured=0;return error;}}
  }
  return NULL;
}
static void command(cJSON *cmd,uint32_t epoch) {
  const char *name=str(cmd,"cmd","");
  if((!strcmp(name,"enable")||!strcmp(name,"signal")||!strcmp(name,"idle"))&&epoch!=cancel_epoch) { reply(cmd,"Cancelled by stop or lease expiry");return; }
  if(!strcmp(name,"capture")) { send_capture(cmd);return; }
  if(!strcmp(name,"analog")) { send_analog(cmd,epoch);return; }
  xSemaphoreTake(io_lock,portMAX_DELAY);const char *error=NULL;
  if((!strcmp(name,"enable")||!strcmp(name,"signal")||!strcmp(name,"idle"))&&epoch!=cancel_epoch){reply(cmd,"Cancelled by stop or lease expiry");xSemaphoreGive(io_lock);return;}
  if(!strcmp(name,"hello")) { cancel_epoch++;for(int i=0;i<configured;i++)if(channels[i].output)release_output(i);armed=false;cJSON *o=event("hello");cJSON_AddStringToObject(o,"firmware","signal-workbench/2");cJSON_AddNumberToObject(o,"protocol",2);cJSON_AddNumberToObject(o,"channels",CHANNELS);cJSON_AddBoolToObject(o,"adc",true);cJSON_AddNumberToObject(o,"leaseMs",LEASE_MS);cJSON_AddNumberToObject(o,"id",num(cmd,"id",0));emit(o); }
  else if(!strcmp(name,"configure"))error=configure(cmd);
  else if(!strcmp(name,"enable")) { if(!configured)error="Configure channels before enabling outputs";else { armed=true;last_ping=now_ms(); } }
  else if(!strcmp(name,"stop"))stop_all();
  else if(!strcmp(name,"signal")||!strcmp(name,"idle")) {
    int i=(int)num(cmd,"ch",-1);
    if(!armed)error="Outputs are disarmed";
    else if(i<0||i>=configured||!channels[i].output)error="Choose a configured output channel";
    else if(!strcmp(name,"idle"))idle_output(i);
    else { cJSON *s=cJSON_GetObjectItem(cmd,"signal");if(!cJSON_IsObject(s))error="Missing signal definition";else error=apply_signal(i,s,!strcmp(str(cmd,"mode","once"),"continuous"),(int)num(cmd,"repeatMs",100));if(error)idle_output(i); }
  } else error="Unknown command";
  if(strcmp(name,"hello"))reply(cmd,error);
  xSemaphoreGive(io_lock);
}
static void reader_task(void *unused) {
  char *line=malloc(COMMAND_BYTES);int n=0;bool overflow=false;uint8_t bytes[256];
  for(;;) {
    int count=uart_read_bytes(UART_NUM_0,bytes,sizeof bytes,pdMS_TO_TICKS(20));
    for(int k=0;k<count;k++) {
      unsigned char c=bytes[k];if(c=='\r'||c=='\n') {
        if(n&&!overflow) { line[n]=0;cJSON *cmd=cJSON_Parse(line);if(cmd) {
          if(!strcmp(str(cmd,"cmd",""),"ping")) { last_ping=now_ms();cJSON_Delete(cmd); }
          else if(!strcmp(str(cmd,"cmd",""),"cancelCapture")) { analog_cancel_epoch++;reply(cmd,NULL);cJSON_Delete(cmd); }
          else if(!strcmp(str(cmd,"cmd",""),"stop")||!strcmp(str(cmd,"cmd",""),"enable")||!strcmp(str(cmd,"cmd",""),"signal")||!strcmp(str(cmd,"cmd",""),"idle")) { command(cmd,cancel_epoch);cJSON_Delete(cmd); }
          else { queued_command_t queued={cmd,cancel_epoch};if(xQueueSend(commands,&queued,0)!=pdTRUE) { reply(cmd,"Command queue full");cJSON_Delete(cmd); } }
        } else { cJSON *o=event("error");cJSON_AddStringToObject(o,"message","Malformed JSON command");emit(o); } }
        if(overflow) { cJSON *o=event("error");cJSON_AddStringToObject(o,"message","Command exceeds 8191 bytes");emit(o); }
        n=0;overflow=false;
      } else if(c>=32) { if(n<COMMAND_BYTES-1)line[n++]=c;else overflow=true; }
    }
  }
}
static void lease_task(void *unused) {
  for(;;) { vTaskDelay(pdMS_TO_TICKS(25));xSemaphoreTake(io_lock,portMAX_DELAY);
    if(armed&&now_ms()-last_ping>LEASE_MS) { stop_all();cJSON *o=event("timeout");cJSON_AddStringToObject(o,"message","Host lease expired; idle signals applied and outputs disarmed");emit(o); }
    xSemaphoreGive(io_lock);
  }
}
void app_main(void) {
  for(int i=0;i<CHANNELS;i++)channels[i].pin=-1;
  io_lock=xSemaphoreCreateMutex();tx_lock=xSemaphoreCreateMutex();captured=xSemaphoreCreateBinary();commands=xQueueCreate(16,sizeof(queued_command_t));
  uart_config_t uart={.baud_rate=921600,.data_bits=UART_DATA_8_BITS,.parity=UART_PARITY_DISABLE,.stop_bits=UART_STOP_BITS_1,.flow_ctrl=UART_HW_FLOWCTRL_DISABLE,.source_clk=UART_SCLK_APB};
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_0,&uart));ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0,16384,8192,0,NULL,0));
  xTaskCreatePinnedToCore(reader_task,"commands",4096,NULL,8,NULL,0);
  xTaskCreatePinnedToCore(lease_task,"lease",4096,NULL,10,NULL,0);
  xTaskCreatePinnedToCore(capture_task,"capture",4096,NULL,15,&capture_handle,1);
  cJSON *o=event("boot");cJSON_AddStringToObject(o,"firmware","signal-workbench/2");emit(o);
  for(;;) { queued_command_t cmd;if(xQueueReceive(commands,&cmd,portMAX_DELAY)) { command(cmd.json,cmd.epoch);cJSON_Delete(cmd.json); } }
}
