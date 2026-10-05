#ifndef TEST_LEDC_H
#define TEST_LEDC_H
#include <stdint.h>
#define ESP_OK 0
#define LEDC_LOW_SPEED_MODE 0
#define LEDC_HIGH_SPEED_MODE 1
#define LEDC_TIMER_1 1
#define LEDC_TIMER_10_BIT 10
#define LEDC_TIMER_14_BIT 14
#define LEDC_USE_APB_CLK 1
typedef int ledc_mode_t;
typedef int ledc_channel_t;
typedef int ledc_timer_t;
typedef struct {int speed_mode,duty_resolution,timer_num;uint32_t freq_hz;int clk_cfg;} ledc_timer_config_t;
typedef struct {int gpio_num,speed_mode,channel,timer_sel;uint32_t duty;} ledc_channel_config_t;
int ledc_timer_config(const ledc_timer_config_t *c);
int ledc_channel_config(const ledc_channel_config_t *c);
int ledc_set_duty(ledc_mode_t mode,ledc_channel_t ch,uint32_t duty);
int ledc_update_duty(ledc_mode_t mode,ledc_channel_t ch);
int ledc_stop(ledc_mode_t mode,ledc_channel_t ch,uint32_t idle);
#endif
