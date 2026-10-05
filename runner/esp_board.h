/* LiftLab hardware profiles. Conservative DevKit pins: reserve flash/PSRAM,
 * boot straps, native USB and UART0. Check your module's schematic before wiring. */
#ifndef LIFTLAB_ESP_BOARD_H
#define LIFTLAB_ESP_BOARD_H
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#if CONFIG_IDF_TARGET_ESP32S3
#define LB_MOTOR_PINS 4,5,6,7
#define LB_SERVO_PINS 12,13
#define LB_SDA 17
#define LB_SCL 18
#define LB_TX 15
#define LB_RX 16
#define LB_LED -1
#define LB_RATE 1000
#define LB_RADIO_UART UART_NUM_2
#define LB_RX_SIGNAL U2RXD_IN_IDX
#define LB_TX_SIGNAL U2TXD_OUT_IDX
static inline int lb_output_pin(int p) { return p == 1 || p == 2 || (p >= 4 && p <= 18) || p == 21 || (p >= 38 && p <= 42) || p == 47 || p == 48; }
#elif CONFIG_IDF_TARGET_ESP32C3
#define LB_MOTOR_PINS 4,5,6,7
#define LB_SERVO_PINS 3,10
#define LB_SDA 0
#define LB_SCL 1
#define LB_TX 3
#define LB_RX 10
#define LB_LED -1
#define LB_RATE 250
#define LB_RADIO_UART UART_NUM_1
#define LB_RX_SIGNAL U1RXD_IN_IDX
#define LB_TX_SIGNAL U1TXD_OUT_IDX
static inline int lb_output_pin(int p) { return p == 0 || p == 1 || (p >= 3 && p <= 7) || p == 10; }
#else
#define LB_MOTOR_PINS 25,26,27,14,32,33,4,13
#define LB_SERVO_PINS 16,17,18,19,23
#define LB_SDA 21
#define LB_SCL 22
#define LB_TX 17
#define LB_RX 16
#define LB_LED 2
#define LB_RATE 1000
#define LB_RADIO_UART UART_NUM_2
#define LB_RX_SIGNAL U2RXD_IN_IDX
#define LB_TX_SIGNAL U2TXD_OUT_IDX
static inline int lb_output_pin(int p) { return p == 4 || (p >= 13 && p <= 14) || (p >= 16 && p <= 19) || (p >= 21 && p <= 23) || (p >= 25 && p <= 27) || p == 32 || p == 33; }
#endif
static inline int lb_input_pin(int p) {
#if CONFIG_IDF_TARGET_ESP32
  return lb_output_pin(p) || (p >= 34 && p <= 39);
#else
  return lb_output_pin(p);
#endif
}
static inline int lb_adc_pin(int p) {
#if CONFIG_IDF_TARGET_ESP32S3
  return p >= 1 && p <= 10 && lb_input_pin(p);
#elif CONFIG_IDF_TARGET_ESP32C3
  return p >= 0 && p <= 4 && lb_input_pin(p);
#else
  return p >= 32 && p <= 39;
#endif
}
#if SOC_LEDC_SUPPORT_HS_MODE
#define LB_PWM_OUTPUTS (2 * SOC_LEDC_CHANNEL_NUM)
#else
#define LB_PWM_OUTPUTS SOC_LEDC_CHANNEL_NUM
#endif
#if CONFIG_FREERTOS_UNICORE
#define LB_FLIGHT_CPU 0
#else
#define LB_FLIGHT_CPU 1
#endif
#endif
