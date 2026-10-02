/* The flight controller's hardware: the wiring (kept in flash, changed over the link), the sensors on the I2C
 * bus, the ESC and servo pulses, and the battery voltage. */
#ifndef DFB_HW_H
#define DFB_HW_H
#include <stdint.h>
#include "fc_core.h"

#define HW_VERSION 3
typedef struct {
  uint32_t version;
  int8_t motor_pin[FC_MAX_MOTORS];   /* −1: not wired */
  int8_t servo_pin[FC_MAX_JOINTS];
  int16_t esc_hz, esc_min_us, esc_max_us;          /* standard PWM ESCs: 50–490 Hz, 1000–2000 µs */
  int16_t servo_center_us[FC_MAX_JOINTS];
  float servo_us_per_rad[FC_MAX_JOINTS];           /* negative: the servo turns the other way */
  int8_t sda, scl;
  int8_t batt_pin; float batt_divider;             /* ADC pin (−1: none) and the divider's ratio (pack V / pin V) */
  int16_t rate_hz, telem_hz;                       /* control loop, telemetry */
  float vref;                                      /* the pack voltage the airframe's thrust is for (16: 4S) */
  int32_t link_baud;                               /* the Pi's serial link (and the USB port): 921600 for the learning */
} hw_config;

void hw_defaults(hw_config *c);
int hw_load(hw_config *c);                         /* from flash; defaults if none. 0 ok */
int hw_save(const hw_config *c);
/* Apply one "key=value" setting (see flight.c's help); the wiring as a whole is checked too (no pin used twice).
 * Returns 0, or −1 with why in err (then nothing changed). */
int hw_set(hw_config *c, const char *line, char *err, int errn);
void hw_describe(const hw_config *c, char *out, int n);

/* airframe blob in flash */
int hw_airframe_load(uint8_t *buf, uint32_t cap, uint32_t *len);
int hw_airframe_save(const uint8_t *buf, uint32_t len);

/* sensors */
typedef struct {
  int imu;                 /* 0 none, 1 MPU-6050 family (gyro + accelerometer), 2 LIS3DH (accelerometer only) */
  int baro;                /* 0 none, 1 BMP280/BME280 */
  char imu_name[40], baro_name[24];
} hw_sensors;
int hw_sensors_init(const hw_config *c, hw_sensors *s, char *log, int logn);
/* One IMU sample in the sensor's axes (rad/s, m/s²). Returns 0 or an error. */
int hw_imu_read(fc_imu *m);
/* Barometer: start/finish a reading; height [m] above where it was switched on. Returns 1 when a new one came. */
int hw_baro_read(float *alt);
void hw_gyro_calibrate(const float bias[3]);

/* outputs: throttles 0–1 (and exactly the ESC's minimum pulse when disarmed), servo angles in rad */
int hw_outputs_init(const hw_config *c, char *log, int logn);
int hw_outputs_ok(int n_motors, int n_servos, char *why, int whyn);   /* 1 if each of them has a working output */
void hw_outputs_set(const fc_out *o, int n_motors, int n_servos);
void hw_outputs_safe(void);   /* every ESC at its minimum pulse */

/* battery: pack volts, 0 if not wired */
int hw_battery_init(const hw_config *c);
float hw_battery_read(void);

#endif
