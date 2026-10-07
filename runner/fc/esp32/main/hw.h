/* The flight controller's hardware: the wiring (kept in flash, changed over the link), the sensors on the I2C
 * bus, the ESC and servo pulses, and the battery voltage. */
#ifndef DFB_HW_H
#define DFB_HW_H
#include <stdint.h>
#include "fc_core.h"
#include "radio_link.h"

#define HW_VERSION 10
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
  int8_t crsf_rx, crsf_tx;                         /* the radio receiver's UART (CRSF at 420000 baud): its TX to crsf_rx, −1: none */
  int16_t elrs_rate, elrs_ratio;                   /* the ExpressLRS packet rate [Hz] and telemetry ratio (1:n) set on the radio */
  int8_t imu_driver, baro_driver, mag_driver; /* -1 disabled, 0 auto, 1/2 named, 3 custom C */
  uint8_t imu_addr, baro_addr, mag_addr; /* 0 auto, otherwise 7-bit I2C address */
  float mag_matrix[9], mag_bias[3], mag_scale[3]; /* sensor axes -> body; bias in microtesla */
  /* Appended: v2–v5 blobs retain their layout and migrate to ESC defaults. */
  uint8_t motor_driver[FC_MAX_MOTORS]; /* 0 ESC pulse PWM, 1 active-high brushed MOSFET duty PWM */
  uint8_t motor_max_pct[FC_MAX_MOTORS]; /* brushed duty ceiling, 1–100%; ESC ignores it */
  int32_t brushed_hz;                 /* one shared brushed timer frequency, 1000–30000 Hz */
  /* Appended in v7 (v6 blobs migrate to ExpressLRS and the defaults): the pilot's radio link (radio_link.h) and the
   * packet links' settings (esp_radio/radio_cfg.h). */
  int8_t radio_kind;                  /* RLINK_ELRS (its rate and ratio above), RLINK_ESPNOW, RLINK_WIFI, RLINK_SERIAL, RLINK_NRF24 */
  int8_t radio_channel;               /* ESP-NOW, Wi-Fi access point: the Wi-Fi channel, 1–13 */
  int8_t radio_opt;                   /* ESP-NOW: long range (1); Wi-Fi: joins a network (1) or makes one (0); serial: half (1) */
  int8_t radio_pad;
  char bind[32];                      /* the binding phrase, 1–31 characters (both ends the same) */
  char wifi_ssid[33], wifi_pass[64];  /* wifi=SSID,PASSWORD (empty: the defaults) */
  /* Appended in v8 (v7 blobs migrate with the default): a serial line's speed (radio=serial,BAUD; radio_opt: half). */
  int32_t radio_baud;
  /* Appended in v9 (v8 blobs migrate with no module): an nRF24L01's pins (nrf24=SCK,MOSI,MISO,CSN,CE; −1: none) and
   * its data rate (radio=nrf24,KBPS). */
  int8_t nrf_pin[5];
  int8_t nrf_pad;
  int16_t radio_kbps;
  /* Appended in v10 (v9 blobs migrate with none): a second radio link at once (radio2=; fc/lmux.h merges the two):
   * its kind (−1: none) and its settings as rlink_make takes them. */
  int8_t radio2_kind;
  int8_t radio2_pad[3];
  int32_t radio2_a, radio2_b;
} hw_config;

void hw_defaults(hw_config *c);
int hw_check(const hw_config *c, char *err, int errn);
int hw_load(hw_config *c);                         /* from flash; defaults if none. 0 ok */
int hw_save(const hw_config *c);
/* Apply one "key=value" setting (see flight.c's help); the wiring as a whole is checked too (no pin used twice).
 * Returns 0, or −1 with why in err (then nothing changed). */
int hw_set(hw_config *c, const char *line, char *err, int errn);
void hw_describe(const hw_config *c, char *out, int n);
/* The pilot's radio link as set (radio_link.h). */
void hw_radio(const hw_config *c, rlink_cfg *L);
/* The second link (radio2=): 0 and L, or −1: none. */
int hw_radio2(const hw_config *c, rlink_cfg *L);

/* airframe blob in flash */
int hw_airframe_load(uint8_t *buf, uint32_t cap, uint32_t *len);
int hw_airframe_save(const uint8_t *buf, uint32_t len);

/* sensors */
typedef struct {
  int imu;                 /* 0 none, 1 MPU-6050 family (gyro + accelerometer), 2 LIS3DH (accelerometer only), 3 custom C */
  int mag; char mag_name[32];
  int baro;                /* 0 none, 1 BMP280/BME280, 2 BMP180, 3 custom C */
  char imu_name[40], baro_name[24];
} hw_sensors;
int hw_sensors_init(const hw_config *c, hw_sensors *s, char *log, int logn);
/* One IMU sample in the sensor's axes (rad/s, m/s²). Returns 0 or an error. */
int hw_imu_read(fc_imu *m);
/* Barometer: start/finish a reading; height [m] above where it was switched on. Returns 1 when a new one came. */
int hw_baro_read(float *alt);
int hw_mag_read(float mag[3]);
void hw_gyro_calibrate(const float bias[3]);

/* outputs: throttles 0–1; zero becomes ESC minimum pulse or MOSFET zero duty, servo angles in rad */
int hw_outputs_init(const hw_config *c, char *log, int logn);
int hw_outputs_ok(int n_motors, int n_servos, char *why, int whyn);   /* 1 if each of them has a working output */
void hw_outputs_set(const fc_out *o, int n_motors, int n_servos);
void hw_outputs_safe(void);   /* ESC minimum pulses, MOSFETs at zero duty */

/* battery: pack volts, 0 if not wired */
int hw_battery_init(const hw_config *c);
float hw_battery_read(void);

#endif
