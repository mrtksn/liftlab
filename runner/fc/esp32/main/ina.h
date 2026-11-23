#ifndef LIFTLAB_INA_H
#define LIFTLAB_INA_H
#include <stdint.h>

/* TI INA219, INA226 and INA3221 power monitors on I2C (pmon=): each channel measures the voltage across a shunt
 * resistor in a pack's + lead (the current: volts / shunt ohms) and the voltage on its load side (IN−) to GND (the
 * pack's). The INA219 and INA226 have one channel, the INA3221 three. Registers are 16-bit, big-endian.
 * The chip is named by its number: 219, 226 or 3221. */
enum { LB_INA219 = 219, LB_INA226 = 226, LB_INA3221 = 3221 };

static inline int lb_ina_channels(int chip) { return chip == LB_INA3221 ? 3 : chip == LB_INA219 || chip == LB_INA226 ? 1 : 0; }
/* The configuration written at start, for the channels in use (bit k: channel k+1): continuous shunt and bus
 * conversions, about 1 ms each, a few averaged, so a full set is ready well inside the 50 ms the firmware reads at.
 *   INA219: 32 V bus range, ±320 mV shunt range, 12-bit bus, 8 shunt samples averaged.
 *   INA226: 4 averaged, 1.1 ms bus and shunt.
 *   INA3221: the channels in use, 4 averaged, 1.1 ms bus and shunt. */
static inline uint16_t lb_ina_config(int chip, int mask) {
  if (chip == LB_INA219) return 0x39DF;
  if (chip == LB_INA226) return 0x4327;
  return (uint16_t)((mask & 1 ? 0x4000 : 0) | (mask & 2 ? 0x2000 : 0) | (mask & 4 ? 0x1000 : 0) | 0x0327);
}
/* The register with channel ch's (0-based) shunt voltage (bus = 0) or bus voltage (bus = 1). */
static inline uint8_t lb_ina_reg(int chip, int ch, int bus) { return (uint8_t)(chip == LB_INA3221 ? 1 + 2 * ch + bus : 1 + bus); }
/* A register's two bytes in volts: the shunt's (signed) or the bus's. */
static inline float lb_ina_volts(int chip, int bus, const uint8_t b[2]) {
  const int16_t s = (int16_t)((uint16_t)b[0] << 8 | b[1]); const uint16_t u = (uint16_t)((uint16_t)b[0] << 8 | b[1]);
  if (chip == LB_INA3221) return (float)(s / 8) * (bus ? 8e-3f : 40e-6f);   /* bits 15..3: 8 mV, 40 µV */
  if (chip == LB_INA226) return bus ? u * 1.25e-3f : s * 2.5e-6f;
  return bus ? (u >> 3) * 4e-3f : s * 10e-6f;                               /* INA219 */
}
/* The most a channel can measure across its shunt [V] (its range, before the reading clips). */
static inline float lb_ina_range(int chip) { return chip == LB_INA3221 ? 0.16376f : chip == LB_INA226 ? 0.08192f : 0.32f; }
#endif
