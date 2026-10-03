/*
 * CRSF (Crossfire / ExpressLRS serial protocol): the frames a flight controller exchanges with its radio receiver over
 * a UART (420000 baud, 8N1). The same frames reach the ground (the transmitter module hands them to the handset or a
 * ground-station app), so a telemetry frame built here is what an EdgeTX radio or a CRSF-aware app would show.
 *
 * A frame: address (0xC8 to the flight controller), length (the bytes after it: type, payload, CRC), type, payload
 * (≤ 60 bytes), CRC-8 (polynomial 0xD5, "DVB-S2") over type and payload. Multi-byte fields are big-endian.
 *
 * Standard frames used: GPS 0x02, vario 0x07, battery 0x08, barometric altitude 0x09, link statistics 0x14,
 * RC channels 0x16 (16 × 11 bits), attitude 0x1E, flight mode 0x21. Anything else rides in 0x80 frames (the type
 * ArduPilot uses, which ExpressLRS passes through): subtype 0xF1 status text (as ArduPilot's), and Drone Force
 * Bench's own subtypes 0xD0 (a telemetry item: tlm_core.h) and 0xD1 (a ground-station command).
 */
#ifndef CRSF_H
#define CRSF_H
#include <stdint.h>

#define CRSF_ADDR_FC 0xC8
#define CRSF_ADDR_HANDSET 0xEA
#define CRSF_ADDR_RX 0xEC
#define CRSF_ADDR_TX 0xEE
#define CRSF_MAX_FRAME 64
#define CRSF_MAX_PAYLOAD 60
#define CRSF_BAUD 420000

enum { CRSF_GPS = 0x02, CRSF_VARIO = 0x07, CRSF_BATTERY = 0x08, CRSF_BARO_ALT = 0x09, CRSF_LINK_STATS = 0x14, CRSF_RC = 0x16,
  CRSF_ATTITUDE = 0x1E, CRSF_FLIGHT_MODE = 0x21, CRSF_EXT = 0x80 };
enum { CRSF_EXT_TEXT = 0xF1, CRSF_EXT_ITEM = 0xD0, CRSF_EXT_CMD = 0xD1 };

/* RC channel values on the wire: 172 (−100%) … 992 (centre) … 1811 (+100%), 11 bits each. */
#define CRSF_CH_MIN 172
#define CRSF_CH_MID 992
#define CRSF_CH_MAX 1811

uint8_t crsf_crc8(const uint8_t *p, int n);
/* A whole frame from type and payload into out (≥ n + 4 bytes). Returns its length, 0 if the payload is too long. */
int crsf_frame(uint8_t *out, uint8_t addr, uint8_t type, const uint8_t *payload, int n);

/* Standard telemetry frames (into out, ≥ CRSF_MAX_FRAME bytes); each returns the frame's length. */
int crsf_battery(uint8_t *out, float volts, float amps, float used_mah, int remaining_pct);
int crsf_gps(uint8_t *out, double lat_deg, double lon_deg, float ground_speed, float course_deg, float alt_m, int sats);
int crsf_attitude(uint8_t *out, float roll, float pitch, float yaw);                       /* [rad] */
int crsf_vario(uint8_t *out, float vz);                                                    /* [m/s] */
int crsf_baro_alt(uint8_t *out, float alt_m, float vz);
int crsf_flight_mode(uint8_t *out, const char *mode);
int crsf_text(uint8_t *out, int severity, const char *text);                              /* ArduPilot's status text */

typedef struct {
  float up_rssi, up_lq, up_snr;          /* what the receiver hears from the transmitter [dBm, %, dB] */
  float down_rssi, down_lq, down_snr;    /* what the transmitter hears from the receiver */
  int rf_mode, tx_power_mw, antenna;
} crsf_link;
int crsf_link_stats(uint8_t *out, uint8_t addr, const crsf_link *L);
void crsf_link_stats_read(const uint8_t *payload, int n, crsf_link *L);

/* RC channels: 16 values, −1…1 (clamped), packed 11 bits each, LSB first. */
int crsf_rc(uint8_t *out, uint8_t addr, const float ch[16]);
void crsf_rc_read(const uint8_t *payload, float ch[16]);

/* Receiving: feed bytes one at a time; when a whole frame with a good CRC is in, returns its length (the frame is in
 * P->buf: P->buf[2] is its type, its payload from P->buf + 3, P->buf[1] − 2 bytes). −1: a bad frame was dropped. */
typedef struct { uint8_t buf[CRSF_MAX_FRAME]; int n; } crsf_parser;
int crsf_feed(crsf_parser *P, uint8_t b);
static inline uint8_t crsf_type(const crsf_parser *P) { return P->buf[2]; }
static inline const uint8_t *crsf_payload(const crsf_parser *P) { return P->buf + 3; }
static inline int crsf_payload_len(const crsf_parser *P) { return P->buf[1] - 2; }

#endif
