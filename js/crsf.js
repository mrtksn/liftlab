'use strict';
// CRSF as the simulated ExpressLRS modules handle it (elrs.js): splitting the byte streams into frames, the channel
// frames the receiver writes to the drone, the link statistics both modules report. The code on either end is C:
// the drone's (runner/fc/crsf.h, tlm_crsf.c) and the command module's (runner/ground/ground_core.c).
// Frames: address, length, type, payload, CRC-8 (polynomial 0xD5) over type and payload; big-endian fields.

const CRSF = {
  GPS: 0x02, VARIO: 0x07, BATTERY: 0x08, BARO_ALT: 0x09, LINK_STATS: 0x14, RC: 0x16, ATTITUDE: 0x1E, FLIGHT_MODE: 0x21, EXT: 0x80,
  EXT_TEXT: 0xF1, EXT_ITEM: 0xD0, EXT_CMD: 0xD1,
  ADDR_FC: 0xC8, ADDR_HANDSET: 0xEA, ADDR_RX: 0xEC, ADDR_TX: 0xEE,
};
// The drone's telemetry items (runner/fc/tlm_core.h), by number: what the Ground station calls each value. The
// command module decodes them (runner/ground/ground_core.c); these are only the names for the screen.
const TLM_ITEMS = {
  5: { key: 'state', fields: ['state', 'flags'] },
  6: { key: 'motors', list: true },
  7: { key: 'pos', fields: ['x', 'y', 'z', 'vx', 'vy', 'vz'] },
  8: { key: 'nav', fields: ['tx', 'ty', 'tz', 'heading', 'bits', 'level'] },
  9: { key: 'learn', fields: ['cal', 'progress', 'learned', 'keep', 'throw', 'fitRot', 'fitForce'] },
  10: { key: 'super', fields: ['mode', 'why', 'margin', 'soc', 'cells', 'amps', 'mah'] },
  11: { key: 'parts', list: true },
  12: { key: 'link', fields: ['rssi', 'lq', 'snr', 'lost'] },
};

function crsfCrc8(b, from, to) {
  let c = 0;
  for (let i = from; i < to; i++) { c ^= b[i]; for (let k = 0; k < 8; k++) c = c & 0x80 ? ((c << 1) ^ 0xD5) & 0xFF : (c << 1) & 0xFF; }
  return c;
}
function crsfFrame(addr, type, payload) {
  const n = payload.length, f = new Uint8Array(n + 4);
  f[0] = addr; f[1] = n + 2; f[2] = type; f.set(payload, 3); f[n + 3] = crsfCrc8(f, 2, n + 3);
  return f;
}
// RC channels: 16 values −1…1 → 172…1811, 11 bits each, LSB first
function crsfRcFrame(ch) {
  const p = new Uint8Array(22); let bit = 0;
  for (let i = 0; i < 16; i++) {
    const v = Math.round(992 + clamp(ch[i] || 0, -1, 1) * (1811 - 992));
    for (let b = 0; b < 11; b++, bit++) if (v & (1 << b)) p[bit >> 3] |= 1 << (bit & 7);
  }
  return crsfFrame(CRSF.ADDR_FC, CRSF.RC, p);
}
function crsfLinkStatsFrame(L) {
  const pw = L.power <= 10 ? 1 : L.power <= 25 ? 2 : L.power <= 50 ? 8 : L.power <= 100 ? 3 : L.power <= 250 ? 7 : L.power <= 500 ? 4 : 5;
  const u8 = x => clamp(Math.round(x), 0, 255), s8 = x => (clamp(Math.round(x), -128, 127) + 256) & 0xFF;
  return crsfFrame(CRSF.ADDR_FC, CRSF.LINK_STATS, [u8(-L.upRssi), u8(-L.upRssi), u8(L.upLq), s8(L.upSnr), 0, L.rfMode, pw, u8(-L.downRssi), u8(L.downLq), s8(L.downSnr)]);
}
// A byte-stream parser: feed bytes, get whole frames back (with a good CRC).
function crsfParser() {
  const buf = new Uint8Array(64); let n = 0;
  return {
    bad: 0,
    feed(bytes, onFrame) {
      for (const b of bytes) {
        if (n === 0) { if (b === 0xC8 || b === 0xEA || b === 0xEC || b === 0xEE) buf[n++] = b; continue; }
        if (n === 1) { if (b < 2 || b > 62) { n = 0; this.bad++; continue; } buf[n++] = b; continue; }
        buf[n++] = b;
        if (n < buf[1] + 2) continue;
        const len = n; n = 0;
        if (crsfCrc8(buf, 2, len - 1) !== buf[len - 1]) { this.bad++; continue; }
        onFrame(buf.slice(0, len));
      }
    },
  };
}
// RC channels back from a frame's payload: 16 values −1…1 (what the transmitter module reads from the command module).
function crsfRcRead(p) {
  const ch = []; let bit = 0;
  for (let i = 0; i < 16; i++) { let v = 0; for (let b = 0; b < 11; b++, bit++) if (p[bit >> 3] & (1 << (bit & 7))) v |= 1 << b; ch.push(clamp((v - 992) / (1811 - 992), -1, 1)); }
  return ch;
}
