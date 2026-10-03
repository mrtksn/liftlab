'use strict';
// CRSF on the ground: what the handset or a ground-station app does with the frames the transmitter module hands it
// (runner/fc/crsf.h is the drone's side). Frames: address, length, type, payload, CRC-8 (polynomial 0xD5) over type
// and payload; big-endian fields.

const CRSF = {
  GPS: 0x02, VARIO: 0x07, BATTERY: 0x08, BARO_ALT: 0x09, LINK_STATS: 0x14, RC: 0x16, ATTITUDE: 0x1E, FLIGHT_MODE: 0x21, EXT: 0x80,
  EXT_TEXT: 0xF1, EXT_ITEM: 0xD0, EXT_CMD: 0xD1,
  ADDR_FC: 0xC8, ADDR_HANDSET: 0xEA, ADDR_RX: 0xEC, ADDR_TX: 0xEE,
};
// The telemetry items that travel as 0x80/0xD0 frames (runner/fc/tlm_core.h): names, fields and the scale each
// value was multiplied by (tlm_scale).
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
function tlmScale(id, k) {   // as tlm_scale in tlm_core.c
  switch (id) {
    case 6: case 11: return k === 0 ? 1 : 1000;
    case 7: return 100;
    case 8: return k < 3 ? 100 : k === 3 ? 1000 : 1;
    case 9: return k === 1 || k >= 5 ? 1000 : 1;
    case 10: return k === 2 ? 100 : k === 3 ? 1000 : k === 5 ? 10 : 1;
    case 12: return 1;
    default: return 100;
  }
}

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
  const pw = L.power <= 10 ? 1 : L.power <= 25 ? 2 : L.power <= 100 ? 3 : L.power <= 250 ? 7 : L.power <= 500 ? 4 : 5;
  const u8 = x => clamp(Math.round(x), 0, 255), s8 = x => (clamp(Math.round(x), -128, 127) + 256) & 0xFF;
  return crsfFrame(CRSF.ADDR_FC, CRSF.LINK_STATS, [u8(-L.upRssi), u8(-L.upRssi), u8(L.upLq), s8(L.upSnr), 0, L.rfMode, pw, u8(-L.downRssi), u8(L.downLq), s8(L.downSnr)]);
}
// A ground-station command (0x80/0xD1): GOTO x y z [m from home] yaw [rad]
let crsfCmdSeq = 0;
function crsfCmdFrame(cmd, values) {
  crsfCmdSeq = crsfCmdSeq % 255 + 1;
  const p = [CRSF.EXT_CMD, cmd, crsfCmdSeq];
  values.forEach((x, k) => { const q = clamp(Math.round(cmd === 1 ? x * (k === 3 ? 1000 : 100) : x), -32768, 32767) & 0xFFFF; p.push(q >> 8, q & 0xFF); });
  return crsfFrame(CRSF.ADDR_FC, CRSF.EXT, p);
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
const be16 = (p, i) => (p[i] << 8) | p[i + 1], sbe16 = (p, i) => { const v = be16(p, i); return v & 0x8000 ? v - 0x10000 : v; };
const sbe32 = (p, i) => ((p[i] << 24) | (p[i + 1] << 16) | (p[i + 2] << 8) | p[i + 3]);
const cstrAt = (p, i) => { let s = ''; for (; i < p.length - 1 && p[i]; i++) s += String.fromCharCode(p[i]); return s; };
// One frame → { kind, …values } as a ground station reads it (null if not one we know).
function crsfDecode(f) {
  const type = f[2], p = f.subarray(3, f.length - 1);
  switch (type) {
    case CRSF.ATTITUDE: return { kind: 'attitude', pitch: sbe16(p, 0) / 1e4, roll: sbe16(p, 2) / 1e4, yaw: sbe16(p, 4) / 1e4 };
    case CRSF.BATTERY: return { kind: 'battery', volts: be16(p, 0) / 10, amps: be16(p, 2) / 10, mah: (p[4] << 16) | (p[5] << 8) | p[6], pct: p[7] };
    case CRSF.GPS: return { kind: 'gps', lat: sbe32(p, 0) * 1e-7, lon: sbe32(p, 4) * 1e-7, speed: be16(p, 8) / 36, course: be16(p, 10) / 100, alt: be16(p, 12) - 1000, sats: p[14] };
    case CRSF.BARO_ALT: { const a = be16(p, 0); return { kind: 'baro', alt: a & 0x8000 ? a & 0x7FFF : (a - 10000) / 10, vz: p.length >= 4 ? sbe16(p, 2) / 100 : null }; }
    case CRSF.VARIO: return { kind: 'vario', vz: sbe16(p, 0) / 100 };
    case CRSF.FLIGHT_MODE: return { kind: 'mode', mode: cstrAt(p, 0) };
    case CRSF.LINK_STATS: return { kind: 'link', upRssi: -Math.min(p[0], p[1] || p[0]), upLq: p[2], upSnr: (p[3] << 24) >> 24, rfMode: p[5], downRssi: -p[7], downLq: p[8], downSnr: (p[9] << 24) >> 24 };
    case CRSF.EXT:
      if (p[0] === CRSF.EXT_TEXT) return { kind: 'text', sev: p[1], text: cstrAt(p, 2) };
      if (p[0] === CRSF.EXT_ITEM) {
        const id = p[1], n = p[2], d = TLM_ITEMS[id]; if (!d) return null;
        const v = []; for (let k = 0; k < n; k++) v.push(sbe16(p, 3 + 2 * k) / tlmScale(id, k));
        if (d.list) return { kind: d.key, n: v[0], values: v.slice(1) };
        const o = { kind: d.key }; d.fields.forEach((f, k) => { o[f] = v[k]; }); return o;
      }
      return null;
  }
  return null;
}
