'use strict';
// Real firmware profiles; these limits are distinct from the simulated board's budget.
const ESP_PROFILES = {
  esp32: { chip: 'esp32', label: 'ESP32', imageId: 0, boot: 0x1000, outputs: 16, rate: 1000,
    motors: [25,26,27,14,32,33,4,13], servos: [16,17,18,19,23], i2c: [21,22], tx: [17,16], link: [1,3],
    pins: [4,13,14,16,17,18,19,21,22,23,25,26,27,32,33], adc: [32,33,34,35,36,37,38,39],
    ground: 'set tx=17,16\nset arm=25\nset fly=26\nset roll=34\nset pitch=35\nset throttle=32\nset yaw=33\nset buzzer=27\nset led=2' },
  s3: { chip: 'esp32s3', label: 'ESP32-S3', imageId: 9, boot: 0, outputs: 8, rate: 1000,
    motors: [4,5,6,7,8,9,10,11], servos: [12,13], i2c: [17,18], tx: [15,16], link: [43,44],
    pins: [1,2,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,21,38,39,40,41,42,47,48], adc: [1,2,4,5,6,7,8,9,10],
    ground: 'set tx=15,16\nset arm=4\nset fly=5\nset roll=6\nset pitch=7\nset throttle=8\nset yaw=9\nset buzzer=10\nset led=-1' },
  c3: { chip: 'esp32c3', label: 'ESP32-C3', imageId: 5, boot: 0, outputs: 6, rate: 250,
    motors: [4,5,6,7], servos: [3,10], i2c: [0,1], tx: [3,10], link: [21,20],
    pins: [0,1,3,4,5,6,7,10], adc: [0,1,3,4],
    ground: 'set tx=3,10\nset arm=5\nset fly=6\nset roll=0\nset pitch=1\nset throttle=-1\nset yaw=4\nset buzzer=7\nset led=-1' },
};
function checkFirmwareImage(bytes, profile, name) {
  if (bytes.length < 24 || bytes[0] !== 0xe9) throw new Error(name + ' is not an ESP firmware image');
  const id = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength).getUint16(12, true);
  if (id !== profile.imageId) throw new Error(name + ' is for a different chip; expected ' + profile.label);
}
function checkBoardWiring(profile, motors, servos, nm, ns) {
  const parse = text => text.trim() ? text.split(',').map(x => { if (!/^\s*\d+\s*$/.test(x)) throw new Error('Pins must be comma-separated GPIO numbers'); return +x; }) : [];
  const m = parse(motors), s = parse(servos), pins = [...m, ...s];
  if (m.length !== nm || s.length !== ns) throw new Error('Provide one pin per motor and servo in this design');
  if (pins.length > profile.outputs) throw new Error(profile.label + ' supports ' + profile.outputs + ' total motor/servo outputs');
  if (new Set(pins).size !== pins.length) throw new Error('A GPIO is assigned more than once');
  for (const p of pins) if (!profile.pins.includes(p)) throw new Error('GPIO ' + p + ' is reserved or unavailable on ' + profile.label);
  if (pins.some(p => profile.i2c.includes(p))) throw new Error('An output overlaps the default I2C pins; change I2C wiring/settings first');
  return { motors: m, servos: s };
}
if (typeof module !== 'undefined') module.exports = { ESP_PROFILES, checkFirmwareImage, checkBoardWiring };
