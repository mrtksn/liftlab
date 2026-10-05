'use strict';
// Regression checks for chip selection, image headers, checksum verification and wiring limits.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const { webcrypto } = require('node:crypto');
const { ESP_PROFILES, checkFirmwareImage, checkBoardWiring } = require('../js/board-hardware');
for (const p of Object.values(ESP_PROFILES)) {
  const used = [];
  for (const line of p.ground.split('\n')) {
    const [,key,value] = /^set (\w+)=(.*)$/.exec(line);
    const pins = value.split(',').map(Number).filter(n => n >= 0);
    if (['roll','pitch','throttle','yaw'].includes(key)) for (const pin of pins) assert.ok(p.adc.includes(pin), key + ' must use ADC1');
    used.push(...pins);
  }
  assert.equal(new Set(used).size, used.length, p.label + ' ground defaults cannot overlap');
  const b = new Uint8Array(24); b[0] = 0xe9; new DataView(b.buffer).setUint16(12, p.imageId, true);
  checkFirmwareImage(b, p, 'test.bin');
  for (const other of Object.values(ESP_PROFILES)) if (other !== p) assert.throws(() => checkFirmwareImage(b, other, 'test.bin'), /different chip/);
  assert.throws(() => checkFirmwareImage(new Uint8Array(2), p, 'bad.bin'), /not an ESP/);
  checkBoardWiring(p, p.motors.slice(0, 4).join(','), '', 4, 0);
  assert.throws(() => checkBoardWiring(p, '4,4,5,6', '', 4, 0), /more than once/);
  assert.throws(() => checkBoardWiring(p, '4,5', '', 4, 0), /one pin/);
}
assert.throws(() => checkBoardWiring(ESP_PROFILES.s3, '4,5,6,7,8,9,10,11', '12', 8, 1), /8 total/);
assert.throws(() => checkBoardWiring(ESP_PROFILES.c3, '4,5,6,7', '3,10,0', 4, 3), /6 total/);
assert.throws(() => checkBoardWiring(ESP_PROFILES.c3, '4,5,6,18', '', 4, 0), /reserved/);
assert.throws(() => checkBoardWiring(ESP_PROFILES.s3, '4,5,6,17', '', 4, 0), /I2C/);
let corrupt = false;
const ctx = vm.createContext({ console, crypto: webcrypto, navigator: {}, Uint8Array, DataView, TextEncoder, TextDecoder,
  $: () => ({ addEventListener() {} }),
  fetch: async url => { const b = fs.readFileSync(url); if (corrupt && url.endsWith('dfb_flight.bin')) b[100] ^= 1;
    return { ok: true, json: async () => JSON.parse(b), arrayBuffer: async () => b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength) }; },
});
vm.runInContext(fs.readFileSync('js/board-hardware.js','utf8'),ctx);
vm.runInContext(fs.readFileSync('js/install-ui.js','utf8')+'\nthis.audit = { firmwareParts, ESP_PROFILES, INST, espSendWiring };',ctx);
(async () => {
  for (const p of Object.values(ctx.audit.ESP_PROFILES)) for (const role of ['flight','ground']) {
    const fw = await ctx.audit.firmwareParts(role,p);
    assert.equal(fw.parts.find(x => x.name === 'bootloader.bin').address,p.boot);
    assert.equal(fw.parts.length,3);
  }
  corrupt = true;
  await assert.rejects(ctx.audit.firmwareParts('flight',ctx.audit.ESP_PROFILES.s3),/checksum/);
  corrupt = false;
  const files = ['bootloader.bin','partition-table.bin','dfb_flight.bin'].map(name => ({ name, arrayBuffer: async () => {
    const b=fs.readFileSync('firmware/esp32-flight/'+name); return b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength); } }));
  ctx.audit.INST.files=files;
  await assert.rejects(ctx.audit.firmwareParts('flight',ctx.audit.ESP_PROFILES.s3),/different chip/);
  ctx.audit.INST.target={kind:'s3'};
  ctx.actuators=()=>Array(4); ctx.joints=()=>[]; ctx.rnCrc32=()=>0;
  ctx.answers=['set servos=', 'set motors=4,5,6,7', 'saved; reboot to use them', 'disarm first'];
  vm.runInContext('espSend = async () => answers.shift();',ctx);
  const msg={textContent:'',className:''};
  await ctx.audit.espSendWiring('4,5,6,7','',msg);
  assert.match(msg.textContent,/restart was not confirmed/);
  assert.match(msg.className,/bad/);
  ctx.answers=['set servos=', 'set motors=4,5,6,7', 'saved; reboot to use them', 'rebooting'];
  await ctx.audit.espSendWiring('4,5,6,7','',msg);
  assert.match(msg.className,/good/);
  console.log('All board installation checks passed (3 chips × 2 roles, wrong-chip rejection, corruption, wiring limits).');
})().catch(e => { console.error(e); process.exitCode=1; });
