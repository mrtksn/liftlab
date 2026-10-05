'use strict';
// Saved hardware connections. Pure planning functions are also used by regression tests.
const DEVICE_PROFILES = {
  imu: { auto: { label: 'Auto-detect MPU / LIS3DH', id: 0, addresses: [0,0x68,0x69,0x18,0x19] }, mpu: { label: 'MPU6050 / 6500 / 9250', id: 1, addresses: [0x68,0x69] }, lis: { label: 'LIS3DH (accelerometer only)', id: 2, addresses: [0x18,0x19] } },
  baro: { auto: { label: 'Auto-detect BMP180 / BMP280 / BME280', id: 0, addresses: [0,0x77,0x76] }, bmp280: { label: 'BMP280 / BME280', id: 1, addresses: [0x76,0x77] }, bmp180: { label: 'BMP180 / BMP085', id: 2, addresses: [0x77] } },
  mag: { none: { label: 'No hardware driver', id: -1, addresses: [0] }, hmc: { label: 'HMC5883L', id: 1, addresses: [0x1e] } },
  fix: { nmea: { label: 'NMEA GPS → Pi serial port', id: 0, addresses: [] } },
  flow: { sim: { label: 'Simulation only (no hardware driver)', id: -1, addresses: [] } },
};
for (const kind of ['imu','baro','mag']) DEVICE_PROFILES[kind].custom = {label:'Custom C driver (rebuild firmware)',id:3,addresses:Array.from({length:112},(_,i)=>i+8)};
const PI_GPIO_PINS = [2,3,4,5,6,7,8,9,10,11,12,13,16,17,18,19,20,21,22,23,24,25,26,27];
function hardwareOwner(C, c) {
  const p = C.wiring && C.wiring.parts && C.wiring.parts[c.id];
  if (p && Object.prototype.hasOwnProperty.call(p,'board')) return C.boards.find(b => b.id === p.board) || null;
  const task = c.type === 'sensor' && ['fix','flow'].includes(c.kind) ? 'nav' : 'core';
  return C.boards.find(b => b.tasks.includes(task)) || null;
}
function hardwarePart(C, c, comps) {
  const owner = hardwareOwner(C,c), saved = C.wiring && C.wiring.parts && C.wiring.parts[c.id] || {};
  const p = owner && ESP_PROFILES[owner.kind];
  const list = comps.filter(x => x.type === c.type && (c.type !== 'sensor' || x.kind === c.kind));
  const index = list.findIndex(x => x.id === c.id);
  const choices = c.type === 'sensor' && DEVICE_PROFILES[c.kind];
  const driver = saved.driver && choices && choices[saved.driver] ? saved.driver : choices ? Object.keys(choices)[0] : 'pwm';
  return { ...saved, board: owner ? owner.id : null, pin: Number.isInteger(saved.pin) ? saved.pin : p ? (c.type === 'motor' ? p.motors : p.servos)[index] ?? -1 : -1,
    driver, address: Number.isInteger(saved.address) ? saved.address : choices ? choices[driver].addresses[0] || 0 : 0 };
}
function hardwareBus(C,b) {
  const p = ESP_PROFILES[b.kind], s = C.wiring && C.wiring.boards && C.wiring.boards[b.id] || {};
  return { sda: p ? p.i2c[0] : 2, scl: p ? p.i2c[1] : 3, escHz: 400, escMin: 1000, escMax: 2000, ...s };
}
function hardwarePlan(C, comps, b) {
  const profile = ESP_PROFILES[b.kind], bus = hardwareBus(C,b), errors = [], warnings = [], motors = [], servos = [], sensors = {};
  const servoConfigs=[];
  const core = C.boards.find(x => x.tasks.includes('core'));
  for (const c of comps.filter(x => ['motor','joint','sensor'].includes(x.type))) {
    const p = hardwarePart(C,c,comps);
    if (c.type === 'motor' || c.type === 'joint') {
      if (!core || p.board !== core.id) errors.push(c.name + ': motor/servo output must be on the flight-core board; distributed outputs are not implemented');
      (c.type === 'motor' ? motors : servos).push(p.pin);
      if(c.type==='joint') { const center=p.center??1500,scale=p.usPerRad??(500/(Math.PI/4)); if(!Number.isFinite(center) || center<800 || center>2200 || !Number.isFinite(scale) || Math.abs(scale)<100 || Math.abs(scale)>2000) errors.push(c.name+': invalid servo pulse calibration');servoConfigs.push({center,scale}); }
    } else if (p.board === b.id) {
      if (sensors[c.kind]) errors.push(c.name + ': real firmware supports one sensor per kind on this bus');
      sensors[c.kind] = { ...p, name:c.name, matrix: typeof knownMount==='function'?knownMount(c):[1,0,0,0,1,0,0,0,1] };
      if(p.driver==='custom') warnings.push(c.name+': rebuild firmware with this board’s exported custom_sensors.h before installation');
      if (['imu','mag','baro'].includes(c.kind) && b.id !== (core && core.id)) errors.push(c.name + ': this sensor driver runs on the flight-core board');
      if(c.kind==='fix' && (ESP_PROFILES[b.kind] || !b.tasks.includes('nav'))) errors.push(c.name+': real NMEA GPS needs the navigation Pi');
      if (c.kind === 'flow') warnings.push(c.name + ': optical-flow hardware driver is not implemented');
      if (c.kind === 'mag' && p.driver === 'none') warnings.push(c.name + ': simulated compass only; choose HMC5883L for hardware');
      if(c.kind==='mag' && p.driver!=='none') { for(const [key,dflt] of [['bias','0,0,0'],['scale','1,1,1']]) { const v=String(p[key]||dflt).split(',').map(Number);if(v.length!==3 || v.some(n=>!Number.isFinite(n) || (key==='scale' && (n<0.01 || n>100)))) errors.push(c.name+': invalid compass '+key); } }
      if (c.kind === 'imu' && p.driver === 'lis') warnings.push(c.name + ': LIS3DH has no gyro and cannot arm for flight');
    }
  }
  if (profile && b.tasks.includes('core')) {
    try { checkBoardWiring({...profile,i2c:[bus.sda,bus.scl]},motors.join(','),servos.join(','),motors.length,servos.length); } catch(e) { errors.push(e.message); }
    if (!profile.pins.includes(bus.sda) || !profile.pins.includes(bus.scl) || bus.sda === bus.scl) errors.push('I²C needs two distinct available GPIOs');
    const seen = new Set();
    for (const [kind,s] of Object.entries(sensors)) if (['imu','baro','mag'].includes(kind)) {
      const def=DEVICE_PROFILES[kind][s.driver];
      if (!def.addresses.includes(s.address)) errors.push(s.name + ': address is not supported by the chosen driver');
      if (def.id >= 0 && s.address) { if (seen.has(s.address)) errors.push('Two sensors use the same I²C address'); seen.add(s.address); }
    }
    if (!Number.isInteger(bus.escHz) || bus.escHz<50 || bus.escHz>490 || bus.escMin<800 || bus.escMax>2200 || bus.escMax-bus.escMin<500 || bus.escMax>1000000/bus.escHz-100) errors.push('Invalid ESC PWM frequency or pulse range');
  }
  return {bus,motors,servos,servoConfigs,sensors,errors:[...new Set(errors)],warnings};
}
function hardwareSettings(plan) {
  const csv=values=>values.map(v=>Number(Number(v).toFixed(6))).join(',');
  const {bus,motors,servos,sensors}=plan;
  const lines=['servos=','motors=','imu=-1,0','baro=-1,0','mag=-1,0','i2c='+bus.sda+','+bus.scl,'motors='+motors.join(','),'servos='+servos.join(','),'esc_hz=50','esc_us='+bus.escMin+','+bus.escMax,'esc_hz='+bus.escHz];
  for (const kind of ['imu','baro','mag']) { const s=sensors[kind], def=s && DEVICE_PROFILES[kind][s.driver]; lines.push(kind+'='+(def ? def.id : -1)+','+(s ? s.address : 0)); }
  if(plan.servoConfigs.length) lines.push('servo_center='+csv(plan.servoConfigs.map(s=>s.center)),'servo_us_per_rad='+csv(plan.servoConfigs.map(s=>s.scale)));
  const mag=sensors.mag;if(mag) lines.push('mag_matrix='+csv(mag.matrix),'mag_bias='+csv(String(mag.bias||'0,0,0').split(',')),'mag_scale='+csv(String(mag.scale||'1,1,1').split(',')));
  return lines;
}
const partWiring = c => hardwarePart(computers(),c,cfg.comps);
const boardWiringPlan = b => hardwarePlan(computers(),cfg.comps,b);
function editWiring(mutator,key) {
  const C=JSON.parse(JSON.stringify(computers())); C.wiring ||= {parts:{},boards:{}}; C.wiring.parts ||= {}; C.wiring.boards ||= {};
  mutator(C.wiring); cfg.computers=fixComputers(C); undoKey='wiring:'+key; brt.sig=null; doReset(); save(); renderComputers(true);
}
function use10Dof(b) {
  const C=JSON.parse(JSON.stringify(computers())); C.wiring ||= {parts:{},boards:{}}; C.wiring.parts ||= {};
  const defs={imu:['mpu',0x68],baro:['bmp180',0x77],mag:['hmc',0x1e]};
  for(const [kind,[driver,address]] of Object.entries(defs)) {
    let c=sensorsOf(kind)[0]; if(!c) { c=mkSensor(kind,SENSOR_KINDS[kind],0,0,0.01); cfg.comps.push(c); }
    C.wiring.parts[c.id]={board:b.id,driver,address};
    c.pos=[0,0,0.01]; c.mount=[0,0,0]; c.known=true;
  }
  cfg.computers=fixComputers(C); undoKey='wiring:10dof'; brt.sig=null; structural(); doReset(); renderComputers(true);
}
if(typeof module!=='undefined') module.exports={DEVICE_PROFILES,hardwareOwner,hardwarePart,hardwareBus,hardwarePlan,hardwareSettings};
