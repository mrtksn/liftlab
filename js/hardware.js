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
const PI_GPIO_PINS = [2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27];
function hardwareOwner(C, c) {
  const p = C.wiring && C.wiring.parts && C.wiring.parts[c.id];
  if (p && Object.prototype.hasOwnProperty.call(p,'board')) return C.boards.find(b => b.id === p.board) || null;
  const task = c.type === 'latch' ? 'cargo' : c.type === 'sensor' && ['fix','flow'].includes(c.kind) ? 'nav' : 'core';
  return C.boards.find(b => b.tasks.includes(task)) || null;
}
function hardwarePart(C, c, comps) {
  const owner = hardwareOwner(C,c), saved = C.wiring && C.wiring.parts && C.wiring.parts[c.id] || {};
  const p = owner && ESP_PROFILES[owner.kind];
  const list = comps.filter(x => x.type === c.type && (c.type !== 'sensor' || x.kind === c.kind));
  const index = list.findIndex(x => x.id === c.id);
  const choices = c.type === 'sensor' && DEVICE_PROFILES[c.kind];
  const driver = c.type==='latch' ? (saved.driver || (index<2?'pwm':'gpio')) : saved.driver && choices && choices[saved.driver] ? saved.driver : choices ? Object.keys(choices)[0] : 'pwm';
  return { ...saved, board: owner ? owner.id : null, pin: Number.isInteger(saved.pin) ? saved.pin : c.type==='latch' ? [18,19,17,27,22,23,24,25][index]??-1 : p ? (c.type === 'motor' ? p.motors : p.servos)[index] ?? -1 : -1,
    driver, address: Number.isInteger(saved.address) ? saved.address : choices ? choices[driver].addresses[0] || 0 : 0 };
}
function hardwareBus(C,b) {
  const p = ESP_PROFILES[b.kind], s = C.wiring && C.wiring.boards && C.wiring.boards[b.id] || {};
  return { sda: p ? p.i2c[0] : 2, scl: p ? p.i2c[1] : 3, escHz: 400, escMin: 1000, escMax: 2000, batteryPin:-1, batteryDivider:11, crsfRx:-1, crsfTx:-1, ...s };
}
function hardwareInputPins(kind) { const p=ESP_PROFILES[kind];return !p?PI_GPIO_PINS:kind==='esp32'?[...p.pins,34,35,36,37,38,39]:p.pins; }
function hardwarePinClaims(C,comps,b) {
  const bus=hardwareBus(C,b),claims=[];
  if(ESP_PROFILES[b.kind]) claims.push({pin:bus.sda,name:'I²C SDA',key:'sda'},{pin:bus.scl,name:'I²C SCL',key:'scl'},{pin:bus.batteryPin,name:'Battery ADC',key:'battery'},{pin:bus.crsfRx,name:'Receiver RX',key:'rx'},{pin:bus.crsfTx,name:'Receiver TX',key:'tx'});
  else { if(comps.some(c=>c.type==='sensor' && ['imu','baro','mag'].includes(c.kind) && hardwareOwner(C,c)===b)) claims.push({pin:bus.sda,name:'I²C SDA',key:'sda'},{pin:bus.scl,name:'I²C SCL',key:'scl'});
    if(b.tasks.some(t=>['nav','learn','super','cargo','tlm'].includes(t)) && (!bus.linkPort || bus.linkPort==='/dev/serial0')) claims.push({pin:14,name:'Flight-controller serial TX',key:'linkTx'},{pin:15,name:'Flight-controller serial RX',key:'linkRx'});
    for(const c of comps.filter(c=>c.type==='sensor'&&c.kind==='fix')){const p=hardwarePart(C,c,comps);if(p.board===b.id && p.port==='/dev/serial0')claims.push({pin:14,name:c.name+' UART TX',key:'gpsTx'+c.id},{pin:15,name:c.name+' UART RX',key:'gpsRx'+c.id});}
    if(bus.receiverPort==='/dev/serial0')claims.push({pin:14,name:'Receiver UART TX',key:'rxTx'},{pin:15,name:'Receiver UART RX',key:'rxRx'});
  }
  for(const c of comps) if(['motor','joint','latch'].includes(c.type)) { const p=hardwarePart(C,c,comps);if(p.board===b.id)claims.push({pin:p.pin,name:c.name,key:'part'+c.id}); }
  return claims.filter(x=>x.pin>=0);
}
function piLatchSettings(C,comps,b) {
  return comps.filter(c=>c.type==='latch').map(c=>{const p=hardwarePart(C,c,comps);return p.board!==b.id || p.pin<0?'dry':p.driver==='pwm'?'pwm'+(p.pin===19?1:0):'gpio'+p.pin;}).join(',');
}
function groundHardware(C) {
  const p=ESP_PROFILES[C.ground.kind];if(!p)return null;
  const defaults=Object.fromEntries(p.ground.split('\n').map(line=>{const [,key,value]=/^set (\w+)=(.*)$/.exec(line);return [key,value.split(',').map(Number)];}));
  return {...defaults,...C.ground.wiring};
}
function groundHardwareSettings(C) { const g=groundHardware(C);return Object.entries(g||{}).map(([key,v])=>'set '+key+'='+v.join(',')).join('\n')+'\nset latch=arm,fly\nsave\nreboot'; }
function groundHardwareErrors(C) {
  const g=groundHardware(C),p=ESP_PROFILES[C.ground.kind],errors=[],used=new Map();if(!g)return errors;
  for(const [key,values] of Object.entries(g)) for(const pin of [...new Set(values)]) {
    const pins=['roll','pitch','throttle','yaw'].includes(key)?p.adc:['tx','buzzer','led'].includes(key)?[...p.pins,...(key==='led'&&C.ground.kind==='esp32'?[2]:[])]:hardwareInputPins(C.ground.kind);
    if(!Number.isInteger(pin)|| (pin>=0&&!pins.includes(pin)) || (key==='tx'&&pin<0))errors.push('Command module '+key+': unavailable GPIO '+pin);
    if(pin>=0){if(used.has(pin))errors.push('Command module GPIO '+pin+': '+used.get(pin)+' and '+key+' overlap');used.set(pin,key);}
  }
  return errors;
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
    if(bus.batteryPin>=0 && !profile.adc.includes(bus.batteryPin))errors.push('Battery voltage: choose an available ADC1 GPIO');
    if(!Number.isFinite(bus.batteryDivider)||bus.batteryDivider<1||bus.batteryDivider>30)errors.push('Battery voltage: divider ratio must be 1–30');
    if((bus.crsfRx>=0)!==(bus.crsfTx>=0))errors.push('Receiver: assign both RX and TX, or disconnect both');
    if(bus.crsfRx>=0 && (!hardwareInputPins(b.kind).includes(bus.crsfRx)||!profile.pins.includes(bus.crsfTx)))errors.push('Receiver: unavailable RX/TX GPIO');
    const seen = new Set();
    for (const [kind,s] of Object.entries(sensors)) if (['imu','baro','mag'].includes(kind)) {
      const def=DEVICE_PROFILES[kind][s.driver];
      if (!def.addresses.includes(s.address)) errors.push(s.name + ': address is not supported by the chosen driver');
      if (def.id >= 0 && s.address) { if (seen.has(s.address)) errors.push('Two sensors use the same I²C address'); seen.add(s.address); }
    }
    if (!Number.isInteger(bus.escHz) || bus.escHz<50 || bus.escHz>490 || bus.escMin<800 || bus.escMax>2200 || bus.escMax-bus.escMin<500 || bus.escMax>1000000/bus.escHz-100) errors.push('Invalid ESC PWM frequency or pulse range');
  }
  if(profile && !b.tasks.includes('core') && b.tasks.includes('tlm') && bus.crsfRx>=0)errors.push('Receiver: standalone ESP radio firmware is not implemented; use the flight-core ESP or a telemetry Pi');
  const used=new Map();for(const claim of hardwarePinClaims(C,comps,b)){if(used.has(claim.pin))errors.push('GPIO '+claim.pin+': '+used.get(claim.pin)+' and '+claim.name+' overlap');used.set(claim.pin,claim.name);}
  for(const c of comps.filter(c=>c.type==='latch')) { const p=hardwarePart(C,c,comps);if(p.board!==b.id)continue;
    if(!b.kind.startsWith('pi') || !b.tasks.includes('cargo'))errors.push(c.name+': real latch outputs need the cargo Pi');
    if(p.driver==='pwm' && ![18,19].includes(p.pin))errors.push(c.name+': Pi hardware PWM uses GPIO 18 or 19');
    if(p.driver==='gpio' && !PI_GPIO_PINS.includes(p.pin))errors.push(c.name+': choose an available Pi GPIO');
    if(!['pwm','gpio'].includes(p.driver))errors.push(c.name+': unsupported latch driver');
  }
  const ports=new Map();
  if(!profile && b.tasks.some(t=>['nav','learn','super','cargo','tlm'].includes(t)))ports.set(bus.linkPort||'/dev/serial0','Flight-controller link');
  if(!profile && b.tasks.includes('tlm')){const port=bus.receiverPort||'/dev/ttyUSB1';if(ports.has(port))errors.push('Receiver: '+port+' is already used by '+ports.get(port));ports.set(port,'Receiver');}
  for(const c of comps.filter(c=>c.type==='sensor'&&c.kind==='fix')){const p=hardwarePart(C,c,comps);if(p.board===b.id){const port=p.port||'/dev/ttyUSB0';if(ports.has(port))errors.push(c.name+': '+port+' is already used by '+ports.get(port));ports.set(port,c.name);}}
  for(const c of comps.filter(c=>c.type==='sensor'&&c.kind==='fix')){const p=hardwarePart(C,c,comps);if(p.board===b.id && p.port && !/^\/[A-Za-z0-9_./-]+$/.test(p.port))errors.push(c.name+': use a serial device path such as /dev/ttyUSB0');}
  if(bus.receiverPort && !/^\/[A-Za-z0-9_./-]+$/.test(bus.receiverPort))errors.push('Receiver: use a serial device path such as /dev/ttyUSB1');
  if(bus.linkPort && !/^\/[A-Za-z0-9_./-]+$/.test(bus.linkPort))errors.push('Flight-controller link: use a serial device path such as /dev/serial0');
  return {bus,motors,servos,servoConfigs,sensors,errors:[...new Set(errors)],warnings};
}
function hardwareSettings(plan) {
  const csv=values=>values.map(v=>Number(Number(v).toFixed(6))).join(',');
  const {bus,motors,servos,sensors}=plan;
  const lines=['servos=','motors=','battery=-1','crsf=-1','imu=-1,0','baro=-1,0','mag=-1,0','i2c='+bus.sda+','+bus.scl,'motors='+motors.join(','),'servos='+servos.join(','),'esc_hz=50','esc_us='+bus.escMin+','+bus.escMax,'esc_hz='+bus.escHz];
  if(bus.batteryPin>=0)lines.push('battery='+bus.batteryPin+','+bus.batteryDivider);
  if(bus.crsfRx>=0 && bus.crsfTx>=0)lines.push('crsf='+bus.crsfRx+','+bus.crsfTx);
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
