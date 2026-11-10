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
// ESC configurations: how a flight board's PWM ESCs want their signal. One configuration per flight board: its ESC
// outputs share one timer (runner/fc/esp32/main/outputs.c). Pulse widths are in µs at escHz; escStop > 0 is a stop
// pulse below the running range (sent at zero throttle instead of escMin), as some toy-drone ESCs want; escCellsMin/Max
// is the pack the ESC takes (checked against the battery). A preset only fills these fields; any can be edited.
const ESC_PRESETS = {
  standard: { label: 'Standard PWM ESC · 400 Hz, 1000–2000 µs', escHz: 400, escMin: 1000, escMax: 2000, escStop: 0, escCellsMin: 2, escCellsMax: 6 },
  servo50: { label: 'Standard PWM ESC · 50 Hz servo rate', escHz: 50, escMin: 1000, escMax: 2000, escStop: 0, escCellsMin: 2, escCellsMax: 6 },
  toyStop: { label: 'Toy-drone arm ESC · 490 Hz with a stop pulse', escHz: 490, escMin: 1070, escMax: 1940, escStop: 550, escCellsMin: 1, escCellsMax: 1,
    note: 'From bench notes on a salvaged toy-drone arm ESC (tools/esc-control): about 490 Hz, stopped at 550 µs, starting near 1070 µs. 1940 µs is the longest pulse that leaves 100 µs low at 490 Hz; the true full-speed pulse is unmeasured.' },
};
const ESC_FIELDS = ['escHz', 'escMin', 'escMax', 'escStop', 'escCellsMin', 'escCellsMax'];
const escPresetOf = bus => Object.keys(ESC_PRESETS).find(k => ESC_FIELDS.every(f => ESC_PRESETS[k][f] === bus[f])) || 'custom';
const escDutyPct = (us, hz) => us * hz / 1e4;   // high time as a share of the period [%]
function escSummary(bus) { return bus.escHz + ' Hz · ' + (bus.escStop > 0 ? 'stop ' + bus.escStop + ' µs · ' : '') + bus.escMin + '–' + bus.escMax + ' µs'; }
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
  const driver = c.type==='motor' ? (saved.driver||'pwm') : c.type==='latch' ? (saved.driver || (index<2?'pwm':'gpio')) : saved.driver && choices && choices[saved.driver] ? saved.driver : choices ? Object.keys(choices)[0] : 'pwm';
  return { ...saved, board: owner ? owner.id : null, pin: Number.isInteger(saved.pin) ? saved.pin : c.type==='latch' ? [18,19,17,27,22,23,24,25][index]??-1 : p ? (c.type === 'motor' ? p.motors : p.servos)[index] ?? -1 : -1,
    driver, address: Number.isInteger(saved.address) ? saved.address : choices ? choices[driver].addresses[0] || 0 : 0 };
}
// Power: a rigid mass marked as a battery feeds the motors (through their ESCs or MOSFET stages) and the boards
// (through their regulators) when it's wired to the power distribution, which it is unless the design says
// otherwise (wiring.parts[id].power === false). With no wired battery on board the drone doesn't turn on.
const batteryParts = comps => comps.filter(c => c.type === 'mass' && c.battery);
const batteryWired = (C, c) => { const p = C.wiring && C.wiring.parts && C.wiring.parts[c.id]; return !(p && p.power === false); };
// The links between onboard boards: every other board with duties talks to the flight core's board over a serial
// link (921600 baud). On a Pi: its UART on GPIO 14/15 (/dev/serial0) or a USB serial adapter, to the ESP's UART0.
const LINK_CARRIES = { nav: 'navigation: attitude and sensors up 100 times a second, the acceleration and heading wanted down',
  learn: 'learning: flight data up (up to 200 times a second), test moves and the learned model down',
  super: 'health supervisor: flight data up, flight limits and motor and servo states down',
  tlm: 'radio: the receiver\'s channels and the telemetry', cargo: 'cargo: latch commands and what they hold' };
function boardLinks(C, comps = typeof cfg !== 'undefined' ? cfg.comps : []) {
  const core = C.boards.find(b => b.tasks.includes('core')); if (!core) return [];
  const wired = b => comps.some(c => ['motor', 'joint', 'sensor', 'latch'].includes(c.type) && hardwareOwner(C, c) === b);   // (a board with only devices on it: a sensor board)
  return C.boards.filter(b => b !== core && (b.tasks.length || wired(b))).map(b => {
    const coreEsp = ESP_PROFILES[core.kind], link = coreEsp && coreEsp.link, bus = hardwareBus(C, b), pi = b.kind.startsWith('pi');
    const port = bus.linkPort || '/dev/serial0', gpio = port === '/dev/serial0', mqtt = pi && bus.linkTransport === 'mqtt';
    if (mqtt) {
      const cs = hardwareBus(C, core), coreEnd = cs.wifiSsid ? 'Wi-Fi station on ' + cs.wifiSsid : 'Wi-Fi station (network not set)';
      return { a: core, b, pi, port: mqttLinkUrl(C, b), transport: 'mqtt', broker: bus.mqttBroker, ends: [{ board: core, text: coreEnd + ' → broker ' + (bus.mqttHost || '?') + ':' + bus.mqttPort }, { board: b, text: (bus.mqttBroker === 'here' ? 'runs the broker (mosquitto)' : 'a client of the broker') + ' · ' + bus.mqttPrefix + '/link/…' }],
        wires: [], baud: 0, carries: [...b.tasks.map(t => LINK_CARRIES[t]).filter(Boolean)], supported: !!link && boardHasNet(C, b),
        note: !link ? core.name + ' has no MQTT link support (not an ESP32).' : !boardHasNet(C, b) ? b.name + ' has no network: it can\'t reach the broker.' : '' };
    }
    const ends = !pi ? null : [
      { board: core, text: link ? 'UART0 · GPIO ' + link[0] + ' (TX), GPIO ' + link[1] + ' (RX)' : 'no UART pins for this chip' },
      { board: b, text: gpio ? port + ' · GPIO 14 (TX), GPIO 15 (RX)' : port + ' · USB serial adapter (no Pi GPIO)' }];
    const wires = !pi || !link ? [] : gpio ? [core.name + ' GPIO ' + link[0] + ' (TX) → ' + b.name + ' GPIO 15 (RX)', b.name + ' GPIO 14 (TX) → ' + core.name + ' GPIO ' + link[1] + ' (RX)', 'GND ↔ GND']
      : [core.name + ' GPIO ' + link[0] + ' (TX) → adapter RX', 'adapter TX → ' + core.name + ' GPIO ' + link[1] + ' (RX)', 'adapter GND ↔ ' + core.name + ' GND', 'adapter USB → ' + b.name + ' (' + port + ')'];
    return { a: core, b, pi, port, transport: 'serial', ends, wires, baud: 921600, carries: [...b.tasks.map(t => LINK_CARRIES[t]).filter(Boolean), ...(wired(b) && comps.some(c => c.type === 'sensor' && hardwareOwner(C, c) === b) ? ['sensor readings: the topics of the sensors wired to it, as the other board asks for them'] : [])],
      supported: pi && !!link, note: !pi ? 'A link between two microcontrollers is simulated; there is no wiring recipe for it yet.' : !link ? core.name + ' has no UART pins for the link.' : '' };
  });
}
function hardwareBus(C,b) {
  const p = ESP_PROFILES[b.kind], s = C.wiring && C.wiring.boards && C.wiring.boards[b.id] || {};
  return { sda: p ? p.i2c[0] : 2, scl: p ? p.i2c[1] : 3, brushedHz:20000, escHz: 400, escMin: 1000, escMax: 2000, escStop: 0, escCellsMin: 1, escCellsMax: 8, batteryPin:-1, batteryDivider:11, crsfRx:-1, crsfTx:-1, nrfPins:[-1,-1,-1,-1,-1],
    linkTransport: 'serial', mqttHost: '', mqttPort: 1883, mqttPrefix: 'liftlab', mqttBroker: 'here', net: b.kind !== 'pizero', wifiSsid: '', ...s };
}
// The link between a companion board and the flight controller: the serial line (UART wires or a USB adapter), or an
// MQTT broker over a network (runner/fc/mqtt_min.h): one link frame a message on PREFIX/link/to-fc and
// PREFIX/link/from-fc. The flight controller joins a Wi-Fi network as a station for it (its UART stays: USB). The broker
// runs on the companion (mosquitto on the Pi: one Wi-Fi hop) or elsewhere on the network (a laptop, a router: two).
const LINK_TRANSPORTS = { serial: 'Serial (UART wires or a USB adapter)', mqtt: 'MQTT broker over a network' };
const linkMqtt = (C, b) => !ESP_PROFILES[b.kind] && hardwareBus(C, b).linkTransport === 'mqtt';
const boardHasNet = (C, b) => !!ESP_PROFILES[b.kind] || !!hardwareBus(C, b).net;   // (an original Pi Zero has none: a Zero W, 2 W, or a USB adapter has)
const mqttLinkUrl = (C, b) => { const s = hardwareBus(C, b); return 'mqtt://' + s.mqttHost + (s.mqttPort !== 1883 ? ':' + s.mqttPort : '') + '/' + s.mqttPrefix; };
const mqttHostOk = s => typeof s === 'string' && /^[A-Za-z0-9.-]{1,47}$/.test(s);
const mqttPrefixOk = s => typeof s === 'string' && s.length > 0 && s.length < 48 && !/^\/|\/$|[\s+#]|[^\x21-\x7e]/.test(s);
function mqttLinkErrors(C, b) {   // the companion's end of an MQTT link: its settings and its network
  const s = hardwareBus(C, b), e = [];
  if (!boardHasNet(C, b)) e.push(b.name + ': no network for the MQTT link. The original Pi Zero has no Wi-Fi: use a Zero W or 2 W, or a USB Wi-Fi or Ethernet adapter, and mark it as having a network');
  if (!mqttHostOk(s.mqttHost)) e.push('MQTT link: the broker\'s address or name (e.g. 192.168.1.100, or the Pi\'s own address when it runs the broker)');
  if (!Number.isInteger(s.mqttPort) || s.mqttPort < 1 || s.mqttPort > 65535) e.push('MQTT link: a broker port from 1 to 65535');
  if (!mqttPrefixOk(s.mqttPrefix)) e.push('MQTT link: a topic prefix of up to 47 characters, no spaces, + or #, no / at either end');
  if (!['here', 'elsewhere'].includes(s.mqttBroker)) e.push('MQTT link: where the broker runs');
  return e;
}
function hardwareInputPins(kind) { const p=ESP_PROFILES[kind];return !p?PI_GPIO_PINS:kind==='esp32'?[...p.pins,34,35,36,37,38,39]:p.pins; }
function hardwarePinClaims(C,comps,b) {
  const bus=hardwareBus(C,b),claims=[];
  if(ESP_PROFILES[b.kind]) claims.push({pin:bus.sda,name:'I²C SDA',key:'sda'},{pin:bus.scl,name:'I²C SCL',key:'scl'},{pin:bus.batteryPin,name:'Battery ADC',key:'battery'},{pin:bus.crsfRx,name:'Receiver RX',key:'rx'},{pin:bus.crsfTx,name:'Receiver TX',key:'tx'});
  else { if(comps.some(c=>c.type==='sensor' && ['imu','baro','mag'].includes(c.kind) && hardwareOwner(C,c)===b)) claims.push({pin:bus.sda,name:'I²C SDA',key:'sda'},{pin:bus.scl,name:'I²C SCL',key:'scl'});
    if(b.tasks.some(t=>['nav','learn','super','cargo','tlm'].includes(t)) && bus.linkTransport!=='mqtt' && (!bus.linkPort || bus.linkPort==='/dev/serial0')) claims.push({pin:14,name:'Flight-controller serial TX',key:'linkTx'},{pin:15,name:'Flight-controller serial RX',key:'linkRx'});
    for(const c of comps.filter(c=>c.type==='sensor'&&c.kind==='fix')){const p=hardwarePart(C,c,comps);if(p.board===b.id && p.port==='/dev/serial0')claims.push({pin:14,name:c.name+' UART TX',key:'gpsTx'+c.id},{pin:15,name:c.name+' UART RX',key:'gpsRx'+c.id});}
    if(bus.receiverPort==='/dev/serial0')claims.push({pin:14,name:'Receiver UART TX',key:'rxTx'},{pin:15,name:'Receiver UART RX',key:'rxRx'});
  }
  if(ESP_PROFILES[b.kind] && b.tasks.includes('tlm') && radioHas('nrf24') && Array.isArray(bus.nrfPins)) ['SCK','MOSI','MISO','CSN','CE'].forEach((n,i)=>claims.push({pin:bus.nrfPins[i],name:'nRF24L01 '+n,key:'nrf'+i}));
  for(const c of comps) if(['motor','joint','latch'].includes(c.type)) { const p=hardwarePart(C,c,comps);if(p.board===b.id)claims.push({pin:p.pin,name:c.name,key:'part'+c.id}); }
  return claims.filter(x=>x.pin>=0);
}
function piLatchSettings(C,comps,b) {
  return comps.filter(c=>c.type==='latch').map(c=>{const p=hardwarePart(C,c,comps);return p.board!==b.id || p.pin<0?'dry':p.driver==='pwm'?'pwm'+(p.pin===19?1:0):'gpio'+p.pin;}).join(',');
}
function groundHardware(C) {
  const p=ESP_PROFILES[C.ground.kind];if(!p)return null;
  const defaults=Object.fromEntries(p.ground.split('\n').map(line=>{const [,key,value]=/^set (\w+)=(.*)$/.exec(line);return [key,value.split(',').map(Number)];}));
  const nrf=radioHas('nrf24'), g={...defaults,...(nrf?{nrf24:[-1,-1,-1,-1,-1]}:{}),...C.ground.wiring};   // (an nRF24L01's pins: SCK, MOSI, MISO, CSN, CE, while it's the link)
  if(!nrf)delete g.nrf24;
  return g;
}
const groundPinList=(C,key,i)=>{const p=ESP_PROFILES[C.ground.kind];return ['roll','pitch','throttle','yaw'].includes(key)?p.adc:['tx','buzzer','led'].includes(key)||(key==='nrf24'&&i!==2)?[...p.pins,...(key==='led'&&C.ground.kind==='esp32'?[2]:[])]:hardwareInputPins(C.ground.kind);};
function groundHardwareSettings(C, radio=hardwareRadio()) { const g=groundHardware(C);return [...Object.entries(g||{}).map(([key,v])=>'set '+key+'='+(key==='nrf24'&&v.some(x=>x<0)?'-1':v.join(','))),...(g&&!g.nrf24?['set nrf24=-1']:[]),...radioSettingLines(radio).map(l=>'set '+l)].join('\n')+'\nset latch=arm,fly\nsave\nreboot'; }
function groundHardwareErrors(C) {
  const g=groundHardware(C),p=ESP_PROFILES[C.ground.kind],errors=[],used=new Map();if(!g)return errors;
  for(const [key,values] of Object.entries(g)) for(const [i,pin] of values.map((v,i)=>[i,v]).filter(([i,v])=>values.indexOf(v)===i)) {
    const pins=groundPinList(C,key,i);
    if(!Number.isInteger(pin)|| (pin>=0&&!pins.includes(pin)) || (key==='tx'&&pin<0))errors.push('Command module '+key+': unavailable GPIO '+pin);
    if(key==='nrf24'&&pin<0)errors.push('Command module nRF24L01: pick all five pins (SCK, MOSI, MISO, CSN, CE)');
    if(pin>=0){if(used.has(pin))errors.push('Command module GPIO '+pin+': '+used.get(pin)+' and '+key+' overlap');used.set(pin,key);}
  }
  return errors;
}
function hardwarePlan(C, comps, b) {
  const profile = ESP_PROFILES[b.kind], bus = hardwareBus(C,b), errors = [], warnings = [], motors = [], servos = [], sensors = {};
  const servoConfigs=[],motorConfigs=[];
  const core = C.boards.find(x => x.tasks.includes('core'));
  for (const c of comps.filter(x => ['motor','joint','sensor'].includes(x.type))) {
    const p = hardwarePart(C,c,comps);
    if (c.type === 'motor' || c.type === 'joint') {
      if (!core || p.board !== core.id) errors.push(c.name + ': motor/servo output must be on the flight-core board; distributed outputs are not implemented');
      (c.type === 'motor' ? motors : servos).push(p.pin);
      if(c.type==='motor'){const maxDuty=p.maxDuty??100;if(!['pwm','brushed'].includes(p.driver))errors.push(c.name+': unsupported motor driver');if(!Number.isInteger(maxDuty)||maxDuty<1||maxDuty>100)errors.push(c.name+': duty limit must be an integer from 1–100%');motorConfigs.push({driver:p.driver,maxDuty});}
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
    if(!Number.isInteger(bus.brushedHz)||bus.brushedHz<1000||bus.brushedHz>30000)errors.push('Brushed PWM frequency must be 1000–30000 Hz');
    if (!Number.isInteger(bus.escHz) || bus.escHz<50 || bus.escHz>490) errors.push('ESC signal frequency: a whole number, 50–490 Hz (it is '+bus.escHz+')');
    else if (!Number.isFinite(bus.escMin) || !Number.isFinite(bus.escMax) || bus.escMin<800 || bus.escMax>2200) errors.push('ESC pulses: start pulse at least 800 µs, full-throttle pulse at most 2200 µs (they are '+bus.escMin+' and '+bus.escMax+')');
    else if (bus.escMax-bus.escMin<500) errors.push('ESC pulses: full throttle needs to be at least 500 µs above the start pulse ('+bus.escMin+'→'+bus.escMax+' is '+(bus.escMax-bus.escMin)+' µs)');
    else if (bus.escMax>1000000/bus.escHz-100) errors.push('ESC pulses: at '+bus.escHz+' Hz the period is '+Math.round(1e6/bus.escHz)+' µs, so the full-throttle pulse can be at most '+Math.floor(1000000/bus.escHz-100)+' µs (it is '+bus.escMax+'): lower the frequency or the pulse');
    if (bus.escStop!==0 && (!Number.isInteger(bus.escStop) || bus.escStop<300 || bus.escStop>bus.escMin-100)) errors.push('ESC stop pulse: 0 (none) or 300 µs to 100 µs under the minimum');
    if (!Number.isInteger(bus.escCellsMin) || !Number.isInteger(bus.escCellsMax) || bus.escCellsMin<1 || bus.escCellsMax>14 || bus.escCellsMin>bus.escCellsMax) errors.push('ESC cells: whole numbers, 1–14S, minimum not above maximum');
    else if (motorConfigs.some(m=>m.driver==='pwm') && typeof battCfg==='function') { const n=battCfg().cells; if (n<bus.escCellsMin || n>bus.escCellsMax) warnings.push('The battery is '+n+'S, but '+b.name+'’s ESC configuration takes '+(bus.escCellsMin===bus.escCellsMax?bus.escCellsMin+'S':bus.escCellsMin+'–'+bus.escCellsMax+'S')); }
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
  if(!profile && bus.linkTransport==='mqtt' && b.tasks.some(t=>['nav','learn','super','cargo','tlm'].includes(t))) errors.push(...mqttLinkErrors(C,b));
  if(profile && b.tasks.includes('core')) {   // the flight controller's end of the MQTT links
    const mq=C.boards.filter(x=>x!==b && linkMqtt(C,x) && x.tasks.some(t=>['nav','learn','super','cargo','tlm'].includes(t)));
    if(mq.length){
      if(!bus.wifiSsid || bus.wifiSsid.length>32 || /[,\x00-\x1f]/.test(bus.wifiSsid)) errors.push('MQTT link: the Wi-Fi network '+b.name+' joins (its name; up to 32 characters, no commas)');
      if(radioHas('wifi') || radioHas('espnow') || peerSettingLines(hardwareRadio()||{}).after.length) errors.push('MQTT link: '+b.name+'\'s Wi-Fi is the pilot\'s radio or the other drones\' link here; not yet both (use ExpressLRS, nRF24L01, a serial line or Bluetooth LE for the radio)');
      if(new Set(mq.map(x=>hardwareBus(C,x).mqttPrefix)).size>1) errors.push('MQTT link: '+b.name+' takes one topic prefix: give the companions the same one');
      warnings.push(b.name+': set its network once in its console, wifi=SSID,PASSWORD (or just wifi=SSID for an open one): LiftLab doesn\'t keep Wi-Fi passwords.');
    }
  }
  if(!profile && bus.linkTransport!=='mqtt' && b.tasks.some(t=>['nav','learn','super','cargo','tlm'].includes(t)))ports.set(bus.linkPort||'/dev/serial0','Flight-controller link');
  if(!profile && b.tasks.includes('tlm')){const port=bus.receiverPort||'/dev/ttyUSB1';if(ports.has(port))errors.push('Receiver: '+port+' is already used by '+ports.get(port));ports.set(port,'Receiver');}
  for(const c of comps.filter(c=>c.type==='sensor'&&c.kind==='fix')){const p=hardwarePart(C,c,comps);if(p.board===b.id){const port=p.port||'/dev/ttyUSB0';if(ports.has(port))errors.push(c.name+': '+port+' is already used by '+ports.get(port));ports.set(port,c.name);}}
  for(const c of comps.filter(c=>c.type==='sensor'&&c.kind==='fix')){const p=hardwarePart(C,c,comps);if(p.board===b.id && p.port && !/^\/[A-Za-z0-9_./-]+$/.test(p.port))errors.push(c.name+': use a serial device path such as /dev/ttyUSB0');}
  if(bus.receiverPort && !/^\/[A-Za-z0-9_./-]+$/.test(bus.receiverPort))errors.push('Receiver: use a serial device path such as /dev/ttyUSB1');
  if(bus.linkPort && !/^\/[A-Za-z0-9_./-]+$/.test(bus.linkPort))errors.push('Flight-controller link: use a serial device path such as /dev/serial0');
  const mqttCompanion=profile && b.tasks.includes('core') ? C.boards.find(x=>x!==b && linkMqtt(C,x) && x.tasks.some(t=>['nav','learn','super','cargo','tlm'].includes(t))) : null;
  return {bus,motors,motorConfigs,servos,servoConfigs,sensors,errors:[...new Set(errors)],warnings:[...new Set(warnings)],radioBoard:b.tasks.includes('tlm'),mqtt:mqttCompanion?hardwareBus(C,mqttCompanion):null};
}
// The pilot's radio link as settings lines (runner/fc/radio_link.h rlink_parse; the packet links' binding phrase):
// radio=elrs,RATE,RATIO | espnow,CHANNEL[,lr] | wifi,ap,CHANNEL | wifi,sta | serial,BAUD[,half] | nrf24,KBPS, then bind=PHRASE
// for a packet link. (A serial line is on the receiver's pins: crsf= comes first in the settings, as the board wants.)
// r: radioCfg (link.js) or one like it; none (a test without the page): no lines.
const hardwareRadio = () => typeof radioCfg !== 'undefined' ? radioCfg : null;
// r2: the second link (radioCfg2; kind '' none): radio2=… (or radio2=none, to clear one set before).
const hardwareRadio2 = () => typeof radioCfg2 !== 'undefined' ? radioCfg2 : null;
// Is this kind one of the links (the first, or a second of another kind)?
const radioHas = k => hardwareRadio()?.kind===k || (hardwareRadio2()?.kind===k && hardwareRadio2().kind!==hardwareRadio()?.kind);
function radioSpec(r) {
  if (r.kind === 'espnow') return 'espnow,'+(r.channel||1)+(r.lr?',lr':'');
  if (r.kind === 'wifi') return r.sta?'wifi,sta':'wifi,ap,'+(r.channel||1);
  if (r.kind === 'serial') return 'serial,'+(r.baud||115200)+(['',',half',',up',',down'][+r.half]||'');
  if (r.kind === 'ble') return 'ble';
  if (r.kind === 'nrf24') return 'nrf24,'+(r.kbps||1000);
  return 'elrs,'+(r.rate||250)+','+(r.ratio||4);
}
function radioSettingLines(r, r2 = r === hardwareRadio() ? hardwareRadio2() : null) {
  if (!r) return [];
  const two = r2 && r2.kind && r2.kind !== r.kind, lines = ['radio='+radioSpec(r)];
  if (r2) lines.push('radio2='+(two ? radioSpec(r2) : 'none'));
  if (r.kind !== 'elrs' || (two && r2.kind !== 'elrs')) lines.push('bind='+(r.bind||'liftlab'));
  return lines;
}
// The other drones (runner/fc/peer.h over ESP-NOW, on the flight ESP32): peers=CHANNEL (an ESP-NOW link's own, else
// the one set on the Ground tab) and fleet=PHRASE, or peers=off (also beside a Wi-Fi or Bluetooth link: not yet).
// The first line clears what was set before, so a new link's channel isn't refused against the old peers'.
function peerSettingLines(r, r2 = r === hardwareRadio() ? hardwareRadio2() : null) {
  if (!r) return { before: [], after: [] };
  const links = [r, r2 && r2.kind && r2.kind !== r.kind ? r2 : null].filter(Boolean), en = links.find(l => l.kind === 'espnow');
  const on = !!r.peers && !links.some(l => l.kind === 'wifi' || l.kind === 'ble');
  return { before: ['peers=off'], after: on ? ['peers=' + (en ? en.channel || 1 : r.peerCh || 1), 'fleet=' + (r.fleet || 'liftlab')] : [] };
}
// What each link needs wired to the radio's board: an ExpressLRS receiver on a UART; nothing for ESP-NOW (built into
// the ESP32) or Wi-Fi (the ESP32's or the Pi's own); a serial line on the receiver's UART pins (or a Pi's port).
function radioWiringRow(b,bus,esp,r) {
  const kind=r?.kind||'elrs';
  if(kind==='espnow')return {device:'ESP-NOW radio',connection:esp?'Built into the ESP32 · no wiring':'Not available on '+b.kind+' · needs an ESP32',note:'Channel '+(r.channel||1)+(r.lr?' · long range':'')+' · the command module needs an ESP32 too'};
  if(kind==='wifi')return {device:'Wi-Fi radio',connection:'Built into the board · no wiring',note:(r.sta?'Joins a network':'Makes the network (access point), channel '+(r.channel||1))+' · UDP port 14570'};
  if(kind==='ble')return {device:'Bluetooth LE radio',connection:b.kind==='s3'||b.kind==='c3'?'Built into the ESP32-S3/C3 · no wiring':'Not available on '+b.kind+' · needs an ESP32-S3 or C3',note:'the drone advertises; the command module (an ESP32-S3 or C3) connects'};
  if(kind==='nrf24')return {device:'nRF24L01 module',connection:esp?(bus.nrfPins&&bus.nrfPins.every(p=>p>=0)?'SPI · SCK '+bus.nrfPins[0]+', MOSI '+bus.nrfPins[1]+', MISO '+bus.nrfPins[2]+', CSN '+bus.nrfPins[3]+', CE '+bus.nrfPins[4]:'SPI · not connected'):(bus.nrfSpi||'/dev/spidev0.0')+' · CE GPIO '+(bus.nrfCe??25),note:(r.kbps||1000)+' kbit/s · 3.3 V with a 10 µF capacitor at the module'};
  if(kind==='serial')return {device:'Serial line',connection:esp?(bus.crsfRx>=0?'GPIO '+bus.crsfRx:'Not connected')+' (RX) ← the line\'s output; '+(bus.crsfTx>=0?'GPIO '+bus.crsfTx:'Not connected')+' (TX) → its input':(bus.receiverPort||'/dev/ttyUSB1')+' · a serial port',note:(r.baud||115200)+' baud'+(['',', one way at a time',', up only',', down only'][+r.half]||'')+' · laser, fibre, infrared, a radio modem or a wire · share GND'};
  return {device:'ExpressLRS receiver',connection:esp?(bus.crsfRx>=0?'GPIO '+bus.crsfRx:'Not connected')+' (RX) ← receiver TX; '+(bus.crsfTx>=0?'GPIO '+bus.crsfTx:'Not connected')+' (TX) → receiver RX':(bus.receiverPort||'/dev/ttyUSB1')+' · USB serial adapter',note:'CRSF UART · share GND'};
}
function hardwareSettings(plan, radio=hardwareRadio()) {
  const csv=values=>values.map(v=>Number(Number(v).toFixed(6))).join(',');
  const {bus,motors,servos,sensors,motorConfigs}=plan;
  const lines=['servos=','motors=','battery=-1','crsf=-1','nrf24=-1','imu=-1,0','baro=-1,0','mag=-1,0','i2c='+bus.sda+','+bus.scl,'motors='+motors.join(','),'servos='+servos.join(','),'esc_hz=50','esc_us='+bus.escMin+','+bus.escMax+(bus.escStop>0?','+bus.escStop:''),'esc_hz='+bus.escHz];
  if(typeof designVref==='function')lines.push('vref='+designVref());   // the pack voltage the exported airframe's tables are for
  lines.push('motor_driver='+motorConfigs.map(m=>m.driver==='brushed'?1:0).join(','),'motor_max='+motorConfigs.map(m=>m.maxDuty).join(','),'brushed_hz='+bus.brushedHz);
  if(bus.batteryPin>=0)lines.push('battery='+bus.batteryPin+','+bus.batteryDivider);
  if(bus.crsfRx>=0 && bus.crsfTx>=0)lines.push('crsf='+bus.crsfRx+','+bus.crsfTx);
  if(plan.radioBoard && radio && radio.kind==='nrf24' && bus.nrfPins && bus.nrfPins.every(p=>p>=0))lines.push('nrf24='+bus.nrfPins.join(','));
  for (const kind of ['imu','baro','mag']) { const s=sensors[kind], def=s && DEVICE_PROFILES[kind][s.driver]; lines.push(kind+'='+(def ? def.id : -1)+','+(s ? s.address : 0)); }
  if(plan.servoConfigs.length) lines.push('servo_center='+csv(plan.servoConfigs.map(s=>s.center)),'servo_us_per_rad='+csv(plan.servoConfigs.map(s=>s.scale)));
  const mag=sensors.mag;if(mag) lines.push('mag_matrix='+csv(mag.matrix),'mag_bias='+csv(String(mag.bias||'0,0,0').split(',')),'mag_scale='+csv(String(mag.scale||'1,1,1').split(',')));
  if(plan.radioBoard) { const P=peerSettingLines(radio); lines.push(...P.before, ...radioSettingLines(radio), ...P.after); }
  if(plan.mqtt) lines.push('link_topic='+plan.mqtt.mqttPrefix,'link=mqtt,'+plan.mqtt.mqttHost+','+plan.mqtt.mqttPort);   // (after the radio: it may not be Wi-Fi with it)
  return lines;
}
// The same duty ceiling applies to simulated physical actuation and the native MOSFET adapter.
function hardwareMotorThrottle(C,c,t) {
  const p=C.wiring?.parts?.[c.id]||{},limit=p.driver==='brushed'?(p.maxDuty??100)/100:1;
  return Number.isFinite(t)?Math.max(0,Math.min(t,limit,1)):0;
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
if(typeof module!=='undefined') module.exports={DEVICE_PROFILES,hardwareOwner,hardwarePart,hardwareBus,hardwarePlan,hardwareSettings,radioSettingLines,batteryParts,batteryWired};

// Read-only connection list derived from the same saved assignments as Install.
function hardwareOverview(C, comps) {
  const core=C.boards.find(b=>b.tasks.includes('core')),radio=C.boards.find(b=>b.tasks.includes('tlm'));
  const groups=C.boards.map(b=>({id:b.id,name:b.name,kind:b.kind,rows:[]})),unassigned=[];
  {   // power first: what feeds everything else
    const bats=batteryParts(comps),g={id:'power',name:'Power',kind:'',rows:[]};
    for(const c of bats)g.rows.push(batteryWired(C,c)?{device:c.name,connection:'+ / − → power distribution: ESCs (or MOSFET stages), and each board through its regulator',note:'Share GND with every board'}:{device:c.name,connection:'Not connected',note:'Unplugged: it powers nothing'});
    if(!bats.length)g.rows.push({device:'Battery',connection:'None on the drone',note:'Nothing powers it: add a battery in Airframe'});
    else if(!bats.some(c=>batteryWired(C,c)))g.rows.push({device:'Power',connection:'No battery connected',note:'The drone doesn\'t turn on'});
    groups.unshift(g);
  }
  const pin=n=>Number.isInteger(n)&&n>=0?'GPIO '+n:'Not connected';
  for(const c of comps.filter(c=>['motor','joint','sensor','latch'].includes(c.type))){
    const p=hardwarePart(C,c,comps),b=hardwareOwner(C,c),row={device:c.name,connection:'',note:''};
    if(!b){row.connection='No board';row.note='Disconnected or task has no board';unassigned.push(row);continue;}
    const bus=hardwareBus(C,b),esp=ESP_PROFILES[b.kind];
    if(c.type==='motor'||c.type==='joint'||c.type==='latch'){
      row.connection=pin(p.pin)+' → '+(c.type==='motor'?(p.driver==='brushed'?'MOSFET gate':'ESC signal'):c.type==='joint'||p.driver==='pwm'?'servo signal':'latch driver');
      row.note=c.type==='motor'?(p.driver==='brushed'?bus.brushedHz+' Hz duty PWM · limit '+(p.maxDuty??100)+'%':escSummary(bus)):c.type==='joint'||p.driver==='pwm'?'50 Hz PWM':'Digital · high = closed';
      if(p.pin<0)row.note='Choose a GPIO';
      if(c.type==='latch'?!b.kind.startsWith('pi'):b.id!==core?.id)row.note+=' · Hardware assignment unsupported';
    }else if(['imu','baro','mag'].includes(c.kind)){
      row.connection=pin(bus.sda)+' → SDA; '+pin(bus.scl)+' → SCL';
      row.note=(DEVICE_PROFILES[c.kind][p.driver]?.label||p.driver)+' · '+(p.address?'address 0x'+p.address.toString(16):'auto address');
      if(!esp||b.id!==core?.id||p.driver==='none')row.note+=' · Hardware driver unsupported';
    }else if(c.kind==='fix'){
      const port=p.port||'/dev/ttyUSB0';row.connection=port==='/dev/serial0'?'GPIO 15 (RX) ← GPS TX; GPIO 14 (TX) → GPS RX':port+' · USB adapter RX ← GPS TX';
      row.note='NMEA serial · GPS RX optional';if(esp||!b.tasks.includes('nav'))row.note+=' · Hardware assignment unsupported';
    }else{row.connection='No physical connection';row.note='Simulation only · driver not implemented';}
    groups.find(g=>g.id===b.id).rows.push(row);
  }
  for(const b of C.boards){
    const g=groups.find(g=>g.id===b.id),bus=hardwareBus(C,b),esp=ESP_PROFILES[b.kind];
    if(esp&&b.id===core?.id)g.rows.push({device:'Battery voltage',connection:pin(bus.batteryPin)+(bus.batteryPin>=0?' ← resistor divider midpoint':''),note:bus.batteryPin>=0?'Divider ratio '+bus.batteryDivider+' · divider GND → board GND':'Optional ADC input'});
    if(b.kind.startsWith('pi')){
      const port=bus.linkPort||'/dev/serial0',link=core&&ESP_PROFILES[core.kind]?.link;
      g.rows.push({device:'Link to '+(core?.name||'flight controller'),connection:port==='/dev/serial0'&&link?'GPIO 14 (TX) → '+core.name+' GPIO '+link[1]+' (RX); GPIO 15 (RX) ← '+core.name+' GPIO '+link[0]+' (TX)':port+' · serial link',note:!core?'No flight-core board':port==='/dev/serial0'&&!link?'Target UART pins unavailable':port==='/dev/serial0'?'Share board GND':'USB serial adapter; no Pi GPIO'});
      if(core&&link)groups.find(x=>x.id===core.id).rows.push({device:'Link to '+b.name,connection:port==='/dev/serial0'?'GPIO '+link[1]+' (RX) ← '+b.name+' GPIO 14 (TX); GPIO '+link[0]+' (TX) → '+b.name+' GPIO 15 (RX)':'GPIO '+link[1]+' (RX) ← USB adapter TX; GPIO '+link[0]+' (TX) → USB adapter RX',note:port==='/dev/serial0'?'UART0 · share board GND':'Pi '+port+' · USB-to-UART adapter · share GND'});
    }else if(esp&&b.id!==core?.id&&b.tasks.length)g.rows.push({device:'Flight-controller link',connection:'Hardware routing not implemented',note:'Inter-board link is simulated; no supported wiring recipe'});
    if(b.id===radio?.id){g.rows.push(radioWiringRow(b,bus,esp,hardwareRadio()));const r2=hardwareRadio2();if(r2&&r2.kind&&r2.kind!==hardwareRadio()?.kind){const row=radioWiringRow(b,bus,esp,r2);g.rows.push({...row,device:'Second link: '+row.device});}}
  }
  const ground=groundHardware(C);
  if(ground){const g={id:'ground',name:C.ground.name,kind:C.ground.kind,rows:[]};groups.push(g);
    const labels={arm:'Arm button',fly:'Fly button',roll:'Roll stick',pitch:'Pitch stick',throttle:'Throttle stick',yaw:'Yaw stick',buzzer:'Buzzer',led:'LED'};
    for(const [key,values] of Object.entries(ground))values.forEach((n,i)=>g.rows.push({device:key==='tx'?'ExpressLRS transmitter':(labels[key]||key)+(values.length>1?' '+(i+1):''),connection:pin(n)+(n<0?'':key==='tx'?(i?' (RX) ← module TX':' (TX) → module RX'):['arm','fly'].includes(key)?' ↔ button ↔ GND':['roll','pitch','throttle','yaw'].includes(key)?' ← stick wiper':' → external '+key),note:n<0?'Disconnected':key==='tx'?((hardwareRadio()?.kind||'elrs')==='elrs'?'CRSF UART · share GND':'Not used with '+(hardwareRadio().kind==='wifi'?'Wi-Fi':'ESP-NOW (built into the ESP32)')):['roll','pitch','throttle','yaw'].includes(key)?'ADC · stick supply/GND to board logic supply/GND':['arm','fly'].includes(key)?'Active low':'Use a suitable driver / LED resistor'}));
  }else if(C.ground)groups.push({id:'ground',name:C.ground.name,kind:C.ground.kind,rows:[{device:'Command module',connection:'USB / host input',note:'GPIO wiring is not configured for this platform'}]});
  return {groups,unassigned};
}
