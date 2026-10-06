'use strict';
const HW_UI={open:new Set(),drafts:new Map()};
function hardwareSelect(label,id,options,value,change) {
  return UI.choice({label,id,options,value,onChange:change,commit:true});
}
function hardwareField(label,control){return UI.field({label,class:'hw-field'},control);}
function hardwareNumber(label,value,change,min,max) {
  const inp=UI.input({type:'number','aria-label':label,value:String(value),min,max});
  inp.addEventListener('change',()=>{const v=Number(inp.value);if(Number.isFinite(v))change(v);});return hardwareField(label,inp);
}
function hardwareText(label,value,change){const inp=UI.input({type:'text','aria-label':label,value});inp.addEventListener('change',()=>change(inp.value.trim()));return hardwareField(label,inp);}
function hardwareDetails(key,title,...children){return UI.details({title,class:'hw-advanced',open:HW_UI.open.has(key),onToggle:open=>{if(open)HW_UI.open.add(key);else HW_UI.open.delete(key);}},...children);}
function hardwareCard(name,kind,connection,board,...children){return UI.card({class:'hw-device'},el('div',{class:'hw-device-title'},el('b',{text:name}),el('span',{class:'hw-kind',text:kind})),board,hardwareField('Connection',el('span',{class:'hw-connection',text:connection})),...children);}
function renderWiringOverview(C){
  const data=hardwareOverview(C,cfg.comps),section=el('div',{class:'hw-group hw-overview',id:'wiringOverview'},el('h3',{text:'Wiring overview'}),el('p',{class:'hint',text:'Your connection list, grouped by board. GPIO numbers use the board’s GPIO names, not header pin numbers. Changes below update this list.'}));
  for(const g of [...data.groups,...(data.unassigned.length?[{name:'Unassigned devices',kind:'',rows:data.unassigned}]:[])]){
    const card=UI.card({class:'hw-device'},el('div',{class:'hw-device-title'},el('b',{text:g.name}),el('span',{class:'hw-kind',text:BOARD_KINDS[g.kind]?.label||g.kind}))),list=el('dl',{class:'hw-wire-list'});
    for(const r of g.rows)list.append(el('dt',{text:r.device}),el('dd',{},el('span',{text:r.connection}),el('small',{text:r.note})));
    card.append(g.rows.length?list:el('p',{class:'hint',text:'No device connections assigned.'}));section.append(card);
  }
  section.append(el('p',{class:'hint',text:'Power: connect each module to its specified supply and share signal GND. Motors take power through their ESC or MOSFET stage; servos need a suitable supply. Battery sensing uses a resistor divider. This list describes signal wiring; check module pinouts and the wiring checks below before installation.'}));
  return section;
}
function renderHardware() {
  const box=$('#hardwareRows');if(!box)return;box.textContent='';
  const C=computers(),parts=cfg.comps.filter(c=>['motor','joint','sensor','latch'].includes(c.type));
  box.append(renderWiringOverview(C));
  const pinsFor=b=>ESP_PROFILES[b.kind]?ESP_PROFILES[b.kind].pins:PI_GPIO_PINS;
  const pinPicker=(b,label,id,value,key,pins,change)=>{
    const claims=hardwarePinClaims(C,cfg.comps,b).filter(x=>x.key!==key),used=new Map(claims.map(x=>[x.pin,x.name]));
    const options=[...(['sda','scl'].includes(key)?[]:[[-1,'Not connected']]),...pins.map(p=>[p,'GPIO '+p+(used.has(p)?' · '+used.get(p):''),used.has(p)])];
    if(value>=0&&!pins.includes(value))options.push([value,'GPIO '+value+' · unavailable',true]);
    return hardwareField(label,hardwareSelect(label,id,options,value,change));
  };
  const editBoard=(b,patch,key)=>editWiring(w=>{w.boards[b.id]={...hardwareBus(computers(),b),...patch};},b.id+key);
  const sensorBusFields=(b,c)=>{
    const bus=hardwareBus(C,b);
    return el('div',{class:'hw-fields'},...['sda','scl'].map(key=>pinPicker(b,c.name+' '+key.toUpperCase()+' GPIO','hw-'+key+'-'+c.id,bus[key],key,pinsFor(b),v=>editBoard(b,{[key]:Number(v)},key))));
  };
  const group=(title,description)=>{const node=el('div',{class:'hw-group'},el('h3',{text:title}),el('p',{class:'hint',text:description}));box.append(node);return node;};
  const outputs=group('Motors & servos','Choose the board, motor driver and signal GPIO. Motor power comes through an ESC or MOSFET stage; servos use a PWM signal.');
  const sensors=group('Sensors','Choose a device driver, then wire the pins shown. I²C sensors on one board share SDA and SCL; each needs its own address.');
  const core=C.boards.find(b=>b.tasks.includes('core'));
  if(core)sensors.append(UI.button({class:'btn hw-preset-btn',type:'button',id:'hw-10dof-'+core.id,text:'Use 10DOF module: MPU6050 + BMP180 + HMC5883L',onclick:()=>use10Dof(core)}));
  let cargo=null;
  for(const c of parts){
    const p=partWiring(c),owner=hardwareOwner(C,c),saved=C.wiring?.parts?.[c.id]||{};
    const update=patch=>editWiring(w=>{w.parts[c.id]={...w.parts[c.id],...patch};},c.id);
    const task=c.type==='latch'?'cargo':c.type==='sensor'&&['fix','flow'].includes(c.kind)?'navigation':'flight core';
    const automatic=!Object.hasOwn(saved,'board');
    const board=hardwareField('Board',hardwareSelect(c.name+' board','hw-board-'+c.id,[['auto','Automatic — '+task+' board'],['off','No board (off)'],...C.boards.map(b=>[b.id,b.name])],automatic?'auto':(saved.board??'off'),v=>{
      if(v==='auto')editWiring(w=>{const q={...w.parts[c.id]};delete q.board;delete q.pin;w.parts[c.id]=q;},c.id);
      else update({board:v==='off'?null:Number(v),...(c.type==='sensor'?{}:{pin:-1})});
    }));
    if(automatic)board.append(el('span',{class:'hint',text:(owner?'Currently uses '+owner.name+'.':'No board has the '+task+' task assigned.')+' Changes automatically when you move the '+task+' task to another board.'}));
    if(c.type==='motor'||c.type==='joint'){
      const brushed=c.type==='motor'&&p.driver==='brushed';
      const bus=owner&&hardwareBus(C,owner),card=hardwareCard(c.name,c.type==='motor'?'Motor':'Servo',c.type==='motor'?(brushed?'Duty PWM · active-high MOSFET':'PWM ESC signal'):'PWM servo · 50 Hz',board);
      if(c.type==='motor')card.append(hardwareField('Motor driver',hardwareSelect(c.name+' motor driver','hw-motor-driver-'+c.id,[['pwm','PWM ESC'],['brushed','Brushed motor · MOSFET']],p.driver,v=>update({driver:v}))));
      if(owner)card.append(pinPicker(owner,c.name+(brushed?' gate GPIO':' signal GPIO'),'hw-pin-'+c.id,p.pin,'part'+c.id,pinsFor(owner),v=>update({pin:Number(v)})));
      if(brushed&&bus)card.append(hardwareNumber(c.name+' PWM frequency (Hz)',bus.brushedHz,v=>editBoard(owner,{brushedHz:v},'brushedHz'),1000,30000),hardwareNumber(c.name+' maximum duty (%)',p.maxDuty??100,v=>update({maxDuty:v}),1,100),el('p',{class:'hint',text:'Frequency is shared by MOSFET motors on this board. Active-high, one direction; zero duty when stopped. Use an external gate pulldown and a suitable MOSFET power stage with flyback protection. Set the motor’s thrust/response in Airframe.'}));
      if(c.type==='motor'&&!brushed&&bus)card.append(el('p',{class:'hint',text:bus.escHz+' Hz · '+bus.escMin+'–'+bus.escMax+' µs · timing in Board settings below'}));
      if(c.type==='joint')card.append(hardwareDetails('servo'+c.id,'Pulse calibration',hardwareNumber(c.name+' centre µs',saved.center??1500,v=>update({center:v}),800,2200),hardwareNumber(c.name+' µs/radian',saved.usPerRad??(500/(Math.PI/4)),v=>update({usPerRad:v}),-2000,2000)));
      outputs.append(card);continue;
    }
    if(c.type==='latch'){
      cargo||=group('Cargo latches','A Pi can drive a servo latch with PWM or a switched latch with a digital GPIO. Set the cargo task to that Pi.');
      const card=hardwareCard(c.name,'Latch',p.driver==='pwm'?'PWM servo · 50 Hz':'Digital on/off · high = closed',board,
        hardwareField('Driver',hardwareSelect(c.name+' latch driver','hw-driver-'+c.id,[['pwm','PWM servo'],['gpio','Digital on/off']],p.driver,v=>update({driver:v,pin:v==='pwm'?18:-1}))));
      if(owner)card.append(pinPicker(owner,c.name+' signal GPIO','hw-pin-'+c.id,p.pin,'part'+c.id,p.driver==='pwm'?[18,19]:pinsFor(owner),v=>update({pin:Number(v)})));
      card.append(el('p',{class:'hint',text:owner?.kind.startsWith('pi')?'PWM GPIO 18/19 uses Pi hardware PWM. Digital outputs control an external latch driver.':'Real latch drivers currently run on a Pi.'}));cargo.append(card);continue;
    }
    const defs=DEVICE_PROFILES[c.kind],def=defs[p.driver],i2c=['imu','baro','mag'].includes(c.kind),custom=p.driver==='custom';
    const card=hardwareCard(c.name,SENSOR_KINDS[c.kind],i2c?'I²C · SDA + SCL':c.kind==='fix'?(p.port==='/dev/serial0'?'UART NMEA · Pi GPIO 14/15':'Serial NMEA · USB adapter'):'Simulation only',board,
      hardwareField('Device / driver',hardwareSelect(c.name+' device','hw-driver-'+c.id,Object.entries(defs).map(([k,d])=>[k,d.label]),p.driver,v=>update({driver:v,address:v==='custom'?(p.address||({imu:0x68,baro:0x77,mag:0x1e}[c.kind])):defs[v].addresses[0]||0}))));
    if(i2c&&owner){
      card.append(sensorBusFields(owner,c),el('p',{class:'hint',text:'Shared with all I²C sensors on '+owner.name+'. Changing either pin updates the whole bus.'}));
      if(def.addresses.length)card.append(hardwareField('I²C address',hardwareSelect(c.name+' I2C address','hw-address-'+c.id,def.addresses.map(a=>[a,a?'0x'+a.toString(16):'Auto address']),p.address,v=>update({address:Number(v)}))));
      card.append(el('p',{class:'hint',text:'Connect sensor GND to board GND. Use the module’s specified supply and 3.3 V logic.'}));
    }
    if(c.kind==='fix')card.append(hardwareText(c.name+' serial port',saved.port||'/dev/ttyUSB0',v=>update({port:v})),el('p',{class:'hint',text:saved.port==='/dev/serial0'?'GPS TX → Pi GPIO 15 (RX), GPS RX ← GPIO 14 (TX), share GND. The flight link must use another port.':'GPS TX → USB adapter RX; GPS RX ← adapter TX (if needed); share GND. A USB adapter uses no Pi GPIO. Use /dev/serial0 for the fixed GPIO 14/15 UART.'}));
    if(c.kind==='flow')card.append(el('p',{class:'hint',text:'No physical driver yet. SPI/UART pin assignment will appear when a device driver is available.'}));
    if(c.kind==='mag'&&p.driver!=='none')card.append(hardwareDetails('mag'+c.id,'Compass calibration',hardwareText('Compass offsets (µT)',saved.bias||'0,0,0',v=>update({bias:v})),hardwareText('Compass scale XYZ',saved.scale||'1,1,1',v=>update({scale:v})),el('p',{class:'hint',text:'Three comma-separated values, X,Y,Z. Set orientation in Airframe.'})));
    if(custom&&owner)card.append(UI.button({class:'btn',type:'button',text:'Edit C driver for '+owner.name,onclick:()=>{const d=$('#hw-editor-'+owner.id);if(d){d.open=true;HW_UI.open.add('driver'+owner.id);$('#hw-code-'+owner.id).focus();}}}));
    sensors.append(card);
  }
  const auxiliary=group('Power & radio','These connections belong to the board running the corresponding task. Settings are included when installing the design.');
  if(core&&ESP_PROFILES[core.kind]){
    const bus=hardwareBus(C,core),battery=cfg.comps.find(c=>c.battery);
    auxiliary.append(hardwareCard(battery?.name||'Battery voltage','Voltage sensor','Analog ADC · resistor divider',el('p',{class:'hw-owner',text:'Board: '+core.name}),
      pinPicker(core,'Battery ADC GPIO','hw-battery-'+core.id,bus.batteryPin,'battery',ESP_PROFILES[core.kind].adc,v=>editBoard(core,{batteryPin:Number(v)},'battery')),
      hardwareDetails('battery'+core.id,'Voltage divider',hardwareNumber('Battery divider ratio',bus.batteryDivider,v=>editBoard(core,{batteryDivider:v},'divider'),1,30),el('p',{class:'hint',text:'Ratio = battery voltage / voltage at the ADC pin. Connect the pack through a suitable resistor divider, never directly to a GPIO.'}))));
  }
  for(const b of C.boards.filter(b=>b.kind.startsWith('pi'))){const bus=hardwareBus(C,b);auxiliary.append(hardwareCard('Flight-controller link','Board link',(!bus.linkPort||bus.linkPort==='/dev/serial0')?'UART · Pi TX GPIO 14 / RX GPIO 15':'USB serial · no Pi GPIO',el('p',{class:'hw-owner',text:'Board: '+b.name}),hardwareText(b.name+' flight link serial port',bus.linkPort||'/dev/serial0',v=>editBoard(b,{linkPort:v},'link')),el('p',{class:'hint',text:'GPIO UART: Pi TX 14 → ESP RX; Pi RX 15 ← ESP TX; share GND. Use a USB serial path to free these Pi pins. ESP UART0 pins are fixed by chip.'})));}
  const radio=C.boards.find(b=>b.tasks.includes('tlm'));
  const rk=radioCfg.kind;
  if(radio){
    // the link itself is picked here as on the Ground tab (both ends switch at once); the card shows what it needs wired
    const pick=hardwareSelect('Link','hw-link-'+radio.id,RADIO_KINDS.filter(k=>RADIO_LINKS[k]).map(k=>[k,RADIO_LINKS[k].label]),rk,v=>{if(v===radioCfg.kind||!RADIO_LINKS[v])return;radioCfg.kind=v;save();boardsRadioCfg();if(typeof renderGs==='function')renderGs(true);renderHardware();});
    const owner=el('p',{class:'hw-owner',text:'Board: '+radio.name}),esp=!!ESP_PROFILES[radio.kind],bus=hardwareBus(C,radio),lines=el('p',{class:'hint',text:'Settings: '+radioSettingLines(radioCfg).join(' ')+'. The link\'s own settings and the binding phrase are on the Ground tab; Install sends them.'});
    if(rk==='ble'){                                                   // Bluetooth LE: the S3's or C3's own radio
      const row=radioWiringRow(radio,bus,esp,radioCfg),ok=radio.kind==='s3'||radio.kind==='c3';
      auxiliary.append(hardwareCard(row.device,'Radio','Bluetooth LE · GATT',owner,hardwareField('Link',pick),
        el('p',{class:ok?'hint':'bad',text:ok?'Built into the ESP32-S3/C3: no wiring. The drone advertises LiftLab\'s service with the binding phrase\'s mark; the command module (an ESP32-S3 or C3, or one on a laptop\'s USB) finds it and connects. Range: tens of metres.':'Bluetooth LE needs an ESP32-S3 or C3: '+radio.name+' can\'t (the ESP32\'s Bluetooth takes memory the flight code needs). Put the Telemetry & radio task on an S3 or C3, or use ESP-NOW.'}),lines));
    }else if(rk==='espnow'||rk==='wifi'){                                   // ESP-NOW, Wi-Fi: the board's own radio, nothing to wire
      const row=radioWiringRow(radio,bus,esp,radioCfg);
      auxiliary.append(hardwareCard(row.device,'Radio',rk==='wifi'?'Wi-Fi · UDP':'ESP-NOW · 802.11',owner,hardwareField('Link',pick),
        el('p',{class:rk==='espnow'&&!esp?'bad':'hint',text:rk==='espnow'?(esp?'Built into the ESP32: no wiring. The command module needs an ESP32 too (or an ESP32 on USB as its bridge). Both ends need the same binding phrase.':'ESP-NOW needs an ESP32: '+radio.name+' can\'t do it. Put the Telemetry & radio task on an ESP32, or use Wi-Fi.')
          :'The board\'s own Wi-Fi: no wiring. '+(radioCfg.sta?'It joins your network; the command module joins the same one.':'It makes the network (access point, channel '+radioCfg.channel+'); the command module joins it.')+' Both ends need the same binding phrase.'}),lines));
    }else if(rk==='nrf24'){                                           // an nRF24L01 module on SPI
      const card=hardwareCard('nRF24L01 module','Radio','SPI · '+(radioCfg.kbps||1000)+' kbit/s',owner,hardwareField('Link',pick));
      if(esp){const pins=Array.isArray(bus.nrfPins)?bus.nrfPins.slice():[-1,-1,-1,-1,-1];
        ['SCK','MOSI','MISO','CSN','CE'].forEach((n,i)=>card.append(pinPicker(radio,'Module '+n+' GPIO','hw-nrf'+i+'-'+radio.id,pins[i],'nrf'+i,i===2?hardwareInputPins(radio.kind):pinsFor(radio),v=>{const p2=(Array.isArray(hardwareBus(computers(),radio).nrfPins)?hardwareBus(computers(),radio).nrfPins:[-1,-1,-1,-1,-1]).slice();p2[i]=Number(v);editBoard(radio,{nrfPins:p2},'nrf'+i);})));
        if(pins.some(p=>p<0))card.append(el('p',{class:'bad',text:'Pick all five pins: the board needs them before it takes radio=nrf24.'}));}
      else card.append(hardwareText('SPI device',bus.nrfSpi||'/dev/spidev0.0',v=>editBoard(radio,{nrfSpi:v},'nrfspi')),hardwareNumber('CE GPIO',bus.nrfCe??25,v=>editBoard(radio,{nrfCe:v},'nrfce'),0,27));
      card.append(el('p',{class:'hint',text:'VCC to 3.3 V, never 5 V, with a 10 µF capacitor (or more) across VCC and GND right at the module: its current comes in bursts, and without one most of these modules drop packets. IRQ isn\'t used. The command module needs one too, on the same data rate and binding phrase; the address and the 8 channels it hops over come from the phrase.'+(esp?'':' On a Pi: SCK pin 23, MOSI 19, MISO 21, CSN pin 24 (CE0, /dev/spidev0.0); SPI on in raspi-config.')}),lines);
      auxiliary.append(card);
    }else{                                                            // ExpressLRS's receiver, or a serial line, on a UART
      const ser=rk==='serial',card=hardwareCard(ser?'Serial line':'ExpressLRS receiver','Radio',ser?'UART · '+(radioCfg.baud||115200)+' baud'+(radioCfg.half?', one way at a time':''):'UART · CRSF',owner,hardwareField('Link',pick));
      if(esp)card.append(pinPicker(radio,ser?'Line output → board RX GPIO':'Receiver TX → board RX GPIO','hw-rx-'+radio.id,bus.crsfRx,'rx',hardwareInputPins(radio.kind),v=>editBoard(radio,{crsfRx:Number(v)},'rx')),
        pinPicker(radio,ser?'Line input ← board TX GPIO':'Receiver RX ← board TX GPIO','hw-tx-'+radio.id,bus.crsfTx,'tx',pinsFor(radio),v=>editBoard(radio,{crsfTx:Number(v)},'tx')));
      else card.append(hardwareText(ser?'Serial line port':'Receiver serial port',bus.receiverPort||'/dev/ttyUSB1',v=>editBoard(radio,{receiverPort:v},'port')));
      if(ser){
        card.append(el('p',{class:'hint',text:'Whatever carries a UART\'s bytes to the command module: a laser diode or LED on TX and a photodiode on RX (each way), fibre transceivers, an infrared pair, a radio modem in transparent mode (HC-12, SiK, LoRa serial), or a wire. Both ends at the same speed; share GND with the transceivers. Radio modems go one way at a time: set that on the Ground tab.'}),lines);
        if(esp&&(bus.crsfRx<0||bus.crsfTx<0))card.append(el('p',{class:'bad',text:'Pick both pins: the board needs them before it takes radio=serial.'}));
      }else card.append(lines);
      auxiliary.append(card);
    }
  }
  renderGroundHardware(group('Command-module wiring','Buttons, sticks and the transmitter on the ground-side board.'),C);
  const advanced=group('Board settings & custom code','Shared timing and source code live here. Most devices work with a built-in driver and need no code edits.');
  for(const b of C.boards){const bus=hardwareBus(C,b);if(!b.tasks.includes('core')||!ESP_PROFILES[b.kind])continue;
    advanced.append(hardwareDetails('timing'+b.id,b.name+' · ESC timing',...[['escHz','ESC PWM Hz',50,490],['escMin','ESC minimum µs',800,1700],['escMax','ESC maximum µs',1300,2200]].map(([k,l,min,max])=>hardwareNumber(l,bus[k],v=>editBoard(b,{[k]:v},k),min,max))),customDriverEditor(C,b));
  }
  const report=el('div',{class:'hw-report',role:'status',id:'hardwareReport'}),errors=new Set(),warnings=new Set();
  for(const b of C.boards){const plan=boardWiringPlan(b);plan.errors.forEach(x=>errors.add(x));plan.warnings.forEach(x=>warnings.add(x));}
  groundHardwareErrors(C).forEach(x=>errors.add(x));
  for(const msg of errors)report.append(el('p',{class:'bad',text:msg}));for(const msg of warnings)report.append(el('p',{class:'hint',text:msg}));
  if(!errors.size&&!warnings.size)report.append(el('p',{class:'hint',text:'Wiring checks passed. Install uses these saved connections.'}));
  report.append(el('p',{class:'hint',text:'Flight outputs and I²C sensor drivers run on the flight-core ESP. Distributed outputs, Pi I²C sensor drivers and physical optical-flow drivers remain unsupported.'}));box.append(report);
}
function renderGroundHardware(box,C){
  const g=groundHardware(C);if(!g)return;
  const p=ESP_PROFILES[C.ground.kind];
  const fields=el('div',{class:'hw-fields'});
  const save=(key,index,value)=>{const next={...groundHardware(computers()),[key]:g[key].map((v,i)=>i===index?Number(value):v)};const D=JSON.parse(JSON.stringify(computers()));D.ground.wiring=next;setComputers(D,'ground-wiring');};
  for(const [key,values] of Object.entries(g))for(let i=0;i<values.length;i++){
    const title=key==='nrf24'?'nRF24L01 '+['SCK','MOSI','MISO','CSN','CE'][i]:key==='tx'?(i?'Module TX → board RX':'Module RX ← board TX'):({arm:'Arm button',fly:'Fly button',roll:'Roll stick ADC',pitch:'Pitch stick ADC',throttle:'Throttle stick ADC',yaw:'Yaw stick ADC',buzzer:'Buzzer output',led:'LED output'}[key]||key);
    const pins=groundPinList(C,key,i);
    const occupied=new Set(Object.entries(g).filter(([k])=>k!==key).flatMap(([,v])=>v).filter(v=>v>=0));
    const options=[...((key==='tx')?[]:[[-1,'Not connected']]),...pins.map(pin=>[pin,'GPIO '+pin+(occupied.has(pin)?' · in use':''),occupied.has(pin)])];
    if(values[i]>=0&&!pins.includes(values[i]))options.push([values[i],'GPIO '+values[i]+' · unavailable',true]);
    fields.append(hardwareField(title,hardwareSelect('Command module '+title,'hw-ground-'+key+'-'+i,options,values[i],v=>save(key,i,v))));
  }
  box.append(hardwareDetails('ground',C.ground.name+' · buttons, sticks & transmitter',el('p',{class:'hint',text:'Buttons: GPIO to GND. Sticks: analog ADC. Buzzer/LED: digital output. Transmitter: CRSF UART. Install uses these assignments; arm/fly buttons toggle on each press.'}),fields));
}
function customDriverEditor(C,b){
  const saved=C.wiring?.boards?.[b.id]||{};
  const preset=UI.select({'aria-label':b.name+' C driver preset',id:'hw-preset-'+b.id},...Object.entries(DRIVER_PRESETS).map(([key,p])=>el('option',{value:key,text:p.label})));
  const code=UI.textarea({class:'code',rows:18,spellcheck:'false','aria-label':b.name+' custom sensor C code',id:'hw-code-'+b.id});code.value=HW_UI.drafts.get(b.id)??saved.driverCode??DRIVER_PRESETS['10dof'].code;
  code.addEventListener('input',()=>HW_UI.drafts.set(b.id,code.value));
  const msg=UI.status({class:'hint',role:'status'});
  const load=UI.button({class:'btn',type:'button',text:'Load preset into editor',onclick:()=>{code.value=DRIVER_PRESETS[preset.value].code;HW_UI.drafts.set(b.id,code.value);msg.textContent='Preset loaded. Save it to include it in this design.';}});
  const apply=UI.button({class:'btn',type:'button',text:'Save driver code in design',onclick:()=>{if(code.value.length>65536){msg.textContent='Driver source limit: 64 KB.';return;}HW_UI.drafts.delete(b.id);editWiring(w=>{w.boards[b.id]={...hardwareBus(computers(),b),driverCode:code.value};},'driver'+b.id);}});
  const download=UI.button({class:'btn',type:'button',text:'Export custom_sensors.h',onclick:()=>{const a=el('a',{href:URL.createObjectURL(new Blob([code.value],{type:'text/plain'})),download:'custom_sensors.h'});document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),1000);msg.textContent='Exported source. Rebuild firmware, install the custom .bin files, then send this design’s settings.';}});
  const d=hardwareDetails('driver'+b.id,b.name+' · custom sensor drivers (C)',el('p',{class:'hint',text:'Built-in drivers need no compilation. Custom C uses the same I²C pins shown on the device cards. Select Custom C driver on those cards to enable these callbacks.'}),hardwareField('Start from a preset',preset),load,code,el('div',{class:'hw-fields'},apply,download),msg,
    hardwareDetails('build'+b.id,'Build & install instructions',el('p',{class:'hint',text:'Browser compilation is possible, but LiftLab does not yet bundle a C compiler or emulate sensor registers. Export source and compile for your ESP chip with ESP-IDF. The simulator currently uses the sensor models in Airframe.'}),el('pre',{class:'inst-code',text:'cp /path/to/custom_sensors.h runner/fc/esp32/main/custom_sensors.h\nsh tools/build_firmware.sh\n# Install → Use firmware files from this computer\n# Select firmware/'+ESP_PROFILES[b.kind].chip+'-flight/*.bin'}),el('p',{class:'hint',text:'init: 0 success. IMU read: 0 success, gyro rad/s, acceleration m/s². Barometer/compass read: 1 new sample, 0 waiting, metres / µT. custom_read/custom_write access I²C registers. Keep reads nonblocking; native driver code affects flight timing.'})));
  d.id='hw-editor-'+b.id;return d;
}
