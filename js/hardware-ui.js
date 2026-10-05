'use strict';
function hardwareSelect(label,id,options,value,change) {
  const sel=el('select',{'aria-label':label,id});
  for(const [v,text,disabled] of options) sel.append(el('option',{value:String(v),text,disabled:disabled?'disabled':null}));
  sel.value=String(value); commitSelect(sel,change,'Press Enter to apply the wiring change'); return sel;
}
function hardwareNumber(label,value,change,min,max) {
  const inp=el('input',{type:'number','aria-label':label,value:String(value),min,max});
  inp.addEventListener('change',()=>{ const v=Number(inp.value); if(Number.isFinite(v)) change(v); });
  return el('label',{class:'hw-field'},el('span',{text:label}),inp);
}
function renderHardware() {
  const box=$('#hardwareRows'); if(!box) return; box.textContent='';
  const C=computers(), parts=cfg.comps.filter(c=>['motor','joint','sensor'].includes(c.type));
  const pinsFor=b=>ESP_PROFILES[b.kind] ? ESP_PROFILES[b.kind].pins : PI_GPIO_PINS;
  const pinPicker=(b,label,id,value,change)=> {
    const profile=ESP_PROFILES[b.kind], bus=hardwareBus(C,b), reserved=new Set([bus.sda,bus.scl]);
    for(const c of parts) if(c.type!=='sensor') { const p=partWiring(c); if(p.board===b.id && p.pin!==value && p.pin>=0) reserved.add(p.pin); }
    const options=[[-1,'Not connected'],...pinsFor(b).map(p=>[p,'GPIO '+p+(reserved.has(p)?' · in use':''),reserved.has(p)])];
    if(value>=0 && !pinsFor(b).includes(value)) options.push([value,'GPIO '+value+' · unavailable',true]);
    return hardwareSelect(label,id,options,value,change);
  };
  for(const b of C.boards) {
    const profile=ESP_PROFILES[b.kind], bus=hardwareBus(C,b);
    const fields=el('div',{class:'hw-fields'});
    const busPin=(key,label)=>hardwareSelect(b.name+' '+label,'hw-'+key+'-'+b.id,pinsFor(b).map(p=>[p,'GPIO '+p]),bus[key],v=>editWiring(w=>{ w.boards[b.id]={...bus,[key]:Number(v)}; },b.id+key));
    fields.append(el('label',{class:'hw-field'},el('span',{text:'I²C SDA'}),busPin('sda','SDA')),el('label',{class:'hw-field'},el('span',{text:'I²C SCL'}),busPin('scl','SCL')));
    if(b.tasks.includes('core') && profile) {
      for(const [k,l,min,max] of [['escHz','ESC PWM Hz',50,490],['escMin','ESC minimum µs',800,1700],['escMax','ESC maximum µs',1300,2200]]) fields.append(hardwareNumber(l,bus[k],v=>editWiring(w=>{w.boards[b.id]={...bus,[k]:v};},b.id+k),min,max));
    }
    const drv = b.tasks.includes('core') && profile ? customDriverEditor(C,b) : null;
    box.append(el('details',{class:'hw-board',open:b.tasks.includes('core')?'open':null},el('summary',{text:b.name+' · shared I²C bus'+(profile?'':' (planning only)')}),
      el('p',{class:'hint',text:'Sensors on this bus share SDA and SCL, each with a different address. Device power and ground are separate connections.'}),fields,
      b.tasks.includes('core') ? el('button',{class:'btn',type:'button',id:'hw-10dof-'+b.id,text:'Use 10DOF module: MPU6050 + BMP180 + HMC5883L',onclick:()=>use10Dof(b)}) : null, drv));
  }
  for(const c of parts) {
    const p=partWiring(c), owner=hardwareOwner(C,c), saved=C.wiring && C.wiring.parts && C.wiring.parts[c.id] || {};
    const update=patch=>editWiring(w=>{w.parts[c.id]={...saved,...patch};},c.id);
    const auto=c.type==='sensor' && ['fix','flow'].includes(c.kind)?'navigation':'flight core';
    const bs=hardwareSelect(c.name+' board','hw-board-'+c.id,[['auto','Follow '+auto],['off','No board (off)'],...C.boards.map(b=>[b.id,b.name])],Object.prototype.hasOwnProperty.call(saved,'board')?(saved.board??'off'):'auto',v=>{if(v==='auto') editWiring(w=>{const q={...saved};delete q.board;delete q.pin;w.parts[c.id]=q;},c.id); else update({board:v==='off'?null:Number(v),pin:-1});});
    const fields=el('div',{class:'hw-part-fields'},bs);
    if(c.type!=='sensor') {
      fields.append(el('span',{class:'hint',text:'Standard PWM '+(c.type==='motor'?'ESC':'servo')}));
      if(owner) fields.append(pinPicker(owner,c.name+' signal GPIO','hw-pin-'+c.id,p.pin,v=>update({pin:Number(v)})));
      if(c.type==='joint') {
        fields.append(hardwareNumber(c.name+' centre µs',saved.center??1500,v=>update({center:v}),800,2200));
        fields.append(hardwareNumber(c.name+' µs/radian',saved.usPerRad??(500/(Math.PI/4)),v=>update({usPerRad:v}),-3000,3000));
      }
    } else {
      const defs=DEVICE_PROFILES[c.kind], def=defs[p.driver];
      fields.append(hardwareSelect(c.name+' device','hw-driver-'+c.id,Object.entries(defs).map(([k,d])=>[k,d.label]),p.driver,v=>update({driver:v,address:v==='custom'?(p.address||({imu:0x68,baro:0x77,mag:0x1e}[c.kind])):defs[v].addresses[0]||0})));
      if(def.addresses.length) fields.append(hardwareSelect(c.name+' I2C address','hw-address-'+c.id,def.addresses.map(a=>[a,a?'0x'+a.toString(16):'Auto address']),p.address,v=>update({address:Number(v)})));
      if(c.kind==='fix') { const inp=el('input',{type:'text','aria-label':c.name+' serial port',value:saved.port||'/dev/ttyUSB0'}); inp.addEventListener('change',()=>update({port:inp.value.trim()}));fields.append(inp); }
      if(c.kind==='mag' && p.driver==='hmc') {
        for(const [key,label,dflt] of [['bias','Compass offsets (µT)','0,0,0'],['scale','Compass scale XYZ','1,1,1']]) { const inp=el('input',{type:'text','aria-label':label,value:(saved[key]||dflt),title:'Three comma-separated values, X,Y,Z'});inp.addEventListener('change',()=>update({[key]:inp.value}));fields.append(el('label',{class:'hw-field'},el('span',{text:label}),inp)); }
      }
    }
    const label=c.type==='sensor'?SENSOR_KINDS[c.kind]:c.type==='motor'?'Motor':'Servo';
    box.append(el('div',{class:'hw-part'},el('div',{},el('b',{text:c.name}),el('span',{class:'hint',text:' · '+label+(owner?' → '+owner.name:' · disconnected')})),fields));
  }
  const report=el('div',{class:'hw-report',role:'status',id:'hardwareReport'});
  for(const b of C.boards) { const plan=boardWiringPlan(b); for(const msg of plan.errors) report.append(el('p',{class:'bad',text:msg})); for(const msg of plan.warnings) report.append(el('p',{class:'hint',text:msg})); }
  if(!report.childElementCount) report.append(el('p',{class:'hint',text:'Wiring checks passed. The installer sends these saved settings to the flight board.'}));
  report.append(el('p',{class:'hint',text:'Real flight outputs, IMU, compass and barometer currently run on the flight-core board. Pi NMEA GPS is supported; distributed motor outputs, Pi I²C sensors and optical-flow hardware are not implemented. Sensor mounting and simulation noise/rates are editable in Airframe.'}));
  box.append(report);
}
function customDriverEditor(C,b) {
  const saved=C.wiring && C.wiring.boards && C.wiring.boards[b.id] || {};
  const preset=el('select',{'aria-label':b.name+' C driver preset',id:'hw-preset-'+b.id},...Object.entries(DRIVER_PRESETS).map(([key,p])=>el('option',{value:key,text:p.label})));
  const code=el('textarea',{class:'code',rows:18,spellcheck:'false','aria-label':b.name+' custom sensor C code',id:'hw-code-'+b.id});code.value=saved.driverCode||DRIVER_PRESETS['10dof'].code;
  const msg=el('p',{class:'hint',role:'status'});
  const load=el('button',{class:'btn',type:'button',text:'Load preset into editor',onclick:()=>{code.value=DRIVER_PRESETS[preset.value].code;msg.textContent='Preset loaded. Save the code to include it in this design.';fitTa(code);}});
  const apply=el('button',{class:'btn',type:'button',text:'Save driver code in design',onclick:()=>{if(code.value.length>65536){msg.textContent='Driver source limit: 64 KB.';return;}editWiring(w=>{w.boards[b.id]={...hardwareBus(C,b),driverCode:code.value};},'driver'+b.id);}});
  const download=el('button',{class:'btn',type:'button',text:'Export custom_sensors.h',onclick:()=>{
    const a=el('a',{href:URL.createObjectURL(new Blob([code.value],{type:'text/plain'})),download:'custom_sensors.h'});document.body.append(a);a.click();a.remove();setTimeout(()=>URL.revokeObjectURL(a.href),1000);
    msg.textContent='Exported source. Copy it into runner/fc/esp32/main/custom_sensors.h, build firmware, then select the resulting three .bin files in Install.';
  }});
  const info=el('p',{class:'hint',text:'Select Custom C driver on the sensor rows to use these callbacks. init returns 0 on success; IMU read returns 0 on success; barometer/compass read returns 1 for a new sample, 0 when waiting. Use custom_read/custom_write for register I²C, keep reads short, and return measurements in the documented units. Mounting and compass calibration are applied outside the custom driver.'});
  const build=el('pre',{class:'inst-code',text:'# With ESP-IDF 5.3.2 activated, in the LiftLab repository:\ncp /path/to/custom_sensors.h runner/fc/esp32/main/custom_sensors.h\nsh tools/build_firmware.sh\n# Install → Use firmware files from this computer → firmware/'+ESP_PROFILES[b.kind].chip+'-flight/*.bin'});
  return el('details',{class:'hw-driver'},el('summary',{text:'Custom low-level sensor drivers (C)'}),info,el('div',{class:'hw-fields'},preset,load),code,el('div',{class:'hw-fields'},apply,download),msg,
    el('p',{class:'hint',text:'Edited C runs on the real ESP board after compilation. This editor does not compile C in your browser; the simulation continues to use the sensor models in Airframe/The world. A compiled driver can affect flight timing and measurements.'}),build);
}
