'use strict';
// Board models and pinouts: the built-ins, a design's edits and its own models, what each pin is wired to, the
// warnings for pins a board doesn't bring out, and that it all travels with the design file.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const ctx=vm.createContext({console,atob,window:{addEventListener(){}},LAWS:{},RN_TASK_FORMULAS:{},Map,Set});
for(const file of ['board-hardware','hardware','driver-presets','boards','part-geometry','designs','board-pinouts']) vm.runInContext(fs.readFileSync('js/'+file+'.js','utf8'),ctx);
vm.runInContext('this.api={BUILTIN_BOARD_MODELS,parsePinLine,boardModel,boardModelId,boardModelErrors,boardPinUses,boardPinoutSvg,boardGpioName,hardwarePlan,boardLinks,readDesignFile,ESP_PROFILES,PI_GPIO_PINS}',ctx);const A=ctx.api;
const j=x=>JSON.stringify(x);
// every built-in parses, lists no GPIO twice, and brings out only GPIOs its chip has
for(const [id,m] of Object.entries(A.BUILTIN_BOARD_MODELS)){
  assert.equal(A.boardModelErrors(m).join(),"",id);
  for(const k of m.kinds){const g=m.cols.flat().map(A.parsePinLine).filter(p=>p.gpio!==null).map(p=>p.gpio),P=A.ESP_PROFILES[k];
    if(P)for(const n of [...P.pins,...P.adc])assert.ok(g.includes(n)||k!=='esp32'||[37,38].includes(n),id+': GPIO '+n+' usable but not on the header');
    if(k.startsWith('pi'))for(const n of A.PI_GPIO_PINS)assert.ok(g.includes(n),id+': GPIO '+n);}
}
assert.equal(j(A.parsePinLine('RX2=16')),j({label:'RX2',gpio:16,fn:'gpio'}));assert.equal(A.parsePinLine('VIN').fn,'5v');assert.equal(A.parsePinLine('GND').fn,'gnd');assert.equal(A.parsePinLine('3V3').fn,'3v3');assert.ok(Number.isNaN(A.parsePinLine('X=a').gpio));
const C={boards:[{id:1,kind:'esp32',name:'FC',tasks:['core']},{id:2,kind:'pizero',name:'Pi',tasks:['nav']}],wiring:{boards:{1:{sda:21,scl:22}},parts:{5:{board:1,driver:'mpu',address:0x68}}}};
const parts=[...Array.from({length:4},(_,i)=>({id:i+1,type:'motor',name:'M'+(i+1),pos:[0,0,0]})),{id:5,type:'sensor',kind:'imu',name:'IMU',pos:[0,0,0]}];
assert.equal(A.boardModelId(C,C.boards[0]),'esp32-devkit30');assert.equal(A.boardModelId(C,C.boards[1]),'pi-header40');
assert.equal(A.boardGpioName(C,C.boards[1],15),'pin 10 (GPIO 15)');assert.equal(A.boardGpioName(C,C.boards[0],16),'RX2 (GPIO 16)');
let U=A.boardPinUses(C,parts,C.boards[0]);
assert.ok(U.get(21)[0].text.startsWith('SDA → IMU')&&U.get(25)[0].text==='ESC signal → M1'&&U.get(1)[0].text==='TX → Pi pin 10 (GPIO 15)','pins in use, the link with the Pi\'s header pin');
assert.ok(U.get('3v3')[0].text.includes('IMU')&&U.get('gnd')[0].text.includes('Pi'),'supplies');
U=A.boardPinUses(C,parts,C.boards[1]);assert.equal(U.get(14)[0].text,'TX → FC RX0 (GPIO 3)');assert.equal(U.get(14)[0].kind,'tx');
assert.ok(A.boardLinks(C,parts)[0].wires[0].includes('pin 10 (GPIO 15)'),'the link\'s wires name the header pin');
const svg=A.boardPinoutSvg(C,parts,C.boards[0]);assert.ok(svg.startsWith('<svg')&&(svg.match(/pd-tag/g)||[]).length===4+2+2+2,'a tag per pin in use, one per supply');
// a design's own model: the board uses it, pins it lacks are flagged
C.wiring.models={'custom-x':{label:'Tiny board',kinds:['esp32'],layout:'sides',top:'USB',cols:[['3V3','GND','D21=21','D22=22'],['25=25','26=26','27=27','1=1','3=3']]}};C.wiring.boards[1].model='custom-x';
assert.equal(A.boardModel(C,C.boards[0]).label,'Tiny board');
let plan=A.hardwarePlan(C,parts,C.boards[0]);assert.ok(plan.warnings.some(w=>w.includes('GPIO 14 isn\'t on FC\'s header')),plan.warnings.join());
// an edited built-in overrides it; a model for another chip isn't used
C.wiring.models['esp32-devkit30']={...A.BUILTIN_BOARD_MODELS['esp32-devkit30'],label:'Edited'};C.wiring.boards[1].model='esp32-devkit30';
assert.ok(A.boardModel(C,C.boards[0]).edited&&A.boardModel(C,C.boards[0]).label==='Edited');
C.wiring.boards[1].model='pi-header40';assert.equal(A.boardModelId(C,C.boards[0]),'esp32-devkit30','a Pi header on an ESP32 falls back');
assert.ok(A.boardModelErrors({label:'x',cols:[['A=4','B=4'],[]]}).some(e=>e.includes('twice')));assert.ok(A.boardModelErrors({label:'x',cols:[['A=x'],[]]}).length);
// it travels with the design
const d=A.readDesignFile(JSON.stringify({frame:.45,comps:parts,computers:C})).design;assert.equal(d.computers.wiring.models['custom-x'].label,'Tiny board');
console.log('Board pinout checks passed: built-in models against the chips, pin lines, the default model per board, pin uses and supplies, header pin names in the link, the drawing, a design\'s own and edited models, pins off the header, the design file.');
