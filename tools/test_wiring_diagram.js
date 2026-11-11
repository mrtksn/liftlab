'use strict';
// The wiring diagram: built from the same claims as the wiring list, the link on its pins, pinouts per board.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const ctx=vm.createContext({console,atob,window:{addEventListener(){}},LAWS:{},RN_TASK_FORMULAS:{},Map,Set});
for(const file of ['board-hardware','hardware','driver-presets','boards','part-geometry','designs','wiring-diagram']) vm.runInContext(fs.readFileSync('js/'+file+'.js','utf8'),ctx);
vm.runInContext('this.api={wiringNetlist,wiringDiagramSvg,boardPinoutKey,BOARD_PINOUTS}',ctx);const A=ctx.api;
const C={boards:[{id:1,kind:'esp32',name:'FC',tasks:['core']},{id:2,kind:'pizero',name:'Pi',tasks:['nav']}],wiring:{boards:{1:{sda:21,scl:22}},parts:{5:{board:1,driver:'mpu',address:0x68}}}};
const parts=[...Array.from({length:4},(_,i)=>({id:i+1,type:'motor',name:'M'+(i+1),pos:[0,0,0]})),{id:5,type:'sensor',kind:'imu',name:'IMU',pos:[0,0,0]}];
for(const P of Object.values(A.BOARD_PINOUTS))if(P.left){const g=[...P.left,...P.right].map(p=>p[0]).filter(n=>n!==null);assert.equal(new Set(g).size,g.length,P.label+': a GPIO twice');}
assert.equal(A.BOARD_PINOUTS.header40.rows.flat().filter(v=>typeof v==='number').length,28,'the Pi header: 28 GPIOs');
assert.equal(A.boardPinoutKey(C,C.boards[0]),'devkit30');assert.equal(A.boardPinoutKey(C,C.boards[1]),'header40');
let N=A.wiringNetlist(C,parts);
assert.equal(N.wires.filter(w=>w.kind==='pwm').length,4,'four ESC signals');assert.equal(N.wires.filter(w=>w.dev.name==='IMU').map(w=>w.term).sort().join(),'SCL,SDA');
assert.equal(JSON.stringify(N.links.map(l=>[l.apin,l.bpin])),JSON.stringify([[1,15],[3,14],['GND','GND']]),'the link on UART0 by default');
let svg=A.wiringDiagramSvg(C,parts);assert.ok(svg.startsWith('<svg')&&(svg.match(/class="wd-wire/g)||[]).length===4+2+3+2+4,'every wire drawn (signals, I²C, link, IMU power, ESC grounds)');
C.wiring.boards[1].fcLinkTx=32;C.wiring.boards[1].fcLinkRx=34;N=A.wiringNetlist(C,parts);assert.equal(N.links.slice(0,2).map(l=>l.apin).join(),'32,34','the link on its own pins');
C.wiring.boards[1].pinout='schematic';svg=A.wiringDiagramSvg(C,parts,{power:false});assert.ok(svg.includes('GPIO 32')&&!/wd-v33/.test(svg),'schematic: only pins in use; no power wires');
C.wiring.boards[1].pinout='header40';assert.equal(A.boardPinoutKey(C,C.boards[0]),'devkit30','a pinout for another kind falls back');
assert.ok(!A.wiringDiagramSvg(C,parts,{linkTo:2}).includes('IMU'),'a link\'s diagram: only the link');
console.log('Wiring diagram checks passed: pinouts, netlist from the claims, the link on UART0 or its own pins, power wires, schematic fallback, a link\'s own diagram.');
