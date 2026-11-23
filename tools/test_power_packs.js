'use strict';
// Power sources and what each powers: batteries (the main pack and others, one per motor if you like), USB ports and
// bench supplies; each board, ESC, the servos and parts with a draw on one source; old designs' packs (what each feeds,
// a board's chosen pack) still resolving; the converters each source needs, draws against what a source gives, the
// second pack's sense input (battery2=), the wiring list and the drawing.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const ctx=vm.createContext({console,atob,window:{addEventListener(){}},LAWS:{},RN_TASK_FORMULAS:{},Map,Set});
for(const file of ['board-hardware','hardware','driver-presets','boards','part-geometry','designs','board-pinouts']) vm.runInContext(fs.readFileSync('js/'+file+'.js','utf8'),ctx);
// the Battery settings: the main pack is 1S 1.8 Ah
vm.runInContext('var battCfg=()=>({cells:1,capacity:1.8,rInt:0.05,maxA:36});this.api={PMON_CHIPS,powerMonitor,pmonIssues,pmonSetting,pmonChannelOf,hardwarePlan,hardwareSettings,sensedPack2,powerIssues,packFeeds,motorPack,mainPack,packSpec,boardPack,boardCells,boardSupply,boardSupplyOf,boardWatts,hardwareOverview,powerDiagramSvg,powerSources,powerConsumers,consumerSource,sourceSpec,POWER_PRESETS,lipoMass}',ctx);
const A=ctx.api,src=(C,comps,key)=>(A.consumerSource(C,comps,key)||{}).id??null;
const C={ground:{kind:'mac',name:'Laptop'},boards:[{id:1,kind:'esp32',name:'Flight controller',tasks:['core','tlm']},{id:2,kind:'pizero',name:'Pi Zero',tasks:['nav']}],wiring:{parts:{},boards:{}}};
const motors=Array.from({length:4},(_,i)=>({id:i+1,type:'motor',name:'M'+i,pos:[0,0,0]}));
const main={id:10,type:'mass',battery:true,name:'Battery'},small={id:11,type:'mass',battery:true,name:'Board pack',pack:{cells:2,capacity:0.45}};

// one pack: it powers everything, as before
let comps=[...motors,main];
assert.equal(A.packFeeds(C,main),'all');assert.equal(A.motorPack(C,comps),main);assert.equal(A.boardPack(C,C.boards[0],comps),main);
assert.equal(A.boardSupply(C,C.boards[0],A.boardCells(C,C.boards[0],comps)).kind,'boost5');
assert.deepEqual([...A.powerIssues(C,comps).errors],[]);
assert.deepEqual([...A.powerConsumers(C,comps).map(k=>k.key)],['board:1','board:2','motor:1','motor:2','motor:3','motor:4']);
// each board's draw: its kind's (the ESP32 and Pi Zero from the bench measurement), or its own
assert.equal(A.boardWatts(C,C.boards[0]),0.4);assert.equal(A.boardWatts(C,C.boards[1]),1.15);
C.wiring.boards[2]={watts:0.9};assert.equal(A.boardWatts(C,C.boards[1]),0.9);delete C.wiring.boards[2];

// an old design with a pack for the boards (what each feeds) still resolves: the motors on one, the boards on the 2S pack
comps=[...motors,main,small];C.wiring.parts={10:{feeds:'motors'},11:{feeds:'boards'}};
assert.equal(A.mainPack(C,comps),main);assert.equal(A.packSpec(C,main,comps).cells,1);assert.equal(A.packSpec(C,small,comps).cells,2);
for(const b of C.boards){assert.equal(A.boardPack(C,b,comps),small,b.name);assert.equal(A.boardSupplyOf(C,b,comps).kind,'reg5');}
let I=A.powerIssues(C,comps);assert.deepEqual([...I.errors],[],I.errors.join());   // 2S: a 5 V regulator
C.wiring.boards[2]={supply:'boost5'};I=A.powerIssues(C,comps);assert.ok(I.errors.some(e=>/Pi Zero: a boost converter only steps up: Board pack, 2S .* above 5 V/.test(e)),'a boost converter on the 2S pack: '+I.errors.join());delete C.wiring.boards[2];
C.wiring.boards[1]={supply:'board',supplyFrom:2};assert.equal(A.boardPack(C,C.boards[0],comps),small,'from another board\'s 5 V pin: that board\'s source');delete C.wiring.boards[1];
C.wiring.parts={10:{feeds:'motors'},11:{feeds:'boards',power:false}};assert.equal(A.boardPack(C,C.boards[0],comps),null,'unplugged: no source');
assert.ok(A.powerIssues(C,comps).errors.some(e=>/Flight controller: no power source/.test(e)));

// the power list: one battery per motor, the boards on a pack of their own
const packs=motors.map((m,i)=>({id:20+i,type:'mass',battery:true,name:'Pack '+(i+1),pack:{cells:1,capacity:0.5,maxA:12.5}}));
comps=[...motors,main,...packs];C.wiring.parts={};
C.wiring.power={'board:1':10,'board:2':10,...Object.fromEntries(motors.map((m,i)=>['motor:'+m.id,20+i]))};
motors.forEach((m,i)=>assert.equal(src(C,comps,'motor:'+m.id),20+i));
assert.equal(A.mainPack(C,comps),packs[0],'the main pack is the first motor\'s');assert.equal(A.packSpec(C,packs[0],comps).capacity,0.5,'it keeps its own specs (copied into the Battery settings)');assert.ok(A.packSpec(C,packs[0],comps).main);
assert.equal(A.packSpec(C,packs[1],comps).capacity,0.5);assert.equal(A.packSpec(C,main,comps).capacity,1.8,'a pack with no specs of its own: the Battery settings');
assert.equal(src(C,comps,'board:1'),10);assert.deepEqual([...A.powerIssues(C,comps).errors],[]);
let svg=A.powerDiagramSvg(C,comps);for(const p of packs)assert.ok(svg.includes('>'+p.name+'<'),'a box for '+p.name);
assert.equal((svg.match(/class="pd-lead pd-vbat"/g)||[]).length>=5,true,'a + lane per source');
C.wiring.power['motor:2']='';assert.ok(A.powerIssues(C,comps).errors.some(e=>/M1's ESC: no power source/.test(e)),'an ESC on nothing');
C.wiring.power['motor:2']=21;C.wiring.parts[21]={power:false};assert.equal(src(C,comps,'motor:2'),null,'its pack unplugged');delete C.wiring.parts[21];

// a USB port: the boards straight in at 5 V, off the drone; motors on it won't spin
comps=[...motors,main];C.wiring={parts:{},boards:{},sources:[{id:'s1',kind:'usb',name:'Laptop USB',volts:5,maxA:0.5}],power:{'board:1':'s1','board:2':'s1','motor:1':10,'motor:2':10,'motor:3':10,'motor:4':10}};
assert.deepEqual([...A.powerSources(C,comps).map(s=>s.id)],[10,'s1']);
for(const b of C.boards)assert.equal(A.boardSupplyOf(C,b,comps).kind,'direct5');
I=A.powerIssues(C,comps);assert.deepEqual([...I.errors],[],I.errors.join());assert.ok(I.warnings.some(w=>/Laptop USB isn't on the drone/.test(w)));
C.wiring.boards[2]={watts:3};assert.ok(A.powerIssues(C,comps).errors.some(e=>/Laptop USB: .*draw about 0\.68 A; it gives at most 0\.5 A/.test(e)),'more than a USB 2 port gives');delete C.wiring.boards[2];
C.wiring.boards[1]={supply:'boost5'};assert.ok(A.powerIssues(C,comps).errors.some(e=>/about 5 V\. Feed it straight in/.test(e)));delete C.wiring.boards[1];
C.wiring.power['motor:1']='s1';assert.ok(A.powerIssues(C,comps).warnings.some(w=>/won't spin the motors/.test(w)));C.wiring.power['motor:1']=10;
svg=A.powerDiagramSvg(C,comps);assert.ok(svg.includes('>Laptop USB<')&&svg.includes('off the drone')&&svg.includes('← Laptop USB +'));
// a part with a draw is a consumer; one named like a board warns that it may count twice
const pi={id:30,type:'mass',name:'Pi Zero',deviceW:0.8},cam={id:31,type:'mass',name:'Video board',deviceW:1};comps=[...motors,main,pi,cam];
assert.ok(A.powerConsumers(C,comps).some(k=>k.key==='part:31'));assert.ok(A.powerIssues(C,comps).warnings.some(w=>/Pi Zero has a draw of its own .* counts twice/.test(w)));
assert.ok(!A.powerIssues(C,comps).warnings.some(w=>/Video board has a draw/.test(w)));
// presets
assert.equal(A.POWER_PRESETS.lipo1s1800.maxA,36);assert.ok(Math.abs(A.lipoMass(1,1.8)-0.0408)<0.002,'a 1S 1800 mAh pack about 40 g');

// the wiring list: each source and what it powers
comps=[...motors,main,small];C.wiring={parts:{},boards:{},power:{'board:1':11,'board:2':11,'motor:1':10,'motor:2':10,'motor:3':10,'motor:4':10}};small.pack.cells=1;
const rows=A.hardwareOverview(C,comps).groups.find(g=>g.id==='power').rows;
assert.match(rows.find(r=>r.device==='Battery').connection,/4 ESCs/);assert.match(rows.find(r=>r.device==='Board pack').connection,/Flight controller's supply, Pi Zero's supply/);
assert.match(rows.find(r=>r.device==='Pi Zero supply').connection,/boost converter.*\(Board pack\)/);
svg=A.powerDiagramSvg(C,comps);
assert.ok(svg.includes('every source')&&svg.includes('5 V boost converter · Board pack'),'the converter on the boards\' pack, one common ground');
assert.equal((svg.match(/>5 V boost converter/g)||[]).length,1,'one converter for both boards');
// a second sense input reads the boards' pack: the battery2= line (only once set), its pin claimed, its divider checked
assert.equal(A.sensedPack2(C,comps),small);
let plan=A.hardwarePlan(C,motors,C.boards[0]);assert.ok(!A.hardwareSettings(plan).some(l=>l.startsWith('battery2')),'no battery2= until it is set (older firmware still installs)');
C.wiring.boards[1]={batteryPin:34,batteryDivider:2,batteryPin2:35,batteryDivider2:2};
plan=A.hardwarePlan(C,motors,C.boards[0]);assert.equal(plan.errors.length,0,plan.errors.join());assert.ok(A.hardwareSettings(plan).includes('battery2=35,2'));
assert.match(A.hardwareOverview(C,comps).groups.find(g=>g.id===1).rows.find(r=>/Board pack voltage/.test(r.device)).connection,/GPIO 35/);
assert.ok(A.powerDiagramSvg(C,comps).includes('Voltage divider · Board pack')&&A.powerDiagramSvg(C,comps).includes('ADC pin'),'its divider drawn on the boards\' pack');
C.wiring.boards[1].batteryPin2=34;assert.ok(A.hardwarePlan(C,motors,C.boards[0]).errors.some(e=>/Second pack ADC|more than once|Battery ADC/.test(e)),'one pin for both packs');
C.wiring.boards[1].batteryPin2=35;C.wiring.boards[1].batteryDivider2=1;assert.ok(A.powerIssues(C,comps).errors.some(e=>/Board pack voltage: .*4\.20 V, more than the chip takes/.test(e)),'1S straight on the pin: 4.2 V, over the chip\'s 3.3 V');
C.wiring.boards[1].batteryPin2=-1;assert.ok(A.hardwareSettings(A.hardwarePlan(C,motors,C.boards[0])).includes('battery2=-1'),'unset again: turned off on the board');
// a power monitor (INA3221): a channel per pack, the motors' pack (the main one) and the boards' (the second);
// its pmon= line only once set; its address, ranges and the pack's voltage checked; the wiring list
{ const C2={...C,wiring:{parts:{},boards:{},power:{'board:1':11,'board:2':11,'motor:1':10,'motor:2':10,'motor:3':10,'motor:4':10}}},cs=[...motors,main,small];
  const plan=()=>A.hardwarePlan(C2,cs,C2.boards[0]),line=()=>A.hardwareSettings(plan()).find(l=>l.startsWith('pmon'));
  assert.equal(line(),undefined,'no pmon= until set (older firmware still installs)');
  C2.wiring.boards[1]={pmonChip:'ina3221',pmonAddress:0x40,pmonCh:[{source:11,shunt:0.01},{source:10,shunt:0.005},{}]};
  const M=A.powerMonitor(C2,cs);assert.deepEqual([...M.channels.map(ch=>ch.use)],[2,1,0]);assert.ok(Math.abs(M.channels[1].maxA-32.75)<0.01);
  assert.equal(line(),'pmon=3221,64,2,0.01,1,0.005,0,0.1');assert.deepEqual([...plan().errors],[],plan().errors.join());
  assert.equal(A.pmonChannelOf(C2,cs,10).k,1);
  const rows=A.hardwareOverview(C2,cs).groups.find(g=>g.id===1).rows;
  assert.ok(rows.some(r=>/Power monitor \(INA3221\)/.test(r.device)&&/0x40 \(A0 → GND\)/.test(r.note)));assert.ok(rows.some(r=>/channel 2 · Battery/.test(r.device)&&/Battery \+ → IN\+; IN− → 4 consumers \+/.test(r.connection)&&/5 mΩ · reads up to 32\.8 A/.test(r.note)),JSON.stringify(rows.filter(r=>/channel/.test(r.device))));
  motors.forEach(m=>m.motorPhysics={currentLimit:10});C2.wiring.boards[1].pmonCh[1].shunt=0.1;
  assert.ok(A.pmonIssues(C2,cs).warnings.some(w=>/channel 2 \(Battery\): its 100 mΩ shunt reads up to 1\.6 A, but Battery can carry about 40\.\d A at full throttle/.test(w)),A.pmonIssues(C2,cs).warnings.join());
  C2.wiring.boards[1].pmonCh[1].shunt=0.001;assert.ok(!A.pmonIssues(C2,cs).warnings.length,A.pmonIssues(C2,cs).warnings.join());motors.forEach(m=>delete m.motorPhysics);
  C2.wiring.boards[1].pmonCh[2]={source:10,shunt:0.01};assert.ok(A.pmonIssues(C2,cs).errors.some(e=>/channel 3 and channel 2 both watch Battery/.test(e)));
  C2.wiring.boards[1].pmonCh[2]={source:99};assert.ok(A.pmonIssues(C2,cs).errors.some(e=>/channel 3: its source was removed/.test(e)));C2.wiring.boards[1].pmonCh[2]={};
  C2.wiring.boards[1].pmonAddress=0x45;assert.ok(plan().errors.some(e=>/answers at 0x40–0x43/.test(e)));C2.wiring.boards[1].pmonAddress=0x41;
  small.pack.cells=7;assert.ok(A.pmonIssues(C2,cs).errors.some(e=>/channel 1: Board pack reaches 29\.4 V; the INA3221 takes at most 26 V/.test(e)));small.pack.cells=1;
  C2.wiring.boards[1]={pmonChip:'',pmonCh:[]};assert.equal(line(),'pmon=-1','set before, now none: turned off on the board');
  C2.wiring.boards[1]={pmonChip:'ina219',pmonAddress:0x45,pmonCh:[{source:10,shunt:0.01}]};assert.equal(line(),'pmon=219,69,1,0.01');
}
// one pack: the drawing as before
const one=A.powerDiagramSvg({...C,wiring:{parts:{},boards:{}}},[...motors,main]);assert.ok(one.includes('+ red · − black')&&one.includes('battery −, every ESC'));
console.log('Power source checks passed: one pack as before, board draws, old designs\' packs, the power list (a battery per motor, the boards on their own), a USB port (straight in, off the drone, its limit, motors on it), parts with a draw, presets, the wiring list, the drawing, the second pack\'s sense input and the power monitor.');
