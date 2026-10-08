'use strict';
// Exercise registered agent tools against the actual design/planning/persistence functions.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
let saves=0,resets=0,renders=0,stored;
const ctx=vm.createContext({console,atob,window:{addEventListener(){}},LAWS:{},RN_TASK_FORMULAS:{},PRESETS:{},Map,Set,
  agent:{cfg:{askFormulas:false}},brt:{sig:'old'},undoKey:'',HW_UI:{drafts:new Map()},
  doReset(){resets++;},save(){saves++;stored=JSON.stringify(ctx.cfg);},renderComputers(){renders++;},syncFlightUi(){},
  structural(){ctx.save();},SENSOR_KINDS:{imu:'IMU',baro:'Baro',mag:'Compass'},
  sensorsOf:kind=>ctx.cfg.comps.filter(c=>c.type==='sensor'&&c.kind===kind),
  mkSensor:(kind,name,x,y,z)=>({id:Math.max(...ctx.cfg.comps.map(c=>c.id),0)+1,type:'sensor',kind,name,pos:[x,y,z]})});
for(const file of ['board-hardware','hardware','driver-presets','boards','part-geometry','designs','agent-tools','agent-hardware'])vm.runInContext(fs.readFileSync('js/'+file+'.js','utf8'),ctx);
vm.runInContext('this.api={AGENT_TOOLS,hardwareOwner,hardwarePart,fixComputers,DRIVER_PRESETS,readDesignFile};',ctx);
const A=ctx.api,T=A.AGENT_TOOLS,clone=x=>JSON.parse(JSON.stringify(x));
const base=()=>({boards:[{id:7,kind:'s3',name:'FC',tasks:['core','tlm']},{id:12,kind:'pizero',name:'Pi',tasks:['nav','learn','super','cargo']}],ground:{kind:'s3',name:'Pilot'},radio:1,
  wiring:{parts:{1:{board:7,pin:4},5:{board:7,driver:'mpu',address:104},8:{board:12,port:'/dev/ttyUSB0'}},boards:{7:{sda:15,scl:16},12:{linkPort:'/dev/serial0'}}}});
const parts=()=>[...Array.from({length:4},(_,i)=>({id:i+1,type:'motor',name:'M'+(i+1),pos:[0,0,0]})),...['imu','baro','mag','fix'].map((kind,i)=>({id:i+5,type:'sensor',kind,name:kind,pos:[0,0,0]}))];
function reset(){ctx.cfg={frame:{mass:.45},comps:parts(),computers:base()};ctx.HW_UI.drafts.clear();ctx.agent.cfg.askFormulas=false;}
function run(name,args={}){return T[name].run(args,{});}
function unchanged(fn,pattern){const before=JSON.stringify(ctx.cfg),n=saves;assert.throws(fn,pattern);assert.equal(JSON.stringify(ctx.cfg),before,'failed edit must be atomic');assert.equal(saves,n);}
async function main(){
  reset();
  assert.deepEqual(clone(run('get_computers').boards.map(b=>b.id)),[7,12]);
  assert.equal(run('get_computers').ground_board_kinds.mac,'Mac or PC');
  assert.equal(run('get_computers').boards[0].flight_loop_hz,1000);
  const wiring=clone(ctx.cfg.computers.wiring);
  run('set_computers',{boards:[{id:12,name:'Navigator',kind:'pi4',tasks:['nav','learn','super','cargo']},{id:7,name:'Controller',kind:'s3',tasks:['core','tlm']}]});
  assert.deepEqual(clone(ctx.cfg.computers.boards.map(b=>b.id)),[12,7]);assert.deepEqual(clone(ctx.cfg.computers.wiring),wiring);
  assert.equal(A.hardwareOwner(ctx.cfg.computers,ctx.cfg.comps[0]).id,7);assert.equal(A.hardwareOwner(ctx.cfg.computers,ctx.cfg.comps[7]).id,12);
  assert.equal(run('get_install_settings',{board:7}).ready,true);
  // Older callers that supply exact names keep IDs, including a chip change.
  run('set_computers',{boards:[{name:'Controller',kind:'esp32',tasks:['core','tlm']},{name:'Navigator',kind:'pizero2',tasks:['nav','learn','super','cargo']}]});
  assert.deepEqual(clone(ctx.cfg.computers.boards.map(b=>b.id)),[7,12]);
  unchanged(()=>run('set_computers',{boards:[{id:99,kind:'s3',tasks:['core']}]}),/Unknown board id/);
  unchanged(()=>run('set_computers',{boards:[{id:7,kind:'s3',tasks:['core','nav']},{id:12,kind:'pi4',tasks:['nav']}]}),/duplicate task/);
  unchanged(()=>run('set_computers',{boards:[{id:7,kind:'s3',tasks:['core','learn']}]}),/cannot run/);
  unchanged(()=>run('set_computers',{boards:[{id:7,kind:'s3',tasks:[]}]}),/Assign core/);
  run('set_computers',{boards:[{id:7,name:'C3',kind:'c3',tasks:['core']}]});
  assert.equal(run('get_computers').boards[0].flight_loop_hz,250);
  // No nextBoardId: deleting/replacing a board must never revive its saved connections.
  reset();
  let result=run('set_computers',{boards:[{id:7,name:'FC',kind:'s3',tasks:['core','tlm']},{name:'New Pi',kind:'pi4',tasks:['nav']}]});
  assert.deepEqual(clone(result.removed_board_ids),[12]);assert.ok(result.boards[1].id>12);
  assert.equal(A.hardwareOwner(ctx.cfg.computers,ctx.cfg.comps[7]),null);
  assert.ok(run('get_wiring_overview').unassigned.some(c=>c.device==='fix'));
  console.log('Agent board regression: IDs survive rename/reorder/chip edits; deleted IDs are never reused.');

  reset();
  assert.equal(run('get_hardware',{board:7}).parts[0].assignment,'explicit');
  assert.equal(run('get_hardware',{board:7}).parts[1].assignment,'automatic');
  unchanged(()=>run('set_hardware',{parts:[{id:1,fields:{pin:5}}]}),/Wiring not changed/);
  unchanged(()=>run('set_hardware',{parts:[{id:1,fields:{pin:5}},{id:2,fields:{pin:16}}]}),/Wiring not changed/);
  result=run('set_hardware',{parts:[{id:1,fields:{pin:5,driver:'brushed',maxDuty:65}},{id:'M2',fields:{pin:4}}],boards:[{id:7,fields:{brushedHz:12000,batteryPin:1,batteryDivider:10,crsfRx:2,crsfTx:21}}]});
  assert.equal(result.ready_to_install,true);assert.equal(ctx.cfg.computers.wiring.parts[1].pin,5);assert.equal(ctx.cfg.computers.wiring.parts[2].pin,4);
  const install=run('get_install_settings',{board:'FC'});
  for(const line of ['set motors=5,4,6,7','set motor_driver=1,0,0,0','set motor_max=65,100,100,100','set brushed_hz=12000','set battery=1,10','set crsf=2,21','save','reboot'])assert.ok(install.settings.includes(line),line);
  assert.match(run('get_wiring_overview',{board:7}).groups[0].rows.find(r=>r.device==='M1').connection,/GPIO 5.*MOSFET/);
  unchanged(()=>run('set_hardware',{parts:[{id:1,fields:{driver:'hbridge'}}],draft:true}),/unsupported driver/);
  unchanged(()=>run('set_hardware',{parts:[{id:1,fields:{maxDuty:0}}]}),/duty limit/);
  unchanged(()=>run('set_hardware',{parts:[{id:1,fields:{pin:'8'}}]}),/expected integer/);
  unchanged(()=>run('set_hardware',{boards:[{id:7,fields:{typo:1}}]}),/unknown field/);
  unchanged(()=>run('set_hardware',{boards:[{id:7,fields:{batteryPin:-2}}]}),/invalid GPIO/);
  unchanged(()=>run('set_hardware',{parts:[{id:1,fields:{board:'missing'}}],draft:true}),/Unknown or ambiguous board/);
  unchanged(()=>run('set_hardware',{parts:[{id:1,fields:{board:'ground'}}]}),/cannot connect/);
  unchanged(()=>run('set_hardware',{parts:[{id:1,fields:{board:12,pin:18}}]}),/distributed outputs/);
  result=run('set_hardware',{parts:[{id:1,fields:{board:'off'}}],draft:true});assert.equal(result.ready_to_install,false);
  assert.equal(run('get_install_settings',{board:7}).ready,false);assert.equal(run('get_install_settings',{board:7}).settings,undefined);
  // A pre-existing error does not prevent unrelated repairs; install still remains blocked.
  result=run('set_hardware',{boards:[{id:7,fields:{brushedHz:15000}}]});assert.equal(result.ready_to_install,false);
  unchanged(()=>run('set_hardware',{parts:[{id:1,fields:{board:'auto',driver:'pwm'}}]}),/overlap/);
  run('set_hardware',{parts:[{id:1,fields:{board:'auto',driver:'pwm'}},{id:2,fields:{pin:5}}]});
  assert.equal(ctx.cfg.computers.wiring.parts[1].board,undefined);assert.equal(ctx.cfg.computers.wiring.parts[1].pin,undefined);
  assert.equal(A.hardwarePart(ctx.cfg.computers,ctx.cfg.comps[0],ctx.cfg.comps).pin,4);
  console.log('Agent wiring checks: atomic edits/conflicts, mixed MOSFET settings and disconnected draft reporting.');

  reset();
  run('set_hardware',{ground:{arm:[14]}});assert.deepEqual(clone(ctx.cfg.computers.ground.wiring.arm),[14]);
  assert.deepEqual(JSON.parse(stored).computers.ground.wiring.arm,[14]);
  assert.match(run('get_install_settings',{board:'ground'}).settings,/set arm=14/);
  assert.equal(run('get_hardware',{board:'ground'}).errors.length,0);
  unchanged(()=>run('set_hardware',{ground:{arm:[6]}}),/overlap/);
  unchanged(()=>run('set_hardware',{ground:{arm:[14,15]}}),/expected 1/);
  unchanged(()=>run('set_hardware',{ground:{roll:['1']}}),/expected array/);
  unchanged(()=>run('set_hardware',{ground:{arm:[-2]},draft:true}),/GPIO must be/);
  run('set_computers',{boards:clone(ctx.cfg.computers.boards),ground:{name:'Desk',kind:'mac'}});
  assert.equal(run('get_install_settings',{board:'ground'}).ready,false);
  unchanged(()=>run('set_hardware',{ground:{arm:[14]}}),/requires an ESP32/);
  reset();ctx.cfg.comps.push({id:9,type:'latch',name:'Hook'});
  run('set_hardware',{parts:[{id:9,fields:{board:12,pin:27,driver:'gpio'}},{id:8,fields:{port:'/dev/serial0'}}],boards:[{id:12,fields:{linkPort:'/dev/ttyUSB2'}}]});
  const pi=run('get_install_settings',{board:12});assert.equal(pi.latch,'gpio27');assert.equal(pi.linkPort,'/dev/ttyUSB2');assert.equal(pi.gps[0].port,'/dev/serial0');
  unchanged(()=>run('set_hardware',{boards:[{id:12,fields:{linkPort:'/dev/serial0'}}]}),/already used/);
  console.log('Agent auxiliary checks: persisted ground GPIOs, Pi latch/serial settings and duplicate-port rejection.');

  reset();ctx.cfg.comps=ctx.cfg.comps.filter(c=>c.type!=='sensor');
  run('apply_hardware_preset',{board:7,preset:'10dof'});
  const sensors=ctx.cfg.comps.filter(c=>c.type==='sensor');assert.equal(sensors.length,3);
  assert.deepEqual(sensors.map(c=>ctx.cfg.computers.wiring.parts[c.id].driver),['mpu','bmp180','hmc']);
  assert.deepEqual(sensors.map(c=>ctx.cfg.computers.wiring.parts[c.id].address),[104,119,30]);
  const before=JSON.stringify(ctx.cfg);assert.throws(()=>run('apply_hardware_preset',{board:12,preset:'10dof'}),/flight-core ESP/);assert.equal(JSON.stringify(ctx.cfg),before);
  reset();
  await run('set_driver_code',{board:7,preset:'10dof'});assert.equal(ctx.cfg.computers.wiring.boards[7].driverCode,A.DRIVER_PRESETS['10dof'].code);
  let code='',offset=0,chunk;
  do {chunk=run('get_driver_code',{board:7,offset,length:2000});assert.ok(JSON.stringify(chunk).length<6000);code+=chunk.code;offset=chunk.next_offset;}while(offset!==null);
  assert.equal(code,A.DRIVER_PRESETS['10dof'].code);
  unchanged(()=>run('get_driver_code',{board:7,offset:-1}),/Invalid source/);
  ctx.HW_UI.drafts.set(7,'unsaved work');const source=ctx.cfg.computers.wiring.boards[7].driverCode;
  await assert.rejects(run('set_driver_code',{board:7,code:'// replacement'}),/unsaved C editor draft/);assert.equal(ctx.cfg.computers.wiring.boards[7].driverCode,source);
  ctx.HW_UI.drafts.clear();ctx.agent.cfg.askFormulas=true;ctx.agentRenderFeed=()=>{};
  const item={},pending=T.set_driver_code.run({board:7,code:'// approved'},item);assert.ok(item.confirm);item.confirm.resolve(false);
  assert.equal((await pending).error,'the person declined this change');assert.equal(ctx.cfg.computers.wiring.boards[7].driverCode,source);
  const approved={},saving=T.set_driver_code.run({board:7,code:'// approved'},approved);approved.confirm.resolve(true);assert.equal((await saving).compiled,false);
  assert.equal(JSON.parse(stored).computers.wiring.boards[7].driverCode,'// approved');
  const race={},racing=T.set_driver_code.run({board:7,code:'// racing'},race);ctx.HW_UI.drafts.set(7,'new draft');race.confirm.resolve(true);await assert.rejects(racing,/unsaved C editor draft/);
  ctx.HW_UI.drafts.clear();ctx.agent.cfg.askFormulas=false;
  await assert.rejects(run('set_driver_code',{board:7,code:'x'.repeat(65537)}),/64 KB/);
  await assert.rejects(run('set_driver_code',{board:7,code:'x',preset:'10dof'}),/exactly one/);
  await assert.rejects(run('set_driver_code',{board:12,preset:'10dof'}),/flight-core ESP/);
  await run('set_driver_code',{board:7,code:'// control characters\n'+'\u0001'.repeat(3000)});
  assert.ok(JSON.stringify(run('get_driver_code',{board:7,length:2000})).length<6000);
  run('set_hardware',{parts:[{id:5,fields:{driver:'custom',address:104}},{id:6,fields:{driver:'bmp180'}},{id:7,fields:{driver:'hmc',bias:'1,2,3',scale:'1,1,1'}}]});
  assert.ok(run('get_install_settings',{board:7}).settings.includes('set imu=3,104'));assert.ok(run('get_hardware',{board:7}).warnings.some(w=>w.includes('rebuild firmware')));
  const design=A.readDesignFile(JSON.stringify({frame:.45,comps:ctx.cfg.comps,computers:ctx.cfg.computers})).design;
  assert.equal(design.computers.wiring.boards[7].driverCode,ctx.cfg.computers.wiring.boards[7].driverCode);
  assert.equal(design.computers.wiring.parts[5].driver,'custom');
  console.log('Agent driver checks: 10DOF additions, source chunking/persistence, confirmation and draft protection.');

  const html=fs.readFileSync('index.html','utf8');assert.ok(html.indexOf('js/agent-tools.js')<html.indexOf('js/agent-hardware.js'));assert.ok(html.indexOf('js/agent-hardware.js')<html.indexOf('js/agent-ui.js'));
  const prompt=fs.readFileSync('js/agent.js','utf8');for(const name of ['get_hardware','set_hardware','get_install_settings','set_driver_code','get_wiring_overview'])assert.ok(prompt.includes(name));
  assert.ok(saves>0&&resets>0&&renders>0);assert.equal(vm.runInContext('brt.sig',ctx),null);assert.match(vm.runInContext('undoKey',ctx),/wiring|computers/);
  console.log('Agent hardware support checks passed. No physical-board APIs are called by these tools.');
}
main().catch(e=>{console.error(e);process.exitCode=1;});
