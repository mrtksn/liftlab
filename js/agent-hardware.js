'use strict';
// Design-only hardware tools. These never access serial, flash firmware or execute C.
const AGENT_BOARD_REF={type:['integer','string'],description:'Board id from get_computers; "ground" for the command module. Unique board names also work.'};
const AGENT_PART_FIELDS={board:{type:['integer','string','null'],description:'Existing board id, "auto" to follow its task, or null / "off" to disconnect.'},pin:{type:'integer'},driver:{type:'string'},address:{type:'integer'},maxDuty:{type:'integer'},center:{type:'number'},usPerRad:{type:'number'},bias:{type:'string'},scale:{type:'string'},port:{type:'string'}};
const AGENT_BUS_FIELDS=Object.fromEntries(['sda','scl','brushedHz','escHz','escMin','escMax','batteryPin','crsfRx','crsfTx'].map(k=>[k,{type:'integer'}]));
Object.assign(AGENT_BUS_FIELDS,{batteryDivider:{type:'number'},linkPort:{type:'string'},receiverPort:{type:'string'}});
const AGENT_GROUND_FIELDS=Object.fromEntries(['tx','arm','fly','roll','pitch','throttle','yaw','buzzer','led'].map(k=>[k,{type:'array',items:{type:'integer'}}]));
const agentClone=x=>JSON.parse(JSON.stringify(x));
function agentFields(value,fields,label){
  if(!value||typeof value!=='object'||Array.isArray(value))throw new Error(label+': expected an object');
  for(const [k,v] of Object.entries(value)){
    if(!Object.hasOwn(fields,k))throw new Error(label+': unknown field '+k);
    const t=fields[k].type,types=Array.isArray(t)?t:[t];
    const valid=types.some(type=>type==='null'?v===null:type==='array'?Array.isArray(v)&&(!fields[k].items||v.every(Number.isInteger)):type==='object'?v!==null&&typeof v==='object'&&!Array.isArray(v):type==='integer'?Number.isInteger(v):type==='number'?typeof v==='number'&&Number.isFinite(v):typeof v===type);
    if(!valid)throw new Error(label+'.'+k+': expected '+types.join(' / '));
  }
}
function agentHardwareBoard(ref,C=computers()){
  if(ref==='ground')return {...C.ground,id:'ground',tasks:[]};
  const matches=C.boards.filter(b=>b.id===ref||b.name===ref);
  if(matches.length!==1)throw new Error('Unknown or ambiguous board '+JSON.stringify(ref)+'; use its id from get_computers');
  return matches[0];
}
function agentHardwareIssues(C=computers()){
  const errors=new Set(),warnings=new Set();
  for(const b of C.boards){const p=hardwarePlan(C,cfg.comps,b);p.errors.forEach(e=>errors.add(e));p.warnings.forEach(e=>warnings.add(e));}
  groundHardwareErrors(C).forEach(e=>errors.add(e));
  return {errors:[...errors],warnings:[...warnings]};
}
function agentSetComputers(a){
  agentFields(a,{boards:{type:'array'},ground:{type:'object'}},'set_computers');
  const C=agentClone(computers()),used=new Set(),tasks=new Set();
  if(!Array.isArray(a.boards)||a.boards.length<1||a.boards.length>BOARD_MAX)throw new Error('Provide 1–'+BOARD_MAX+' drone boards');
  const boards=a.boards.map(input=>{
    agentFields(input,{id:{type:'integer'},name:{type:'string'},kind:{type:'string'},tasks:{type:'array'}},'board');
    if(!Object.hasOwn(BOARD_KINDS,input.kind)||BOARD_KINDS[input.kind].groundOnly)throw new Error('Unsupported drone board kind '+input.kind);
    if(!Array.isArray(input.tasks)||input.tasks.some(t=>typeof t!=='string'))throw new Error('Each board needs a tasks array');
    for(const t of input.tasks){if(!Object.hasOwn(TASKS,t)||tasks.has(t))throw new Error('Unknown or duplicate task '+t);if(TASKS[t].mcuOnly&&!BOARD_KINDS[input.kind].mcu||TASKS[t].piOnly&&BOARD_KINDS[input.kind].mcu)throw new Error('Task '+t+' cannot run on '+input.kind);tasks.add(t);}
    let existing;
    if(Object.hasOwn(input,'id')){existing=C.boards.find(b=>b.id===input.id);if(!existing)throw new Error('Unknown board id '+input.id+'; omit id to add a new board');}
    else if(input.name){const matches=C.boards.filter(b=>b.name===input.name);if(matches.length>1)throw new Error('Ambiguous board name; supply its id');existing=matches[0];}
    if(existing&&used.has(existing.id))throw new Error('Duplicate board id '+existing.id);
    if(existing)used.add(existing.id);
    return {...(existing||{}),...input,...(existing?{id:existing.id}:{}),tasks:input.tasks.slice()};
  });
  if(!tasks.has('core'))throw new Error('Assign core to exactly one microcontroller');
  if(a.ground){agentFields(a.ground,{kind:{type:'string'},name:{type:'string'}},'ground');if(Object.hasOwn(a.ground,'kind')&&!Object.hasOwn(BOARD_KINDS,a.ground.kind))throw new Error('Unknown ground board kind');C.ground={...C.ground,...a.ground};}
  const removed=C.boards.filter(b=>!used.has(b.id)).map(b=>b.id);
  // Older/default designs may not have nextBoardId yet. Never recycle a removed board's ID.
  C.nextBoardId=Math.max(C.nextBoardId||1,...C.boards.map(b=>b.id+1));
  C.boards=boards;setComputers(C,'agent');
  return {boards:agentClone(computers().boards),ground:agentClone(computers().ground),removed_board_ids:removed,...agentHardwareIssues(),note:'Removed boards leave explicit wiring disconnected; automatic wiring follows its task.'};
}
function agentGetHardware(a={}){
  const C=computers();
  if(a.board==null)return {boards:C.boards.map(b=>({id:b.id,name:b.name,kind:b.kind,tasks:b.tasks})),ground:{id:'ground',name:C.ground.name,kind:C.ground.kind},devices:cfg.comps.filter(c=>['motor','joint','sensor','latch'].includes(c.type)).map(c=>({id:c.id,name:c.name,type:c.type,kind:c.kind,board:hardwareOwner(C,c)?.id??null})),drivers:Object.fromEntries(Object.entries(DEVICE_PROFILES).map(([k,p])=>[k,Object.fromEntries(Object.entries(p).map(([id,d])=>[id,d.label]))])),motor_drivers:['pwm','brushed'],c_presets:Object.fromEntries(Object.entries(DRIVER_PRESETS).map(([k,p])=>[k,p.label])),...agentHardwareIssues(),note:'Query a board id for pins and settings. Custom C needs an ESP-IDF rebuild; hardware tools do not flash boards.'};
  const b=agentHardwareBoard(a.board,C),profile=ESP_PROFILES[b.kind];
  if(b.id==='ground')return {board:b,wiring:groundHardware(C),pins:profile?{signal:hardwareInputPins(b.kind),adc:profile.adc}:null,errors:groundHardwareErrors(C),settings:groundHardware(C)?groundHardwareSettings(C):null};
  const bus=hardwareBus(C,b),{driverCode,...settings}=bus,plan=hardwarePlan(C,cfg.comps,b);
  return {board:b,settings,custom_code_saved:typeof driverCode==='string',custom_code_length:driverCode?.length||0,pins:profile?{output:profile.pins,input:hardwareInputPins(b.kind),adc:profile.adc,uart0:profile.link,max_outputs:profile.outputs}:{gpio:PI_GPIO_PINS},claims:hardwarePinClaims(C,cfg.comps,b),parts:cfg.comps.filter(c=>['motor','joint','sensor','latch'].includes(c.type)&&hardwareOwner(C,c)?.id===b.id).map(c=>({id:c.id,name:c.name,type:c.type,kind:c.kind,assignment:Object.hasOwn(C.wiring?.parts?.[c.id]||{},'board')?'explicit':'automatic',...hardwarePart(C,c,cfg.comps)})),errors:plan.errors,warnings:plan.warnings};
}
function agentSetHardware(a){
  agentFields(a,{parts:{type:'array'},boards:{type:'array'},ground:{type:'object'},draft:{type:'boolean'}},'set_hardware');
  const C=agentClone(computers()),before=agentHardwareIssues(C);
  C.wiring||={};C.wiring.parts||={};C.wiring.boards||={};
  const seen=new Set();
  for(const change of a.parts||[]){
    agentFields(change,{id:{type:['integer','string']},fields:{type:'object'}},'part change');
    const c=partOf(change.id);if(seen.has(c.id))throw new Error('Duplicate part change '+c.id);seen.add(c.id);
    const keys=c.type==='motor'?['board','pin','driver','maxDuty']:c.type==='joint'?['board','pin','center','usPerRad']:c.type==='latch'?['board','pin','driver']:c.type==='sensor'?['board','driver',...(['imu','baro','mag'].includes(c.kind)?['address']:[]),...(c.kind==='mag'?['bias','scale']:[]),...(c.kind==='fix'?['port']:[])]:[];
    agentFields(change.fields,Object.fromEntries(keys.map(k=>[k,AGENT_PART_FIELDS[k]])),c.name);
    const patch=agentClone(change.fields),saved=C.wiring.parts[c.id]||{};
    if(Object.hasOwn(patch,'board')){
      if(patch.board==='auto'){delete saved.board;if(!Object.hasOwn(patch,'pin'))delete saved.pin;delete patch.board;}
      else {patch.board=patch.board==='off'?null:patch.board;if(patch.board!==null){const owner=agentHardwareBoard(patch.board,C);if(owner.id==='ground')throw new Error('Drone parts cannot connect to the command module');patch.board=owner.id;}if(c.type!=='sensor'&&patch.board!==saved.board&&!Object.hasOwn(patch,'pin'))patch.pin=-1;}
    }
    if(Object.hasOwn(patch,'driver')){
      const allowed=c.type==='motor'?['pwm','brushed']:c.type==='joint'?['pwm']:c.type==='latch'?['pwm','gpio']:Object.keys(DEVICE_PROFILES[c.kind]||{});
      if(!allowed.includes(patch.driver))throw new Error(c.name+': unsupported driver '+patch.driver);
      if(c.type==='sensor'&&['imu','baro','mag'].includes(c.kind)&&!Object.hasOwn(patch,'address'))patch.address=patch.driver==='custom'?(saved.address||{imu:0x68,baro:0x77,mag:0x1e}[c.kind]):DEVICE_PROFILES[c.kind][patch.driver].addresses[0]||0;
    }
    if(Object.hasOwn(patch,'pin')&&patch.pin< -1)throw new Error('GPIO must be -1 or nonnegative');
    C.wiring.parts[c.id]={...saved,...patch};
  }
  seen.clear();
  for(const change of a.boards||[]){
    agentFields(change,{id:{type:['integer','string']},fields:{type:'object'}},'board change');const b=agentHardwareBoard(change.id,C);
    if(b.id==='ground'||seen.has(b.id))throw new Error('Use ground for command-module wiring; board changes must be unique');seen.add(b.id);
    const fields=ESP_PROFILES[b.kind]?Object.fromEntries(Object.entries(AGENT_BUS_FIELDS).filter(([k])=>!['linkPort','receiverPort'].includes(k))):Object.fromEntries(['sda','scl','linkPort','receiverPort'].map(k=>[k,AGENT_BUS_FIELDS[k]]));
    agentFields(change.fields,fields,b.name);
    for(const [k,v] of Object.entries(change.fields))if(['sda','scl','batteryPin','crsfRx','crsfTx'].includes(k)&&v<(['sda','scl'].includes(k)?0:-1))throw new Error(b.name+'.'+k+': invalid GPIO');
    C.wiring.boards[b.id]={...C.wiring.boards[b.id],...change.fields};
  }
  if(a.ground){
    const current=groundHardware(C);if(!current)throw new Error('GPIO command-module wiring requires an ESP32 board');
    agentFields(a.ground,AGENT_GROUND_FIELDS,'ground');for(const [k,v] of Object.entries(a.ground)){if(v.length!==current[k].length)throw new Error('ground.'+k+': expected '+current[k].length+' GPIO values');if(v.some(pin=>pin< -1))throw new Error('ground.'+k+': GPIO must be -1 or nonnegative');}
    C.ground.wiring={...C.ground.wiring,...a.ground};
  }
  if(!seen.size&&!(a.parts||[]).length&&!a.ground)throw new Error('Provide at least one wiring change');
  const after=agentHardwareIssues(C),newErrors=after.errors.filter(e=>!before.errors.includes(e));
  if(newErrors.length&&!a.draft)throw new Error('Wiring not changed: '+newErrors.join('; ')+'. Batch related pin changes, or use draft=true for deliberately incomplete wiring.');
  setComputers(C,'agent-wiring');
  return {saved:true,draft:!!a.draft,ready_to_install:!after.errors.length,...after,note:'Saved design only; nothing sent to a physical board.'};
}
Object.assign(AGENT_TOOLS,{
  get_hardware:{desc:'Read hardware profiles, board/device ids and wiring checks. Without board returns an index; with a board id or "ground" returns its GPIO choices, current assignments, shared settings and errors. Custom C source is read separately with get_driver_code.',params:obj({board:AGENT_BOARD_REF}),run:a=>agentGetHardware(a)},
  get_wiring_overview:{desc:'Read the wiring connection list grouped by board, including motors, sensors, power/radio, ground buttons/sticks, and both ends of board links. Optional board filters the list.',params:obj({board:AGENT_BOARD_REF}),run:a=>{
    const C=computers(),overview=hardwareOverview(C,cfg.comps);if(a.board!=null){const b=agentHardwareBoard(a.board,C);overview.groups=overview.groups.filter(g=>g.id===b.id);overview.unassigned=[];}return {...overview,...agentHardwareIssues(C)};
  }},
  set_hardware:{desc:'Patch saved design wiring atomically; restarts simulation, supports undo. Parts: id plus fields (board id / auto / off, pin=-1 for disconnected, motor driver pwm/brushed and maxDuty 1–100; sensor driver/address, compass bias/scale strings, GPS port; servo center/usPerRad; latch pwm/gpio). Boards: id plus shared settings (SDA/SCL, brushedHz 1000–30000, escHz/min/max, batteryPin/divider, crsfRx/Tx; Pi linkPort/receiverPort). Ground fields are GPIO arrays matching get_hardware. Unknown fields/types/drivers rejected. New wiring errors reject the whole edit; draft=true saves incomplete/unsupported wiring and reports installation blockers. Batch swaps and related bus changes together.',params:obj({parts:{type:'array',items:obj({id:{type:['integer','string']},fields:obj(AGENT_PART_FIELDS)},['id','fields'])},boards:{type:'array',items:obj({id:AGENT_BOARD_REF,fields:obj(AGENT_BUS_FIELDS)},['id','fields'])},ground:obj(AGENT_GROUND_FIELDS),draft:{type:'boolean'} }),run:a=>agentSetHardware(a)},
  apply_hardware_preset:{desc:'Configure a 10DOF module on the flight-core ESP: MPU6050 at 0x68, BMP180 at 0x77, HMC5883L at 0x1e. Adds missing sensors; replaces their wiring/mounting with frame-origin defaults, as the UI preset button does. Restarts simulation; undo restores previous design. This uses built-in drivers; set_driver_code handles C source presets separately.',params:obj({board:AGENT_BOARD_REF,preset:{type:'string',enum:['10dof']}},['board','preset']),run:a=>{
    const b=agentHardwareBoard(a.board);if(a.preset!=='10dof'||b.id==='ground'||!b.tasks.includes('core')||!ESP_PROFILES[b.kind])throw new Error('10DOF preset requires the flight-core ESP board');use10Dof(b);return {configured:a.preset,board:b.id,sensors:cfg.comps.filter(c=>c.type==='sensor'&&['imu','baro','mag'].includes(c.kind)).map(c=>({id:c.id,name:c.name,...partWiring(c)})),...agentHardwareIssues()};
  }},
  get_driver_code:{desc:'Read saved custom sensor C source, or a preset, in chunks. Source is not compiled/executed here. Default source matches the C editor. offset is a character index; length defaults to 1600 (max 2000). Continue from next_offset until null.',params:obj({board:AGENT_BOARD_REF,preset:{type:'string'},offset:{type:'integer'},length:{type:'integer'}},['board']),run:a=>{
    const b=agentHardwareBoard(a.board);if(b.id==='ground'||!b.tasks.includes('core')||!ESP_PROFILES[b.kind])throw new Error('C sensor drivers require the flight-core ESP board');
    if(a.preset!=null&&!Object.hasOwn(DRIVER_PRESETS,a.preset))throw new Error('Unknown C preset');
    const saved=computers().wiring?.boards?.[b.id]?.driverCode,source=a.preset!=null?DRIVER_PRESETS[a.preset].code:saved??DRIVER_PRESETS['10dof'].code;
    const offset=a.offset??0,length=a.length??1600;if(!Number.isInteger(offset)||offset<0||offset>source.length||!Number.isInteger(length)||length<1||length>2000)throw new Error('Invalid source offset/length');
    let end=Math.min(offset+length,source.length);
    // Leave space for metadata inside the agent's 6000-character JSON result limit.
    while(JSON.stringify(source.slice(offset,end)).length>5000)end=offset+Math.max(1,Math.floor((end-offset)/2));
    return {board:b.id,source:a.preset??(saved!=null?'saved':'editor default'),code:source.slice(offset,end),total_length:source.length,next_offset:end<source.length?end:null,compiled:false};
  }},
  set_driver_code:{desc:'Save a full custom sensor C header or named C preset in the design (exactly one of code/preset). Source limit 64 KB; no browser compilation, execution or automatic flashing. Requires the flight-core ESP and no unsaved editor draft. Uses the formula-edit confirmation preference. Select driver="custom" on relevant sensors, export custom_sensors.h, rebuild ESP-IDF firmware and install the custom binaries.',params:obj({board:AGENT_BOARD_REF,code:{type:'string'},preset:{type:'string'}},['board']),run:async(a,item)=>{
    const b=agentHardwareBoard(a.board);if(b.id==='ground'||!b.tasks.includes('core')||!ESP_PROFILES[b.kind])throw new Error('C sensor drivers require the flight-core ESP board');
    if((a.code!=null)===(a.preset!=null))throw new Error('Provide exactly one of code or preset');if(a.preset!=null&&!Object.hasOwn(DRIVER_PRESETS,a.preset))throw new Error('Unknown C preset');
    const source=a.preset!=null?DRIVER_PRESETS[a.preset].code:a.code;if(typeof source!=='string'||!source.trim()||source.length>65536)throw new Error('Provide nonempty C source up to 64 KB');
    const checkDraft=()=>{if(typeof HW_UI!=='undefined'&&HW_UI.drafts.has(b.id))throw new Error('Save or discard the unsaved C editor draft before changing source through the agent');};checkDraft();
    if(agent.cfg.askFormulas&&!(await agentConfirm(item,'Save custom sensor C source for '+b.name+'?')))return {error:'the person declined this change'};
    checkDraft();const live=agentHardwareBoard(b.id);if(live.kind!==b.kind||!live.tasks.includes('core'))throw new Error('Board changed while awaiting confirmation; read hardware again');
    editWiring(w=>{w.boards[b.id]={...w.boards[b.id],driverCode:source};},'agent-code-'+b.id);
    return {saved:true,board:b.id,length:source.length,compiled:false,note:'Saved source only. Export custom_sensors.h, rebuild ESP-IDF, install custom firmware and send design settings. Sensors must select the custom driver.'};
  }},
  get_install_settings:{desc:'Read settings derived from saved wiring, as Install does. Never sends them or flashes a board. Returns no ready settings when that board has wiring errors. ESP flight core: set/save/reboot lines; ground ESP: command-module settings; Pi: serial/GPS/receiver ports and latch argument.',params:obj({board:AGENT_BOARD_REF},['board']),run:a=>{
    const C=computers(),b=agentHardwareBoard(a.board,C),plan=b.id==='ground'?{errors:groundHardwareErrors(C),warnings:[]}:hardwarePlan(C,cfg.comps,b);
    if(plan.errors.length)return {board:b.id,ready:false,errors:plan.errors,warnings:plan.warnings};
    if(b.id==='ground')return {board:b.id,ready:!!groundHardware(C),settings:groundHardware(C)?groundHardwareSettings(C):null,note:'Use Install for host/gamepad setup on a Pi or Mac.'};
    if(ESP_PROFILES[b.kind]){if(!b.tasks.includes('core'))return {board:b.id,ready:false,note:'This MCU role has no supported physical wiring installer'};return {board:b.id,ready:true,settings:[...hardwareSettings(plan).map(l=>'set '+l),'save','reboot'],warnings:plan.warnings,note:'Configuration only. Custom C requires matching rebuilt firmware. Disconnect motor power until settings are saved and the board restarts; factory/reset defaults use ESC pulses.'};}
    const bus=hardwareBus(C,b);return {board:b.id,ready:true,linkPort:bus.linkPort||'/dev/serial0',gps:cfg.comps.filter(c=>c.type==='sensor'&&c.kind==='fix'&&hardwareOwner(C,c)?.id===b.id).map(c=>({id:c.id,port:hardwarePart(C,c,cfg.comps).port||'/dev/ttyUSB0'})),receiverPort:b.tasks.includes('tlm')?bus.receiverPort||'/dev/ttyUSB1':null,latch:b.tasks.includes('cargo')?piLatchSettings(C,cfg.comps,b):null,warnings:plan.warnings,note:'Use Install for build/start commands and exported design files; settings not sent.'};
  }},
});
