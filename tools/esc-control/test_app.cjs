const {test}=require('node:test');
const assert=require('node:assert/strict');
const M=require('./model.js');
const Transport=require('./serial.js');

test('built-in presets validate and use distinct usable GPIOs',()=>{for(const p of M.defaults())assert.equal(M.validatePreset(p),p);});
test('reserved pins, duplicate pins, input-only outputs and overlong PWM are rejected',()=>{
 for(const pin of [0,1,2,3,5,6,12,15,34,35,36,39]){const p=M.defaults()[0];p.channels[0].pin=pin;assert.throws(()=>M.validatePreset(p));}
 let p=M.defaults()[0];p.channels[1].pin=18;assert.throws(()=>M.validatePreset(p),/once/);
 p=M.defaults()[0];p.controls.find(c=>c.mode==='continuous').max=3000;assert.throws(()=>M.validatePreset(p),/period/);
 p=M.defaults()[0];p.controls.find(c=>c.mode==='continuous').channel=1;assert.throws(()=>M.validatePreset(p),/output/);
});
test('preset copies can be edited without changing originals; all button modes validate',()=>{
 const p=M.defaults()[0],copy=M.clone(p);copy.name='Edited';copy.controls[0].value=1500;assert.equal(p.controls[0].value,1070);
 for(const mode of Object.keys(M.MODES)){copy.controls[0].mode=mode;M.validatePreset(copy);}
 const s=M.resolveSignal(p.controls.find(c=>c.mode==='continuous'),1800);assert.equal(s.value,1800);assert.equal(p.controls[0].on.value,1500);
});
test('signal validation rejects malformed payloads and impossible values',()=>{
 for(const s of [{kind:'uart',baud:115200,data:'XZ'},{kind:'pulse',pulses:[[2,10]]},{kind:'digital',value:2},{kind:'dshot',rate:600,value:2048},{kind:'pwm',hz:0,value:550},{kind:'pwm',hz:'490',value:550},{kind:'digital',value:'1'}])assert.throws(()=>M.validateSignal(s));
});
test('demo captures contain only explicitly simulated data and correct PWM timing',()=>{
 const p=M.defaults()[1],c=M.demoCapture(p,60);assert.equal(c.demo,true);assert.ok(c.edges.length>30);const a=M.analyze(c,1);assert.ok(Math.abs(a.hz-490)<.1);assert.ok(Math.abs(a.high-1500)<.1);
});
function fixture(initial,changes,duration){let state=initial;const edges=[];for(const [t,ch,value]of changes.sort((a,b)=>a[0]-b[0])){state=value?state|(1<<ch):state&~(1<<ch);if(edges.length&&edges.at(-1)[0]===t)edges.at(-1)[1]=state;else edges.push([t,state]);}return{initial,edges,duration};}
test('UART 8N1 decodes bytes and exposes framing errors',()=>{
 const bit=1e6/9600,changes=[];let start=100;
 for(const byte of [0x55,0xA3]){changes.push([start,0,0]);for(let k=0;k<8;k++)changes.push([start+(k+1)*bit,0,(byte>>k)&1]);changes.push([start+9*bit,0,1]);start+=12*bit;}
 const cap=fixture(1,changes,start+bit),rows=M.uartDecode(cap,0,9600);assert.deepEqual(rows.map(r=>r.data),['0x55','0xA3']);assert.ok(rows.every(r=>r.type==='UART byte'));
 const bad=fixture(1,[[100,0,0]],100+12*bit);assert.equal(M.uartDecode(bad,0,9600)[0].type,'Framing error');
});
test('I2C distinguishes start, address, ACK, data and stop',()=>{
 const changes=[[10,1,0]];let t=20;const addByte=value=>{for(let k=7;k>=-1;k--){changes.push([t,0,0],[t+2,1,k<0?0:(value>>k)&1],[t+5,0,1]);t+=10;}};
 addByte(0xA0);addByte(0x42);changes.push([t,0,0],[t+1,1,0],[t+5,0,1],[t+10,1,1]);
 const rows=M.i2cDecode(fixture(3,changes,t+20),0,1);assert.equal(rows[0].type,'START');assert.equal(rows[1].type,'Address');assert.equal(rows[1].data,'0x50');assert.equal(rows[1].detail,'Write · ACK');assert.equal(rows[2].data,'0x42');assert.equal(rows.at(-1).type,'STOP');
});
test('SPI samples the selected mode and ignores inactive CS',()=>{
 const changes=[];for(let k=7;k>=0;k--){let t=10+(7-k)*10;changes.push([t,1,(0xA5>>k)&1],[t+2,0,1],[t+5,0,0]);}
 assert.equal(M.spiDecode(fixture(0,changes,100),0,1,-1,0)[0].data,'0xA5');assert.equal(M.spiDecode(fixture(4,changes,100),0,1,2,0).length,0);
});
test('transport Stop invalidates queued output and enable commands',async()=>{
 const t=new Transport(()=>{},()=>{}),sent=[];t.active=true;t.writer={write:async bytes=>sent.push(JSON.parse(new TextDecoder().decode(bytes)))};
 const run=t.command('signal',{ch:0,signal:{kind:'digital',value:1}}).catch(e=>e.message),arm=t.command('enable').catch(e=>e.message);
 const stop=t.stop();await new Promise(r=>setImmediate(r));assert.equal(await run,'Cancelled');assert.equal(await arm,'Cancelled');assert.deepEqual(sent.map(m=>m.cmd),['stop']);
 const pending=t.pending.get(sent[0].id);clearTimeout(pending.timer);t.pending.delete(sent[0].id);pending.resolve({type:'ack'});await stop;
});
test('serial loss rejects waiting requests and prevents subsequent writes',async()=>{
 let lost=0;const t=new Transport(()=>{},()=>lost++);t.active=true;t.writer={write:async()=>{throw Error('cable removed');}};
 await assert.rejects(t.command('enable'),/cable removed/);assert.equal(t.active,false);assert.equal(t.pending.size,0);assert.equal(lost,1);await assert.rejects(t.write({cmd:'ping'}),/disconnected/);
});

const B=require('./bytes.js'),Q=require('./sequence.js');
test('hex and escaped text preserve raw bytes, Unicode and UTF-8 BOMs',()=>{
 assert.equal(B.encode('0x31, 0x32 33','hex'),'313233');
 assert.equal(B.encode('123\\n','text'),'3132330A');
 for(const hex of ['55AA000A0D095C','C3A9F09F9880','EFBBBF3132'])assert.equal(B.encode(B.decode(hex,'text'),'text'),hex);
 const all=Array.from({length:256},(_,i)=>i),hex=B.toHex(all);assert.equal(B.encode(B.decode(hex,'text'),'text','little',256),hex);
 assert.equal(B.encode('😀😀😀','text').length,24);assert.throws(()=>B.encode('😀😀😀😀','text'),/12 bytes/);
 assert.throws(()=>B.encode('0','hex'),/complete hex/);
});
test('typed numbers round-trip with signedness, float encoding and byte order',()=>{
 assert.equal(B.encode('-1234, 42','int16','little'),'2EFB2A00');
 assert.equal(B.encode('-1234, 42','int16','big'),'FB2E002A');
 assert.equal(B.decode('2EFB2A00','int16','little'),'-1234, 42');
 assert.equal(B.encode('4294967295','uint32'),'FFFFFFFF');
 assert.equal(B.encode('12.5','float32'),'00004841');assert.equal(B.decode('00004841','float32'),'12.5');
 assert.equal(B.encode('1.5','float64'),'000000000000F83F');
 for(const [text,format] of [['256','uint8'],['-1','uint16'],['1.5','int32'],['1e40','float32']])assert.throws(()=>B.encode(text,format));
 assert.throws(()=>B.decode('313233','uint16'),/groups of 2/);
 assert.deepEqual(B.inspect('FF2EFB2A00AA','int16','little',1),{text:'-1234, 42',count:2,remainder:1,bytes:5});
});
test('migration preserves custom controls and allows toolbar buttons to be deleted',()=>{
 const old={id:'custom',name:'My old preset',channels:[M.channel('Out',18)],controls:[M.control('Original')]};
 delete old.schemaVersion;const p=M.migratePreset(old);assert.equal(p.controls[0].name,'Original');assert.equal(p.controls.filter(c=>c.mode==='stop').length,1);
 p.controls=p.controls.filter(c=>c.mode!=='stop');assert.equal(M.migratePreset(p).controls.length,1);
 const original=M.defaults()[0],arm=original.controls.find(c=>c.name==='Arm ESC');assert.equal(arm.mode,'sequence');assert.equal(arm.placement,'toolbar');assert.deepEqual(arm.steps[0].signal,{kind:'pwm',hz:490,value:550});
 M.validatePreset({...original,controls:original.controls.map(c=>({...c,color:'#123456',activeColor:'#ABCDEF'}))});
});
test('sequence validation rejects input targets, excessive waits and early global stops',()=>{
 const p=M.defaults()[0],c=p.controls.find(c=>c.mode==='sequence');
 const copy=()=>M.clone(p);
 let x=copy();x.controls[0].steps[0].channel=1;assert.throws(()=>M.validatePreset(x),/output channel/);
 x=copy();x.controls[0].steps=[{action:'wait',ms:-1}];assert.throws(()=>M.validatePreset(x),/wait/);
 x=copy();x.controls[0].steps=[{action:'stop'},{action:'wait',ms:1}];assert.throws(()=>M.validatePreset(x),/last/);
 assert.deepEqual(M.controlChannels(c),[0]);
});
test('sequences execute signals, delays and per-channel idle in authored order',async()=>{
 const calls=[],steps=[{action:'signal',channel:0,signal:{kind:'digital',value:1},mode:'continuous',repeatMs:100},{action:'wait',ms:500},{action:'signal',channel:1,signal:{kind:'uart',data:'3132',baud:9600},mode:'once',repeatMs:100},{action:'idle',channel:0}];
 await Q.execute(steps,{send:async(ch,s,mode)=>calls.push(['send',ch,s.kind,mode]),idle:async ch=>calls.push(['idle',ch]),delay:async ms=>calls.push(['wait',ms])},new AbortController().signal);
 assert.deepEqual(calls,[['send',0,'digital','continuous'],['wait',500],['send',1,'uart','once'],['idle',0]]);
});
test('cancelling a running wait prevents later sequence signals',async()=>{
 const abort=new AbortController(),calls=[];let entered;
 const atWait=new Promise(r=>entered=r);
 const result=Q.execute([{action:'wait',ms:1000},{action:'signal',channel:0,signal:{kind:'digital',value:1}}],{send:async()=>calls.push('sent'),delay:(ms,signal)=>{entered();return Q.wait(ms,signal);}},abort.signal);
 await atWait;abort.abort();await assert.rejects(result,/Cancelled/);assert.deepEqual(calls,[]);
});
test('a cancelled sequence cannot write a signal queued behind serial work',async()=>{
 const t=new Transport(()=>{},()=>{}),abort=new AbortController(),sent=[];t.active=true;t.writer={write:async bytes=>sent.push(new TextDecoder().decode(bytes))};
 const result=t.command('signal',{ch:0,signal:{kind:'digital',value:1}},1000,()=>!abort.signal.aborted);abort.abort();await assert.rejects(result,/Cancelled/);assert.deepEqual(sent,[]);
});
test('ADC configurations accept exposed analog pins and reject digital-only pins',()=>{
 const p=M.defaults().find(p=>p.id==='analog-adc');M.validatePreset(p);
 for(const pin of M.ADC_PINS){const one=M.clone(p);one.channels=[M.channel('ADC',pin,'analog')];M.validatePreset(one);}
 for(const pin of [18,19,21,22,23]){const one=M.clone(p);one.channels[0].pin=pin;assert.throws(()=>M.validatePreset(one),/ADC pin/);}
 const bad=M.clone(p);bad.channels[0].attenuation=3;assert.throws(()=>M.validatePreset(bad),/attenuation/);
 const cap=M.demoCapture(p,60,true);assert.equal(cap.edges.length,0);assert.equal(cap.analysis.length,0);
});

const W=require('./wiring.js');
test('wiring follows preset configuration after GPIO reassignment and preset renaming',()=>{
 const p=M.defaults()[1];p.channels[0].pin=23;p.channels[1].pin=35;p.id='arbitrary-imported-id';
 assert.match(W.circuit(p,0,'loopback',{probe:1}).svg,/GPIO23/);
 assert.match(W.circuit(p,0,'loopback',{probe:1}).svg,/GPIO35/);
 assert.match(W.board(p,0),/GPIO23 · CH1/);
 const motor=M.defaults()[0];motor.id='another-id';assert.equal(W.defaultDiagram(motor,motor.channels[0]).id,motor.wiring.defaults.out);
 assert.ok(!W.available(p,p.channels[1]).some(d=>d.directions.includes('out')));
});
test('configured calculations distinguish ADC measurement range from GPIO ceiling',()=>{
 const p=M.defaults().find(p=>p.channels.some(c=>c.direction==='analog'));
 const result=W.circuit(p,0,'divider',{rTop:20,rBottom:10,sourceV:5.1});
 assert.equal(result.calculations.ratio,3);assert.ok(Math.abs(result.calculations.sourceMax-7.35)<1e-12);assert.ok(Math.abs(result.calculations.sourcePinLimit-9.9)<1e-12);assert.ok(Math.abs(result.calculations.cutoffHz-238.7324)<.001);
 assert.ok(result.limits.some(l=>l.text.includes('1.7 V at pin')));
 assert.ok(W.circuit(p,0,'divider',{sourceV:12}).limits.some(l=>l.severity==='danger'));
 const changed=W.circuit(p,0,'divider',{rTop:40});assert.equal(changed.calculations.ratio,5);
 assert.throws(()=>W.circuit(p,0,'divider',{rTop:0}),/R1/);
});
test('custom diagram labels, component values and formulas survive duplicate/export/import',()=>{
 const p=M.defaults()[0];p.id='custom-device';const d=p.wiring.diagrams.find(d=>d.id===p.wiring.defaults.out);
 d.id='fully-custom';p.wiring.defaults.out=d.id;d.title='My laboratory widget';d.limits=[{text:'Configured supply 4.2 V'}];d.components=['R9: 8.2 kΩ'];
 d.elements=[{tag:'text',attrs:{x:20,y:30,fill:'#AA22BB'},text:'MY DEVICE on {{channel.pinLabel}}'}];
 const imported=M.migratePreset(JSON.parse(JSON.stringify(p)));M.validatePreset(imported);const out=W.circuit(imported,0,W.defaultDiagram(imported,imported.channels[0]).id);
 assert.equal(out.title,'My laboratory widget');assert.match(out.svg,/MY DEVICE on GPIO18/);assert.match(out.svg,/#AA22BB/);assert.deepEqual(out.parts,['R9: 8.2 kΩ']);assert.deepEqual(out.limits.map(l=>l.text),['Configured supply 4.2 V']);
});
test('removed wiring diagrams and buttons stay removed across migration',()=>{
 const p=M.defaults()[0];p.wiring.diagrams=[];p.wiring.defaults={};p.controls=[];
 const roundTrip=M.migratePreset(p);M.validatePreset(roundTrip);assert.equal(W.available(roundTrip,roundTrip.channels[0]).length,0);assert.equal(roundTrip.controls.length,0);
});
test('legacy presets retain edited channels and controls while receiving configuration data',()=>{
 const p=M.defaults()[0];delete p.wiring;p.schemaVersion=2;p.channels[0].pin=23;p.controls=p.controls.filter(c=>c.mode!=='sequence');
 const upgraded=M.migratePreset(p);assert.equal(upgraded.channels[0].pin,23);assert.equal(upgraded.controls.length,p.controls.length);assert.ok(upgraded.wiring.diagrams.length);assert.equal(upgraded.schemaVersion,3);
});
test('new presets use generic configured defaults and no device-specific diagram',()=>{
 const p=M.newPreset();M.validatePreset(p);assert.equal(p.controls[0].on.hz,require('./configuration.js').defaults.signal.hz);
 assert.ok(p.wiring.diagrams.every(d=>d.includeInNew));
});
test('diagram imports reject executable SVG and expressions and detect calculation cycles',()=>{
 let p=M.defaults()[0];p.wiring.diagrams[0].elements=[{tag:'script',attrs:{},text:'alert(1)'}];assert.throws(()=>M.validatePreset(p),/SVG/);
 p=M.defaults()[0];p.wiring.diagrams[0].elements=[{tag:'text',attrs:{onclick:'alert(1)'},text:'x'}];assert.throws(()=>M.validatePreset(p),/attribute/);
 p=M.defaults()[0];let d=p.wiring.diagrams.find(d=>d.id==='output');d.calculations={x:['eval','danger']};assert.throws(()=>M.validatePreset(p),/executable/);
 p=M.defaults()[0];d=p.wiring.diagrams.find(d=>d.id==='output');d.calculations={x:{ref:'calc.y'},y:{ref:'calc.x'}};assert.throws(()=>W.circuit(p,0,d.id),/cycle/);
 p=M.defaults()[0];p.wiring.board.pinStyle.radius='5" onload="alert(1)';assert.throws(()=>M.validatePreset(p),/presentation/);
 assert.throws(()=>W.expression({ref:'constructor.prototype'},()=>0),/Unknown/);
});
