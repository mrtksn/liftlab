'use strict';
// Several battery packs: what each feeds, which pack a board's supply draws on, each pack's own cells for the
// converter checks, the problems (no pack for the motors or the boards, packs in parallel), the wiring list and the
// power drawing with a box and a + lane per pack.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
const ctx=vm.createContext({console,atob,window:{addEventListener(){}},LAWS:{},RN_TASK_FORMULAS:{},Map,Set});
for(const file of ['board-hardware','hardware','driver-presets','boards','part-geometry','designs','board-pinouts']) vm.runInContext(fs.readFileSync('js/'+file+'.js','utf8'),ctx);
// the Battery settings: the main pack is 1S 1.8 Ah
vm.runInContext('var battCfg=()=>({cells:1,capacity:1.8});this.api={powerIssues,packFeeds,motorPack,mainPack,packSpec,boardPack,boardCells,boardSupply,hardwareOverview,powerDiagramSvg}',ctx);
const A=ctx.api;
const C={ground:{kind:'mac',name:'Laptop'},boards:[{id:1,kind:'esp32',name:'Flight controller',tasks:['core','tlm']},{id:2,kind:'pizero',name:'Pi Zero',tasks:['nav']}],wiring:{parts:{},boards:{}}};
const motors=Array.from({length:4},(_,i)=>({id:i+1,type:'motor',name:'M'+i,pos:[0,0,0]}));
const main={id:10,type:'mass',battery:true,name:'Battery'},small={id:11,type:'mass',battery:true,name:'Board pack',pack:{cells:2,capacity:0.45}};

// one pack: it feeds everything, as before
let comps=[...motors,main];
assert.equal(A.packFeeds(C,main),'all');assert.equal(A.motorPack(C,comps),main);assert.equal(A.boardPack(C,C.boards[0],comps),main);
assert.equal(A.boardSupply(C,C.boards[0],A.boardCells(C,C.boards[0],comps)).kind,'boost5');
assert.deepEqual([...A.powerIssues(C,comps).errors],[]);

// a second pack for the boards: the main one feeds the motors only, the boards draw on the 2S pack (a regulator, automatically)
comps=[...motors,main,small];C.wiring.parts={10:{feeds:'motors'},11:{feeds:'boards'}};
assert.equal(A.mainPack(C,comps),main);assert.equal(A.packSpec(C,main,comps).cells,1);assert.equal(A.packSpec(C,small,comps).cells,2);
assert.equal(A.motorPack(C,comps),main);
for(const b of C.boards){assert.equal(A.boardPack(C,b,comps),small,b.name);assert.equal(A.boardCells(C,b,comps),2);assert.equal(A.boardSupply(C,b,2).kind,'reg5');}
let I=A.powerIssues(C,comps);assert.deepEqual([...I.errors],[],I.errors.join());   // 2S: a 5 V regulator
C.wiring.boards[2]={supply:'boost5'};I=A.powerIssues(C,comps);assert.ok(I.errors.some(e=>/Pi Zero: a boost converter only steps up: a Board pack, 2S/.test(e)),'a boost converter on the 2S pack: '+I.errors.join());delete C.wiring.boards[2];
small.pack.cells=1;I=A.powerIssues(C,comps);assert.deepEqual([...I.errors],[],I.errors.join());   // 1S: a boost converter
// a board fed from another board's 5 V pin draws on that board's pack
C.wiring.boards[1]={supply:'board',supplyFrom:2};assert.equal(A.boardPack(C,C.boards[0],comps),small);delete C.wiring.boards[1];
// a board can pick its pack; one that doesn't feed boards is refused (with a warning) and the first that does is taken
C.wiring.parts[10].feeds='all';C.wiring.boards[2]={pack:10};assert.equal(A.boardPack(C,C.boards[1],comps),main);
C.wiring.parts[10].feeds='motors';assert.equal(A.boardPack(C,C.boards[1],comps),small);assert.ok(A.powerIssues(C,comps).warnings.some(w=>/chosen for it/.test(w)));
delete C.wiring.boards[2];
// no pack for the motors, no pack for the boards, two packs on the motors
C.wiring.parts[10].feeds='boards';assert.ok(A.powerIssues(C,comps).errors.some(e=>/feeds the motors/.test(e)));
C.wiring.parts={10:{feeds:'motors'},11:{feeds:'motors'}};assert.ok(A.powerIssues(C,comps).errors.some(e=>/no connected pack feeds the boards/.test(e)));
assert.ok(A.powerIssues(C,comps).warnings.some(w=>/in parallel/.test(w)));
// unplugging the boards' pack leaves the boards without a supply
C.wiring.parts={10:{feeds:'motors'},11:{feeds:'boards',power:false}};assert.equal(A.boardPack(C,C.boards[0],comps),null);

// the wiring list and the drawing name both packs
C.wiring.parts={10:{feeds:'motors'},11:{feeds:'boards'}};
const rows=A.hardwareOverview(C,comps).groups.find(g=>g.id==='power').rows;
assert.match(rows.find(r=>r.device==='Battery').connection,/ESCs .* only/);assert.match(rows.find(r=>r.device==='Board pack').connection,/boards' supplies only/);
assert.match(rows.find(r=>r.device==='Pi Zero supply').connection,/boost converter.*\(Board pack\)/);
const svg=A.powerDiagramSvg(C,comps);
assert.ok(svg.includes('>Battery<')&&svg.includes('>Board pack<'),'a box per pack');
assert.ok(svg.includes('→ motors')&&svg.includes('→ boards'),'what each pack feeds');
assert.ok(svg.includes('every pack')&&svg.includes('5 V boost converter · Board pack'),'the converter on the boards\' pack, one common ground');
assert.equal((svg.match(/>5 V boost converter/g)||[]).length,1,'one converter for both boards');
// one pack: the drawing as before
const one=A.powerDiagramSvg({...C,wiring:{parts:{},boards:{}}},[...motors,main]);assert.ok(one.includes('+ red · − black')&&one.includes('battery −, every ESC'));
console.log('Power pack checks passed: what each pack feeds, a board\'s pack (chosen, fallback, through another board), each pack\'s cells for its converter, no pack for the motors or the boards, packs in parallel, the wiring list and the drawing.');
