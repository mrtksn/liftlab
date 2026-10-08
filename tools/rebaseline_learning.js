#!/usr/bin/env node
// Recompute only the RLS fixture outputs/state after the full-regressor covariance fix. Recorded inputs stay
// unchanged; tools/test_learning.js checks against an independent information-matrix reference and both VMs.
'use strict';
const fs=require('fs'),vm=require('vm'),zlib=require('zlib');
const file='tools/golden.json.gz',G=JSON.parse(zlib.gunzipSync(fs.readFileSync(file)).toString()),c=vm.createContext({Math});
for(const f of ['math.js','laws.js'])vm.runInContext(fs.readFileSync('js/'+f,'utf8'),c);
const fn=vm.runInContext('identifyEffectiveness',c);
for(const s of G.identifyEffectiveness){const args=JSON.parse(JSON.stringify(s.args));s.ret=fn(...args);if(s.stAfter)s.stAfter=args[0];}
fs.writeFileSync(file,zlib.gzipSync(JSON.stringify(G)));
console.log('Rebaselined RLS expected covariance; recorded inputs and other formulas retained');
