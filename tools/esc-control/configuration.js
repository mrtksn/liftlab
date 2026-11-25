/* Node entry point for the same JSON configuration loaded by bootstrap.js. */
'use strict';
const manifest=require('./config/manifest.json');
module.exports=Object.fromEntries(Object.entries(manifest.resources).map(([key,file])=>[key,require('./config/'+file)]));
