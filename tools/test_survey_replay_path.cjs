#!/usr/bin/env node
'use strict';
// survey/replay-path.js: the Replay path node's poses, and how each says it was made.
const fs = require('fs'), path = require('path'), vm = require('vm'), assert = require('assert');
const context = {};
vm.createContext(context);
vm.runInContext(fs.readFileSync(path.join(__dirname, '..', 'survey', 'replay-path.js'), 'utf8') + ';this.replayPath=replayPath;', context);
const {replayPath} = context;
const same = (a, b, message) => assert.strictEqual(JSON.stringify(a), JSON.stringify(b), message);   // (arrays from the vm context)
const near = (a, b) => a.forEach((v, i) => assert(Math.abs(v - b[i]) < 1e-9, a + ' != ' + b));
const all = {fill_gaps: true, extend_ends: true, end_frames: 4, blend_directions: true};
const none = {fill_gaps: false, extend_ends: false, end_frames: 4, blend_directions: false};

// Ten frames 100 ms apart; given poses at frames 2, 3, 4 and 7 moving 1 m per frame along x.
const frames = Array.from({length: 10}, (_, i) => ({id: String(i).padStart(7, '0'), ms: i * 100}));
const level = [[1, 0, 0], [0, 1, 0], [0, 0, 1]], turned = [[0, -1, 0], [1, 0, 0], [0, 0, 1]];
const poses = [2, 3, 4, 7].map(i => ({frame: frames[i].id, xyz: [i, 0, 0], ...(i === 2 ? {rotation: level} : i === 7 ? {rotation: turned} : {})}));

// Settings off: only the given poses, as they are.
let out = replayPath(poses, frames, none);
same(out.map(p => p.source), ['mapped', 'mapped', 'mapped', 'mapped']);
assert(out.every(p => !p.axes));

out = replayPath(poses, frames, all);
assert.strictEqual(out.length, 10, 'one pose per frame');
same(out.map(p => p.source), ['extrapolated', 'extrapolated', 'mapped', 'mapped', 'mapped',
  'interpolated', 'interpolated', 'mapped', 'extrapolated', 'extrapolated']);
out.forEach((p, i) => near(p.xyz, [i, 0, 0]));                 // straight lines and constant speed: exactly on the line
same(out[5].between, [frames[4].id, frames[7].id], 'a filled frame names the poses it lies between');
same(out[0].from, [frames[2].id, frames[4].id], 'an extended frame names the poses that set its speed');
// Directions: given rotations kept, the frames without one blended between the nearest given ones.
same(out[2].rotation, level);
assert.strictEqual(out[5].direction, 'blended');
assert(out.slice(3, 7).every(p => p.axes && p.direction === 'blended'));

// One lone pose: no speed to measure at either end, so both ends are held there.
const lone = [{frame: frames[5].id, xyz: [5, 0, 0]}];
out = replayPath(lone, frames, {...all, fill_gaps: false});
assert(out.every(p => p.source === 'mapped' || p.source === 'held'), 'nothing to measure a speed with: held');
out.forEach(p => near(p.xyz, [5, 0, 0]));

// Poses on frames the walk didn't record are left out, not guessed.
out = replayPath([{frame: 'elsewhere', xyz: [9, 9, 9]}, ...poses], frames, none);
assert.strictEqual(out.length, 4);
console.log('Replay path: given poses, filled gaps, extended and held ends, blended directions passed');
