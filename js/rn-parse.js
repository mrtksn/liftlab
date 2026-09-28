'use strict';
// Parser for the subset of JavaScript the formulas are written in. It produces a small syntax tree (ESTree-like)
// for the step compiler (rn-compile.js). No outside library: the flight code has to be compiled in any browser,
// and later on the companion computer, from the same source.
//
// Supported: function declarations and arrow functions; const / let (with [a, b] destructuring); if / else;
// for (;;), for (const x of list); return, break, continue; numbers, strings, null, true / false; arrays with
// ...spread; object literals; member access a.b and a[i]; calls; new Array(n); the operators
// = += -= *= /= ?: ?? || && === !== == != < <= > >= + - * / % ** ! unary - and ++ / --.

const RN_PUNCT = ['...', '===', '!==', '**=', '=>', '==', '!=', '<=', '>=', '&&', '||', '??', '**', '++', '--', '+=', '-=', '*=', '/=',
  '{', '}', '(', ')', '[', ']', ';', ',', '.', '?', ':', '=', '<', '>', '+', '-', '*', '/', '%', '!'];
const RN_WORDS = new Set(['function', 'const', 'let', 'var', 'if', 'else', 'for', 'of', 'return', 'break', 'continue', 'null', 'true', 'false', 'new', 'undefined', 'typeof']);

class RnSyntaxError extends Error {
  constructor(msg, pos, src) {
    const before = src.slice(0, pos), line = before.split('\n').length, col = pos - before.lastIndexOf('\n');
    super(`${msg} (line ${line}, column ${col})`); this.pos = pos; this.line = line;
  }
}

function rnTokenize(src) {
  const out = []; let i = 0;
  const isId0 = c => /[A-Za-z_$À-￿]/.test(c), isId = c => /[A-Za-z0-9_$À-￿]/.test(c);
  while (i < src.length) {
    const c = src[i];
    if (c === ' ' || c === '\t' || c === '\n' || c === '\r') { i++; continue; }
    if (c === '/' && src[i + 1] === '/') { while (i < src.length && src[i] !== '\n') i++; continue; }
    if (c === '/' && src[i + 1] === '*') { const e = src.indexOf('*/', i + 2); if (e < 0) throw new RnSyntaxError('Unclosed comment', i, src); i = e + 2; continue; }
    const start = i;
    if (/[0-9]/.test(c) || (c === '.' && /[0-9]/.test(src[i + 1]))) {
      const m = /^(0[xX][0-9a-fA-F]+|(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?)/.exec(src.slice(i));
      out.push({ t: 'num', v: Number(m[0]), pos: start }); i += m[0].length; continue;
    }
    if (c === '"' || c === "'") {
      let j = i + 1, s = '';
      while (j < src.length && src[j] !== c) { if (src[j] === '\\') { s += src[j + 1]; j += 2; } else s += src[j++]; }
      if (j >= src.length) throw new RnSyntaxError('Unclosed string', i, src);
      out.push({ t: 'str', v: s, pos: start }); i = j + 1; continue;
    }
    if (c === '`') throw new RnSyntaxError('Template strings (`…`) aren\'t supported in flight code', i, src);
    if (isId0(c)) {
      let j = i + 1; while (j < src.length && isId(src[j])) j++;
      const w = src.slice(i, j); out.push({ t: RN_WORDS.has(w) ? 'kw' : 'id', v: w, pos: start }); i = j; continue;
    }
    const p = RN_PUNCT.find(p => src.startsWith(p, i));
    if (!p) throw new RnSyntaxError(`Unexpected character "${c}"`, i, src);
    out.push({ t: 'p', v: p, pos: start }); i += p.length;
  }
  out.push({ t: 'eof', v: '', pos: src.length });
  return out;
}

function rnParse(src) {
  const toks = rnTokenize(src); let k = 0;
  const peek = (o = 0) => toks[k + o], next = () => toks[k++];
  const is = (v, o = 0) => { const t = toks[k + o]; return (t.t === 'p' || t.t === 'kw') && t.v === v; };
  const fail = (msg, t = peek()) => { throw new RnSyntaxError(msg, t.pos, src); };
  const eat = v => { if (!is(v)) fail(`Expected "${v}" but found "${peek().v || 'the end'}"`); return next(); };
  const opt = v => is(v) ? next() : null;
  const ident = () => { const t = next(); if (t.t !== 'id') fail(`Expected a name but found "${t.v}"`, t); return t.v; };
  const node = (type, pos, o) => Object.assign({ type, pos }, o);

  function program() {
    const t = peek();
    if (is('function')) { const f = funcDecl(); if (peek().t !== 'eof') fail('Only one function is allowed'); return f; }
    const e = expression();                       // an arrow function or `(function …)` also works
    if (e.type !== 'Arrow') fail('The code must be a single function', t);
    return e;
  }
  function funcDecl() {
    const t = eat('function'), name = peek().t === 'id' ? ident() : null;
    eat('('); const params = [];
    if (!is(')')) do params.push(pattern(true)); while (opt(','));
    eat(')');
    return node('Function', t.pos, { name, params, body: block() });
  }
  function pattern(param) {
    const t = peek();
    if (opt('[')) { const els = []; if (!is(']')) do els.push(is(',') ? null : pattern()); while (opt(',')); eat(']'); return node('ArrayPattern', t.pos, { elements: els }); }
    const name = ident();
    if (param && opt('=')) return node('Default', t.pos, { name, value: assign() });
    return node('Id', t.pos, { name });
  }
  function block() { const t = eat('{'), body = []; while (!is('}')) body.push(statement()); eat('}'); return node('Block', t.pos, { body }); }
  function statement() {
    const t = peek();
    if (is('{')) return block();
    if (is(';')) { next(); return node('Empty', t.pos); }
    if (is('const') || is('let') || is('var')) { const d = varDecl(); opt(';'); return d; }
    if (is('if')) {
      next(); eat('('); const test = expression(); eat(')');
      const cons = statement(); const alt = opt('else') ? statement() : null;
      return node('If', t.pos, { test, cons, alt });
    }
    if (is('for')) {
      next(); eat('(');
      if ((is('const') || is('let')) && toks[k + 2] && toks[k + 2].t === 'kw' && toks[k + 2].v === 'of') {
        const kind = next().v, id = pattern(); eat('of'); const list = expression(); eat(')');
        return node('ForOf', t.pos, { kind, id, list, body: statement() });
      }
      const init = is(';') ? null : (is('let') || is('const')) ? varDecl() : expression(); eat(';');
      const test = is(';') ? null : expression(); eat(';');
      const update = is(')') ? null : expression(); eat(')');
      return node('For', t.pos, { init, test, update, body: statement() });
    }
    if (is('return')) { next(); const arg = is(';') || is('}') ? null : expression(); opt(';'); return node('Return', t.pos, { arg }); }
    if (is('break')) { next(); opt(';'); return node('Break', t.pos); }
    if (is('continue')) { next(); opt(';'); return node('Continue', t.pos); }
    if (is('function')) fail('Declare helpers with const name = (…) => …');
    const e = expression(); opt(';');
    return node('ExprStmt', t.pos, { expr: e });
  }
  function varDecl() {
    const t = next(), decls = [];
    do { const id = pattern(); const init = opt('=') ? assign() : null; decls.push({ id, init }); } while (opt(','));
    return node('VarDecl', t.pos, { kind: t.v === 'var' ? 'let' : t.v, decls });
  }
  function expression() {
    const t = peek(), e = assign();
    if (is(',')) { const list = [e]; while (opt(',')) list.push(assign()); return node('Sequence', t.pos, { list }); }
    return e;
  }
  // Arrow function if the tokens ahead look like `x =>` or `(a, b) =>`.
  function arrowAhead() {
    if (peek().t === 'id' && is('=>', 1)) return true;
    if (!is('(')) return false;
    let d = 0, j = k;
    for (; j < toks.length; j++) {
      const tt = toks[j]; if (tt.t !== 'p') continue;
      if (tt.v === '(' || tt.v === '[' || tt.v === '{') d++;
      else if (tt.v === ')' || tt.v === ']' || tt.v === '}') { d--; if (d === 0) break; }
    }
    return toks[j + 1] && toks[j + 1].t === 'p' && toks[j + 1].v === '=>';
  }
  function arrow() {
    const t = peek(), params = [];
    if (peek().t === 'id') params.push(node('Id', t.pos, { name: ident() }));
    else { eat('('); if (!is(')')) do params.push(pattern(true)); while (opt(',')); eat(')'); }
    eat('=>');
    const body = is('{') ? block() : assign();
    return node('Arrow', t.pos, { params, body });
  }
  function assign() {
    if (arrowAhead()) return arrow();
    const t = peek(), left = conditional();
    for (const op of ['=', '+=', '-=', '*=', '/=', '**=']) if (is(op)) {
      next();
      if (left.type !== 'Id' && left.type !== 'Member') fail('Can only assign to a name, a field or an element', t);
      return node('Assign', t.pos, { op, target: left, value: assign() });
    }
    return left;
  }
  function conditional() {
    const t = peek(), test = binary(0);
    if (opt('?')) { const cons = assign(); eat(':'); const alt = assign(); return node('Cond', t.pos, { test, cons, alt }); }
    return test;
  }
  const LEVELS = [['??'], ['||'], ['&&'], ['===', '!==', '==', '!='], ['<', '<=', '>', '>='], ['+', '-'], ['*', '/', '%']];
  function binary(level) {
    if (level === LEVELS.length) return power();
    const t = peek(); let left = binary(level + 1);
    for (;;) {
      const op = LEVELS[level].find(o => is(o)); if (!op) return left;
      next(); const right = binary(level + 1);
      left = node(op === '&&' || op === '||' || op === '??' ? 'Logical' : 'Binary', t.pos, { op, left, right });
    }
  }
  function power() {
    const t = peek(), base = unary();
    if (opt('**')) return node('Binary', t.pos, { op: '**', left: base, right: power() });
    return base;
  }
  function unary() {
    const t = peek();
    if (is('!') || is('-') || is('+')) { next(); return node('Unary', t.pos, { op: t.v, arg: unary() }); }
    if (is('typeof')) fail('typeof isn\'t supported in flight code');
    if (is('++') || is('--')) { next(); return node('Update', t.pos, { op: t.v, prefix: true, target: unary() }); }
    let e = postfix();
    if (is('++') || is('--')) e = node('Update', t.pos, { op: next().v, prefix: false, target: e });
    return e;
  }
  function postfix() {
    const t = peek(); let e = primary();
    for (;;) {
      if (opt('.')) { const nt = next(); if (nt.t !== 'id' && nt.t !== 'kw') fail('Expected a field name', nt); e = node('Member', t.pos, { obj: e, prop: nt.v, computed: false }); }
      else if (opt('[')) { const idx = expression(); eat(']'); e = node('Member', t.pos, { obj: e, index: idx, computed: true }); }
      else if (opt('(')) { const args = []; if (!is(')')) do args.push(opt('...') ? node('Spread', peek().pos, { arg: assign() }) : assign()); while (opt(',')); eat(')'); e = node('Call', t.pos, { callee: e, args }); }
      else return e;
    }
  }
  function primary() {
    const t = next();
    if (t.t === 'num') return node('Num', t.pos, { value: t.v });
    if (t.t === 'str') return node('Str', t.pos, { value: t.v });
    if (t.t === 'id') return node('Id', t.pos, { name: t.v });
    if (t.t === 'kw') {
      if (t.v === 'null' || t.v === 'undefined') return node('Null', t.pos);
      if (t.v === 'true' || t.v === 'false') return node('Num', t.pos, { value: t.v === 'true' ? 1 : 0, bool: true });
      if (t.v === 'new') {
        const c = ident(); if (c !== 'Array') fail('Only new Array(n) is supported', t);
        eat('('); const n = assign(); eat(')');
        return node('NewArray', t.pos, { n });
      }
      if (t.v === 'function') { k--; return funcDecl(); }
      fail(`Unexpected "${t.v}"`, t);
    }
    if (t.v === '(') { const e = expression(); eat(')'); return e; }
    if (t.v === '[') {
      const els = [];
      if (!is(']')) do { if (is(']')) break; els.push(opt('...') ? node('Spread', peek().pos, { arg: assign() }) : assign()); } while (opt(','));
      eat(']'); return node('ArrayLit', t.pos, { elements: els });
    }
    if (t.v === '{') {
      const props = [];
      if (!is('}')) do {
        if (is('}')) break;
        const kt = next(); if (kt.t !== 'id' && kt.t !== 'str' && kt.t !== 'kw') fail('Expected a field name', kt);
        if (opt(':')) props.push({ key: kt.v, value: assign() });
        else props.push({ key: kt.v, value: node('Id', kt.pos, { name: kt.v }) });   // shorthand { q, r }
      } while (opt(','));
      eat('}'); return node('ObjectLit', t.pos, { props });
    }
    fail(`Unexpected "${t.v || 'end of code'}"`, t);
  }
  return program();
}

if (typeof module !== 'undefined') module.exports = { rnParse, rnTokenize, RnSyntaxError };
