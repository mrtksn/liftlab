'use strict';
// The step runner's instruction set, shared by the compiler (rn-compile.js), the JavaScript runner (rn-vm.js)
// and the C runner (runner/rn_ops.h is generated from this file by runner/gen_ops.js).
//
// A program is a list of 32-bit integers: an opcode, then its operands. Values live in one array of 32-bit
// floats, the arena. Most operands are arena addresses fixed when the program is loaded, so a step does no name
// lookups at all. The arena starts with the constants, which are read-only.
//
// Operand kinds:
//   d  scalar written          a  scalar read           t  jump target (offset in the program)
//   i  integer immediate       r  register written: a scalar slot that will hold an arena address
//   R  register read           o  address that may be -1 (absent optional input)
//   g  register read, or -1 (none)
// `blk` lists the blocks an instruction touches beyond single scalars, so the loader can check them:
//   [operand index, 'r' | 'w', size] with size a number, '$k' (the immediate in operand k), or
//   'L<s>,<c>' (a list: 1 + cap × stride, stride and cap given by operands s and c), 'R<s>,<c>' (a ring:
//   2 + cap × stride) or 'N<c>' (a list of numbers: 1 + cap).

const RN_OPS = [
  // scalars
  ['NOP', ''], ['MOV', 'da'],
  ['ADD', 'daa'], ['SUB', 'daa'], ['MUL', 'daa'], ['DIV', 'daa'], ['MOD', 'daa'], ['POW', 'daa'], ['MIN', 'daa'], ['MAX', 'daa'], ['ATAN2', 'daa'],
  ['LT', 'daa'], ['LE', 'daa'], ['EQ', 'daa'], ['NE', 'daa'],
  ['NEG', 'da'], ['ABS', 'da'], ['SQRT', 'da'], ['SIN', 'da'], ['COS', 'da'], ['TAN', 'da'], ['ASIN', 'da'], ['ACOS', 'da'], ['ATAN', 'da'],
  ['EXP', 'da'], ['LOG', 'da'], ['FLOOR', 'da'], ['CEIL', 'da'], ['ROUND', 'da'], ['SIGN', 'da'], ['NOT', 'da'], ['TRUTH', 'da'],
  ['SEL', 'daaa'], ['CLAMP', 'daaa'], ['FMA', 'daaa'],
  // control
  ['JMP', 't'], ['JZ', 'at'], ['JNZ', 'at'], ['TRAP', 'i'],
  // blocks
  ['CPY', 'iii', [[0, 'w', '$2'], [1, 'r', '$2']]],              // CPY dst src n
  ['FILL', 'iai', [[0, 'w', '$2']]],                             // FILL dst value n
  // lists: [len, element 0, element 1, …]; ring lists: [len, head, elements…]
  ['LLEN', 'iaii', [[0, 'w', 'L3,2']]],                          // LLEN list n cap stride: len = n (checked ≤ cap)
  ['LFILL', 'iaii', [[0, 'w', 'L2,3']]],                         // LFILL list value stride cap: every element's floats = value
  ['CPYL', 'iiii', [[0, 'w', 'L2,3'], [1, 'r', 'L2,3']]],        // CPYL dst src stride dcap: copy len and elements (len ≤ dcap)
  ['PUSH', 'iiii', [[0, 'w', 'L2,3'], [1, 'r', '$2']]],          // PUSH list src stride cap
  ['RPUSH', 'iiii', [[0, 'w', 'R2,3'], [1, 'r', '$2']]],         // RPUSH ring src stride cap
  ['RSHIFT', 'iii', [[0, 'w', 'R1,2']]],                         // RSHIFT ring stride cap
  ['RCLR', 'iii', [[0, 'w', 'R1,2']]],                           // RCLR ring stride cap: empty it
  ['IDX', 'riaii'],                                              // IDX r base idx bound stride: r = base + idx·stride, 0 ≤ idx < arena[bound]
  ['IDXI', 'rRiaai'],                                            // IDXI r rb off idx bound stride: r = arena[rb] + off + idx·stride
  ['RIDX', 'riaii', [[1, 'r', 'R3,4']]],                         // RIDX r ring idx stride cap: element idx of a ring (idx < len)
  ['LDI', 'dRi'], ['STI', 'Ria'],                                // LDI d r off / STI r off s: through a register
  ['CPI', 'iRii', [[0, 'w', '$3']]], ['CPO', 'Riii', [[2, 'r', '$3']]],   // CPI dst r off n / CPO r off src n
  ['AR', 'ri'],                                                  // AR r addr: r = addr
  // kernels: the heavy math, native in the runner
  ['M3V', 'iii', [[0, 'w', 3], [1, 'r', 9], [2, 'r', 3]]],
  ['M3M', 'iii', [[0, 'w', 9], [1, 'r', 9], [2, 'r', 9]]],
  ['M3T', 'ii', [[0, 'w', 9], [1, 'r', 9]]],
  ['CRS', 'iii', [[0, 'w', 3], [1, 'r', 3], [2, 'r', 3]]],
  ['QMUL', 'iii', [[0, 'w', 4], [1, 'r', 4], [2, 'r', 4]]],
  ['QMAT', 'ii', [[0, 'w', 9], [1, 'r', 4]]],
  ['QNORM', 'ii', [[0, 'w', 4], [1, 'r', 4]]],
  ['M2Q', 'ii', [[0, 'w', 4], [1, 'r', 9]]],
  ['UNIT3', 'ii', [[0, 'w', 3], [1, 'r', 3]]],
  ['NRM3', 'di', [[1, 'r', 3]]],
  ['DOT3', 'dii', [[1, 'r', 3], [2, 'r', 3]]],
  ['ADD3', 'iii', [[0, 'w', 3], [1, 'r', 3], [2, 'r', 3]]],
  ['SUB3', 'iii', [[0, 'w', 3], [1, 'r', 3], [2, 'r', 3]]],
  ['SCL3', 'iia', [[0, 'w', 3], [1, 'r', 3]]],
  // BLS dst cols lo hi w W pullQ pullR rel K cap: bounded weighted least squares (math.js bls)
  ['BLS', 'iiiiiiooaii', [[0, 'w', 'N10'], [1, 'r', 'L9,10'], [2, 'r', 'N10'], [3, 'r', 'N10'], [4, 'r', '$9'], [5, 'r', '$9'], [6, 'r', 'N10'], [7, 'r', 'N10']]],
  // Fused steps over whole lists. A view is three operands: a register (or -1), an offset and a stride; the
  // element k is at arena[register] + offset + k·stride (or offset + k·stride without a register). count is
  // the number of elements (checked ≤ cap); the loader checks views without a register against cap, and the
  // runner checks the others when it runs.
  ['VV', 'igiigiigiiai'],                                        // VV kind dst… a… b… count cap: dst[k] = a[k] ∘ b[k]
  ['VS', 'igiigiiaai'],                                          // VS kind dst… a… s count cap: dst[k] = a[k] ∘ s (kinds rsub, rdiv: s ∘ a[k])
  ['VDOT', 'dgiigiiai'],                                         // VDOT d a… b… count cap: d = Σ a[k]·b[k]
];
const RN_VKIND = { add: 0, sub: 1, mul: 2, div: 3, rsub: 4, rdiv: 5 };
// Operand positions of each view: [register, offset, stride, write?].
const RN_VIEWS = { VV: [[1, 2, 3, 1], [4, 5, 6, 0], [7, 8, 9, 0]], VS: [[1, 2, 3, 1], [4, 5, 6, 0]], VDOT: [[1, 2, 3, 0], [4, 5, 6, 0]] };
const RN_OP = {}; RN_OPS.forEach(([name], i) => { RN_OP[name] = i; });
const RN_MAGIC = 0x52464244;       // 'DBFR'
const RN_VERSION = 2;

if (typeof module !== 'undefined') module.exports = { RN_OPS, RN_OP, RN_VKIND, RN_VIEWS, RN_MAGIC, RN_VERSION };
