'use strict';
// Vector, matrix and solver helpers.
// Everything declared at the top level here is also available inside the formulas in laws.js
// and inside formulas edited live in the Formulas tab.
//
// Conventions: vectors are [x, y, z] arrays. 3×3 matrices are 9-element arrays in row-major order.
// Quaternions are [w, x, y, z]. Body axes: X forward, Y left, Z up. World Z is up.

const G = 9.81, D2R = Math.PI / 180, R2D = 180 / Math.PI;

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const crs = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const nrm = a => Math.hypot(a[0], a[1], a[2]);
const unit = a => { const n = nrm(a); return n > 1e-12 ? scl(a, 1 / n) : [0, 0, 1]; };
const clamp = (x, a, b) => x < a ? a : x > b ? b : x;
const cosd = a => Math.cos(a * D2R), sind = a => Math.sin(a * D2R);

// 3×3 matrices
const m3v = (M, v) => [M[0] * v[0] + M[1] * v[1] + M[2] * v[2], M[3] * v[0] + M[4] * v[1] + M[5] * v[2], M[6] * v[0] + M[7] * v[1] + M[8] * v[2]];
const m3T = M => [M[0], M[3], M[6], M[1], M[4], M[7], M[2], M[5], M[8]];
function m3m(A, B) {
  const C = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
  return C;
}
function m3inv(M) {
  const [a, b, c, d, e, f, g, h, i] = M;
  const A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (Math.abs(det) < 1e-15) return [0, 0, 0, 0, 0, 0, 0, 0, 0];
  const s = 1 / det;
  return [A * s, -(b * i - c * h) * s, (b * f - c * e) * s, B * s, (a * i - c * g) * s, -(a * f - c * d) * s, C * s, -(a * h - b * g) * s, (a * e - b * d) * s];
}

// Quaternions
const qmul = (a, b) => [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3], a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2], a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1], a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
function qmat(q) {
  const [w, x, y, z] = q;
  return [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x), 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)];
}
const qnorm = q => { const n = Math.hypot(q[0], q[1], q[2], q[3]); return q.map(c => c / n); };
function matToQuat(M) {
  const tr = M[0] + M[4] + M[8]; let w, x, y, z;
  if (tr > 0) { const s = Math.sqrt(tr + 1) * 2; w = 0.25 * s; x = (M[7] - M[5]) / s; y = (M[2] - M[6]) / s; z = (M[3] - M[1]) / s; }
  else if (M[0] > M[4] && M[0] > M[8]) { const s = Math.sqrt(1 + M[0] - M[4] - M[8]) * 2; w = (M[7] - M[5]) / s; x = 0.25 * s; y = (M[1] + M[3]) / s; z = (M[2] + M[6]) / s; }
  else if (M[4] > M[8]) { const s = Math.sqrt(1 + M[4] - M[0] - M[8]) * 2; w = (M[2] - M[6]) / s; x = (M[1] + M[3]) / s; y = 0.25 * s; z = (M[5] + M[7]) / s; }
  else { const s = Math.sqrt(1 + M[8] - M[0] - M[4]) * 2; w = (M[3] - M[1]) / s; x = (M[2] + M[6]) / s; y = (M[5] + M[7]) / s; z = 0.25 * s; }
  return qnorm([w, x, y, z]);
}

// Rotation whose third column is n and whose first column points as close to xref as possible.
function frameFrom(n, xref) {
  let h = sub(xref, scl(n, dot(xref, n)));
  if (nrm(h) < 1e-6) { const y = [0, 1, 0]; h = sub(y, scl(n, dot(y, n))); }
  h = unit(h); const k = crs(n, h);
  return [h[0], k[0], n[0], h[1], k[1], n[1], h[2], k[2], n[2]];
}

// Dense linear algebra
function solveLin(A, b) { // Gaussian elimination with partial pivoting
  const n = b.length, M = A.map((r, i) => r.slice().concat([b[i]]));
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (p !== c) { const t = M[p]; M[p] = M[c]; M[c] = t; }
    const pv = M[c][c]; if (Math.abs(pv) < 1e-14) continue;
    for (let r = c + 1; r < n; r++) { const f = M[r][c] / pv; if (f === 0) continue; for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k]; }
  }
  const x = new Array(n).fill(0);
  for (let r = n - 1; r >= 0; r--) { let s = M[r][n]; for (let k = r + 1; k < n; k++) s -= M[r][k] * x[k]; x[r] = Math.abs(M[r][r]) < 1e-14 ? 0 : s / M[r][r]; }
  return x;
}
function det(M) {
  const n = M.length, A = M.map(r => r.slice()); let d = 1;
  for (let c = 0; c < n; c++) {
    let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    if (Math.abs(A[p][c]) < 1e-15) return 0;
    if (p !== c) { const t = A[p]; A[p] = A[c]; A[c] = t; d = -d; }
    d *= A[c][c];
    for (let r = c + 1; r < n; r++) { const f = A[r][c] / A[c][c]; for (let k = c; k < n; k++) A[r][k] -= f * A[c][k]; }
  }
  return d;
}
function rankOf(vecs, k) {
  if (!vecs.length) return 0;
  const A = vecs.map(v => v.slice()); let mx = 0; for (const v of A) for (const x of v) mx = Math.max(mx, Math.abs(x));
  const tol = mx * 1e-7 + 1e-12; let rank = 0; const rows = A.length;
  for (let c = 0; c < k && rank < rows; c++) {
    let p = rank; for (let r = rank + 1; r < rows; r++) if (Math.abs(A[r][c]) > Math.abs(A[p][c])) p = r;
    if (Math.abs(A[p][c]) < tol) continue;
    const t = A[p]; A[p] = A[rank]; A[rank] = t;
    for (let r = rank + 1; r < rows; r++) { const f = A[r][c] / A[rank][c]; for (let j = c; j < k; j++) A[r][j] -= f * A[rank][j]; }
    rank++;
  }
  return rank;
}

// Bounded weighted least squares:
//   minimize  Σ_k W[k]·(Σ_j cols[j][k]·x[j] − w[k])²   subject to  lo[j] ≤ x[j] ≤ hi[j]
// Active-set method that clips the worst bound violation each pass. A tiny ridge term picks the
// minimum-effort solution when several inputs can do the same job.
function bls(cols, lo, hi, w, W) {
  const n = cols.length, x = new Array(n).fill(0), fixed = new Array(n).fill(false);
  if (!n) return x;
  const K = w.length;
  for (let iter = 0; iter <= n; iter++) {
    const F = []; for (let i = 0; i < n; i++) if (!fixed[i]) F.push(i);
    if (!F.length) break;
    const r = w.slice();
    for (let i = 0; i < n; i++) if (fixed[i]) for (let k = 0; k < K; k++) r[k] -= cols[i][k] * x[i];
    const m = F.length, H = [], g = [];
    for (let a = 0; a < m; a++) { H.push(new Array(m).fill(0)); const ca = cols[F[a]]; let s = 0; for (let k = 0; k < K; k++) s += W[k] * ca[k] * r[k]; g.push(s); }
    for (let a = 0; a < m; a++) { const ca = cols[F[a]]; for (let b = 0; b <= a; b++) { const cb = cols[F[b]]; let s = 0; for (let k = 0; k < K; k++) s += W[k] * ca[k] * cb[k]; H[a][b] = s; H[b][a] = s; } }
    let meanEff = 0; for (let a = 0; a < m; a++) { const sp = hi[F[a]] - lo[F[a]] || 1; meanEff += H[a][a] * sp * sp; } meanEff /= m;
    const lam = 1e-8 * meanEff + 1e-12;
    for (let a = 0; a < m; a++) { const sp = hi[F[a]] - lo[F[a]] || 1; H[a][a] += lam / (sp * sp); }
    const y = solveLin(H, g);
    let worst = -1, wv = 1e-9;
    for (let a = 0; a < m; a++) { const j = F[a], sp = hi[j] - lo[j] || 1; const viol = Math.max(lo[j] - y[a], y[a] - hi[j]) / sp; if (viol > wv) { wv = viol; worst = a; } }
    if (worst < 0 || iter === n) { for (let a = 0; a < m; a++) { const j = F[a]; x[j] = clamp(y[a], lo[j], hi[j]); } break; }
    const j = F[worst]; x[j] = y[worst] < lo[j] ? lo[j] : hi[j]; fixed[j] = true;
  }
  return x;
}
