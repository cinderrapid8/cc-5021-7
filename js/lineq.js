/* lineq.js — small dense linear algebra for the closure solver.
 * One-sided Jacobi SVD, no dependencies. Matrices are plain nested arrays
 * (row-major). Sizes here are tiny (joint counts), clarity beats speed. */
(function (global) {
  'use strict';

  const MR = global.MR || (global.MR = {});

  function zeros(m, n) {
    const A = new Array(m);
    for (let i = 0; i < m; i++) A[i] = new Array(n).fill(0);
    return A;
  }

  function identity(n) {
    const A = zeros(n, n);
    for (let i = 0; i < n; i++) A[i][i] = 1;
    return A;
  }

  function matCopy(A) {
    return A.map((row) => row.slice());
  }

  /* One-sided Jacobi SVD: A = U · diag(s) · V^T.
   * Orthogonalises the columns of a working copy U0; V accumulates the
   * rotations. Returns singular values sorted descending together with the
   * reordered U/V columns. */
  function svd(Ain, maxSweeps = 60) {
    const m = Ain.length;
    const n = Ain[0].length;
    const U = matCopy(Ain);
    const V = identity(n);
    const eps = 2.220446049250313e-16;

    for (let sweep = 0; sweep < maxSweeps; sweep++) {
      let off = 0;
      let norm = 0;
      for (let p = 0; p < n; p++) {
        for (let q = p + 1; q < n; q++) {
          let alpha = 0, beta = 0, gamma = 0;
          for (let i = 0; i < m; i++) {
            const up = U[i][p], uq = U[i][q];
            alpha += up * up;
            beta += uq * uq;
            gamma += up * uq;
          }
          norm += alpha + beta;
          off += gamma * gamma;
          if (alpha === 0 && beta === 0) continue;
          if (Math.abs(gamma) <= eps * Math.sqrt(alpha * beta)) continue;

          const zeta = (beta - alpha) / (2 * gamma);
          const t = (zeta >= 0 ? 1 : -1) / (Math.abs(zeta) + Math.sqrt(1 + zeta * zeta));
          const c = 1 / Math.sqrt(1 + t * t);
          const s = c * t;
          for (let i = 0; i < m; i++) {
            const up = U[i][p], uq = U[i][q];
            U[i][p] = c * up - s * uq;
            U[i][q] = s * up + c * uq;
          }
          for (let i = 0; i < n; i++) {
            const vp = V[i][p], vq = V[i][q];
            V[i][p] = c * vp - s * vq;
            V[i][q] = s * vp + c * vq;
          }
        }
      }
      if (off <= 1e-30 || off <= (eps * eps) * norm) break;
    }

    const sigma = new Array(n);
    const uCols = new Array(n);
    for (let j = 0; j < n; j++) {
      let len = 0;
      const col = new Array(m);
      for (let i = 0; i < m; i++) { len += U[i][j] * U[i][j]; col[i] = U[i][j]; }
      sigma[j] = Math.sqrt(len);
      if (len > 0) for (let i = 0; i < m; i++) col[i] /= sigma[j];
      uCols[j] = col;
    }
    const order = sigma.map((_, j) => j).sort((a, b) => sigma[b] - sigma[a]);
    return {
      sigma: order.map((j) => sigma[j]),
      U: order.map((j) => uCols[j]),     // columns of U, sorted
      V: order.map((j) => V.map((row) => row[j])) // columns of V, sorted
    };
  }

  /* Minimum-norm least-squares solution A x ≅ b via SVD pseudoinverse.
   * Reports effective rank, nullspace dimension and condition number so the
   * caller can flag singular / redundant constraint Jacobians. */
  function leastSquares(A, b, rankTol) {
    const m = A.length;
    const n = A[0].length;
    const { sigma, U, V } = svd(A);
    const smax = sigma[0] || 0;
    const tol = rankTol ?? Math.max(m, n) * (smax || 1) * 1e-11;
    let rank = 0;
    while (rank < sigma.length && sigma[rank] > tol) rank++;
    const x = new Array(n).fill(0);
    const utb = new Array(sigma.length).fill(0);
    for (let j = 0; j < sigma.length; j++) {
      for (let i = 0; i < m; i++) utb[j] += U[j][i] * b[i];
    }
    for (let j = 0; j < rank; j++) {
      const coeff = utb[j] / sigma[j];
      const vcol = V[j];
      for (let i = 0; i < n; i++) x[i] += coeff * vcol[i];
    }
    const smin = rank > 0 ? sigma[rank - 1] : 0;
    return {
      x,
      rank,
      nullDim: n - rank,
      sigma,
      condition: smin > 0 ? smax / smin : Infinity
    };
  }

  /* Damped least squares (Levenberg–Marquardt style): solves
   * (J^T J + lambda D^T D) dx = J^T r through the augmented system
   * [J; sqrt(lambda) D] dx = [r; 0], reusing the SVD path. D holds the
   * Marquardt column scales so lambda stays dimensionless. */
  function dampedStep(J, r, lambda) {
    const m = J.length;
    const n = J[0].length;
    const scale = new Array(n).fill(0);
    for (let i = 0; i < m; i++) {
      for (let j = 0; j < n; j++) scale[j] += J[i][j] * J[i][j];
    }
    for (let j = 0; j < n; j++) scale[j] = Math.sqrt(scale[j]) || 1;
    const rows = J.map((row) => row.slice());
    const rhs = r.slice();
    const k = Math.sqrt(Math.max(lambda, 0));
    for (let j = 0; j < n; j++) {
      const row = new Array(n).fill(0);
      row[j] = k * scale[j];
      rows.push(row);
      rhs.push(0);
    }
    return leastSquares(rows, rhs);
  }

  MR.LinEq = { zeros, identity, svd, leastSquares, dampedStep };
  global.MR = MR;
})(typeof window !== 'undefined' ? window : globalThis);
