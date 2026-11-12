/* solver.js — closure-constraint solver.
 *
 * The tree hierarchy is still evaluated by the plain forward-kinematics path
 * in kinematics.js; this module only handles rig.constraints. Driven joints
 * keep their curve values (actuator inputs are read-only here), free joints
 * (node.driven === false, non-fixed) are the unknowns. A damped Newton
 * iteration (finite-difference Jacobian + SVD least squares) drives the
 * constraint residual to zero and reports every iteration honestly. If the
 * warm start misses the convergence basin, a small multi-start retry tries
 * alternative assembly seeds instead of silently jumping branches. */
(function (global) {
  'use strict';

  const MR = global.MR;
  const { LinEq } = MR;

  const AXIS_INDEX = { x: 0, y: 1, z: 2 };
  const DEFAULTS = {
    tolerance: 1e-8,      // metres: max |residual component| accepted
    maxIterations: 30,
    maxStepRad: 0.5,      // per-iteration step limit (rotation, radians)
    maxStepM: 0.25,       // per-iteration limit (translation, metres)
    lambda0: 1e-4,
    lambdaUp: 2.5,
    lambdaDown: 0.6,
    maxInner: 6,
    fdEps: 1e-7
  };

  const isFree = (n) => n.driven === false && n.joint.type !== 'fixed';
  const isRot = (n) => n.joint.type === 'revolute' || n.joint.type === 'continuous';

  function baseFactor(node) {
    return isRot(node)
      ? (MR.Rig.ANGLE_TO_RAD[node.joint.unit] || 1)
      : (MR.Rig.LENGTH_TO_M[node.joint.unit] || 1);
  }

  function toDisplay(node, baseValue) {
    return baseValue / baseFactor(node);
  }

  function toBase(node, displayValue) {
    return displayValue * baseFactor(node);
  }

  /* Clip a base-unit unknown into the joint's declared range. */
  function clipToRange(node, baseValue) {
    const j = node.joint;
    if (j.type === 'continuous' || j.limitType === 'continuous') return baseValue;
    const lo = toBase(node, j.min);
    const hi = toBase(node, j.max);
    return Math.min(hi, Math.max(lo, baseValue));
  }

  function displayMap(freeNodes, baseValues) {
    const m = new Map();
    for (let j = 0; j < freeNodes.length; j++) {
      m.set(freeNodes[j].id, toDisplay(freeNodes[j], baseValues[j]));
    }
    return m;
  }

  /* Build the residual vector for a given set of free-joint display values.
   * Also returns per-constraint breakdowns for the UI. */
  function residualAt(rig, t, drivenDisplay, freeDisplay, constraints) {
    const overrides = new Map();
    for (const [id, v] of drivenDisplay) overrides.set(id, v);
    for (const [id, v] of freeDisplay) overrides.set(id, v);
    const pose = MR.Kinematics.evaluate(rig, t, overrides);
    const r = [];
    const perConstraint = [];
    for (const c of constraints) {
      const fA = pose.frames.get(c.nodeA);
      const fB = pose.frames.get(c.nodeB);
      const pA = MR.Kinematics.anchorWorld(fA, c.offsetA);
      const pB = MR.Kinematics.anchorWorld(fB, c.offsetB);
      const comps = [];
      if (c.type === 'coincident') {
        for (const ax of c.axes) {
          const k = AXIS_INDEX[ax];
          comps.push({ axis: ax, value: pA[k] - pB[k] });
          r.push(pA[k] - pB[k]);
        }
      } else {
        let d2 = 0;
        for (const ax of c.axes) {
          const k = AXIS_INDEX[ax];
          d2 += (pA[k] - pB[k]) ** 2;
        }
        const dist = Math.sqrt(d2);
        comps.push({ axis: c.axes.join(''), value: dist - c.distance });
        r.push(dist - c.distance);
      }
      perConstraint.push({
        id: c.id, name: c.name, type: c.type,
        components: comps,
        norm: Math.hypot(...comps.map((q) => q.value))
      });
    }
    const norm = Math.hypot(...r);
    const maxAbs = r.length ? Math.max(...r.map((v) => Math.abs(v))) : 0;
    if (!Number.isFinite(norm)) {
      return { r, norm: Infinity, maxAbs: Infinity, perConstraint, pose };
    }
    return { r, norm, maxAbs, perConstraint, pose };
  }

  function structuralCheck(rig) {
    const errors = MR.Rig.validateConstraints(rig);
    const byId = new Map(rig.nodes.map((n) => [n.id, n]));
    const constraints = (rig.constraints || []).filter((c) => c.enabled !== false);
    for (const c of constraints) {
      const nA = byId.get(c.nodeA), nB = byId.get(c.nodeB);
      if (!nA || !nB) continue;
      for (const n of [nA, nB]) {
        if (n.driven === false && n.joint.type === 'fixed') {
          errors.push(`节点「${n.name}」被标记为自由关节，但固定关节不能作为未知量`);
        }
      }
    }
    return { errors, constraints };
  }

  /* Alternative warm starts for the multi-start fallback. Only used when the
   * preferred seed fails, so cost is irrelevant in the common path.
   * Planar revolute assembly basins are separated by singular poses; a
   * coarse angular grid reliably lands near each basin. */
  function alternativeGuesses(free) {
    const mid = free.map((n) => ((n.joint.min ?? -90) + (n.joint.max ?? 90)) / 2);
    const lo = free.map((n) => n.joint.min ?? -90);
    const hi = free.map((n) => n.joint.max ?? 90);
    const allRot = free.every((n) => isRot(n));
    const grid = allRot ? [0, 45, 90, 135, 180, -45, -90, -135] : [0];
    const patterns = [
      (_n, j) => mid[j],
      (_n, j) => (j % 2 ? lo[j] : hi[j]),
      (_n, j) => (j % 2 ? hi[j] : lo[j])
    ];
    if (free.length === 1) {
      for (const a of grid) patterns.push(() => a);
    } else if (free.length === 2 && allRot) {
      for (const a of grid) {
        patterns.push((_n, j) => (j === 0 ? a : 0));
        patterns.push((_n, j) => (j === 1 ? a : 0));
      }
    }
    const seen = new Set();
    const out = [];
    for (const pat of patterns) {
      const m = new Map();
      const key = [];
      free.forEach((n, j) => { const v = pat(n, j); m.set(n.id, v); key.push(v); });
      const k = key.join('|');
      if (!seen.has(k)) { seen.add(k); out.push(m); }
    }
    return out;
  }

  function initialSeed(free, t, guess) {
    const m = new Map();
    for (const n of free) {
      const g = guess instanceof Map ? guess.get(n.id) : guess?.[n.id];
      m.set(n.id, g !== undefined ? g : MR.Rig.evalCurve(n.curve, t));
    }
    return m;
  }

  /* One damped-Newton run from a concrete seed. */
  function iterateNewton(ctx, seedMap, attemptNo) {
    const { rig, t, cfg, free, constraints, equationCount, drivenDisplay } = ctx;
    const clampedFree = [];
    const singularEvents = [];
    let x = free.map((n) => {
      const v = seedMap.get(n.id) ?? MR.Rig.evalCurve(n.curve, t);
      const b = toBase(n, Number.isFinite(v) ? v : 0);
      return clipToRange(n, b);
    });
    let lambda = cfg.lambda0;
    let current = residualAt(rig, t, drivenDisplay, displayMap(free, x), constraints);
    if (!Number.isFinite(current.norm)) {
      return {
        converged: false, status: 'no-progress', iterations: 0,
        iterationsLog: [], rank: 0, nullDim: free.length,
        singular: false, singularEvents: [], clampedFree: [],
        finalResidual: { norm: Infinity, maxAbs: Infinity, perConstraint: current.perConstraint },
        pose: current.pose,
        values: new Map(), displayValues: new Map(), seed: new Map()
      };
    }
    let status = 'max-iterations';
    let lastRankInfo = { rank: 0, nullDim: free.length, condition: Infinity };
    const log = [];

    for (let iter = 0; iter < cfg.maxIterations; iter++) {
      // central-difference Jacobian
      const J = current.r.map(() => new Array(free.length).fill(0));
      for (let j = 0; j < free.length; j++) {
        const h = cfg.fdEps * Math.max(1, Math.abs(x[j]));
        const xp = x.slice(); xp[j] += h;
        const xm = x.slice(); xm[j] -= h;
        const rp = residualAt(rig, t, drivenDisplay, displayMap(free, xp), constraints);
        const rm = residualAt(rig, t, drivenDisplay, displayMap(free, xm), constraints);
        for (let i = 0; i < current.r.length; i++) J[i][j] = (rp.r[i] - rm.r[i]) / (2 * h);
      }

      const ls = LinEq.dampedStep(J, current.r, lambda);
      lastRankInfo = { rank: ls.rank, nullDim: ls.nullDim, condition: ls.condition };
      // structural nullspace of the constraint problem = unknowns minus the
      // number of independent constraint equations (the SVD nullDim only
      // captures numerical rank deficiency of the current Jacobian)
      if (ls.nullDim > 0 && equationCount >= free.length) singularEvents.push(iter + 1);

      // step limiting keeps the Newton walk inside the convergence basin
      let dx = ls.x;
      for (let j = 0; j < free.length; j++) {
        const cap = isRot(free[j]) ? cfg.maxStepRad : cfg.maxStepM;
        if (Math.abs(dx[j]) > cap) dx[j] = Math.sign(dx[j]) * cap;
      }
      const dxNorm = Math.hypot(...dx);

      let accepted = false;
      let innerLambda = lambda;
      for (let inner = 0; inner < cfg.maxInner; inner++) {
        const unclipped = x.map((v, j) => v - dx[j]);
        const xTry = unclipped.map((v, j) => clipToRange(free[j], v));
        const resTry = residualAt(rig, t, drivenDisplay, displayMap(free, xTry), constraints);
        if (resTry.norm < current.norm) {
          for (let j = 0; j < free.length; j++) {
            if (Math.abs(xTry[j] - unclipped[j]) > 1e-12) clampedFree.push(free[j].name);
          }
          x = xTry;
          current = resTry;
          lambda = innerLambda * cfg.lambdaDown;
          accepted = true;
          break;
        }
        innerLambda *= cfg.lambdaUp;
        const retry = LinEq.dampedStep(J, current.r, innerLambda);
        dx = retry.x;
        for (let j = 0; j < free.length; j++) {
          const cap = isRot(free[j]) ? cfg.maxStepRad : cfg.maxStepM;
          if (Math.abs(dx[j]) > cap) dx[j] = Math.sign(dx[j]) * cap;
        }
      }

      log.push({
        attempt: attemptNo,
        iter: iter + 1,
        residualNorm: current.norm,
        maxAbs: current.maxAbs,
        rank: lastRankInfo.rank,
        nullDim: lastRankInfo.nullDim,
        condition: Number.isFinite(lastRankInfo.condition) ? lastRankInfo.condition : null,
        lambda,
        dxNorm,
        accepted
      });

      if (current.maxAbs <= cfg.tolerance) {
        status = 'converged';
        break;
      }
      if (!accepted) {
        status = singularEvents.length ? 'unreachable' : 'no-progress';
        break;
      }
      if (dxNorm < 1e-13 && current.maxAbs > cfg.tolerance) {
        status = singularEvents.length ? 'unreachable' : 'max-iterations';
        break;
      }
    }

    const values = new Map();
    const displayValues = new Map();
    const seed = new Map();
    for (let j = 0; j < free.length; j++) {
      values.set(free[j].id, x[j]);
      const disp = toDisplay(free[j], x[j]);
      displayValues.set(free[j].id, disp);
      seed.set(free[j].id, disp);
    }
    return {
      converged: status === 'converged',
      status,
      iterations: log.length,
      iterationsLog: log,
      rank: lastRankInfo.rank,
      nullDim: lastRankInfo.nullDim,
      singular: singularEvents.length > 0,
      singularEvents,
      clampedFree: [...new Set(clampedFree)],
      finalResidual: {
        norm: current.norm,
        maxAbs: current.maxAbs,
        perConstraint: current.perConstraint
      },
      pose: current.pose,
      values, displayValues, seed
    };
  }

  /* Main entry.
   * options.guess: Map<nodeId, displayValue> warm start (previous frame).
   * Returns a full diagnostic report; the caller decides how to render a
   * failed frame and whether to pause playback. */
  function solve(rig, t, options = {}) {
    const cfg = { ...DEFAULTS, ...options };
    const { errors: structuralErrors, constraints } = structuralCheck(rig);

    const driven = rig.nodes.filter((n) => n.driven !== false && n.joint.type !== 'fixed');
    const free = rig.nodes.filter(isFree);

    const base = {
      status: 'no-constraints',
      converged: false,
      attempts: 0,
      iterations: 0,
      iterationsLog: [],
      finalResidual: null,
      equationCount: 0,
      unknownCount: free.length,
      drivenCount: driven.length,
      rank: 0,
      nullDim: 0,
      singular: false,
      warnings: [],
      structuralErrors,
      clampedFree: [],
      values: new Map(),
      displayValues: new Map(),
      seed: new Map(),
      pose: null
    };

    if (structuralErrors.length) {
      base.status = 'structural-error';
      base.pose = MR.Kinematics.evaluate(rig, t);
      return base;
    }
    if (!constraints.length) {
      base.converged = true;
      base.pose = MR.Kinematics.evaluate(rig, t);
      return base;
    }

    // actuator inputs: curves + limit policy. They are constants from now on.
    const drivenDisplay = new Map();
    const drivenClamped = [];
    for (const n of driven) {
      const raw = MR.Rig.evalCurve(n.curve, t);
      const lim = MR.Kinematics.applyJointLimit(n, raw);
      drivenDisplay.set(n.id, lim.applied);
      if (lim.clamped) drivenClamped.push({ id: n.id, name: n.name, status: lim.status });
    }

    const equationCount = constraints.reduce(
      (s, c) => s + (c.type === 'distance' ? 1 : c.axes.length), 0
    );
    base.equationCount = equationCount;

    if (free.length === 0) {
      // Every joint is driven: the loop can only be checked, never forced.
      const res = residualAt(rig, t, drivenDisplay, new Map(), constraints);
      base.pose = res.pose;
      base.finalResidual = { norm: res.norm, maxAbs: res.maxAbs, perConstraint: res.perConstraint };
      base.iterationsLog.push({ attempt: 1, iter: 0, residualNorm: res.norm, maxAbs: res.maxAbs, rank: 0, nullDim: 0, condition: null, lambda: 0, dxNorm: 0 });
      const ok = Number.isFinite(res.maxAbs) && res.maxAbs <= cfg.tolerance;
      base.converged = ok;
      base.status = ok ? 'converged' : 'inconsistent';
      base.attempts = 1;
      base.equationCount = equationCount;
      if (!ok) {
        base.warnings.push('所有关节均为驱动关节，无法通过调整自由关节闭合；驱动值未被修改，残差如实保留');
      }
      return base;
    }
    if (free.length > equationCount) {
      base.warnings.push(`自由度不足：${free.length} 个自由未知量 > ${equationCount} 个约束方程，机构存在 ${free.length - equationCount} 个内部自由度，采用最小范数解`);
    }
    if (equationCount > free.length) {
      base.warnings.push(`过约束：${equationCount} 个约束方程 > ${free.length} 个自由未知量（冗余 ${equationCount - free.length}），仅当方程相容时残差才能归零`);
    }

    const ctx = { rig, t, cfg, free, constraints, equationCount, drivenDisplay, drivenClamped };
    const seedMaps = [initialSeed(free, t, cfg.guess), ...alternativeGuesses(free)];
    let best = null;
    let attempts = 0;
    for (const seedMap of seedMaps) {
      attempts++;
      const rep = iterateNewton(ctx, seedMap, attempts);
      if (!best || rep.finalResidual.maxAbs < best.finalResidual.maxAbs) best = rep;
      if (rep.converged) break;
    }

    base.attempts = attempts;
    base.iterations = best.iterations;
    base.iterationsLog = best.iterationsLog;
    base.rank = best.rank;
    base.nullDim = Math.max(0, free.length - equationCount);
    base.singular = best.singular;
    base.clampedFree = best.clampedFree;
    base.finalResidual = best.finalResidual;
    base.pose = best.pose;
    base.converged = best.converged;
    base.status = best.status;
    base.values = best.values;
    base.displayValues = best.displayValues;
    base.seed = best.seed;

    if (attempts > 1 && !best.converged) {
      base.warnings.push(`热启动未收敛，已尝试 ${attempts} 组装配初值（多起点回退）仍失败`);
    } else if (attempts > 1) {
      base.warnings.push(`首选初值未命中，第 ${attempts} 组装配初值收敛（注意可能切换了解的分支）`);
    }
    if (!base.converged && equationCount > free.length) {
      base.status = 'inconsistent';
      base.warnings.push('过约束方程组不相容：残差无法归零，已保留最小二乘姿态与真实残差（驱动值未被改动）');
    }
    if (!base.converged && base.singular) {
      base.warnings.push('奇异位置附近迭代不稳定：请缩小时间步长，或检查驱动是否越过可达范围/死点');
    }
    if (!base.converged && !base.singular && equationCount <= free.length) {
      base.warnings.push('未在迭代上限内收敛：可缩小时间步长重试，或检查当前驱动值是否超出可达范围');
    }
    if (base.clampedFree.length) {
      base.warnings.push(`自由关节触达行程极限：${base.clampedFree.join('、')}`);
    }
    if (drivenClamped.length) {
      base.warnings.push(`驱动关节按曲线限制钳制：${drivenClamped.map((d) => d.name).join('、')}`);
    }
    return base;
  }

  /* True when a rig carries active closure constraints (everything else keeps
   * using the original direct-evaluation path). */
  function hasConstraints(rig) {
    return Array.isArray(rig.constraints) &&
      rig.constraints.some((c) => c.enabled !== false);
  }

  MR.Solver = { solve, hasConstraints, DEFAULTS, isFree };
})(typeof window !== 'undefined' ? window : globalThis);
